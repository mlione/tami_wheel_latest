#!/usr/bin/env python3
"""
ZED 双目数据集采集节点（从 avoid_controller 订阅）

本节点订阅 avoid_controller 发布的话题，采集并保存：
  - 左目 RGB 图像 (rgb_image，若没有则使用 /debug/viz 兜底) -> left/
  - 右目 RGB 图像 (rgb_image_r) -> right/
  - 深度图 (depth_image) -> depth/
  - 路边界数据 (perception/output 或旧版 /roadside_data) -> roadside/
  - LQR/控制线速度和角速度 (/cmd_vel) -> control/
  - 里程计 (/odom 或 /zed/odom) -> odom/
  - IMU (/zed/imu) -> imu/
  - /cmd_vel、/velocity_feedback、perception/output、/road_mask、里程计、IMU 的接收频率 -> CSV

用法:
    # 构建
    cd /home/lee/wheel_together && colcon build --packages-select wheel
    
    # 终端1：运行 avoid_controller 节点
    source install/setup.bash
    ros2 run data_transform avoid_controller
    
    # 终端2：默认无GUI自动采集到 ~/zed_dataset/haizhuhuN
    source install/setup.bash
    python3 src/wheel/scripts/zed_capture_dataset.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
try:
    from wheel_msgs.msg import PerceptionOutput
except ImportError:
    PerceptionOutput = None

try:
    from data_transform.msg import ControlFeedback, Roadside
except ImportError:
    ControlFeedback = None
    Roadside = None
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import json
import argparse
import time
from pathlib import Path
from collections import deque
import threading


class TopicRateTracker:
    """用最近若干次回调到达时间估算 topic 频率。"""

    def __init__(self, window_size=50):
        self.times = deque(maxlen=window_size)

    def tick(self, stamp):
        self.times.append(stamp)

    def hz(self):
        if len(self.times) < 2:
            return ''
        elapsed = self.times[-1] - self.times[0]
        if elapsed <= 0:
            return ''
        return (len(self.times) - 1) / elapsed

    def latest_time(self):
        return self.times[-1] if self.times else ''


class ZEDDatasetCollector(Node):
    """ROS2 节点，从话题订阅和保存 ZED 数据"""
    
    def __init__(self, output_dir, interval=0.2, max_frames=1000, headless=True):
        super().__init__('zed_dataset_collector')
        
        self.output_dir = Path(output_dir)
        self.interval = interval
        self.max_frames = max_frames
        self.headless = headless
        self.bridge = CvBridge()
        
        # 数据缓冲
        self.latest_left = None
        self.latest_left_source = None
        self.latest_right = None
        self.latest_depth = None
        self.latest_roadside = None
        self.latest_cmd_vel = None
        self.latest_bluetooth_feedback = None
        self.latest_road_mask = None
        self.latest_odom = None
        self.latest_imu = None
        self.topic_rates = {
            'cmd_vel': TopicRateTracker(),
            'bluetooth_feedback': TopicRateTracker(),
            'roadside': TopicRateTracker(),
            'road_mask': TopicRateTracker(),
            'odom': TopicRateTracker(),
            'zed_odom': TopicRateTracker(),
            'imu': TopicRateTracker(),
        }
        self.frame_id = 0
        self.frames_received = 0  # 初始化帧计数器，跳过前20帧
        
        self.lock = threading.Lock()
        
        # 创建输出目录结构
        self._create_output_dirs()
        
        # 订阅话题
        self.left_sub = self.create_subscription(
            Image, 'rgb_image', self.left_callback, 10)
        self.debug_viz_sub = self.create_subscription(
            Image, '/debug/viz', self.debug_viz_callback, qos_profile_sensor_data)
        self.right_sub = self.create_subscription(
            Image, 'rgb_image_r', self.right_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, 'depth_image', self.depth_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.road_mask_sub = self.create_subscription(
            Image, '/road_mask', self.road_mask_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile_sensor_data)
        self.zed_odom_sub = self.create_subscription(
            Odometry, '/zed/odom', self.zed_odom_callback, qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(
            Imu, '/zed/imu', self.imu_callback, qos_profile_sensor_data)

        self.perception_sub = None
        if PerceptionOutput is not None:
            self.perception_sub = self.create_subscription(
                PerceptionOutput, '/perception/output', self.perception_callback, 10)
        else:
            self.get_logger().warn("未找到 wheel_msgs.msg.PerceptionOutput，跳过 perception/output 订阅")

        self.roadside_sub = None
        if Roadside is not None:
            self.roadside_sub = self.create_subscription(
                Roadside, '/roadside_data', self.roadside_callback, 10)

        self.bluetooth_feedback_sub = None
        if ControlFeedback is not None:
            self.bluetooth_feedback_sub = self.create_subscription(
                ControlFeedback, '/velocity_feedback', self.bluetooth_feedback_callback, 10)
        
        self.get_logger().info(f"✓ 数据集采集节点已启动，输出目录: {self.output_dir}")
        self.get_logger().info(
            f"✓ 订阅话题: rgb_image, /debug/viz, rgb_image_r, depth_image, "
            f"/perception/output, /roadside_data(可选), /cmd_vel, "
            f"/velocity_feedback(可选), /road_mask, /odom, /zed/odom, /zed/imu")
        self.get_logger().info(f"⏭️  前20帧将被跳过（初始化阶段）")
        
        # 自动采集定时器
        if self.interval > 0:
            self.timer = self.create_timer(self.interval, self.auto_capture)
            self.get_logger().info(f"✓ 自动采集已启用，间隔 {self.interval}s")
        
        self.start_time = time.time()
        
    def _create_output_dirs(self):
        """创建输出目录结构"""
        dirs = ['left', 'right', 'depth', 'roadside', 'control', 'odom', 'imu']
        for d in dirs:
            (self.output_dir / d).mkdir(parents=True, exist_ok=True)
        self.control_csv_path = self.output_dir / 'lqr_angular_velocity.csv'
        if not self.control_csv_path.exists():
            with open(self.control_csv_path, 'w') as f:
                f.write(
                    'frame_id,capture_time,roadside_time,right_distance,right_angle,'
                    'cmd_vel_receive_time,linear_x,angular_z_lqr,cmd_vel_hz,'
                    'bluetooth_feedback_time,bluetooth_velocity,bluetooth_steering,'
                    'bluetooth_feedback_hz,roadside_hz,road_mask_time,road_mask_hz,'
                    'odom_time,odom_receive_time,odom_source,odom_x,odom_y,odom_z,odom_hz,'
                    'imu_time,imu_receive_time,imu_qx,imu_qy,imu_qz,imu_qw,imu_hz\n')
        self.get_logger().info(f"✓ 创建输出目录: {self.output_dir}")

    def _stamp_to_sec(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def _vector3_to_list(self, vector):
        return [float(vector.x), float(vector.y), float(vector.z)]

    def _quaternion_to_list(self, quat):
        return [float(quat.x), float(quat.y), float(quat.z), float(quat.w)]
    
    def left_callback(self, msg):
        """左目图像回调"""
        with self.lock:
            try:
                self.latest_left = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.latest_left_source = 'rgb_image'
            except Exception as e:
                self.get_logger().warn(f"左目图像转换失败: {e}")

    def debug_viz_callback(self, msg):
        """run_launch.py live 模式下的可视化图像兜底。"""
        with self.lock:
            if self.latest_left_source == 'rgb_image':
                return
            try:
                self.latest_left = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.latest_left_source = '/debug/viz'
            except Exception as e:
                self.get_logger().warn(f"debug/viz 图像转换失败: {e}")
    
    def right_callback(self, msg):
        """右目图像回调"""
        with self.lock:
            try:
                self.latest_right = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().warn(f"右目图像转换失败: {e}")
    
    def depth_callback(self, msg):
        """深度图回调"""
        with self.lock:
            try:
                # 深度图通常是 float32
                self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            except Exception as e:
                self.get_logger().warn(f"深度图转换失败: {e}")
    
    def roadside_callback(self, msg):
        """roadside_data 回调，记录右侧边界距离和偏角"""
        with self.lock:
            receive_time = self.get_clock().now().nanoseconds * 1e-9
            self.topic_rates['roadside'].tick(receive_time)
            self.latest_roadside = {
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'receive_time': receive_time,
                'right_distance': float(msg.right_distance),
                'right_angle': float(msg.right_angle)
            }

    def perception_callback(self, msg):
        """当前 wheel_perception 发布的感知结果回调。"""
        with self.lock:
            receive_time = self.get_clock().now().nanoseconds * 1e-9
            self.topic_rates['roadside'].tick(receive_time)
            self.latest_roadside = {
                'timestamp': self._stamp_to_sec(msg.header.stamp),
                'receive_time': receive_time,
                'source_topic': '/perception/output',
                'has_road_edge': bool(msg.has_road_edge),
                'right_distance': float(msg.right_distance),
                'right_angle': float(msg.road_yaw_error),
                'min_distance': float(msg.min_distance),
                'min_point': self._vector3_to_list(msg.min_point),
            }

    def cmd_vel_callback(self, msg):
        """cmd_vel 回调，angular.z 作为 avoid_controller 输出的 LQR 角速度记录"""
        with self.lock:
            receive_time = self.get_clock().now().nanoseconds * 1e-9
            self.topic_rates['cmd_vel'].tick(receive_time)
            self.latest_cmd_vel = {
                'receive_time': receive_time,
                'linear': [
                    float(msg.linear.x),
                    float(msg.linear.y),
                    float(msg.linear.z)
                ],
                'angular': [
                    float(msg.angular.x),
                    float(msg.angular.y),
                    float(msg.angular.z)
                ],
                'angular_z_lqr': float(msg.angular.z)
            }

    def bluetooth_feedback_callback(self, msg):
        """velocity_feedback 回调，作为蓝牙控制反馈/循环频率记录来源"""
        with self.lock:
            receive_time = self.get_clock().now().nanoseconds * 1e-9
            self.topic_rates['bluetooth_feedback'].tick(receive_time)
            self.latest_bluetooth_feedback = {
                'receive_time': receive_time,
                'velocity': float(msg.velocity),
                'steering': float(msg.steering),
                'gear': int(msg.gear),
                'mode': int(msg.mode),
                'alive_roll': int(msg.alive_roll),
            }

    def road_mask_callback(self, msg):
        """road_mask 回调，仅记录语义分割 mask 图接收频率和尺寸"""
        with self.lock:
            receive_time = self.get_clock().now().nanoseconds * 1e-9
            self.topic_rates['road_mask'].tick(receive_time)
            self.latest_road_mask = {
                'receive_time': receive_time,
                'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                'width': int(msg.width),
                'height': int(msg.height),
                'encoding': msg.encoding,
            }

    def odom_callback(self, msg):
        """run_launch.py 中 FusionNode 发布的 /odom 回调。"""
        self._store_odom(msg, '/odom', 'odom')

    def zed_odom_callback(self, msg):
        """兼容数据集回放或其他 ZED 节点发布的 /zed/odom。"""
        self._store_odom(msg, '/zed/odom', 'zed_odom')

    def _store_odom(self, msg, source_topic, rate_key):
        with self.lock:
            receive_time = self.get_clock().now().nanoseconds * 1e-9
            self.topic_rates[rate_key].tick(receive_time)
            pose = msg.pose.pose
            twist = msg.twist.twist
            self.latest_odom = {
                'timestamp': self._stamp_to_sec(msg.header.stamp),
                'receive_time': receive_time,
                'source_topic': source_topic,
                'frame_id': msg.header.frame_id,
                'child_frame_id': msg.child_frame_id,
                'position': self._vector3_to_list(pose.position),
                'orientation': self._quaternion_to_list(pose.orientation),
                'linear_velocity': self._vector3_to_list(twist.linear),
                'angular_velocity': self._vector3_to_list(twist.angular),
                'pose_covariance': [float(v) for v in msg.pose.covariance],
                'twist_covariance': [float(v) for v in msg.twist.covariance],
            }

    def imu_callback(self, msg):
        """ZED IMU 回调。"""
        with self.lock:
            receive_time = self.get_clock().now().nanoseconds * 1e-9
            self.topic_rates['imu'].tick(receive_time)
            self.latest_imu = {
                'timestamp': self._stamp_to_sec(msg.header.stamp),
                'receive_time': receive_time,
                'frame_id': msg.header.frame_id,
                'orientation': self._quaternion_to_list(msg.orientation),
                'angular_velocity': self._vector3_to_list(msg.angular_velocity),
                'linear_acceleration': self._vector3_to_list(msg.linear_acceleration),
                'orientation_covariance': [float(v) for v in msg.orientation_covariance],
                'angular_velocity_covariance': [float(v) for v in msg.angular_velocity_covariance],
                'linear_acceleration_covariance': [float(v) for v in msg.linear_acceleration_covariance],
            }
    
    def auto_capture(self):
        """自动采集一帧数据"""
        self.save_frame()
    
    def save_frame(self):
        """保存当前帧所有数据"""
        with self.lock:
            # 递增接收帧计数
            self.frames_received += 1
            
            # 跳过前20帧（初始化期间）
            if self.frames_received <= 20:
                self.get_logger().info(f"⏭️  跳过初始化帧 {self.frames_received}/20")
                return
            
            if self.latest_left is None:
                self.get_logger().warn("尚未接收到左目图像，跳过保存")
                return
            
            try:
                capture_time = self.get_clock().now().nanoseconds * 1e-9

                # 保存左目
                left_path = self.output_dir / f'left' / f'{self.frame_id:06d}.png'
                cv2.imwrite(str(left_path), self.latest_left)
                
                # 保存右目
                if self.latest_right is not None:
                    right_path = self.output_dir / f'right' / f'{self.frame_id:06d}.png'
                    cv2.imwrite(str(right_path), self.latest_right)
                
                # 保存深度图（float32 NPY 格式）
                if self.latest_depth is not None:
                    depth_path = self.output_dir / f'depth' / f'{self.frame_id:06d}.npy'
                    np.save(str(depth_path), self.latest_depth)
                
                # 保存 roadside_data 中的偏角和距离（JSON）
                if self.latest_roadside is not None:
                    roadside_path = self.output_dir / 'roadside' / f'{self.frame_id:06d}.json'
                    with open(roadside_path, 'w') as f:
                        json.dump(self.latest_roadside, f, indent=2)

                # 保存 avoid_controller 输出的 LQR/控制角速度（JSON）
                if self.latest_cmd_vel is not None:
                    control_path = self.output_dir / 'control' / f'{self.frame_id:06d}.json'
                    with open(control_path, 'w') as f:
                        json.dump(self.latest_cmd_vel, f, indent=2)

                # 保存里程计和 IMU（JSON），布局与 zed_dataset_player.py 兼容
                if self.latest_odom is not None:
                    odom_path = self.output_dir / 'odom' / f'{self.frame_id:06d}.json'
                    with open(odom_path, 'w') as f:
                        json.dump(self.latest_odom, f, indent=2)

                if self.latest_imu is not None:
                    imu_path = self.output_dir / 'imu' / f'{self.frame_id:06d}.json'
                    with open(imu_path, 'w') as f:
                        json.dump(self.latest_imu, f, indent=2)

                cmd_vel_hz = self.topic_rates['cmd_vel'].hz()
                bluetooth_feedback_hz = self.topic_rates['bluetooth_feedback'].hz()
                roadside_hz = self.topic_rates['roadside'].hz()
                road_mask_hz = self.topic_rates['road_mask'].hz()
                odom_hz = self.topic_rates['odom'].hz()
                if self.latest_odom and self.latest_odom['source_topic'] == '/zed/odom':
                    odom_hz = self.topic_rates['zed_odom'].hz()
                imu_hz = self.topic_rates['imu'].hz()

                roadside_time = ''
                right_distance = ''
                right_angle = ''
                if self.latest_roadside is not None:
                    roadside_time = self.latest_roadside['timestamp']
                    right_distance = self.latest_roadside['right_distance']
                    right_angle = self.latest_roadside['right_angle']

                cmd_vel_receive_time = ''
                linear_x = ''
                angular_z_lqr = ''
                if self.latest_cmd_vel is not None:
                    cmd_vel_receive_time = self.latest_cmd_vel['receive_time']
                    linear_x = self.latest_cmd_vel['linear'][0]
                    angular_z_lqr = self.latest_cmd_vel['angular_z_lqr']

                bluetooth_feedback_time = ''
                bluetooth_velocity = ''
                bluetooth_steering = ''
                if self.latest_bluetooth_feedback is not None:
                    bluetooth_feedback_time = self.latest_bluetooth_feedback['receive_time']
                    bluetooth_velocity = self.latest_bluetooth_feedback['velocity']
                    bluetooth_steering = self.latest_bluetooth_feedback['steering']

                road_mask_time = ''
                if self.latest_road_mask is not None:
                    road_mask_time = self.latest_road_mask['receive_time']

                odom_time = ''
                odom_receive_time = ''
                odom_source = ''
                odom_x = ''
                odom_y = ''
                odom_z = ''
                if self.latest_odom is not None:
                    odom_time = self.latest_odom['timestamp']
                    odom_receive_time = self.latest_odom['receive_time']
                    odom_source = self.latest_odom['source_topic']
                    odom_x, odom_y, odom_z = self.latest_odom['position']

                imu_time = ''
                imu_receive_time = ''
                imu_qx = ''
                imu_qy = ''
                imu_qz = ''
                imu_qw = ''
                if self.latest_imu is not None:
                    imu_time = self.latest_imu['timestamp']
                    imu_receive_time = self.latest_imu['receive_time']
                    imu_qx, imu_qy, imu_qz, imu_qw = self.latest_imu['orientation']

                with open(self.control_csv_path, 'a') as f:
                    f.write(
                        f'{self.frame_id},{capture_time},{roadside_time},{right_distance},'
                        f'{right_angle},{cmd_vel_receive_time},{linear_x},{angular_z_lqr},'
                        f'{cmd_vel_hz},{bluetooth_feedback_time},{bluetooth_velocity},'
                        f'{bluetooth_steering},{bluetooth_feedback_hz},{roadside_hz},'
                        f'{road_mask_time},{road_mask_hz},'
                        f'{odom_time},{odom_receive_time},{odom_source},{odom_x},{odom_y},{odom_z},{odom_hz},'
                        f'{imu_time},{imu_receive_time},{imu_qx},{imu_qy},{imu_qz},{imu_qw},{imu_hz}\n')
                
                status = "✓"
                detail = f"帧 {self.frame_id}"
                if self.latest_roadside:
                    detail += (
                        f" Roadside:✓(dist:{self.latest_roadside['right_distance']:.3f}m,"
                        f" angle:{self.latest_roadside['right_angle']:.2f}deg,"
                        f" {self._format_hz(roadside_hz)})")
                else:
                    detail += " Roadside:✗"

                if self.latest_cmd_vel:
                    detail += (
                        f" CmdVel:✓(v:{linear_x:.3f}, w:{angular_z_lqr:.3f}, "
                        f"{self._format_hz(cmd_vel_hz)})")
                else:
                    detail += " CmdVel:✗"

                if self.latest_bluetooth_feedback:
                    detail += (
                        f" BLE_FB:✓(v:{bluetooth_velocity:.3f}, steer:{bluetooth_steering:.3f}, "
                        f"{self._format_hz(bluetooth_feedback_hz)})")
                else:
                    detail += " BLE_FB:✗"

                if self.latest_road_mask:
                    detail += f" Mask:✓({self._format_hz(road_mask_hz)})"
                else:
                    detail += " Mask:✗"

                if self.latest_odom:
                    detail += (
                        f" Odom:✓({odom_source}, x:{odom_x:.3f}, y:{odom_y:.3f}, "
                        f"{self._format_hz(odom_hz)})")
                else:
                    detail += " Odom:✗"

                if self.latest_imu:
                    detail += (
                        f" IMU:✓(qz:{imu_qz:.3f}, qw:{imu_qw:.3f}, "
                        f"{self._format_hz(imu_hz)})")
                else:
                    detail += " IMU:✗"
                
                self.get_logger().info(f"{status} {detail}")
                self.frame_id += 1
                
                # 检查是否达到最大帧数
                if self.max_frames > 0 and self.frame_id >= self.max_frames:
                    self.get_logger().info(f"已达到最大帧数 ({self.max_frames})，停止采集")
                    if hasattr(self, 'timer'):
                        self.timer.cancel()
                    
            except Exception as e:
                self.get_logger().error(f"保存帧失败: {e}")
    
    def print_status(self):
        """打印采集状态"""
        elapsed = time.time() - self.start_time
        fps = self.frame_id / elapsed if elapsed > 0 else 0
        self.get_logger().info(f"采集统计: 已保存 {self.frame_id} 帧，耗时 {elapsed:.1f}s，速率 {fps:.2f} fps")

    def _format_hz(self, hz):
        if hz == '':
            return 'Hz:--'
        return f'Hz:{hz:.2f}'


def get_next_dataset_dir(base_dir, prefix):
    """返回第一个不存在的 prefixN 数据集目录路径"""
    base_path = Path(base_dir)
    index = 0
    while True:
        candidate = base_path / f'{prefix}{index}'
        if not candidate.exists():
            return str(candidate)
        index += 1


def main():
    parser = argparse.ArgumentParser(description="ZED 数据集采集节点 (从 avoid_controller 订阅)")
    parser.add_argument('--output', type=str, default=None,
                        help='数据集保存目录 (默认: ~/zed_dataset/haizhuhuN 自动递增)')
    parser.add_argument('--interval', type=float, default=0.2,
                        help='自动采集间隔(秒)，0表示手动模式 (默认: 0.2)')
    parser.add_argument('--max_frames', type=int, default=1000,
                        help='最大采集帧数，0表示不限制 (默认: 1000)')
    parser.add_argument('--headless', action='store_true',
                        help='无 GUI 模式（SSH 远程时使用）')
    parser.add_argument('--gui', dest='headless', action='store_false',
                        help='启用 GUI 模式')
    parser.set_defaults(headless=True)
    
    args = parser.parse_args()
    
    # 展开 ~ 路径；默认使用 ~/zed_dataset/haizhuhuN，若 haizhuhu0 已存在则自动递增。
    if args.output is None:
        output_dir = get_next_dataset_dir(os.path.expanduser('~/zed_dataset'), 'haizhuhu')
    else:
        output_dir = os.path.expanduser(args.output)
    
    # 初始化 ROS2
    rclpy.init()
    
    # 创建节点
    node = ZEDDatasetCollector(
        output_dir=output_dir,
        interval=args.interval,
        max_frames=args.max_frames,
        headless=args.headless
    )
    
    try:
        # 启动自动采集后的使用提示
        node.get_logger().info(f"✓ 节点运行中，按 Ctrl+C 停止采集")
        if args.interval > 0:
            node.get_logger().info(f"✓ 自动采集模式，间隔 {args.interval}s")
        else:
            node.get_logger().info(f"✓ 手动采集模式，调用 node.save_frame() 保存数据")
        
        # 当interval=0时，可通过其他方式手动触发（这里主要是自动模式）
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info(f"\n✓ 正在停止...")
        node.print_status()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
