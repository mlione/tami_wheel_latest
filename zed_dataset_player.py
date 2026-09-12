#!/usr/bin/env python3
"""
ZED 数据集回放节点
加载 zed_capture_dataset.py 采集的数据集，以 ROS2 话题发布，
供 zw_Bisenet_CRF.py / depth_combine.py 等节点使用。

发布话题:
    /rgb_image   (sensor_msgs/Image, bgra8)  - 左目图像，与 avoid_controller 发布格式一致
    /depth_image (sensor_msgs/Image, 32FC1)  - 深度图，单位: 米
    /zed/imu     (sensor_msgs/Imu)           - IMU 传感器数据
    /zed/odom    (nav_msgs/Odometry)         - 里程计数据

用法:
    # 调整回放帧率
    ros2 run wheel zed_dataset_player.py --ros-args -p dataset_path:=/path/to/dataset -p fps:=15.0

    # 循环回放
    ros2 run wheel zed_dataset_player.py --ros-args -p dataset_path:=~/zed_dataset/text1 -p loop:=true

    python3 zed_dataset_player.py --dataset /home/tensorlab/zed_dataset/smy --fps 10 --loop
    python3 zed_dataset_player.py --dataset /home/tensorlab/zed_dataset/haizhuhu7 --fps 10 --loop
    python3 zed_dataset_player.py --dataset /home/tensorlab/zed_dataset/shengwudao1 --fps 10 --loop
"""

import os
import sys
import json
import argparse
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Twist
from cv_bridge import CvBridge


class DatasetPlayerNode(Node):
    def __init__(self, dataset_path=None, fps=None, loop=None):
        super().__init__('zed_dataset_player')

        # ROS2 参数
        self.declare_parameter('dataset_path', dataset_path or os.path.expanduser('~/zed_dataset'))
        self.declare_parameter('fps', fps or 10.0)
        self.declare_parameter('loop', loop if loop is not None else True)
        self.declare_parameter('rgb_topic', '/rgb_image')
        self.declare_parameter('depth_topic', '/depth_image')

        self.dataset_path = os.path.expanduser(self.get_parameter('dataset_path').value)
        self.fps = float(self.get_parameter('fps').value)
        self.loop = bool(self.get_parameter('loop').value)
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value

        if not self.dataset_path or not os.path.isdir(self.dataset_path):
            self.get_logger().fatal(f'数据集路径无效: "{self.dataset_path}"')
            self.get_logger().fatal('请检查 ~/zed_dataset 是否存在，或通过 --dataset /path/to/dataset 指定')
            raise RuntimeError('dataset_path invalid')

        # 加载元数据
        meta_path = os.path.join(self.dataset_path, 'metadata.json')
        if os.path.isfile(meta_path):
            with open(meta_path, 'r') as f:
                self.metadata = json.load(f)
            self.total_frames = self.metadata.get('total_frames', 0)
        else:
            # 没有 metadata，按文件名扫描
            self.metadata = None
            left_dir = os.path.join(self.dataset_path, 'left')
            self.total_frames = len([f for f in os.listdir(left_dir) if f.endswith('.png')]) if os.path.isdir(left_dir) else 0

        if self.total_frames == 0:
            self.get_logger().fatal('数据集中没有帧数据!')
            raise RuntimeError('empty dataset')

        # 检测可用的深度数据源。
        # 兼容两种数据集布局：
        # 1. 新布局：depth_raw/ depth_npy/ depth_png/
        # 2. 旧布局：depth/*.npy （由 zed_capture_dataset.py 生成）
        self.depth_raw_dir = os.path.join(self.dataset_path, 'depth_raw')
        self.depth_npy_dir = os.path.join(self.dataset_path, 'depth_npy')
        self.depth_legacy_dir = os.path.join(self.dataset_path, 'depth')
        self.depth_png_dir = os.path.join(self.dataset_path, 'depth_png')
        self.depth_dir = None

        if self._has_depth_files(self.depth_raw_dir, '.bin'):
            self.depth_source = 'raw'
            self.depth_dir = self.depth_raw_dir
        elif self._has_depth_files(self.depth_npy_dir, '.npy'):
            self.depth_source = 'npy'
            self.depth_dir = self.depth_npy_dir
        elif self._has_depth_files(self.depth_legacy_dir, '.npy'):
            self.depth_source = 'npy'
            self.depth_dir = self.depth_legacy_dir
            self.get_logger().info('检测到旧版数据集布局: depth/*.npy，将按 NPY 深度图加载')
        elif self._has_depth_files(self.depth_png_dir, '.png'):
            self.depth_source = 'png'
            self.depth_dir = self.depth_png_dir
        else:
            self.depth_source = None
            self.get_logger().warn('未找到深度数据，仅发布 RGB 图像')

        # 加载相机标定信息 (可选，仅打印)
        cam_info_path = os.path.join(self.dataset_path, 'camera_info.json')
        if os.path.isfile(cam_info_path):
            with open(cam_info_path, 'r') as f:
                cam_info = json.load(f)
            left_cam = cam_info.get('left_camera', {})
            res = cam_info.get('resolution', {})
            self.img_width = res.get('width', 0)
            self.img_height = res.get('height', 0)
            self.get_logger().info(
                f'相机标定: fx={left_cam.get("fx")}, fy={left_cam.get("fy")}, '
                f'cx={left_cam.get("cx")}, cy={left_cam.get("cy")}, '
                f'分辨率={self.img_width}x{self.img_height}')
        else:
            self.img_width = 0
            self.img_height = 0

        # 离线 RGB-D 单帧较大，BEST_EFFORT 在本机 DDS 压力较高时会大量
        # 丢帧，造成融合图像长时间不更新。图像改用小队列可靠传输；IMU/
        # odom 仍保持低延迟 SensorDataQoS。
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.rgb_pub = self.create_publisher(Image, rgb_topic, image_qos)
        self.depth_pub = self.create_publisher(Image, depth_topic, image_qos)
        self.imu_pub = self.create_publisher(Imu, '/zed/imu', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/zed/odom', qos_profile_sensor_data)
        self.bridge = CvBridge()

        self.current_frame = 0
        interval = 1.0 / max(0.1, self.fps)
        self.timer = self.create_timer(interval, self.publish_frame)

        self.get_logger().info(
            f'数据集回放节点已启动: {self.total_frames} 帧, '
            f'{self.fps} FPS, 循环={self.loop}, 深度源={self.depth_source}')
        if self.depth_dir:
            self.get_logger().info(f'  Depth dir -> {self.depth_dir}')
        self.get_logger().info(f'  RGB -> {rgb_topic} (bgra8)')
        self.get_logger().info(f'  Depth -> {depth_topic} (32FC1)')
        self.get_logger().info(f'  IMU -> /zed/imu (sensor_msgs/Imu)')
        self.get_logger().info(f'  Odometry -> /zed/odom (nav_msgs/Odometry)')

    def _has_depth_files(self, directory, suffix):
        if not os.path.isdir(directory):
            return False
        return any(name.endswith(suffix) for name in os.listdir(directory))

    def load_imu(self, fname):
        """加载 IMU 数据, 返回字典或 None"""
        imu_path = os.path.join(self.dataset_path, 'imu', f'{fname}.json')
        if os.path.isfile(imu_path):
            try:
                with open(imu_path, 'r') as f:
                    return json.load(f)
            except:
                return None
        return None
    
    def load_odom(self, fname):
        """加载里程计数据, 返回字典或 None"""
        odom_path = os.path.join(self.dataset_path, 'odom', f'{fname}.json')
        if os.path.isfile(odom_path):
            try:
                with open(odom_path, 'r') as f:
                    return json.load(f)
            except:
                return None
        return None

    def load_depth(self, fname):
        """加载深度图, 返回 float32 numpy array (单位: 米)"""
        if self.depth_source == 'raw':
            raw_path = os.path.join(self.depth_dir, f'{fname}.bin')
            if os.path.isfile(raw_path):
                depth = np.fromfile(raw_path, dtype=np.float32)
                # 需要知道图像尺寸来 reshape
                if self.img_height > 0 and self.img_width > 0:
                    expected_size = self.img_height * self.img_width
                    if depth.size != expected_size:
                        raise ValueError(
                            f'raw depth size mismatch: got {depth.size}, expected {expected_size}')
                    depth = depth.reshape(self.img_height, self.img_width)
                else:
                    depth = depth.reshape(-1)  # 返回 1D，下面会处理
                return depth
        elif self.depth_source == 'npy':
            npy_path = os.path.join(self.depth_dir, f'{fname}.npy')
            if os.path.isfile(npy_path):
                return np.load(npy_path).astype(np.float32)
        elif self.depth_source == 'png':
            png_path = os.path.join(self.depth_dir, f'{fname}.png')
            if os.path.isfile(png_path):
                depth_mm = cv2.imread(png_path, cv2.IMREAD_UNCHANGED)
                if depth_mm is not None:
                    return depth_mm.astype(np.float32) / 1000.0
        return None

    def publish_frame(self):
        if self.current_frame >= self.total_frames:
            if self.loop:
                self.current_frame = 0
                self.get_logger().info('循环回放: 从头开始')
            else:
                self.get_logger().info('回放结束')
                self.timer.cancel()
                return

        fname = f'{self.current_frame:06d}'
        try:
            self._publish_frame(fname)
        except Exception as e:
            self.get_logger().warn(f'帧 {fname} 数据异常, 已跳过: {e}')
        finally:
            self.current_frame += 1

    def _publish_frame(self, fname):
        stamp = self.get_clock().now().to_msg()

        # 读取左目图像
        left_path = os.path.join(self.dataset_path, 'left', f'{fname}.png')
        if not os.path.isfile(left_path):
            self.get_logger().warn(f'帧 {fname} 左图不存在, 跳过')
            return

        left_bgr = cv2.imread(left_path, cv2.IMREAD_COLOR)
        if left_bgr is None:
            self.get_logger().warn(f'帧 {fname} 左图读取失败, 跳过')
            return

        # 转换为 BGRA8 (与 avoid_controller 发布的 /rgb_image 编码一致)
        left_bgra = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2BGRA)

        # 更新图像尺寸 (如果还不知道)
        if self.img_height == 0:
            self.img_height, self.img_width = left_bgra.shape[:2]

        # 先完成本帧全部消息构造，避免坏帧只发布出一部分话题。
        rgb_msg = self.bridge.cv2_to_imgmsg(left_bgra, encoding='bgra8')
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = 'camera_link'

        depth_msg = None
        depth = self.load_depth(fname)
        if depth is not None and depth.ndim == 2:
            depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='32FC1')
            depth_msg.header.stamp = stamp
            depth_msg.header.frame_id = 'camera_link'

        imu_msg = None
        imu_data = self.load_imu(fname)
        if imu_data is not None:
            imu_msg = Imu()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = 'camera'
            
            # 加速度
            linear_acc = imu_data.get('linear_acceleration', [0, 0, 0])
            imu_msg.linear_acceleration.x = float(linear_acc[0])
            imu_msg.linear_acceleration.y = float(linear_acc[1])
            imu_msg.linear_acceleration.z = float(linear_acc[2])
            
            # 角速度
            angular_vel = imu_data.get('angular_velocity', [0, 0, 0])
            imu_msg.angular_velocity.x = float(angular_vel[0])
            imu_msg.angular_velocity.y = float(angular_vel[1])
            imu_msg.angular_velocity.z = float(angular_vel[2])
            
            # 方向四元数
            orientation = imu_data.get('orientation', [0, 0, 0, 1])
            imu_msg.orientation.x = float(orientation[0])
            imu_msg.orientation.y = float(orientation[1])
            imu_msg.orientation.z = float(orientation[2])
            imu_msg.orientation.w = float(orientation[3])
            
            # 设置协方差
            imu_msg.linear_acceleration_covariance = [0.0] * 9
            imu_msg.angular_velocity_covariance = [0.0] * 9
            imu_msg.orientation_covariance = [0.0] * 9

        odom_msg = None
        odom_data = self.load_odom(fname)
        if odom_data is not None:
            odom_msg = Odometry()
            odom_msg.header.stamp = stamp
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'camera'
            
            # 位置
            position = odom_data.get('position', [0, 0, 0])
            odom_msg.pose.pose.position.x = float(position[0])
            odom_msg.pose.pose.position.y = float(position[1])
            odom_msg.pose.pose.position.z = float(position[2])
            
            # 方向四元数
            orientation = odom_data.get('orientation', [0, 0, 0, 1])
            odom_msg.pose.pose.orientation.x = float(orientation[0])
            odom_msg.pose.pose.orientation.y = float(orientation[1])
            odom_msg.pose.pose.orientation.z = float(orientation[2])
            odom_msg.pose.pose.orientation.w = float(orientation[3])
            
            # 设置协方差（可选）
            odom_msg.pose.covariance = [0.0] * 36
            odom_msg.twist.covariance = [0.0] * 36

        # 发布 RGB 图像 (BGRA8)
        self.rgb_pub.publish(rgb_msg)

        # 发布深度图 (32FC1)
        if depth_msg is not None:
            self.depth_pub.publish(depth_msg)

        # 发布 IMU 数据
        if imu_msg is not None:
            self.imu_pub.publish(imu_msg)

        # 发布里程计数据
        if odom_msg is not None:
            self.odom_pub.publish(odom_msg)

        frame_index = int(fname)
        if frame_index % 50 == 0:
            self.get_logger().info(f'回放进度: {frame_index}/{self.total_frames}')


def main(args=None):
    # 支持命令行参数 (非 ROS 方式运行时)
    parser = argparse.ArgumentParser(description='ZED 数据集 ROS2 回放节点')
    parser.add_argument('--dataset', type=str, default=os.path.expanduser('~/zed_dataset'), help='数据集路径 (默认: ~/zed_dataset)')
    parser.add_argument('--fps', type=float, default=10.0, help='回放帧率 (默认: 10.0)')
    parser.add_argument('--loop', dest='loop', action='store_true', help='循环回放 (默认: 启用)')
    parser.add_argument('--no-loop', dest='loop', action='store_false', help='禁用循环回放')
    parser.set_defaults(loop=True)
    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    try:
        node = DatasetPlayerNode(
            dataset_path=cli_args.dataset,
            fps=cli_args.fps,
            loop=cli_args.loop,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'节点错误: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
