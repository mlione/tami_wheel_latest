#!/usr/bin/env python3
"""从视频文件逐帧发布 /rgb_image 话题 (bgra8)，用于 fusion_node 数据回放测试"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
import cv2
import numpy as np


class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')

        # 参数
        self.declare_parameter('video_path', '/home/tensorlab/alphawheel/wheel_cuda/haizhuhu.mp4')
        self.declare_parameter('fps', 20.0)  # 0 表示用视频原始帧率

        video_path = self.get_parameter('video_path').get_parameter_value().string_value
        self.target_fps = self.get_parameter('fps').get_parameter_value().double_value

        # 打开视频
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            self.get_logger().error(f'无法打开视频: {video_path}')
            return

        video_fps = self.cap.get(cv2.CAP_PROP_FPS)
        total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        if self.target_fps <= 0:
            self.target_fps = video_fps

        self.get_logger().info(
            f'视频: {video_path}\n'
            f'  分辨率: {width}x{height}\n'
            f'  帧率: {video_fps} fps\n'
            f'  总帧数: {total_frames}\n'
            f'  发布帧率: {self.target_fps} fps'
        )

        # 发布者，QoS 与 fusion_node 订阅端匹配
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub = self.create_publisher(Image, '/rgb_image', qos)

        self.frame_idx = 0
        period = 1.0 / self.target_fps
        self.timer = self.create_timer(period, self.tick)

    def tick(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info(f'视频播放完毕，共 {self.frame_idx} 帧，重新循环...')
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            self.frame_idx = 0
            ret, frame = self.cap.read()
            if not ret:
                return

        # OpenCV 读取为 BGR 3通道，fusion_node 期望 bgra8 (4通道)
        bgra = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height, msg.width = bgra.shape[:2]
        msg.encoding = 'bgra8'
        msg.is_bigendian = 0
        msg.step = bgra.shape[1] * 4  # width * 4 bytes
        msg.data = bgra.tobytes()

        self.pub.publish(msg)

        if self.frame_idx % 60 == 0:
            self.get_logger().info(f'已发布帧 {self.frame_idx}')

        self.frame_idx += 1


def main():
    rclpy.init()
    node = VideoPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
