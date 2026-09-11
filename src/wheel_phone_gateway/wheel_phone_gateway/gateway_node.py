#!/usr/bin/env python3
"""将 ROS2 原始调试图像通过 HTTP/WebSocket 转发给手机 App。"""

import base64
import hashlib
import json
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Dict, List, Optional, Set, Type

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image

from wheel_phone_gateway.launch_process import LaunchProcessManager
from wheel_phone_gateway.launch_process import PERCEPTION_LAUNCH_COMMAND
from wheel_phone_gateway.launch_process import validate_launch_request


WEBSOCKET_MAGIC = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'


def encode_websocket_frame(payload: bytes, opcode: int = 0x1) -> bytes:
    """编码服务端到客户端的未掩码 WebSocket 数据帧。"""
    length = len(payload)
    if length < 126:
        header = struct.pack('!BB', 0x80 | opcode, length)
    elif length <= 0xFFFF:
        header = struct.pack('!BBH', 0x80 | opcode, 126, length)
    else:
        header = struct.pack('!BBQ', 0x80 | opcode, 127, length)
    return header + payload


def read_exact(connection: socket.socket, size: int) -> bytes:
    """读取指定长度的数据，连接关闭时抛出 ConnectionError。"""
    chunks = bytearray()
    while len(chunks) < size:
        chunk = connection.recv(size - len(chunks))
        if not chunk:
            raise ConnectionError('WebSocket 客户端已断开')
        chunks.extend(chunk)
    return bytes(chunks)


def resize_for_stream(
    image: np.ndarray,
    width: int,
    height: int,
) -> np.ndarray:
    """将图像缩放到手机传输尺寸，相同尺寸时避免额外复制。"""
    if image.shape[1] == width and image.shape[0] == height:
        return image
    return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)


class WebSocketClient:
    """线程安全的 WebSocket 客户端连接。"""

    def __init__(self, connection: socket.socket) -> None:
        self.connection = connection
        self.send_lock = threading.Lock()

    def send_json(self, payload: Dict[str, Any]) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode('utf-8')
        with self.send_lock:
            self.connection.sendall(encode_websocket_frame(data))

    def send_control(self, opcode: int, payload: bytes = b'') -> None:
        with self.send_lock:
            self.connection.sendall(encode_websocket_frame(payload, opcode))

    def send_binary(self, payload: bytes) -> None:
        """发送二进制数据，图片不再经过 base64 和 JSON。"""
        with self.send_lock:
            self.connection.sendall(encode_websocket_frame(payload, 0x2))

    def send_image(self, jpeg: bytes, fps: float) -> None:
        """按顺序发送帧信息和二进制 JPEG。"""
        metadata = json.dumps({
            'type': 'frame',
            'fps': fps,
        }).encode('utf-8')
        frames = (
            encode_websocket_frame(metadata)
            + encode_websocket_frame(jpeg, 0x2)
        )
        with self.send_lock:
            self.connection.sendall(frames)


class DebugVizGateway(Node):
    """订阅 `/debug/viz`，编码 JPEG 后推送给手机。"""

    def __init__(self) -> None:
        super().__init__('debug_viz_gateway')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8080)
        self.declare_parameter('image_topic', '/debug/viz')
        self.declare_parameter('stream_fps', 15.0)
        self.declare_parameter('jpeg_quality', 55)
        self.declare_parameter('output_width', 640)
        self.declare_parameter('output_height', 360)

        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.stream_fps = max(
            float(self.get_parameter('stream_fps').value),
            0.1,
        )
        self.jpeg_quality = min(
            max(int(self.get_parameter('jpeg_quality').value), 1),
            100,
        )
        self.output_width = max(
            int(self.get_parameter('output_width').value),
            1,
        )
        self.output_height = max(
            int(self.get_parameter('output_height').value),
            1,
        )

        self.bridge = CvBridge()
        self.ws_clients: Set[WebSocketClient] = set()
        self.ws_clients_lock = threading.Lock()
        self.frame_lock = threading.Lock()
        self.latest_jpeg: Optional[bytes] = None
        self.latest_frame_id = 0
        self.sent_frame_id = -1
        self.last_encode_time = 0.0
        self.last_frame_time: Optional[float] = None
        self.frame_times: List[float] = []
        self.streaming_enabled = True
        self.stop_event = threading.Event()
        self.launch_manager = LaunchProcessManager(PERCEPTION_LAUNCH_COMMAND)
        self.last_launch_state = 'stopped'

        # 只保留最新一帧：丢帧优先于排队，避免手机看到过期画面。
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            image_qos,
        )

        handler_class = self._create_handler()
        self.http_server = ThreadingHTTPServer(
            (self.host, self.port),
            handler_class,
        )
        self.http_server.daemon_threads = True
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            daemon=True,
        )
        self.broadcast_thread = threading.Thread(
            target=self._broadcast_loop,
            daemon=True,
        )
        self.launch_monitor_thread = threading.Thread(
            target=self._launch_monitor_loop,
            daemon=True,
        )
        self.http_thread.start()
        self.broadcast_thread.start()
        self.launch_monitor_thread.start()
        self.get_logger().info(
            f'手机图像网关已启动：http://{self.host}:{self.port}，'
            f'订阅={self.image_topic}，输出={self.output_width}x'
            f'{self.output_height}@{self.stream_fps:.1f}FPS，'
            f'JPEG={self.jpeg_quality}'
        )

    def _image_callback(self, message: Image) -> None:
        """接收 ROS Image，限频转换为 BGR8 并编码为 JPEG。"""
        now = time.monotonic()
        if not self.streaming_enabled:
            return
        if now - self.last_encode_time < 1.0 / self.stream_fps:
            return

        try:
            bgr_image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='bgr8',
            )
            stream_image = resize_for_stream(
                bgr_image,
                self.output_width,
                self.output_height,
            )
            success, encoded = cv2.imencode(
                '.jpg',
                stream_image,
                [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
            )
        except (CvBridgeError, cv2.error) as error:
            self.get_logger().warning(f'图像转换失败：{error}')
            return
        if not success:
            self.get_logger().warning('JPEG 编码失败，本帧已跳过')
            return

        self.last_encode_time = now
        with self.frame_lock:
            self.latest_jpeg = encoded.tobytes()
            self.latest_frame_id += 1
            self.last_frame_time = now
            self.frame_times.append(now)
            cutoff = now - 1.0
            self.frame_times = [
                timestamp
                for timestamp in self.frame_times
                if timestamp >= cutoff
            ]

    def _broadcast_loop(self) -> None:
        period = 1.0 / self.stream_fps
        while not self.stop_event.wait(period):
            with self.frame_lock:
                if (
                    not self.streaming_enabled
                    or self.latest_jpeg is None
                    or self.latest_frame_id == self.sent_frame_id
                ):
                    continue
                jpeg = self.latest_jpeg
                frame_id = self.latest_frame_id
                fps = float(len(self.frame_times))
                self.sent_frame_id = frame_id

            self._broadcast_image(jpeg, fps)

    def _broadcast_image(self, jpeg: bytes, fps: float) -> None:
        """向客户端发送最新 JPEG，慢连接不会影响 ROS2 图像回调。"""
        with self.ws_clients_lock:
            clients = list(self.ws_clients)
        stale_clients: List[WebSocketClient] = []
        for client in clients:
            try:
                client.send_image(jpeg, fps)
            except (ConnectionError, OSError):
                stale_clients.append(client)
        if stale_clients:
            with self.ws_clients_lock:
                self.ws_clients.difference_update(stale_clients)

    def _broadcast(self, payload: Dict[str, Any]) -> None:
        with self.ws_clients_lock:
            clients = list(self.ws_clients)
        stale_clients: List[WebSocketClient] = []
        for client in clients:
            try:
                client.send_json(payload)
            except (ConnectionError, OSError):
                stale_clients.append(client)
        if stale_clients:
            with self.ws_clients_lock:
                self.ws_clients.difference_update(stale_clients)

    def _health_payload(self) -> Dict[str, Any]:
        now = time.monotonic()
        with self.frame_lock:
            fps = float(len(self.frame_times))
            frame_age = (
                None
                if self.last_frame_time is None
                else round(now - self.last_frame_time, 3)
            )
        with self.ws_clients_lock:
            client_count = len(self.ws_clients)
        return {
            'ok': True,
            'topic': self.image_topic,
            'streaming': self.streaming_enabled,
            'clients': client_count,
            'fps': fps,
            'last_frame_age_sec': frame_age,
            'output_width': self.output_width,
            'output_height': self.output_height,
            'transport': 'binary_websocket',
            'launch': self.launch_manager.status(),
        }

    def _publish_launch_status(self, result: Dict[str, Any]) -> None:
        """将 launch 运行状态同步给所有已连接手机。"""
        self.last_launch_state = str(result['state'])
        self._broadcast({
            'type': 'status',
            'message': result['message'],
            'launch_state': result['state'],
        })

    def _launch_monitor_loop(self) -> None:
        """监视 launch 异常退出，避免手机长期显示错误运行状态。"""
        while not self.stop_event.wait(0.5):
            status = self.launch_manager.status()
            state = str(status['state'])
            if state == self.last_launch_state:
                continue
            if state == 'failed':
                status['message'] = (
                    'wheel_perception Launch 异常退出，'
                    f"退出码={status['exit_code']}"
                )
            else:
                status['message'] = 'wheel_perception Launch 已停止'
            self._publish_launch_status(status)

    def _create_handler(self) -> Type[BaseHTTPRequestHandler]:
        gateway = self

        class RequestHandler(BaseHTTPRequestHandler):
            server_version = 'WheelPhoneGateway/0.1'

            def log_message(self, message_format: str, *args: Any) -> None:
                gateway.get_logger().info(message_format % args)

            def _send_json(
                self,
                status: int,
                payload: Dict[str, Any],
            ) -> None:
                data = json.dumps(payload, ensure_ascii=False).encode('utf-8')
                self.send_response(status)
                self.send_header(
                    'Content-Type',
                    'application/json; charset=utf-8',
                )
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(data)

            def do_GET(self) -> None:
                if self.path == '/health':
                    self._send_json(200, gateway._health_payload())
                    return
                if self.path == '/api/launch/status':
                    self._send_json(200, gateway.launch_manager.status())
                    return
                if self.path != '/ws/mobile':
                    self._send_json(404, {'message': '接口不存在'})
                    return
                self._serve_websocket()

            def do_POST(self) -> None:
                paths = ('/api/launch/start', '/api/launch/stop')
                if self.path not in paths:
                    self._send_json(404, {'message': '接口不存在'})
                    return
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if length > 4096:
                        self._send_json(413, {'message': '请求体过大'})
                        return
                    payload = json.loads(self.rfile.read(length) or b'{}')
                    validate_launch_request(payload)
                except (ValueError, json.JSONDecodeError) as error:
                    self._send_json(400, {'message': str(error)})
                    return
                try:
                    if self.path.endswith('/start'):
                        gateway.streaming_enabled = True
                        result = gateway.launch_manager.start()
                    else:
                        result = gateway.launch_manager.stop()
                except RuntimeError as error:
                    self._send_json(500, {'message': str(error)})
                    return
                gateway._publish_launch_status(result)
                self._send_json(200, result)

            def do_OPTIONS(self) -> None:
                self.send_response(204)
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header(
                    'Access-Control-Allow-Headers',
                    'Content-Type',
                )
                self.send_header(
                    'Access-Control-Allow-Methods',
                    'GET, POST, OPTIONS',
                )
                self.end_headers()

            def _serve_websocket(self) -> None:
                key = self.headers.get('Sec-WebSocket-Key')
                is_websocket = (
                    self.headers.get('Upgrade', '').lower() == 'websocket'
                )
                if not is_websocket or not key:
                    self._send_json(426, {'message': '需要 WebSocket Upgrade'})
                    return
                accept_source = (key + WEBSOCKET_MAGIC).encode('ascii')
                accept_digest = hashlib.sha1(accept_source).digest()
                accept = base64.b64encode(accept_digest).decode('ascii')
                self.send_response(101, 'Switching Protocols')
                self.send_header('Upgrade', 'websocket')
                self.send_header('Connection', 'Upgrade')
                self.send_header('Sec-WebSocket-Accept', accept)
                self.end_headers()

                client = WebSocketClient(self.connection)
                with gateway.ws_clients_lock:
                    gateway.ws_clients.add(client)
                gateway.get_logger().info('手机 WebSocket 已连接')
                try:
                    launch_status = gateway.launch_manager.status()
                    client.send_json({
                        'type': 'status',
                        'message': f'已订阅 {gateway.image_topic}',
                        'launch_state': launch_status['state'],
                    })
                    self._receive_websocket(client)
                except (ConnectionError, OSError):
                    pass
                finally:
                    with gateway.ws_clients_lock:
                        gateway.ws_clients.discard(client)
                    gateway.get_logger().info('手机 WebSocket 已断开')
                    self.close_connection = True

            def _receive_websocket(self, client: WebSocketClient) -> None:
                while not gateway.stop_event.is_set():
                    first, second = read_exact(self.connection, 2)
                    opcode = first & 0x0F
                    masked = bool(second & 0x80)
                    length = second & 0x7F
                    if length == 126:
                        length = struct.unpack(
                            '!H', read_exact(self.connection, 2)
                        )[0]
                    elif length == 127:
                        length = struct.unpack(
                            '!Q', read_exact(self.connection, 8)
                        )[0]
                    mask = read_exact(self.connection, 4) if masked else b''
                    payload = read_exact(self.connection, length)
                    if masked:
                        payload = bytes(
                            value ^ mask[index % 4]
                            for index, value in enumerate(payload)
                        )
                    if opcode == 0x8:
                        client.send_control(0x8)
                        return
                    if opcode == 0x9:
                        client.send_control(0xA, payload)

        return RequestHandler

    def destroy_node(self) -> bool:
        self.stop_event.set()
        self.launch_manager.stop()
        self.http_server.shutdown()
        self.http_server.server_close()
        self.http_thread.join(timeout=2.0)
        self.broadcast_thread.join(timeout=2.0)
        self.launch_monitor_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[DebugVizGateway] = None
    try:
        node = DebugVizGateway()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except OSError as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f'debug_viz_gateway: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
