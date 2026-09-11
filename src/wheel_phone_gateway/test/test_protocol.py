import numpy as np
import pytest
from unittest.mock import Mock, patch

from wheel_phone_gateway.gateway_node import encode_websocket_frame
from wheel_phone_gateway.gateway_node import resize_for_stream
from wheel_phone_gateway.gateway_node import WebSocketClient
from wheel_phone_gateway.launch_process import LaunchProcessManager
from wheel_phone_gateway.launch_process import PERCEPTION_LAUNCH_COMMAND
from wheel_phone_gateway.launch_process import validate_launch_request


class RecordingConnection:
    """记录 WebSocket 客户端写入的数据。"""

    def __init__(self) -> None:
        self.data = b''

    def sendall(self, data: bytes) -> None:
        self.data += data


def test_encode_short_text_frame() -> None:
    """短文本应生成 FIN + text opcode 的标准 WebSocket 帧。"""
    assert encode_websocket_frame(b'ok') == b'\x81\x02ok'


def test_encode_large_text_frame() -> None:
    """大于 65535 字节的数据应使用 64 位扩展长度。"""
    payload = b'x' * 65536
    frame = encode_websocket_frame(payload)
    assert frame[:10] == b'\x81\x7f\x00\x00\x00\x00\x00\x01\x00\x00'
    assert frame[10:] == payload


def test_resize_for_stream() -> None:
    """720p 输入应缩放为配置的 640×360 输出。"""
    image = np.zeros((720, 1280, 3), dtype=np.uint8)
    resized = resize_for_stream(image, width=640, height=360)
    assert resized.shape == (360, 640, 3)


def test_send_binary_uses_binary_opcode() -> None:
    """JPEG 必须使用二进制 WebSocket 帧，而不是 base64 文本。"""
    connection = RecordingConnection()
    client = WebSocketClient(connection)  # type: ignore[arg-type]
    client.send_binary(b'jpeg')
    assert connection.data == b'\x82\x04jpeg'


def test_validate_launch_request_only_accepts_perception() -> None:
    """手机端只能请求固定的 wheel_perception launch。"""
    assert validate_launch_request({'launch_id': 'perception'}) == 'perception'
    with pytest.raises(ValueError):
        validate_launch_request({'launch_id': 'custom'})
    with pytest.raises(ValueError):
        validate_launch_request({
            'launch_id': 'perception',
            'arguments': 'dangerous:=true',
        })


@patch('wheel_phone_gateway.launch_process.subprocess.Popen')
def test_launch_manager_start_is_idempotent(popen: Mock) -> None:
    """重复点击启动时不应创建第二个 launch 进程。"""
    process = Mock(pid=1234)
    process.poll.return_value = None
    popen.return_value = process
    manager = LaunchProcessManager(PERCEPTION_LAUNCH_COMMAND)

    assert manager.start()['state'] == 'running'
    assert manager.start()['pid'] == 1234
    popen.assert_called_once()


@patch('wheel_phone_gateway.launch_process.subprocess.Popen')
def test_launch_manager_reports_unexpected_exit(popen: Mock) -> None:
    """launch 异常退出后应更新为 failed，不继续报告 running。"""
    process = Mock(pid=1234)
    process.poll.return_value = 9
    popen.return_value = process
    manager = LaunchProcessManager(PERCEPTION_LAUNCH_COMMAND)
    manager.start()

    status = manager.status()
    assert status['state'] == 'failed'
    assert status['exit_code'] == 9


@patch('wheel_phone_gateway.launch_process.os.killpg')
@patch('wheel_phone_gateway.launch_process.subprocess.Popen')
def test_launch_manager_stops_process_group(
    popen: Mock,
    killpg: Mock,
) -> None:
    """停止操作应向 launch 进程组发送 SIGINT。"""
    process = Mock(pid=4321)
    process.poll.side_effect = [None, None, 0]
    popen.return_value = process
    manager = LaunchProcessManager(PERCEPTION_LAUNCH_COMMAND)
    manager.start()

    assert manager.stop()['state'] == 'stopped'
    killpg.assert_called_once_with(4321, 2)
