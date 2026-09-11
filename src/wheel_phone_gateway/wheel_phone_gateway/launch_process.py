"""受控启停 ROS2 launch 进程组。"""

import os
import signal
import subprocess
import threading
from typing import Any, Dict, Optional, Sequence


PERCEPTION_LAUNCH_ID = 'perception'
PERCEPTION_LAUNCH_COMMAND = (
    'ros2',
    'launch',
    'wheel_perception',
    'run_launch.py',
)


def validate_launch_request(payload: Any) -> str:
    """校验手机端的 launch 请求，不允许传入任意命令或参数。"""
    if not isinstance(payload, dict):
        raise ValueError('请求体必须是 JSON 对象')
    launch_id = payload.get('launch_id')
    if launch_id != PERCEPTION_LAUNCH_ID:
        raise ValueError('仅允许启动 wheel_perception/run_launch.py')
    arguments = payload.get('arguments', '')
    if arguments not in ('', None):
        raise ValueError('当前 launch 不允许手机端传入自定义参数')
    return launch_id


class LaunchProcessManager:
    """管理唯一的 ROS2 launch 进程组，防止重复启动和孤儿节点。"""

    def __init__(self, command: Sequence[str]) -> None:
        self.command = tuple(command)
        self._lock = threading.Lock()
        self._process: Optional[subprocess.Popen] = None
        self._last_exit_code: Optional[int] = None

    def _status_locked(self) -> Dict[str, Any]:
        process = self._process
        if process is None:
            is_failed = self._last_exit_code not in (None, 0)
            state = 'failed' if is_failed else 'stopped'
            return {
                'state': state,
                'pid': None,
                'exit_code': self._last_exit_code,
            }

        exit_code = process.poll()
        if exit_code is None:
            return {'state': 'running', 'pid': process.pid, 'exit_code': None}

        self._process = None
        self._last_exit_code = exit_code
        state = 'failed' if exit_code != 0 else 'stopped'
        return {'state': state, 'pid': None, 'exit_code': exit_code}

    def status(self) -> Dict[str, Any]:
        """返回 launch 当前状态，并回收已退出的子进程。"""
        with self._lock:
            return self._status_locked()

    def start(self) -> Dict[str, Any]:
        """启动白名单中的 launch；已运行时保持幂等。"""
        with self._lock:
            status = self._status_locked()
            if status['state'] == 'running':
                status['message'] = 'wheel_perception Launch 已在运行'
                return status

            try:
                # 单独创建进程组，停止时可以一并关闭 launch 及其子节点。
                self._process = subprocess.Popen(
                    self.command,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            except OSError as error:
                raise RuntimeError(
                    '无法启动 ros2 launch，请确认网关进程已 source ROS2 '
                    '与 wheel_cuda/install/setup.bash'
                ) from error

            self._last_exit_code = None
            return {
                'state': 'running',
                'pid': self._process.pid,
                'exit_code': None,
                'message': 'wheel_perception Launch 已启动',
            }

    def stop(self, timeout_sec: float = 8.0) -> Dict[str, Any]:
        """先用 SIGINT 优雅停止，超时后逐级终止整个进程组。"""
        with self._lock:
            status = self._status_locked()
            process = self._process
            if process is None or status['state'] != 'running':
                status['message'] = 'wheel_perception Launch 未运行'
                return status

            try:
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=timeout_sec)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2.0)

            self._last_exit_code = process.poll()
            self._process = None
            return {
                'state': 'stopped',
                'pid': None,
                'exit_code': self._last_exit_code,
                'message': 'wheel_perception Launch 已停止',
            }
