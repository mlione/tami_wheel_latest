# BLE底盘桥接迁移修改报告

## 1. 修改目标

本次将旧的 `wheel_perception/scripts/sub.py` 替换为独立 ROS 2 功能包
`ble_hardware_bridge`。感知/规划和蓝牙底盘仍分开启动，没有增加一键联合启动。

改造后的唯一接口约定为：

```text
/cmd_vel.linear.x  = m/s
/cmd_vel.angular.z = rad/s（ROS正值表示左转）
```

`ControllerNode` 只发布物理速度，蓝牙桥是唯一负责挡位选择、遥感映射和 BLE 帧编码的模块。

## 2. 删除的旧逻辑

- 删除 `src/wheel_perception/scripts/sub.py`。
- 从 `wheel_perception/CMakeLists.txt` 删除 `sub.py` 安装项。
- 删除 `LqrController::Config::lqr_gain`。
- 删除 `LqrController::toActuatorCommand()` 和未使用的 `compute()` 协议转换入口。
- 删除 `params.yaml` 中的 `lqr.gain: 80.0`。
- 删除 `ControllerNode` 中 `lqr.gain` 的声明和读取。

此次没有修改 LQR 的 `Q/R`、误差状态或 DARE 求解逻辑；删除的是与旧蓝牙协议耦合的放大与符号转换。

## 3. 控制链修改

### 3.1 标准 cmd_vel

`controller_node.cpp` 从：

```cpp
command.angular.z = lqr_->toActuatorCommand(tracked_w);
```

改为：

```cpp
command.angular.z = tracked_w;
```

因此 `/dwa/planner_cmd` 表示 DWA 参考物理速度，`/cmd_vel` 表示 LQR 跟踪后的最终物理速度。

### 3.2 DWA硬件可执行域

DWA 新增保守的 BLE 底盘约束：

| 线速度 | 最大角速度 |
|---|---:|
| 停车 | 仅允许 `(0,0)` |
| `0.15 <= v < 0.35 m/s` | `abs(w) <= 0.30 rad/s` |
| `0.35 <= v < 0.70 m/s` | `abs(w) <= 0.60 rad/s` |
| `v >= 0.70 m/s` | `abs(w) <= 0.90 rad/s` |

非零且低于 `0.15 m/s` 的候选会被拒绝，避免 DWA 选中“数学上在运动、实车因遥感死区静止”的轨迹。

从零速起步时，规划器允许一次跨过执行器死区到 `0.15 m/s`；在最小可执行速度时也允许直接回到零速。安全层停车、无路径停车和 DWA 主动选择 `(0,0)` 都仍然有效。

### 3.3 LQR之后再限幅

DWA 候选先经过硬件可执行域筛选；LQR 加入跟踪修正后，最终 `tracked_w` 再按同一挡位限制截断。这防止 LQR 将合法的 DWA 参考推出蓝牙底盘可执行范围。

## 4. BLE桥接修改

`ble_hardware_bridge` 现在从 YAML 读取：

- `adapter_path`：BlueZ 适配器路径；
- `device_address`：轮椅 BLE MAC；
- `characteristic_uuid`：GATT 写特征 UUID；
- `steering_sign`：硬件方向修正，只允许 `1.0` 或 `-1.0`。

这些信息不再硬编码在 `bluez_ble_client.cpp`。如悬空轮测试发现左右相反，只修改：

```yaml
steering_sign: -1.0
```

不应再改 DWA/LQR 的符号。

指令超时由 `1.0 s` 缩短为 `0.3 s`；超时、无效指令和节点退出都会尝试发送停车帧。软件超时不能替代底盘原生失联停车和物理急停。

## 5. 参数

`src/wheel_perception/config/params.yaml`：

```yaml
dwa:
  hardware_constraints:
    enabled: true
    minimum_moving_velocity: 0.15
    gear_0001_threshold: 0.35
    gear_0003_threshold: 0.70
    gear_0001_max_angular: 0.30
    gear_0003_max_angular: 0.60
    gear_0005_max_angular: 0.90
```

`ble_hardware_bridge/config/ble_hardware_bridge.yaml` 保留精确的遥感标定、轮距和三挡映射。DWA 参数是其保守子集，修改挡位阈值时必须同时核对两个 YAML。

## 6. 启动方式

### 6.1 数据集回放

```bash
cd ~/tami/tami_wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.sh
ros2 launch wheel_perception dwa_dataset_test.launch.py \
  dataset_path:=/home/x/tami/datasets6/text6_stereo
```

数据集回放时不得启动 BLE 桥。

### 6.2 实车感知与规划

```bash
ros2 launch wheel_perception run_launch.py
```

该命令不会启动蓝牙。先检查感知、RViz、`/dwa/planner_cmd` 和 `/cmd_vel`。

### 6.3 蓝牙底盘（独立终端）

```bash
source /opt/ros/humble/setup.bash
source ~/tami/tami_wheel_latest/install/setup.sh
ros2 launch ble_hardware_bridge ble_hardware_bridge.launch.py
```

不要再运行 `ros2 run wheel_perception sub.py`；该脚本已删除。

## 7. 验证结果

- `ble_hardware_bridge` 编译通过；
- `wheel_perception` 编译通过；
- BLE `ControlProtocol` 11 项单元测试全部通过；
- DWA 13 项单元测试全部通过；
- 全工作区构建被新增的可选 `zed2i_nitros` 包阻断，原因是本机没有 `isaac_ros_common`；与本次 BLE/DWA 修改无关。

## 8. 实车验收顺序

1. 不连底盘运行感知和控制，确认 `/cmd_vel.angular.z` 与日志 `w_track` 一致且不再出现几十的协议值。
2. 关闭 `run_launch.py`，将驱动轮悬空，单独启动 BLE 桥。
3. 分别发布 `(0.15, 0)`、`(0.15, +0.10)`、`(0.15, -0.10)` 和 `(0,0)`。
4. 确认左右相反时只修改 `steering_sign`。
5. 用 `ros2 topic info /cmd_vel -v` 确认完整运行时只有 ControllerNode 是常规发布者。
6. 依次完成悬空轮、地面空载、无障碍、远障碍、近障碍和急停测试；未通过前禁止载人。
