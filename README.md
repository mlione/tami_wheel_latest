# TAMI Wheel：道路约束 DWA + LQR 轮椅导航

> 当前开发版本：**DWA + LQR v0.4（2026-09-19）**
> 完整改进、参数、测试结果与已知问题见 [`dwa+lqr_v0.4.md`](dwa+lqr_v0.4.md)。

本项目在原有 ZED RGB-D、TensorRT BiSeNet 语义分割、CUDA 点云融合、道路边界提取和 LQR 巡线基础上，增加道路约束 DWA 局部规划、分级安全层、RViz 轨迹可视化、ZED 运动估计接口及独立 BLE 底盘桥。

> **安全声明**：本项目涉及载人轮椅。当前结果只支持数据集、室内空载和受控低速研究测试，不能视为已达到载人自主运行条件。实车必须配备独立物理急停、安全员，并完成速度、制动距离、外参、轮椅外廓和 BLE 失联测试。

## 系统架构

```text
ZED RGB-D / 历史数据集
        |
BiSeNet TensorRT + CUDA 点云融合
        |
ObstacleFusion：道路边界、道路方向、障碍信息
        |
Safety Layer：紧急距离 / 感知超时 / 无安全轨迹
        |
        +-- 无近障：CRUISE 道路参考轨迹
        +-- 有近障：道路约束 DWA 候选与最优轨迹
                              |
                     LQR 前视点轨迹跟踪
                              |
                    /cmd_vel（m/s、rad/s）
                              |
                 ble_hardware_bridge（独立启动）
                              |
                          轮椅底盘
```

主要代码：

| 模块 | 路径 |
|---|---|
| 感知与融合 | `src/wheel_perception/src/fusion_node.cpp` |
| ZED 驱动与视觉里程计读取 | `src/wheel_perception/src/core/zed_driver.cpp` |
| 控制与安全层 | `src/wheel_perception/src/controller_node.cpp` |
| DWA | `src/wheel_perception/dwa_controller/src/DWAPlanner.cpp` |
| LQR | `src/wheel_perception/include/wheel_perception/core/lqr_controller.hpp` |
| 统一导航参数 | `src/wheel_perception/config/params.yaml` |
| BLE 桥 | `ble_hardware_bridge/ble_hardware_bridge/` |

## v0.4 主要变化

- DWA 根据障碍点云预测常值 `(v,w)` 轨迹，并施加碰撞、道路边界、动态窗口、最小转弯半径和 BLE 可执行域硬约束；
- 允许低速滚动转弯，当前仍禁止 `v=0,w!=0` 的原地转向；
- 无近障时使用确定性 CRUISE 轨迹，达到激活点数后才使用 DWA；
- 轨迹评分包含道路航向、障碍净空、速度、道路位置和平滑代价；
- LQR 不再直接根据道路误差决定整条路径，而是跟踪 DWA/CRUISE 局部轨迹的前视点；
- `/cmd_vel` 统一为标准 SI 单位，旧 `sub.py` 和 `lqr.gain` 协议缩放已移除；
- 新增独立 `ble_hardware_bridge`，负责物理速度到摇杆/GATT 协议的转换；
- ZED WORLD 绝对位姿按图像时间戳差分为 `base_link` 速度并发布 `/odom`，SDK Twist 只作诊断；
- 数据集模式使用上一控制输出构造动态窗口，实车模式才尝试使用 `/odom.twist`，两者互不污染；
- RViz 区分绿色巡线、蓝色 DWA 路径、蓝色停车点以及有效/无效候选。

当前代码**尚未实现**此前讨论的“净空改善奖励”“安全条件停车惩罚”“障碍空间聚类”和“普通急停 2/3 帧确认”。这些内容仍列为后续工作，详见 v0.4 文档。

## DWA 与 LQR

DWA 使用二维运动学模型：

```text
x_dot   = v*cos(yaw)
y_dot   = v*sin(yaw)
yaw_dot = w
```

当前评分函数为：

```text
score =
    weight_heading  * cos(yaw_end - road_yaw_error)
  - weight_obstacle * (1 / minimum_clearance)
  - weight_velocity * (max_velocity - v)
  - weight_road     * mean_road_offset
  - weight_smooth   * smooth_cost
```

碰撞、越过道路边界、超过动态能力或 BLE 执行域的轨迹直接无效。轮椅以半径 `robot_radius` 的圆形 footprint 近似。LQR 在选中轨迹上取 `lookahead_time` 前视点，利用横向误差和航向误差修正 DWA 参考角速度。

当前主要参数：

```yaml
dwa:
  max_velocity: 1.0
  max_angular_velocity: 1.0
  max_acceleration: 0.5
  max_angular_acceleration: 5.0
  prediction_time: 3.0
  simulation_time_step: 0.1
  minimum_turning_radius: 0.1
  cruise_velocity: 0.6
  robot_radius: 0.45
  activation:
    max_x: 5.0
    min_y: -1.0
    max_y: 1.0
    minimum_points: 20
    clear_frames: 5
safety:
  emergency_stop_distance: 0.5
```

这些是当前调试值，不是最终载人安全参数。ROS 2 参数类型严格，例如 `prediction_time` 必须写 `3.0`，不能写成整数 `3`。

## 构建

```bash
cd ~/tami/tami_wheel_latest
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.sh
```

当前工作区生成的是 `install/setup.sh`。目标 GPU 需要显式 CUDA 架构时：

```bash
colcon build --symlink-install \
  --cmake-args -DWHEEL_CUDA_ARCHITECTURES="75;86"
```

请先检查 `src/wheel_perception/config/params.yaml` 中的 `ai.engine_path`。当前值仍指向旧目录：

```text
/home/x/tami/wheel_latest/src/wheel_perception/config/combined5.engine
```

当前仓库本身没有该 engine；迁移到开发板前必须改成目标机真实存在、且与 GPU/CUDA/TensorRT 匹配的文件。

运行 DWA 测试：

```bash
cmake --build build/wheel_perception --target test_dwa_planner -j2
./build/wheel_perception/test_dwa_planner
```

当前源码有 13 项 DWA 回归测试。

## 数据集回放

确认 `params.yaml`：

```yaml
zed:
  use_dataset_mode: true
```

启动一次 `text6_stereo` 回放：

```bash
cd ~/tami/tami_wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.sh

ros2 launch wheel_perception dwa_dataset_test.launch.py \
  dataset_path:=/home/x/tami/datasets6/text6_stereo \
  fps:=20.0 \
  loop:=false
```

观察规划推荐 `10~20 FPS`，评估吞吐再使用 `30 FPS`。过低 FPS 会改变动态窗口；`loop:=true` 会在首尾产生数据突变，不适合统计一次完整控制曲线。数据集 launch 不启动 BLE 桥。

常用检查：

```bash
ros2 topic hz /perception/output
ros2 topic echo /perception/output --once
ros2 topic echo /dwa/planner_cmd
ros2 topic echo /cmd_vel
```

记录一次回放：

```bash
mkdir -p bags
ros2 bag record -o bags/dwa_dataset_once \
  /perception/output /dwa/planner_cmd /cmd_vel \
  /dwa/obstacle_cloud /dwa/local_trajectory /dwa/best_path
```

## RViz

```bash
rviz2 -d install/wheel_perception/share/wheel_perception/rviz/dwa_navigation.rviz
```

Fixed Frame 设为 `base_link`：

- 亮绿色：CRUISE 最优路径，DWA 未参与；
- 蓝色曲线：DWA 选中的避障路径；
- 蓝色圆点：DWA 主动选择停车；
- 半透明绿色：有效 DWA 候选；
- 半透明红色：无效候选；
- 黄色：道路右边界调试点。

主要话题：

| 话题 | 含义 |
|---|---|
| `/perception/output` | 道路和融合障碍结果 |
| `/zed/point_cloud` | 完整调试点云 |
| `/dwa/obstacle_cloud` | 实际进入 DWA 的 ROI 点云 |
| `/dwa/planner_cmd` | DWA 参考 `(v,w)` |
| `/dwa/local_trajectory` | LQR 正在跟踪的局部轨迹 |
| `/dwa/best_path_marker` | 绿色/蓝色选中轨迹或蓝色停车点 |
| `/dwa/candidate_paths` | DWA 候选轨迹 |
| `/perception/debug/right_road_edge` | 道路右边界点 |
| `/debug/viz` | 语义分割和边界叠加图 |
| `/odom` | 实车模式下实验性的 ZED 基座运动估计 |
| `/zed/diagnostics/sdk_twist` | 原始 SDK Twist 的 `base_link` 诊断值，不参与控制 |
| `/cmd_vel` | LQR 后的最终标准物理速度 |

## 只接 ZED 的规划测试

将 `zed.use_dataset_mode` 改为 `false`，不启动 BLE：

```bash
ros2 launch wheel_perception run_launch.py
```

此模式能验证实时感知、DWA 路径和 ZED `/odom`，但不能验证底盘跟踪效果。检查：

```bash
ros2 topic hz /odom
ros2 topic echo /odom --field twist.twist
ros2 topic echo /zed/diagnostics/sdk_twist --field twist
ros2 run tf2_ros tf2_echo base_link zed_left_camera_frame
```

## 实车启动

先启动感知与控制：

```bash
cd ~/tami/tami_wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.sh
ros2 launch wheel_perception run_launch.py
```

确认道路、点云、轨迹、急停和 `/cmd_vel` 正常后，才在独立终端启动 BLE：

```bash
source /opt/ros/humble/setup.bash
source ~/tami/tami_wheel_latest/install/setup.sh
ros2 launch ble_hardware_bridge ble_hardware_bridge.launch.py
```

不要在数据集回放时启动 BLE。蓝牙地址、特征 UUID 和速度标定在：

```text
ble_hardware_bridge/ble_hardware_bridge/config/ble_hardware_bridge.yaml
```

## ZED 速度反馈现状

旧版 rosbag 测试曾表明直接使用 SDK Twist 的绝对量级不可靠：

- `/odom` 实际约 `16.23 Hz`，而 ZED 配置为 `60 FPS`；
- WORLD pose 差分速度约为旧 SDK Twist 的 `0.27` 倍，二者趋势高度相关；
- 原始 `linear.x` 峰值约 `4.61 m/s`，对轮椅明显不合理；
- 控制器偶发报告反馈 stale，疑似 `/odom` 回调与耗时控制回调互斥导致排队。

当前代码已经改为：使用 ZED 图像时间戳对连续 WORLD 绝对位姿差分，先通过外参消除相机杆臂，再生成 `/odom.twist`；旧 SDK Twist 仅发布到 `/zed/diagnostics/sdk_twist`。数据集模式继续使用上一控制输出，两种模式互不影响。

该实现已经通过直行速度、杆臂补偿和异常时间间隔单元测试，但尚需用已知距离、秒表、轮编码器或动捕完成实车尺度验证。因此 `/odom.twist` 仍属于视觉惯性估计，不应当作轮编码器真值。当前配置保持：

```yaml
velocity_feedback:
  stop_on_timeout: false
```

详细实现和测试步骤见 [`zed_world_pose_velocity_report.md`](zed_world_pose_velocity_report.md)。下一步应让 `/odom` 使用独立/reentrant callback group，并用已知距离或轮编码器标定。

## 常见问题

| 现象 | 排查方向 |
|---|---|
| `/perception/output` 类型 invalid | source ROS 和当前工作区的 `install/setup.sh`。 |
| `/rgb_image` 正常但 `/debug/viz` 卡住 | lifecycle、TensorRT engine、GPU 推理和感知频率。 |
| 只有绿色路径 | 障碍点未达到激活区域的 20 点门槛。 |
| 蓝色圆点 | DWA 正常运行，但停车轨迹评分最高。 |
| 很多浅绿色线 | 有效候选，不是多个最终路径；蓝线才是选中结果。 |
| 障碍可见但 DWA 不响应 | 看 `/dwa/obstacle_cloud`，不要只看完整点云。 |
| 绿线大幅摆动且 `w_track` 饱和 | 检查 `road_yaw_error`、右边界方向和距离，不一定是 DWA。 |
| 近障起步不动 | 净空倒数、平滑项、heading/road 约束及硬件可执行域共同使停车得分更高。 |
| 参数 invalid type | 需要 double 的值必须保留小数点。 |
| 换目录后 CMake 指向旧源码 | 删除确认无用的旧 build/install 缓存后重新构建。 |

## 当前边界与后续优先级

当前适合验证静态局部障碍物和道路约束，不具备完整全局导航、动态障碍速度预测、非圆形 footprint、倒车脱困或原地转向。

后续优先级：

1. 实车标定并验证 WORLD 位姿差分速度尺度；
2. 为 `/odom` 分离 callback group；
3. 增加障碍空间聚类和多帧一致性；
4. 实现“真实净空改善奖励 + 有条件停车惩罚”；
5. 标定轮椅 footprint、转弯能力和制动距离；
6. 完成 BLE 断连和独立安全制动测试。

更完整的测试数据、问题原因和部署注意事项请阅读 [`dwa+lqr_v0.4.md`](dwa+lqr_v0.4.md)。
