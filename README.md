# TAMI Wheel Latest：DWA + LQR 轮椅自主导航

本项目是面向载人轮椅的 ROS 2 自主导航系统。在原有“ZED RGB-D + TensorRT BiSeNet 语义分割 + 点云融合 + 道路边界 + LQR 巡线”的基础上，增加了受道路边界约束的 DWA（Dynamic Window Approach）局部规划器。

当前版本不重构原始感知和底盘通信链路，而是在其上增加局部避障、轨迹安全性判断、候选轨迹平滑和可视化能力。

> **安全声明**：本项目涉及载人轮椅。数据集回放、RViz 和低速空载测试通过，不代表可直接载人运行。实车前必须完成急停、坐标系、底盘协议、制动距离、轮椅外廓和场地安全验证。

## 系统结构

```text
ZED RGB-D / 历史数据集
        |
TensorRT BiSeNet 语义分割 + CUDA 点云融合
        |
ObstacleFusion：道路右边界、道路宽度、道路朝向、最小距离
        |
        +-------------------- Safety Layer ---------------------+
        |     紧急距离 / 感知超时 / 无安全轨迹 → 立即停车       |
        +--------------------------------------------------------+
        |
有效点云 ROI + DWA 激活判断
        |
        +-- 无近距离障碍：道路巡线参考轨迹（CRUISE，绿色）
        |
        +-- 有近距离障碍：DWA 候选预测与评分（DWA，蓝色）
                                      |
                               局部最优轨迹 (v, w)
                                      |
                           LQR 前视点跟踪与角速度限幅
                                      |
                         cmd_vel → 原有轮椅底盘通信桥
```

`FusionNode` 和 `ControllerNode` 以 ROS 2 composable node 的形式运行在 `wheel_container` 中；`FusionNode` 使用 lifecycle 管理。核心入口为：

- 感知：`src/wheel_perception/src/fusion_node.cpp`
- 控制：`src/wheel_perception/src/controller_node.cpp`
- DWA：`src/wheel_perception/dwa_controller/src/DWAPlanner.cpp`
- LQR：`src/wheel_perception/include/wheel_perception/core/lqr_controller.hpp`
- 统一参数：`src/wheel_perception/config/params.yaml`

## 相比原始 LQR 巡线版本的创新与改进

### 1. 从直接巡线到分层局部规划

原始控制链以道路边界误差和航向误差为主，直接由 LQR 输出转向，缺少对未来局部轨迹和障碍物净空的显式预测。当前链路变为：

```text
道路边界 + 障碍点云 → DWA 生成安全局部轨迹 → LQR 跟踪该轨迹 → 底盘
```

DWA 负责“往哪里走、走多快、是否安全”，LQR 保留原有横向和航向闭环能力，负责平滑地跟踪 DWA 给出的局部参考。

### 2. 道路约束 DWA，而不是纯避障 DWA

DWA 同时使用右道路边界距离、道路宽度、动态右侧目标距离、道路航向误差和道路安全内缩量。候选轨迹越过左右道路边界时直接判为无效，而不是仅仅给予较高代价；避障不能以驶出道路为代价。

### 3. 障碍触发 DWA，直路使用稳定巡线

DWA 不再在每一帧、每一段直路都持续切换候选路径。只有足够多的有效障碍点进入前方激活区域时才参与规划：

```yaml
dwa:
  activation:
    max_x: 2.5
    min_y: -0.9
    max_y: 0.9
    minimum_points: 10
    clear_frames: 5
```

无障碍时生成确定性的道路巡线参考轨迹，再交给 LQR 跟踪。这减少了无意义的候选切换和左右摆动。

### 4. DWA 专用点云 ROI

完整点云仍发布到 `/zed/point_cloud` 供调试，但 DWA 只消费以下三维 ROI 内的点，并投影为二维碰撞点：

```yaml
dwa:
  obstacle_roi:
    min_x: 0.3
    max_x: 2.5
    min_y: -1.5
    max_y: 1.5
    min_z: -0.2
    max_z: 1.0
```

处理顺序为“先 ROI 过滤、后均匀降采样、再投影到二维”，避免树冠等密集高处点占满采样预算。实际进入规划器的点云可通过 `/dwa/obstacle_cloud` 检查。

### 5. 轨迹平滑与轨迹保持

除障碍、道路和速度评分外，DWA 使用上一帧命令和加速度历史，惩罚：

```text
|v(k) - v(k-1)|、|w(k) - w(k-1)|
|a(k)|、|alpha(k)|
|jerk(k)|、|angular_jerk(k)|
```

如果上一轨迹仍安全，且新轨迹的评分或净空优势不足，就继续使用上一轨迹，避免两个相近候选逐帧来回切换。

### 6. 当前阶段禁用原地转向

当 `v < minimum_turning_velocity` 时，非零 `w` 候选会被拒绝。当前不包含原地左转或原地右转状态，仅保留 `(v=0, w=0)` 安全停车候选。

### 7. 数据集与实车速度反馈隔离

- 数据集模式：动态窗口使用本次控制器的 `last_output_velocity_`，不使用历史录制速度作为当前反馈。
- 实车模式：优先使用真实 `/odom.twist`；不可用时才回退到上一控制输出。

这避免历史数据集车速和当前回放控制输出互相污染，同时保持实车使用真实速度反馈。

## DWA 工作原理

### 运动学和轮椅外形模型

DWA 使用局部二维单轨/差速型运动学模型：

```text
x_dot   = v * cos(yaw)
y_dot   = v * sin(yaw)
yaw_dot = w
```

每个候选 `(v, w)` 在预测周期内保持常值；默认以 `0.1 s` 积分 `2.5 s`。动态窗口由当前速度、最大线加速度和最大角加速度构造。

轮椅碰撞外形当前近似为一个圆：

```yaml
dwa:
  robot_radius: 0.45  # m
```

任一预测点到障碍点的距离减去 `robot_radius` 后，若小于等于零，则该轨迹碰撞并被淘汰。

当前实现没有显式轴距、轮距、转向角、轮胎侧偏或电机动力学模型；它假设底盘能跟随受加速度限制的 `(v, w)` 指令。实车应将 `robot_radius` 标定为可覆盖轮椅、脚踏板和乘员的保守外廓。

### 硬约束与评分函数

候选轨迹只有同时满足以下条件才有效：位于动态速度窗口内、加速度不超限、不与点云碰撞、不驶出道路边界、且不产生低速非零转向。

有效轨迹最大化以下评分：

```text
score =
    weight_heading  * heading_reward
  - weight_obstacle * obstacle_cost
  - weight_velocity * velocity_cost
  - weight_road     * road_cost
  - weight_smooth   * smooth_cost
```

- `heading_reward`：预测终点朝向和道路朝向的相似度；
- `obstacle_cost`：整条路径最小净空的倒数；
- `velocity_cost`：`max_velocity - v`，鼓励安全前进；
- `road_cost`：轨迹偏离目标巡线位置的平均距离；
- `smooth_cost`：速度、角速度、加速度和 jerk 的归一化变化量。

碰撞、越界或动态不可达的候选评分直接设为无效。DWA 已激活却找不到安全路径时，若 `safety.stop_on_no_path: true`，系统停车而不沿用旧命令。

## LQR 在新框架中的职责

DWA 输出最优局部轨迹和物理参考角速度 `w_ref`。控制器取 `lqr.lookahead_time`（默认 `0.6 s`）处的局部轨迹点，计算横向误差和航向误差：

```text
w_track = w_ref + u_lqr(lateral_error, heading_error)
```

`w_track` 再经过角速度和角加速度限幅，最后转成原有轮椅通信协议所需的转向量。

> `/dwa/planner_cmd.angular.z` 是物理角速度，单位 `rad/s`；`/cmd_vel.angular.z` 已经过 `lqr.gain` 和符号转换，是底盘桥使用的协议转向值，不能把它直接当成物理 `rad/s`。

## 主要话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/perception/output` | `wheel_msgs/msg/PerceptionOutput` | 道路边界、道路宽度、道路航向、融合最小距离 |
| `/zed/point_cloud` | `sensor_msgs/msg/PointCloud2` | 感知调试用完整点云 |
| `/dwa/obstacle_cloud` | `sensor_msgs/msg/PointCloud2` | 实际进入 DWA 的 ROI 二维障碍点云 |
| `/dwa/planner_cmd` | `geometry_msgs/msg/Twist` | DWA 的物理 `(v, w)` 输出 |
| `/dwa/local_trajectory` | `nav_msgs/msg/Path` | 当前 LQR 跟踪的局部轨迹 |
| `/dwa/best_path` | `nav_msgs/msg/Path` | 当前最优局部轨迹 |
| `/dwa/best_path_marker` | `visualization_msgs/msg/Marker` | 绿色巡线、蓝色 DWA 避障轨迹 |
| `/dwa/candidate_paths` | `visualization_msgs/msg/MarkerArray` | DWA 候选；绿为有效，红为无效 |
| `/perception/debug/right_road_edge` | `sensor_msgs/msg/PointCloud2` | 右道路边界点云 |
| `/debug/viz` | `sensor_msgs/msg/Image` | 语义分割、道路及调试叠加图 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 最终发给原有底盘通信桥的控制指令 |

## 编译

在项目根目录执行：

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash

# 编译
colcon build 
source install/setup.bash
```

若目标机 CUDA 架构需要显式指定，可在构建命令末尾增加：

```bash
--cmake-args -DWHEEL_CUDA_ARCHITECTURES="75;86"
```

该架构值必须与目标 GPU 一致；不要把桌面 GPU 架构盲目复制到 Jetson。修改 C++、CMake、launch 或 YAML 后，建议重新构建对应包并重新 `source install/setup.bash`。

运行 DWA 单元测试：

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon test --packages-select wheel_perception
colcon test-result --verbose
```

## 数据集回放

### 1. 回放前检查

确认 `src/wheel_perception/config/params.yaml`：

```yaml
zed:
  use_dataset_mode: true
```

同时确认 `ai.engine_path` 指向目标机可用的 TensorRT engine；engine 通常与 GPU、CUDA 和 TensorRT 版本相关，不能保证跨设备直接复用。

### 2. 启动回放

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch wheel_perception dwa_dataset_test.launch.py \
  dataset_path:=/home/x/tami/shengwudao2 \
  fps:=20.0 \
  loop:=true
```

此 launch 只启动数据集播放器、感知和控制组件，**不会**启动蓝牙底盘桥。建议先从 `15~20 FPS` 开始评估，再测试 `30 FPS` 下的稳定性和计算负载。

正常启动后应依次看到：

```text
FusionNode Configuring parameters
Transitioning successful
zed_dataset_player: 数据集回放节点已启动
CRUISE+LQR 或 DWA+LQR
```

`CRUISE+LQR` 表示当前没有满足触发条件的近距离障碍；`DWA+LQR` 表示 DWA 正在避障。这两种日志均为正常状态。

### 3. 记录控制话题

另开终端：

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /dwa/planner_cmd
ros2 topic echo /cmd_vel
```

分析 DWA 时应优先使用 `/dwa/planner_cmd` 的物理 `v/w`；不要把 `/cmd_vel.angular.z` 误当成 DWA 的物理角速度。

## RViz 可视化

启动回放或感知系统后，在另一个终端执行：

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.bash

rviz2 -d install/wheel_perception/share/wheel_perception/rviz/dwa_navigation.rviz
```

建议将 Fixed Frame 设为 `base_link`。重点观察：

- `/dwa/obstacle_cloud`：确认树冠等无关点没有进入 DWA；
- `/dwa/candidate_paths`：确认候选是否大量碰撞或越界；
- `/dwa/best_path_marker`：绿色为普通巡线，蓝色为 DWA 避障；
- `/perception/debug/right_road_edge`：黄色右道路边界；
- `/debug/viz`：检查语义分割与道路边界是否持续更新。

## 实车运行

### 1. 实车前配置

修改 `params.yaml`：

```yaml
zed:
  use_dataset_mode: false
```

还必须逐项确认：

1. ZED 已连接，TensorRT engine 能在目标机加载；
2. `/odom` 是真实定位/底盘反馈，且 `twist.twist.linear.x`、`twist.twist.angular.z` 单位正确；
3. `base_link -> zed_left_camera_frame` 是实测外参。当前 launch 中为零位姿占位，实车不能默认相机与底盘原点重合；
4. `dwa.robot_radius` 覆盖乘员、脚踏板和轮椅外廓；
5. `dwa.obstacle_roi.min_z/max_z` 已按相机安装高度标定；
6. 蓝牙地址、特征句柄、转向符号、线速度比例、紧急停止均已单独验证；
7. 独立物理急停有效，并有安全员、空载和低速测试场地。

### 2. 启动感知与规划

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch wheel_perception run_launch.py
```

确认感知、道路边界和 `/dwa/planner_cmd` 正常后，再在**独立终端**启动原有底盘通信桥：

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run wheel_perception sub.py
```

不要在数据集回放期间启动 `sub.py`，避免回放控制指令发送给真实轮椅。

## 调试与注意事项

### 推荐调试顺序

1. 数据集回放确认 `debug/viz`、道路边界和 `/dwa/obstacle_cloud`；
2. 回放中记录 `/dwa/planner_cmd`，检查线速度、物理角速度和模式切换；
3. 不接底盘的实车传感器测试；
4. 悬空或低速空载测试，确认 `/cmd_vel` 协议符号和比例；
5. 受控场地、低速、带安全员测试；
6. 最后才考虑载人测试。

### 常见现象

| 现象 | 说明 / 排查方向 |
|---|---|
| 直路显示绿色轨迹 | 正常，表示 CRUISE + LQR；此时不需要激活 DWA。 |
| 树冠仍在 `/zed/point_cloud` | 正常；检查它是否出现在 `/dwa/obstacle_cloud`。若不在，就不会影响 DWA。 |
| 候选全红或频繁停车 | 检查 `robot_radius`、`road_margin`、道路宽度/边界和 ROI 内真实障碍。 |
| DWA 频繁绿蓝切换 | 调大 `activation.minimum_points` 或 `activation.clear_frames`，并检查 ROI 中噪声点。 |
| 轮椅左右摆动 | 先检查道路边界稳定性，再检查 `weight_delta_w`、`weight_angular_acceleration`、`weight_angular_jerk` 与轨迹保持阈值；不要先盲目增大 LQR 增益。 |
| 启动后短暂 `Safety stop: perception timeout` | 感知和 TensorRT 尚未完成初始化时的保护行为；若持续出现，说明 FusionNode 未正常输出感知。 |
| `RuntimeError: !rclpy.ok()` 出现在 lifecycle 命令 | 通常是 ROS 2 CLI daemon 状态异常；停止当前 launch，执行 `ros2 daemon stop` 后重新 source 环境并启动。 |

### 调参原则

- 想更早开始绕障：减小 `activation.minimum_points` 或增大激活区域，但更容易被噪声触发；
- 想更保守：增大 `robot_radius`、`road_margin`、`weight_obstacle`，但可行轨迹会变少；
- 想更平滑：优先增加平滑子项权重或轨迹保持阈值，避免直接大幅修改 LQR；
- 想更快：提高 `cruise_velocity` 或 `max_velocity` 前，必须同时验证制动距离、点云有效距离、`prediction_time` 和底盘响应；
- 修改 ROI 前先在 RViz 查看 `/dwa/obstacle_cloud`，不要仅凭完整点云判断。

## 当前实现边界

当前版本适合验证“道路约束下的静态局部障碍物避让”。以下能力尚未完整实现，不能误认为已具备：

- 对行人、自行车等动态障碍物的速度预测；
- 非圆形精确轮椅 footprint；
- 完整 Ackermann/轮椅动力学与底盘闭环模型；
- 全局地图、全局路径规划与重定位；
- 原地转向、倒车脱困和多阶段行为决策。

后续如需提升实车可靠性，优先建议完成真实相机外参、轮椅 footprint、底盘速度反馈和独立安全制动验证，再扩展动态障碍物与全局规划能力。
