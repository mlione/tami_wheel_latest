# DWA + LQR v0.1：轮椅自主导航改造与运行说明

本文记录 `wheel_latest` 从“道路边界 + 固定策略避障 + LQR 巡线”演进到
“ObstacleFusion + DWA 局部规划 + LQR 轨迹跟踪 + Safety Layer”的过程、实现边界和运行方法。

本文描述的是当前工作区代码，不是一个独立 DWA 示例；DWA 直接复用现有 ZED RGB-D、
TensorRT BiSeNet、点云融合、道路边界和数据集播放器。

## 1. 改造前：原始控制链路

原始工程的感知链路已经具备：

```text
ZED RGB-D
  ├─ RGB -> CUDA 预处理 -> TensorRT BiSeNet -> 语义 mask
  └─ 深度/点云 -> CUDA ROI、降采样、ObstacleFusion
                                  |
                           道路边界/最近障碍物
                                  |
                         perception/output
                                  |
                    固定状态机 + LQR 巡线
                                  |
                               cmd_vel
```

原 `ControllerNode` 以 `PerceptionOutput.min_distance`、障碍物横向位置和固定阈值切换
`CRUISE`、`AVOID_HARD_LEFT`、`AVOID_HARD_RIGHT`、`AVOID_SLOW`、`STOP` 等状态。
没有障碍物时，LQR 使用右侧道路边界距离和道路航向误差直接生成转向命令。

这种方式保留了有效的道路跟踪能力，但存在以下局限：

1. 避障方向由少量阈值决定，无法同时比较多条可能轨迹。
2. 无法预先验证“转过去后”是否会碰撞或驶出道路。
3. 速度、转向和道路边界约束不是在同一个评分/可行性框架中计算。
4. 状态切换和直接转向容易引发左右摆动，特别是在边界噪声或障碍物接近时。

## 2. 本版本总体架构

当前控制链路如下：

```text
ZED RGB-D / 历史 RGB-D 数据集
             |
TensorRT BiSeNet + CUDA 点云处理
             |
       ObstacleFusion
             |
道路边界、道路宽度、道路航向误差、障碍物点云、最近障碍物距离
             |
        Safety Layer（紧急停车优先）
             |
     DWA 局部规划（候选 v,w 和未来轨迹）
             |
       LQR 跟踪 DWA 的局部前视点
             |
   转向变化率限幅 + 底盘协议转换
             |
           cmd_vel -> 蓝牙底盘桥
```

核心原则：

- DWA 决定“该走哪一条局部安全轨迹”和“以什么物理速度 `v,w` 走”。
- LQR 不再直接跟踪整条道路边界，而是跟踪 DWA 最优轨迹上的前视点，修正横向和航向误差。
- Safety Layer 可以绕过 DWA/LQR 直接发布零速度。
- 原有 LQR 控制器和其底盘协议增益被保留，而不是删除或重写。

## 3. 改造过程与代码变更

### 3.1 新增 DWA 模块

新增目录：

```text
src/wheel_perception/dwa_controller/
├── include/wheel_perception/dwa_controller/DWAPlanner.hpp
└── src/DWAPlanner.cpp
```

`DWAPlanner` 采用纯 C++、无 ROS 依赖的设计，因此可以单独测试。其输入为：

- 机器人位姿：`x`、`y`、`yaw`；位姿无效（NaN/Inf）时拒绝规划。
- 当前线速度与角速度：优先使用 `/odom` 或 `/zed/odom`，没有可靠速度反馈时使用上一帧规划值。
- 道路模型：右边界距离、道路宽度、动态目标右侧距离、道路航向误差。
- 障碍物：`zed/point_cloud` 中的 `x,y` 点，按 `dwa.max_obstacle_points` 限制数量。
- 上一帧角速度：用于抑制角速度突变。

输出为 `Result`：最优轨迹、全部候选轨迹、最小间隙、评分、碰撞/道路可行性标志。

### 3.2 DWA 动态窗口与轨迹生成

在每一个控制周期，DWA 先根据加速度约束构造速度窗口：

```text
v ∈ [clamp(v_now - a_max*dt), clamp(v_now + a_max*dt)]
w ∈ [clamp(w_now - alpha_max*dt), clamp(w_now + alpha_max*dt)]
```

然后依据 `velocity_resolution` 和 `angular_resolution` 对窗口采样。对每一组 `(v,w)`，
使用单轨/差速近似模型积分 `prediction_time`：

```text
x(k+1)   = x(k) + v*cos(yaw(k))*dt
y(k+1)   = y(k) + v*sin(yaw(k))*dt
yaw(k+1) = yaw(k) + w*dt
```

轨迹坐标使用 `base_link` 局部坐标：X 向前、Y 向左、Z 向上。

### 3.3 评价函数：道路约束 DWA

本项目不是传统只考虑朝向和障碍物的 DWA。每条候选轨迹的评分为：

```text
score = weight_heading  * heading_cost
      - weight_obstacle * obstacle_cost
      - weight_velocity * velocity_cost
      - weight_road     * road_cost
      - weight_smooth   * smooth_cost
```

各项定义：

| 代价/收益 | 当前实现 | 作用 |
|---|---|---|
| `heading_cost` | `cos(endpoint_yaw - road_yaw_error)` | 使轨迹末端方向贴近道路走向 |
| `obstacle_cost` | `1 / minimum_clearance` | 越接近 ZED 点云障碍物，扣分越大 |
| `velocity_cost` | `max_velocity - v` | 在安全前提下优先更高速度 |
| `road_cost` | 轨迹到道路目标线的平均横向偏差 | 保持在道路目标区域附近 |
| `smooth_cost` | `abs(w - previous_w)` | 抑制左右摆动和突变转向 |

道路还承担硬约束，而不仅仅是软评分：

- 轨迹点进入右侧边界以内（考虑 `road_margin`）即判为越界。
- 有道路宽度时，轨迹点越过左侧边界（同样考虑 `road_margin`）即判为越界。
- 点云到轨迹中心线距离小于 `robot_radius` 时判为碰撞。
- 碰撞或越界的轨迹评分为负无穷，不能成为最优轨迹。

### 3.4 ControllerNode：DWA 输出接入 LQR

`src/wheel_perception/src/controller_node.cpp` 已由原先的固定避障状态机改为分层控制节点：

1. 订阅 `perception/output`，得到最近障碍物、道路边界、道路宽度和 `road_yaw_error`。
2. 订阅 `zed/point_cloud`，提取最多 `dwa.max_obstacle_points` 个平面障碍点。
3. 订阅 `/odom` 与 `/zed/odom`，获取位姿和速度反馈。
4. 先检查紧急距离；通过后运行 DWA。
5. 从最优轨迹中按 `lqr.lookahead_time` 选择局部前视点。
6. 以 `lateral_error=-reference.y`、`heading_error=-reference.yaw` 调用 LQR。
7. 将 DWA 的 `w` 作为 LQR 前馈项，LQR 只承担跟踪修正。
8. 对跟踪后的角速度做最大角速度和角加速度限幅，再发布 `cmd_vel`。

接口扩展在 `lqr_controller.hpp`：

```cpp
double computeTracking(double lateral_error,
                       double heading_error,
                       double reference_angular_velocity);
```

其中输出为“物理角速度”的跟踪结果；`toActuatorCommand()` 保留既有底盘协议方向与增益转换。
因此：

- `/dwa/planner_cmd.angular.z` 是 DWA 的物理角速度，单位 rad/s。
- `/cmd_vel.linear.x` 是 DWA 选择的线速度，单位 m/s。
- `/cmd_vel.angular.z` 是适配现有 `sub.py` 底盘协议后的转向量，不应直接当作 rad/s 解读。

### 3.5 Safety Layer

安全层优先级高于规划和轨迹跟踪：

1. `min_distance < safety.emergency_stop_distance`：立即停车。
2. DWA 没有任一“无碰撞且在道路内”的轨迹，且 `stop_on_no_path=true`：停车。
3. 超过 `safety.perception_timeout` 未收到感知消息：看门狗停车。
4. 只有以上检查通过后，才执行 DWA + LQR 并发布运动命令。

停车时同时发布：

- `cmd_vel = 0`
- `dwa/planner_cmd = 0`
- 空的 `dwa/local_trajectory`、`dwa/best_path`
- `DELETEALL` 候选轨迹 Marker

并复位 LQR 积分项和上一帧规划速度，防止恢复时沿用旧转向。

### 3.6 可视化与离线测试

新增发布话题：

| 话题 | 类型 | 内容 |
|---|---|---|
| `/dwa/planner_cmd` | `geometry_msgs/Twist` | DWA 原始 `v,w` |
| `/dwa/local_trajectory` | `nav_msgs/Path` | LQR 当前跟踪的局部最优轨迹 |
| `/dwa/best_path` | `nav_msgs/Path` | DWA 最优路径 |
| `/dwa/candidate_paths` | `visualization_msgs/MarkerArray` | 候选轨迹：绿=可行，红=碰撞/越界 |
| `/zed/point_cloud` | `sensor_msgs/PointCloud2` | ObstacleFusion 过滤后的障碍点云 |
| `/debug/viz` | `sensor_msgs/Image` | 语义分割、道路边界与调试叠加图 |

新增文件：

```text
src/wheel_perception/launch/dwa_dataset_test.launch.py
src/wheel_perception/rviz/dwa_navigation.rviz
src/wheel_perception/test/test_dwa_planner.cpp
```

离线 launch 只启动：静态 TF、FusionNode、ControllerNode、数据集播放器；不会启动蓝牙底盘桥。
数据集播放器在感知节点完成生命周期激活后延迟 8 秒启动，避免首批 RGB-D 帧丢失。

### 3.7 数据集回放与全绿画面问题修复

在引入 DWA 后，数据集回放曾出现 `/debug/viz` 全绿、边界消失或画面不刷新的问题。
根因不是 DWA 改写了语义分割，而是以下两个运行/构建问题叠加：

1. GTX 1660 SUPER 是 CUDA `SM 7.5`，但旧构建缓存曾把 CUDA 内核只编译为 `SM 8.6`。
   TensorRT 推理成功，但 GPU ArgMax 后处理无法执行；旧代码没有检查 CUDA 错误，未初始化的全零 mask 被解释为类别 0（道路），于是整幅图变绿。
2. 数据集通常只发布 10 FPS，而 FusionNode 定时器约 60 Hz，会反复处理同一张缓存图像；DWA 加入后，重复规划和 Marker 发布进一步拥塞 DDS/rqt 队列。

当前修复：

- 默认桌面构建生成 `SM 75;86` CUDA 代码；`GTX 1660 SUPER` 使用 `sm_75`。
- TensorRT 和 CUDA ArgMax 失败时返回 `false`，FusionNode 丢弃该帧，不再发布错误的全绿 mask。
- 数据集模式仅对每一张新 RGB 消息运行一次推理。
- RGB 与深度改为 `Reliable + KeepLast(2)`，FusionNode 使用匹配 QoS。
- 每 60 帧输出一次道路 mask 占比诊断；道路比例超过 98% 会发出警告。
- CMake 改为自动寻找 x86_64 或 Jetson aarch64 的 TensorRT include/library，清 CMake 缓存后仍可构建。

已使用历史数据帧验证：GPU ArgMax 与 CPU ArgMax 一致，类别 0（道路）占比约 7.86%，不再是 100% 道路。

## 4. 当前工程结构

```text
wheel_latest/
├── dwa+lqr_v0.1.md                         # 本文档
├── zed_dataset_player.py                   # RGB-D/IMU/odom 数据集回放
└── src/
    ├── wheel_msgs/
    │   └── msg/
    │       ├── PerceptionOutput.msg         # 感知到控制的道路/障碍物信息
    │       └── ObstacleInfo.msg
    └── wheel_perception/
        ├── config/params.yaml               # 保持原参数体系，新增 dwa/safety
        ├── dwa_controller/
        │   ├── include/.../DWAPlanner.hpp
        │   └── src/DWAPlanner.cpp
        ├── include/wheel_perception/core/
        │   ├── lqr_controller.hpp
        │   ├── ai_engine.hpp
        │   └── obstacle_fusion.hpp
        ├── src/
        │   ├── controller_node.cpp           # DWA + LQR + Safety
        │   ├── fusion_node.cpp               # RGB-D 感知/道路边界/输出
        │   └── core/
        │       ├── ai_engine.cpp
        │       └── obstacle_fusion.cu
        ├── launch/
        │   ├── run_launch.py                 # 实时感知/控制组件
        │   └── dwa_dataset_test.launch.py    # 离线回放验证
        ├── rviz/dwa_navigation.rviz
        └── test/test_dwa_planner.cpp
```

## 5. 参数说明

参数仍全部位于 `src/wheel_perception/config/params.yaml` 的既有 ROS YAML 结构中，
没有新增独立参数系统。

### 5.1 关键 DWA 参数

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `dwa.max_velocity` | 1.0 | 最大线速度，m/s |
| `dwa.min_velocity` | 0.0 | 最小线速度；默认不允许倒车 |
| `dwa.max_angular_velocity` | 1.0 | 最大物理角速度，rad/s |
| `dwa.max_acceleration` | 0.5 | 最大线加速度，m/s² |
| `dwa.max_angular_acceleration` | 1.5 | 最大角加速度，rad/s² |
| `dwa.prediction_time` | 2.5 | 轨迹预测时域，s |
| `dwa.simulation_time_step` | 0.1 | 积分和控制采样周期，s |
| `dwa.velocity_resolution` | 0.1 | 速度采样分辨率，m/s |
| `dwa.angular_resolution` | 0.1 | 角速度采样分辨率，rad/s |
| `dwa.robot_radius` | 0.45 | 轮椅含乘员外廓的碰撞半径，m |
| `dwa.road_margin` | 0.15 | 道路边界内缩安全裕量，m |
| `dwa.max_obstacle_points` | 2500 | DWA 使用的点云上限 |
| `dwa.visualization.max_candidates` | 80 | RViz 最大显示候选数 |

调参建议：先降低 `max_velocity`（例如 0.2~0.4 m/s）再进行实车空载测试；
若左右摆动明显，可适度提高 `weight_smooth` 或降低 `max_angular_acceleration`；
若贴近道路边界，可提高 `weight_road` 或 `road_margin`。

### 5.2 安全参数

| 参数 | 当前值 | 说明 |
|---|---:|---|
| `safety.emergency_stop_distance` | 0.8 | 紧急停车距离，m |
| `safety.perception_timeout` | 0.5 | 感知超时停车阈值，s |
| `safety.stop_on_no_path` | true | 无安全 DWA 路径时停车 |

### 5.3 LQR 参数

原 LQR 参数保持在 `lqr.*`：`gain`、`q_pos`、`q_ang`、`q_integral`、
`integral_limit`、`k_w`、`model_v`、`aim_dist`。

新增 `lqr.lookahead_time`，它决定 LQR 在 DWA 轨迹上跟踪多远的前视点。前视过短会更灵敏，
过长会更平滑但转弯响应更慢；当前值为 0.6 秒。

## 6. 编译

### 6.1 当前 x86_64 + GTX 1660 SUPER 工作站

本机的精简版 colcon 不支持 `--packages-select`，使用 `--paths`。必须为 GTX 1660 SUPER
编译 `SM 75` CUDA 内核；引号不能省略，否则 shell 会将分号当作命令分隔符。

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash

colcon build --paths src/wheel_msgs src/wheel_perception \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args '-DWHEEL_CUDA_ARCHITECTURES=75;86'

source install/setup.sh
```

`wheel_msgs` 出现 `WHEEL_CUDA_ARCHITECTURES was not used` 是正常警告：它没有 CUDA 代码，
参数会在 `wheel_perception` 中生效。

验证构建产物：

```bash
ctest --test-dir build/wheel_perception --output-on-failure
cuobjdump -lelf build/wheel_perception/libwheel_core.so | head
```

输出中应至少出现 `sm_75`。DWA 单元测试覆盖：空道路前进、道路边界约束、全路障拒绝、异常里程计拒绝。

### 6.2 Jetson / 其他 NVIDIA 平台

在目标设备上必须按本机 GPU 架构重新编译，不应把 x86_64 生成的二进制直接复制过去。
例如已有的 Jetson 配置可使用：

```bash
colcon build --paths src/wheel_msgs src/wheel_perception \
  --symlink-install \
  --cmake-clean-cache \
  --cmake-args '-DWHEEL_CUDA_ARCHITECTURES=72;87'
```

实际架构需以目标 Jetson 的 GPU 为准。TensorRT engine 也应在与目标 TensorRT/CUDA 兼容的设备上生成。

## 7. 数据集离线验证流程

### 7.1 数据集目录格式

播放器至少需要：

```text
<dataset>/
├── left/000000.png
├── left/000001.png
├── depth/000000.npy              # 旧格式，或以下任一种
├── depth_npy/000000.npy
├── depth_raw/000000.bin
├── depth_png/000000.png
├── odom/000000.json               # 可选
├── imu/000000.json                # 可选
├── metadata.json                  # 可选
└── camera_info.json               # 可选
```

播放器优先级为：`depth_raw`、`depth_npy`、旧 `depth`、`depth_png`。RGB 使用
`bgra8` 发布到 `/rgb_image`，深度以米为单位、`32FC1` 发布到 `/depth_image`。

### 7.2 启动离线回放

确认 `params.yaml` 中：

```yaml
zed:
  use_dataset_mode: true
```

启动：

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.sh

ros2 launch wheel_perception dwa_dataset_test.launch.py \
  dataset_path:=/home/x/tami/shengwudao2 \
  fps:=10.0 \
  loop:=true
```

启动后等待约 8 秒，再观察数据流。10 FPS 数据集下 `/debug/viz` 接近 10 Hz 是正确现象；
它表示每一帧只处理一次，而不是旧逻辑中将同一帧重复处理到约 60 Hz。

终端检查：

```bash
source /opt/ros/humble/setup.bash
source ~/tami/wheel_latest/install/setup.sh

ros2 topic hz /debug/viz
ros2 topic echo /perception/output --once
ros2 topic echo /dwa/planner_cmd
ros2 topic hz /dwa/candidate_paths
```

若 `ros2 topic echo /perception/output` 报 `wheel_msgs/msg/PerceptionOutput is invalid`，
说明当前终端没有 source 本工作区，重新执行：

```bash
source ~/tami/wheel_latest/install/setup.sh
```

## 8. rqt 与 RViz 可视化

### 8.1 rqt：二维分割与道路边界

在已 source 环境的新终端中运行：

```bash
rqt
```

选择 `Plugins -> Visualization -> Image View`，话题选择 `/debug/viz`。
该画面用于检查 RGB、语义道路区域、边界拟合和调试绘制。

### 8.2 RViz：点云与 DWA 轨迹

数据集回放启动后，在新终端运行：

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.sh

rviz2 -d "$(ros2 pkg prefix wheel_perception)/share/wheel_perception/rviz/dwa_navigation.rviz"
```

现成配置的固定坐标系是 `base_link`，显示：

- `ObstacleFusion Cloud`：`/zed/point_cloud`。
- `DWA Best Path`：`/dwa/best_path` 的绿色最优轨迹。
- `DWA Candidates`：`/dwa/candidate_paths`；绿色为可行，红色为碰撞或越界。

如需在 RViz 中额外显示语义图，点击 `Add -> By topic -> /debug/viz -> Image`。

若 RViz 没有显示最优路径，依次检查：

```bash
ros2 topic list | grep -E 'dwa|point_cloud|debug/viz'
ros2 topic echo /dwa/best_path --once
ros2 topic echo /dwa/candidate_paths --once
```

只有候选路径而没有最优路径通常表示所有候选都碰撞或越界，Safety Layer 会停车，这是预期的安全行为。

## 9. 实车部署流程

轮椅是载人设备，实车必须按“离线 -> 空载低速 -> 有安全员低速”的顺序推进。

1. 在目标硬件上构建与 GPU 匹配的 CUDA 架构和 TensorRT engine。
2. 将 `zed.use_dataset_mode` 改为 `false`，检查 `ai.engine_path` 指向目标机存在的 engine。
3. 先不启动蓝牙桥，仅运行感知和控制，确认 `/debug/viz`、点云、DWA 路径及 `cmd_vel`。
4. 把 `dwa.max_velocity` 临时设为 0.2~0.4 m/s，适当提高 `road_margin` 与
   `emergency_stop_distance`。
5. 确认急停、人工接管和安全员已到位，再启动硬件桥：

```bash
source /opt/ros/humble/setup.bash
source ~/tami/wheel_latest/install/setup.sh
ros2 run wheel_perception sub.py
```

6. 蓝牙桥会订阅 `cmd_vel`；它自身也有“超过 1 秒未收到命令则置零”的保护。
7. 先做直道、单障碍、窄道、道路边界缺失和感知中断测试。任何一次出现误判、转向抖动或
   边界错误，都应先停止实车并回放录制数据集调参。

注意：`sub.py` 的底盘编码与实际车体、蓝牙地址和协议强耦合。`lqr.gain`、底盘转向方向和
最大速度需要以空载测试结果为准，不能仅依据 RViz 轨迹直接提高速度。

## 10. 相对原始版本的提升与边界

### 避障能力

- 原始版本：按照最近障碍物所在区间选择预设左/右/慢行状态。
- 当前版本：同时模拟多条未来轨迹，以轮椅半径检查点云间隙，拒绝碰撞和离开道路的轨迹，
  再从可行轨迹中选择得分最高者。

### 平滑性

- DWA 动态窗口限制速度与角速度随时间的变化。
- `smooth_cost` 抑制相邻帧的角速度跳变。
- LQR 只对 DWA 的局部参考做闭环修正，并保留 DWA 曲率前馈。
- 控制输出还有角速度幅值和角加速度限幅。

### 安全性

- 紧急距离、无可行路径、感知超时都优先触发停车。
- 推理或 CUDA 后处理失败时不会消费旧数据或发布“全绿”伪道路结果。
- 实车仍需独立硬件急停和人工监管；软件 Safety Layer 不能替代物理安全措施。

## 11. 修改文件清单

| 文件 | 修改目的 |
|---|---|
| `src/wheel_perception/dwa_controller/include/.../DWAPlanner.hpp` | DWA 数据结构、参数、规划接口 |
| `src/wheel_perception/dwa_controller/src/DWAPlanner.cpp` | 动态窗口采样、轨迹预测、道路/障碍物/平滑性评价 |
| `src/wheel_perception/src/controller_node.cpp` | DWA + LQR 分层控制、安全层、话题与 RViz 发布 |
| `src/wheel_perception/include/.../lqr_controller.hpp` | 增加局部轨迹跟踪和前馈角速度接口 |
| `src/wheel_perception/config/params.yaml` | 新增 `dwa.*`、`safety.*`、`lqr.lookahead_time` 参数 |
| `src/wheel_perception/CMakeLists.txt` | 编译/安装 DWA、测试、RViz；便携 TensorRT 查找；CUDA 架构配置 |
| `src/wheel_perception/package.xml` | 添加 `visualization_msgs` 依赖 |
| `src/wheel_perception/launch/run_launch.py` | 多线程组件容器，降低 DWA 对感知回调的影响 |
| `src/wheel_perception/launch/dwa_dataset_test.launch.py` | 离线 RGB-D -> 感知 -> DWA -> LQR 启动入口 |
| `src/wheel_perception/rviz/dwa_navigation.rviz` | DWA 路径、候选轨迹、点云显示配置 |
| `src/wheel_perception/test/test_dwa_planner.cpp` | DWA 核心可行性单元测试 |
| `src/wheel_perception/src/core/ai_engine.cpp` | 推理失败显式返回，禁止使用无效输出 |
| `src/wheel_perception/src/core/obstacle_fusion.cu` | CUDA ArgMax/后处理错误检查 |
| `src/wheel_perception/src/fusion_node.cpp` | 数据集逐帧处理、失败丢帧、mask 诊断、匹配 QoS |
| `zed_dataset_player.py` | RGB-D 可靠小队列 QoS，降低大图像回放丢帧 |

