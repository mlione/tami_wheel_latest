# DWA + LQR v0.4 开发、测试与问题记录

> 文档日期：2026-09-19
>
> 对应工作区：`~/tami/tami_wheel_latest`
>
> 对比基线：`dwa+lqr_v0.3.2.md` 及其后的实车/数据集调试版本

本文记录当前仓库中**已经落地的代码**、测试方法、已观察到的问题和下一步工作。文中将“代码已实现”“离线测试通过”和“实车已验证”分开描述，避免把具备接口误写成已具备实车可靠性。

## 1. v0.4 的目标

v0.4 没有推翻原项目的 ZED、TensorRT BiSeNet、CUDA 点云融合、道路边界和 LQR 主链路，而是继续完善以下闭环：

```text
ZED RGB-D / 数据集
        |
语义分割 + 深度点云 + ObstacleFusion
        |
道路边界、道路方向、障碍点云
        |
安全层 ── 急停 / 感知超时 / 无安全轨迹
        |
无障碍：CRUISE 道路参考轨迹
有障碍：道路约束 DWA 局部规划
        |
LQR 跟踪局部轨迹前视点
        |
/cmd_vel（m/s、rad/s）
        |
ble_hardware_bridge
        |
轮椅底盘
```

本版的重点不只是“增加 DWA”，还包括底盘可执行域约束、ZED 运动估计接口、相机外参统一、数据集/实车状态隔离以及更完整的诊断流程。

## 2. 相比 v0.3.2 已完成的改进

### 2.1 DWA 候选适配真实 BLE 底盘可执行域

原先 DWA 可以输出数学上连续、但蓝牙摇杆协议无法执行的低速命令。当前规划器增加了硬件约束：

- `(v,w)=(0,0)` 始终允许，作为明确停车状态；
- 禁止 `v=0,w!=0`，当前版本不规划原地转向；
- 非零前进速度必须跨过 BLE 前进死区；
- 根据 `0001 / 0003 / 0005` 挡位限制最大角速度；
- 从静止起步时显式加入最小可执行速度候选；
- 最终 LQR 角速度也受到同一硬件能力和最小转弯半径限制。

对应参数位于 `params.yaml` 的 `dwa.hardware_constraints`，底盘转换参数位于 `ble_hardware_bridge.yaml`。两处阈值必须保持一致。

### 2.2 允许低速滚动转弯，但仍禁止原地转向

当前规则为：

```text
v < minimum_turning_velocity 且 |w| > 0  -> 拒绝
|w| > 0 且 |v/w| < minimum_turning_radius -> 拒绝
```

当前配置：

```yaml
minimum_turning_velocity: 0.001
minimum_turning_radius: 0.1
```

因此轮椅可以用正向低速弧线绕障，但不能以 `v=0` 原地旋转。`0.1 m` 是当前调试值，不等于已经实测确认的安全物理半径。

### 2.3 动态窗口使用真实控制周期

DWA 不再假定固定回调周期，而是使用实际 `control_dt` 构造速度窗口，并把异常调度间隔限制在 `0.01~0.25 s`：

```text
v in [v_now - a_max*dt, v_now + a_max*dt]
w in [w_now - alpha_max*dt, w_now + alpha_max*dt]
```

这避免一次长时间卡顿使下一帧速度窗口突然扩大。

### 2.4 轨迹评分和平滑保持

当前有效候选最大化：

```text
score =
    weight_heading  * cos(yaw_end - road_yaw_error)
  - weight_obstacle * (1 / minimum_clearance)
  - weight_velocity * (max_velocity - v)
  - weight_road     * mean_road_offset
  - weight_smooth   * smooth_cost
```

其中：

- `minimum_clearance = min(distance(path_point, obstacle) - robot_radius)`；
- 净空小于等于零的轨迹直接碰撞无效；
- 越过道路边界的轨迹直接无效；
- `smooth_cost` 包含命令变化、线/角加速度和可选 jerk；
- 上一命令对应的轨迹仍安全且新轨迹优势不足时，保留上一轨迹，减少候选来回切换。

当前 jerk 权重为零，因此 `max_jerk` 和 `max_angular_jerk` 只保留参数接口，不参与现有评分。

> 重要现状：此前讨论过的“净空逐步改善奖励”和“安全条件下停车惩罚”**不在当前 `DWAPlanner.cpp` 中**。当前只有净空倒数惩罚和轨迹保持中的净空切换条件，不能把讨论方案当作已实现功能。

### 2.5 障碍触发 DWA，无障碍时稳定巡线

只有 DWA 激活矩形内点数达到阈值才进入避障：

```yaml
activation:
  max_x: 5.0
  min_y: -1.0
  max_y: 1.0
  minimum_points: 20
  clear_frames: 5
```

- 未激活：以 `cruise_velocity` 生成确定性的道路参考轨迹；
- 已激活：采样并评价 DWA 候选；
- 连续 5 帧低于阈值后才退出 DWA，避免模式抖动。

规划点云还会经过独立 ROI：`x=0.3~5.0 m`、`y=-1.5~1.5 m`、`z=-0.2~1.0 m`。完整点云仍可显示，但只有 `/dwa/obstacle_cloud` 中的点会影响 DWA。

> 当前只具备体素降采样和 ROI 过滤。曾讨论的“以最近点为中心做空间聚类”和“连续 2/3 帧确认后才进入普通急停”**没有出现在当前控制代码中**。单个被保留的近噪声点仍可能主导 `minimum_clearance` 或融合层 `min_distance`。

### 2.6 BLE 桥替代旧 `sub.py`

旧的 Python `sub.py` 及其 `lqr.gain` 协议缩放已经从控制链移除。现在：

- 控制器发布标准 ROS `/cmd_vel`；
- `linear.x` 单位为 `m/s`；
- `angular.z` 单位为 `rad/s`；
- 独立的 `ble_hardware_bridge` 负责挡位选择、连续速度反算、摇杆死区、GATT 写入、断线重连和命令超时停车；
- 感知/规划与 BLE 桥仍分开启动，数据集测试绝不启动 BLE。

这样做使规划器不再依赖某个脚本中的协议常数，也便于单独测试底盘映射。

### 2.7 ZED `/odom` 与相机杆臂修正

ZED 驱动已增加 CAMERA 增量位姿、twist、协方差和跟踪有效性的读取。FusionNode 将其转换到 `base_link` 后发布 `/odom`：

```text
相机位姿：T_B0Bt = T_BC * T_C0Ct * inverse(T_BC)
基座线速度：v_B = R_BC*v_C - omega_B × r_BC
```

当前按相机位于轮椅中心“前方 0.2 m、右侧 0.2 m”配置。在 ROS 坐标中 X 向前、Y 向左，所以：

```yaml
translation_x: 0.2
translation_y: -0.2
translation_z: 0.0
```

同一外参还用于：

- `/odom` pose、twist 和 covariance；
- `/perception/output` 的最小障碍点、道路边界距离与方向；
- DWA 输入点云从相机系到 `base_link` 的转换；
- `/dwa/obstacle_cloud`；
- `base_link -> zed_left_camera_frame` 静态 TF。

### 2.8 数据集模式与实车速度状态隔离

- 数据集模式使用 `last_output_velocity_` 构造下一帧动态窗口；
- 实车模式才尝试使用新鲜的 `/odom.twist`；
- 历史数据集中的运动状态不会污染当前回放控制；
- 反馈超时时当前配置回退到上一控制输出，而不是停车。

这解决了“离线回放没有真实轮速却错误依赖 `/odom`”的问题，但不代表实车速度反馈已经验证合格，详见第 8 节。

### 2.9 RViz 状态更直观

- 亮绿色：未激活 DWA 的 CRUISE 最优轨迹；
- 蓝色曲线：DWA 选中的避障轨迹；
- 蓝色圆点：DWA 正常运行，但主动选中停车 `(0,0)`；
- 半透明绿色候选：碰撞、道路和动态约束均通过；
- 半透明红色候选：至少一项硬约束未通过；
- 黄色点：道路右边界调试点云。

## 3. 当前关键参数快照

下表记录当前 YAML，而不是推荐的最终实车值。

| 参数 | 当前值 | 作用与风险 |
|---|---:|---|
| `max_velocity` | `1.0 m/s` | DWA 最大速度；实车初测应另外限制到底盘安全低速。 |
| `max_angular_velocity` | `1.0 rad/s` | 规划器总角速度上限。 |
| `max_acceleration` | `0.5 m/s²` | 动态窗口线速度扩张速度。 |
| `max_angular_acceleration` | `5.0 rad/s²` | 当前较大，允许角速度快速进入候选窗口。 |
| `prediction_time` | `3.0 s` | 轨迹预测时域；必须写成浮点数。 |
| `simulation_time_step` | `0.1 s` | 轨迹积分步长。 |
| `weight_heading` | `1.0` | 终点航向贴近道路方向。 |
| `weight_obstacle` | `3.5` | 惩罚小净空。过大可能偏好停车。 |
| `weight_velocity` | `4.0` | 鼓励前进，过大可能降低保守性。 |
| `weight_road` | `1.0` | 惩罚偏离道路目标线。 |
| `weight_smooth` | `0.02` | 平滑总代价缩放。 |
| `robot_radius` | `0.45 m` | 轮椅圆形碰撞半径；不是急停距离的附加项。 |
| `emergency_stop_distance` | `0.5 m` | Fusion 输出的 `min_distance` 小于该值直接停车。 |
| `cruise_velocity` | `0.6 m/s` | 无障碍巡线参考速度。 |
| `activation.max_x` | `5.0 m` | DWA 前向激活距离。 |
| `activation.minimum_points` | `20` | 激活区内总点数门槛，不等价于一个真实点簇。 |
| `minimum_turning_radius` | `0.1 m` | 曲率硬约束，当前值很激进。 |
| `lookahead_time` | `0.5 s` | LQR 在 DWA 轨迹上的前视时间。 |

安全层的 `0.5 m` 和 DWA 的 `robot_radius=0.45 m` 是两套判据：

- 急停直接比较 Fusion 生成的 `min_distance`；代码没有再加 `robot_radius`；
- DWA 计算候选净空时会从几何点距中减去 `robot_radius`。

因此不能把急停距离理解为自动变成 `0.5+0.45=0.95 m`。

## 4. 构建与环境准备

### 4.1 当前项目目录

```bash
cd ~/tami/tami_wheel_latest
source /opt/ros/humble/setup.bash
```

### 4.2 TensorRT engine 路径必须先检查

当前 `params.yaml` 仍配置为：

```yaml
ai:
  engine_path: "/home/x/tami/wheel_latest/src/wheel_perception/config/combined5.engine"
```

这是旧项目目录。当前仓库的 `src/wheel_perception/config/` 中没有该 engine，但旧路径上的文件仍存在，所以本机可能碰巧能运行，复制到开发板后会失败。部署前必须把该字段改成目标机上真实存在的绝对路径，或把匹配 GPU/CUDA/TensorRT 版本的 engine 放到约定位置。

检查：

```bash
test -f /home/x/tami/wheel_latest/src/wheel_perception/config/combined5.engine \
  && echo "engine exists" \
  || echo "engine missing"
```

### 4.3 编译

```bash
colcon build --symlink-install
source install/setup.sh
```

当前环境没有 `install/setup.bash`，使用 `install/setup.sh`。若 `cv_bridge` 的 symlink 构建报“existing path cannot be removed”，通常是旧 build 状态和当前源码目录不一致；确认目录无重要产物后再清理对应的 `build/cv_bridge` 与 `install/cv_bridge`，不要直接删除数据集或源码。

修改 CUDA 架构时可追加：

```bash
colcon build --symlink-install \
  --cmake-args -DWHEEL_CUDA_ARCHITECTURES="75;86"
```

架构号必须与目标 GPU 相符。

### 4.4 DWA 回归测试

```bash
cmake --build build/wheel_perception --target test_dwa_planner -j2
./build/wheel_perception/test_dwa_planner
```

当前源码包含 13 个 DWA 测试，覆盖清晰道路、空点云巡线、道路越界、横向障碍墙、无效里程计、禁止原地转向、静止起步、低速滚动转弯、BLE 挡位可执行域、停车候选、最小转弯半径、轨迹保持和加速度约束。

## 5. 数据集回放测试指令

### 5.1 参数准备

确认：

```yaml
zed:
  use_dataset_mode: true
```

### 5.2 使用 text6_stereo 回放

```bash
cd ~/tami/tami_wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.sh

ros2 launch wheel_perception dwa_dataset_test.launch.py \
  dataset_path:=/home/x/tami/datasets6/text6_stereo \
  fps:=20.0 \
  loop:=false
```

观察轨迹时推荐 `10~20 FPS`；分析实时吞吐时再提高到 `30 FPS`。过低帧率会改变控制回调间隔和动态窗口，不能代表实车控制效果。`loop:=false` 适合采集一次完整数据；循环回放会在数据集首尾产生不连续跳变，影响速度曲线和统计。

launch 已把播放器延迟 8 秒启动，以等待 TensorRT 和 lifecycle 激活。默认数据集路径仍为 `~/tami/shengwudao2`，测试其他数据集时应显式给出 `dataset_path`。

### 5.3 基础话题检查

```bash
ros2 node list
ros2 topic list | sort
ros2 topic hz /perception/output
ros2 topic hz /cmd_vel
ros2 topic echo /perception/output --once
ros2 topic echo /dwa/planner_cmd
ros2 topic echo /cmd_vel
```

若提示 `wheel_msgs/msg/PerceptionOutput is invalid`，通常是终端没有 source 当前工作区：

```bash
source /opt/ros/humble/setup.bash
source ~/tami/tami_wheel_latest/install/setup.sh
ros2 interface show wheel_msgs/msg/PerceptionOutput
```

### 5.4 RViz

```bash
rviz2 -d install/wheel_perception/share/wheel_perception/rviz/dwa_navigation.rviz
```

Fixed Frame 使用 `base_link`。重点观察：

1. `/dwa/obstacle_cloud` 是否只含真正影响规划的点；
2. `/perception/debug/right_road_edge` 是否连续且方向正确；
3. `/dwa/candidate_paths` 是否大量变红；
4. `/dwa/best_path_marker` 是绿色、蓝线、蓝点，还是完全消失；
5. `/debug/viz` 是否持续刷新，而不是只有 `/rgb_image` 正常。

### 5.5 记录一次完整回放

先启动回放，然后另开终端：

```bash
mkdir -p bags
ros2 bag record -o bags/dwa_dataset_once \
  /perception/output \
  /dwa/planner_cmd \
  /cmd_vel \
  /dwa/obstacle_cloud \
  /dwa/local_trajectory \
  /dwa/best_path
```

数据集播放结束后按 `Ctrl+C` 停止 rosbag。检查：

```bash
ros2 bag info bags/dwa_dataset_once
ros2 bag play bags/dwa_dataset_once
```

## 6. 只接 ZED、不连接底盘的测试

将 `zed.use_dataset_mode` 改为 `false`，然后：

```bash
ros2 launch wheel_perception run_launch.py
```

不要启动 BLE 桥。此时可以验证实时语义分割、道路边界、点云、DWA 候选、最优轨迹和 ZED `/odom`，但无法验证轮椅是否真的能跟随命令，也无法用 `/cmd_vel` 代替真实速度真值。

常用检查：

```bash
ros2 topic hz /odom
ros2 topic echo /odom
ros2 topic echo /odom --field twist.twist
ros2 topic echo /cmd_vel
ros2 run tf2_ros tf2_echo base_link zed_left_camera_frame
```

## 7. 实车测试指令

### 7.1 启动顺序

终端 1：感知、规划、控制。

```bash
cd ~/tami/tami_wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.sh
ros2 launch wheel_perception run_launch.py
```

终端 2：RViz 和话题确认。确认系统稳定前不要连接 BLE。

```bash
source /opt/ros/humble/setup.bash
source ~/tami/tami_wheel_latest/install/setup.sh
rviz2 -d ~/tami/tami_wheel_latest/install/wheel_perception/share/wheel_perception/rviz/dwa_navigation.rviz
```

终端 3：确认急停、方向、速度限幅和场地安全后，单独启动 BLE：

```bash
source /opt/ros/humble/setup.bash
source ~/tami/tami_wheel_latest/install/setup.sh
ros2 launch ble_hardware_bridge ble_hardware_bridge.launch.py
```

严禁在数据集回放时启动 BLE 桥。

### 7.2 BLE 诊断

```bash
bluetoothctl show
bluetoothctl devices
systemctl status bluetooth --no-pager
ros2 node list | grep ble
ros2 topic hz /cmd_vel
```

检查配置：

```bash
sed -n '1,120p' ble_hardware_bridge/ble_hardware_bridge/config/ble_hardware_bridge.yaml
```

常见蓝牙失败原因包括地址或 UUID 错误、设备被手机占用、服务未解析、适配器未上电、权限不足、轮椅未进入可连接状态和距离/干扰问题。桥接节点带 `0.3 s` 命令超时停车，但物理急停必须独立存在。

### 7.3 手动移动轮椅并记录 ZED 速度反馈

BLE 桥不启动，用原有摇杆缓慢做“静止—直行—左转—右转—静止”，同时记录：

```bash
mkdir -p bags
ros2 bag record -o bags/zed_velocity_test \
  /odom /cmd_vel /dwa/planner_cmd /perception/output /tf /tf_static /rosout
```

结束后：

```bash
ros2 bag info bags/zed_velocity_test
```

`/odom.twist` 是 ZED 视觉惯性估计，不是电机编码器速度；`/cmd_vel` 是软件期望值，也不是手动摇杆的真实输入。

## 8. 本轮测试发现的问题

### 8.1 ZED twist 绝对量级目前不可信

已记录的 `bags/zed_velocity_test` 约 95.66 秒，包含 1553 条 `/odom`，平均约 `16.23 Hz`。统计结果：

- `linear.x` 范围约 `-0.476~4.611 m/s`；
- `angular.z` 范围约 `-3.975~3.683 rad/s`；
- 自动识别静止段的速度均值接近零，静止漂移较小；
- 由 pose 差分得到的线速度约为 twist 的 `0.271` 倍，相关系数约 `0.985`；
- 由 pose 差分得到的角速度约为 twist 的 `0.274` 倍，相关系数约 `0.987`。

`0.271` 与 `16.23/60≈0.2705` 非常接近，强烈提示 ZED 配置的 60 FPS 与实际约 16 Hz 的抓取/处理周期之间存在时间尺度问题。趋势和符号可能正确，但 `4.61 m/s` 对轮椅明显不合理，因此当前 `/odom.twist` 不能直接视为已标定的真实 SI 速度。

建议后续：

1. 用 ZED 图像时间戳和 WORLD 绝对 pose 差分计算基座速度；
2. 把 SDK 原始 twist 保留为诊断量；
3. 将 grab/里程计采集与较慢的语义分割解耦，避免漏采运动增量；
4. 用已知 2 m 距离、秒表、轮编码器或动捕做绝对尺度验证；
5. 可临时把 ZED 配置成 15 FPS 做 A/B 诊断，但不能把它当作根本修复。

在完成验证前保持：

```yaml
velocity_feedback:
  stop_on_timeout: false
```

并把实车速度反馈视为实验功能，而不是安全闭环依据。

### 8.2 控制器偶发认为速度反馈过期

bag 中 `/odom` 最大相邻间隔低于 `0.1 s`，小于 `velocity_feedback.timeout=0.3 s`，但日志仍多次出现：

```text
Velocity feedback unavailable or stale; using last controller output
```

较可能的原因是 ControllerNode 的感知回调和 `/odom` 回调处于默认互斥 callback group；DWA 计算期间，排队的里程计回调无法及时更新反馈时间。即使使用多线程 component container，也不代表同一互斥组内可并行。

后续应将 `/odom` 放入独立或 reentrant callback group，并记录消息时间戳、接收时间和控制取样时间验证延迟。

### 8.3 近障起步容易选择停车

当前评分中停车轨迹具有以下天然优势：

- 不继续靠近障碍，净空代价可能优于短小的前进轨迹；
- 不改变速度和角速度，平滑代价最低；
- 低速时预测路径短，真正绕行轨迹在有限时域内未必已经表现出净空改善；
- heading 和 road cost 会惩罚早期明显偏转的绕行轨迹；
- 最小转弯半径、BLE 挡位角速度限制和道路边界进一步缩小可行集。

这解释了“蓝线很短或显示蓝色停车点、日志速度只有 0.0x”的现象。降低 `weight_obstacle` 虽可能减少停车倾向，也会让轨迹更愿意贴近障碍，不能单独作为解决方案。

后续推荐增加两个**尚未实现**的评分项：

1. 只奖励沿轨迹逐步增加、且终点明显优于起点的净空改善；
2. 仅当存在满足最低净空和制动条件的运动轨迹时，对停车施加有限惩罚。

不能为了“避免停车”无条件奖励运动。

### 8.4 噪声点仍会影响激活、规划和急停

`activation.minimum_points=20` 统计的是区域内总点数，而不是同一物体的点簇。若较近噪声点和较远真实障碍点合计超过 20：

- DWA 会被激活；
- 较近噪声点可能成为最小净空点；
- Fusion 的 `min_distance` 若被噪声影响，还可能触发安全层急停。

后续应实现：体素邻域内至少 `3~5` 点的空间聚类，以及普通急停的 `2/3` 帧时间一致性；超近距离、传感器失联和硬件急停仍应走立即停车通道。

### 8.5 道路方向不稳会让 LQR 饱和

历史日志中出现过：

```text
road_yaw_error ≈ -2.9 rad
CRUISE+LQR: w_ref=0.00, w_track≈-1.00
```

这说明绿色路径大范围摆动并不一定来自 DWA：道路直线方向反转或拟合抖动会使 LQR 修正饱和。当前道路拟合增加了朝 `+X` 的方向统一、角度范围和可信结果保持，但仍需通过 `/perception/output` 和右边界点云验证输入是否稳定。

### 8.6 当前路径和构建残留

- `ai.engine_path` 仍依赖旧目录；
- `build/` 和 `install/` 中仍有已经移除源码的 `zed2i_nitros` 残留，它不是当前运行依赖；
- 当前源码树中已没有 `src/zed2i_nitros`；
- 换目录后直接复用旧 CMake cache 会出现“source directory does not exist”。这种情况应在确认路径后重建 build/install，而不是修改 CMake 去迎合旧绝对路径。

## 9. 故障定位速查

| 现象 | 先检查 |
|---|---|
| `/rgb_image` 正常但 `/debug/viz` 卡住 | Fusion lifecycle、TensorRT 推理日志、`/perception/output` 频率、GPU/engine 是否匹配。 |
| 没有绿色、蓝色路径 | `/controller_node` 是否存在、参数类型是否正确、感知是否超时、是否无有效道路/无安全轨迹。 |
| 只有蓝色圆点 | DWA 有效且停车候选得分最高；看净空、候选颜色和日志 score。 |
| 很多浅绿线 | 正常的有效 DWA 候选；亮蓝线才是被选中的轨迹。 |
| 只有绿色路径 | DWA 未达到激活点数，或障碍不在激活 ROI。 |
| 蓝线过弯 | 检查 `minimum_turning_radius`、硬件挡位角速度上限和 LQR 最终角速度。 |
| 障碍可见但不触发 | 对比 `/zed/point_cloud` 与 `/dwa/obstacle_cloud`，检查 x/y/z ROI 和激活点数。 |
| `prediction_time` 类型错误 | 必须写 `3.0`，不能写 `3`。ROS 2 参数类型严格。 |
| `/perception/output` 类型 invalid | source `/opt/ros/humble/setup.bash` 和当前 `install/setup.sh`。 |
| 蓝牙无法连接 | 地址、UUID、BlueZ、设备占用、服务解析、权限和轮椅状态。 |

## 10. 实车安全边界

当前代码可用于数据集、室内空载和受控低速研究测试，但尚不能据此宣称可安全载人自主避障。载人前至少完成：

1. 独立物理急停和安全员；
2. 轮椅含脚踏板、乘员的真实 footprint；
3. 相机外参和点云高度 ROI 实测；
4. ZED twist 的绝对尺度与时序修复，或接入可靠轮速反馈；
5. 急停距离按最高速度、端到端延迟和实测制动距离标定；
6. BLE 断连、超时、方向和挡位边界测试；
7. 空载低速的静态障碍、窄道、近障起步和退出避障测试；
8. 噪声点空间/时间一致性过滤；
9. 动态行人场景单独设计和验证，当前 DWA 未预测障碍物速度。

## 11. v0.4 结论

v0.4 已把 DWA、LQR、ZED 外参、实验性视觉速度反馈和 BLE 底盘桥连成较完整的工程链路，并明确区分数据集和实车状态。与原始纯 LQR 相比，它具备未来轨迹碰撞检查、道路硬边界、速度选择、底盘可执行域和无路径停车能力。

当前最重要的未完成项不是继续增大某个权重，而是：修复 ZED twist 时间尺度、避免 `/odom` 回调饥饿、加入障碍点簇/多帧确认，并以受控实测标定 footprint、制动距离和转弯能力。在这些问题解决前，系统应保持空载、低速、独立急停和安全员监督。
