# TAMI Wheel：道路约束 DWA + LQR 轮椅导航

> 文档更新：**2026-10-10**，对应 v0.5 之后的当前开发代码，已包含 2026-10-08 的感知时效与最终轨迹越界修复。
> 本 README 描述当前实现；[`dwa+lqr_v0.5.md`](dwa+lqr_v0.5.md) 等版本文档保留当时的参数、测试结果和已知问题，不应直接作为当前配置。

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
输入检查：紧急距离 / 数据时效 / 同帧点云 / 道路有效性
        |
        +-- 无近障：CRUISE 道路参考轨迹
        +-- 有近障：道路约束 DWA 候选与最优轨迹
                              |
                     LQR 前视点轨迹跟踪
                              |
             最终限幅 + 碰撞/道路边界复查
                              |
                    /cmd_vel（m/s、rad/s）
                              |
                 ble_hardware_bridge（独立启动）
                              |
                          轮椅底盘
```

实车速度反馈走独立链路：`ZED WORLD 位姿 → 相机外参修正 → 轮椅中心位姿差分 → EMA → /odom → ControllerNode`。BLE 桥只发送控制指令，不读取底盘轮编码器速度。室内测试模式每帧运行 DWA、忽略道路模型并跳过 LQR，详见后文。

主要代码：

| 模块 | 路径 |
|---|---|
| 感知与融合 | `src/wheel_perception/src/fusion_node.cpp` |
| ZED 驱动与视觉里程计读取 | `src/wheel_perception/src/core/zed_driver.cpp` |
| WORLD 位姿差分速度 | `src/wheel_perception/src/core/world_pose_velocity_estimator.cpp` |
| 道路几何计算 | `src/wheel_perception/include/wheel_perception/core/road_geometry.hpp` |
| 控制与安全层 | `src/wheel_perception/src/controller_node.cpp` |
| DWA | `src/wheel_perception/dwa_controller/src/DWAPlanner.cpp` |
| LQR | `src/wheel_perception/include/wheel_perception/core/lqr_controller.hpp` |
| 统一导航参数 | `src/wheel_perception/config/params.yaml` |
| BLE 桥 | `ble_hardware_bridge/ble_hardware_bridge/` |

## 已实现的主要改进

v0.4已经完成道路约束DWA、LQR局部轨迹跟踪、BLE可执行域和独立底盘桥。v0.5保持该导航结构，重点完善实车速度反馈：

- ZED改用 `REFERENCE_FRAME::WORLD` 绝对位姿，不再累加CAMERA相对运动；
- 使用同一ZED图像时间戳对连续WORLD位姿差分，产生 `/odom.twist`；
- 差分前利用相机外参恢复轮椅旋转中心位姿，消除转向杆臂速度；
- SDK `Pose.twist` 从控制链移除，仅发布 `/zed/diagnostics/sdk_twist` 诊断话题；
- 新增采样周期上下限、跟踪恢复重置、独立速度协方差和WORLD速度EMA；
- 新增 `WorldPoseVelocityEstimator` 模块及直行、杆臂、异常周期单元测试；
- 根据原地转向 bag 将相机前向外参由 `0.20 m` 调整为 `0.45 m`，仍需结合实际安装位置核实；
- 历史实车 bag 中一/二/三档视觉速度呈不同量级，静止漂移约毫米每秒；
- 在当时 bag 中 WORLD 差分与 SDK Twist 趋势高度相关，SDK 数值约为 WORLD 差分的 3.4 倍；
- 数据集模式继续使用上一控制输出，实车模式使用新鲜 `/odom.twist`，两者互不污染。

v0.5 之后已加入：

- `/odom`、点云和感知看门狗分别使用独立的 `MutuallyExclusive` callback group；实车 `/odom` 与感知订阅采用 `KeepLast(1)`，减少旧消息排队。
- 室内 drive 模式先求“实测角速度可达窗口”和“上一实际发布指令限幅窗口”的交集，再生成候选；交集为空时停车。该窗口交集目前只用于室内 drive，室外尚未统一。
- 条件性停车/非改善轨迹惩罚：以同速度直行的全程最小净空为基准，只有存在安全且确实改善净空的转弯轨迹时才应用，避免只惩罚停车后选择缓慢靠近障碍。
- 道路方向统一朝向车头 +X，正确区分垂直距离与 Y 轴截距；碰撞体到道路边界采用有符号垂直距离检查。
- 每帧校验感知源时间戳，并匹配同时间戳点云；室外道路无效时停车，不把旧道路拟合当作当前有效观测。
- LQR 修正和最终角速度限幅后，重新检查实际待发送 `(v,w)` 的碰撞与道路边界；额外使用当前未经过道路 EMA 的右边界直线复查。
- 独立感知看门狗与发布前时效检查，避免规划期间超时后又发出旧运动指令。
- `record_dwa_bag.sh` 一键录制，并尝试保存运行参数、源/安装 YAML 和 Git 版本。

上述安全修复已在代码中落实，并配有相应单元测试；**仍需在部署该版本后进行室外起步巡线、左右绕障与恢复巡线验证，不能据此认定越界问题已经彻底解决**。

目前尚未实现独立的净空改善奖励评分项、WORLD 速度物理异常过滤、障碍空间聚类、普通急停 2/3 帧确认及跨帧短时障碍记忆。三帧点云缓存用于同帧匹配，不是障碍记忆。

## DWA 与 LQR

DWA 使用二维运动学模型：

```text
x_dot   = v*cos(yaw)
y_dot   = v*sin(yaw)
yaw_dot = w
```

当前评分函数为：

```text
base_score =
    weight_heading  * cos(yaw_end - road_yaw_error)
  - weight_obstacle * (1 / max(minimum_clearance, 0.01))
  - weight_velocity * (max_velocity - v)
  - weight_road     * mean_road_offset
  - weight_smooth   * smooth_cost
```

评分越高越优。`minimum_clearance` 是预测全程到障碍点的最小二维距离减去 `robot_radius`；`mean_road_offset` 是预测点到道路目标线的平均绝对 Y 向偏差。平滑项包含归一化后的线/角速度变化、线/角加速度以及 jerk；当前两个 jerk 权重为零，`max_jerk` 和 `max_angular_jerk` 仅是归一化尺度，不是硬限制。

基础评分之后还有条件性惩罚：存在全程安全、动态/硬件可执行、最小净空至少为 `minimum_moving_clearance`，且比**同速度直行**至少多出 `minimum_clearance_gain` 净空的滚动转弯轨迹时，对其他有效候选（包括停车及未改善运动）扣除 `stop_preference.penalty`。无满足条件的转弯轨迹时不扣分；该机制不是无条件强制运动。随后使用轨迹保持机制，抑制评分接近的候选来回切换。

碰撞、越过道路边界、超过动态能力或 BLE 执行域的轨迹直接无效。轮椅以半径 `robot_radius` 的圆形 footprint 近似，道路硬约束要求车体中心到边界的垂直距离至少为 `robot_radius + road_margin`；将 `weight_road` 设为零不会取消该硬约束。

每条候选使用一组固定 `(v,w)` 积分预测，主要形成直线或圆弧。采样从动态窗口下限开始，并补上上限、上一指令、直行等特殊候选；`velocity_resolution=0.1` 不代表输出必须是 0.1 的整数倍。`visualization.max_candidates` 只限制显示数量，不限制规划候选总数。

室外 DWA 激活时，LQR 在选中轨迹上取 `lookahead_time` 前视点，利用横向/航向误差跟踪，角速度修正最多为参考角速度的 `±0.15 rad/s`，再受最终限幅及安全检查约束。该修正限制作用于整个 DWA 模式，不仅是模式切换时。

无近障时运行 `CRUISE+LQR`，不使用 DWA 评分或速度采样。线速度由巡线函数将 `cruise_velocity` 限制到当前反馈速度的加速度窗口内，LQR 主要决定角速度。室内模式 `road_cost=0`，但 heading 仍以车头 +X 为目标方向。

当前主要参数：

```yaml
dwa:
  max_velocity: 1.0
  min_velocity: 0.0
  max_angular_velocity: 1.0
  max_acceleration: 1.0
  max_angular_acceleration: 1.5
  prediction_time: 2.5
  simulation_time_step: 0.1
  velocity_resolution: 0.1
  angular_resolution: 0.05
  weight_heading: 1.0
  weight_obstacle: 3.5
  weight_velocity: 4.0
  weight_road: 1.0
  weight_smooth: 0.02
  minimum_turning_radius: 0.1
  cruise_velocity: 0.6
  robot_radius: 0.45
  road_margin: 0.15
  activation:
    max_x: 5.0
    min_y: -1.0
    max_y: 1.0
    minimum_points: 20
    clear_frames: 5
  stop_preference:
    enabled: true
    penalty: 6.0
    minimum_moving_clearance: 0.10
    minimum_clearance_gain: 0.05
  trajectory_hold:
    enabled: true
    score_switch_margin: 0.10
    relative_switch_margin: 0.01
    minimum_clearance: 0.30
    clearance_switch_margin: 0.10
  hardware_constraints:
    enabled: true
    minimum_moving_velocity: 0.15
  indoor_test:
    mode: "off"
    max_velocity: 0.30
safety:
  emergency_stop_distance: 0.5
  perception_timeout: 0.5
  max_sensor_age: 0.25
  stop_on_no_path: true
velocity_feedback:
  timeout: 0.3
  stop_on_timeout: false
```

以上是源 YAML 的部分当前调试值，实际运行参数还可能被 launch 覆盖。ROS 2 参数类型严格，例如 `prediction_time` 必须写 `2.5` 或 `3.0`，整数 `3` 会导致类型错误。多数规划参数在节点构造时载入；修改 YAML 后重新启动，不能只用 `ros2 param set` 成功就认定内部规划器已同步更新。

## 感知范围、坐标与安全检查

`base_link` 使用 X 向前、Y 向左、Z 向上；当前相机平移外参为 `(0.45, -0.20, 0.0) m`，旋转为零。`translation_z=0` 是当前外参配置，不能据此认为相机与地面的距离为零，部署时应实测轮椅中心、相机位置和姿态。

| 参数 | 当前范围/数值 | 使用坐标及用途 |
|---|---|---|
| `zed.depth_min/max` | 0.3～10.0 m | SDK 深度工作范围，不等于完整可靠可见区域 |
| `perception.roi` | X=[0.3,4.6]、Y=[-1.3,1.7]、Z=[-0.2,2.0] m | 相机坐标中的第一层点云过滤 |
| `dwa.obstacle_roi` | X=[0.3,5.0]、Y=[-1.5,1.5]、Z=[-0.2,1.0] m | 外参转换后的 base_link 点云过滤 |
| `dwa.activation` | 前方 5.0 m、Y=[-1.0,1.0] m，至少 20 点 | 决定室外是否启动 DWA；连续 5 帧未达到门槛后退出 |
| `perception.road_fit.max_abs_yaw` | 1.2 rad | 当前道路拟合可信方向范围；超范围/退化拟合标记无效 |
| `perception.obstacle` | `6*x*x + 225*y*y < 50` | 保留的感知危险椭圆，用于安全最近点候选，与 DWA 点云 ROI 不同 |

过滤是串联的：上游已经删除的点不会因扩大 DWA ROI 而恢复。当前零旋转外参下，相机前向 ROI 的近端约为轮椅中心前方 0.75 m；还需考虑视场、遮挡和深度质量。Z 范围必须结合相机高度检查，避免低矮障碍被过滤。

普通模式急停使用 `/perception/output.min_distance < 0.5`；该值是危险候选点转换到 base_link 后的**前向 X 距离**，并非二维欧氏距离或扣除碰撞半径后的净空。此门槛本身不再加 `robot_radius`。室内 drive 使用 `max(safety.emergency_stop_distance, robot_radius + 0.35)`，当前为 0.80 m；两种门槛都受有效危险点输入限制，不能把漏检区域视为空闲。

2026-10-08 增加的时效与边界防护包括：

1. 回调入口按图像源时间戳拒绝年龄超过 `safety.max_sensor_age=0.25 s` 的帧；感知与点云要求时间戳完全匹配，最多等待同帧点云 20 ms。
2. 室外缺失有效道路拟合或当前原始右边界线时停车；超出可信范围的拟合不会继续以旧 EMA 标记为有效道路。
3. 对 LQR 修正、角速度限幅后的最终 `(v,w)` 重新预测，并检查障碍物与道路边界；再以当前两个调试拟合点构建的未经过道路 EMA 的右边界线复查。
4. 独立看门狗处理感知回调中断；发布指令前再次检查源数据时效和停车状态，防止超时后的旧结果重新驱动。
5. 实车模式的亮绿色/蓝色最优路径显示最终通过安全检查的指令预测轨迹；候选仍表示 DWA 参考轨迹，预览模式则显示假设速度下的规划结果。

当前道路检查仍是拟合直线，不是对全部黄色点逐点构建道路轮廓；对弯路、局部凸起和感知盲区仍需验证。

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

当前 `test_dwa_planner.cpp` 定义 33 个测试用例（30 个 DWA、3 个道路几何），覆盖起步、BLE 执行域、条件性惩罚、轨迹保持、室内窗口交集、垂直道路距离及最终指令碰撞/越界等。用例数量是源码状态，不代表本文更新时已重新完成实车验证。

## 数据集回放

当前 `params.yaml` 默认启用数据集模式。`dwa_dataset_test.launch.py` 会包含主 launch，但没有强制覆盖数据集模式；回放前确认：

```yaml
zed:
  use_dataset_mode: true
```

启动一次 `shengwudao2` 回放：

```bash
cd ~/tami/tami_wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.sh

ros2 launch wheel_perception dwa_dataset_test.launch.py \
  dataset_path:=/home/x/tami/shengwudao2 \
  fps:=20.0 \
  loop:=false
```

不指定参数时，该 launch 默认使用 `~/tami/shengwudao2`、`fps:=10.0`、`loop:=false`；切换其他数据集只需修改 `dataset_path`，例如 `/home/x/tami/datasets6/text6_stereo`。播放器延迟 8 秒启动，仍应确认感知节点已激活、TensorRT 已就绪。

观察规划可用 `10~20 FPS`，评估吞吐再使用 `30 FPS`。回放帧率会改变实际控制周期及动态窗口，低速回放不等于实车控制效果；`loop:=true` 在首尾产生数据突变，不适合统计一次完整控制曲线。回放沿历史相机轨迹提供感知，不会根据新控制指令重新生成环境观测，因此不能验证真实底盘响应或完整闭环避障。数据集 launch 不启动 BLE 桥。

常用检查：

```bash
ros2 topic hz /perception/output
ros2 topic echo /perception/output --once
ros2 topic echo /dwa/planner_cmd
ros2 topic echo /cmd_vel
```

一键录包（实车、室内测试、数据集回放均可；先启动对应节点）：

```bash
bash record_dwa_bag.sh dwa_test
# 长时间轻量记录，不录规划点云和 Marker：
bash record_dwa_bag.sh dwa_test --light
```

脚本尝试保存运行参数快照和 Git 版本，并录制控制、速度反馈、路径与日志。参数服务不可用时会保留失败提示并继续录制；源/安装 YAML 不能替代成功获取的运行参数快照。
输出位于 `bags/测试名称_时间_唯一后缀/`，按 `Ctrl+C` 正常结束录包；
**结束录包不会停止轮椅**。具体话题、文件结构和注意事项见 [录包说明](录包说明.md)。

手动记录一次回放：

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
| `/dwa/planner_cmd` | DWA 或 CRUISE 的参考 `(v,w)`，在室外 LQR 修正之前 |
| `/dwa/local_trajectory` | 实车最终安全指令预测路径；室内预览为假设速度下的路径 |
| `/dwa/best_path` | 与局部最优路径相同的 Path 输出 |
| `/dwa/best_path_marker` | 绿色/蓝色选中轨迹或蓝色停车点 |
| `/dwa/candidate_paths` | DWA 候选轨迹 |
| `/perception/debug/right_road_edge` | 道路右边界点 |
| `/debug/viz` | 语义分割和边界叠加图 |
| `/odom` | 实车模式下实验性的 ZED 基座运动估计 |
| `/zed/diagnostics/sdk_twist` | 原始 SDK Twist 的 `base_link` 诊断值，不参与控制 |
| `/cmd_vel` | 最终物理单位速度指令；室外含 LQR 修正，室内 drive 跳过 LQR |

## 只接 ZED 的规划测试

显式使用实车相机模式，不启动 BLE：

```bash
ros2 launch wheel_perception run_launch.py use_dataset_mode:=false
```

此模式能验证实时感知、DWA 路径和 ZED `/odom`，但不能验证底盘跟踪效果。检查：

```bash
ros2 topic hz /odom
ros2 topic echo /odom --field twist.twist
ros2 topic echo /zed/diagnostics/sdk_twist --field twist
ros2 run tf2_ros tf2_echo base_link zed_left_camera_frame
```

## 室内 DWA 单独测试（不依赖道路边界）

室内专用 launch 沿用同一套 ZED、TensorRT、ObstacleFusion 和 DWA 参数，
但用正前方（base_link +X）作为参考方向，忽略室内不可靠的道路边界，
每帧运行 DWA，跳过 LQR。正常 `run_launch.py` 默认仍是原来的 DWA＋LQR。

先检查 `ai.engine_path` 指向本机可用的 TensorRT engine，并检查相机外参、
`perception.roi`、`dwa.obstacle_roi` 能稳定保留雪糕筒点云。
**建议先不启动 BLE**，运行只规划模式：

```bash
cd ~/tami/tami_wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.sh
ros2 launch wheel_perception indoor_dwa_test.launch.py
```

此模式的 `/cmd_vel` 始终为零；`/dwa/indoor_test_cmd` 和
`/dwa/planner_cmd` 是**假设当前速度为 0.30 m/s**时的规划结果，
不代表静止轮椅实际可执行的起步命令。可通过
`dwa.indoor_test.preview_velocity` 在原 YAML 中改变该假设。
RViz 使用上面的配置，重点看 `/dwa/obstacle_cloud`、蓝色最优路径、
浅绿色/红色候选路径及蓝色停车点。终端会打印候选数、可行数、净空与分数。

```bash
ros2 topic echo /cmd_vel --field linear.x
ros2 topic echo /dwa/indoor_test_cmd
ros2 topic echo /dwa/obstacle_cloud --once
ros2 topic echo /odom --field twist.twist
```

只有在雪糕筒点云稳定、候选路径与近距离停车反复验证后，才考虑
**无人乘坐、物理急停可用、有安全员**的低速移动测试：

```bash
ros2 launch wheel_perception indoor_dwa_test.launch.py mode:=drive allow_motion:=true
```

另开终端单独启动 BLE 桥（上面的 launch **不会**启动 BLE）。移动测试要求
真实且新鲜的 `/odom.twist` 和障碍点云；任何一项超时、DWA 无安全路径、
急停距离触发，都会向 `/cmd_vel` 发零速度。室内移动模式的急停阈值
至少为 `robot_radius + 0.35 m`（当前参数下为 0.80 m），正常模式的
0.50 m 不变。室内线速度上限 `dwa.indoor_test.max_velocity` 默认为
0.30 m/s，仍受 BLE 最低速度
0.15 m/s 和角速度约束。若后置限幅改变 DWA 检查过的角速度，也直接停车。
这不是载人自主运行配置；相机漏检雪糕筒的问题必须先解决。

## 实车启动

先启动感知与控制：

```bash
cd ~/tami/tami_wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.sh
ros2 launch wheel_perception run_launch.py use_dataset_mode:=false
```

`run_launch.py` 的数据集默认值来自 YAML，当前为 `true`；实车应保留上述 `use_dataset_mode:=false`，避免误用数据集的上一输出速度基准。确认道路、点云、轨迹、急停和 `/cmd_vel` 正常后，在独立终端启动 BLE：

```bash
source /opt/ros/humble/setup.bash
source ~/tami/tami_wheel_latest/install/setup.sh
ros2 launch ble_hardware_bridge ble_hardware_bridge.launch.py
```

不要在数据集回放时启动 BLE。蓝牙地址、特征 UUID 和速度标定在：

```text
ble_hardware_bridge/ble_hardware_bridge/config/ble_hardware_bridge.yaml
```

BLE 桥将 `/cmd_vel.linear.x`（m/s）和 `/cmd_vel.angular.z`（rad/s）按标定映射为摇杆幅值和蓝牙帧。它按目标线速度选择 0001/0003/0005 档，并在各档内调整幅值，实现可调目标速度；离散挡位、死区与有限 PWM 精度仍存在，不能据此保证任意小速度都可实现或实际速度精确等于指令。当前配置发送频率为 20 Hz，超过 0.3 秒未收到新指令时发送停车帧，无效指令也排队停车；连接失败自动重试。不要同时运行旧 `sub.py` 和 BLE 桥。

运行后可检查实际参数和速度：

```bash
ros2 param get /controller_node zed.use_dataset_mode
ros2 param get /controller_node dwa.indoor_test.mode
ros2 param get /controller_node dwa.cruise_velocity
ros2 param get /controller_node safety.max_sensor_age
ros2 topic hz /perception/output
ros2 topic hz /odom
ros2 topic echo /cmd_vel --field linear.x
ros2 topic echo /odom --field twist.twist.linear.x
```

## ZED 速度反馈现状

当前 `/odom.twist` 使用ZED图像时间戳对连续WORLD绝对位姿差分，并先通过外参消除相机杆臂；SDK Twist只发布到 `/zed/diagnostics/sdk_twist`，不参与DWA。数据集模式继续使用上一控制输出，两种模式互不影响。

v0.5 的历史实车 bag 结果（对应当时的摇杆操作、参数和场地，并非当前所有运行条件的标定真值）：

- 完整链路实际约 `16~17 Hz`，WORLD速度尺度不再依赖配置的 `60 FPS`；
- 一档、二档、三档稳定段约为 `0.47 / 0.66~0.68 / 0.88 m/s`；
- 静止 `linear.x` 标准差约 `0.001 m/s`；
- `translation_x=0.45 m` 后，原地转向横向杆臂残差从约 `0.25 m` 降到约 `0.005~0.012 m`；
- WORLD/SDK的线速度中位比例约 `0.299`、相关系数约 `0.958`；
- WORLD/SDK的角速度中位比例约 `0.292`、相关系数约 `0.985`；
- 在这些 bag 中 SDK Twist 与 WORLD 差分趋势相近，但数值约为后者的 `3.3~3.6` 倍；尚无独立真值证明哪一个绝对值完全准确，因此当前仅将 SDK Twist 用于诊断。

WORLD估计器已通过直行速度、杆臂补偿和异常时间间隔单元测试，但它仍是视觉惯性里程计，不是轮编码器真值。当前配置保持：

```yaml
velocity_feedback:
  stop_on_timeout: false
```

`/odom` 已放入独立 `MutuallyExclusive` callback group；这减少与规划回调互相阻塞，但仍需通过新 bag 验证实际处理时效。详细历史实现和测试步骤见 [`dwa+lqr_v0.5.md`](dwa+lqr_v0.5.md) 和 [`zed_world_pose_velocity_report.md`](zed_world_pose_velocity_report.md)。

近期历史 bag 复核支持软件侧线速度指令已可连续调节，视觉反馈独立于指令；记录的 `/odom` 频率约 12～17 Hz。室外包 4 的几个加速片段中，指令与视觉反馈中位数相差约 0.06 m/s，同时日志主要为 `CRUISE+LQR` 并使用 `zed_odom`。该差距与每周期 `max_acceleration * dt` 同量级，还可能叠加底盘响应和滤波滞后，不能直接认定为固定蓝牙映射误差。旧 bag 也出现过反馈过期/回退日志，不能证明当前代码全程稳定取用实测值。

当前反馈用于规划动态窗口和巡线速度限幅，**没有专门的纵向速度误差积分补偿控制器**。CRUISE 在低于目标时通常输出 `v_feedback + max_acceleration * dt`，再处理最低可执行速度；上坡实际速度长期偏低时，指令可能一直留在低速区。需要用平地/缓上坡固定速度平台、已知距离与计时验证执行和测速，再决定如何修改纵向控制。

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
| 近障起步不动 | 区分急停、无可行路径、评分选停车、窗口交集为空及最终安全拒绝；停车惩罚仅在存在合格绕行候选时生效。 |
| 调高停车惩罚但 score 不变 | 可能未满足同速度直行净空改善条件，或运行参数未更新；检查候选和实际参数。 |
| CRUISE 巡线长期低速 | 对比指令、视觉反馈和独立测速；查实际目标/加速度、反馈基准限幅、反复停车和底盘执行。DWA 采样间隔不参与此模式。 |
| `NO_WINDOW_INTERSECTION` | 室内 drive 的实测角速度窗口与上一发布指令窗口不相交；检查测速、响应滞后和人工接管。 |
| 蓝线穿过黄色右边界 | 确认部署最新版本与室外模式，检查时效、外参、拟合直线与原始点差异；当前新增最终轨迹与原始右边界线复查。 |
| `Final command rejected` | LQR/限幅后的最终轨迹未通过碰撞或道路检查；查看原因和道路数据，不能仅调评分权重。 |
| 起步或运行中反复停车 | 查看感知过期、同帧点云缺失、道路无效、看门狗、BLE 拒绝/超时等日志。 |
| 参数 invalid type | 需要 double 的值必须保留小数点。 |
| 换目录后 CMake 指向旧源码 | 删除确认无用的旧 build/install 缓存后重新构建。 |

## 当前边界与后续优先级

当前适合验证静态局部障碍物和道路约束，不具备完整全局导航、动态障碍速度预测、非圆形 footprint、倒车脱困或原地转向，也没有完整的基于制动距离的速度可停性约束。相机盲区、低矮障碍漏检和绕障后观测丢失仍是限制。

建议下一轮验证顺序：

1. 固定代码和参数，在平地/缓上坡保持低速目标，结合距离计时检查蓝牙执行、视觉速度及指令差值。
2. 无障碍道路从相同起点进行对齐、轻微左偏/右偏起步，验证新时效与边界检查既不越界，也不过度停车。
3. 固定单障碍位置，分别验证开始绕障、完整通过与恢复巡线，并记录每次人工接管。
4. 保存实际参数、Git 版本及原始 bag；将因即将碰撞/越界而接管的试验记为失败。仅在这些基础场景稳定后进行连续路线测试。

后续开发方向：

- 增加轻量逐周期诊断：所用数据时间戳、速度来源/基准、动态窗口、参考/最终指令及拒绝原因，区分感知、规划和执行问题。
- 根据新数据评估室外规划与最终转向限幅基准是否需要统一；验证角速度响应，避免直接放宽约束。
- 标定相机高度/姿态与过滤范围，改善低矮障碍保留；增加空间去噪和时间一致性，但保留紧急威胁处理。
- 实现短时障碍记忆，使暂时离开视野的障碍同时参与规划和最终安全检查，避免恢复巡线时再次靠近它。
- 增加速度物理异常检查，并验证轮椅外廓、延迟、制动能力和 BLE 失联行为，再完善可停性约束。

当前实现以源码、运行参数和本文为准；v0.5/v0.4 的版本差异及当时测试数据分别保留在 [`dwa+lqr_v0.5.md`](dwa+lqr_v0.5.md) 和 [`dwa+lqr_v0.4.md`](dwa+lqr_v0.4.md)。
