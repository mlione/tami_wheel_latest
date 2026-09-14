# DWA + LQR v0.3：今日改进、当前状态与运行说明

本文档记录 `wheel_latest` 在 2026-09-14 的实际开发与调试结果，重点说明
`DWA + LQR v0.2` 到 `v0.3` 的增量变化。它不代替原始感知链路，也没有替换 LQR 或
底盘通信协议。

> **安全状态**：v0.3 仍是数据集和 RViz 调试版。当前已观察到巡线 LQR 角速度饱和、
> 单个近距离点可主导 DWA 净空以及假设标定深度不可用于实车测距等未闭环问题，
> 因此不能把“编译通过”理解为“可直接载人运行”。

## 1. v0.3 概要

v0.3 围绕“起步遇障有可视化、可解释的结果”完成了三项核心改动：

1. 从“低速不能转弯”改为“允许低速滚动转弯，仍禁止原地旋转”；
2. 增加最小转弯半径硬约束，同时限制 DWA 候选和 LQR 最终角速度；
3. DWA 主动选择停车时，RViz 用蓝色停车圆点显示，不再把零长度轨迹误认为“没有路径”。

同时完成了数据集默认路径、双目深度转换辅助工具、启动期控制数据记录以及
参数类型故障诊断。

## 2. v0.2 与 v0.3 对比

| 项目 | v0.2 | v0.3 |
|---|---|---|
| 低速转弯 | `v < 0.10 m/s` 时禁止非零 `w` | 允许所有正向低速滚动弧线，仅 `v≈0` 时禁止 `w≠0` |
| 原地旋转 | 禁止 | 仍禁止 |
| 曲率约束 | 没有显式最小半径 | 候选和最终跟踪均满足 `|v/w| >= R_min` |
| 起步候选 | 主要保留直行/停车 | 增加左右对称的半径边界候选 |
| DWA 选择停车 | 零长度 `LINE_STRIP`，RViz 几乎不可见 | 蓝色扁球形圆点，直径约 `0.18 m` |
| 数据集 launch | `dataset_path` 必填 | 默认 `~/tami/shengwudao2`，仍可命令行覆盖 |
| DWA 测试 | 9 项 | 11 项，增加低速滚动转弯和最小半径检查 |
| 故障可解释性 | 蓝线消失时难以区分停车与无解 | 停车蓝点、节点/话题/日志联合诊断 |

## 3. 当前控制链路

```text
ZED RGB-D / 历史 RGB-D 数据
        |
TensorRT BiSeNet + CUDA 点云融合
        |
ObstacleFusion
  |-- PerceptionOutput：右边界、道路宽度、航向误差、最近距离
  `-- /zed/point_cloud
        |
DWA 专用三维 ROI -> 均匀降采样 -> 二维碰撞点
        |
        +-- 激活区点数 < minimum_points -> CRUISE 道路参考轨迹
        |
        `-- 激活区点数 >= minimum_points -> DWA 候选预测与评分
                                                        |
                                                最优 (v_ref,w_ref)
                                                        |
                                    LQR 前视横向/航向误差跟踪
                                                        |
                  角速度、角加速度、最小转弯半径限幅
                                                        |
                                      原有协议增益与符号转换
                                                        |
                                                     cmd_vel
```

Safety Layer 仍优先于上述计算：最近距离过小、感知超时或 DWA 无有效路径时立即发布停车。

## 4. 今日的代码改动

### 4.1 低速滚动转弯

`minimum_turning_velocity` 从实质上的低速转弯禁止阈值改为数值零阈值：

```yaml
minimum_turning_velocity: 0.001
```

当 `v > 0.001 m/s` 时允许滚动转弯；当 `v < 0.001 m/s` 时仍只允许 `w=0`。这样既能
在静止起步时采样低速弧线，又没有恢复当前阶段不考虑的原地旋转状态。

### 4.2 最小转弯半径硬约束

新增：

```yaml
minimum_turning_radius: 0.2
```

候选轨迹必须满足：

```text
abs(v / w) >= minimum_turning_radius
```

对每个线速度采样，规划器还显式添加 `w=±v/R_min` 边界点，防止角速度分辨率较粗时
只采样到一侧。LQR 生成 `w_track` 后也会再次按同一半径限幅，防止跟踪修正突破 DWA 几何约束。

> `0.2 m` 是当前工作区调试值，对载人轮椅可能过小。它必须由实际轮距、底盘能力、
脚踏板扫过区域和乘员舒适性标定，不能只为了看到更弯的 RViz 轨迹而继续减小。

### 4.3 DWA 停车圆点

过去 `(v,w)=(0,0)` 的预测点全部重合于原点，`LINE_STRIP` 没有可见长度。v0.3 将这种情况显示为：

```text
Marker type: SPHERE
scale: 0.18 x 0.18 x 0.08 m
color: blue
frame: base_link
```

可视化语义为：

- 绿色曲线：CRUISE 巡线，DWA 未介入；
- 蓝色曲线：DWA 介入并选择运动轨迹；
- 蓝色圆点：DWA 结果有效，但停车候选评分最高；
- 选中 Marker 被删除：紧急停车、感知超时或无有效路径。

该改动只影响 RViz，不修改 DWA 评分或底盘指令。

### 4.4 数据集启动便利性

`dwa_dataset_test.launch.py` 现在为 `dataset_path` 提供默认值：

```text
~/tami/shengwudao2
```

因此默认数据集存在时可直接启动，仍可用 `dataset_path:=...` 选择其他数据。感知/TensorRT
初始化后再延时 8 秒启动播放器，减少首帧被丢弃。

### 4.5 双目数据转换工具

新增辅助工具：

| 文件 | 作用 |
|---|---|
| `tools/export_zed_calibration.py` | 连接真实 ZED 时导出对应分辨率的左右内参和基线 |
| `tools/convert_text6_stereo_dataset.py` | 使用 StereoSGBM 将已校正左右图转换为项目可回放的 `left/ + depth_npy/` |
| `tools/assumed_zed_hd720_calibration.json` | 无相机时用于管线调试的假设标定 |

转换器按 `Z=fx*baseline/disparity` 产生米制深度，但只有“采集该数据的同一台相机、同一分辨率、
已立体校正图像”才具有可信的尺度。假设标定产生的深度只能验证数据格式和软件链路，不能用于
`robot_radius`、`clearance` 或紧急停车距离的实车判断。

## 5. DWA 模型、动态窗口和评分

### 5.1 运动学模型

候选 `(v,w)` 在一段预测时间内保持常值：

```text
x(k+1)   = x(k) + v*cos(yaw(k))*dt
y(k+1)   = y(k) + v*sin(yaw(k))*dt
yaw(k+1) = yaw(k) + w*dt
```

这是二维差速/单轨等效运动学模型，并非包含轮距、轮胎侧偏、电机滞后和负载的完整轮椅动力学模型。

当前速度为 `(v0,w0)`，实际控制周期为 `control_dt` 时：

```text
v in [v0-a_max*control_dt, v0+a_max*control_dt]
w in [w0-alpha_max*control_dt, w0+alpha_max*control_dt]
```

`control_dt` 限制在 `0.01~0.25 s`。从静止以 20 Hz 控制且 `a_max=0.5 m/s^2` 起步时，
首个动态窗口的最大线速度只有约 `0.025 m/s`。因此起步前障碍物过近时，局部规划器可能持续选择停车。

### 5.2 碰撞模型

轮椅当前用半径 `robot_radius` 的圆形近似。对轨迹每个 pose 和每个二维障碍点：

```text
clearance = hypot(pose.x-obstacle.x, pose.y-obstacle.y) - robot_radius
```

任一 `clearance <= 0` 就把轨迹判为碰撞；整条轨迹的净空是所有值中的最小值。

### 5.3 评分函数

```text
score =
    weight_heading  * cos(endpoint_yaw-road_yaw_error)
  - weight_obstacle * (1/minimum_clearance)
  - weight_velocity * (max_velocity-v)
  - weight_road     * mean_road_offset
  - weight_smooth   * normalized_motion_change
```

碰撞、越出道路或超过加速度窗口的轨迹评分直接为负无穷，不参与最优选择。

`score < 0` 不表示轨迹无效；评分只用于有效候选之间比较。今日回放中观察到
`clearance=0.15~0.30 m` 时 DWA 多次选择 `v=0,w=0`，而当 `clearance=1.29 m` 时选择了
`v=0.11,w=-0.07`，说明低速滚动候选已工作，但近点净空和平滑代价仍可使停车成为局部最优。

## 6. 当前 YAML 调试快照

以本文档写入时的 `src/wheel_perception/config/params.yaml` 为准：

| 分类 | 参数 | 当前值 | 说明/风险 |
|---|---|---:|---|
| 评分 | `weight_heading` | 1.0 | 允许更大绕障偏航 |
| 评分 | `weight_obstacle` | 1.5 | 减少停车的净空评分优势，碰撞硬约束仍保留 |
| 评分 | `weight_velocity` | 6.0 | 偏积极的前进值，实车建议从 `3.0` 左右重新验证 |
| 评分 | `weight_road` | 2.5 | 保留道路目标线约束 |
| 平滑 | `weight_smooth` | 0.02 | 降低起步 jerk 对运动候选的过度惩罚 |
| 平滑 | `weight_jerk / weight_angular_jerk` | 0.3 / 0.5 | 当前起步调试值 |
| 几何 | `minimum_turning_velocity` | 0.001 m/s | 只禁止近似原地旋转 |
| 几何 | `minimum_turning_radius` | 0.2 m | 未经实车标定，可能过小 |
| 激活 | `minimum_points` | 20 | 按点数而非点簇激活 |
| 激活 | `clear_frames` | 5 | 障碍消失后延迟退出 DWA |
| 轨迹保持 | `enabled` | true | 当前已启用 |
| 轨迹保持 | `score_switch_margin` | 0.1 | 值较大时可能保持旧停车候选 |
| 安全 | `robot_radius` | 0.45 m | 应覆盖轮椅、脚踏板和乘员 |
| 安全 | `emergency_stop_distance` | 0.3 m | 当前调试值偏小，实车必须按制动距离重新标定 |
| 道路 | `dynamic_aim.enabled` | false | 当前使用 `lqr.aim_dist=0.8 m` |

必须保持 ROS 2 参数类型：C++ 声明为 `double` 的参数在 YAML 中也必须写小数。

```yaml
weight_velocity: 6.0  # 正确：float
weight_velocity: 6    # 错误：integer，ControllerNode 构造失败
```

今日 RViz 所有 DWA 路径一度消失的根因就是 `weight_velocity` 被写成整数。该故障时
`FusionNode` 仍正常运行，但组件容器中没有 `/controller_node`，因此不会出现任何 `/dwa/*` 话题。

## 7. 今日诊断结论

### 7.1 起步无蓝线的三种不同含义

| 现象 | 判定方法 | 含义 |
|---|---|---|
| 没有 `/controller_node` 和 `/dwa/*` | `ros2 node list` / `ros2 topic list` | 组件加载失败，本次是 YAML 整数/浮点类型冲突 |
| 有 DWA 日志且 `v=0,w=0` | 蓝色停车点 | 有效停车轨迹评分最高 |
| `DWA found no collision-free...` 或 Safety stop | 选中 Marker 被清除 | 无道路内无碰撞轨迹，或安全层介入 |

数据播放在感知初始化后延迟约 8 秒开始，在首帧感知到达前没有路径是启动时序的正常结果。

### 7.2 混合近噪声点会主导 DWA

`activation.minimum_points=20` 只检查激活矩形内的总点数。它不要求近20个点属于同一物体，
也不检查点簇和多帧持续性。因此“少量近噪声 + 大量远障碍点”可以激活 DWA。

评分阶段使用所有被保留障碍点的最小净空，所以只要一个近噪声点通过 ROI 和降采样，
它就可能主导 `obstacle_cost`，甚至把运动轨迹判为碰撞。

调高 `minimum_points` 不能根治该问题；降低 `weight_obstacle` 也无法恢复已被硬性判为碰撞的轨迹。
后续应在 DWA 输入层增加体素/空间聚类和多帧确认，同时保留对细杆和桌腿等小障碍物的安全检测。

### 7.3 巡线 LQR 持续饱和

回放日志中出现：

```text
CRUISE+LQR: v=0.60 w_ref=0.00 w_track=-1.00
```

这不是 DWA 规划了大角速度，而是 DWA 未介入的 CRUISE 轨迹被 LQR 修正到全局角速度下限。
CRUISE 参考线为：

```text
target_offset = aim_dist - right_distance
y(x) = target_offset + tan(road_yaw_error)*x
```

因此必须在实车前验证 `has_road_edge`、`right_distance`、`road_yaw_error` 的单位、符号、稳定性和
DWA/CRUISE 切换时 LQR 历史状态。单纯继续调 DWA 权重不能修复该问题。

### 7.4 实车速度反馈仍未完成系统级验收

控制器在数据集模式明确使用 `last_output_velocity_`，避免历史回放速度污染当前动态窗口。
实车模式的控制器支持读取 `/odom.twist`，但本项目自身的 `/odom` 生成链路尚未证明能持续提供有效 `twist`。
不接真实里程计时会回退到上一控制输出，本质上仍是命令估计而非底盘速度闭环。

### 7.5 性能日志的正确含义

`ZED Grab + AI Infer + Fusion Kernel + Cloud Pub + Vis & Logic` 的 `CURRENT FPS` 是感知链单帧计算耗时的倒数，
当前这段 profiling 不包含 DWA/LQR 耗时，也不等于数据集播放速率或实车端到端控制帧率。

## 8. 修改文件说明

| 文件 | v0.3 相关改动 |
|---|---|
| `src/wheel_perception/dwa_controller/include/.../DWAPlanner.hpp` | 将低速阈值改为近零阈值，新增 `minimum_turning_radius` |
| `src/wheel_perception/dwa_controller/src/DWAPlanner.cpp` | 生成对称半径边界采样，过滤转弯半径过小的候选 |
| `src/wheel_perception/src/controller_node.cpp` | 加载半径参数，限制 LQR 最终曲率，发布蓝色停车圆点 |
| `src/wheel_perception/test/test_dwa_planner.cpp` | 新增低速滚动转弯和最小半径测试，共 11 项 |
| `src/wheel_perception/config/params.yaml` | 当前 DWA 权重、低速阈值、转弯半径、激活点数等调试值 |
| `src/wheel_perception/launch/dwa_dataset_test.launch.py` | 增加默认数据集路径 |
| `zed_dataset_player.py` | 直接执行时的默认数据集路径改为 `~/tami/shengwudao2` |
| `tools/*.py/json` | ZED 标定导出、立体重建转换和假设标定 |
| `README.md` | 同步 v0.3 可视化、参数、运行和风险说明 |

## 9. 编译与测试

当前工作区可用：

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.sh
```

当前环境的 `colcon` 缺少包选择扩展，因此 `--packages-select` 可能报未识别。对已配置的构建目录，
只编译控制组件可使用：

```bash
cmake --build build/wheel_perception --target controller_node -j2
cmake --install build/wheel_perception
```

DWA 测试：

```bash
cmake --build build/wheel_perception --target test_dwa_planner -j2
./build/wheel_perception/test_dwa_planner
```

v0.3 代码变更后 `controller_node` 已编译通过，11 项 DWA 单元测试在新增最小转弯半径时全部通过。

## 10. 数据集回放

确认：

```yaml
zed:
  use_dataset_mode: true
```

使用默认数据集：

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.sh
ros2 launch wheel_perception dwa_dataset_test.launch.py
```

显式指定路径、帧率且只播放一次：

```bash
ros2 launch wheel_perception dwa_dataset_test.launch.py \
  dataset_path:=/absolute/path/to/dataset \
  fps:=20.0 \
  loop:=false
```

循环回放便于 RViz 观察，但每轮结束到开始的场景跳变会产生非物理的速度和误差不连续。
正式控制曲线分析应使用 `loop:=false`。

## 11. RViz 可视化

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.sh
rviz2 -d install/wheel_perception/share/wheel_perception/rviz/dwa_navigation.rviz
```

Fixed Frame 设为 `base_link`，检查：

| Display | 话题 | 用途 |
|---|---|---|
| PointCloud2 | `/dwa/obstacle_cloud` | 只显示真正进入 DWA 的二维障碍点 |
| MarkerArray | `/dwa/candidate_paths` | 浅绿为有效候选，红色为无效候选 |
| Marker | `/dwa/best_path_marker` | 绿色 CRUISE、蓝色 DWA 曲线或蓝色停车点 |
| PointCloud2 | `/perception/debug/right_road_edge` | 黄色右道路边界 |
| Image | `/debug/viz` | 语义分割和道路边界叠加图 |

如果全部 DWA Display 都消失，先检查：

```bash
ros2 node list | grep controller
ros2 topic list | grep dwa
ros2 component list
```

正常应在 `/wheel_container` 内同时看到 `/fusion_node` 和 `/controller_node`。

## 12. 实车运行前提

首先修改：

```yaml
zed:
  use_dataset_mode: false
```

启动感知和规划，但暂不启动蓝牙底盘桥：

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.sh
ros2 launch wheel_perception run_launch.py
```

只有在道路边界、净空、轨迹、安全停车和指令符号全部验证后，才单独启动原有底盘桥：

```bash
ros2 run wheel_perception sub.py
```

数据集回放时不得启动 `sub.py`。

实车前必须完成：

1. 使用真实 ZED 标定，检查相机到 `base_link` 外参；
2. 把 `minimum_turning_radius`、`robot_radius` 与脚踏板扫过区域按实物标定；
3. 根据最大车速、系统时延和制动距离标定 `emergency_stop_distance`；
4. 解决或证明 CRUISE 下 `w_track` 不会持续饱和；
5. 确认真实 `/odom.twist` 的发布者、单位、符号和更新频率；
6. 先不连底盘观察，再悬空/空载/低速/受控场地逐级测试；
7. 使用独立物理急停和安全员，不得仅依赖软件停车。

## 13. v0.3 仍未完成的能力

- DWA 障碍点还没有空间聚类、统计离群点过滤和多帧确认；
- 对行人等动态障碍物没有速度预测；
- 轮椅 footprint 仍是圆形，没有精确脚踏板外形；
- 恒 `(v,w)` 预测不是完整动力学轨迹；
- 禁止原地旋转和倒车，因此对近距离起步堵塞没有完整脱困状态；
- 没有全局地图、全局路径和重定位；
- 实车里程计速度反馈和载人安全尚未完成系统级验收。

## 14. 建议的 v0.4 优先级

1. 在不改变紧急安全层优先级的前提下，为 DWA 规划点云加入体素点簇和多帧确认；
2. 在日志中增加 `has_road_edge/right_distance/road_yaw_error/lateral_error/heading_error`，定位 LQR 饱和；
3. 使用真实 ZED 标定重建可度量深度数据；
4. 闭合真实 `/odom.twist` 链路，完成指令速度和实测速度对比；
5. 用非循环数据集统一记录 DWA 模式、`v_ref/w_ref`、`w_track`、净空和安全停车原因。
