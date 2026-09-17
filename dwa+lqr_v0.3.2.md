# DWA + LQR v0.3.2：参数调试快照与阶段进度

本文档记录 `wheel_latest` 在 2026-09-17 的开发与数据集回放结果，基线为 Git 提交：

```text
fdd8e6b3ac23a95423376f037f955a15ab4430f5  v0.3.1
```

当前分支为 `test_dwa`。在编写本文档之前，工作区相对 v0.3.1 只有
`src/wheel_perception/config/params.yaml` 存在未提交修改；没有修改 DWA、LQR、感知或底盘通信的 C++ 实现。

> **版本性质**：v0.3.2 是参数调试和问题定位版本，不是已经完成实车安全验收的发布版本。
> 数据集测试和 RViz 可视化不能代替空载实车制动距离、轮椅外廓、底盘响应和急停测试。

## 1. v0.3.2 概要

v0.3.2 在保留 v0.3.1 全部控制结构的基础上完成了以下参数调整：

1. DWA 前向激活、点云 ROI 和障碍评价距离统一从 `4.0 m` 扩大到 `5.0 m`；
2. 最大角加速度从 `2.5 rad/s²` 提高到 `5.0 rad/s²`；
3. 最小滚动转弯半径从 `0.2 m` 放宽到 `0.1 m`；
4. 线 jerk 和角 jerk 的评分权重临时设为零，用于排除 jerk 对近障起步的影响；
5. 通过多次数据集回放确认：近障时持续停车的主要原因仍是安全层抢占、零速动态窗口较小，以及停车轨迹在现有评分中占优，并非单纯由最小转弯半径造成。

本版本没有实现原地转向，也没有改变安全层优先级。

## 2. 相比 v0.3.1 的精确代码差异

Git 差异统计：

```text
src/wheel_perception/config/params.yaml | 14 行变化
7 insertions, 7 deletions
```

| 参数 | v0.3.1 | v0.3.2 | 目的 |
|---|---:|---:|---|
| `dwa.max_angular_acceleration` | `2.5` | `5.0 rad/s²` | 允许角速度更快建立，增强低速绕障响应 |
| `dwa.smooth.weight_jerk` | `0.3` | `0.0` | 临时取消线 jerk 软惩罚 |
| `dwa.smooth.weight_angular_jerk` | `0.5` | `0.0` | 临时取消角 jerk 软惩罚 |
| `dwa.minimum_turning_radius` | `0.2` | `0.1 m` | 允许更紧的低速滚动弧线 |
| `dwa.activation.max_x` | `4.0` | `5.0 m` | 更早检测并激活局部规划 |
| `dwa.obstacle_distance_threshold` | `4.0` | `5.0 m` | 将碰撞与净空评价范围同步到 5 m |
| `dwa.obstacle_roi.max_x` | `4.0` | `5.0 m` | 保证 5 m 障碍点实际进入规划器 |

三项前向距离必须同步修改。如果只修改 `activation.max_x`，而点云 ROI 或障碍评价仍在 4 m 截断，
控制器虽然可能提前切换到 DWA 模式，却无法使用 4～5 m 的障碍点进行完整评价。

## 3. v0.3.2 没有改变的内容

以下功能与 v0.3.1 完全一致：

- DWA 的二维差速/独轮车运动学模型；
- 每条候选在预测周期内使用常值 `(v,w)`；
- DWA 原有代价函数和五类代价项；
- 圆形轮椅碰撞模型 `robot_radius=0.45 m`；
- `v≈0,w≠0` 的原地转向候选仍被禁止；
- LQR 的状态构造、前视点跟踪、协议增益和输出转换；
- 数据集模式使用 `last_output_velocity_`，实车模式优先读取 `/odom.twist`；
- 紧急停车、感知超时和无有效路径停车；
- ZED、TensorRT、BiSeNet、CUDA 点云融合和道路边界提取；
- 原有底盘蓝牙/速度指令通信接口。

特别注意：虽然轮椅硬件可能支持原地旋转，但当前代码仍有两层禁止：

```text
DWA 候选层：v < minimum_turning_velocity 且 w != 0 时淘汰候选
最终控制层：最优线速度低于阈值时强制 w_track = 0
```

`minimum_turning_radius=0.1 m` 只允许更紧的正向滚动弧线，不代表允许半径为零的原地旋转。

## 4. 当前控制结构

```text
ZED RGB-D / 历史 RGB-D 数据集
        |
TensorRT BiSeNet + CUDA 点云融合
        |
ObstacleFusion
  |-- 道路右边界、道路宽度、道路方向
  |-- 椭圆危险区 min_distance
  `-- /zed/point_cloud
        |
Safety Layer
  |-- min_distance < 0.5 m：立即停车，跳过 DWA
  |-- 感知超时：停车
  `-- DWA 无有效路径：停车
        |
DWA 点云 ROI（当前前向 0.3～5.0 m）
        |
  +-- 激活区不足 20 点：CRUISE 道路轨迹
  `-- 激活区达到 20 点：DWA 候选预测与评分
                                  |
                         最优局部轨迹 (v,w_ref)
                                  |
                         LQR 前视点跟踪
                                  |
             角速度、角加速度、最小半径再次限幅
                                  |
                              /cmd_vel
```

## 5. 当前 DWA 参数快照

### 5.1 动态窗口与预测

```yaml
dwa:
  max_velocity: 1.0
  min_velocity: 0.0
  max_angular_velocity: 1.0
  max_acceleration: 0.5
  max_angular_acceleration: 5.0
  prediction_time: 3.0
  simulation_time_step: 0.1
  velocity_resolution: 0.1
  angular_resolution: 0.05
  minimum_turning_velocity: 0.001
  minimum_turning_radius: 0.1
```

动态窗口使用真实控制回调间隔 `control_dt`，并将其限制在 `0.01～0.25 s`：

```text
v ∈ [v_current-a_max*control_dt, v_current+a_max*control_dt]
w ∈ [w_current-alpha_max*control_dt, w_current+alpha_max*control_dt]
```

例如回调频率约为 40 Hz、当前速度为零时：

```text
control_dt ≈ 0.025 s
v ∈ [0, 0.0125] m/s
w ∈ [-0.125, +0.125] rad/s
```

在 `minimum_turning_radius=0.1 m` 下，最大起步速度对应：

```text
|w| <= v/R = 0.0125/0.1 = 0.125 rad/s
```

因此半径约束已允许使用整个起步角速度窗口，但首帧预测轨迹仍只会产生很小的前进和横向位移。

### 5.2 当前评分权重

```yaml
dwa:
  weight_heading: 1.0
  weight_obstacle: 3.5
  weight_velocity: 4.0
  weight_road: 1.0
  weight_smooth: 0.02
```

评分函数没有在 v0.3.2 中修改：

```text
score =
    weight_heading  * cos(endpoint_yaw-road_yaw_error)
  - weight_obstacle * (1/minimum_clearance)
  - weight_velocity * (max_velocity-v)
  - weight_road     * mean_road_offset
  - weight_smooth   * normalized_motion_change
```

碰撞、越过道路边界或违反动态约束的候选直接无效。

### 5.3 当前平滑参数

```yaml
dwa:
  smooth:
    weight_delta_v: 1.5
    weight_delta_w: 2.0
    weight_acceleration: 0.8
    weight_angular_acceleration: 1.0
    weight_jerk: 0.0
    weight_angular_jerk: 0.0
    max_jerk: 2.0
    max_angular_jerk: 3.0
```

`max_jerk` 和 `max_angular_jerk` 仍保留在代码中，但对应权重为零，因此当前不影响候选得分。
速度差、角速度差、线加速度和角加速度仍然参与平滑代价；角加速度还同时是硬动态窗口约束。

### 5.4 当前激活与点云范围

```yaml
dwa:
  activation:
    max_x: 5.0
    min_y: -1.0
    max_y: 1.0
    minimum_points: 20
    clear_frames: 5

  obstacle_distance_threshold: 5.0
  obstacle_roi:
    min_x: 0.3
    max_x: 5.0
    min_y: -1.5
    max_y: 1.5
    min_z: -0.2
    max_z: 1.0
```

扩大到 5 m 后可能提前激活 DWA，但也会引入更多远处点和深度噪声。`max_obstacle_points` 仍为 2500，
因此必须通过 `/dwa/obstacle_cloud` 确认远处密集点没有挤占近处有效障碍物的采样预算。

## 6. 数据集回放结论

多次对同一段约 140 帧数据进行回放，观察到以下稳定现象：

1. 开头局部激活区域内通常有约 98～138 个障碍点，DWA 能正常激活；
2. 障碍仍存在时，最优结果多次为 `v=0,w_ref=0,w_track=0`；
3. 同一阶段多次出现 `Safety stop: emergency obstacle distance`，这些帧在进入 DWA 前就被安全层截断；
4. 已记录的 DWA 外廓净空主要约为 `0.20～0.35 m`；
5. DWA 恢复运动时常已出现 `obstacles=0`，此时日志中的 `clearance=5.00` 是无障碍默认上限，不是实测到障碍物距离为 5 m；
6. 一次恢复轨迹为 `v=0.07,w_ref=-0.11`，对应半径约 `0.64 m`，远大于 `0.1 m`，说明被选中的轨迹并未触发最小半径边界；
7. 道路右边界距离和航向仍有明显帧间波动，CRUISE 阶段可出现 LQR 角速度饱和。

因此，从 `0.2 m` 放宽到 `0.1 m` 只增加了可选的急弯候选，但当前评分没有在障碍存在时选择这些候选。
扩大到 5 m 也无法解决“起步时障碍已经很近”的问题，因为此时 DWA 原本就已激活。

## 7. 为什么当前仍可能认为停车最优

取一帧实际日志：

```text
right_distance = 2.29 m
road_yaw_error = -0.060 rad
minimum_clearance = 0.26 m
selected v = 0
selected w = 0
logged score = -17.97
```

停车轨迹评分近似为：

```text
heading = cos(0.060)                         ≈ +0.998
obstacle = -3.5 * (1/0.26)                  ≈ -13.46
velocity = -4.0 * (1.0-0.0)                 = -4.00
road = -1.0 * |0-(0.8-2.29)|                = -1.49

score ≈ 0.998 - 13.46 - 4.00 - 1.49
      ≈ -17.95
```

该结果与日志 `-17.97` 基本一致。近障起步时，非零候选只能取得很小的速度收益，却可能让最小净空进一步下降，
障碍物倒数代价会迅速增大。因此即使存在低速弧线，停车仍可能得分最高。

## 8. v0.3.2 风险与注意事项

### 8.1 最大角加速度加倍

`5.0 rad/s²` 约等于 `286.5°/s²`。在约 40 Hz 控制周期下，每次回调允许物理角速度变化约：

```text
5.0 * 0.025 = 0.125 rad/s ≈ 7.16°/s
```

这有利于建立转向，但可能增加底盘冲击和乘员不适，必须空载低速验证。

### 8.2 最小转弯半径减小

`0.1 m` 是算法允许值，不等同于已经验证的轮椅安全扫掠半径。当前碰撞模型是圆形，不能完整描述脚踏板、乘员腿部、
扶手和旋转时的真实外廓。硬件能原地旋转也不代表近障时原地旋转一定安全。

### 8.3 jerk 权重关闭

关闭 jerk 评分有助于判断其是否导致近障起步过于保守，但会降低对加速度变化突兀程度的抑制。
这不影响紧急停车，也不取消角加速度硬约束，但不建议未经实车舒适性测试长期作为载人参数。

### 8.4 5 m 点云范围

更远点云会增加 DWA 输入的不确定性。树木、路缘、深度飞点或地面残留可能提前激活规划器。
应同时观察完整点云和 `/dwa/obstacle_cloud`，不能只根据 RViz 中的蓝线判断规划正确。

### 8.5 安全急停和 DWA 使用不同判据

安全层的 `min_distance` 来自融合模块的椭圆危险区域，DWA 使用自己的 ROI 和圆形碰撞模型。
少量椭圆区近点可能触发安全停车，但不足 20 个 DWA 激活点。当前日志已经出现 CRUISE 阶段障碍点很少但仍触发 Safety stop 的现象。

## 9. 已讨论但尚未实现的改进

以下内容不属于当前 v0.3.2 代码，本文只保存设计方向：

### 9.1 净空改善奖励

计划为轨迹记录：

```text
initial_clearance
terminal_clearance
clearance_gain = terminal_clearance - initial_clearance
```

并在原评分中增加：

```text
+ weight_clearance_gain * clearance_gain
```

其目的是区分“始终停车保持当前净空”和“起步后逐渐远离障碍物”的轨迹。

### 9.2 安全条件下的停车惩罚

计划先生成全部候选，再判断是否存在满足下列条件的运动轨迹：

```text
无碰撞
没有越过道路边界
满足动态约束
最小净空高于阈值
末端没有继续接近障碍物
```

只有存在上述轨迹时才对停车候选扣分；如果没有安全运动方案，停车仍保留最高安全优先级。

以上两项预计封装为 `DWAPlanner` 私有函数，并复用现有轨迹—障碍物遍历，不增加第二次点云距离循环。
候选后处理只增加一次 `O(candidate_count)` 扫描，相比现有
`O(candidate_count * pose_count * obstacle_count)` 碰撞计算，额外耗时预计很小。

## 10. 编译与验证

当前环境的 colcon 缺少 `--packages-select` 扩展，推荐按路径构建：

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash
colcon build --paths src/wheel_perception --symlink-install
source install/setup.bash
```

v0.3.2 参数调整后已完成上述构建，`wheel_perception` 构建成功。

DWA 单元测试：

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.bash
./build/wheel_perception/test_dwa_planner
```

结果：

```text
11 tests from DWAPlanner
11 passed
```

测试覆盖无障碍前进、道路边界、完全阻塞、非法里程计状态、禁止原地转向、低速滚动转弯、最小转弯半径、
轨迹保持和加速度限制。现有测试尚未覆盖计划中的净空改善奖励和条件停车惩罚。

## 11. 数据集回放

确认：

```yaml
zed:
  use_dataset_mode: true
```

启动指定数据集：

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch wheel_perception dwa_dataset_test.launch.py \
  dataset_path:=/home/x/tami/你的数据集目录 \
  fps:=10.0 \
  loop:=false
```

默认数据集路径为：

```text
~/tami/shengwudao2
```

如果该目录存在，也可以直接运行：

```bash
ros2 launch wheel_perception dwa_dataset_test.launch.py
```

建议完整分析时使用 `loop:=false`，避免循环边界把末帧到首帧的突变混入控制数据。

## 12. RViz 可视化

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2 -d install/wheel_perception/share/wheel_perception/rviz/dwa_navigation.rviz
```

固定坐标系设置为：

```text
base_link
```

重点观察：

| 话题 | 用途 |
|---|---|
| `/dwa/obstacle_cloud` | 真正进入 DWA 的二维障碍点 |
| `/perception/debug/cloud_ellipse` | 可能触发紧急停车的椭圆危险区点 |
| `/dwa/best_path_marker` | 绿色巡线、蓝色避障路径或蓝色停车点 |
| `/dwa/candidate_paths` | 有效和无效候选轨迹 |
| `/perception/debug/right_road_edge` | 道路右边界拟合点 |
| `/debug/viz` | 语义分割和道路调试图像 |

如果终端显示：

```text
Safety stop: emergency obstacle distance
```

说明该帧在进入 DWA 之前已经由安全层停车。此时调节 DWA 评分或最小转弯半径不会改变该帧输出。

## 13. 实车测试前检查

实车模式需要将：

```yaml
zed:
  use_dataset_mode: false
```

然后重新启动节点。实车前至少确认：

1. 相机坐标系和 `base_link` 方向正确；
2. `/dwa/obstacle_cloud` 不包含树冠、地面和轮椅自身结构；
3. `/perception/debug/cloud_ellipse` 中没有持续近距离飞点；
4. `robot_radius=0.45 m` 能覆盖轮椅、脚踏板和乘员；
5. `emergency_stop_distance=0.5 m` 满足当前速度下的实测制动距离；
6. `/odom.twist` 是否真实可用；不可用时控制器会使用上一输出估计动态窗口；
7. `0.1 m` 最小转弯半径和 `5.0 rad/s²` 角加速度已在空载、低速、开阔场地验证；
8. 蓝牙通信、人工急停和现场监护均已准备。

在上述项目完成前，不应进行载人自动避障测试。

## 14. 当前阶段结论

v0.3.2 成功保存了“扩大前向规划范围、提高角响应、允许更紧滚动弧线、暂时关闭 jerk 评分”的参数实验状态，
现有 C++ 逻辑和 11 项 DWA 单元测试保持稳定。

数据集结果同时说明：近障起步持续停车不是单一转弯半径参数造成的。安全层会在部分帧抢占控制；未触发急停时，
现有最小净空倒数代价也会使停车轨迹优于低速弧线。下一阶段若继续解决该问题，应优先实现带安全门槛的
“净空改善奖励 + 条件停车惩罚”，并保留紧急停车、碰撞和道路边界硬约束。
