# DWA + LQR v0.2：相对 v0.1 的改进、参数与运行说明

本文档说明 `wheel_latest` 项目从 `DWA + LQR v0.1` 演进到当前 `v0.2` 的具体变化。
重点不是重复 v0.1 已经完成的 DWA 接入，而是记录 v0.2 为解决下列实际问题所做的增量改进：

- 数据集回放中线速度、角速度频繁波动；
- DWA 候选轨迹在评分接近时来回切换；
- 低速阶段可能选择转向但不前进的候选；
- 树冠、高处点云和道路外点云干扰局部规划；
- 数据集历史里程计速度与当前回放控制指令不一致；
- 无障碍直路仍持续运行 DWA，造成不必要的路径变化；
- RViz 无法直接判断当前轨迹是道路巡线还是 DWA 避障结果。

v0.2 没有重构感知架构，也没有替换 LQR。ZED、TensorRT BiSeNet、ObstacleFusion、
道路边界、数据集播放器和原有 ROS YAML 参数体系均继续使用。

## 1. v0.1 与 v0.2 总体差异

| 项目 | v0.1 | v0.2 |
|---|---|---|
| DWA 运行方式 | 每个感知周期都运行 DWA | 仅当有效区域内出现足够障碍点时运行 |
| 无障碍巡线 | 仍由 DWA 在候选中选择 | 生成确定性道路巡线轨迹，再由 LQR 跟踪 |
| 平滑代价 | 只计算 `abs(w-w_previous)` | 同时计算速度差、角速度差、加速度、角加速度和 jerk |
| 候选切换 | 每帧直接选择最高分轨迹 | 上一轨迹安全且新轨迹优势不足时保持上一轨迹 |
| 原地转向 | 可能出现低线速度、大角速度候选 | 当前阶段禁用低速转向，停车候选只允许 `w=0` |
| 控制周期 | 使用固定 `simulation_time_step` 构造动态窗口 | 使用真实回调周期，限制在 `0.01~0.25 s` |
| 点云输入 | 从二维 `x,y` 点直接抽样 | 先按三维 ROI 过滤，再降采样和投影到二维规划平面 |
| 树冠点云 | 可能参与 DWA | 超出高度带的点仍可显示，但不参与 DWA |
| 数据集速度源 | 可能使用历史 `/zed/odom.twist` | 使用本次回放的 `last_output_velocity_` |
| 实车速度源 | `/odom` 和 `/zed/odom` 都可能进入控制器 | 实车模式只订阅真实 `/odom` 并使用其 `twist` |
| 模式退出 | 无模式概念 | 连续多帧无触发点后才退出 DWA，带迟滞 |
| RViz 最优路径 | 固定绿色 Path | 绿色=普通巡线，蓝色=DWA 最优避障路径 |
| 测试 | 4 个基本 DWA 测试 | 9 个测试，增加平滑、禁原地转向和轨迹保持检查 |

## 2. v0.2 当前控制链路

```text
ZED RGB-D / 历史数据集
        |
TensorRT BiSeNet + 点云融合 + 道路边界
        |
ObstacleFusion / perception/output / zed/point_cloud
        |
        +--> Safety Layer
        |      min_distance < emergency_stop_distance
        |      感知超时 / DWA 无安全路径
        |                     |
        |                   立即停车
        |
        +--> 有效点云 ROI + DWA 激活区域判断
                       |
             +---------+----------+
             |                    |
       未达到触发点数          达到触发点数
             |                    |
      道路巡线轨迹生成          DWA 候选预测
       CRUISE，绿色路径          DWA，蓝色最优路径
             |                    |
             +---------+----------+
                       |
                 LQR 前视点跟踪
                       |
              角速度限幅/协议转换
                       |
                    cmd_vel
```

安全层的优先级没有因为按需 DWA 而降低。紧急停车判断发生在 DWA 激活判断之前。

## 3. v0.2 增加了什么

### 3.1 按障碍物范围激活 DWA

控制器先统计进入激活区域的有效障碍点：

```text
0 < x <= dwa.activation.max_x
dwa.activation.min_y <= y <= dwa.activation.max_y
点数 >= dwa.activation.minimum_points
```

注意：参与统计的点已经先通过 `dwa.obstacle_roi` 三维过滤，因此实际前向最小距离还受到
`dwa.obstacle_roi.min_x` 限制。

当前 YAML 参数下，实际激活区域约为：

```text
0.3 m <= x <= 2.5 m
-0.9 m <= y <= 0.9 m
-0.2 m <= z <= 1.0 m
有效点数 >= 10
```

检测到足够点时立即进入 DWA。障碍消失后不会立刻退出，而是等待
`dwa.activation.clear_frames` 个连续无障碍帧，防止点云短暂丢失造成模式来回切换。

20 FPS 下当前 `clear_frames=5` 相当于约 `0.25 s`；30 FPS 下约为 `0.17 s`。

### 3.2 无障碍时的确定性道路巡线轨迹

未激活 DWA 时，不再对大量 `(v,w)` 候选进行评分，而是生成一条道路参考轨迹：

```text
target_offset = target_right_distance - right_distance
y(x) = target_offset + tan(road_yaw_error) * x
```

巡线目标线速度为 `dwa.cruise_velocity`，但每个控制周期仍受
`dwa.max_acceleration` 限制，不会从零瞬间跳到目标速度。

此时：

- 规划层参考角速度 `w_ref=0`；
- LQR 根据局部轨迹的横向误差和航向误差产生闭环转向；
- `/dwa/candidate_paths` 没有 DWA 候选，这是正常现象；
- `/dwa/best_path_marker` 显示绿色。

### 3.3 完整的运动平滑代价

v0.1 的平滑项只有角速度变化：

```text
smooth_cost_v0.1 = abs(w - w_previous)
```

v0.2 保存上一指令、上一线加速度和上一角加速度，并使用：

```text
delta_v = v - v_previous
delta_w = w - w_previous

a     = delta_v / dt
alpha = delta_w / dt

jerk_v = (a - a_previous) / dt
jerk_w = (alpha - alpha_previous) / dt
```

综合平滑代价为：

```text
smooth_cost =
    weight_delta_v            * normalized(abs(delta_v))
  + weight_delta_w            * normalized(abs(delta_w))
  + weight_acceleration       * normalized(abs(a))
  + weight_angular_acceleration * normalized(abs(alpha))
  + weight_jerk               * normalized(abs(jerk_v))
  + weight_angular_jerk       * normalized(abs(jerk_w))
```

各项按加速度能力、控制周期和 jerk 标度归一化，使参数不会因为回放帧率改变而完全失去意义。
jerk 只作为舒适性软代价，不会阻止紧急停车。

候选轨迹还增加 `dynamic_feasible` 标志。超过线加速度或角加速度限制的轨迹直接判为无效，
不会成为最优轨迹。

### 3.4 上一轨迹保持机制

v0.2 将上一条安全指令显式加入下一周期候选集合。当满足以下条件时保持上一轨迹：

1. 上一轨迹对应候选仍然无碰撞、未越界且满足动态约束；
2. 上一轨迹净空不小于 `trajectory_hold.minimum_clearance`；
3. 新最优轨迹的得分优势小于绝对和相对切换门槛；
4. 新轨迹没有带来足够明显的净空改善。

该机制用于减少两个相近候选在连续帧中交替成为最高分，从而降低左右摆动。若新轨迹明显更安全，
净空改善会绕过保持机制，允许及时切换。

### 3.5 禁止当前阶段的原地转向

当前轮椅阶段不考虑原地转向。v0.2 对候选施加：

```text
if v < minimum_turning_velocity and abs(w) > 0:
    reject candidate
```

仍保留 `(v=0,w=0)` 安全停车候选。为保证从静止状态可以沿直线起步，动态窗口跨过零角速度时会
强制加入 `w=0` 候选，避免采样步长恰好跳过零。

### 3.6 DWA 有效点云 ROI

v0.2 在 `ControllerNode` 中增加 DWA 专用三维点云过滤：

```text
dwa.obstacle_roi.min_x <= x <= dwa.obstacle_roi.max_x
dwa.obstacle_roi.min_y <= y <= dwa.obstacle_roi.max_y
dwa.obstacle_roi.min_z <= z <= dwa.obstacle_roi.max_z
```

处理顺序改为：

```text
/zed/point_cloud
    -> 三维 ROI 过滤
    -> 在有效点中均匀降采样
    -> 最多保留 max_obstacle_points
    -> 投影成 DWA 二维障碍点
```

先过滤再降采样很重要：树冠等大量无关点不会抢占有限的点云采样预算。完整调试点云仍保留在
`/zed/point_cloud`，实际进入规划器的二维投影发布到 `/dwa/obstacle_cloud`。

### 3.7 数据集模式与实车模式速度源隔离

v0.2 根据 `zed.use_dataset_mode` 建立互斥的数据源：

| 模式 | 订阅 | DWA 动态窗口速度 |
|---|---|---|
| 数据集 `true` | `/zed/odom` | `last_output_velocity_` |
| 实车 `false` | `/odom` | 真实 `/odom.twist`，不可用时才回退上一输出 |

数据集的 `/zed/odom` 位姿仍可用于状态和显示，但它的 `twist` 属于历史采集过程，不能代表
当前回放时控制器刚刚输出的速度。因此数据集模式不再把历史 `twist` 输入新的动态窗口。

这项改动不会改变实车控制路径：实车继续使用真实轮椅里程计速度，数据集和实车互不干扰。

### 3.8 使用真实控制周期

动态窗口和平滑代价使用相邻感知回调的真实时间差 `control_dt`。为避免系统调度暂停一次后产生
异常大的可达窗口，`control_dt` 被限制在 `0.01~0.25 s`。

`dwa.simulation_time_step` 仍用于一条未来轨迹内部的数值积分，不再错误地代替实际控制周期。

### 3.9 RViz 模式颜色与有效障碍点显示

新增话题：

| 话题 | 类型 | 含义 |
|---|---|---|
| `/dwa/obstacle_cloud` | `sensor_msgs/PointCloud2` | 实际进入 DWA 的 ROI 点云投影 |
| `/dwa/best_path_marker` | `visualization_msgs/Marker` | 能动态携带颜色的最终局部轨迹 |

路径颜色：

- 绿色：普通道路巡线，DWA 未参与；
- 蓝色：DWA 已激活，显示其最优避障轨迹；
- 半透明绿色候选：DWA 可行候选；
- 半透明红色候选：碰撞、越界或不满足动态约束的候选。

原 `/dwa/best_path` 和 `/dwa/local_trajectory` 仍保留，避免破坏已有订阅者。因为
`nav_msgs/Path` 本身不含颜色，动态颜色使用新 Marker 话题实现。安全停车时 Path 和 Marker
都会被清除。

### 3.10 测试覆盖增加

当前共有 9 个 DWA 单元测试，覆盖：

1. 空道路选择前进轨迹；
2. 空点云下产生稳定巡线结果；
3. 不跨越道路边界；
4. 道路被障碍墙封闭时拒绝规划；
5. 非法里程计状态时拒绝输出；
6. 不生成原地转向候选；
7. 真实加速度限制下从静止直线起步；
8. 得分接近时保持上一安全轨迹；
9. 相对历史指令满足线/角加速度限制。

## 4. v0.2 删除或停用了什么

### 4.1 删除“始终运行 DWA”行为

DWA 模块没有删除，但不再在无障碍直路上持续计算和切换最优候选。无障碍阶段由确定性道路轨迹
承担参考生成，DWA 只在障碍进入激活区域后工作。

### 4.2 删除原地转向候选

低于 `minimum_turning_velocity` 时的非零角速度候选被禁止。当前版本没有“原地左转”或
“原地右转”状态。

### 4.3 替换单一平滑项

v0.1 的 `abs(w-w_previous)` 单项平滑代价被完整运动历史代价替换。不是简单叠加一个更大的
`weight_smooth`，而是对每个动态量分别配置权重。

### 4.4 停止使用历史数据集速度作为当前控制反馈

数据集模式下 `/zed/odom.twist` 不再参与动态窗口速度计算，防止回放 FPS、历史车速和当前输出
互相矛盾。

### 4.5 删除旧回正时间参数

`logic.recover_time` 已从 YAML 和当前控制逻辑移除。旧的 `logic.pass_clearance` 仅为历史兼容保留，
当前 DWA 控制链不使用它。

### 4.6 从 DWA 输入中排除无关点云

高处树冠、过近相机噪声、规划距离外和横向范围外的点不会进入 DWA；这些点没有从原始点云显示
中删除，因此仍能在 `/zed/point_cloud` 中观察和调试。

## 5. 参数总表与调参影响

所有参数仍位于：

```text
src/wheel_perception/config/params.yaml
```

没有新增第二套参数文件或独立参数服务器。以下数值是当前工作区 YAML 的实际值；运行时 YAML
会覆盖 C++ 中的默认值。

### 5.1 DWA 速度、窗口与采样参数

| 参数 | 当前值 | 增大后的影响 | 减小后的影响 |
|---|---:|---|---|
| `max_velocity` | 1.0 m/s | DWA 可选择更高避障速度，制动距离和风险增加 | 更安全、更易调试，但通行效率下降 |
| `min_velocity` | 0.0 m/s | 若提高，会减少停车/低速候选，可能导致狭窄处无解 | 保留低速和停车能力；负值会允许倒车，当前不建议 |
| `max_angular_velocity` | 1.0 rad/s | 允许更急转弯，也更容易产生突兀转向 | 转向更温和，但可能绕不过近障碍 |
| `max_acceleration` | 0.5 m/s² | 加减速响应更快，舒适性下降 | 更平滑，但遇障碍时可达速度窗口变化更慢 |
| `max_angular_acceleration` | 1.5 rad/s² | 方向切换更快，左右摆动风险增加 | 抑制转向突变，但急弯响应变慢 |
| `prediction_time` | 2.5 s | 看得更远、更保守、计算量增加 | 计算更快，但可能过晚发现未来碰撞 |
| `simulation_time_step` | 0.1 s | 步长更粗、计算更快、碰撞检查可能漏过细节 | 轨迹和碰撞检查更细，计算量增加 |
| `velocity_resolution` | 0.1 m/s | 候选更少、速度跳档更明显、计算更快 | 速度选择更细、更平滑，但候选数量增加 |
| `angular_resolution` | 0.05 rad/s | 转向候选更少、角速度跳变更明显 | 转向选择更细，计算量增加 |

说明：无障碍巡线速度主要由 `cruise_velocity` 决定；`max_velocity` 是 DWA 激活阶段的全局上限。

### 5.2 DWA 评分权重

当前评分仍为：

```text
score = weight_heading  * heading_cost
      - weight_obstacle * obstacle_cost
      - weight_velocity * velocity_cost
      - weight_road     * road_cost
      - weight_smooth   * smooth_cost
```

| 参数 | 当前值 | 增大后的主要影响 | 过大风险 |
|---|---:|---|---|
| `weight_heading` | 2.0 | 更偏向道路方向 | 避障时不愿偏离道路朝向 |
| `weight_obstacle` | 3.0 | 更偏向大净空轨迹 | 过度绕行或更容易选择低速 |
| `weight_velocity` | 2.0 | 更倾向保持前进速度 | 可能压缩安全和舒适性的评分空间 |
| `weight_road` | 3.0 | 更靠近道路目标线 | 绕障横向空间不足 |
| `weight_smooth` | 0.08 | 放大所有平滑子项，减少命令变化 | 过大时可能长期保持低速或不愿转弯 |

v0.1 到 v0.2 的当前参数变化：

```text
angular_resolution: 0.10 -> 0.05
weight_velocity:     1.0 -> 2.0
weight_smooth:       2.0 -> 0.08
```

`weight_smooth` 数值变小是因为 v0.2 的子项已经归一化并且数量更多，不能与 v0.1 的单一
`abs(delta_w)` 数值直接比较。

### 5.3 平滑子项参数

| 参数 | 当前值 | 增大后的影响 |
|---|---:|---|
| `smooth.weight_delta_v` | 1.5 | 更不愿切换线速度，减少速度跳变 |
| `smooth.weight_delta_w` | 2.0 | 更不愿切换角速度，优先抑制左右摆动 |
| `smooth.weight_acceleration` | 0.8 | 更偏好较小线加速度 |
| `smooth.weight_angular_acceleration` | 1.0 | 更偏好较小角加速度 |
| `smooth.weight_jerk` | 1.2 | 加减速变化更柔和 |
| `smooth.weight_angular_jerk` | 1.5 | 转向加速度变化更柔和，减少突然反打 |
| `smooth.max_jerk` | 1.0 m/s³ | 归一化分母增大，相同线 jerk 的惩罚反而减小 |
| `smooth.max_angular_jerk` | 3.0 rad/s³ | 归一化分母增大，相同角 jerk 的惩罚反而减小 |

推荐调参顺序：先调 `weight_smooth` 总体强度，再重点微调 `weight_delta_w` 和
`weight_angular_jerk`。不要同时大幅修改所有子项，否则难以判断是哪一项改善或恶化了控制。

### 5.4 禁止原地转向参数

| 参数 | 当前值 | 影响 |
|---|---:|---|
| `minimum_turning_velocity` | 0.10 m/s | `v` 低于该值时只允许 `w=0`；提高会扩大禁止低速转向范围，降低会更早允许边走边转 |

若设置过高，轮椅在狭窄或刚起步阶段可能无法获得足够转向；若接近零，则会重新接近 v0.1 的
低速转向行为。当前阶段不建议设为零。

### 5.5 按需激活参数

| 参数 | 当前值 | 增大后的影响 | 减小后的影响 |
|---|---:|---|---|
| `cruise_velocity` | 0.6 m/s | 无障碍巡线更快 | 巡线更稳、更适合初次实车测试 |
| `activation.max_x` | 2.5 m | 更早激活 DWA，但远处噪声更易触发 | 更晚激活，留给避障的距离减少 |
| `activation.min_y` | -0.9 m | 数值向零增大时缩小右侧触发区域 | 更负时扩大右侧触发区域 |
| `activation.max_y` | 0.9 m | 扩大左侧触发区域 | 缩小左侧触发区域 |
| `activation.minimum_points` | 10 | 抗单点噪声更强，但小障碍可能漏触发 | 更敏感，但误触发概率提高 |
| `activation.clear_frames` | 5 | DWA 退出更慢、更稳定 | 恢复巡线更快，但模式可能抖动 |

`activation.minimum_points` 应结合点云密度调整。若小而真实的障碍只产生少量点，可逐步从 10
降低到 5 或 3；若树叶、深度飞点仍频繁触发，可适当提高，但不能以漏检障碍为代价。

### 5.6 上一轨迹保持参数

| 参数 | 当前值 | 增大后的影响 |
|---|---:|---|
| `trajectory_hold.enabled` | true | 布尔值；关闭后每帧直接取最高分候选 |
| `trajectory_hold.score_switch_margin` | 0.01 | 新轨迹需要更大的绝对得分优势，轨迹更稳定 |
| `trajectory_hold.relative_switch_margin` | 0.01 | 当前评分幅值越大时要求更明显的相对改善 |
| `trajectory_hold.minimum_clearance` | 0.30 m | 只有更安全的旧轨迹才能被保持，保持机制更谨慎 |
| `trajectory_hold.clearance_switch_margin` | 0.15 m | 新轨迹需要更大的净空改善才能立即切换，稳定性增加但避障响应可能变慢 |

如果出现最优路径左右切换，可小幅提高两个 score margin；如果绕障反应迟钝，应优先降低
`clearance_switch_margin` 或 score margin，而不是先提高最大角速度。

### 5.7 点云和碰撞参数

| 参数 | 当前值 | 增大后的影响 | 减小后的影响 |
|---|---:|---|---|
| `obstacle_distance_threshold` | 2.5 m | 评价更远障碍，计算量和保守程度增加 | 只考虑近障碍，可能反应过晚 |
| `robot_radius` | 0.45 m | 碰撞外廓更保守、安全间隙更大 | 可通过更窄间隙，但安全裕量降低 |
| `road_margin` | 0.15 m | 离道路边界更远，可行空间缩小 | 可行空间扩大，但更贴边 |
| `max_obstacle_points` | 2500 | 点云更密、碰撞检查更完整、CPU 负载增加 | 计算更快，但可能丢掉小障碍 |
| `obstacle_roi.min_x` | 0.3 m | 忽略更大的近端区域 | 纳入更近点，可能包含车体/相机噪声 |
| `obstacle_roi.max_x` | 2.5 m | 纳入更远点 | 减少计算但缩短观察距离 |
| `obstacle_roi.min_y` | -1.5 m | 向零增大时缩小右侧范围 | 更负时扩大右侧范围 |
| `obstacle_roi.max_y` | 1.5 m | 扩大左侧范围 | 缩小左侧范围 |
| `obstacle_roi.min_z` | -0.2 m | 提高会过滤更多低点/地面点 | 降低会纳入地面噪声 |
| `obstacle_roi.max_z` | 1.0 m | 纳入更高障碍，也可能重新受树冠影响 | 更彻底过滤高处点，但可能漏掉会碰到乘员的障碍 |

高度阈值必须依据 ZED 坐标系和相机安装高度实车标定。轮椅是载人设备，不能仅为了让轨迹更直
就降低碰撞外廓或大量过滤点云。

### 5.8 可视化参数

| 参数 | 当前值 | 影响 |
|---|---:|---|
| `visualization.max_candidates` | 80 | 增大会显示更多候选但提高 RViz/DDS 负载；减小只影响显示，不改变 DWA 实际规划 |

### 5.9 Safety Layer 参数

| 参数 | 当前值 | 影响 |
|---|---:|---|
| `safety.emergency_stop_distance` | 0.8 m | 最近融合障碍距离小于该值立即停车，优先于 DWA/LQR |
| `safety.perception_timeout` | 0.5 s | 超过该时间没有感知输出即停车 |
| `safety.stop_on_no_path` | true | DWA 激活但无无碰撞道路内轨迹时停车 |

紧急停车使用 `PerceptionOutput.min_distance`，不依赖 DWA 激活点数。因此即使
`activation.minimum_points` 设置较高，进入紧急距离的融合障碍仍会优先触发停车。

### 5.10 LQR 参数状态

v0.2 的主要调参集中在 DWA，本轮没有调整 LQR 算法结构或参数。当前 YAML 实际值为：

| 参数 | 当前值 | 作用 |
|---|---:|---|
| `lqr.gain` | 80.0 | 将物理角速度控制量转换为底盘协议转向量 |
| `lqr.q_pos` | 10.0 | 横向误差权重 |
| `lqr.q_ang` | 10.0 | 航向误差权重 |
| `lqr.q_integral` | 0.2 | 横向稳态误差积分权重 |
| `lqr.integral_limit` | 1.5 | 积分限幅 |
| `lqr.k_w` | 10.0 | LQR 模型输入矩阵增益 |
| `lqr.model_v` | 0.5 | LQR 内部模型速度 |
| `lqr.aim_dist` | 0.8 m | 右边界目标距离回退值 |
| `lqr.lookahead_time` | 0.6 s | 在局部轨迹上选择前视点的时间 |

注意：参数文件中 `lqr.gain` 上方注释仍提到 60，但 ROS 实际加载值是 `80.0`，运行判断应以
键值为准。`/cmd_vel.angular.z` 是底盘协议量，不应直接作为 rad/s 解读；物理参考角速度查看
`/dwa/planner_cmd.angular.z`。

## 6. v0.2 修改文件清单

| 文件 | 相对 v0.1 的作用 |
|---|---|
| `dwa_controller/include/.../DWAPlanner.hpp` | 增加 `MotionHistory`、动态可行性和平滑/轨迹保持配置 |
| `dwa_controller/src/DWAPlanner.cpp` | 真实周期动态窗口、完整平滑代价、轨迹迟滞、禁原地转向 |
| `src/controller_node.cpp` | 按需 DWA、确定性巡线、点云 ROI、速度源隔离、动态颜色 Marker |
| `config/params.yaml` | 增加 `smooth`、`activation`、`trajectory_hold`、`obstacle_roi` 参数组 |
| `rviz/dwa_navigation.rviz` | 增加有效点云，改用绿/蓝动态最优路径 Marker |
| `test/test_dwa_planner.cpp` | 从 4 个测试扩展到 9 个测试 |

本次没有新增独立参数系统，没有删除 DWA/LQR 模块，也没有修改数据集格式。

## 7. 编译方法

### 7.1 当前数据集工作站

当前机器的 colcon 不支持标准的 `--packages-select`，使用 `--paths`：

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash

colcon build --paths src/wheel_msgs src/wheel_perception \
  --symlink-install \
  --cmake-args '-DWHEEL_CUDA_ARCHITECTURES=75;86'

source install/setup.sh
```

只有 CUDA 架构、TensorRT 路径或 CMake 检测结果异常时才需要增加 `--cmake-clean-cache`。
GTX 1660 SUPER 需要包含 `SM 75`。

验证：

```bash
ctest --test-dir build/wheel_perception --output-on-failure
```

当前结果应为 9 个 DWA 测试全部通过。

如果 `cv_bridge` 的 symlink 构建目录残留导致全工程构建失败，优先使用上面的 `--paths` 只构建
本项目包，不要删除不确定的系统目录。

## 8. 数据集回放运行方法

确认 `params.yaml`：

```yaml
zed:
  use_dataset_mode: true
```

推荐只播放一次，以便记录完整且不跨越数据集首尾的控制数据：

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch wheel_perception dwa_dataset_test.launch.py \
  dataset_path:=/home/x/tami/shengwudao2 \
  fps:=20.0 \
  loop:=false
```

该 launch 会先启动感知和控制组件，约 8 秒后自动启动播放器，不需要另开终端手动运行
`zed_dataset_player.py`。

`loop:=true` 适合长时间观察 RViz，但不适合一次性控制曲线统计。循环首尾会造成场景、时间和
里程计状态突然跳变，使速度曲线出现不属于真实连续行驶的变化。

常用检查命令：

```bash
ros2 topic hz /debug/viz
ros2 topic echo /perception/output --once
ros2 topic echo /dwa/planner_cmd
ros2 topic echo /dwa/best_path_marker --once
ros2 topic echo /cmd_vel
```

如果自定义消息类型无效，先在当前终端重新执行：

```bash
source ~/tami/wheel_latest/install/setup.sh
```

## 9. RViz 与 rqt 可视化

数据集 launch 运行后，另开一个终端：

```bash
source /opt/ros/humble/setup.bash
source ~/tami/wheel_latest/install/setup.bash

rviz2 -d ~/tami/wheel_latest/src/wheel_perception/rviz/dwa_navigation.rviz
```

RViz 主要显示项：

| 显示项 | 话题 | 判断方法 |
|---|---|---|
| `ObstacleFusion Cloud` | `/zed/point_cloud` | 完整融合调试点云，可能包含树冠 |
| `DWA Effective Obstacles` | `/dwa/obstacle_cloud` | 真正参与 DWA 的 ROI 点云 |
| `Selected Path (Green=Cruise, Blue=DWA)` | `/dwa/best_path_marker` | 绿色巡线、蓝色 DWA 避障 |
| `DWA Candidates` | `/dwa/candidate_paths` | DWA 激活时出现候选轨迹 |

若仍看到固定绿色旧路径，请删除手动添加的 `/dwa/best_path` Path 显示项，使用
`/dwa/best_path_marker` Marker。原 Path 话题没有动态颜色字段。

rqt 查看语义分割和边界：

```bash
rqt
```

选择 `Plugins -> Visualization -> Image View`，话题选择 `/debug/viz`。

## 10. 实车运行方法

### 10.1 切换实车模式

修改：

```yaml
zed:
  use_dataset_mode: false
```

重新构建或确认 symlink 安装已正确指向源参数，然后 source：

```bash
cd ~/tami/wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheel_perception run_launch.py
```

先不要启动底盘桥，检查：

```bash
ros2 topic hz /perception/output
ros2 topic hz /odom
ros2 topic echo /dwa/planner_cmd
ros2 topic echo /cmd_vel
```

确认道路、有效点云、绿/蓝轨迹、急停和无路径停车都符合预期后，再单独启动硬件桥：

```bash
source /opt/ros/humble/setup.bash
source ~/tami/wheel_latest/install/setup.sh
ros2 run wheel_perception sub.py
```

### 10.2 首次实车建议

载人轮椅第一次验证应空载低速进行。建议临时设置：

```yaml
dwa:
  max_velocity: 0.3
  cruise_velocity: 0.2
safety:
  emergency_stop_distance: 1.0
```

依次测试：

1. 无障碍直道绿色巡线；
2. 障碍进入激活区域后绿色切换为蓝色；
3. 障碍离开后经过 `clear_frames` 恢复绿色；
4. 单侧障碍绕行；
5. 道路完全封闭时停车；
6. 感知话题中断时停车；
7. 独立物理急停和人工接管。

软件 Safety Layer 不能替代物理急停。提高速度前必须用实车实际制动距离重新标定
`emergency_stop_distance`、`robot_radius` 和点云高度范围。

## 11. 调参推荐流程

为了避免参数互相掩盖，推荐按以下顺序：

1. 固定低速，标定 `obstacle_roi`，确保 `/dwa/obstacle_cloud` 只包含会碰撞轮椅/乘员的点；
2. 调 `activation`，确认真实障碍稳定触发且噪声不触发；
3. 标定 `robot_radius`、`road_margin` 和 `emergency_stop_distance`；
4. 调 `weight_obstacle` 与 `weight_road`，先保证避障和道路内可行；
5. 调 `weight_velocity`，解决过慢或长期选静止轨迹；
6. 调 `weight_smooth`、`weight_delta_w` 和 `weight_angular_jerk`，降低左右摆动；
7. 最后调 `trajectory_hold` 门槛，减少评分接近时的候选切换；
8. 所有低速测试通过后，再逐步增加 `cruise_velocity` 和 `max_velocity`。

每次只修改一组参数，并使用同一段 `loop:=false` 数据集或同一实车路线对比 `v,w` 曲线。

## 12. 已知边界

- CRUISE 模式不做完整 DWA 候选预测，但 Safety Layer 仍持续有效；
- DWA 激活判断依赖过滤、降采样后的点数，`minimum_points` 必须与点云密度一起标定；
- 高度 ROI 过滤树冠的同时，也可能过滤悬空但会碰到乘员的障碍，必须实车校准；
- 蓝色路径表示 DWA 已参与，不代表一定可以继续通行；无有效路径时系统会清除轨迹并停车；
- `/cmd_vel.angular.z` 是现有底盘协议转向量，分析物理角速度应优先使用
  `/dwa/planner_cmd.angular.z` 或按实际协议增益换算；
- 数据集回放可以验证感知、规划和控制输出一致性，但不能替代底盘动力学、制动距离和乘坐舒适性测试。

## 13. v0.2 结果总结

v0.2 将 v0.1 的“始终运行 DWA”改成更适合当前轮椅巡线场景的双模式结构：无障碍时使用稳定、
确定的道路巡线参考；障碍进入有效区域后才启用 DWA 做未来轨迹安全预测。点云 ROI、完整动态
平滑代价、轨迹保持和禁止原地转向共同降低无关点云与候选切换造成的控制波动；数据集/实车速度
源隔离保证离线调试不会改变实车反馈路径；绿/蓝 Marker 让当前控制模式可以在 RViz 中直接确认。

