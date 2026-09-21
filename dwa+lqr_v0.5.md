# DWA + LQR v0.5：ZED WORLD 差分速度与实车标定报告

> 文档日期：2026-09-21  
> 当前分支：`feature/dwa_speed`  
> 对比基线：`dwa+lqr_v0.4.md` / v0.4 代码  
> 对应工作区：`~/tami/tami_wheel_latest`

## 1. v0.5 的定位

v0.4 已经完成道路约束 DWA、LQR 局部轨迹跟踪、分级安全层、RViz 可视化、标准 SI `/cmd_vel` 和独立 BLE 底盘桥。v0.5 不改变这套导航结构，重点修复实车模式下 DWA 当前速度不可靠的问题：

```text
v0.4 初始方案
ZED CAMERA 相对位姿累加 / SDK Pose.twist
                    |
                    v
              /odom.twist
                    |
                    v
                  DWA

v0.5 当前方案
ZED WORLD 绝对位姿 + ZED 图像时间戳
                    |
        相机外参转换到轮椅旋转中心
                    |
            连续 WORLD 位姿差分
                    |
           有效周期检查 + EMA
                    |
              /odom.twist
                    |
                  DWA

SDK Pose.twist ----------------> /zed/diagnostics/sdk_twist
                                  仅诊断，不参与控制
```

数据集模式仍使用上一控制输出 `last_output_velocity_` 构造动态窗口，不使用历史数据集的 `/odom.twist`。实车和数据集速度状态继续保持隔离。

## 2. 相比 v0.4 增加和修改的内容

| 项目 | v0.4 | v0.5 |
|---|---|---|
| ZED 位姿来源 | CAMERA 相对运动并在程序中累加 | `REFERENCE_FRAME::WORLD` 绝对位姿 |
| DWA 实车速度 | 直接依赖 SDK Twist，绝对尺度未验证 | 连续 WORLD 位姿按图像时间戳差分 |
| 相机杆臂处理 | 对 CAMERA motion/twist 做转换 | 先求 `WORLD_T_base`，再对轮椅中心位姿差分 |
| SDK Twist | 进入 `/odom.twist` | 只发布 `/zed/diagnostics/sdk_twist` |
| 时间基准 | SDK Twist 内部时间尺度不明确 | 同一 ZED grab 的 IMAGE timestamp |
| 异常周期 | 缺少专用差分周期保护 | `velocity_min_dt` / `velocity_max_dt` |
| 速度协方差 | 沿用 SDK Twist covariance | WORLD 差分使用独立配置标准差 |
| 相机前向外参 | `translation_x=0.20 m` | 经原地转向数据标定为 `0.45 m` |
| 自动测试 | DWA 测试 | 新增 WORLD 直行、杆臂、异常周期测试 |
| 实车证据 | SDK Twist 数值放大，尚未解决 | 三次 bag 验证档位、静止漂移、杆臂和 SDK 比例 |

v0.5 没有改变：

- TensorRT BiSeNet 推理和 CUDA 点云融合；
- 道路边界提取；
- DWA 评价函数；
- LQR 参数和跟踪结构；
- 紧急停车优先级；
- BLE GATT 协议映射；
- 数据集回放方式。

## 3. WORLD 位姿差分速度原理

### 3.1 获取绝对相机位姿

ZED 驱动在每次成功 `grab()` 后读取：

```cpp
getPosition(world_camera_pose, sl::REFERENCE_FRAME::WORLD)
```

帧时间戳使用：

```cpp
zed.getTimestamp(sl::TIME_REFERENCE::IMAGE)
```

因此速度分母来自真正产生该位姿的图像时间，而不是ROS回调到达时间、配置FPS或TensorRT处理时间。

### 3.2 从相机位姿恢复轮椅中心位姿

定义：

```text
W_T_C：相机在WORLD中的绝对位姿
B_T_C：相机在base_link中的固定外参
W_T_B：轮椅旋转中心在WORLD中的绝对位姿
```

计算：

```text
W_T_B = W_T_C * inverse(B_T_C)
```

该步骤在差分前去除相机绕轮椅旋转中心运动产生的杆臂速度。

### 3.3 连续位姿差分

```text
B(k-1)_T_B(k) = inverse(W_T_B(k-1)) * W_T_B(k)

v = translation(B(k-1)_T_B(k)) / dt
w = Log(rotation(B(k-1)_T_B(k))) / dt
```

相对变换表达在上一时刻 `base_link` 中，符合 `nav_msgs/Odometry.twist` 的车体速度语义。ControllerNode读取：

```text
linear.x  -> DWA当前线速度
angular.z -> DWA当前角速度
```

### 3.4 异常处理

- 第一帧只能建立差分基准，不发布伪速度；
- ZED tracking不是 `OK` 时停止发布并清除速度历史；
- 时间戳重复、倒退时拒绝该次差分；
- `dt` 超出配置范围时拒绝该次速度，并用当前位姿重新建立下一次差分基准；
- 跟踪恢复后需要两个连续有效位姿才重新发布速度；
- 速度结果继续经过原有EMA。

这些保护不会阻止真正的安全停车，也不会让SDK Twist重新进入控制链。

## 4. 新增模块和接口

### 4.1 WORLD速度估计器

新增：

```text
src/wheel_perception/include/wheel_perception/core/world_pose_velocity_estimator.hpp
src/wheel_perception/src/core/world_pose_velocity_estimator.cpp
```

`WorldPoseVelocityEstimator` 独立封装：

- 外参消除；
- 本地odom原点建立；
- SE(3)连续位姿差分；
- 图像时间戳周期检查；
- 跟踪中断后的速度历史重置。

FusionNode只负责ROS参数、消息构造、EMA和发布，便于以后独立调试估计器。

### 4.2 SDK Twist诊断话题

新增：

```text
/zed/diagnostics/sdk_twist
```

类型：

```text
geometry_msgs/msg/TwistStamped
```

该话题已转换到 `base_link`，但ControllerNode不订阅它。它只能用于和 `/odom.twist` 对照，不参与DWA或底盘控制。

### 4.3 单元测试

新增：

```text
src/wheel_perception/test/test_world_pose_velocity_estimator.cpp
```

覆盖：

1. 使用图像时间戳恢复直行速度；
2. 原地转向时消除相机杆臂线速度；
3. 拒绝超时采样间隔，并从新基准恢复。

当前测试结果：

```text
test_dwa_planner                      Passed
test_world_pose_velocity_estimator    Passed
100% tests passed
```

## 5. v0.5 参数

### 5.1 ZED速度参数

```yaml
zed:
  odometry:
    enabled: true
    odom_frame: odom
    base_frame: base_link
    velocity_filter_alpha: 0.25
    velocity_min_dt: 0.005
    velocity_max_dt: 0.2
    linear_velocity_stddev: 0.20
    angular_velocity_stddev: 0.25
    publish_sdk_twist_diagnostic: true
```

| 参数 | 作用 | 注意事项 |
|---|---|---|
| `velocity_filter_alpha` | WORLD差分速度EMA中新样本权重 | 越小越平滑，但停止和加速响应越慢 |
| `velocity_min_dt` | 最小有效图像周期 | 防止过小分母放大噪声 |
| `velocity_max_dt` | 最大有效图像周期 | 卡帧/跟踪中断后不对长间隔直接求速度 |
| `linear_velocity_stddev` | `/odom.twist`线速度协方差参数 | 不改变速度数值 |
| `angular_velocity_stddev` | `/odom.twist`角速度协方差参数 | 不改变速度数值 |
| `publish_sdk_twist_diagnostic` | 发布SDK诊断量 | 关闭后不影响WORLD控制速度 |

### 5.2 当前相机外参

```yaml
extrinsic:
  translation_x: 0.45
  translation_y: -0.2
  translation_z: 0.0
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

坐标定义：X向前、Y向左、Z向上。`translation_x=0.45` 表示ZED光学中心位于轮椅差速旋转中心前方约0.45米；旋转中心通常是驱动轮轴线中点，不是座椅外壳几何中心。

## 6. 三次bag验证结论

测试bag保存在本地 `bags/`，该目录已被 `.gitignore` 排除，不提交Git。

### 6.1 第一次：速度档位和静止漂移

约103.85秒，`/odom` 1703条，平均约16.4 Hz。

| 阶段 | WORLD速度中位数 |
|---|---:|
| 静止 | 接近 `0 m/s` |
| 一档稳定直行 | `0.477 m/s` |
| 三档稳定直行 | `0.879 m/s` |
| 二档稳定直行 | `0.658 m/s` |

静止段典型标准差：

```text
linear.x ≈ 0.0012 m/s
angular.z ≈ 0.0015 rad/s
```

WORLD差分已经能够反映真实档位，且没有旧SDK Twist的数倍放大。

### 6.2 第二次：相机前向外参从0.20改为0.45米

原地转向横向速度对比：

| 转向方向 | `x=0.20` 的 `linear.y` | `x=0.45` 的 `linear.y` |
|---|---:|---:|
| 正向 | `+0.202 m/s` | `-0.003 m/s` |
| 负向 | `-0.225 m/s` | `+0.001 m/s` |

原配置的残余等效杆臂约0.25米；修改为0.45米后残差下降到毫米至厘米级，改善超过95%。因此当前保留 `translation_x=0.45`。

`translation_y=-0.2` 尚未发现同样明确的系统误差，暂不修改。负向转向偶尔存在小幅前向爬行，更可能与底盘、摇杆输入和地面摩擦有关。

### 6.3 第三次：WORLD差分与SDK Twist直接对比

约114.66秒：

```text
/odom                         1957条
/zed/diagnostics/sdk_twist    1960条
/perception/output            1959条
/cmd_vel                      1836条
```

总体对比：

| 项目 | WORLD/SDK中位比例 | 相关系数 |
|---|---:|---:|
| 线速度 | `0.299` | `0.958` |
| 角速度 | `0.292` | `0.985` |

典型直行：

```text
WORLD linear.x = 0.472 m/s
SDK   linear.x = 1.573 m/s
```

典型正向原地转向：

```text
WORLD angular.z = 0.877 rad/s
SDK   angular.z = 3.048 rad/s
```

SDK趋势正确，但绝对值约放大3.3~3.6倍。完整链路实际约17 Hz，而ZED配置为60 FPS，`17/60≈0.283`，与WORLD/SDK比例高度吻合。因此SDK Twist不能重新作为控制速度。

### 6.4 静止漂移

第三次bag初始静止段：

```text
linear.x中位数 ≈ 0.00011 m/s
linear.x标准差 ≈ 0.00101 m/s
angular.z中位数 ≈ -0.00003 rad/s
angular.z标准差 ≈ 0.00285 rad/s
```

停止后最初约1秒可能保留 `0.004~0.005 m/s` 的EMA尾迹，随后回到约 `0.001 m/s` 内。这是 `velocity_filter_alpha=0.25` 的平滑延迟。

## 7. `/cmd_vel` 为什么仍有台阶

v0.5的 `/cmd_vel` 是标准浮点物理速度，不是旧 `sub.py` 的固定协议档位。第三次bag中，保留到0.001后：

```text
linear.x 有241种不同数值
angular.z有314种不同数值
```

但曲线仍呈台阶，原因是：

1. 控制回调实际约17 Hz，两条消息之间保持上一值；
2. DWA从离散候选中选择，当前 `velocity_resolution=0.1`、`angular_resolution=0.05`；
3. 17 Hz下单周期线速度动态窗口宽度约 `2*0.5*0.059=0.059 m/s`，小于 `0.1 m/s` 分辨率，常只剩窗口上下界和上一命令；
4. BLE前进死区要求从0直接跨到 `minimum_moving_velocity=0.15 m/s`；
5. 不同硬件速度区间把角速度限制在 `0.3/0.6/0.9 rad/s`；
6. LQR连续修正角速度，但最终线速度仍直接采用DWA/CRUISE结果。

因此当前是“连续数值、离散时间、离散候选”，不是“无限频率连续曲线”。可以后续试验：

```yaml
velocity_resolution: 0.02
angular_resolution: 0.025
```

但该调整尚未写入当前配置，会增加候选数量和CPU计算量，必须重新测量控制周期。

## 8. 构建、启动与测试

### 8.1 构建

```bash
cd ~/tami/tami_wheel_latest
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.sh
```

测试：

```bash
ctest --test-dir build/wheel_perception --output-on-failure
```

### 8.2 数据集回放

配置：

```yaml
zed:
  use_dataset_mode: true
```

启动：

```bash
ros2 launch wheel_perception dwa_dataset_test.launch.py \
  dataset_path:=/home/x/tami/datasets6/text6_stereo \
  fps:=20.0 \
  loop:=false
```

数据集模式不会验证WORLD速度，因为它按设计使用当前控制器上一输出构造DWA动态窗口。

### 8.3 只接ZED、手动摇杆测试

配置：

```yaml
zed:
  use_dataset_mode: false
  odometry:
    enabled: true
```

不启动BLE桥：

```bash
ros2 launch wheel_perception run_launch.py
```

检查：

```bash
ros2 topic hz /odom
ros2 topic echo /odom --field twist.twist
ros2 topic echo /zed/diagnostics/sdk_twist --field twist
```

录制：

```bash
mkdir -p bags
ros2 bag record -o bags/world_pose_velocity_test \
  /odom \
  /zed/diagnostics/sdk_twist \
  /cmd_vel \
  /perception/output \
  /rosout
```

启动录制后应确认终端明确显示已经订阅SDK诊断和 `/rosout`。

### 8.4 实车自主控制

先启动感知、规划和控制：

```bash
ros2 launch wheel_perception run_launch.py
```

确认 `/odom`、道路、障碍、轨迹、急停和 `/cmd_vel` 正常后，再单独启动BLE：

```bash
ros2 launch ble_hardware_bridge ble_hardware_bridge.launch.py
```

数据集回放时严禁启动BLE桥。

## 9. 当前已知问题和未完成项

### 9.1 `/odom` 还没有独立callback group

ControllerNode中的 `/odom` 和感知/DWA回调当前仍在默认互斥callback group。即使使用 `component_container_mt`，同一互斥组内也不能并行。历史bag曾在 `/odom` 发布间隔小于超时阈值时仍出现：

```text
Velocity feedback unavailable or stale
```

独立callback group目前只是后续方案，尚未修改代码。实现时应使用独立的 `MutuallyExclusive` group，并继续用 `velocity_mutex_` 保护共享状态；DWA只在锁内复制快照，不能持锁运行完整规划。

### 9.2 偶发WORLD位姿差分尖峰

第三次bag仍出现少量极值：

```text
|linear.x| > 0.9 m/s：1帧
|linear.y| > 0.2 m/s：2帧
|angular.z| > 1.0 rad/s：3帧
```

`velocity_max_dt` 只能处理时间间隔异常，不能拒绝“时间戳正常但位姿突然跳变”。后续应增加基于轮椅物理能力的速度、加速度和角速度异常检查；急停不得被舒适滤波延迟。

### 9.3 实际处理频率低于配置FPS

完整感知与控制链约16~17 Hz，不等于ZED参数中的60 FPS。WORLD差分使用图像时间戳后，速度尺度不再依赖处理频率，但较低控制频率仍影响：

- DWA动态窗口；
- `/cmd_vel`阶梯感；
- 障碍响应延迟；
- 速度反馈超时裕量。

第三次bag有3次发布间隔超过0.2秒，最大约0.30秒，已经接近 `velocity_feedback.timeout=0.3`。下一步应同时排查抓帧/处理阻塞和Controller回调排队。

### 9.4 仍不是轮编码器真值

WORLD位姿差分比控制命令或错误尺度的SDK Twist更接近真实运动，但仍属于ZED视觉惯性里程计。弱纹理、反光、遮挡、运动模糊和震动会影响结果。载人前仍需已知距离、秒表、轮编码器或动捕完成尺度和延迟验证。

### 9.5 其他v0.4遗留项

以下功能在v0.5中仍未实现：

- 障碍物空间聚类；
- 普通急停2/3帧时间一致性；
- 净空逐步改善奖励；
- 安全条件下的停车有限惩罚；
- 动态障碍速度预测；
- 非圆形轮椅footprint；
- 倒车脱困与DWA原地转向；
- 最终 `/cmd_vel` 的安全感知输出整形器。

## 10. 部署注意事项

当前 `params.yaml` 的 TensorRT engine仍指向旧工作区：

```text
/home/x/tami/wheel_latest/src/wheel_perception/config/combined5.engine
```

当前电脑上该文件存在，但仓库不包含 `.engine`。部署到开发板前必须复制与目标GPU/CUDA/TensorRT匹配的engine，并修改 `ai.engine_path`。

开发板还必须重新确认：

- `zed.use_dataset_mode: false`；
- 相机相对驱动轮旋转中心的实际外参；
- CUDA架构；
- BLE地址、适配器和UUID；
- DWA最大速度、制动距离和footprint；
- 物理急停可独立工作。

## 11. v0.5 结论

v0.5把v0.4中“已经接通但绝对尺度错误”的ZED速度反馈，改成了有明确坐标语义、时间基准、异常周期保护和单元测试的WORLD位姿差分链路。实车bag证明：

- 一、二、三档速度量级合理；
- 静止漂移约毫米每秒量级；
- `translation_x=0.45 m` 能基本消除原地转向杆臂速度；
- SDK Twist与真实运动趋势高度相关，但绝对值放大约3.4倍；
- DWA应继续使用 `/odom` WORLD差分，SDK Twist只作诊断。

当前最高优先级不再是“修复SDK Twist尺度”，而是：让 `/odom` 回调独立及时执行、增加WORLD速度物理异常过滤、确认处理链0.3秒间隙来源，并以外部真值完成最终标定。在完成这些工作前，系统仍应保持空载、低速、独立物理急停和安全员监督。
