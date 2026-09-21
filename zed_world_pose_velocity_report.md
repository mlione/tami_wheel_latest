# ZED WORLD 位姿差分速度修改报告

## 1. 修改目标

本次修改解决原速度反馈链路中两个核心问题：

1. 不再累加 `REFERENCE_FRAME::CAMERA` 返回的相对位姿。
2. 不再把 ZED SDK 的 `Pose.twist` 直接作为 DWA 的当前速度。

实车模式现在使用连续两帧 ZED `WORLD` 绝对位姿和对应的图像时间戳，计算轮椅中心 `base_link` 的线速度与角速度，并通过 `/odom.twist` 提供给控制器。SDK Twist 仅保留为诊断数据。

数据集模式没有改变，仍使用 `last_output_velocity_` 构造 DWA 动态窗口，不会把历史数据集的里程计速度反馈给当前控制过程。

## 2. 新速度链路

```text
ZED grab
  ├─ IMAGE timestamp
  ├─ REFERENCE_FRAME::WORLD absolute camera pose
  └─ REFERENCE_FRAME::CAMERA SDK Twist（仅诊断）
          |
          v
使用相机外参消除杆臂运动
          |
          v
连续 base_link WORLD 位姿差分
          |
          v
有效时间间隔检查 + EMA
          |
          v
/odom.twist.linear.x、/odom.twist.angular.z
          |
          v
ControllerNode / DWA 动态窗口
```

位姿换算：

```text
W_T_B = W_T_C * inverse(B_T_C)
```

其中 `B_T_C` 是配置文件中的相机相对轮椅中心外参。这样相机安装在轮椅中心前方或侧方时，转向产生的相机圆周运动会在求速度前被消除。

连续位姿差分：

```text
B(k-1)_T_B(k) = inverse(W_T_B(k-1)) * W_T_B(k)
v = translation(B(k-1)_T_B(k)) / dt
w = Log(rotation(B(k-1)_T_B(k))) / dt
```

`dt` 来自两帧 ZED 图像时间戳，不使用 ROS 回调到达时间，因此推理耗时、线程调度和 ROS bag 开销不会直接改变速度的时间分母。

## 3. 安全与异常处理

- 第一帧只有绝对位姿，没有差分基准，因此不发布 `/odom`。
- 跟踪状态不是 `OK` 时停止发布 `/odom`，并清除速度差分历史。
- 时间戳倒退、重复或 `dt` 超出配置范围时，不发布该次速度。
- 跟踪恢复后先重新建立差分基准，再从下一帧发布速度，避免把中断期间的位移除以错误周期。
- `/odom.twist` 使用独立配置的协方差，不再冒用 SDK Twist 的协方差。
- 原有速度 EMA 保留，应用对象改为 WORLD 位姿差分结果。

## 4. SDK Twist 诊断隔离

新增诊断话题：

```text
/zed/diagnostics/sdk_twist
```

消息类型：

```text
geometry_msgs/msg/TwistStamped
```

该话题已按相机外参转换到 `base_link`，可与 `/odom.twist` 对比，但 ControllerNode 不订阅它，所以不会参与 DWA 或底盘控制。

## 5. 新增参数

文件：`src/wheel_perception/config/params.yaml`

```yaml
zed:
  odometry:
    velocity_filter_alpha: 0.25
    velocity_min_dt: 0.005
    velocity_max_dt: 0.2
    linear_velocity_stddev: 0.20
    angular_velocity_stddev: 0.25
    publish_sdk_twist_diagnostic: true
```

参数说明：

- `velocity_filter_alpha`：差分速度 EMA 中新数据的权重。越小越平滑，但延迟越大。
- `velocity_min_dt`：允许参与差分的最小图像时间间隔，防止时间戳过近导致除法放大噪声。
- `velocity_max_dt`：允许参与差分的最大图像时间间隔，防止卡帧或跟踪中断生成错误速度。
- `linear_velocity_stddev`：写入 `/odom.twist.covariance` 的线速度标准差，不直接改变速度值。
- `angular_velocity_stddev`：写入 `/odom.twist.covariance` 的角速度标准差，不直接改变速度值。
- `publish_sdk_twist_diagnostic`：是否发布原始 SDK Twist 诊断话题。

现有相机外参继续生效：

```yaml
extrinsic:
  translation_x: 0.2
  translation_y: -0.2
  translation_z: 0.0
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

当前坐标定义是 `base_link` 的 X 向前、Y 向左，因此“相机在中心前方 0.2 m、右侧 0.2 m”对应 `x=0.2, y=-0.2`。

## 6. 修改文件

- `src/wheel_perception/src/core/zed_driver.cpp`
  - 改为读取 `REFERENCE_FRAME::WORLD` 绝对位姿。
  - 删除 CAMERA 相对位姿累加。
  - SDK Twist 改为独立诊断数据。
- `src/wheel_perception/include/wheel_perception/core/zed_driver.hpp`
  - 明确 WORLD 位姿和 SDK 诊断 Twist 的语义。
  - 新增 `sdk_twist_valid`。
- `src/wheel_perception/include/wheel_perception/core/world_pose_velocity_estimator.hpp`
  - 新增可独立测试的世界位姿差分估计器接口。
- `src/wheel_perception/src/core/world_pose_velocity_estimator.cpp`
  - 实现外参补偿、SE(3) 位姿差分、时间有效性检查和重置逻辑。
- `src/wheel_perception/src/fusion_node.cpp`
  - 用估计器产生 `/odom`。
  - 保留速度 EMA。
  - 新增 `/zed/diagnostics/sdk_twist`。
- `src/wheel_perception/src/controller_node.cpp`
  - 明确实车速度来源是 WORLD 位姿差分 `/odom.twist`。
  - 数据集模式逻辑不变。
- `src/wheel_perception/config/params.yaml`
  - 加入时间间隔、协方差和诊断开关参数。
- `src/wheel_perception/test/test_world_pose_velocity_estimator.cpp`
  - 测试直行速度、相机杆臂补偿和异常时间间隔处理。
- `src/wheel_perception/CMakeLists.txt`
  - 编译新估计器并注册测试。

## 7. 构建与自动测试

```bash
cd ~/tami/tami_wheel_latest
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只运行本功能包已有测试：

```bash
ctest --test-dir build/wheel_perception --output-on-failure
```

本次验证结果：

```text
test_dwa_planner                      Passed
test_world_pose_velocity_estimator    Passed
100% tests passed
```

## 8. 实车验证流程

先确认配置为实车模式：

```yaml
zed:
  use_dataset_mode: false
  odometry:
    enabled: true
```

启动：

```bash
cd ~/tami/tami_wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wheel_perception wheel_perception.launch.py
```

检查频率和速度：

```bash
ros2 topic hz /odom
ros2 topic echo /odom --field twist.twist
ros2 topic echo /zed/diagnostics/sdk_twist --field twist
ros2 topic echo /cmd_vel
```

用 `rqt_plot` 对比新差分速度、SDK 诊断速度和控制命令：

```bash
rqt_plot /odom/twist/twist/linear/x /zed/diagnostics/sdk_twist/twist/linear/x /cmd_vel/linear/x
```

角速度对比：

```bash
rqt_plot /odom/twist/twist/angular/z /zed/diagnostics/sdk_twist/twist/angular/z /cmd_vel/angular/z
```

建议录制：

```bash
ros2 bag record -o bags/world_pose_velocity_test \
  /odom /zed/diagnostics/sdk_twist /cmd_vel /perception/output
```

测试顺序建议：静止 10 秒、直行、原地或低速转向、反向、停车。重点检查：

1. 静止时 `/odom.twist` 是否接近零。
2. 直行时 `linear.x` 是否与实测距离/时间一致。
3. 转向时相机杆臂是否不再造成明显的虚假 `linear.x`。
4. `/odom` 时间戳和频率是否连续。
5. DWA 日志中的当前速度是否随真实运动变化，而不是只跟随 `/cmd_vel`。

## 9. 注意事项与边界

- WORLD 位姿差分是视觉惯性里程计速度，比把控制命令当作真实速度更接近轮椅实际运动，但它仍不是轮编码器真值。
- 弱纹理、强反光、运动模糊、相机遮挡或剧烈震动会降低 ZED 位姿质量；此时本实现会尽量停止发布异常速度，但仍需实车低速验证。
- 相机外参尤其是 `translation_x`、`translation_y` 和 `yaw` 必须与实际安装一致，否则转向时仍会出现线速度耦合。
- 不建议仅为了让曲线好看而把 `velocity_filter_alpha` 调得过小，否则 DWA 动态窗口会使用明显滞后的速度。
- 数据集回放仍采用控制器上一输出速度，这次修改不会改变数据集模式的 DWA 行为。
