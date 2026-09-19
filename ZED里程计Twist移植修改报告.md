# ZED里程计Twist移植修改报告

## 1. 修改目标

将 `zed2i_nitros` 中可用的 ZED CAMERA 增量里程计方法移植到现有
`ZedDriver -> FusionNode -> /odom -> ControllerNode` 链路。不启动第二个 ZED
节点，现有驱动继续独占相机，RGB、深度、点云和 TensorRT 感知入口保持不变。

当前外参按用户提供的安装位置处理：

```text
ZED相机 = 轮椅中心前方 0.2 m、右侧 0.2 m
base_link坐标 = X向前、Y向左，因此平移外参为 (0.2, -0.2, 0.0) m
相机和 base_link 坐标轴方向暂按对齐处理
```

外参现已参数化，未来标定后只需修改 YAML，不需要改动 ZED 取数或 ControllerNode。

## 2. 数据链

```text
ZED grab
  +-- RGB / depth / cloud -> 原有感知链
  +-- CAMERA incremental pose + twist + covariance
             |
       ZedGpuFrame
             |
  makeBaseOdometryMessage()
             |
  filterOdometryTwist() (EMA)
             |
           /odom
             |
  ControllerNode freshness check
             |
  measured_velocity_ -> DWA dynamic window
```

## 3. 代码修改

### 3.1 ZedDriver

`ZedGpuFrame` 新增：

- 3轴线速度；
- 3轴角速度；
- pose/twist 6x6 协方差；
- `odometry_valid` 跟踪有效标志。

`populateOdometryFrame()` 每帧执行：

1. 调用 `getPosition(..., sl::REFERENCE_FRAME::CAMERA)`；
2. 检查 positional tracking、odometry status 和 pose validity；
3. 累积 CAMERA 增量位姿并归一化四元数；
4. 填充 twist 和协方差；
5. 跟踪失效时只置 `odometry_valid=false`，不伪造零速度。

旧的全局 `flag_odom` 及其切换/重置逻辑已删除，改为项目原有 YAML 参数系统。

### 3.2 FusionNode

新增并封装：

- `makeBaseOdometryMessage()`：相机里程计转为 `base_link` Odometry；
- `transformCameraMotionPoseToBase()`：通过外参共轭将相机相对位姿换算到轮椅旋转中心；
- `transformCameraTwistToBase()`：完成坐标轴旋转和杆臂速度扣除；
- `transformCameraCovarianceToBase()`：使用同一刚体运动 Jacobian 转换 6x6 协方差；
- `filterOdometryTwist()`：对六轴 twist 做一阶 EMA；
- `publish_odometry()`：负责有效性、警告和发布。

同一外参还应用到感知/规划坐标链：

- FusionNode 在发布 `/perception/output` 前，把最近障碍点、道路边界距离、道路角度和调试点换算到 `base_link`；
- ControllerNode 先把 `zed/point_cloud` 中候选障碍点变换到 `base_link`，再执行 DWA ROI 过滤和代价计算；
- `/dwa/obstacle_cloud` 现在明确使用 `base_link` frame；
- `run_launch.py` 从同一 YAML 参数发布 `base_link -> zed_left_camera_frame`，避免 RViz 与算法坐标不一致。

当ZED跟踪失效时不发布伪造 `/odom`，ControllerNode 通过时间超时发现失联。

### 3.3 ControllerNode

- 接受有限的 `(v,w)=(0,0)` 作为合法停车反馈；
- 记录最后速度反馈接收时间；
- `velocityFeedbackFreshLocked()` 判断新鲜度；
- `planningMotionSnapshot()` 在多线程容器中一次性复制 pose、velocity 和有效状态；
- 超时时可选回退到上一控制输出或立即停车；
- 日志显示 `zed_odom`、`controller_output(fallback)` 或
  `controller_output(dataset)`。

## 4. 新增参数

```yaml
zed:
  odometry:
    enabled: true
    odom_frame: odom
    base_frame: base_link
    velocity_filter_alpha: 0.25
    extrinsic:
      translation_x: 0.2
      translation_y: -0.2
      translation_z: 0.0
      roll: 0.0
      pitch: 0.0
      yaw: 0.0

velocity_feedback:
  timeout: 0.3
  stop_on_timeout: false
  accept_zero_twist: true
```

`velocity_filter_alpha` 越小越平滑但延迟越大。`stop_on_timeout=false`
是首次空载调试的兼容设置：超时回退到上一输出并警告。ZED跟踪经验证后，载人前建议评估改为 `true`。

## 5. 与数据集的隔离

数据集模式仍然：

- 不创建实车 `ZedDriver`；
- 只用历史 `/zed/odom` 的 pose；
- 不使用历史 twist 或回放FPS差分速度；
- DWA 使用本轮回放的 `last_output_velocity_`。

因此新的实车ZED速度反馈不改变数据集规划结果的速度来源。

## 6. 编译和验证

实车前先将 `params.yaml` 中的 `zed.use_dataset_mode` 改为 `false`，
同时保持 `zed.odometry.enabled: true`。

```bash
cd ~/tami/tami_wheel_latest
source /opt/ros/humble/setup.bash
source install/setup.sh
colcon build --symlink-install --base-paths src/wheel_perception
```

实车接入ZED后检查：

```bash
ros2 topic hz /odom
ros2 topic echo /odom --field twist.twist
ros2 topic echo /odom --field child_frame_id
ros2 topic echo /cmd_vel
```

预期 `child_frame_id=base_link`，直行时 `linear.x>0`，左转时
`angular.z>0`，静止时两者稳定趋近零。控制日志应显示
`velocity_source=zed_odom`。

本次已完成的验证：

- `wheel_perception` 编译通过，本机 ZED SDK 支持移植的 CAMERA pose/twist/协方差 API；
- DWA 13 项回归测试全部通过；
- 用 `/home/x/tami/datasets6/text6_stereo` 完成循环回放回归；
- 回放日志持续显示 `velocity_source=controller_output(dataset)`，证明没有误用历史 twist；
- 回放期间 `/perception/output`、`/dwa/*` 和 `/cmd_vel` 正常发布；
- 由于本轮没有连接真实 ZED，`velocity_source=zed_odom` 的数值方向、漂移和失效恢复仍需实车验收。

## 7. 影响和限制

- RGB-D、TensorRT、点云、道路边界和 BLE 链路未更换数据入口；
- ZED positional tracking 原本已在驱动打开，新增工作为每帧读取、小量计算和一条 Odometry 发布；
- 实车 DWA 动态窗口会开始反映实际运动估计，与旧的开环目标速度行为不完全相同；
- ZED twist 不是轮编码器反馈，会受纹理、光照、遮挡和震动影响；
- 当前外参是手工估计值 `(0.2, -0.2, 0.0) m`，已实现位姿、Twist 和协方差修正，但实车仍需测量/标定后替换该数值。
- 当前 TensorRT `combined5.engine` 仍位于旧工程目录，YAML 暂保留可用的旧路径；迁移开发板时需复制/重建 engine 并更新 `ai.engine_path`。
