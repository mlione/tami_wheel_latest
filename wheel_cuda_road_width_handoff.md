# wheel_cuda 道路宽度与调试可视化交接

生成日期：2026-07-13  
工程目录：`/home/lee/wheel/wheel_cuda`

## 交接目标

后续会话继续验证和调优：

- `/debug/viz_mouse_left` 道路 mask 叠加及文字指标；
- 左右道路边界宽度估计；
- 宽路场景（实际约 6m）中避免将图像左边框误判为道路左边界；
- 数据集热切换后道路宽度状态恢复。

## 建议技能

- `diagnose`：复现并区分 mask、深度、边界配对和历史状态问题。
- `ai-coding-standard`：修改前检查结构、保持最小改动并添加中文注释。
- `ros2-development`：检查生命周期、QoS、话题和参数加载。
- `robot-bringup`：数据集播放器、外置硬盘和 ROS2 运行态诊断。
- `robotics-testing`：如果要为边界筛选增加回放或回归测试。

## 项目约束

- 必须使用中文回复。
- 修改任何代码前，先说明修改范围、原因和影响，并取得用户明确同意。
- 不覆盖或清理用户已有未提交改动。
- ROS2 尽量兼容 Foxy 及以上，Ubuntu 20.04 及以上。

## 本轮修改文件清单

本轮针对“调试图像显示道路宽度”和“排除左侧图像边框伪边界”实际修改了以下 3 个文件：

| 修改文件 | 具体位置/搜索关键词 | 修改或新增内容 |
|---|---|---|
| `src/wheel_perception/src/fusion_node.cpp` | 约第 133～185 行，搜索 `pub_viz_mouse_left_` | 新增 `/debug/viz_mouse_left` 生命周期发布器，并加入激活、停用和清理流程 |
| `src/wheel_perception/src/fusion_node.cpp` | 约第 650～715 行，搜索 `update_dynamic_target_from_edges` | 道路宽度使用的左右边界前向范围由 `1～3.5m` 改为 `2～8m`；合法宽度默认上限改为 `8m` |
| `src/wheel_perception/src/fusion_node.cpp` | 约第 935～1040 行，搜索 `Target Dist` | 道路 mask 叠加图左上角新增目标距离、右侧边界距离和道路宽度文字；无效宽度显示 `N/A` |
| `src/wheel_perception/src/core/obstacle_fusion.cu` | 约第 194～209 行，搜索 `left_border_margin` | 新增左边框保护；首个道路像素落入左侧约 1% 保护区时，该行左边界判为不可见 |
| `src/wheel_perception/config/params.yaml` | 约第 69～79 行，搜索 `dynamic_aim` | 将 `dynamic_aim.max_width` 从 `5.0` 改为 `8.0` |

### 文件级变更摘要

#### `src/wheel_perception/src/fusion_node.cpp`

新增内容：

- 成员 `pub_viz_mouse_left_`；
- ROS2 图像话题 `/debug/viz_mouse_left`；
- 三行 OpenCV 文字 `Target Dist`、`Right Dist`、`Road Width`；
- `<cstdio>` 头文件，用于 `std::snprintf`。

调整内容：

- 道路宽度左右边界采样范围改为 `2～8m`；
- `dynamic_aim_max_width_` 成员默认值由 `5.0f` 改为 `8.0f`；
- `dynamic_aim.max_width` 的参数声明默认值由 `5.0f` 改为 `8.0f`；
- 可视化无人订阅判断同时检查 `/debug/viz` 和 `/debug/viz_mouse_left`。

未调整内容：

- LQR 右边界候选范围仍为 `0.5～15m`；
- LQR 双区间拟合、EMA、控制输出均未修改。

#### `src/wheel_perception/src/core/obstacle_fusion.cu`

新增内容：

- `left_border_margin = max(2, ai_w / 100)`；
- 左边界候选落入保护区时执行 `found_left = false`。

本轮没有修改右侧边界扫描方向、道路类别 ID、点云投影方式或 CUDA 输出结构。

#### `src/wheel_perception/config/params.yaml`

调整内容：

- `dynamic_aim.max_width: 5.0 -> 8.0`。

文件中的 `zed.use_dataset_mode: true` 是本轮任务开始前已经存在的用户改动，不属于本轮修改，不能误删或还原。

### 本轮未修改但工作区已有改动的文件

以下文件虽然出现在 `git status` 中，但不是本轮道路宽度/可视化任务新增的修改：

- `.gitignore`
- `src/wheel_perception/CMakeLists.txt`
- `src/wheel_perception/include/wheel_perception/core/obstacle_fusion.hpp`
- `src/wheel_perception/include/wheel_perception/core/mask_postprocess.hpp`
- `src/wheel_perception/src/core/mask_postprocess.cu`
- `src/wheel_perception/src/core/obstacle_fusion.cu` 中已有的最大道路连通域 GPU 后处理部分

## 本轮完成的修改

### 1. 左相机调试可视化

文件：`src/wheel_perception/src/fusion_node.cpp`

- 新增生命周期图像发布器 `/debug/viz_mouse_left`，消息类型为 `sensor_msgs/msg/Image`，QoS depth 为 10。
- 保留原有 `/debug/viz`，两者发布同一张渲染图，避免破坏已有订阅者。
- 两个话题均无人订阅时，跳过 CPU mask 下载和 OpenCV 渲染。
- 道路类别 mask 继续以绿色半透明方式叠加在 BGR 原图上。
- 左上角增加三行文字：
  - `Target Dist`：动态右侧目标距离；
  - `Right Dist`：LQR 使用的右侧边界距离；
  - `Road Width`：道路宽度；当 `has_road_width=false` 时显示 `N/A`。
- 新发布器已加入 lifecycle 的 configure、activate、deactivate、cleanup 流程。

运行时查看：

```bash
ros2 run rqt_image_view rqt_image_view /debug/viz_mouse_left
```

### 2. 排除图像左边框伪边界

文件：`src/wheel_perception/src/core/obstacle_fusion.cu`

- 在 `extract_road_edges_kernel()` 中增加左边框保护。
- 保护宽度为 `max(2, ai_w / 100)`，即至少 2 像素、通常约为图像宽度的 1%。
- 如果某行第一个道路像素落入保护区，该行左边界设置为无效。
- 目的：道路 mask 延伸至图像左边缘时，真实左边界实际位于视野之外，不能把裁剪边缘当作道路边界。
- 真实左边界没有进入画面时，宁可使宽度显示 `N/A`，也不输出约 3m 的虚假宽度。

代码定位：搜索 `left_border_margin`。

### 3. 道路宽度采样前向范围改为 2～8m

文件：`src/wheel_perception/src/fusion_node.cpp`

- `update_dynamic_target_from_edges()` 中，左右边界点均要求：
  - `2.0 <= x <= 8.0m`；
  - 右边界 `y < -0.05m`；
  - 左边界 `y > 0.05m`；
  - 同一配对左右点的前向距离差 `abs(right.x-left.x) <= 1.2m`。
- 宽度仍按 `left.y - right.y` 计算，对多个有效样本取中位数。
- 有效样本仍要求至少 8 个。

该修改仅影响道路宽度和动态目标距离，不影响 LQR 右边界拟合。

### 4. 合法道路宽度上限提高到 8m

文件：

- `src/wheel_perception/config/params.yaml`
- `src/wheel_perception/src/fusion_node.cpp`

修改：

- `dynamic_aim.max_width`：`5.0 -> 8.0`；
- C++ 成员初值和 `declare_parameter()` 默认值同步改为 `8.0f`。

目的：允许实际约 6m 的宽路通过合法性检查。

## 明确未修改的 LQR 行为

LQR 右边界候选仍使用：

- 前向距离 `0.5 < x < 15.0m`；
- 仅右侧点 `y < 0`；
- 有效点序列远端 `20%～40%` 和近端 `60%～80%` 分别求重心；
- 使用两重心计算航向误差和车辆到拟合直线的垂直距离；
- EMA 系数仍为 `0.3`。

因此本轮宽度范围改为 2～8m，不会改变 LQR 主巡线边界或控制基准。

## 参数调整说明

配置位置：`src/wheel_perception/config/params.yaml` 的 `dynamic_aim`。

| 参数 | 当前值 | 调大效果 | 调小效果 |
|---|---:|---|---|
| `min_width` | 1.2m | 拒绝更多窄路/错误配对 | 允许更窄道路，但伪宽度更容易通过 |
| `max_width` | 8.0m | 允许更宽道路，也会扩大错误测量接纳范围 | 更严格，但实际宽路可能显示 `N/A` |
| `ratio` | 0.35 | 目标位置向道路左侧/中心移动 | 目标更靠右侧边界 |
| `min_dist` | 0.65m | 窄路时仍保持更大的右侧距离 | 允许更贴近右边界 |
| `max_dist` | 1.0m | 宽路时目标可继续向道路中心移动 | 宽路时仍限制靠右 |
| `fallback_dist` | 0.8m | 宽度长期无效后回退得更靠左 | 回退得更靠右 |
| `max_target_step` | 0.05m/帧 | 目标响应更快但更易跳变 | 更平滑但响应变慢 |
| `hold_frames` | 10 帧 | 宽度短时失效时保持旧目标更久 | 更快进入 fallback |

注意：以下当前不是 ROS 参数，调整需要修改并重新编译：

- 宽度采样范围 `2.0～8.0m`：在 `fusion_node.cpp` 搜索 `right.x < 2.0f`；
- 左边框保护比例：在 `obstacle_fusion.cu` 搜索 `left_border_margin`；
- 左右点前向配对容差 `1.2m`：在 `fusion_node.cpp` 搜索 `abs(right.x - left.x)`；
- 最少有效样本数 8：搜索 `widths.size() >= 8`。

修改 YAML 参数后需重新启动/重新 configure `fusion_node`。修改 C++/CUDA 后必须重新编译并重启 launch。

## 参数与输出的典型关系

- 左侧真实边界在 2～8m 内可见且深度有效：应输出接近实际的道路宽度。
- 道路 mask 接触图像左边框：相应行左边界无效；有效行不足 8 个时显示 `Road Width: N/A`。
- 实测宽度大于 `max_width` 或小于 `min_width`：显示 `N/A`。
- 左右点前向距离差大于 1.2m：该配对被丢弃。
- `target_right_distance = clamp(road_width * ratio, min_dist, max_dist)`，并受 `max_target_step` 限速。

例如道路宽度 6m、`ratio=0.35` 时原始目标为 2.1m，但受 `max_dist=1.0m` 限制，最终目标仍为 1.0m。因此提高 `max_width` 只会让 6m 测量合法，不会自动让车辆驶向道路中央。

## 已知问题：数据集热切换后的宽度状态锁死

`fusion_node` 持有 `last_valid_road_width_`。当前保护逻辑会拒绝相对上一可信宽度突降超过 0.4m 的新测量：

```text
measured_width < last_valid_road_width - 0.4
```

海珠湖约 3.5m 直接切换到生物岛约 2m 时，新宽度会持续被拒绝：

- `has_road_width=false`，可视化显示 `N/A`；
- 日志仍打印缓存的旧宽度；
- 重启 `fusion_node` 可临时清除状态。

该问题尚未修复。建议后续采用“连续若干帧稳定的新宽度后接受新基准”，或由数据集切换事件显式重置宽度状态。修改前仍需用户确认。

## 验证状态

已执行：

```bash
colcon build --packages-select wheel_perception --symlink-install
```

结果：

```text
Summary: 1 package finished [49.2s]
```

存在两条原有 CUDA 警告：变量 `found` 和 `last_record_y` 被赋值但未使用，与本轮修改无关。

尚未完成真实数据集回放回归：外置硬盘曾在强制拔插后出现 USB 设备被枚举但容量为 `0B`、无分区和挂载点。需要先恢复硬盘识别，再分别回放窄路和约 6m 宽路场景。

## 建议的下一步验证

1. 确认外置硬盘在 `lsblk` 中显示正确容量、分区和挂载点。
2. 重启 launch，确保加载最新二进制和 YAML。
3. 回放约 6m 宽路，观察 `/debug/viz_mouse_left`。
4. 同时查看：

   ```bash
   ros2 topic echo /perception/output
   ```

5. 重点记录 `has_road_width`、`left_distance`、`right_distance`、`road_width` 和日志中的 `samples`。
6. 如果持续 `N/A`，优先判断：
   - 左边界是否仍未进入画面；
   - 2～8m 是否有至少 8 对有效深度点；
   - 左右点 `x` 差是否普遍超过 1.2m；
   - 深度在远端是否大量 NaN/Inf。
7. 回放较窄道路，验证 2～8m 范围没有引入远端深度噪声。
8. 做海珠湖到生物岛热切换，单独复现并修复上述状态锁死。

## 工作区状态与保护事项

工作区存在多项未提交修改，不能使用 `git reset --hard`、`git checkout --` 或覆盖式还原：

```text
M  .gitignore
M  src/wheel_perception/CMakeLists.txt
M  src/wheel_perception/config/params.yaml
M  src/wheel_perception/include/wheel_perception/core/obstacle_fusion.hpp
M  src/wheel_perception/src/core/obstacle_fusion.cu
M  src/wheel_perception/src/fusion_node.cpp
?? src/wheel_perception/include/wheel_perception/core/mask_postprocess.hpp
?? src/wheel_perception/src/core/mask_postprocess.cu
```

其中最大道路连通域 GPU 后处理、`mask_postprocess.*`、CMakeLists 和其他 CUDA 改动在本次边界任务开始前已经存在。继续工作前应使用定向 `git diff -- <file>`，不要把整份文件恢复到 HEAD。

## 关键代码索引

- 调试话题及文字绘制：`src/wheel_perception/src/fusion_node.cpp`，搜索 `viz_mouse_left`、`Target Dist`。
- 宽度配对与中位数：同文件，搜索 `update_dynamic_target_from_edges`。
- LQR 右边界拟合：同文件，搜索 `Double Interval`。
- 左右 mask 边界提取：`src/wheel_perception/src/core/obstacle_fusion.cu`，搜索 `extract_road_edges_kernel`。
- 动态目标参数：`src/wheel_perception/config/params.yaml`，搜索 `dynamic_aim`。
- 消息字段：`src/wheel_msgs/msg/PerceptionOutput.msg`。
