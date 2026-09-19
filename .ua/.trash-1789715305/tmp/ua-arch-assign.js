#!/usr/bin/env node
'use strict';

const fs = require('fs');

const [inputPath, structuralPath, outputPath] = process.argv.slice(2);
if (!inputPath || !structuralPath || !outputPath) {
  throw new Error('usage: ua-arch-assign.js INPUT STRUCTURAL_RESULTS OUTPUT');
}

const input = JSON.parse(fs.readFileSync(inputPath, 'utf8'));
const structural = JSON.parse(fs.readFileSync(structuralPath, 'utf8'));
if (!structural.scriptCompleted || structural.fileStats.totalFileNodes !== input.fileNodes.length) {
  throw new Error('structural analysis is incomplete or inconsistent');
}

const definitions = [
  {
    id: 'layer:perception',
    name: '感知与传感器层',
    description: '负责 ZED RGB-D 取流、TensorRT 语义分割、CUDA 掩膜后处理、点云融合和道路边界提取，并向导航链路提供环境观测。',
  },
  {
    id: 'layer:planning-control',
    name: '规划与控制层',
    description: '实现道路巡航与 DWA 局部避障决策，以 LQR 跟踪局部轨迹并发布候选路径、最佳路径和速度控制结果。',
  },
  {
    id: 'layer:vision-foundation',
    name: '视觉基础库层',
    description: '提供 cv_bridge 图像互操作、Python/OpenCV 绑定以及单目和双目相机几何投影能力，为轮椅视觉感知提供底层支撑。',
  },
  {
    id: 'layer:ros-integration',
    name: 'ROS 2 接口与集成层',
    description: '定义轮椅感知消息，组织 ROS 2 launch、蓝牙底盘桥接和手机调试网关，连接感知、规划、可视化与执行端。',
  },
  {
    id: 'layer:offline-tooling',
    name: '离线数据与模型工具层',
    description: '承担 ZED 数据采集回放、双目标定与数据集转换，以及 BiSeNet/ONNX/TensorRT 模型检查、转换和离线推理。',
  },
  {
    id: 'layer:test',
    name: '测试与验证层',
    description: '覆盖 cv_bridge、相机几何、OpenCV ROS 集成、手机网关和 DWA 规划器的单元测试及数据集验证入口。',
  },
  {
    id: 'layer:config',
    name: '配置与构建层',
    description: '集中管理 ROS 2 包元数据、运行参数、Python/CMake 构建设置、CI 脚本、RViz 依赖资源和分析工具配置。',
  },
  {
    id: 'layer:documentation',
    name: '文档与工程记录层',
    description: '汇总系统说明、版本演进、实车测试手册、API 文档、构建清单、许可证和上游组件变更记录。',
  },
];

const members = new Map(definitions.map((layer) => [layer.id, []]));

function assign(node) {
  const p = (node.filePath || '').toLowerCase();
  const tags = (node.tags || []).join(' ').toLowerCase();

  // Node types are the primary signal for non-code artifacts.
  if (node.type === 'document') return 'layer:documentation';
  if (node.type === 'config') return 'layer:config';

  // Documentation-like files that the scanner represented as generic files.
  if (/(^|\/)license(-|$)/.test(p) || /\/doc\/(conf\.py|mainpage\.dox)$/.test(p)) {
    return 'layer:documentation';
  }

  // Tests and dedicated validation packages.
  if (/\/test\//.test(p) || /\/opencv_tests\//.test(p) || /(^|\/)test_[^/]+\./.test(p) || p.endsWith('/dwa_dataset_test.launch.py')) {
    return 'layer:test';
  }

  // Build, CI and package registration artifacts represented as generic files.
  if (p.endsWith('/.travis.sh') || p.endsWith('.cmake.in') || p.endsWith('/setup.py') || /\/resource\/[^/]+$/.test(p)) {
    return 'layer:config';
  }

  // Local planning, LQR tracking and planning visualization.
  if (p.includes('/dwa_controller/') || p.endsWith('/lqr_controller.hpp') || p.endsWith('/controller_node.cpp') || p.endsWith('/dwa_navigation.rviz')) {
    return 'layer:planning-control';
  }

  // The runtime perception pipeline and sensor/CUDA implementation.
  if (p.includes('/wheel_perception/include/wheel_perception/core/') || p.includes('/wheel_perception/src/core/') || p.endsWith('/wheel_perception/src/fusion_node.cpp')) {
    return 'layer:perception';
  }

  // Dataset, calibration, model conversion and standalone inference utilities.
  if (p.startsWith('tools/') || ['video_pub.py', 'zed_capture_dataset.py', 'zed_dataset_player.py'].includes(p) || p.includes('/wheel_perception/config/') || p.includes('/wheel_perception/scripts/video_infer_trt.py')) {
    return 'layer:offline-tooling';
  }

  // ROS messages, launch orchestration, Bluetooth and phone debug integration.
  if (p.startsWith('src/wheel_msgs/') || p.startsWith('src/wheel_phone_gateway/') || p.endsWith('/wheel_perception/launch/run_launch.py') || p.endsWith('/wheel_perception/scripts/sub.py')) {
    return 'layer:ros-integration';
  }

  // Reusable upstream computer-vision libraries.
  if (p.startsWith('src/vision_opencv/')) return 'layer:vision-foundation';

  throw new Error(`unclassified file node: ${node.id} (${tags})`);
}

for (const node of input.fileNodes) members.get(assign(node)).push(node.id);

const layers = definitions.map((definition) => ({
  ...definition,
  nodeIds: members.get(definition.id),
}));

if (layers.length < 3 || layers.length > 10 || layers.some((layer) => layer.nodeIds.length === 0)) {
  throw new Error('layer count must be 3-10 and every layer must be non-empty');
}

const assigned = layers.flatMap((layer) => layer.nodeIds);
const unique = new Set(assigned);
const expected = new Set(input.fileNodes.map((node) => node.id));
const missing = [...expected].filter((id) => !unique.has(id));
const invented = [...unique].filter((id) => !expected.has(id));
if (assigned.length !== input.fileNodes.length || unique.size !== assigned.length || missing.length || invented.length) {
  throw new Error(`invalid assignments: assigned=${assigned.length}, unique=${unique.size}, expected=${expected.size}, missing=${missing.length}, invented=${invented.length}`);
}

fs.writeFileSync(outputPath, `${JSON.stringify(layers, null, 2)}\n`);
