import json
from pathlib import Path

ROOT = Path('/home/x/tami/wheel_latest')
UA = ROOT / '.ua'
BATCHES = json.loads((UA / 'intermediate/batches.json').read_text())['batches']


def complexity(lines):
    return 'simple' if lines < 50 else ('moderate' if lines <= 200 else 'complex')


def file_summary(path):
    p = path.lower()
    name = Path(path).name
    special = {
        'src/wheel_perception/dwa_controller/include/wheel_perception/dwa_controller/DWAPlanner.hpp': '定义道路约束 DWA 局部规划器的数据模型、参数、候选轨迹与规划接口。',
        'src/wheel_perception/dwa_controller/src/DWAPlanner.cpp': '实现动态窗口采样、差速运动学轨迹预测，以及航向、道路、障碍物、速度和平滑性综合评分。',
        'src/wheel_perception/src/controller_node.cpp': 'ROS 2 控制节点，在道路巡航与 DWA 避障间切换，并使用 LQR 跟踪局部轨迹后发布速度和可视化结果。',
        'src/wheel_perception/src/fusion_node.cpp': '系统感知主节点，组织 ZED 取流、TensorRT 推理、CUDA 点云融合、道路边界分析及 PerceptionOutput 发布。',
        'src/wheel_perception/include/wheel_perception/core/lqr_controller.hpp': '提供离散 LQR 控制器，根据横向误差和航向误差计算平滑角速度并进行执行器映射。',
        'src/wheel_perception/src/core/obstacle_fusion.cu': '实现 CUDA 点云过滤、障碍物体素化、道路边缘提取与盲区 BEV 融合，是感知到规划的核心数据处理层。',
        'src/wheel_perception/include/wheel_perception/core/obstacle_fusion.hpp': '声明障碍物融合参数、道路边缘数据结构及 GPU 感知处理接口。',
        'src/wheel_perception/src/core/ai_engine.cpp': '封装 TensorRT 引擎加载、CUDA 缓冲区管理和语义分割推理。',
        'src/wheel_perception/include/wheel_perception/core/ai_engine.hpp': '声明语义分割 TensorRT 推理引擎的初始化、推理和输入尺寸接口。',
        'src/wheel_perception/src/core/zed_driver.cpp': '封装 ZED 相机打开、抓帧、图像与点云获取及相机内参读取。',
        'src/wheel_perception/include/wheel_perception/core/zed_driver.hpp': '定义 ZED GPU 帧和相机驱动公开接口。',
        'src/wheel_perception/src/core/mask_postprocess.cu': '使用 CUDA 连通域标记保留主要道路区域，过滤语义分割掩膜中的零散噪声。',
        'src/wheel_perception/src/core/preprocess.cu': '实现 ZED 图像的 CUDA 双线性缩放、归一化以及网络输出 argmax。',
        'src/wheel_perception/config/params.yaml': '集中配置 ZED、感知、道路边界、DWA、LQR、安全层及数据集模式参数。',
        'src/wheel_perception/launch/dwa_dataset_test.launch.py': '启动离线数据集播放器和轮椅感知控制流水线，用于无实车验证 DWA 路径规划。',
        'src/wheel_perception/launch/run_launch.py': '装载感知与控制组合节点、静态坐标变换及项目参数，作为主要 ROS 2 启动入口。',
        'src/wheel_perception/rviz/dwa_navigation.rviz': '预配置 RViz 中的障碍物点云、道路边界、候选轨迹、最佳路径及巡航路径显示。',
        'src/wheel_perception/scripts/sub.py': 'ROS 2 蓝牙底盘桥接节点，将 cmd_vel 转换为差速轮控制协议并异步收发 BLE 数据。',
        'src/wheel_perception/scripts/video_infer_trt.py': '独立视频推理工具，直接调用 TensorRT 与 CUDA 执行 BiSeNet 道路分割并生成叠加视频。',
        'src/wheel_perception/test/test_dwa_planner.cpp': '覆盖 DWA 动态窗口、碰撞判断、道路约束、平滑代价与停车选择行为的单元测试。',
        'zed_capture_dataset.py': 'ROS 2 数据采集器，同步保存 ZED 双目、深度、感知结果、控制指令、里程计和 IMU 数据。',
        'zed_dataset_player.py': '离线数据集回放节点，按配置帧率发布历史 RGB-D、IMU 与里程计消息以复现完整感知控制链路。',
        'tools/convert_text6_stereo_dataset.py': '将 text6 左右目图像转换为项目回放器可读取的数据集目录，并可计算假设标定下的双目深度。',
        'tools/export_zed_calibration.py': '从已连接 ZED 相机读取双目标定参数并导出 JSON。',
        'tools/assumed_zed_hd720_calibration.json': '保存无相机时使用的 HD720 假设双目标定、基线和图像尺寸。',
        'video_pub.py': '按定时器读取视频帧并发布 ROS 图像话题，供视觉模块进行简单回放测试。',
        'src/wheel_perception/config/pth2engine.py': '将 BiSeNetV2 PyTorch 权重包装并导出 ONNX/TensorRT 引擎。',
        'src/wheel_perception/config/seg_trt.py': '将 MMSegmentation 模型适配为推理模块并导出 TensorRT 引擎。',
        'src/wheel_perception/config/check_onnx.py': '检查 ONNX 模型结构、输入输出及 ONNX Runtime 推理结果。',
        'src/wheel_perception/config/xtx_trt.py': '通过 trtexec 命令将 ONNX 文件构建为 TensorRT engine。',
        'src/wheel_msgs/msg/ObstacleInfo.msg': '定义单个三维障碍物的位置、距离和尺寸消息。',
        'src/wheel_msgs/msg/PerceptionOutput.msg': '定义感知到控制层的聚合消息，包含最近障碍物、道路边界、道路航向和障碍物列表。',
        'src/wheel_phone_gateway/config/gateway.yaml': '配置手机调试网关的图像、感知和控制话题及发布频率。',
        'src/wheel_phone_gateway/launch/debug_viz_gateway.launch.py': '启动轮椅感知管线和手机端调试可视化网关。',
    }
    if path in special:
        return special[path]
    if 'pinhole_camera_model' in p:
        return '实现或声明 ROS CameraInfo 与 OpenCV 针孔相机模型之间的标定、投影、畸变校正和 ROI 变换。'
    if 'stereo_camera_model' in p:
        return '实现或声明双目相机模型、视差重投影矩阵及视差到三维点云的转换。'
    if '/test/' in p or 'utest' in p:
        return f'测试 {name} 所覆盖的相机模型或图像几何行为。'
    if 'opencv_tests' in p:
        return f'ROS 2 OpenCV 集成测试组件 {name}，用于图像发布、订阅或人脸检测验证。'
    if name == 'CMakeLists.txt':
        return '定义对应 ROS 2 包的编译目标、依赖、测试及安装规则。'
    if name == 'package.xml':
        return '声明对应 ROS 2 包的元数据、构建工具及运行依赖。'
    if name == 'setup.py' or name == 'setup.cfg':
        return '配置 Python ROS 2 包的安装、资源索引和控制台入口。'
    if name == '__init__.py':
        return '标记 Python 包并提供包级初始化入口。'
    if 'visibility_control' in p:
        return '定义跨平台共享库符号导入、导出和可见性宏。'
    if 'mainpage.dox' in p:
        return '提供该 ROS 图像处理包的 Doxygen 首页和功能概述。'
    if '/resource/' in p:
        return 'ROS 2 ament 资源索引标记文件，用于发现 Python 包。'
    if 'global_flags' in p:
        return '集中定义感知模块使用的全局运行标志和状态开关。'
    if 'mask_postprocess.hpp' in p:
        return '声明语义分割掩膜 CUDA 后处理入口。'
    return f'{name} 是项目中的运行、构建或支持文件。'


def file_tags(path, category):
    p = path.lower()
    if category == 'config': return ['configuration', 'ros2', 'runtime']
    if category == 'docs': return ['documentation', 'build-system', 'ros2']
    if 'test' in p or 'utest' in p: return ['test', 'validation', 'quality']
    if 'dwa' in p or 'controller_node' in p: return ['local-planner', 'dwa', 'control']
    if 'lqr' in p: return ['lqr', 'control', 'trajectory-tracking']
    if p.endswith('.cu'): return ['cuda', 'gpu', 'perception']
    if 'zed' in p: return ['zed', 'rgbd', 'sensor']
    if 'launch' in p: return ['entry-point', 'ros2', 'launch']
    if 'msg/' in p: return ['type-definition', 'ros2', 'message']
    if 'opencv' in p or 'image_geometry' in p: return ['opencv', 'camera-model', 'vision']
    if 'dataset' in p: return ['dataset', 'offline-test', 'ros2']
    if 'trt' in p or 'engine' in p or 'onnx' in p: return ['tensorrt', 'inference', 'model-conversion']
    return ['ros2', 'component', 'wheelchair']


def symbol_summary(name, path, kind):
    known = {
        'plan': '根据动态窗口采样候选速度，仿真并筛选安全且综合评分最高的局部轨迹。',
        'simulate': '使用差速机器人运动学模型在预测时域内积分生成候选轨迹。',
        'evaluate': '计算轨迹的道路航向、道路位置、障碍物净空、速度、平滑及停车相关代价。',
        'perceptionCallback': '消费融合感知结果，执行安全判定、DWA/巡航规划与 LQR 跟踪控制。',
        'updateDwaActivation': '根据有效障碍点数量和滞回阈值更新 DWA 激活状态。',
        'makeCruiseTrajectory': '在未激活 DWA 时生成沿道路方向的巡航参考轨迹。',
        'publishStop': '发布零速度并清理控制历史，执行安全停车。',
        'publishVisualization': '发布候选路径、最佳路径及停车标记供 RViz 观察。',
        'update_loop': '执行相机取流、AI 推理、点云融合、道路分析与话题发布的主循环。',
        'analyze_scene': '从障碍物和道路边缘估计最近距离、道路方向、左右边界与动态目标距离。',
        'publish_perception_msg': '将场景指标和障碍物列表封装为 PerceptionOutput 消息。',
        'processBlindZoneOnGPU': '将语义与点云投影到 BEV 历史缓冲并执行盲区逻辑。',
        'filterCloud': '在 GPU 上过滤并体素化点云，提取规划所需障碍物。',
        'infer': '执行一次 TensorRT 语义分割推理并返回 GPU 输出。',
        'grab': '抓取一帧 ZED 图像、深度、点云和位姿数据。',
        'retrievePointCloud': '将 ZED GPU 点云转换并填充 ROS PointCloud2 消息。',
        'main': '解析运行参数并启动对应 ROS 2 节点或离线工具。',
        'generate_launch_description': '构造并返回 ROS 2 LaunchDescription，声明参数并启动相关节点。',
    }
    if name in known: return known[name]
    return f'{kind} {name} 实现 {Path(path).name} 中的核心职责，并供该模块内部或外部调用。'


def add_symbol_nodes(nodes, edges, result):
    path = result['path']; file_id = f'file:{path}'
    exported = {e['name'] for e in result.get('exports') or []}
    functions = {}
    for f in result.get('functions', []):
        old = functions.get(f['name'])
        if old is None or f['endLine'] - f['startLine'] > old['endLine'] - old['startLine']:
            functions[f['name']] = f
    for name, f in functions.items():
        if f['endLine'] - f['startLine'] + 1 < 10 and name not in exported:
            continue
        nid = f'function:{path}:{name}'
        nodes.append({'id':nid,'type':'function','name':name,'filePath':path,
                      'lineRange':[f['startLine'],f['endLine']],
                      'summary':symbol_summary(name,path,'函数'),
                      'tags':['function','implementation','module-api'],
                      'complexity':complexity(f['endLine']-f['startLine']+1)})
        edges.append({'source':file_id,'target':nid,'type':'contains','direction':'forward','weight':1.0})
        if name in exported:
            edges.append({'source':file_id,'target':nid,'type':'exports','direction':'forward','weight':0.8})
    for c in result.get('classes', []):
        length=c['endLine']-c['startLine']+1
        if length < 20 and len(c.get('methods',[])) < 2 and c['name'] not in exported:
            continue
        nid=f'class:{path}:{c["name"]}'
        nodes.append({'id':nid,'type':'class','name':c['name'],'filePath':path,
                      'lineRange':[c['startLine'],c['endLine']],
                      'summary':symbol_summary(c['name'],path,'类'),
                      'tags':['class','component','module-api'],
                      'complexity':complexity(length)})
        edges.append({'source':file_id,'target':nid,'type':'contains','direction':'forward','weight':1.0})
        if c['name'] in exported:
            edges.append({'source':file_id,'target':nid,'type':'exports','direction':'forward','weight':0.8})


MANUAL_CLASSES = {
 'src/vision_opencv/image_geometry/include/image_geometry/pinhole_camera_model.h': [('Exception',15,20),('PinholeCameraModel',25,406)],
 'src/vision_opencv/image_geometry/include/image_geometry/stereo_camera_model.h': [('StereoCameraModel',13,151)],
 'src/wheel_perception/include/wheel_perception/core/lqr_controller.hpp': [('LqrController',11,149)],
}


def extra_edges(idx):
    def e(s,t,typ,w): return {'source':s,'target':t,'type':typ,'direction':'forward','weight':w}
    if idx==11:
        return [
          e('file:src/vision_opencv/image_geometry/src/pinhole_camera_model.cpp','file:src/vision_opencv/image_geometry/include/image_geometry/pinhole_camera_model.h','depends_on',0.6),
          e('file:src/vision_opencv/image_geometry/src/stereo_camera_model.cpp','file:src/vision_opencv/image_geometry/include/image_geometry/stereo_camera_model.h','depends_on',0.6),
          e('file:src/vision_opencv/image_geometry/src/stereo_camera_model.cpp','file:src/vision_opencv/image_geometry/include/image_geometry/pinhole_camera_model.h','depends_on',0.6),
          e('file:src/vision_opencv/image_geometry/include/image_geometry/pinhole_camera_model.h','file:src/vision_opencv/image_geometry/test/utest.cpp','tested_by',0.5),
          e('file:src/vision_opencv/image_geometry/include/image_geometry/pinhole_camera_model.h','file:src/vision_opencv/image_geometry/test/utest_equi.cpp','tested_by',0.5),
        ]
    if idx==12:
        return [
          e('file:src/wheel_perception/dwa_controller/src/DWAPlanner.cpp','file:src/wheel_perception/dwa_controller/include/wheel_perception/dwa_controller/DWAPlanner.hpp','depends_on',0.6),
          e('file:src/wheel_perception/src/controller_node.cpp','file:src/wheel_perception/dwa_controller/include/wheel_perception/dwa_controller/DWAPlanner.hpp','depends_on',0.6),
          e('file:src/wheel_perception/src/controller_node.cpp','file:src/wheel_perception/include/wheel_perception/core/lqr_controller.hpp','depends_on',0.6),
          e('file:src/wheel_perception/src/fusion_node.cpp','file:src/wheel_perception/include/wheel_perception/core/ai_engine.hpp','depends_on',0.6),
          e('file:src/wheel_perception/src/fusion_node.cpp','file:src/wheel_perception/include/wheel_perception/core/obstacle_fusion.hpp','depends_on',0.6),
          e('file:src/wheel_perception/src/fusion_node.cpp','file:src/wheel_perception/include/wheel_perception/core/zed_driver.hpp','depends_on',0.6),
          e('file:src/wheel_perception/dwa_controller/src/DWAPlanner.cpp','file:src/wheel_perception/test/test_dwa_planner.cpp','tested_by',0.5),
          e('file:src/wheel_perception/launch/run_launch.py','file:src/wheel_perception/src/fusion_node.cpp','depends_on',0.6),
          e('file:src/wheel_perception/launch/run_launch.py','file:src/wheel_perception/src/controller_node.cpp','depends_on',0.6),
        ]
    return [
      e('file:tools/convert_text6_stereo_dataset.py','config:tools/assumed_zed_hd720_calibration.json','depends_on',0.6),
    ]


for idx in (11,12,13):
    batch = next(b for b in BATCHES if b['batchIndex']==idx)
    extraction = json.loads((UA/f'tmp/ua-file-extract-results-{idx}.json').read_text())
    by_path={r['path']:r for r in extraction['results']}
    nodes=[]; edges=[]
    for f in batch['files']:
        path=f['path']; category=f['fileCategory']
        ntype='config' if category=='config' else ('document' if category=='docs' else 'file')
        prefix=ntype
        lines=by_path.get(path,{}).get('nonEmptyLines',f['sizeLines'])
        nodes.append({'id':f'{prefix}:{path}','type':ntype,'name':Path(path).name,
                      'filePath':path,'summary':file_summary(path),
                      'tags':file_tags(path,category),'complexity':complexity(lines)})
        if path in by_path:
            add_symbol_nodes(nodes,edges,by_path[path])
        for name,start,end in MANUAL_CLASSES.get(path,[]):
            nid=f'class:{path}:{name}'
            if not any(n['id']==nid for n in nodes):
                nodes.append({'id':nid,'type':'class','name':name,'filePath':path,
                              'lineRange':[start,end], 'summary':symbol_summary(name,path,'类'),
                              'tags':['class','camera-model','module-api'],
                              'complexity':complexity(end-start+1)})
                edges.append({'source':f'file:{path}','target':nid,'type':'contains','direction':'forward','weight':1.0})
                edges.append({'source':f'file:{path}','target':nid,'type':'exports','direction':'forward','weight':0.8})
    # Import map is empty in all three assigned batches; still enumerate deterministically.
    for path, targets in batch['batchImportData'].items():
        for target in targets:
            edges.append({'source':f'file:{path}','target':f'file:{target}','type':'imports','direction':'forward','weight':0.7})
    valid_ids={n['id'] for n in nodes}
    for edge in extra_edges(idx):
        if edge['source'] in valid_ids or edge['source'].startswith(('file:','config:','document:')):
            edges.append(edge)
    # De-duplicate edges while preserving order.
    seen=set(); unique=[]
    for edge in edges:
        key=(edge['source'],edge['target'],edge['type'])
        if key not in seen:
            seen.add(key); unique.append(edge)
    edges=unique
    # Split exactly according to file-analyzer limits, grouping alphabetically by filePath.
    parts=max((len(nodes)+59)//60,(len(edges)+119)//120,1)
    files=sorted(f['path'] for f in batch['files'])
    chunk=(len(files)+parts-1)//parts
    for part in range(parts):
        chosen=set(files[part*chunk:(part+1)*chunk])
        pn=[n for n in nodes if n.get('filePath') in chosen]
        pids={n['id'] for n in pn}
        pe=[e for e in edges if e['source'] in pids]
        suffix='' if parts==1 else f'-part-{part+1}'
        out=UA/f'intermediate/batch-{idx}{suffix}.json'
        out.write_text(json.dumps({'nodes':pn,'edges':pe},ensure_ascii=False,indent=2)+'\n')
    print(idx, 'parts',parts,'nodes',len(nodes),'edges',len(edges),'skipped',len(extraction.get('filesSkipped',[])))
