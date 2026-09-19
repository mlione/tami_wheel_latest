import fs from 'node:fs';
import path from 'node:path';

const root = process.cwd();
const ua = path.join(root, '.ua');
const input = i => JSON.parse(fs.readFileSync(path.join(ua, 'tmp', `ua-file-analyzer-input-${i}.json`), 'utf8'));
const extract = i => JSON.parse(fs.readFileSync(path.join(ua, 'tmp', `ua-file-extract-results-${i}.json`), 'utf8'));
const edge = (source, target, type, weight) => ({source, target, type, direction: 'forward', weight});
const complexity = lines => lines > 200 ? 'complex' : lines >= 50 ? 'moderate' : 'simple';
const basename = p => p.split('/').at(-1);

const fileMeta = {
  'src/vision_opencv/cv_bridge/src/module.cpp': ['为 cv_bridge 构建 Boost.Python 扩展入口，将 NumPy/OpenCV 图像转换、颜色编码转换及 OpenCV 类型查询暴露给 Python。', ['python-绑定', '图像转换', 'opencv', 'ros2'], '通过 Boost.Python 宏注册扩展模块，并在 Python 对象、cv::Mat 与 ROS 图像编码之间桥接。'],
  'src/vision_opencv/cv_bridge/src/module.hpp': ['声明 cv_bridge Python 扩展共享的 NumPy C API、OpenCV 转换函数和兼容宏，是不同 OpenCV 实现文件的公共接口。', ['接口声明', '兼容层', 'numpy', 'opencv'], '头文件集中管理 Python/NumPy C API 初始化与 cv::Mat 转换声明。'],
  'src/vision_opencv/cv_bridge/src/module_opencv2.cpp': ['实现面向 OpenCV 2 的 NumPy 与 cv::Mat 双向内存桥接，包括引用计数适配、数组布局检查和转置处理。', ['兼容层', '内存管理', 'numpy', 'opencv2'], '通过自定义 MatAllocator 复用 NumPy 缓冲区，并手工协调 Python 引用计数。'],
  'src/vision_opencv/cv_bridge/src/module_opencv3.cpp': ['实现面向 OpenCV 3/4 API 的 NumPy 与 cv::Mat 转换层，处理 GIL、数据类型、步幅、连续性和多通道数组。', ['兼容层', '内存管理', 'numpy', 'opencv3'], '使用 RAII 管理 Python GIL，并以 UMatData 和自定义 MatAllocator 维持零拷贝所有权。'],
  'src/vision_opencv/cv_bridge/src/pycompat.hpp': ['提供 Python 2/3 与 NumPy C API 的条件编译兼容定义，统一整数、字符串和模块初始化接口。', ['兼容层', 'python-c-api', 'numpy', '条件编译'], '大量预处理宏用于抹平 Python 2 与 Python 3 C API 差异。'],
  'src/vision_opencv/image_geometry/image_geometry/__init__.py': ['作为 image_geometry Python 包入口，重新导出单目和双目相机模型。', ['入口点', '包导出', '相机几何'], null],
  'src/vision_opencv/image_geometry/image_geometry/cameramodels.py': ['实现 ROS CameraInfo 驱动的单目与双目相机几何模型，支持校正、像素与三维投影、视差和深度换算。', ['相机模型', '几何投影', '图像校正', 'ros2'], '使用 NumPy 矩阵保存 K、D、R、P、Q，并调用 OpenCV 完成去畸变与重映射。'],
  'src/vision_opencv/image_geometry/test/directed.py': ['以固定相机标定数据验证单目校正、双目投影互逆关系以及像素位移与空间位移换算。', ['测试', '相机标定', '双目几何'], null],
  'src/wheel_phone_gateway/test/test_protocol.py': ['覆盖手机网关的 WebSocket 帧编码、图像缩放、二进制传输、launch 请求白名单和进程组生命周期。', ['测试', 'websocket', '进程管理', '安全验证'], null],
  'src/wheel_phone_gateway/wheel_phone_gateway/gateway_node.py': ['实现 ROS 2 手机调试网关：订阅调试图像、压缩并经 WebSocket 推流，同时通过 HTTP API 受控启停感知 launch。', ['ros2-节点', 'websocket', '图像流', 'http-服务'], '将 ROS 回调、HTTP 线程、广播线程和 launch 监控线程解耦，并用锁保护共享帧与客户端集合。'],
  'src/wheel_phone_gateway/wheel_phone_gateway/launch_process.py': ['提供受白名单约束的 ROS 2 launch 进程管理器，以进程组方式实现幂等启动、状态回收和分级停止。', ['进程管理', '安全控制', 'ros2-launch', '并发'], '通过 start_new_session 创建独立进程组，并按 SIGINT、SIGTERM、SIGKILL 顺序升级停止。'],
  'README.md': ['项目总览文档，说明载人轮椅 DWA + LQR 导航架构、感知与控制链路、构建运行、可视化及安全边界。', ['文档', '项目概览', '自主导航', '安全'], null],
  'dwa+lqr_v0.1.md': ['记录从固定策略避障与 LQR 巡线演进到 ObstacleFusion、DWA 局部规划、LQR 跟踪和 Safety Layer 的首版改造。', ['文档', '版本记录', 'dwa', 'lqr'], null],
  'dwa+lqr_v0.2.md': ['说明 v0.2 对 DWA 激活、轨迹平滑、点云 ROI、速度来源、候选保持和可视化的增量优化及调参方法。', ['文档', '版本记录', '参数调优', '轨迹平滑'], null],
  'dwa+lqr_v0.3.2.md': ['记录 v0.3.2 的精确参数与代码差异、DWA 调试快照、数据集回放结论、风险和实车前检查。', ['文档', '版本记录', '调试快照', '风险评估'], null],
  'dwa+lqr_v0.3.md': ['总结 v0.3 的低速滚动转弯、最小转弯半径约束、停车可视化和数据集工具，并给出诊断结论与后续优先级。', ['文档', '版本记录', '运动规划', '诊断'], null],
  'symlink_install_manifest.txt': ['列出 wheel_perception 软链接安装空间中的库、头文件、launch、模型、配置与 ament 元数据路径，供安装产物核查。', ['文档', '安装清单', '构建产物'], null],
  'wheel_cuda_road_width_handoff.md': ['交接道路宽度估计与调试可视化改动，涵盖边框伪边界过滤、采样范围、参数影响、已知状态锁死问题和验证建议。', ['文档', '工程交接', '道路宽度', '调试'], null],
  '实车测试指令.md': ['提供载人轮椅实车测试的分阶段操作手册，覆盖硬件检查、感知规划验证、RViz、蓝牙底盘、数据录制、故障和急停。', ['文档', '实车测试', '安全', '验收'], null],
  'src/vision_opencv/.travis.sh': ['在 Travis 容器内安装 ROS 依赖、建立 catkin 工作区，并依次执行构建、测试、安装模式重建及计时折叠。', ['ci-cd', '构建脚本', 'ros', 'catkin'], 'Bash 函数封装 Travis 日志折叠和纳秒级阶段计时，主流程采用 set -e 快速失败。'],
  'src/vision_opencv/.travis.yml': ['配置 Ubuntu Trusty 与 ROS Kinetic 的 Travis 任务，在 Xenial 容器中执行仓库自带的构建测试脚本并采集失败日志。', ['配置', 'ci-cd', 'travis', '容器化'], null],
  'src/vision_opencv/README.md': ['概述 vision_opencv 元仓库及 cv_bridge、image_geometry、opencv_tests 等 ROS 2 与 OpenCV 集成包。', ['文档', '项目概览', 'ros2', 'opencv'], null],
  'src/vision_opencv/pytest.ini': ['将 pytest 的 JUnit 报告格式固定为 xunit2，供测试报告工具兼容读取。', ['配置', '测试', 'pytest', 'junit'], null],
};

const symbolMeta = {
  cvtColor2Wrap: ['把 Python NumPy 图像包装为 cv_bridge 图像，执行目标编码转换后返回 Python 可用数组。', ['图像转换', 'python-绑定', 'opencv']],
  cvtColorForDisplayWrap: ['按显示选项执行颜色转换，并支持动态值域缩放与显式最小、最大像素值。', ['显示转换', '动态缩放', 'python-绑定']],
  CV_MAT_CNWrap: ['向 Python 暴露 OpenCV 类型编码中的通道数提取宏。', ['类型工具', 'opencv', 'python-绑定']],
  CV_MAT_DEPTHWrap: ['向 Python 暴露 OpenCV 类型编码中的像素深度提取宏。', ['类型工具', 'opencv', 'python-绑定']],
  BOOST_PYTHON_MODULE: ['初始化 cv_bridge_boost 扩展并注册类型查询、颜色转换与显示转换 API。', ['模块初始化', 'python-绑定', '入口点']],
  failmsg: ['格式化类型错误信息并设置 Python 异常状态，供数组转换失败路径复用。', ['错误处理', 'python-c-api', '工具函数']],
  PyAllowThreads: ['以 RAII 方式暂时释放 Python GIL，使 OpenCV 操作可在不持锁时运行。', ['raii', '线程控制', 'python-c-api']],
  '~PyAllowThreads': ['在作用域结束时恢复此前释放的 Python 线程状态。', ['raii', '线程控制', '资源清理']],
  PyEnsureGIL: ['以 RAII 方式确保当前线程持有 Python GIL。', ['raii', '线程控制', 'python-c-api']],
  '~PyEnsureGIL': ['在作用域结束时释放通过 PyGILState_Ensure 获取的 GIL 状态。', ['raii', '线程控制', '资源清理']],
  ArgInfo: ['携带参数名称与输出参数标志，为 OpenCV Python 数组转换提供诊断上下文。', ['参数元数据', '兼容层', '数据转换']],
  NumpyAllocator: ['在 NumPy 缓冲区与 OpenCV Mat 所有权之间协调分配、步幅及引用生命周期。', ['内存分配', 'numpy', 'opencv']],
  allocate: ['为 OpenCV 矩阵建立或委托 NumPy/UMatData 存储，并同步维度、通道与步幅。', ['内存分配', '数组布局', 'numpy']],
  deallocate: ['释放与 OpenCV 矩阵关联的 Python 对象引用或 UMatData。', ['内存释放', '引用计数', 'numpy']],
  convert_to_CvMat2: ['把 Python 对象转换为 cv::Mat，校验数据类型、维度和步幅并尽量复用底层内存。', ['数据转换', 'numpy', 'opencv']],
  pyopencv_to: ['完成 Python 标量、元组或 ndarray 到 cv::Mat 的全面转换，必要时复制或强制类型转换。', ['数据转换', '数组布局', '类型校验']],
  mkmat: ['将扁平数值序列构造成指定行列的 float64 NumPy 矩阵。', ['工具函数', '矩阵构造', 'numpy']],
  PinholeCameraModel: ['封装单目相机内外参、ROI 与 binning 修正，并提供图像校正和像素/三维射线投影。', ['相机模型', '几何投影', '图像校正']],
  StereoCameraModel: ['组合左右单目模型并构造重投影矩阵，提供双目像素、三维点、视差与深度换算。', ['双目视觉', '深度估计', '几何投影']],
  TestDirected: ['使用已知标定矩阵验证单目与双目相机模型的投影一致性和位移换算。', ['测试', '双目几何', '相机标定']],
  RecordingConnection: ['模拟 socket 连接并累计 sendall 数据，便于断言 WebSocket 帧的精确字节。', ['测试替身', 'websocket', '字节协议']],
  encode_websocket_frame: ['按载荷长度选择 WebSocket 基本、16 位或 64 位长度格式，生成服务端未掩码帧。', ['websocket', '协议编码', '序列化']],
  read_exact: ['循环读取 socket 直到满足指定字节数，并在对端关闭时报告连接错误。', ['网络io', 'socket', '错误处理']],
  resize_for_stream: ['将图像缩放为移动端输出尺寸，相同尺寸时直接复用输入数组。', ['图像处理', '缩放', '性能优化']],
  WebSocketClient: ['封装单个客户端的线程安全发送，支持 JSON、控制帧、二进制 JPEG 与帧元数据。', ['websocket', '并发', '消息传输']],
  DebugVizGateway: ['协调 ROS 图像订阅、JPEG 编码、HTTP/WebSocket 服务、客户端广播和 launch 状态监控。', ['ros2-节点', '网关服务', '并发', '图像流']],
  main: ['初始化 rclpy 和手机调试网关，运行事件循环并在中断或错误后有序释放资源。', ['入口点', 'ros2', '生命周期']],
  validate_launch_request: ['仅接受固定 perception launch 且拒绝自定义参数，阻止手机端注入任意命令。', ['输入校验', '安全控制', '白名单']],
  LaunchProcessManager: ['用互斥锁管理唯一 launch 进程组，提供状态查询、幂等启动及带超时升级的停止。', ['进程管理', '并发', '生命周期']],
  travis_time_start: ['开启 Travis 日志折叠并记录当前构建阶段的纳秒时间戳。', ['ci-cd', '计时', '日志']],
  travis_time_end: ['结束 Travis 日志折叠，计算并打印阶段耗时。', ['ci-cd', '计时', '日志']],
};

const testSummary = name => ({
  test_encode_short_text_frame: '验证短文本载荷使用 FIN 与 text opcode 的基本 WebSocket 帧格式。',
  test_encode_large_text_frame: '验证超过 65535 字节的文本载荷使用 64 位扩展长度字段。',
  test_resize_for_stream: '验证 720p 图像按配置缩放为 640×360。',
  test_send_binary_uses_binary_opcode: '验证 JPEG 数据使用 binary opcode 发送而非 base64 文本。',
  test_validate_launch_request_only_accepts_perception: '验证 launch 请求仅允许 perception 标识且拒绝自定义参数。',
  test_launch_manager_start_is_idempotent: '验证重复启动请求不会创建第二个 launch 进程。',
  test_launch_manager_reports_unexpected_exit: '验证异常退出的 launch 会被报告为 failed 并保留退出码。',
  test_launch_manager_stops_process_group: '验证停止操作向整个 launch 进程组发送 SIGINT。',
}[name] || '验证相关协议与进程管理行为。');

function buildCodeBatch(i) {
  const inp = input(i), ext = extract(i);
  const nodes = [], edges = [];
  for (const result of ext.results) {
    const [summary, tags, languageNotes] = fileMeta[result.path];
    const fileNode = {id:`file:${result.path}`, type:'file', name:basename(result.path), filePath:result.path, summary, tags, complexity:complexity(result.nonEmptyLines)};
    if (languageNotes) fileNode.languageNotes = languageNotes;
    nodes.push(fileNode);
    const exported = new Set((result.exports || []).map(x => x.name));
    for (const fn of result.functions || []) {
      if (fn.endLine - fn.startLine + 1 < 10 && !exported.has(fn.name)) continue;
      const meta = symbolMeta[fn.name] || [testSummary(fn.name), ['测试', '行为验证', '回归保护']];
      const id = `function:${result.path}:${fn.name}`;
      nodes.push({id, type:'function', name:fn.name, filePath:result.path, lineRange:[fn.startLine,fn.endLine], summary:meta[0], tags:meta[1], complexity:complexity(fn.endLine-fn.startLine+1)});
      edges.push(edge(`file:${result.path}`, id, 'contains', 1.0));
      if (exported.has(fn.name)) edges.push(edge(`file:${result.path}`, id, 'exports', 0.8));
    }
    for (const cls of result.classes || []) {
      if (cls.endLine - cls.startLine + 1 < 20 && (cls.methods || []).length < 2 && !exported.has(cls.name)) continue;
      const meta = symbolMeta[cls.name] || [testSummary(cls.name), ['测试', '行为验证', '回归保护']];
      const id = `class:${result.path}:${cls.name}`;
      nodes.push({id, type:'class', name:cls.name, filePath:result.path, lineRange:[cls.startLine,cls.endLine], summary:meta[0], tags:meta[1], complexity:complexity(cls.endLine-cls.startLine+1)});
      edges.push(edge(`file:${result.path}`, id, 'contains', 1.0));
      if (exported.has(cls.name)) edges.push(edge(`file:${result.path}`, id, 'exports', 0.8));
    }
  }
  for (const [source, targets] of Object.entries(inp.batchImportData)) {
    for (const target of targets) edges.push(edge(`file:${source}`, `file:${target}`, 'imports', 0.7));
  }
  if (i === 2) {
    edges.push(edge('file:src/vision_opencv/image_geometry/image_geometry/cameramodels.py','file:src/vision_opencv/image_geometry/test/directed.py','tested_by',0.5));
  }
  if (i === 3) {
    const test = 'file:src/wheel_phone_gateway/test/test_protocol.py';
    edges.push(edge('file:src/wheel_phone_gateway/wheel_phone_gateway/gateway_node.py',test,'tested_by',0.5));
    edges.push(edge('file:src/wheel_phone_gateway/wheel_phone_gateway/launch_process.py',test,'tested_by',0.5));
  }
  return {nodes, edges};
}

function buildDocsBatch() {
  const ext = extract(4), nodes = [], edges = [];
  const all = new Map(ext.results.map(result => [result.path, result]));
  for (const file of input(4).batchFiles) {
    const result = all.get(file.path) || {path:file.path, nonEmptyLines:file.sizeLines};
    const [summary,tags,languageNotes] = fileMeta[result.path];
    const n = {id:`document:${result.path}`,type:'document',name:basename(result.path),filePath:result.path,summary,tags,complexity:complexity(result.nonEmptyLines)};
    if (languageNotes) n.languageNotes = languageNotes;
    nodes.push(n);
  }
  edges.push(edge('document:README.md','document:dwa+lqr_v0.3.md','related',0.5));
  edges.push(edge('document:dwa+lqr_v0.1.md','document:dwa+lqr_v0.2.md','related',0.5));
  edges.push(edge('document:dwa+lqr_v0.2.md','document:dwa+lqr_v0.3.md','related',0.5));
  edges.push(edge('document:dwa+lqr_v0.3.md','document:dwa+lqr_v0.3.2.md','related',0.5));
  edges.push(edge('document:实车测试指令.md','document:README.md','related',0.5));
  edges.push(edge('document:wheel_cuda_road_width_handoff.md','document:dwa+lqr_v0.3.2.md','related',0.5));
  return {nodes,edges};
}

function buildInfraBatch() {
  const ext = extract(5), nodes = [], edges = [];
  const all = new Map(ext.results.map(r => [r.path,r]));
  all.set('src/vision_opencv/pytest.ini',{path:'src/vision_opencv/pytest.ini',fileCategory:'config',nonEmptyLines:2,functions:[],classes:[],exports:[]});
  for (const p of input(5).batchFiles.map(x=>x.path)) {
    const result = all.get(p);
    const [summary,tags,languageNotes] = fileMeta[p];
    const type = result.fileCategory === 'config' ? 'config' : result.fileCategory === 'docs' ? 'document' : 'file';
    const prefix = type;
    const n = {id:`${prefix}:${p}`,type,name:basename(p),filePath:p,summary,tags,complexity:complexity(result.nonEmptyLines)};
    if (languageNotes) n.languageNotes = languageNotes;
    nodes.push(n);
    const exported = new Set((result.exports||[]).map(x=>x.name));
    for (const fn of result.functions || []) {
      if (fn.endLine-fn.startLine+1<10 && !exported.has(fn.name)) continue;
      const meta = symbolMeta[fn.name];
      const id=`function:${p}:${fn.name}`;
      nodes.push({id,type:'function',name:fn.name,filePath:p,lineRange:[fn.startLine,fn.endLine],summary:meta[0],tags:meta[1],complexity:complexity(fn.endLine-fn.startLine+1)});
      edges.push(edge(`file:${p}`,id,'contains',1.0));
    }
  }
  edges.push(edge('config:src/vision_opencv/.travis.yml','file:src/vision_opencv/.travis.sh','configures',0.6));
  return {nodes,edges};
}

const batches = {1:buildCodeBatch(1),2:buildCodeBatch(2),3:buildCodeBatch(3),4:buildDocsBatch(),5:buildInfraBatch()};
for (const [i, graph] of Object.entries(batches)) {
  if (graph.nodes.length > 60 || graph.edges.length > 120) throw new Error(`批次 ${i} 需要分片`);
  fs.writeFileSync(path.join(ua,'intermediate',`batch-${i}.json`),JSON.stringify(graph,null,2)+'\n');
}
