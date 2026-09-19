#include "wheel_perception/core/zed_driver.hpp"

// ZED SDK (仅在 .cpp 内部包含，隔绝污染)
#include <sl/Camera.hpp>

// 标准库与 ROS 依赖
#include <iostream>
#include <cuda_runtime.h>
#include <opencv2/imgproc/imgproc.hpp> 
#include <sensor_msgs/point_cloud2_iterator.hpp> // 必须包含
#include <rclcpp/rclcpp.hpp> // 用于获取时间戳
#include <unordered_set>  // 对应 std::unordered_set
#include <cmath>          // 对应 std::isfinite
#include <algorithm> // 用于 std::min/max
#include "wheel_perception/core/zed_driver.hpp"


namespace wheel_perception {
namespace core {

// =============================================================================
// PIMPL 实现类 (隐藏 ZED SDK 细节)
// =============================================================================
struct ZedDriver::Impl {
    sl::Camera zed;
    sl::InitParameters init_params;
    sl::RuntimeParameters runtime_params;

    // ZED 显存容器 (GPU Memory) - 用于 Zero-Copy
    sl::Mat mat_rgb_gpu;
    sl::Mat mat_depth_gpu;
    sl::Mat mat_cloud_gpu;
    
    // CPU 缓存容器 - 用于 ROS 发布数据
    sl::Mat mat_rgb_cpu;
    sl::Mat mat_cloud_cpu; 

    bool odometry_enabled = false;
    sl::Transform accumulated_odom_pose;
    sl::Pose camera_delta_pose;

    Impl() {
        // 默认参数初始化
        init_params.camera_resolution = sl::RESOLUTION::HD720;
        init_params.depth_mode = sl::DEPTH_MODE::PERFORMANCE; 
        init_params.coordinate_units = sl::UNIT::METER;
        // ROS 坐标系: X 前, Y 左, Z 上
        init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
        init_params.sdk_gpu_id = 0;
    }
};

namespace {

void clearOdometryFrame(ZedGpuFrame& frame) {
    frame.pose_x = 0.0f;
    frame.pose_y = 0.0f;
    frame.pose_z = 0.0f;
    frame.quat_x = 0.0f;
    frame.quat_y = 0.0f;
    frame.quat_z = 0.0f;
    frame.quat_w = 1.0f;
    frame.linear_velocity_x = 0.0f;
    frame.linear_velocity_y = 0.0f;
    frame.linear_velocity_z = 0.0f;
    frame.angular_velocity_x = 0.0f;
    frame.angular_velocity_y = 0.0f;
    frame.angular_velocity_z = 0.0f;
    frame.pose_covariance.fill(0.0);
    frame.twist_covariance.fill(0.0);
    frame.odometry_valid = false;
}

template<typename DriverImpl>
bool populateOdometryFrame(DriverImpl& impl, ZedGpuFrame& frame) {
    clearOdometryFrame(frame);
    if (!impl.odometry_enabled) return false;

    const auto tracking_state =
        impl.zed.getPosition(impl.camera_delta_pose, sl::REFERENCE_FRAME::CAMERA);
    const auto tracking_status = impl.zed.getPositionalTrackingStatus();
    if (tracking_state != sl::POSITIONAL_TRACKING_STATE::OK ||
        tracking_status.odometry_status != sl::ODOMETRY_STATUS::OK ||
        !impl.camera_delta_pose.valid) {
        return false;
    }

    impl.accumulated_odom_pose =
        impl.accumulated_odom_pose * impl.camera_delta_pose.pose_data;
    auto orientation = impl.accumulated_odom_pose.getOrientation();
    orientation.normalise();
    impl.accumulated_odom_pose.setOrientation(orientation);

    const auto translation = impl.accumulated_odom_pose.getTranslation();
    frame.pose_x = translation.tx;
    frame.pose_y = translation.ty;
    frame.pose_z = translation.tz;
    frame.quat_x = orientation.ox;
    frame.quat_y = orientation.oy;
    frame.quat_z = orientation.oz;
    frame.quat_w = orientation.ow;

    frame.linear_velocity_x = impl.camera_delta_pose.twist[0];
    frame.linear_velocity_y = impl.camera_delta_pose.twist[1];
    frame.linear_velocity_z = impl.camera_delta_pose.twist[2];
    frame.angular_velocity_x = impl.camera_delta_pose.twist[3];
    frame.angular_velocity_y = impl.camera_delta_pose.twist[4];
    frame.angular_velocity_z = impl.camera_delta_pose.twist[5];
    for (std::size_t index = 0; index < frame.pose_covariance.size(); ++index) {
        frame.pose_covariance[index] = impl.camera_delta_pose.pose_covariance[index];
        frame.twist_covariance[index] = impl.camera_delta_pose.twist_covariance[index];
    }
    frame.odometry_valid = true;
    return true;
}

}  // namespace

// =============================================================================
// 构造与析构
// =============================================================================
ZedDriver::ZedDriver() : pimpl_(std::make_unique<Impl>()) {}
ZedDriver::~ZedDriver() { close(); }

// =============================================================================
// Open / Close / Check
// =============================================================================
bool ZedDriver::open(const Config& config){
    if (pimpl_->zed.isOpened()) return true;

    // 1. 设置分辨率
    switch (config.resolution_id) {
        case 0: pimpl_->init_params.camera_resolution = sl::RESOLUTION::HD2K; break;
        case 1: pimpl_->init_params.camera_resolution = sl::RESOLUTION::HD1080; break;
        case 2: pimpl_->init_params.camera_resolution = sl::RESOLUTION::HD720; break;
        case 3: pimpl_->init_params.camera_resolution = sl::RESOLUTION::VGA; break;
        default: pimpl_->init_params.camera_resolution = sl::RESOLUTION::HD720;
    }

    // 2. 应用参数
    pimpl_->init_params.camera_fps = config.fps;
    pimpl_->init_params.depth_minimum_distance = config.depth_min;
    pimpl_->init_params.depth_maximum_distance = config.depth_max;
    pimpl_->init_params.camera_image_flip = config.flip_camera;
    pimpl_->odometry_enabled = config.enable_odometry;
    
    // 强制转换为 SDK 枚举
    // pimpl_->init_params.coordinate_system = static_cast<sl::COORDINATE_SYSTEM>(config.coordinate_system);

    // SVO 回放支持
    if (!config.svo_path.empty()) {
        pimpl_->init_params.input.setFromSVOFile(config.svo_path.c_str());
    }

    // 3. 打开相机
    sl::ERROR_CODE err = pimpl_->zed.open(pimpl_->init_params);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "[ZedDriver] Open Failed: " << sl::toString(err) << std::endl;
        return false;
    }

    // 4. 启用位置跟踪以获取里程计数据
    sl::PositionalTrackingParameters tracking_parameters;
    tracking_parameters.enable_area_memory = true;  
    tracking_parameters.enable_imu_fusion = true;   
    tracking_parameters.set_floor_as_origin = false; 
    
    sl::ERROR_CODE err_track = pimpl_->zed.enablePositionalTracking(tracking_parameters);
    if (err_track != sl::ERROR_CODE::SUCCESS) {
        std::cerr << "[ZedDriver] Enable Positional Tracking Failed: " << sl::toString(err_track) << std::endl;
    } else {
        std::cout << "[ZedDriver] Positional Tracking Enabled!" << std::endl;
    }
    pimpl_->accumulated_odom_pose.setIdentity();
    return true;
}

void ZedDriver::close(){
    if (pimpl_->zed.isOpened()){
        pimpl_->zed.close();
    }
}

bool ZedDriver::isOpened() const {
    return pimpl_->zed.isOpened();
}

// =============================================================================
// 核心: Grab (获取 GPU 数据指针)
// =============================================================================
bool ZedDriver::grab(ZedGpuFrame& out_frame){
    if (!pimpl_->zed.isOpened()) return false;

    // 1. 触发 ZED 计算
    sl::ERROR_CODE err = pimpl_->zed.grab(pimpl_->runtime_params);
    if (err != sl::ERROR_CODE::SUCCESS){
        return false;
    }

    // 2. Retrieve 数据到显存 (Zero-Copy)
    pimpl_->zed.retrieveImage(pimpl_->mat_rgb_gpu, sl::VIEW::LEFT, sl::MEM::GPU);
    pimpl_->zed.retrieveMeasure(pimpl_->mat_depth_gpu, sl::MEASURE::DEPTH, sl::MEM::GPU);
    pimpl_->zed.retrieveMeasure(pimpl_->mat_cloud_gpu, sl::MEASURE::XYZRGBA, sl::MEM::GPU);

    // 3. 填充输出结构体
    out_frame.timestamp_ns = pimpl_->zed.getTimestamp(sl::TIME_REFERENCE::IMAGE).getNanoseconds();
    out_frame.width  = pimpl_->mat_rgb_gpu.getWidth();
    out_frame.height = pimpl_->mat_rgb_gpu.getHeight();

    // 4. 与当前 RGB-D 帧同时获取位姿、Twist 和协方差。
    populateOdometryFrame(*pimpl_, out_frame);

    // GPU 指针
    out_frame.rgb_ptr_dev   = pimpl_->mat_rgb_gpu.getPtr<sl::uchar4>(sl::MEM::GPU);
    out_frame.depth_ptr_dev = pimpl_->mat_depth_gpu.getPtr<sl::float1>(sl::MEM::GPU);
    out_frame.cloud_ptr_dev = pimpl_->mat_cloud_gpu.getPtr<sl::float4>(sl::MEM::GPU);

    // Step (字节步长)
    out_frame.rgb_step_bytes   = pimpl_->mat_rgb_gpu.getStepBytes(sl::MEM::GPU);
    out_frame.depth_step_bytes = pimpl_->mat_depth_gpu.getStepBytes(sl::MEM::GPU);
    out_frame.cloud_step_bytes = pimpl_->mat_cloud_gpu.getStepBytes(sl::MEM::GPU);

    return true;
}

// =============================================================================
// CPU 接口 1: 获取图像 (用于调试显示)
// =============================================================================
void ZedDriver::retrieveImage(cv::Mat& out_img) {
    if (!pimpl_->zed.isOpened()) return;

    // SDK 显存 -> 系统内存 (CPU)，这是一次内存拷贝
    pimpl_->zed.retrieveImage(pimpl_->mat_rgb_cpu, sl::VIEW::LEFT, sl::MEM::CPU);

    // 包装为 cv::Mat (此时是 BGRA)
    cv::Mat sl_wrapper(
        pimpl_->mat_rgb_cpu.getHeight(),
        pimpl_->mat_rgb_cpu.getWidth(),
        CV_8UC4,
        pimpl_->mat_rgb_cpu.getPtr<sl::uchar1>(sl::MEM::CPU),
        pimpl_->mat_rgb_cpu.getStepBytes()
    );

    // 转为 BGR
    if (!sl_wrapper.empty()) {
        cv::cvtColor(sl_wrapper, out_img, cv::COLOR_BGRA2BGR);
    }
}


// =============================================================================
// CPU 接口 2: 获取 ROS 点云 (坐标系修正版)
// =============================================================================
void ZedDriver::retrievePointCloud(sensor_msgs::msg::PointCloud2& msg, const std::string& frame_id) {
    if (!pimpl_->zed.isOpened()) return;

    pimpl_->zed.retrieveMeasure(pimpl_->mat_cloud_cpu, sl::MEASURE::XYZRGBA, sl::MEM::CPU);
    if (!pimpl_->mat_cloud_cpu.isInit()) return;

    int w = pimpl_->mat_cloud_cpu.getWidth();
    int h = pimpl_->mat_cloud_cpu.getHeight();

    // ================= [参数调整] =================
    const float VOXEL_SIZE = 0.05f;
    const float INV_VOXEL_SIZE = 1.0f / VOXEL_SIZE;
    const int SKIP_STEP = 2;
    
    // [修复] 椭圆阈值 (对应 avoid_lib 的 6x^2 + 100y^2 < 36)
    // 注意：这里我们适当放宽一点，让 Rviz 能看到更多轮廓
    const float ELLIPSE_THRES = 50.0f; 

    // 准备 ROS 消息
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = frame_id;
    msg.is_bigendian = false;
    msg.is_dense = false;
    msg.height = 1; 
    msg.width = (w * h) / 10; 

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(msg.width);

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(msg, "rgb");

    std::unordered_set<long long> voxel_grid;
    voxel_grid.reserve(msg.width); 

    float* data_ptr = pimpl_->mat_cloud_cpu.getPtr<float>(sl::MEM::CPU);
    int step_float = pimpl_->mat_cloud_cpu.getStepBytes() / sizeof(float);

    int valid_count = 0;

    for (int r = 0; r < h; r += SKIP_STEP) {
        float* row_ptr = data_ptr + r * step_float;
        for (int c = 0; c < w; c += SKIP_STEP) {
            float* pt = row_ptr + c * 4;
            
            // [关键修正] 在 ROS 坐标系下 (RIGHT_HANDED_Z_UP_X_FWD):
            // pt[0] 是 X (前方距离)
            // pt[1] 是 Y (左右偏移)
            // pt[2] 是 Z (垂直高度)
            float x = pt[0]; 
            float y = pt[1]; 
            float z = pt[2];

            // 1. NaN 检查
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

            // 2. 范围剪裁 (修正轴向)
            // 距离：只看前方 0.2米 到 1米 的点 (用 X 轴!)  
            if (x < 0.0f || x > 5.0f) continue; 
            
            // 高度：只看地面以上 -1米 到 3米 的点 (用 Z 轴!)  
            // 这能把天花板和地底深处的噪点去掉
            if (z < -0.1f || z > 2.0f) continue;

            // 左右：只看左右各 5米 (用 Y 轴!)   
            if (y < -3.0f || y > 3.0f) continue;

            // 3. 椭圆滤波 (修正公式)
            // 应该是：6 * 前方^2 + 100 * 左右^2
            // 即: 6 * x^2 + 100 * y^2
            if ((6.0f * x * x + 100.0f * y * y) > ELLIPSE_THRES) continue;

            // // 4. 体素哈希 (不变)
            long long ix = static_cast<long long>(x * INV_VOXEL_SIZE);
            long long iy = static_cast<long long>(y * INV_VOXEL_SIZE);
            long long iz = static_cast<long long>(z * INV_VOXEL_SIZE);
            long long key = (ix * 73856093) ^ (iy * 19349663) ^ (iz * 83492791);

            if (voxel_grid.count(key)) continue;
            voxel_grid.insert(key);

            *iter_x = x; *iter_y = y; *iter_z = z;
            
            float color_val = pt[3];
            uint8_t* c_ptr = (uint8_t*)&color_val;
            iter_rgb[0] = c_ptr[0]; iter_rgb[1] = c_ptr[1]; iter_rgb[2] = c_ptr[2];

            ++iter_x; ++iter_y; ++iter_z; ++iter_rgb;
            valid_count++;
            if (valid_count >= (int)msg.width) break;
        }
        if (valid_count >= (int)msg.width) break;
    }

    modifier.resize(valid_count);
    msg.width = valid_count;
    msg.row_step = msg.width * msg.point_step;
    msg.data.resize(msg.row_step);
}

// =============================================================================
// 其他接口
// =============================================================================
void* ZedDriver::getRawImageGPUPtr() {
    if (!pimpl_->zed.isOpened()) return nullptr;
    return pimpl_->mat_rgb_gpu.getPtr<sl::uchar1>(sl::MEM::GPU);
}

ZedDriver::CameraParams ZedDriver:: getCameraParams() const {
    CameraParams params = {0,0,0,0,0};
    if (pimpl_->zed.isOpened()) {
        auto calib = pimpl_->zed.getCameraInformation().camera_configuration.calibration_parameters.left_cam;
        params.fx = calib.fx;
        params.fy = calib.fy;
        params.cx = calib.cx;
        params.cy = calib.cy;
        // 获取基线
        params.baseline = pimpl_->zed.getCameraInformation().camera_configuration.calibration_parameters.stereo_transform.getTranslation().x;
    }
    return params;
}

} // namespace core
} // namespace wheel_perception
