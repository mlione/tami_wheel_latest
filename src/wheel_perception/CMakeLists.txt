cmake_minimum_required(VERSION 3.8)
project(wheel_perception)

# --- 1. 基础配置 ---
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# --- 2. 寻找依赖 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED) # [必须] 组件化依赖
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(wheel_msgs REQUIRED)        # [必须] 自定义消息包
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# --- 3. CUDA & TensorRT & ZED 配置 ---
enable_language(CUDA)
find_package(CUDA REQUIRED)
set(CUDA_LIB_PATH "/usr/local/cuda/lib64")
set(CUDA_INCLUDE_PATH "/usr/local/cuda/include")

# 避免使用 `native`：在部分 CMake + CUDA 11.8 组合下会被翻译成
# nvcc 不识别的 `compute_native`，从而导致编译失败。
# 如有需要，可在 colcon/cmake 命令行中覆盖，例如：
#   --cmake-args -DWHEEL_CUDA_ARCHITECTURES=86
set(WHEEL_CUDA_ARCHITECTURES "" CACHE STRING
  "Semicolon-separated CUDA architectures for wheel_perception (e.g. 72;75;86;87)")

if(NOT WHEEL_CUDA_ARCHITECTURES)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|ARM64)$")
    set(WHEEL_CUDA_ARCHITECTURES "72;87")
  else()
    set(WHEEL_CUDA_ARCHITECTURES "75;86")
  endif()
endif()

message(STATUS "wheel_perception CUDA architectures: ${WHEEL_CUDA_ARCHITECTURES}")

find_library(CUDART_LIB NAMES cudart HINTS ${CUDA_LIB_PATH})
find_library(CUDA_LIB NAMES cuda HINTS ${CUDA_LIB_PATH} /usr/lib/wsl/lib)

# TensorRT：同时支持实车 Jetson(aarch64) 和数据集工作站(x86_64)。
# 旧配置把路径固定为 aarch64，并依赖 CMake 缓存中残留的 x86_64
# 查找结果；一旦使用 --cmake-clean-cache 就会找不到 libnvinfer。
find_path(TRT_INCLUDE_DIR
  NAMES NvInfer.h
  HINTS
    /usr/include/${CMAKE_LIBRARY_ARCHITECTURE}
    /usr/include/aarch64-linux-gnu
    /usr/include/x86_64-linux-gnu
    /usr/local/TensorRT/include
)
find_library(NVINFER_LIB
  NAMES nvinfer
  HINTS
    /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
    /usr/local/TensorRT/lib
)
find_library(NVINFER_PLUGIN_LIB
  NAMES nvinfer_plugin
  HINTS
    /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}
    /usr/lib/aarch64-linux-gnu
    /usr/lib/x86_64-linux-gnu
    /usr/local/TensorRT/lib
)

if(NOT TRT_INCLUDE_DIR OR NOT NVINFER_LIB OR NOT NVINFER_PLUGIN_LIB)
  message(FATAL_ERROR
    "TensorRT not found (include=${TRT_INCLUDE_DIR}, "
    "nvinfer=${NVINFER_LIB}, plugin=${NVINFER_PLUGIN_LIB})")
endif()

message(STATUS "TensorRT include: ${TRT_INCLUDE_DIR}")
message(STATUS "TensorRT nvinfer: ${NVINFER_LIB}")

# ZED SDK
set(ZED_INCLUDE_DIRS "/usr/local/zed/include")
set(ZED_LIBRARY_DIR "/usr/local/zed/lib")
find_library(ZED_LIB NAMES sl_zed HINTS ${ZED_LIBRARY_DIR})

include_directories(
  include
  ${CUDA_INCLUDE_DIRS}
  ${TRT_INCLUDE_DIR}
  ${ZED_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# =========================================================
# 4. 核心算法库 (Wheel Core)
# =========================================================
add_library(wheel_core SHARED
  src/core/zed_driver.cpp
  src/core/world_pose_velocity_estimator.cpp
  src/core/ai_engine.cpp
  src/core/obstacle_fusion.cu
  src/core/mask_postprocess.cu
  src/core/preprocess.cu
  src/core/global_flags.cpp  # 新增
)

if(CMAKE_CUDA_COMPILER)
  set_property(TARGET wheel_core PROPERTY CUDA_ARCHITECTURES ${WHEEL_CUDA_ARCHITECTURES})
endif()

# 链接核心库依赖
target_link_libraries(wheel_core
  ${ZED_LIB}
  ${CUDA_LIB}
  ${CUDART_LIB}
  ${NVINFER_LIB}
  ${NVINFER_PLUGIN_LIB}
  ${OpenCV_LIBS}
)
target_include_directories(wheel_core PRIVATE ${Eigen3_INCLUDE_DIRS})

# [关键] 核心库也需要 ROS 依赖
ament_target_dependencies(wheel_core
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  cv_bridge
)

# =========================================================
# 5. 感知节点组件 (FusionNode) - 编译为 SHARED 库
# =========================================================
# 注意：这里绝不能用 add_executable，必须是 add_library
add_library(fusion_node SHARED src/fusion_node.cpp)

target_include_directories(fusion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 在包含目录里添加 Eigen 的路径
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

# 链接核心库
target_link_libraries(fusion_node wheel_core )

# 添加 ROS 依赖
ament_target_dependencies(fusion_node
  rclcpp
  rclcpp_lifecycle
  rclcpp_components  # 组件化核心
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  wheel_msgs         # 你的自定义消息
  nav_msgs           # 导航消息
)

# [关键] 注册为组件
rclcpp_components_register_nodes(fusion_node "FusionNode")

# =========================================================
# 6. 控制节点组件 (ControllerNode)
# =========================================================
add_library(controller_node SHARED src/controller_node.cpp)

add_library(dwa_controller SHARED
  dwa_controller/src/DWAPlanner.cpp
)

target_include_directories(dwa_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/dwa_controller/include>
  $<INSTALL_INTERFACE:include>
)


target_include_directories(controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/dwa_controller/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(controller_node wheel_core dwa_controller)
ament_target_dependencies(controller_node geometry_msgs nav_msgs tf2 tf2_geometry_msgs
  rclcpp
  rclcpp_components
  wheel_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

# [关键] 注册为组件
# 注意：代码里是 wheel_control::ControllerNode，这里名字要对应
rclcpp_components_register_nodes(controller_node "wheel_control::ControllerNode")

# =========================================================
# 7. 安装规则
# =========================================================
install(TARGETS 
  wheel_core 
  dwa_controller
  fusion_node 
  controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY dwa_controller/include/
  DESTINATION include
)

# 安装 Launch 文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${CMAKE_CURRENT_SOURCE_DIR}/../../zed_dataset_player.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dwa_planner test/test_dwa_planner.cpp)
  target_link_libraries(test_dwa_planner dwa_controller)
  target_include_directories(test_dwa_planner PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/dwa_controller/include
  )

  ament_add_gtest(test_world_pose_velocity_estimator
    test/test_world_pose_velocity_estimator.cpp
    src/core/world_pose_velocity_estimator.cpp
  )
  target_include_directories(test_world_pose_velocity_estimator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${Eigen3_INCLUDE_DIRS}
  )
endif()

# Install config directory
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch directory (顺便检查一下 launch 也没漏)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
