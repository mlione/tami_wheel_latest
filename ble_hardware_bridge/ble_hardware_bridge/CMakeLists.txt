cmake_minimum_required(VERSION 3.8)
project(ble_hardware_bridge VERSION 1.0.0)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)

pkg_check_modules(GIO REQUIRED IMPORTED_TARGET gio-2.0)

add_executable(
  ble_hardware_bridge_node
  src/ble_hardware_bridge_node.cpp
  src/bluez_ble_client.cpp
  src/control_protocol.cpp
)
target_include_directories(
  ble_hardware_bridge_node
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(
  ble_hardware_bridge_node
  PkgConfig::GIO
)
ament_target_dependencies(
  ble_hardware_bridge_node
  geometry_msgs
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(
    control_protocol_test
    test/control_protocol_test.cpp
    src/control_protocol.cpp
  )
  target_include_directories(
    control_protocol_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  add_test(
    NAME constraint_visualization_test
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/plot_control_constraints.py
      --config ${CMAKE_CURRENT_SOURCE_DIR}/config/ble_hardware_bridge.yaml
      --output ${CMAKE_CURRENT_BINARY_DIR}/constraint_visualization_test.png
      --resolution 201
      --dpi 72
  )
  set_tests_properties(
    constraint_visualization_test
    PROPERTIES ENVIRONMENT "MPLCONFIGDIR=${CMAKE_CURRENT_BINARY_DIR}/matplotlib"
  )
  add_test(
    NAME velocity_constraint_visualization_test
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/scripts/plot_control_constraints.py
      --config ${CMAKE_CURRENT_SOURCE_DIR}/config/ble_hardware_bridge.yaml
      --output ${CMAKE_CURRENT_BINARY_DIR}/velocity_constraint_visualization_test.png
      --space velocity
      --resolution 201
      --dpi 72
  )
  set_tests_properties(
    velocity_constraint_visualization_test
    PROPERTIES ENVIRONMENT "MPLCONFIGDIR=${CMAKE_CURRENT_BINARY_DIR}/matplotlib"
  )
endif()

install(
  TARGETS ble_hardware_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY config docs launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  PROGRAMS scripts/plot_control_constraints.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
