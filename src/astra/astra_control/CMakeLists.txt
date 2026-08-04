cmake_minimum_required(VERSION 3.8)
project(astra_control)


# 未显式指定构建类型时默认使用 Release（-O3），保证 ESDF/MINCO/MPC 等计算密集模块的实时性能。
# 调试时可用 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug 覆盖。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 控制功能包（对应报告 5.5.5）：轨迹跟踪控制器与底盘速度指令输出。
# 依赖 astra_planning 是因为控制器复用其 MINCO 轨迹表示来重建参考轨迹。

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(astra_common REQUIRED)
find_package(astra_planning REQUIRED)
find_package(astra_third_party REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(astra_control
  src/preview_controller.cpp
  src/se2_mpc_preview.cpp
  src/se2_mpc_controller.cpp
  src/chassis_kinematics.cpp
  src/high_rate_state_estimator.cpp
)
target_include_directories(astra_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
# qpoases 静态库由 astra_third_party 导出（报告 5.5.5 的稠密 QP 求解器）。
target_link_libraries(astra_control Eigen3::Eigen astra_third_party::qpoases)
ament_target_dependencies(astra_control astra_common astra_planning astra_third_party)

# 控制节点。
add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node astra_control)
ament_target_dependencies(controller_node
  astra_common astra_planning rclcpp nav_msgs geometry_msgs sensor_msgs)
install(TARGETS controller_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS astra_control
  EXPORT export_astra_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(astra_control)
ament_export_targets(export_astra_control HAS_LIBRARY_TARGET)
ament_export_dependencies(astra_common astra_planning astra_third_party rclcpp nav_msgs geometry_msgs sensor_msgs Eigen3)

if(BUILD_TESTING)
  add_executable(astra_control_test test/astra_control_test.cpp)
  target_link_libraries(astra_control_test astra_control)
  ament_target_dependencies(astra_control_test astra_common astra_planning)
  # 测试依赖 assert 检查不变量；Release 的 -DNDEBUG 会禁用 assert，
  # 故对测试目标取消 NDEBUG，保证断言在任何构建类型下都生效。
  target_compile_options(astra_control_test PRIVATE -UNDEBUG)

  add_test(NAME astra_control_test COMMAND astra_control_test)
endif()

ament_package()
