cmake_minimum_required(VERSION 3.8)
project(astra_simulation)


# 未显式指定构建类型时默认使用 Release（-O3），保证 ESDF/MINCO/MPC 等计算密集模块的实时性能。
# 调试时可用 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug 覆盖。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 仿真功能包：工程内轻量仿真世界节点，发布仿真点云、里程计与目标点。
# 后续如需从 /home/tumi/ros_ws 复制更完整的仿真，也应纳入本包。

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(astra_common REQUIRED)
find_package(astra_perception REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(sim_world_node src/sim_world_node.cpp)
ament_target_dependencies(sim_world_node
  astra_common astra_perception rclcpp sensor_msgs nav_msgs geometry_msgs)
install(TARGETS sim_world_node DESTINATION lib/${PROJECT_NAME})

ament_package()
