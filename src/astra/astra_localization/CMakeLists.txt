cmake_minimum_required(VERSION 3.8)
project(astra_localization)


# 未显式指定构建类型时默认使用 Release（-O3），保证 ESDF/MINCO/MPC 等计算密集模块的实时性能。
# 调试时可用 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug 覆盖。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 定位功能包（对应报告 5.1 Batch-LIWO 与 5.2 全局定位）。
# 当前阶段只有里程计接口代理节点，占住接口边界，后续替换为完整 C++ 里程计实现。

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(batch_liwo_proxy_node src/batch_liwo_proxy_node.cpp)
ament_target_dependencies(batch_liwo_proxy_node rclcpp nav_msgs)
install(TARGETS batch_liwo_proxy_node DESTINATION lib/${PROJECT_NAME})

ament_package()
