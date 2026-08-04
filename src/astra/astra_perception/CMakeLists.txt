cmake_minimum_required(VERSION 3.8)
project(astra_perception)


# 未显式指定构建类型时默认使用 Release（-O3），保证 ESDF/MINCO/MPC 等计算密集模块的实时性能。
# 调试时可用 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug 覆盖。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 感知功能包（对应报告 5.3）：点云预处理与三维滚动占据栅格。
# 注意：将三维占据栅格转为二维障碍图的 occupancy_node 归入 astra_mapping，
# 因为它产出的是建图层的二维障碍图，依赖本包的感知库。

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(astra_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 感知核心库：点云工具 + 三维占据栅格。
add_library(astra_perception
  src/pointcloud_utils.cpp
  src/rolling_occupancy_grid.cpp
)
target_include_directories(astra_perception PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(astra_perception Eigen3::Eigen)
ament_target_dependencies(astra_perception astra_common rclcpp sensor_msgs)

install(TARGETS astra_perception
  EXPORT export_astra_perception
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(astra_perception)
ament_export_targets(export_astra_perception HAS_LIBRARY_TARGET)
ament_export_dependencies(astra_common rclcpp sensor_msgs Eigen3)
ament_package()
