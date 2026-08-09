cmake_minimum_required(VERSION 3.8)
project(astra_mapping)


# 未显式指定构建类型时默认使用 Release（-O3），保证 ESDF/MINCO/MPC 等计算密集模块的实时性能。
# 调试时可用 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug 覆盖。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 建图功能包（对应报告 5.4 障碍识别与 5.5.2 障碍梯度表示）：
# 二维障碍栅格提取 + 二维有符号距离场 ESDF（二次插值、O(N) 距离变换）。
# occupancy_node 订阅点云、维护三维占据栅格并发布二维障碍图，是感知到规划之间的建图桥梁。

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
find_package(Eigen3 REQUIRED)
# 连续代价场膨胀直接复用 HWSentryNav26 的 cv::distanceTransform / connectedComponentsWithStats。
find_package(OpenCV REQUIRED)

add_library(astra_mapping
  src/map_utils.cpp
  src/cost_map_sampler.cpp
  src/obstacle_extractor.cpp
  src/esdf_map.cpp
)
target_include_directories(astra_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(astra_mapping Eigen3::Eigen ${OpenCV_LIBS})
target_include_directories(astra_mapping PUBLIC ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(astra_mapping astra_common astra_perception)

# 占据栅格 / 二维障碍图节点：订阅点云，维护三维占据栅格，发布二维障碍图。
add_executable(occupancy_node src/occupancy_node.cpp)
target_link_libraries(occupancy_node astra_mapping)
ament_target_dependencies(occupancy_node
  astra_common astra_perception rclcpp sensor_msgs nav_msgs geometry_msgs)
install(TARGETS occupancy_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS astra_mapping
  EXPORT export_astra_mapping
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(astra_mapping)
ament_export_targets(export_astra_mapping HAS_LIBRARY_TARGET)
ament_export_dependencies(astra_common astra_perception Eigen3 OpenCV)

if(BUILD_TESTING)
  add_executable(astra_mapping_test test/astra_mapping_test.cpp)
  target_link_libraries(astra_mapping_test astra_mapping)
  ament_target_dependencies(astra_mapping_test astra_common astra_perception)
  # 测试依赖 assert 检查不变量；Release 的 -DNDEBUG 会禁用 assert，
  # 故对测试目标取消 NDEBUG，保证断言在任何构建类型下都生效。
  target_compile_options(astra_mapping_test PRIVATE -UNDEBUG)

  add_test(NAME astra_mapping_test COMMAND astra_mapping_test)
endif()

ament_package()
