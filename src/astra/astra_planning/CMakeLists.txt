cmake_minimum_required(VERSION 3.8)
project(astra_planning)


# 未显式指定构建类型时默认使用 Release（-O3），保证 ESDF/MINCO/MPC 等计算密集模块的实时性能。
# 调试时可用 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug 覆盖。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 规划功能包（对应报告 5.5.3 前端搜索与 5.5.4 MINCO 轨迹优化）：
# 连续代价场空间 A* 前端、端点 BFS 重定位、MINCO 轨迹表示、L-BFGS 优化、重规划。

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(astra_common REQUIRED)
find_package(astra_mapping REQUIRED)
find_package(astra_third_party REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(astra_planning
  src/spatial_grid_astar.cpp
  src/endpoint_nudge.cpp
  src/replan_strategy.cpp
  src/minco_adapter.cpp
  src/minco_trajectory.cpp
  src/minco_optimizer.cpp
  src/waypoint_lbfgs_optimizer.cpp
  src/trajectory_optimizer.cpp
)
target_include_directories(astra_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(astra_planning Eigen3::Eigen)
ament_target_dependencies(astra_planning
  astra_common astra_mapping astra_third_party)

# 规划节点。
add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node astra_planning)
ament_target_dependencies(planner_node
  astra_common astra_mapping rclcpp nav_msgs geometry_msgs builtin_interfaces)
install(TARGETS planner_node DESTINATION lib/${PROJECT_NAME})

install(TARGETS astra_planning
  EXPORT export_astra_planning
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(astra_planning)
ament_export_targets(export_astra_planning HAS_LIBRARY_TARGET)
ament_export_dependencies(astra_common astra_mapping astra_third_party Eigen3)

if(BUILD_TESTING)
  add_executable(astra_planning_test test/astra_planning_test.cpp)
  target_link_libraries(astra_planning_test astra_planning)
  ament_target_dependencies(astra_planning_test astra_common astra_mapping)
  # 测试依赖 assert 检查不变量；Release 的 -DNDEBUG 会禁用 assert，
  # 故对测试目标取消 NDEBUG，保证断言在任何构建类型下都生效。
  target_compile_options(astra_planning_test PRIVATE -UNDEBUG)

  add_test(NAME astra_planning_test COMMAND astra_planning_test)
endif()

ament_package()
