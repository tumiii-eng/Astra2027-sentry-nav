cmake_minimum_required(VERSION 3.8)
project(astra_pb2025_bridge)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# 动态障碍多级过滤：OpenCV 连通域分析 + PCL 统计离群点过滤。
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

# 全局障碍图节点：先验 /map(静态层) + 实时 terrain_map(动态层) -> 两层全场障碍图（供 Astra 全局规划）。
add_executable(global_obstacle_map_node src/global_obstacle_map_node.cpp)
target_include_directories(global_obstacle_map_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(global_obstacle_map_node ${OpenCV_LIBS} ${PCL_LIBRARIES})
ament_target_dependencies(global_obstacle_map_node
  rclcpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros)
install(TARGETS global_obstacle_map_node DESTINATION lib/${PROJECT_NAME})

# 联合 launch、参数覆盖与 RViz 配置。
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

ament_package()
