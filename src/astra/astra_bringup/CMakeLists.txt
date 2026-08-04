cmake_minimum_required(VERSION 3.8)
project(astra_bringup)


# 未显式指定构建类型时默认使用 Release（-O3），保证 ESDF/MINCO/MPC 等计算密集模块的实时性能。
# 调试时可用 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug 覆盖。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 启动与参数功能包：聚合各功能包节点的 launch 文件与基础/场景参数配置。
# 不含编译产物，只安装 launch 与 config。

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
