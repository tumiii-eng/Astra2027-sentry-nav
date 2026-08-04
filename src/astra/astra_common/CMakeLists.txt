cmake_minimum_required(VERSION 3.8)
project(astra_common)


# 未显式指定构建类型时默认使用 Release（-O3），保证 ESDF/MINCO/MPC 等计算密集模块的实时性能。
# 调试时可用 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug 覆盖。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 通用基础库：被其它所有导航功能包依赖，提供二维位姿/速度/轨迹点数据结构与通用数学函数。

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(astra_common src/common.cpp)
target_include_directories(astra_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(astra_common Eigen3::Eigen)
ament_target_dependencies(astra_common geometry_msgs)

install(TARGETS astra_common
  EXPORT export_astra_common
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(astra_common)
ament_export_targets(export_astra_common HAS_LIBRARY_TARGET)
ament_export_dependencies(geometry_msgs Eigen3)

if(BUILD_TESTING)
  add_executable(astra_common_test test/astra_common_test.cpp)
  target_link_libraries(astra_common_test astra_common)
  ament_target_dependencies(astra_common_test geometry_msgs)
  # 测试依赖 assert 检查不变量；Release 的 -DNDEBUG 会禁用 assert，
  # 故对测试目标取消 NDEBUG，保证断言在任何构建类型下都生效。
  target_compile_options(astra_common_test PRIVATE -UNDEBUG)

  add_test(NAME astra_common_test COMMAND astra_common_test)
endif()

ament_package()
