cmake_minimum_required(VERSION 3.8)
project(astra_third_party)


# 未显式指定构建类型时默认使用 Release（-O3），保证 ESDF/MINCO/MPC 等计算密集模块的实时性能。
# 调试时可用 colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug 覆盖。
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 第三方头文件副本（仅头文件）：GCOPTER / DDR-opt 派生的二维 MINCO、L-BFGS、轨迹与求根头文件。
# 原始工程只读未修改，保留原 MIT 许可。业务代码通过适配层访问，避免节点直接依赖第三方接口。

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# 头文件库（INTERFACE）：不产生编译产物，只导出 include 路径与 Eigen 依赖。
# 导出两个 include 根：
#   1) include/                     —— 业务代码使用 "astra_third_party/gcopter/xxx.hpp"，路径自带包名，便于阅读；
#   2) include/astra_third_party/   —— 兼容 gcopter 头文件之间原有的 "gcopter/xxx.hpp" 互相包含，
#                                      保持第三方源码与上游一致、不做侵入式修改。
add_library(astra_third_party INTERFACE)
target_include_directories(astra_third_party INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/astra_third_party>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/astra_third_party>
)
target_link_libraries(astra_third_party INTERFACE Eigen3::Eigen)

# ---- qpOASES 稠密 QP 求解器（报告 5.5.5 控制器使用）----
# 来源：coin-or/qpOASES 3.2（LGPL 2.1），仅复制 src/ 与 include/ 到工程内 qpoases/。
# 使用 qpOASES 自带的 BLAS/LAPACK Replacement（src/BLASReplacement.cpp、src/LAPACKReplacement.cpp），
# 不引入外部 BLAS/LAPACK 依赖，保证工程零额外系统依赖即可构建。
file(GLOB QPOASES_SRC ${CMAKE_CURRENT_SOURCE_DIR}/qpoases/src/*.cpp)
add_library(qpoases STATIC ${QPOASES_SRC})
target_include_directories(qpoases PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/qpoases/include>
  $<INSTALL_INTERFACE:include/qpoases>
)
# __NO_COPYRIGHT__：抑制 qpOASES 启动时打印版权信息；-w：第三方源码不纳入工程告警。
target_compile_definitions(qpoases PUBLIC __NO_COPYRIGHT__)
target_compile_options(qpoases PRIVATE -w)
set_target_properties(qpoases PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS astra_third_party qpoases EXPORT export_astra_third_party
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY qpoases/include/ DESTINATION include/qpoases)

ament_export_include_directories(include include/astra_third_party include/qpoases)
ament_export_targets(export_astra_third_party HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
