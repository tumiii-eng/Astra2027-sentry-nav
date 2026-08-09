#pragma once

// 端点重定位直接复制自 HWSentryNav26
// (nav_executor/src/path_planner/path_planner.cpp 的 is_map_point_feasible /
//  nudge_point_to_free)，BFS 逻辑与可行性判定逐字一致，只裁掉 Astra 没有的
// 方向场模长条件（direction->value.norm() < endpoint_direction_norm_max）。
//
// 为什么替换原来的直线采样：直线采样只能把端点沿固定射线方向推出去，
// 一旦端点被凹形障碍包住，射线方向上全是障碍就直接失败；BFS 按真实欧氏
// 距离由近及远遍历整个邻域，只要 max_nudge_distance 内存在可行格就一定能找到，
// 且找到的是距离最近的那个。

#include <optional>

#include "astra_mapping/obstacle_extractor.hpp"

namespace astra_nav
{

// 在代价图上按 BFS 由近及远搜索满足硬通行条件的最近格心。
// 输入点本身可行时原样返回；max_nudge_distance 内无可行格时返回 nullopt。
std::optional<Point2D> nudge_point_to_free(
  const Grid2D & cost_map,
  const Point2D & point,
  double max_nudge_distance,
  int occupied_threshold = kOccupiedCostThreshold);

}  // namespace astra_nav
