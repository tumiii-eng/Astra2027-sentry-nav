#pragma once

// 通行性判定直接复制自 HWSentryNav26
// (nav_executor/src/path_planner/search/grid_utils.cpp:13-35)，判定逻辑逐字一致，
// 只把 CostMap/Eigen::Vector2i 换成 Astra 的 Grid2D/(x, y)。

#include <cstdlib>

#include "astra_mapping/obstacle_extractor.hpp"

namespace astra_nav
{

// 硬通行判定：越界不可通行，格子代价 < occupied_threshold 才可通行。
inline bool grid_cell_traversable(
  const Grid2D & cost_map, int x, int y, int occupied_threshold)
{
  if (x < 0 || y < 0 || x >= cost_map.width || y >= cost_map.height) {
    return false;
  }
  return static_cast<int>(cost_map.at(x, y)) < occupied_threshold;
}

// 禁止切角：对角移动要求两个正交邻格同时可通行，避免从两个障碍的对角缝隙穿过。
inline bool grid_edge_avoids_corner_cutting(
  const Grid2D & cost_map, int from_x, int from_y, int to_x, int to_y,
  int occupied_threshold)
{
  const int dx = to_x - from_x;
  const int dy = to_y - from_y;
  if (std::abs(dx) != 1 || std::abs(dy) != 1) {
    return true;
  }
  return grid_cell_traversable(cost_map, from_x + dx, from_y, occupied_threshold) &&
         grid_cell_traversable(cost_map, from_x, from_y + dy, occupied_threshold);
}

}  // namespace astra_nav
