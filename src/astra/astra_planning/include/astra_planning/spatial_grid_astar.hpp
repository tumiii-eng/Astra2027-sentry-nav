#pragma once

// 本前端搜索直接复制自 HWSentryNav26
// (nav_executor/{include,src}/nav_executor/path_planner/search/spatial_grid_astar.{hpp,cpp})，
// 搜索逻辑与代价公式逐字一致，只裁掉 Astra 没有的地形方向场/地形通道部分。
//
// 为什么替换原来的 JPS：JPS 的跳点剪枝（自然邻居 + 强制邻居）只有在“通行性是二值”
// 的前提下才是保守正确的剪枝——它假设被跳过的格子之间代价完全等价。改成 0-255
// 连续代价场后，跳跃区间内部的代价不再相同，跳点剪枝会漏掉真正的最优路径，
// 因此前端必须换回逐格展开的 A*。

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "astra_mapping/obstacle_extractor.hpp"

namespace astra_nav
{

struct SpatialRoute
{
  std::vector<std::pair<int, int>> raw_path;
  // 逐格路径对应的格心世界坐标；由 plan() 填充，search() 只产出 raw_path。
  std::vector<Point2D> world_points;
  int expansions = 0;
  std::size_t open_peak = 0;
};

class SpatialGridAstar
{
public:
  struct Params
  {
    // 归一化障碍软代价乘数；1.0 表示接近占用阈值时边代价约翻倍。
    double obstacle_weight = 1.0;
    int max_expansions = 1000000;
    // 统一硬占用阈值；格子代价 >= 该值视为不可通行。
    int occupied_threshold = kOccupiedCostThreshold;
  };

  struct Result
  {
    SpatialRoute route;
    bool success = false;
    std::string error;
  };

  SpatialGridAstar(const Grid2D & cost_map, Params params)
  : cost_map_(cost_map), params_(params) {}

  Result search(const std::pair<int, int> & start, const std::pair<int, int> & goal) const;

  // 世界坐标接口：内部完成 world->grid 转换，成功时顺带填好 route.world_points。
  // 失败原因与扩展统计随 Result 一起返回，便于上层如实报错而不是笼统“未找到路径”。
  Result plan(const Point2D & start, const Point2D & goal) const;

  std::pair<int, int> world_to_grid(const Point2D & point) const;
  Point2D grid_to_world(int x, int y) const;

private:
  Grid2D cost_map_;
  Params params_;
};

}  // namespace astra_nav
