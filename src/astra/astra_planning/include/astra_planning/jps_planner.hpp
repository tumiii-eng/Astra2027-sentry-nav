#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "astra_mapping/obstacle_extractor.hpp"

namespace astra_nav
{

struct GridPath
{
  std::vector<std::pair<int, int>> grid_points;
  std::vector<Point2D> world_points;
};

class JpsPlanner
{
public:
  explicit JpsPlanner(const Grid2D & obstacle_grid);

  // 使用跳点搜索（JPS）规划栅格路径。返回的 world_points 已展开为整段折线端点。
  std::optional<GridPath> plan(const Point2D & start, const Point2D & goal) const;

  // 仅供测试/对照：使用标准 8 邻域 A* 规划，返回最优路径代价（栅格步数加权）。
  // JPS 是 A* 的等价加速，二者最优代价必须一致——这是验证 JPS 正确性的关键不变量。
  std::optional<double> astar_cost(const Point2D & start, const Point2D & goal) const;

private:
  bool in_bounds(int x, int y) const;
  bool is_free(int x, int y) const;
  std::pair<int, int> world_to_grid(const Point2D & point) const;
  Point2D grid_to_world(int x, int y) const;
  std::optional<std::pair<int, int>> nearest_free(std::pair<int, int> seed) const;
  int flat_index(int x, int y) const { return y * grid_.width + x; }
  double heuristic(int x, int y, int gx, int gy) const;

  // JPS 跳点核心：从 (x,y) 沿方向 (dx,dy) 跳跃，直到遇到跳点/目标/障碍。
  // 命中跳点返回其坐标，否则返回 nullopt。
  std::optional<std::pair<int, int>> jump(
    int x, int y, int dx, int dy, int gx, int gy) const;
  // 判断从 (x,y) 沿 (dx,dy) 前进时，(nx,ny) 处是否存在强制邻居。
  bool has_forced_neighbor(int x, int y, int dx, int dy) const;
  // 根据父节点方向对当前节点做邻居剪枝，返回应继续探索的方向集合
  // （自然邻居 + 因障碍产生的强制邻居方向）。需要节点坐标以判定强制邻居。
  std::vector<std::pair<int, int>> pruned_directions(int x, int y, int dx, int dy) const;
  // 把跳点序列展开为连续栅格直线段端点（便于后续重采样）。
  std::vector<std::pair<int, int>> reconstruct_path(
    const std::vector<int> & parent, int goal_index) const;

  Grid2D grid_;
};

}  // namespace astra_nav

