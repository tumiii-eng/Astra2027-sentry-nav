#include "astra_planning/goal_reachability_resolver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <queue>
#include <vector>

#include "astra_common/common.hpp"

namespace astra_nav
{

GoalReachabilityResolver::GoalReachabilityResolver(const GoalReachabilityConfig & config)
: config_(config)
{
  config_.required_clearance = std::max(0.0, config_.required_clearance);
  config_.max_search_radius = std::max(0.0, config_.max_search_radius);
}

GoalReachabilityResult GoalReachabilityResolver::resolve(
  const Point2D & goal, const EsdfMap & esdf) const
{
  GoalReachabilityResult result;
  result.original = goal;
  result.point = goal;
  if (!esdf.valid()) {
    return result;
  }
  result.original_distance = esdf.in_map(goal.x, goal.y) ? esdf.query_distance(goal.x, goal.y) : -1.0;
  result.final_distance = result.original_distance;

  // 第 1 步：可达性检测。可达则直接返回，不做任何重定位（对应流程图“可达 -> 模式选择”）。
  if (is_reachable(goal, esdf)) {
    result.reachable = true;
    result.success = true;
    return result;
  }

  // 第 2 步：BFS 在目标点附近找到第一个 ESDF 值不为 0（距离为正、可拿到有效梯度）的种子点。
  Point2D seed = goal;
  result.bfs_used = true;
  if (!bfs_find_seed(goal, esdf, seed, result.bfs_expansions)) {
    // 搜索半径内找不到任何有效种子，重定位失败。
    return result;
  }

  // 若 BFS 找到的种子本身已满足可达性，则无需 DFS。
  Point2D relocated = seed;
  if (!is_reachable(seed, esdf)) {
    // 第 3 步：DFS 从种子点沿 ESDF 梯度方向优先扩展，直到满足可达性。
    result.dfs_used = true;
    if (!dfs_along_gradient(seed, esdf, relocated, result.dfs_expansions)) {
      return result;
    }
  }

  result.point = relocated;
  result.relocated = true;
  result.success = true;
  result.displacement = distance_2d(goal, relocated);
  result.final_distance = esdf.query_distance(relocated.x, relocated.y);
  return result;
}

bool GoalReachabilityResolver::is_reachable(const Point2D & point, const EsdfMap & esdf) const
{
  if (!esdf.in_map(point.x, point.y)) {
    return false;
  }
  return esdf.query_distance(point.x, point.y) >= config_.required_clearance;
}

bool GoalReachabilityResolver::bfs_find_seed(
  const Point2D & goal, const EsdfMap & esdf, Point2D & seed, int & expansions) const
{
  const auto goal_grid = esdf.world_to_grid(goal.x, goal.y);
  const int radius_cells =
    std::max(1, static_cast<int>(std::ceil(config_.max_search_radius / esdf.resolution())));

  const int width = esdf.width();
  const int height = esdf.height();
  std::vector<std::uint8_t> visited(
    static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);

  auto in_radius = [&](int gx, int gy) {
    return std::abs(gx - goal_grid.first) <= radius_cells &&
           std::abs(gy - goal_grid.second) <= radius_cells;
  };
  auto mark = [&](int gx, int gy) -> bool {
    if (!esdf.in_bounds(gx, gy)) {
      return false;
    }
    auto & flag = visited[static_cast<std::size_t>(gy) * static_cast<std::size_t>(width) + gx];
    if (flag != 0) {
      return false;
    }
    flag = 1;
    return true;
  };

  // 从目标栅格起步做四邻域 BFS；若目标栅格越界，则把搜索窗口钳到地图内的最近栅格。
  const int seed_gx = std::clamp(goal_grid.first, 0, width - 1);
  const int seed_gy = std::clamp(goal_grid.second, 0, height - 1);
  std::queue<std::pair<int, int>> frontier;
  mark(seed_gx, seed_gy);
  frontier.emplace(seed_gx, seed_gy);

  constexpr std::array<std::pair<int, int>, 4> kNeighbors{
    {{1, 0}, {-1, 0}, {0, 1}, {0, -1}}};

  while (!frontier.empty()) {
    const auto [gx, gy] = frontier.front();
    frontier.pop();
    ++expansions;

    const Point2D center = esdf.grid_to_world(gx, gy);
    // 报告语义“ESDF 值不为 0”：取距离为正（不在障碍内部）的第一个栅格作为种子。
    if (esdf.query_distance(center.x, center.y) > 0.0) {
      seed = center;
      return true;
    }

    for (const auto & step : kNeighbors) {
      const int nx = gx + step.first;
      const int ny = gy + step.second;
      if (!in_radius(nx, ny)) {
        continue;
      }
      if (mark(nx, ny)) {
        frontier.emplace(nx, ny);
      }
    }
  }
  return false;
}

bool GoalReachabilityResolver::dfs_along_gradient(
  const Point2D & seed, const EsdfMap & esdf, Point2D & result, int & expansions) const
{
  const auto seed_grid = esdf.world_to_grid(seed.x, seed.y);
  const int radius_cells =
    std::max(1, static_cast<int>(std::ceil(config_.max_search_radius / esdf.resolution())));

  const int width = esdf.width();
  const int height = esdf.height();
  std::vector<std::uint8_t> visited(
    static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);

  auto in_radius = [&](int gx, int gy) {
    return std::abs(gx - seed_grid.first) <= radius_cells &&
           std::abs(gy - seed_grid.second) <= radius_cells;
  };
  auto index = [&](int gx, int gy) {
    return static_cast<std::size_t>(gy) * static_cast<std::size_t>(width) + gx;
  };

  // 八邻域 DFS：每次从栈顶取出当前栅格，按“与 ESDF 梯度方向的对齐程度”对邻居排序，
  // 把最贴合梯度（指向远离障碍方向）的邻居最后压栈、最先弹出，实现报告所述
  // “根据梯度方向作为优先扩展方向”的深度优先扩展，直到 clearance 满足可达性。
  std::vector<std::pair<int, int>> stack;
  if (!esdf.in_bounds(seed_grid.first, seed_grid.second)) {
    return false;
  }
  visited[index(seed_grid.first, seed_grid.second)] = 1;
  stack.emplace_back(seed_grid.first, seed_grid.second);

  constexpr std::array<std::pair<int, int>, 8> kNeighbors{
    {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}}};

  while (!stack.empty()) {
    const auto [gx, gy] = stack.back();
    stack.pop_back();
    ++expansions;

    const Point2D center = esdf.grid_to_world(gx, gy);
    if (is_reachable(center, esdf)) {
      result = center;
      return true;
    }

    // 当前栅格的 ESDF 梯度方向（指向 clearance 增大的方向）。
    const auto query = esdf.query_distance_and_gradient(center.x, center.y);
    Point2D grad = query.second;
    const double grad_norm = std::hypot(grad.x, grad.y);
    if (grad_norm > 1.0e-9) {
      grad.x /= grad_norm;
      grad.y /= grad_norm;
    }

    // 收集可扩展邻居，并按与梯度方向的点积升序排序：
    // 点积大的（最贴合梯度）排在末尾，最后压栈、最先弹出，获得最高扩展优先级。
    std::array<std::pair<double, std::pair<int, int>>, 8> scored;
    int count = 0;
    for (const auto & step : kNeighbors) {
      const int nx = gx + step.first;
      const int ny = gy + step.second;
      if (!in_radius(nx, ny) || !esdf.in_bounds(nx, ny)) {
        continue;
      }
      if (visited[index(nx, ny)] != 0) {
        continue;
      }
      const double sn = std::hypot(static_cast<double>(step.first), static_cast<double>(step.second));
      const double align =
        grad_norm > 1.0e-9 ?
        (step.first * grad.x + step.second * grad.y) / std::max(sn, 1.0e-9) :
        0.0;
      scored[static_cast<std::size_t>(count)] = {align, {nx, ny}};
      ++count;
    }
    std::sort(
      scored.begin(), scored.begin() + count,
      [](const auto & a, const auto & b) { return a.first < b.first; });
    for (int i = 0; i < count; ++i) {
      const auto & cell = scored[static_cast<std::size_t>(i)].second;
      visited[index(cell.first, cell.second)] = 1;
      stack.emplace_back(cell.first, cell.second);
    }
  }
  return false;
}

}  // namespace astra_nav
