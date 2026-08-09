#include "astra_planning/spatial_grid_astar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>

#include "astra_mapping/grid_utils.hpp"

namespace astra_nav
{

namespace
{

const std::array<std::pair<int, int>, 8> NEIGHBORS {{
  {1, 0}, {-1, 0}, {0, 1}, {0, -1},
  {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
}};

std::size_t index_of(const std::pair<int, int> & cell, const int width)
{
  return static_cast<std::size_t>(cell.second) * static_cast<std::size_t>(width) +
         static_cast<std::size_t>(cell.first);
}

bool same_cell(const std::pair<int, int> & lhs, const std::pair<int, int> & rhs)
{
  return lhs.first == rhs.first && lhs.second == rhs.second;
}

double octile_distance(
  const std::pair<int, int> & from,
  const std::pair<int, int> & to,
  const double resolution)
{
  const int abs_dx = std::abs(to.first - from.first);
  const int abs_dy = std::abs(to.second - from.second);
  const int diagonal = std::min(abs_dx, abs_dy);
  const int straight = std::max(abs_dx, abs_dy) - diagonal;
  return resolution *
         (std::sqrt(2.0) * static_cast<double>(diagonal) + static_cast<double>(straight));
}

struct OpenEntry
{
  double f = 0.0;
  double g = 0.0;
  std::pair<int, int> cell{0, 0};

  bool operator>(const OpenEntry & other) const
  {
    if (f != other.f) {return f > other.f;}
    if (g != other.g) {return g > other.g;}
    if (cell.second != other.cell.second) {return cell.second > other.cell.second;}
    return cell.first > other.cell.first;
  }
};

}  // namespace

SpatialGridAstar::Result SpatialGridAstar::search(
  const std::pair<int, int> & start, const std::pair<int, int> & goal) const
{
  Result result;
  const int occupied_threshold = params_.occupied_threshold;
  if (params_.obstacle_weight < 0.0 || params_.max_expansions <= 0 ||
    occupied_threshold <= 0 ||
    !grid_cell_traversable(cost_map_, start.first, start.second, occupied_threshold) ||
    !grid_cell_traversable(cost_map_, goal.first, goal.second, occupied_threshold))
  {
    result.error = "spatial grid A* configuration or endpoints are invalid";
    return result;
  }

  const int width = cost_map_.width;
  const std::size_t cell_count =
    static_cast<std::size_t>(width) * static_cast<std::size_t>(cost_map_.height);
  std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
  std::vector<std::pair<int, int>> parent(cell_count, {-1, -1});
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, std::greater<>> open;
  g_score[index_of(start, width)] = 0.0;
  open.push({octile_distance(start, goal, cost_map_.resolution), 0.0, start});

  while (!open.empty()) {
    result.route.open_peak = std::max(result.route.open_peak, open.size());
    const OpenEntry current = open.top();
    open.pop();
    if (current.g > g_score[index_of(current.cell, width)] + 1e-12) {continue;}
    if (same_cell(current.cell, goal)) {
      for (std::pair<int, int> cell = goal;; cell = parent[index_of(cell, width)]) {
        result.route.raw_path.push_back(cell);
        if (same_cell(cell, start)) {break;}
        if (parent[index_of(cell, width)].first < 0) {
          result.error = "spatial grid A* parent chain is incomplete";
          return result;
        }
      }
      std::reverse(result.route.raw_path.begin(), result.route.raw_path.end());
      result.success = true;
      return result;
    }
    if (result.route.expansions >= params_.max_expansions) {
      result.error = "spatial grid A* expansion limit reached";
      return result;
    }
    ++result.route.expansions;

    for (const auto & delta : NEIGHBORS) {
      const std::pair<int, int> next{
        current.cell.first + delta.first, current.cell.second + delta.second};
      if (!grid_cell_traversable(cost_map_, next.first, next.second, occupied_threshold) ||
        !grid_edge_avoids_corner_cutting(
          cost_map_, current.cell.first, current.cell.second, next.first, next.second,
          occupied_threshold))
      {
        continue;
      }
      const double length =
        std::hypot(static_cast<double>(delta.first), static_cast<double>(delta.second)) *
        cost_map_.resolution;
      const double normalized_cost = static_cast<double>(std::max(
          cost_map_.at(current.cell.first, current.cell.second),
          cost_map_.at(next.first, next.second))) / static_cast<double>(occupied_threshold);
      const double candidate = current.g + length *
        (1.0 + params_.obstacle_weight * normalized_cost);
      const std::size_t next_index = index_of(next, width);
      if (candidate + 1e-12 >= g_score[next_index]) {continue;}
      g_score[next_index] = candidate;
      parent[next_index] = current.cell;
      open.push({
        candidate + octile_distance(next, goal, cost_map_.resolution),
        candidate,
        next,
      });
    }
  }
  result.error = "spatial grid A* found no path through hard planner obstacles";
  return result;
}

SpatialGridAstar::Result SpatialGridAstar::plan(const Point2D & start, const Point2D & goal) const
{
  if (cost_map_.width <= 0 || cost_map_.height <= 0) {
    Result result;
    result.error = "spatial grid A* cost map is empty";
    return result;
  }
  Result result = search(world_to_grid(start), world_to_grid(goal));
  if (!result.success) {
    return result;
  }
  result.route.world_points.reserve(result.route.raw_path.size());
  for (const auto & cell : result.route.raw_path) {
    result.route.world_points.push_back(grid_to_world(cell.first, cell.second));
  }
  return result;
}

std::pair<int, int> SpatialGridAstar::world_to_grid(const Point2D & point) const
{
  return {
    static_cast<int>(std::floor((point.x - cost_map_.origin.x) / cost_map_.resolution)),
    static_cast<int>(std::floor((point.y - cost_map_.origin.y) / cost_map_.resolution))};
}

Point2D SpatialGridAstar::grid_to_world(int x, int y) const
{
  return {
    cost_map_.origin.x + (static_cast<double>(x) + 0.5) * cost_map_.resolution,
    cost_map_.origin.y + (static_cast<double>(y) + 0.5) * cost_map_.resolution};
}

}  // namespace astra_nav
