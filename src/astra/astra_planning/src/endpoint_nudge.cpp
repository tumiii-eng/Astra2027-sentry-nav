#include "astra_planning/endpoint_nudge.hpp"

#include <cmath>
#include <cstdint>
#include <queue>
#include <utility>
#include <vector>

#include "astra_mapping/cost_map_sampler.hpp"

namespace astra_nav
{

namespace
{

bool is_map_point_feasible(
  const Grid2D & cost_map, const Point2D & point, const int occupied_threshold)
{
  const auto cost = sample_cost_map(cost_map, point);
  if (!cost) {return false;}
  return cost->value < static_cast<double>(occupied_threshold);
}

bool contains_map_point(const Grid2D & cost_map, const Point2D & point)
{
  const double max_x = cost_map.origin.x + cost_map.resolution * cost_map.width;
  const double max_y = cost_map.origin.y + cost_map.resolution * cost_map.height;
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         point.x >= cost_map.origin.x && point.y >= cost_map.origin.y &&
         point.x < max_x && point.y < max_y;
}

Point2D cell_center(const Grid2D & cost_map, const int x, const int y)
{
  return {
    cost_map.origin.x + cost_map.resolution * (static_cast<double>(x) + 0.5),
    cost_map.origin.y + cost_map.resolution * (static_cast<double>(y) + 0.5)};
}

double distance_to(const Point2D & from, const Point2D & to)
{
  return std::hypot(to.x - from.x, to.y - from.y);
}

}  // namespace

std::optional<Point2D> nudge_point_to_free(
  const Grid2D & cost_map,
  const Point2D & point,
  const double max_nudge_distance,
  const int occupied_threshold)
{
  const int width = cost_map.width;
  const int height = cost_map.height;
  if (width <= 0 || height <= 0) {
    return std::nullopt;
  }

  const auto key = [width](const int x, const int y) {
      return static_cast<std::size_t>(y) * static_cast<std::size_t>(width) +
             static_cast<std::size_t>(x);
    };

  if (contains_map_point(cost_map, point) &&
    is_map_point_feasible(cost_map, point, occupied_threshold))
  {
    return point;
  }

  if (!contains_map_point(cost_map, point)) {
    return std::nullopt;
  }
  const int sx = static_cast<int>(std::floor((point.x - cost_map.origin.x) / cost_map.resolution));
  const int sy = static_cast<int>(std::floor((point.y - cost_map.origin.y) / cost_map.resolution));

  std::vector<std::uint8_t> visited(
    static_cast<std::size_t>(width) * static_cast<std::size_t>(height), 0);
  std::queue<std::pair<int, int>> q;
  q.push({sx, sy});
  visited[key(sx, sy)] = 1;

  static constexpr int dx[] = {0, 1, 0, -1, 1, 1, -1, -1};
  static constexpr int dy[] = {1, 0, -1, 0, 1, -1, 1, -1};

  while (!q.empty()) {
    const auto current = q.front();
    q.pop();

    const Point2D candidate = cell_center(cost_map, current.first, current.second);
    const double dist = distance_to(point, candidate);
    if (dist > max_nudge_distance) {continue;}

    if (is_map_point_feasible(cost_map, candidate, occupied_threshold)) {
      return candidate;
    }

    for (int i = 0; i < 8; i++) {
      const int nx = current.first + dx[i];
      const int ny = current.second + dy[i];
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {continue;}
      const double ndist = distance_to(point, cell_center(cost_map, nx, ny));
      if (ndist > max_nudge_distance) {continue;}
      const std::size_t nk = key(nx, ny);
      if (visited[nk]) {continue;}
      visited[nk] = 1;
      q.push({nx, ny});
    }
  }

  return std::nullopt;
}

}  // namespace astra_nav
