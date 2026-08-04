#include "astra_planning/jps_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>

namespace astra_nav
{

namespace
{
constexpr double kSqrt2 = 1.41421356237309514880;
}  // namespace

JpsPlanner::JpsPlanner(const Grid2D & obstacle_grid)
: grid_(obstacle_grid)
{
}

bool JpsPlanner::has_forced_neighbor(int x, int y, int dx, int dy) const
{
  // 标准 Harabor-Grastien 强制邻居判定。
  if (dx != 0 && dy != 0) {
    // 对角移动：检查两个被障碍“逼出”的强制邻居。
    if (!is_free(x - dx, y) && is_free(x - dx, y + dy)) {
      return true;
    }
    if (!is_free(x, y - dy) && is_free(x + dx, y - dy)) {
      return true;
    }
  } else if (dx != 0) {
    // 水平移动：上下方被障碍挡住但其前方可通行 -> 强制邻居。
    if (!is_free(x, y + 1) && is_free(x + dx, y + 1)) {
      return true;
    }
    if (!is_free(x, y - 1) && is_free(x + dx, y - 1)) {
      return true;
    }
  } else if (dy != 0) {
    // 垂直移动：左右方被障碍挡住但其前方可通行 -> 强制邻居。
    if (!is_free(x + 1, y) && is_free(x + 1, y + dy)) {
      return true;
    }
    if (!is_free(x - 1, y) && is_free(x - 1, y + dy)) {
      return true;
    }
  }
  return false;
}

std::optional<std::pair<int, int>> JpsPlanner::jump(
  int x, int y, int dx, int dy, int gx, int gy) const
{
  int nx = x + dx;
  int ny = y + dy;
  if (!is_free(nx, ny)) {
    return std::nullopt;
  }
  if (nx == gx && ny == gy) {
    return std::pair<int, int>{nx, ny};
  }
  if (has_forced_neighbor(nx, ny, dx, dy)) {
    return std::pair<int, int>{nx, ny};
  }

  if (dx != 0 && dy != 0) {
    // 对角跳跃：先沿两个正交分量探查，命中跳点则当前点即跳点。
    if (jump(nx, ny, dx, 0, gx, gy) || jump(nx, ny, 0, dy, gx, gy)) {
      return std::pair<int, int>{nx, ny};
    }
    // 对角推进要求至少一个正交方向可通行，避免穿过障碍角。
    if (!is_free(nx + dx, ny) && !is_free(nx, ny + dy)) {
      return std::nullopt;
    }
  }

  return jump(nx, ny, dx, dy, gx, gy);
}

std::vector<std::pair<int, int>> JpsPlanner::pruned_directions(int x, int y, int dx, int dy) const
{
  std::vector<std::pair<int, int>> dirs;
  if (dx == 0 && dy == 0) {
    // 起点：展开全部 8 邻域。
    for (int ix = -1; ix <= 1; ++ix) {
      for (int iy = -1; iy <= 1; ++iy) {
        if (ix != 0 || iy != 0) {
          dirs.emplace_back(ix, iy);
        }
      }
    }
    return dirs;
  }

  if (dx != 0 && dy != 0) {
    // 对角：自然邻居为两个正交方向与对角方向本身。
    dirs.emplace_back(dx, 0);
    dirs.emplace_back(0, dy);
    dirs.emplace_back(dx, dy);
    // 强制邻居：被障碍逼出的对角方向。
    if (!is_free(x - dx, y)) {
      dirs.emplace_back(-dx, dy);
    }
    if (!is_free(x, y - dy)) {
      dirs.emplace_back(dx, -dy);
    }
  } else if (dx != 0) {
    // 水平直线：自然邻居为前进方向；强制邻居为被上下障碍逼出的两个对角方向。
    dirs.emplace_back(dx, 0);
    if (!is_free(x, y + 1)) {
      dirs.emplace_back(dx, 1);
    }
    if (!is_free(x, y - 1)) {
      dirs.emplace_back(dx, -1);
    }
  } else {
    // 垂直直线：自然邻居为前进方向；强制邻居为被左右障碍逼出的两个对角方向。
    dirs.emplace_back(0, dy);
    if (!is_free(x + 1, y)) {
      dirs.emplace_back(1, dy);
    }
    if (!is_free(x - 1, y)) {
      dirs.emplace_back(-1, dy);
    }
  }
  return dirs;
}

std::optional<GridPath> JpsPlanner::plan(const Point2D & start, const Point2D & goal) const
{
  auto start_grid = nearest_free(world_to_grid(start));
  auto goal_grid = nearest_free(world_to_grid(goal));
  if (!start_grid || !goal_grid) {
    return std::nullopt;
  }
  const int gx = goal_grid->first;
  const int gy = goal_grid->second;

  struct QueueItem
  {
    double f{0.0};
    double g{0.0};
    int x{0};
    int y{0};
    int dx{0};
    int dy{0};
    bool operator<(const QueueItem & other) const { return f > other.f; }
  };

  const int total = grid_.width * grid_.height;
  std::vector<double> g_score(static_cast<std::size_t>(total), std::numeric_limits<double>::infinity());
  std::vector<int> parent(static_cast<std::size_t>(total), -1);
  std::vector<std::uint8_t> closed(static_cast<std::size_t>(total), 0);
  std::priority_queue<QueueItem> open;

  const int start_idx = flat_index(start_grid->first, start_grid->second);
  const int goal_idx = flat_index(gx, gy);
  g_score[static_cast<std::size_t>(start_idx)] = 0.0;
  open.push({heuristic(start_grid->first, start_grid->second, gx, gy), 0.0,
      start_grid->first, start_grid->second, 0, 0});

  while (!open.empty()) {
    const auto current = open.top();
    open.pop();
    const int current_idx = flat_index(current.x, current.y);
    if (closed[static_cast<std::size_t>(current_idx)] != 0) {
      continue;
    }
    closed[static_cast<std::size_t>(current_idx)] = 1;

    if (current_idx == goal_idx) {
      GridPath path;
      path.grid_points = reconstruct_path(parent, goal_idx);
      for (const auto & p : path.grid_points) {
        path.world_points.push_back(grid_to_world(p.first, p.second));
      }
      return path;
    }

    for (const auto & dir : pruned_directions(current.x, current.y, current.dx, current.dy)) {
      const auto jp = jump(current.x, current.y, dir.first, dir.second, gx, gy);
      if (!jp) {
        continue;
      }
      const int jx = jp->first;
      const int jy = jp->second;
      const int next_idx = flat_index(jx, jy);
      if (closed[static_cast<std::size_t>(next_idx)] != 0) {
        continue;
      }
      const double seg = std::hypot(jx - current.x, jy - current.y);
      const double new_g = g_score[static_cast<std::size_t>(current_idx)] + seg;
      if (new_g < g_score[static_cast<std::size_t>(next_idx)]) {
        g_score[static_cast<std::size_t>(next_idx)] = new_g;
        parent[static_cast<std::size_t>(next_idx)] = current_idx;
        const int ndx = (jx > current.x) - (jx < current.x);
        const int ndy = (jy > current.y) - (jy < current.y);
        open.push({new_g + heuristic(jx, jy, gx, gy), new_g, jx, jy, ndx, ndy});
      }
    }
  }

  return std::nullopt;
}

std::optional<double> JpsPlanner::astar_cost(const Point2D & start, const Point2D & goal) const
{
  auto start_grid = nearest_free(world_to_grid(start));
  auto goal_grid = nearest_free(world_to_grid(goal));
  if (!start_grid || !goal_grid) {
    return std::nullopt;
  }
  const int gx = goal_grid->first;
  const int gy = goal_grid->second;

  struct QueueItem
  {
    double f{0.0};
    double g{0.0};
    int x{0};
    int y{0};
    bool operator<(const QueueItem & other) const { return f > other.f; }
  };

  const int total = grid_.width * grid_.height;
  std::vector<double> g_score(static_cast<std::size_t>(total), std::numeric_limits<double>::infinity());
  std::vector<std::uint8_t> closed(static_cast<std::size_t>(total), 0);
  std::priority_queue<QueueItem> open;

  const int start_idx = flat_index(start_grid->first, start_grid->second);
  const int goal_idx = flat_index(gx, gy);
  g_score[static_cast<std::size_t>(start_idx)] = 0.0;
  open.push({heuristic(start_grid->first, start_grid->second, gx, gy), 0.0,
      start_grid->first, start_grid->second});

  const int dirs[8][2] = {
    {-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

  while (!open.empty()) {
    const auto current = open.top();
    open.pop();
    const int current_idx = flat_index(current.x, current.y);
    if (closed[static_cast<std::size_t>(current_idx)] != 0) {
      continue;
    }
    closed[static_cast<std::size_t>(current_idx)] = 1;
    if (current_idx == goal_idx) {
      return current.g;
    }

    for (const auto & dir : dirs) {
      const int nx = current.x + dir[0];
      const int ny = current.y + dir[1];
      if (!is_free(nx, ny)) {
        continue;
      }
      if (dir[0] != 0 && dir[1] != 0) {
        // 禁止穿越障碍角，与 JPS 的对角规则保持一致。
        if (!is_free(current.x + dir[0], current.y) && !is_free(current.x, current.y + dir[1])) {
          continue;
        }
      }
      const int next_idx = flat_index(nx, ny);
      if (closed[static_cast<std::size_t>(next_idx)] != 0) {
        continue;
      }
      const double step = (dir[0] != 0 && dir[1] != 0) ? kSqrt2 : 1.0;
      const double tentative = current.g + step;
      if (tentative < g_score[static_cast<std::size_t>(next_idx)]) {
        g_score[static_cast<std::size_t>(next_idx)] = tentative;
        open.push({tentative + heuristic(nx, ny, gx, gy), tentative, nx, ny});
      }
    }
  }
  return std::nullopt;
}

bool JpsPlanner::in_bounds(int x, int y) const
{
  return x >= 0 && y >= 0 && x < grid_.width && y < grid_.height;
}

bool JpsPlanner::is_free(int x, int y) const
{
  return in_bounds(x, y) && grid_.at(x, y) == 0;
}

std::pair<int, int> JpsPlanner::world_to_grid(const Point2D & point) const
{
  return {
    static_cast<int>(std::floor((point.x - grid_.origin.x) / grid_.resolution)),
    static_cast<int>(std::floor((point.y - grid_.origin.y) / grid_.resolution))};
}

Point2D JpsPlanner::grid_to_world(int x, int y) const
{
  return {
    grid_.origin.x + (static_cast<double>(x) + 0.5) * grid_.resolution,
    grid_.origin.y + (static_cast<double>(y) + 0.5) * grid_.resolution};
}

std::optional<std::pair<int, int>> JpsPlanner::nearest_free(std::pair<int, int> seed) const
{
  if (is_free(seed.first, seed.second)) {
    return seed;
  }
  constexpr int max_radius = 30;
  for (int r = 1; r <= max_radius; ++r) {
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        if (std::max(std::abs(dx), std::abs(dy)) != r) {
          continue;
        }
        const int x = seed.first + dx;
        const int y = seed.second + dy;
        if (is_free(x, y)) {
          return std::pair<int, int>{x, y};
        }
      }
    }
  }
  return std::nullopt;
}

std::vector<std::pair<int, int>> JpsPlanner::reconstruct_path(
  const std::vector<int> & parent, int goal_index) const
{
  // 跳点序列：相邻跳点之间是直线，这里把每个跳点端点保留。
  std::vector<std::pair<int, int>> reversed;
  int current = goal_index;
  while (current >= 0) {
    const int x = current % grid_.width;
    const int y = current / grid_.width;
    reversed.emplace_back(x, y);
    current = parent[static_cast<std::size_t>(current)];
  }
  std::reverse(reversed.begin(), reversed.end());
  return reversed;
}

double JpsPlanner::heuristic(int x, int y, int gx, int gy) const
{
  // 八方向（octile）距离启发，可采纳且与对角步代价一致。
  const int adx = std::abs(x - gx);
  const int ady = std::abs(y - gy);
  return static_cast<double>(std::max(adx, ady)) +
    (kSqrt2 - 1.0) * static_cast<double>(std::min(adx, ady));
}

}  // namespace astra_nav
