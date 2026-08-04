#include "astra_planning/local_goal_selector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "astra_common/common.hpp"

namespace astra_nav
{

LocalGoalSelector::LocalGoalSelector(const LocalGoalSelectorConfig & config)
: config_(config)
{
  config_.required_clearance = std::max(0.0, config_.required_clearance);
  config_.backoff_distance = std::max(0.0, config_.backoff_distance);
  config_.sample_step = std::max(1.0e-3, config_.sample_step);
  config_.min_progress_distance = std::max(0.0, config_.min_progress_distance);
}

LocalGoalSelectionResult LocalGoalSelector::select(
  const Point2D & start, const Point2D & global_goal, const Grid2D & traversable_grid,
  const EsdfMap & esdf) const
{
  LocalGoalSelectionResult result;
  result.global_goal = global_goal;
  result.local_goal = global_goal;
  result.distance_to_global_goal = 0.0;

  if (!esdf.valid() || traversable_grid.width <= 0 || traversable_grid.height <= 0) {
    return result;
  }

  const double total_distance = distance_2d(start, global_goal);
  if (total_distance < 1.0e-6) {
    result.success = is_valid(global_goal, traversable_grid, esdf);
    result.progress_ratio = result.success ? 1.0 : 0.0;
    return result;
  }

  if (is_valid(global_goal, traversable_grid, esdf)) {
    result.success = true;
    result.progress_ratio = 1.0;
    return result;
  }

  const double step = std::max(config_.sample_step, 0.5 * esdf.resolution());
  const int sample_count = std::max(1, static_cast<int>(std::ceil(total_distance / step)));
  double best_s = -1.0;
  Point2D best_point = start;

  for (int i = 0; i <= sample_count; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(sample_count);
    const Point2D candidate{
      start.x + (global_goal.x - start.x) * ratio,
      start.y + (global_goal.y - start.y) * ratio};
    if (is_valid(candidate, traversable_grid, esdf)) {
      best_s = ratio * total_distance;
      best_point = candidate;
    } else {
      // 视线检查（M6）：所有候选点都在 start->global_goal 的同一条直线上，
      // 一旦某采样点不可通行，其后的点都被该障碍沿直线遮挡，与起点失去直线视线。
      // 此时停止前向扫描，避免选到“障碍另一侧”的局部目标（否则会浪费一次 JPS 规划）。
      break;
    }
  }

  if (best_s < 0.0) {
    return result;
  }

  const double target_s = std::max(0.0, best_s - config_.backoff_distance);
  Point2D selected = best_point;
  double selected_s = best_s;
  for (int i = 0; i <= sample_count; ++i) {
    const double s = target_s - static_cast<double>(i) * step;
    if (s < 0.0) {
      break;
    }
    const double ratio = s / total_distance;
    const Point2D candidate{
      start.x + (global_goal.x - start.x) * ratio,
      start.y + (global_goal.y - start.y) * ratio};
    if (is_valid(candidate, traversable_grid, esdf)) {
      selected = candidate;
      selected_s = s;
      break;
    }
  }

  if (selected_s < config_.min_progress_distance && total_distance > config_.min_progress_distance) {
    const auto lateral = select_lateral_goal(start, global_goal, traversable_grid, esdf, total_distance);
    if (lateral.success) {
      return lateral;
    }
  }

  result.local_goal = selected;
  result.success = true;
  result.clipped = true;
  result.progress_ratio = std::clamp(selected_s / total_distance, 0.0, 1.0);
  result.distance_to_global_goal = distance_2d(selected, global_goal);
  return result;
}

LocalGoalSelectionResult LocalGoalSelector::select_lateral_goal(
  const Point2D & start, const Point2D & global_goal, const Grid2D & traversable_grid,
  const EsdfMap & esdf, double total_distance) const
{
  LocalGoalSelectionResult result;
  result.global_goal = global_goal;
  result.local_goal = global_goal;
  result.distance_to_global_goal = distance_2d(start, global_goal);

  if (total_distance < 1.0e-6) {
    return result;
  }

  const Point2D direction{
    (global_goal.x - start.x) / total_distance,
    (global_goal.y - start.y) / total_distance};
  const Point2D normal{-direction.y, direction.x};

  double best_score = -std::numeric_limits<double>::infinity();
  double best_progress = 0.0;
  Point2D best_point = start;

  for (int y = 0; y < traversable_grid.height; ++y) {
    for (int x = 0; x < traversable_grid.width; ++x) {
      const Point2D candidate{
        traversable_grid.origin.x + (static_cast<double>(x) + 0.5) * traversable_grid.resolution,
        traversable_grid.origin.y + (static_cast<double>(y) + 0.5) * traversable_grid.resolution};
      if (!is_valid(candidate, traversable_grid, esdf)) {
        continue;
      }

      const Point2D delta{candidate.x - start.x, candidate.y - start.y};
      const double progress = delta.x * direction.x + delta.y * direction.y;
      if (progress < config_.min_progress_distance) {
        continue;
      }
      const double lateral = std::abs(delta.x * normal.x + delta.y * normal.y);
      const double distance_to_global = distance_2d(candidate, global_goal);
      const double effective_progress = std::min(progress, total_distance);
      const double overshoot = std::max(0.0, progress - total_distance);
      const double score =
        effective_progress - 0.75 * lateral - 0.20 * distance_to_global - 1.00 * overshoot;
      if (score > best_score) {
        best_score = score;
        best_progress = effective_progress;
        best_point = candidate;
      }
    }
  }

  if (!std::isfinite(best_score)) {
    return result;
  }

  result.local_goal = best_point;
  result.success = true;
  result.clipped = true;
  result.progress_ratio = std::clamp(best_progress / total_distance, 0.0, 1.0);
  result.distance_to_global_goal = distance_2d(best_point, global_goal);
  return result;
}

bool LocalGoalSelector::is_valid(
  const Point2D & point, const Grid2D & traversable_grid, const EsdfMap & esdf) const
{
  const auto esdf_index = esdf.world_to_grid(point.x, point.y);
  if (!esdf.in_bounds(esdf_index.first, esdf_index.second)) {
    return false;
  }
  if (esdf.query_distance(point.x, point.y) < config_.required_clearance) {
    return false;
  }
  if (!config_.require_grid_free) {
    return true;
  }

  const int gx = static_cast<int>(std::floor((point.x - traversable_grid.origin.x) / traversable_grid.resolution));
  const int gy = static_cast<int>(std::floor((point.y - traversable_grid.origin.y) / traversable_grid.resolution));
  if (gx < 0 || gy < 0 || gx >= traversable_grid.width || gy >= traversable_grid.height) {
    return false;
  }
  return traversable_grid.at(gx, gy) == 0;
}

}  // namespace astra_nav
