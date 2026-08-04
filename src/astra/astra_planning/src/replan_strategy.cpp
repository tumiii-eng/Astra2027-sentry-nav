#include "astra_planning/replan_strategy.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace astra_nav
{

ReplanStrategy::ReplanStrategy(const ReplanDecisionConfig & config)
: config_(config)
{
  config_.goal_change_threshold = std::max(0.0, config_.goal_change_threshold);
  config_.localization_deviation_threshold = std::max(0.0, config_.localization_deviation_threshold);
  config_.collision_clearance = std::max(0.0, config_.collision_clearance);
  config_.good_tracking_threshold = std::max(0.0, config_.good_tracking_threshold);
}

const char * ReplanStrategy::mode_name(ReplanMode mode)
{
  switch (mode) {
    case ReplanMode::FullReplan:
      return "完全重规划";
    case ReplanMode::PartialReplan:
      return "部分重规划";
    case ReplanMode::OptimizeOnly:
      return "仅优化";
    default:
      return "未知";
  }
}

bool ReplanStrategy::forward_collision_free(
  const MincoTrajectory & trajectory, double from_t, const EsdfMap & esdf,
  double & min_clearance) const
{
  min_clearance = std::numeric_limits<double>::infinity();
  if (trajectory.empty() || !esdf.valid()) {
    return true;
  }
  const double duration = trajectory.duration();
  from_t = std::clamp(from_t, 0.0, duration);
  // 只检查映射点到末端的“前方”轨迹：机器人已走过的部分不再影响安全性。
  constexpr double kStep = 0.05;
  for (double t = from_t; t <= duration + 1.0e-9; t += kStep) {
    const auto p = trajectory.evaluate(std::min(t, duration));
    if (!esdf.in_map(p.x, p.y)) {
      continue;
    }
    min_clearance = std::min(min_clearance, esdf.query_distance(p.x, p.y));
  }
  if (!std::isfinite(min_clearance)) {
    min_clearance = 0.0;
    return true;
  }
  return min_clearance >= config_.collision_clearance;
}

ReplanDecision ReplanStrategy::decide(
  const MincoTrajectory & previous_trajectory, bool has_previous,
  const Point2D & current_position, const Point2D & current_goal,
  const Point2D & last_goal, bool has_last_goal, const EsdfMap & esdf) const
{
  ReplanDecision decision;

  // 目标跳变量。
  decision.goal_change = has_last_goal ? distance_2d(current_goal, last_goal) : 0.0;
  decision.goal_stable = decision.goal_change <= config_.goal_change_threshold;

  // 没有可继承的上一轨迹（首次规划），或目标大幅跳变：完全重规划。
  if (!has_previous || previous_trajectory.empty()) {
    decision.mode = ReplanMode::FullReplan;
    decision.mapped_position = current_position;
    return decision;
  }
  if (has_last_goal && !decision.goal_stable) {
    decision.mode = ReplanMode::FullReplan;
    decision.mapped_position = current_position;
    return decision;
  }

  // 当前位置在上一轨迹上的映射点（最近点投影）。
  const double mapped_t = previous_trajectory.project(current_position);
  const auto mapped = previous_trajectory.evaluate(mapped_t);
  decision.mapped_t = mapped_t;
  decision.mapped_position = {mapped.x, mapped.y};
  decision.mapped_velocity = {mapped.vx, mapped.vy};

  // 定位偏离：当前位置到映射点的距离。严重偏离 -> 完全重规划。
  decision.localization_deviation = distance_2d(current_position, decision.mapped_position);
  if (decision.localization_deviation > config_.localization_deviation_threshold) {
    decision.mode = ReplanMode::FullReplan;
    return decision;
  }

  // 前方轨迹碰撞检测与跟踪质量。
  double min_clearance = 0.0;
  decision.trajectory_collision_free =
    forward_collision_free(previous_trajectory, mapped_t, esdf, min_clearance);
  decision.min_trajectory_clearance = min_clearance;
  decision.tracking_good = decision.localization_deviation <= config_.good_tracking_threshold;

  // 仅优化：轨迹无碰撞 + 目标不跳变 + 跟踪良好。
  if (config_.optimize_only_enabled && decision.trajectory_collision_free &&
    decision.goal_stable && decision.tracking_good)
  {
    decision.mode = ReplanMode::OptimizeOnly;
    return decision;
  }

  // 其余情况：部分重规划（若禁用则退化为完全重规划）。
  decision.mode = config_.partial_enabled ? ReplanMode::PartialReplan : ReplanMode::FullReplan;
  return decision;
}

}  // namespace astra_nav
