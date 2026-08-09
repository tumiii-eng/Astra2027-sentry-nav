#include "astra_control/route_tracker.hpp"

#include <algorithm>
#include <cmath>

namespace astra_nav
{

RouteGeometry::RouteGeometry(const std::vector<TrajectoryPoint> & trajectory)
{
  if (trajectory.size() < 2) {
    return;
  }
  samples_ = trajectory;
  arc_lengths_.resize(samples_.size());
  segment_yaws_.resize(samples_.size() - 1);
  arc_lengths_[0] = 0.0;
  double last_yaw = samples_[0].yaw;
  for (std::size_t i = 1; i < samples_.size(); ++i) {
    const double dx = samples_[i].x - samples_[i - 1].x;
    const double dy = samples_[i].y - samples_[i - 1].y;
    const double length = std::hypot(dx, dy);
    arc_lengths_[i] = arc_lengths_[i - 1] + length;
    // 弦长退化时切线不可定义，沿用上一段方向，避免 atan2(0,0) 产生 0 航向断点。
    last_yaw = length > 1.0e-9 ? std::atan2(dy, dx) : last_yaw;
    segment_yaws_[i - 1] = last_yaw;
  }
  for (const auto & point : samples_) {
    if (std::hypot(point.vx, point.vy) > 1.0e-6) {
      has_speed_profile_ = true;
      break;
    }
  }
}

RouteSample RouteGeometry::eval_arc_length(const double arc_length) const
{
  RouteSample sample;
  if (samples_.empty()) {
    return sample;
  }
  if (samples_.size() == 1) {
    sample.position = {samples_[0].x, samples_[0].y};
    sample.tangent_yaw = samples_[0].yaw;
    sample.profile_speed = std::hypot(samples_[0].vx, samples_[0].vy);
    sample.time = samples_[0].t;
    return sample;
  }

  const double total = arc_lengths_.back();
  const double s = std::clamp(arc_length, 0.0, total);
  // 前缀弧长单调不减，直接二分定位所在段。
  const auto upper = std::upper_bound(arc_lengths_.begin(), arc_lengths_.end(), s);
  std::size_t index = upper == arc_lengths_.begin()
    ? 0
    : static_cast<std::size_t>(std::distance(arc_lengths_.begin(), upper)) - 1;
  index = std::min(index, samples_.size() - 2);

  const double segment_length = arc_lengths_[index + 1] - arc_lengths_[index];
  const double ratio = segment_length > 1.0e-9 ? (s - arc_lengths_[index]) / segment_length : 0.0;
  const auto & a = samples_[index];
  const auto & b = samples_[index + 1];
  sample.position = {a.x + ratio * (b.x - a.x), a.y + ratio * (b.y - a.y)};
  sample.tangent_yaw = segment_yaws_[index];
  const double speed_a = std::hypot(a.vx, a.vy);
  const double speed_b = std::hypot(b.vx, b.vy);
  sample.profile_speed = speed_a + ratio * (speed_b - speed_a);
  sample.time = a.t + ratio * (b.t - a.t);
  return sample;
}

void RouteTracker::reset()
{
  geometry_ = RouteGeometry{};
  has_geometry_ = false;
  revision_ = 0;
  hypotheses_.clear();
  reported_arc_length_ = 0.0;
  last_stamp_.reset();
}

std::optional<RouteEstimate> RouteTracker::update(
  const std::vector<TrajectoryPoint> & trajectory, const std::uint64_t trajectory_revision,
  const Pose2D & pose, const Twist2D & world_velocity, const double stamp_seconds)
{
  if (trajectory.size() < 2) {
    reset();
    return std::nullopt;
  }
  if (!has_geometry_ || trajectory_revision != revision_) {
    geometry_ = RouteGeometry(trajectory);
    revision_ = trajectory_revision;
    has_geometry_ = true;
    hypotheses_.clear();
    reported_arc_length_ = 0.0;
    last_stamp_.reset();
  }
  if (geometry_.empty()) {
    reset();
    return std::nullopt;
  }

  const RouteGeometry & geometry = geometry_;
  const double total_length = geometry.total_arc_length();
  const Point2D position{pose.x, pose.y};
  // 全向底盘：地图系速度矢量直接可观测，无需由前向标量速度与航向重建。
  const double velocity_map_x = world_velocity.vx;
  const double velocity_map_y = world_velocity.vy;
  const double spacing = std::max(params_.hypothesis_spacing, 1.0e-3);

  // 在 [lo, hi] 上按 hypothesis_spacing 枚举有向进度候选，端点必取。
  const auto for_each_candidate = [&](const double lo, const double hi, auto && visit) {
      visit(lo);
      const int first = static_cast<int>(std::floor(lo / spacing)) + 1;
      const int last = static_cast<int>(std::ceil(hi / spacing)) - 1;
      for (int index = first; index <= last; ++index) {
        visit(static_cast<double>(index) * spacing);
      }
      if (hi > lo) {
        visit(hi);
      }
    };

  const double dt = last_stamp_
    ? std::clamp(stamp_seconds - *last_stamp_, 0.0, params_.prediction_time_limit)
    : 0.0;
  const bool initializing = hypotheses_.empty();
  const std::vector<Hypothesis> initial_state{Hypothesis{0.0, 0.0, 0.0}};
  const std::vector<Hypothesis> & previous_states = initializing ? initial_state : hypotheses_;

  const double velocity_weight = 1.0 / std::pow(params_.velocity_sigma, 2);
  const double progress_weight = 1.0 / std::pow(params_.progress_sigma, 2);
  const bool has_speed_profile = geometry.has_speed_profile();
  const double profile_weight =
    has_speed_profile ? 1.0 / std::pow(params_.profile_speed_sigma, 2) : 0.0;
  const double dynamics_weight = 1.0 / std::pow(params_.speed_dynamics_sigma, 2);

  std::vector<Hypothesis> candidates;
  for (const Hypothesis & previous : previous_states) {
    const double nominal_advance = previous.path_speed * dt;
    const double lo = initializing ? 0.0 : reported_arc_length_;
    const double hi = initializing
      ? std::min(total_length, params_.initial_search_distance)
      : std::min(total_length, previous.arc_length + 2.0 * nominal_advance + spacing);
    for_each_candidate(
      lo, std::max(lo, hi), [&](const double arc_length) {
        const RouteSample sample = geometry.eval_arc_length(arc_length);
        const double tangent_x = std::cos(sample.tangent_yaw);
        const double tangent_y = std::sin(sample.tangent_yaw);
        const double profile_speed =
        !has_speed_profile ? previous.path_speed : sample.profile_speed;

        // 固定 s 后，统一后验关于 nu 是一维二次函数，可直接求其受限最小值。
        const double half_dt = 0.5 * dt;
        const double progress_offset =
        arc_length - previous.arc_length - half_dt * previous.path_speed;
        const double denominator = velocity_weight + profile_weight + dynamics_weight +
        progress_weight * half_dt * half_dt;
        const double numerator =
        velocity_weight * (tangent_x * velocity_map_x + tangent_y * velocity_map_y) +
        profile_weight * profile_speed +
        dynamics_weight * previous.path_speed +
        progress_weight * half_dt * progress_offset;
        const double path_speed =
        std::clamp(numerator / denominator, 0.0, params_.max_path_speed);
        const double predicted =
        previous.arc_length + half_dt * (previous.path_speed + path_speed);
        const double position_residual =
        std::hypot(position.x - sample.position.x, position.y - sample.position.y) /
        params_.position_sigma;
        const double velocity_residual =
        std::hypot(
          velocity_map_x - path_speed * tangent_x,
          velocity_map_y - path_speed * tangent_y) / params_.velocity_sigma;
        const double progress_residual = (arc_length - predicted) / params_.progress_sigma;
        const double profile_residual = has_speed_profile
        ? (path_speed - profile_speed) / params_.profile_speed_sigma
        : 0.0;
        const double dynamics_residual =
        (path_speed - previous.path_speed) / params_.speed_dynamics_sigma;
        candidates.push_back(
          Hypothesis{
            arc_length,
            path_speed,
            previous.cost +
            position_residual * position_residual +
            velocity_residual * velocity_residual +
            progress_residual * progress_residual +
            profile_residual * profile_residual +
            dynamics_residual * dynamics_residual,
          });
      });
  }
  if (candidates.empty()) {
    reset();
    return std::nullopt;
  }

  // 按代价保留领先的竞争分支；弧长过近的候选只留最优者，避免假设集退化为同一分支。
  std::sort(
    candidates.begin(), candidates.end(),
    [](const Hypothesis & a, const Hypothesis & b) { return a.cost < b.cost; });
  const double best_cost = candidates.front().cost;
  const double cost_ceiling =
    best_cost + params_.hypothesis_prune_ratio * std::max(std::abs(best_cost), 1.0);
  hypotheses_.clear();
  for (const Hypothesis & candidate : candidates) {
    if (static_cast<int>(hypotheses_.size()) >= params_.max_hypotheses) {
      break;
    }
    if (candidate.cost > cost_ceiling) {
      break;
    }
    const bool duplicate = std::any_of(
      hypotheses_.begin(), hypotheses_.end(),
      [&](const Hypothesis & kept) {
        return std::abs(kept.arc_length - candidate.arc_length) < spacing;
      });
    if (duplicate) {
      continue;
    }
    hypotheses_.push_back(candidate);
  }
  // 代价按帧归一，防止长时间跟随后累计量溢出并保持假设间的相对可比性。
  for (Hypothesis & hypothesis : hypotheses_) {
    hypothesis.cost -= best_cost;
  }

  const Hypothesis & best = hypotheses_.front();
  const RouteSample reference = geometry.eval_arc_length(best.arc_length);
  const double tracking_error =
    std::hypot(reference.position.x - position.x, reference.position.y - position.y);
  reported_arc_length_ = best.arc_length;
  last_stamp_ = stamp_seconds;

  RouteEstimate estimate;
  estimate.status = tracking_error <= params_.max_tracking_error
    ? RouteTrackingStatus::TRACKED
    : RouteTrackingStatus::LOST;
  estimate.arc_length = best.arc_length;
  estimate.path_speed = best.path_speed;
  estimate.remaining_length = std::max(0.0, total_length - best.arc_length);
  estimate.reference_position = reference.position;
  estimate.tracking_error = tracking_error;
  estimate.reference_time = reference.time;
  estimate.tangent_yaw = reference.tangent_yaw;
  estimate.profile_speed = reference.profile_speed;
  estimate.hypothesis_count = static_cast<int>(hypotheses_.size());
  return estimate;
}

}  // namespace astra_nav
