#include "astra_control/se2_mpc_preview.hpp"

#include <algorithm>
#include <cmath>

namespace astra_nav
{

Se2MpcPreview::Se2MpcPreview(const Se2MpcPreviewConfig & config)
: config_(config)
{
  config_.steps = std::max(1, config_.steps);
  config_.dt = std::max(1.0e-3, config_.dt);
}

Se2MpcPreviewResult Se2MpcPreview::predict(
  const Pose2D & pose, const Twist2D & held_command,
  const std::vector<TrajectoryPoint> & trajectory, double current_time_from_start) const
{
  Se2MpcPreviewResult result;
  if (trajectory.empty()) {
    return result;
  }

  Pose2D predicted = pose;
  double error_sum = 0.0;
  double max_error = 0.0;
  for (int i = 1; i <= config_.steps; ++i) {
    const double cy = std::cos(predicted.yaw);
    const double sy = std::sin(predicted.yaw);
    const double vx_world = cy * held_command.vx - sy * held_command.vy;
    const double vy_world = sy * held_command.vx + cy * held_command.vy;
    predicted.x += vx_world * config_.dt;
    predicted.y += vy_world * config_.dt;
    predicted.yaw = normalize_angle(predicted.yaw + held_command.wz * config_.dt);

    const double target_time = current_time_from_start + static_cast<double>(i) * config_.dt;
    const auto reference = nearest_reference(trajectory, target_time);
    const double error = std::hypot(predicted.x - reference.x, predicted.y - reference.y);
    error_sum += error;
    max_error = std::max(max_error, error);

    if (i == config_.steps) {
      result.final_prediction.t = target_time;
      result.final_prediction.x = predicted.x;
      result.final_prediction.y = predicted.y;
      result.final_prediction.yaw = predicted.yaw;
      result.final_prediction.vx = vx_world;
      result.final_prediction.vy = vy_world;
      result.final_prediction.ax = 0.0;
      result.final_prediction.ay = 0.0;
      result.final_reference = reference;
      result.final_position_error = error;
    }
  }

  result.valid = true;
  result.steps = config_.steps;
  result.horizon_duration = static_cast<double>(config_.steps) * config_.dt;
  result.mean_position_error = error_sum / static_cast<double>(config_.steps);
  result.max_position_error = max_error;
  return result;
}

TrajectoryPoint Se2MpcPreview::nearest_reference(
  const std::vector<TrajectoryPoint> & trajectory, double target_time) const
{
  const auto it = std::min_element(
    trajectory.begin(), trajectory.end(),
    [target_time](const auto & a, const auto & b) {
      return std::abs(a.t - target_time) < std::abs(b.t - target_time);
    });
  return *it;
}

}  // namespace astra_nav
