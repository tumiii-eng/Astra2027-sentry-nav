#include "astra_control/preview_controller.hpp"

#include <algorithm>
#include <cmath>

namespace astra_nav
{

PreviewController::PreviewController(const PreviewControllerConfig & config)
: config_(config)
{
}

Twist2D PreviewController::compute(
  const Pose2D & pose, const Twist2D & world_velocity,
  const std::vector<TrajectoryPoint> & trajectory,
  double current_time_from_start)
{
  if (trajectory.empty()) {
    last_cmd_ = {};
    last_debug_ = {};
    return last_cmd_;
  }

  const double target_t = current_time_from_start + config_.lookahead_time;
  const auto nearest_it = std::min_element(
    trajectory.begin(), trajectory.end(),
    [current_time_from_start](const auto & a, const auto & b) {
      return std::abs(a.t - current_time_from_start) < std::abs(b.t - current_time_from_start);
    });
  const auto target_it = std::min_element(
    trajectory.begin(), trajectory.end(),
    [target_t](const auto & a, const auto & b) {
      return std::abs(a.t - target_t) < std::abs(b.t - target_t);
    });
  const auto & nearest = *nearest_it;
  const auto & target = *target_it;

  const double dx = target.x - pose.x;
  const double dy = target.y - pose.y;
  const double nearest_position_error = std::hypot(nearest.x - pose.x, nearest.y - pose.y);
  const double target_position_error = std::hypot(dx, dy);
  const double nearest_reference_speed = std::hypot(nearest.vx, nearest.vy);
  const double target_reference_speed = std::hypot(target.vx, target.vy);
  const double cy = std::cos(pose.yaw);
  const double sy = std::sin(pose.yaw);
  const double err_x_body = cy * dx + sy * dy;
  const double err_y_body = -sy * dx + cy * dy;
  const double ff_vx_body = cy * target.vx + sy * target.vy;
  const double ff_vy_body = -sy * target.vx + cy * target.vy;
  // 真实速度旋到机体系，用于速度阻尼项（真实速度反馈，替代旧的"阻尼自身上一条命令"）。
  const double actual_vx_body = cy * world_velocity.vx + sy * world_velocity.vy;
  const double actual_vy_body = -sy * world_velocity.vx + cy * world_velocity.vy;
  const double raw_target_heading_error = normalize_angle(target.yaw - pose.yaw);
  double heading_control_weight = 0.0;
  double target_heading_error = 0.0;
  if (config_.path_heading_tracking_enabled &&
    target_reference_speed >= config_.heading_control_min_reference_speed)
  {
    heading_control_weight = 1.0;
    if (config_.heading_startup_ramp_time > 1.0e-3) {
      heading_control_weight = std::clamp(
        current_time_from_start / config_.heading_startup_ramp_time, 0.0, 1.0);
    }

    const double abs_heading_error = std::abs(raw_target_heading_error);
    if (abs_heading_error > config_.heading_error_deadband) {
      target_heading_error =
        std::copysign(abs_heading_error - config_.heading_error_deadband, raw_target_heading_error);
    }
  }

  Twist2D cmd;
  const bool stationary_reference =
    nearest_reference_speed <= config_.arrival_reference_speed_tolerance &&
    target_reference_speed <= config_.arrival_reference_speed_tolerance;
  const bool reached_hold_target =
    stationary_reference && target_position_error <= config_.arrival_position_tolerance;

  if (!reached_hold_target) {
    // 控制律 = 前馈 + 位置比例 - 速度阻尼(真实速度相对参考速度的超出量)。
    //   cmd = v_ref + kp*e_p - kd*(v_actual - v_ref)
    // 巡航段 v_actual≈v_ref → 阻尼项≈0，纯前馈+位置跟踪，不损跟踪性能；
    // 终点段 v_ref→0 → 退化为 kp*e_p - kd*v_actual 的 PD 主动刹车，惯性被真实速度直接抵消，
    // 消除"冲过终点→仅按位置回拉→欠阻尼振荡"的过冲（停不稳根因）。
    cmd.vx = ff_vx_body + config_.position_kp * err_x_body -
      config_.velocity_damping * (actual_vx_body - ff_vx_body);
    cmd.vy = ff_vy_body + config_.position_kp * err_y_body -
      config_.velocity_damping * (actual_vy_body - ff_vy_body);

    const double speed = std::hypot(cmd.vx, cmd.vy);
    if (speed > config_.max_linear_velocity) {
      const double scale = config_.max_linear_velocity / std::max(speed, 1e-6);
      cmd.vx *= scale;
      cmd.vy *= scale;
    }

    cmd.wz = config_.heading_kp * target_heading_error;
    cmd.wz = std::clamp(cmd.wz, -config_.max_angular_velocity, config_.max_angular_velocity);
  }
  last_cmd_ = cmd;

  last_debug_.valid = true;
  last_debug_.current_time = current_time_from_start;
  last_debug_.lookahead_time = config_.lookahead_time;
  last_debug_.nearest_reference_time = nearest.t;
  last_debug_.target_reference_time = target.t;
  last_debug_.nearest_position_error = nearest_position_error;
  last_debug_.target_position_error = target_position_error;
  last_debug_.raw_target_heading_error = raw_target_heading_error;
  last_debug_.target_heading_error = target_heading_error;
  last_debug_.heading_control_weight = heading_control_weight;
  last_debug_.nearest_reference_speed = nearest_reference_speed;
  last_debug_.target_reference_speed = target_reference_speed;
  last_debug_.actual_body_speed = std::hypot(actual_vx_body, actual_vy_body);
  last_debug_.command_linear_speed = std::hypot(cmd.vx, cmd.vy);
  last_debug_.command_angular_speed = std::abs(cmd.wz);
  last_debug_.stopped_by_arrival_tolerance = reached_hold_target;
  last_debug_.nearest_reference = nearest;
  last_debug_.target_reference = target;
  return cmd;
}

const PreviewControlDebug & PreviewController::last_debug() const
{
  return last_debug_;
}

}  // namespace astra_nav
