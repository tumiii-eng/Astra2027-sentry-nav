#include "astra_control/preview_controller.hpp"

#include <algorithm>
#include <cmath>

namespace astra_nav
{

namespace
{

// 无外部进度参考时的兜底：在采样折线上按时间取最近点，并用相邻弦方向作为切线。
// 只取【当前时刻】的点，不做时间前瞻——与上游 reference_frame(s) 语义一致。
RouteReferenceFrame reference_at_time(
  const std::vector<TrajectoryPoint> & trajectory, double time_from_start)
{
  RouteReferenceFrame frame;
  const auto nearest_it = std::min_element(
    trajectory.begin(), trajectory.end(),
    [time_from_start](const auto & a, const auto & b) {
      return std::abs(a.t - time_from_start) < std::abs(b.t - time_from_start);
    });
  const std::size_t index = static_cast<std::size_t>(
    std::distance(trajectory.begin(), nearest_it));
  const auto & point = *nearest_it;
  frame.position = {point.x, point.y};
  frame.profile_speed = std::hypot(point.vx, point.vy);
  frame.reference_time = point.t;

  // 切线取相邻弦方向，与 RouteGeometry::eval_arc_length 的 segment_yaws_ 同一约定
  // （真实运行时参考点来自 RouteTracker，兜底路径必须用同一几何定义，否则两条路径的
  // 切/法向分解不一致）。弦长退化时退回轨迹速度矢量，再退回采样点自身航向。
  if (trajectory.size() >= 2) {
    const std::size_t a = index + 1 < trajectory.size() ? index : index - 1;
    const double dx = trajectory[a + 1].x - trajectory[a].x;
    const double dy = trajectory[a + 1].y - trajectory[a].y;
    if (std::hypot(dx, dy) > 1.0e-9) {
      frame.tangent_yaw = std::atan2(dy, dx);
    } else if (frame.profile_speed > 1.0e-6) {
      frame.tangent_yaw = std::atan2(point.vy, point.vx);
    } else {
      frame.tangent_yaw = point.yaw;
    }
  } else {
    frame.tangent_yaw = point.yaw;
  }

  // 剩余弧长按折线累加，供到达判定使用。
  double remaining = 0.0;
  for (std::size_t i = index; i + 1 < trajectory.size(); ++i) {
    remaining += std::hypot(
      trajectory[i + 1].x - trajectory[i].x, trajectory[i + 1].y - trajectory[i].y);
  }
  frame.remaining_length = remaining;
  return frame;
}

}  // namespace

PreviewController::PreviewController(const PreviewControllerConfig & config)
: config_(config)
{
}

Twist2D PreviewController::compute(
  const Pose2D & pose, const Twist2D & world_velocity,
  const std::vector<TrajectoryPoint> & trajectory,
  double current_time_from_start,
  const RouteReferenceFrame * external_reference)
{
  if (trajectory.empty()) {
    last_cmd_ = {};
    last_debug_ = {};
    return last_cmd_;
  }

  const RouteReferenceFrame reference = external_reference
    ? *external_reference
    : reference_at_time(trajectory, current_time_from_start);

  const double tangent_x = std::cos(reference.tangent_yaw);
  const double tangent_y = std::sin(reference.tangent_yaw);
  // 位置误差【只】相对 s 处参考点，不含时间前瞻（上游 follow_problem residual(0)(1)）。
  const double ex = reference.position.x - pose.x;
  const double ey = reference.position.y - pose.y;
  const double position_error = std::hypot(ex, ey);
  // 分解到参考点的切/法向：lag 为切向(纵向)分量，contour 为法向(横向)分量。
  const double lag_error = tangent_x * ex + tangent_y * ey;
  const double contour_error = -tangent_y * ex + tangent_x * ey;

  // 世界系回正速度 = 前馈(沿切线的剖面速度) + 切/法向分别加权的位置回正。
  const double correction_x = config_.lag_kp * lag_error * tangent_x -
    config_.contour_kp * contour_error * tangent_y;
  const double correction_y = config_.lag_kp * lag_error * tangent_y +
    config_.contour_kp * contour_error * tangent_x;

  // 真实速度在切线上的投影，对应上游 projected_velocity = v·cos(θ−θ_path)。
  const double projected_speed = tangent_x * world_velocity.vx + tangent_y * world_velocity.vy;
  // 速度阻尼只惩罚【超出参考速度的部分】，沿切线作用；不足时不加速，交给前馈+位置项。
  const double speed_excess = std::max(0.0, projected_speed - reference.profile_speed);
  const double damping_x = config_.velocity_damping * speed_excess * tangent_x;
  const double damping_y = config_.velocity_damping * speed_excess * tangent_y;

  double world_vx = reference.profile_speed * tangent_x + correction_x - damping_x;
  double world_vy = reference.profile_speed * tangent_y + correction_y - damping_y;

  const double raw_heading_error = normalize_angle(reference.tangent_yaw - pose.yaw);
  double heading_control_weight = 0.0;
  double heading_error = 0.0;
  if (config_.path_heading_tracking_enabled &&
    reference.profile_speed >= config_.heading_control_min_reference_speed)
  {
    heading_control_weight = 1.0;
    if (config_.heading_startup_ramp_time > 1.0e-3) {
      heading_control_weight = std::clamp(
        current_time_from_start / config_.heading_startup_ramp_time, 0.0, 1.0);
    }

    const double abs_heading_error = std::abs(raw_heading_error);
    if (abs_heading_error > config_.heading_error_deadband) {
      heading_error =
        std::copysign(abs_heading_error - config_.heading_error_deadband, raw_heading_error);
    }
  }

  // 到达判定：终点直线距离与剩余弧长须同时满足（上游 path_executor.misc 的
  // stop_threshold_dist 与 stop_threshold_remaining_distance 双判据）。
  const auto & goal = trajectory.back();
  const double goal_distance = std::hypot(goal.x - pose.x, goal.y - pose.y);
  const bool reached_hold_target =
    goal_distance <= config_.arrival_position_tolerance &&
    reference.remaining_length <= config_.arrival_remaining_distance_tolerance;

  Twist2D cmd;
  if (!reached_hold_target) {
    // 线速度上限与规划器 max_velocity 同源，控制器不得比轨迹本身跑得更快。
    const double world_speed = std::hypot(world_vx, world_vy);
    if (world_speed > config_.max_linear_velocity) {
      const double scale = config_.max_linear_velocity / std::max(world_speed, 1.0e-6);
      world_vx *= scale;
      world_vy *= scale;
    }
    // 世界系 → 机体系。
    const double cy = std::cos(pose.yaw);
    const double sy = std::sin(pose.yaw);
    cmd.vx = cy * world_vx + sy * world_vy;
    cmd.vy = -sy * world_vx + cy * world_vy;

    cmd.wz = config_.heading_kp * heading_error;
    cmd.wz = std::clamp(cmd.wz, -config_.max_angular_velocity, config_.max_angular_velocity);
  }
  last_cmd_ = cmd;

  const double cy = std::cos(pose.yaw);
  const double sy = std::sin(pose.yaw);
  last_debug_.valid = true;
  last_debug_.current_time = current_time_from_start;
  last_debug_.reference_time = reference.reference_time;
  last_debug_.reference_arc_length = reference.arc_length;
  last_debug_.remaining_length = reference.remaining_length;
  last_debug_.position_error = position_error;
  last_debug_.contour_error = contour_error;
  last_debug_.lag_error = lag_error;
  last_debug_.goal_distance = goal_distance;
  last_debug_.raw_heading_error = raw_heading_error;
  last_debug_.heading_error = heading_error;
  last_debug_.heading_control_weight = heading_control_weight;
  last_debug_.reference_speed = reference.profile_speed;
  last_debug_.projected_speed = projected_speed;
  last_debug_.actual_body_speed = std::hypot(
    cy * world_velocity.vx + sy * world_velocity.vy,
    -sy * world_velocity.vx + cy * world_velocity.vy);
  last_debug_.command_linear_speed = std::hypot(cmd.vx, cmd.vy);
  last_debug_.command_angular_speed = std::abs(cmd.wz);
  last_debug_.stopped_by_arrival_tolerance = reached_hold_target;
  last_debug_.reference = reference;
  return cmd;
}

const PreviewControlDebug & PreviewController::last_debug() const
{
  return last_debug_;
}

}  // namespace astra_nav
