#pragma once

#include <vector>

#include "astra_common/common.hpp"

namespace astra_nav
{

// 有向路径参考点，逐项对应上游 HWSentryNav26 follow_problem.cpp 的 ReferenceFrame：
//   reference.sample.p        → position
//   reference.tangent         → (cos tangent_yaw, sin tangent_yaw)
//   reference.nominal_path_speed → profile_speed
//   reference.path_progress   → arc_length
// 上游的参考点【只由弧长进度 s 决定】，不含任何时间前瞻；前瞻是 MPC 预测时域的职责。
struct RouteReferenceFrame
{
  Point2D position{};
  double tangent_yaw{0.0};
  double profile_speed{0.0};
  double arc_length{0.0};
  double remaining_length{0.0};
  // Astra 侧轨迹是时间参数化采样，保留 s 对应的时间参数仅用于日志与诊断。
  double reference_time{0.0};
};

struct PreviewControllerConfig
{
  // 线速度上限须与规划器 max_velocity 同源：上游跟随器的 ṡ 上界与速度剖面优化用的是
  // 同一个 capability.command_envelope.velocity.max，控制器不允许比轨迹本身跑得更快。
  double max_linear_velocity{1.5};
  double max_angular_velocity{2.5};
  // 位置误差在参考点切/法向上分解后的回正增益，对应上游 follow.tracking_weights：
  // contour = 法向(横向)误差，lag = 切向(纵向)误差。上游默认二者等权 1.6。
  double contour_kp{1.4};
  double lag_kp{1.4};
  double heading_kp{2.2};
  // 速度阻尼增益 kd：作用于【真实速度与参考点速度之差】。对应上游 residual(3)
  // (projected_velocity − path_speed_cmd)：真实速度超出参考速度时被拉回。
  // 参考速度取自 s 处速度剖面，终点处 v_ref→0，退化为 -kd*v_actual 的主动刹车。
  double velocity_damping{0.6};
  // 到达判定（对应上游 path_executor.misc）：终点距离与剩余弧长须【同时】满足。
  // 上游不用参考速度做到达判据，因为速度剖面在急弯处也会低速，会误判为到达。
  double arrival_position_tolerance{0.18};
  double arrival_remaining_distance_tolerance{0.18};
  bool path_heading_tracking_enabled{false};
  double heading_startup_ramp_time{0.8};
  double heading_error_deadband{0.03};
  double heading_control_min_reference_speed{0.05};
};

struct PreviewControlDebug
{
  bool valid{false};
  double current_time{0.0};
  double reference_time{0.0};
  double reference_arc_length{0.0};
  double remaining_length{0.0};
  double position_error{0.0};   // 与 s 处参考点的距离
  double contour_error{0.0};    // 法向误差（上游 residual(0)）
  double lag_error{0.0};        // 切向误差（上游 residual(1)）
  double goal_distance{0.0};    // 到轨迹终点的直线距离（到达判定用）
  double raw_heading_error{0.0};
  double heading_error{0.0};
  double heading_control_weight{0.0};
  double reference_speed{0.0};  // s 处速度剖面速度（上游 nominal_path_speed）
  double projected_speed{0.0};  // 真实速度在切线上的投影（上游 projected_velocity）
  double actual_body_speed{0.0};
  double command_linear_speed{0.0};
  double command_angular_speed{0.0};
  bool stopped_by_arrival_tolerance{false};
  RouteReferenceFrame reference{};
};

class PreviewController
{
public:
  explicit PreviewController(const PreviewControllerConfig & config);
  // world_velocity：机器人在【世界系】的真实速度(vx,vy 世界系, wz)。
  // reference：由 RouteTracker 的弧长进度求出的有向参考点（上游 ReferenceFrame）。
  //   传 nullptr 时退化为按 current_time_from_start 在采样折线上取最近点自建参考点，
  //   仍不做任何时间前瞻。
  Twist2D compute(
    const Pose2D & pose, const Twist2D & world_velocity,
    const std::vector<TrajectoryPoint> & trajectory,
    double current_time_from_start,
    const RouteReferenceFrame * reference = nullptr);
  const PreviewControlDebug & last_debug() const;
  const PreviewControllerConfig & config() const { return config_; }

private:
  PreviewControllerConfig config_;
  Twist2D last_cmd_;
  PreviewControlDebug last_debug_;
};

}  // namespace astra_nav
