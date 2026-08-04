#pragma once

#include <vector>

#include "astra_common/common.hpp"

namespace astra_nav
{

struct PreviewControllerConfig
{
  double lookahead_time{0.18};
  double max_linear_velocity{1.8};
  double max_angular_velocity{2.5};
  double position_kp{1.4};
  double heading_kp{2.2};
  // 速度阻尼增益 kd：作用于【真实速度与参考速度之差】(v_actual-v_ref) 的机体系分量。
  // 巡航时 v_actual≈v_ref，阻尼≈0，不干扰前馈跟踪；接近终点 v_ref→0 时退化为
  // -kd*v_actual 的主动刹车，配合位置项 kp*e_p 构成 PD 控制，消除到点过冲/振荡（停不稳根因）。
  // 旧实现此项阻尼的是"控制器自己上一条命令"(last_cmd_)，无真实速度反馈，结构上无法刹车。
  double velocity_damping{0.6};
  double arrival_position_tolerance{0.18};
  double arrival_reference_speed_tolerance{0.05};
  bool path_heading_tracking_enabled{false};
  double heading_startup_ramp_time{0.8};
  double heading_error_deadband{0.03};
  double heading_control_min_reference_speed{0.05};
};

struct PreviewControlDebug
{
  bool valid{false};
  double current_time{0.0};
  double lookahead_time{0.0};
  double nearest_reference_time{0.0};
  double target_reference_time{0.0};
  double nearest_position_error{0.0};
  double target_position_error{0.0};
  double raw_target_heading_error{0.0};
  double target_heading_error{0.0};
  double heading_control_weight{0.0};
  double nearest_reference_speed{0.0};
  double target_reference_speed{0.0};
  double actual_body_speed{0.0};   // 真实机体速度幅值(速度阻尼项输入,观测用)
  double command_linear_speed{0.0};
  double command_angular_speed{0.0};
  bool stopped_by_arrival_tolerance{false};
  TrajectoryPoint nearest_reference;
  TrajectoryPoint target_reference;
};

class PreviewController
{
public:
  explicit PreviewController(const PreviewControllerConfig & config);
  // world_velocity：机器人在【世界系】的真实速度(vx,vy 世界系, wz)。控制器内部会旋到机体系，
  // 用于速度阻尼项，实现接近终点的主动刹车。调用方须传入里程计/状态估计得到的真实速度。
  Twist2D compute(
    const Pose2D & pose, const Twist2D & world_velocity,
    const std::vector<TrajectoryPoint> & trajectory,
    double current_time_from_start);
  const PreviewControlDebug & last_debug() const;

private:
  PreviewControllerConfig config_;
  Twist2D last_cmd_;
  PreviewControlDebug last_debug_;
};

}  // namespace astra_nav
