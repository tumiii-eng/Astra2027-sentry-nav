#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "astra_common/common.hpp"
#include "astra_control/preview_controller.hpp"
#include "astra_control/se2_mpc_preview.hpp"
#include "astra_control/se2_mpc_controller.hpp"
#include "astra_control/chassis_kinematics.hpp"
#include "astra_control/high_rate_state_estimator.hpp"
#include "astra_planning/minco_trajectory.hpp"

namespace
{

void test_controller()
{
  std::vector<astra_nav::Point2D> points{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  const auto times = astra_nav::cumulative_times(points, 1.0, 1.0);
  const auto trajectory = astra_nav::MincoTrajectory::fit(points, times).sample(0.05);
  astra_nav::PreviewController controller({});
  const auto cmd = controller.compute({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, trajectory, 0.0);
  const auto & debug = controller.last_debug();
  assert(cmd.vx > 0.0);
  assert(std::abs(cmd.vy) < 1e-6);
  assert(debug.valid);
  assert(debug.target_position_error > 0.0);
  assert(debug.nearest_reference_speed > 0.0);
  assert(debug.target_reference_speed > 0.0);
  assert(debug.command_linear_speed > 0.0);
  assert(std::abs(debug.raw_target_heading_error) < 1e-6);
  assert(std::abs(debug.target_heading_error) < 1e-6);
  assert(debug.heading_control_weight < 1e-6);

  const std::vector<astra_nav::TrajectoryPoint> diagonal_trajectory{
    {0.0, 0.0, 0.0, 1.2, 0.30, 0.0, 0.0, 0.0},
    {0.2, 0.2, 0.0, 1.2, 0.30, 0.0, 0.0, 0.0},
    {0.4, 0.4, 0.0, 1.2, 0.30, 0.0, 0.0, 0.0}};
  astra_nav::PreviewController omni_controller({});
  const auto omni_cmd = omni_controller.compute(
    {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, diagonal_trajectory, 0.2);
  const auto & omni_debug = omni_controller.last_debug();
  assert(std::abs(omni_debug.raw_target_heading_error) > 1.0);
  assert(std::abs(omni_debug.target_heading_error) < 1e-9);
  assert(std::abs(omni_cmd.wz) < 1e-9);

  astra_nav::PreviewControllerConfig heading_config;
  heading_config.path_heading_tracking_enabled = true;
  heading_config.heading_startup_ramp_time = 0.0;
  heading_config.heading_error_deadband = 0.0;
  astra_nav::PreviewController heading_controller(heading_config);
  const auto heading_cmd = heading_controller.compute(
    {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, diagonal_trajectory, 0.2);
  const auto & heading_debug = heading_controller.last_debug();
  assert(heading_debug.heading_control_weight > 0.99);
  assert(std::abs(heading_debug.target_heading_error) > 1.0);
  assert(heading_cmd.wz > 0.0);

  astra_nav::PreviewControllerConfig hold_config;
  hold_config.arrival_position_tolerance = 0.2;
  hold_config.arrival_reference_speed_tolerance = 0.05;
  astra_nav::PreviewController hold_controller(hold_config);
  const std::vector<astra_nav::TrajectoryPoint> hold_trajectory{
    {0.0, 0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.25, 0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.50, 0.15, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  const auto hold_cmd = hold_controller.compute(
    {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, hold_trajectory, 0.0);
  const auto & hold_debug = hold_controller.last_debug();
  assert(std::hypot(hold_cmd.vx, hold_cmd.vy) < 1e-9);
  assert(std::abs(hold_cmd.wz) < 1e-9);
  assert(hold_debug.stopped_by_arrival_tolerance);

  // —— 终点主动刹车（停不稳根因修复）——
  // 参考轨迹终点静止(v_ref=0)，机器人尚未进容差但正以高速冲向终点：
  // 有真实速度反馈时，速度阻尼项 -kd*v_actual 应压制甚至反转位置比例项，产生刹车（命令显著低于
  // 纯位置比例 kp*e_p），从而消除冲过终点的过冲。无真实速度反馈的旧实现只会 kp*e_p 继续加速。
  astra_nav::PreviewControllerConfig brake_config;
  brake_config.position_kp = 1.4;
  brake_config.velocity_damping = 0.6;
  brake_config.arrival_position_tolerance = 0.18;
  brake_config.arrival_reference_speed_tolerance = 0.05;
  const std::vector<astra_nav::TrajectoryPoint> brake_trajectory{
    {0.0, 0.7, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {0.5, 0.85, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
    {1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};  // 终点(1.0,0)静止
  // 机器人在 (0.7,0)，距终点 0.3m(>容差)，正以 1.5m/s 冲向 +x。
  astra_nav::PreviewController brake_controller(brake_config);
  const auto brake_cmd =
    brake_controller.compute({0.7, 0.0, 0.0}, {1.5, 0.0, 0.0}, brake_trajectory, 1.0);
  const double kp_only = brake_config.position_kp * 0.3;  // 纯位置比例项 = 0.42 m/s
  assert(brake_cmd.vx < kp_only);  // 刹车：命令被真实速度阻尼显著拉低
  // 对照：同一场景真实速度为零时不应刹车，命令回到纯位置比例项量级。
  astra_nav::PreviewController brake_ctrl_zero(brake_config);
  const auto no_brake_cmd =
    brake_ctrl_zero.compute({0.7, 0.0, 0.0}, {0.0, 0.0, 0.0}, brake_trajectory, 1.0);
  assert(no_brake_cmd.vx > brake_cmd.vx + 0.5);  // 有速度时明显更小（差约 kd*1.5=0.9）

  astra_nav::Se2MpcPreviewConfig preview_config;
  preview_config.steps = 8;
  preview_config.dt = 0.05;
  astra_nav::Se2MpcPreview preview(preview_config);
  const auto prediction = preview.predict({0.0, 0.0, 0.0}, cmd, trajectory, 0.0);
  assert(prediction.valid);
  assert(prediction.steps == preview_config.steps);
  assert(prediction.horizon_duration > 0.0);
  assert(std::isfinite(prediction.mean_position_error));
  assert(std::isfinite(prediction.max_position_error));
  assert(std::isfinite(prediction.final_position_error));
}

// 构造一条世界系直线参考轨迹（沿 +x，匀速 v），航向由速度切线决定。
std::vector<astra_nav::TrajectoryPoint> make_straight_world_trajectory(double v, double dt, int n)
{
  std::vector<astra_nav::TrajectoryPoint> traj;
  traj.reserve(n);
  for (int i = 0; i < n; ++i) {
    astra_nav::TrajectoryPoint p;
    p.t = i * dt;
    p.x = v * p.t;
    p.y = 0.0;
    p.yaw = 0.0;
    p.vx = v;
    p.vy = 0.0;
    traj.push_back(p);
  }
  return traj;
}

void test_se2_mpc_controller()
{
  // —— 1) 直线轨迹：机体系航向对齐，应前进且无横向、无旋转 ——
  astra_nav::Se2MpcConfig config;
  config.horizon = 20;
  config.dt = 0.05;
  const auto straight = make_straight_world_trajectory(1.0, 0.05, 40);
  astra_nav::Se2MpcController mpc(config);
  const auto cmd = mpc.compute({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, straight, 0.0);
  const auto & dbg = mpc.last_debug();
  assert(dbg.valid);
  assert(dbg.first_qp_success);       // 首次 QP 必须求解成功
  assert(cmd.vx > 0.0);               // 沿轨迹前进
  assert(std::abs(cmd.vy) < 1e-3);    // 无明显横向
  assert(std::abs(cmd.wz) < 1e-3);    // 航向已对齐，无明显旋转
  assert(std::isfinite(cmd.vx) && std::isfinite(cmd.vy) && std::isfinite(cmd.wz));

  // —— 2) 世界系→机体系旋转：机器人朝向 +y（yaw=π/2），轨迹沿世界 +x ——
  // 期望机体命令主要为横向负方向（v_body = R(yaw)^T v_world）。
  astra_nav::Se2MpcController mpc_rot(config);
  const double half_pi = M_PI / 2.0;
  const auto cmd_rot = mpc_rot.compute({0.0, 0.0, half_pi}, {0.0, 0.0, 0.0}, straight, 0.0);
  const auto & dbg_rot = mpc_rot.last_debug();
  assert(dbg_rot.valid);
  assert(cmd_rot.vy < 0.0);                       // 世界 +x 在机体系为 -y
  assert(std::abs(cmd_rot.vx) < std::abs(cmd_rot.vy)); // 机体系以横向为主
  assert(cmd_rot.wz < 0.0);                       // 航向从 π/2 向 0 收敛，角速度为负

  // —— 3) 到达保持：投影到终点、终点零速、距离在容差内，应输出零速 ——
  astra_nav::Se2MpcConfig hold_config = config;
  hold_config.arrival_position_tolerance = 0.2;
  hold_config.arrival_speed_tolerance = 0.05;
  astra_nav::Se2MpcController mpc_hold(hold_config);
  const std::vector<astra_nav::TrajectoryPoint> arrival_traj{
    {0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0},
    {0.5, 0.5, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0},
    {1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  const auto hold_cmd = mpc_hold.compute({1.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, arrival_traj, 1.0);
  const auto & hold_dbg = mpc_hold.last_debug();
  assert(hold_dbg.stopped_by_arrival);
  assert(std::hypot(hold_cmd.vx, hold_cmd.vy) < 1e-9);
  assert(std::abs(hold_cmd.wz) < 1e-9);

  // —— 4) 二次轨迹开关（报告 5.5.5.4.3）——
  // 开启时若二次 QP 成功则采用二次结果，且方向余弦指标被记录（≤ 1）。
  assert(dbg.secondary_min_cos <= 1.0 + 1e-9);
  astra_nav::Se2MpcConfig no_second = config;
  no_second.secondary_pass_enabled = false;
  astra_nav::Se2MpcController mpc_no_second(no_second);
  const auto cmd_ns = mpc_no_second.compute({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, straight, 0.0);
  (void)cmd_ns;
  assert(!mpc_no_second.last_debug().used_secondary);  // 关闭后不得使用二次轨迹

  // —— 5) 轨迹过短：少于 2 点应安全返回零速、无效 ——
  astra_nav::Se2MpcController mpc_empty(config);
  const std::vector<astra_nav::TrajectoryPoint> too_short{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  const auto cmd_empty = mpc_empty.compute({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, too_short, 0.0);
  assert(!mpc_empty.last_debug().valid);
  assert(std::hypot(cmd_empty.vx, cmd_empty.vy) < 1e-12);
}

// 底盘正/逆运动学往返一致：体速度 → 轮速 → 体速度 应还原（报告 5.5.5.4.1 轮速换算层）。
void test_chassis_kinematics()
{
  const astra_nav::Twist2D probes[] = {
    {1.0, 0.0, 0.0},   // 纯前进
    {0.0, 1.0, 0.0},   // 纯左移
    {0.0, 0.0, 1.0},   // 纯自转
    {0.6, -0.4, 0.3},  // 复合运动
  };

  // 麦轮。
  astra_nav::ChassisKinematicsConfig mec_cfg;
  mec_cfg.type = astra_nav::ChassisType::Mecanum;
  astra_nav::ChassisKinematics mecanum(mec_cfg);
  for (const auto & v : probes) {
    const auto wheels = mecanum.inverse(v);
    const auto back = mecanum.forward(wheels);
    assert(std::abs(back.vx - v.vx) < 1e-9);
    assert(std::abs(back.vy - v.vy) < 1e-9);
    assert(std::abs(back.wz - v.wz) < 1e-9);
  }
  // 麦轮纯前进时四轮同号同速。
  {
    const auto w = mecanum.inverse({1.0, 0.0, 0.0});
    assert(w.fl > 0.0 && std::abs(w.fl - w.fr) < 1e-9 &&
      std::abs(w.fl - w.rl) < 1e-9 && std::abs(w.fl - w.rr) < 1e-9);
  }

  // 全向轮。
  astra_nav::ChassisKinematicsConfig omni_cfg;
  omni_cfg.type = astra_nav::ChassisType::Omni;
  astra_nav::ChassisKinematics omni(omni_cfg);
  for (const auto & v : probes) {
    const auto wheels = omni.inverse(v);
    const auto back = omni.forward(wheels);
    assert(std::abs(back.vx - v.vx) < 1e-9);
    assert(std::abs(back.vy - v.vy) < 1e-9);
    assert(std::abs(back.wz - v.wz) < 1e-9);
  }
  // 全向轮纯自转时四轮同号同速。
  {
    const auto w = omni.inverse({0.0, 0.0, 1.0});
    assert(w.fl > 0.0 && std::abs(w.fl - w.fr) < 1e-9 &&
      std::abs(w.fl - w.rl) < 1e-9 && std::abs(w.fl - w.rr) < 1e-9);
  }

  // 逐轮极性修正应可逆（sign²=1 不改变往返一致性）。
  astra_nav::ChassisKinematicsConfig sign_cfg = mec_cfg;
  sign_cfg.wheel_direction_sign = {{1.0, -1.0, 1.0, -1.0}};
  astra_nav::ChassisKinematics signed_chassis(sign_cfg);
  const astra_nav::Twist2D v{0.5, 0.2, -0.3};
  const auto back = signed_chassis.forward(signed_chassis.inverse(v));
  assert(std::abs(back.vx - v.vx) < 1e-9);
  assert(std::abs(back.vy - v.vy) < 1e-9);
  assert(std::abs(back.wz - v.wz) < 1e-9);
}

void test_high_rate_estimator()
{
  using astra_nav::HighRateStateEstimator;
  using astra_nav::HighRateStateEstimatorConfig;
  using astra_nav::MotionSample;

  // —— 1) 无锚点：返回未外推 ——
  {
    HighRateStateEstimator est{HighRateStateEstimatorConfig{}};
    const auto s = est.estimate(1.0);
    assert(!est.has_anchor());
    assert(!s.extrapolated);
  }

  // —— 2) 体速度积分位置：锚点已以 1m/s 稳态前进，0.1s 后应外推到 x=0.1 ——
  // 锚点速度与样本一致（稳态），ZOH 积分严格闭合；也不会触发打滑闸门（加速度=0）。
  {
    HighRateStateEstimatorConfig cfg;
    cfg.max_extrapolation_time = 0.5;
    HighRateStateEstimator est(cfg);
    est.set_anchor({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.0);
    for (int i = 1; i <= 10; ++i) {
      MotionSample m;
      m.stamp = i * 0.01;
      m.body_vx = 1.0;
      m.has_body_velocity = true;
      est.add_motion_sample(m);
    }
    const auto s = est.estimate(0.1);
    assert(s.extrapolated);
    assert(std::abs(s.pose.x - 0.1) < 1e-3);
    assert(std::abs(s.pose.y) < 1e-6);
    assert(std::abs(s.world_velocity.vx - 1.0) < 1e-9);
  }

  // —— 3) IMU 航向积分 + 体速度旋到世界系：稳态 1m/s 体速度 + 1rad/s 航向角速度 ——
  {
    HighRateStateEstimatorConfig cfg;
    cfg.max_extrapolation_time = 0.5;
    HighRateStateEstimator est(cfg);
    est.set_anchor({0.0, 0.0, 0.0}, {1.0, 0.0, 1.0}, 0.0);
    for (int i = 1; i <= 10; ++i) {
      MotionSample m;
      m.stamp = i * 0.01;
      m.yaw_rate = 1.0;
      m.has_yaw_rate = true;
      m.body_vx = 1.0;
      m.has_body_velocity = true;
      est.add_motion_sample(m);
    }
    const auto s = est.estimate(0.1);
    assert(s.extrapolated);
    assert(std::abs(s.pose.yaw - 0.1) < 1e-2);  // 约 0.1 rad
    assert(s.pose.y > 0.0);                      // 航向左偏，世界系产生 +y 分量
    assert(s.world_velocity.wz > 0.0);
  }

  // —— 4) 锚点过旧：超出外推上限，直接返回锚点位姿 ——
  {
    HighRateStateEstimatorConfig cfg;
    cfg.max_extrapolation_time = 0.05;
    HighRateStateEstimator est(cfg);
    est.set_anchor({1.0, 2.0, 0.5}, {0.0, 0.0, 0.0}, 0.0);
    MotionSample m;
    m.stamp = 0.1;
    m.body_vx = 5.0;
    m.has_body_velocity = true;
    est.add_motion_sample(m);
    const auto s = est.estimate(0.2);  // age=0.2 > 0.05
    assert(!s.extrapolated);
    assert(std::abs(s.pose.x - 1.0) < 1e-9 && std::abs(s.pose.y - 2.0) < 1e-9);
  }

  // —— 5) 打滑闸门：突变的高加速度体速度样本被拒绝，沿用上一速度 ——
  {
    HighRateStateEstimatorConfig cfg;
    cfg.max_extrapolation_time = 0.5;
    cfg.enable_slip_gate = true;
    cfg.max_body_acceleration = 5.0;  // 阈值低，便于触发
    HighRateStateEstimator est(cfg);
    est.set_anchor({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 0.0);  // 锚点体速度 1m/s（朝+x）
    MotionSample good;
    good.stamp = 0.01;
    good.body_vx = 1.0;
    good.has_body_velocity = true;
    est.add_motion_sample(good);
    MotionSample slip;  // 1ms 内从 1 跳到 100 m/s，隐含加速度极大
    slip.stamp = 0.011;
    slip.body_vx = 100.0;
    slip.has_body_velocity = true;
    est.add_motion_sample(slip);
    const auto s = est.estimate(0.02);
    assert(s.extrapolated);
    assert(s.rejected_slip_samples >= 1);
    // 打滑样本被拒，位置增量应接近 1m/s 而非 100m/s 的量级。
    assert(s.pose.x < 0.1);
  }

  // —— 6) use_body_velocity 关闭：退化为锚点世界速度匀速外推（打滑免疫兜底）——
  {
    HighRateStateEstimatorConfig cfg;
    cfg.max_extrapolation_time = 0.5;
    cfg.use_body_velocity = false;
    HighRateStateEstimator est(cfg);
    est.set_anchor({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, 0.0);
    MotionSample m;  // 即使体速度样本荒谬，关闭后也不采用
    m.stamp = 0.05;
    m.body_vx = 999.0;
    m.has_body_velocity = true;
    est.add_motion_sample(m);
    const auto s = est.estimate(0.1);
    assert(s.extrapolated);
    assert(std::abs(s.pose.x - 0.2) < 1e-6);  // 2m/s * 0.1s
    assert(std::abs(s.world_velocity.vx - 2.0) < 1e-9);
  }
}

}  // namespace

int main()
{
  test_controller();
  test_se2_mpc_controller();
  test_chassis_kinematics();
  test_high_rate_estimator();
  std::cout << "astra_control 测试通过。" << std::endl;
  return 0;
}
