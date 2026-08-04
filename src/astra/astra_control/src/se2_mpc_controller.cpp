#include "astra_control/se2_mpc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <qpOASES.hpp>

namespace astra_nav
{

namespace
{
// 速度方向有效阈值：参考速度模长低于此值时认为方向未定义（接近停车）。
constexpr double kSpeedEps = 1.0e-4;
}  // namespace

Se2MpcController::Se2MpcController(const Se2MpcConfig & config)
: config_(config)
{
  n_ = std::max(1, config_.horizon);
  nv_ = 3 * n_;
  build_constant_matrices();
}

void Se2MpcController::build_constant_matrices()
{
  const double dt = config_.dt;
  const double m = std::max(1.0e-6, config_.mass);
  const double Iz = std::max(1.0e-6, config_.inertia);

  // —— 连续双积分模型经 ZOH 精确离散（A_c 幂零，A_c^2=0）——
  // 状态 [px,py,θ,vx,vy,ω]，控制 [Fx,Fy,Mz]，世界系下平移与旋转解耦。
  Ad_.setIdentity();
  Ad_(0, 3) = dt;
  Ad_(1, 4) = dt;
  Ad_(2, 5) = dt;

  Bd_.setZero();
  Bd_(0, 0) = 0.5 * dt * dt / m;
  Bd_(1, 1) = 0.5 * dt * dt / m;
  Bd_(2, 2) = 0.5 * dt * dt / Iz;
  Bd_(3, 0) = dt / m;
  Bd_(4, 1) = dt / m;
  Bd_(5, 2) = dt / Iz;

  // —— 预测矩阵 X = Sx·x0 + Su·U （报告 5.5.5.3.1）——
  // X = [x_1; x_2; ...; x_N]（6N），U = [u_0; ...; u_{N-1}]（3N）。
  Sx_ = Eigen::MatrixXd::Zero(6 * n_, 6);
  Su_ = Eigen::MatrixXd::Zero(6 * n_, nv_);
  Eigen::Matrix<double, 6, 6> Apow = Ad_;  // A^(i+1)
  for (int i = 0; i < n_; ++i) {
    Sx_.block<6, 6>(6 * i, 0) = Apow;
    // Su 第 i 行块：x_{i+1} = sum_{j=0..i} A^{i-j} B u_j
    Eigen::Matrix<double, 6, 6> Ak = Eigen::Matrix<double, 6, 6>::Identity();
    for (int j = i; j >= 0; --j) {
      Su_.block<6, 3>(6 * i, 3 * j) = Ak * Bd_;
      Ak = Ak * Ad_;
    }
    Apow = Apow * Ad_;
  }

  // —— 误差权重 Q̄（6N 对角）与控制权重 R̄（3N 对角）——
  Qbar_ = Eigen::MatrixXd::Zero(6 * n_, 6 * n_);
  Eigen::VectorXd rdiag = Eigen::VectorXd::Zero(nv_);
  for (int i = 0; i < n_; ++i) {
    Qbar_(6 * i + 0, 6 * i + 0) = config_.weight_position;
    Qbar_(6 * i + 1, 6 * i + 1) = config_.weight_position;
    Qbar_(6 * i + 2, 6 * i + 2) = config_.weight_yaw;
    Qbar_(6 * i + 3, 6 * i + 3) = config_.weight_velocity;
    Qbar_(6 * i + 4, 6 * i + 4) = config_.weight_velocity;
    Qbar_(6 * i + 5, 6 * i + 5) = config_.weight_omega;
    rdiag(3 * i + 0) = config_.weight_force;
    rdiag(3 * i + 1) = config_.weight_force;
    rdiag(3 * i + 2) = config_.weight_torque;
  }

  // —— 常量 Hessian：H = 2(Suᵀ Q̄ Su + R̄)（报告 5.5.5.3.2）——
  H_ = 2.0 * (Su_.transpose() * Qbar_ * Su_);
  H_ += 2.0 * Eigen::MatrixXd(rdiag.asDiagonal());
  // 数值对称化，保证 qpOASES 接受。
  H_ = 0.5 * (H_ + H_.transpose());

  // —— 力/力矩盒约束（QP 变量上下界 lb/ub）——
  lb_ = Eigen::VectorXd::Zero(nv_);
  ub_ = Eigen::VectorXd::Zero(nv_);
  for (int i = 0; i < n_; ++i) {
    lb_(3 * i + 0) = -config_.max_force;
    ub_(3 * i + 0) = config_.max_force;
    lb_(3 * i + 1) = -config_.max_force;
    ub_(3 * i + 1) = config_.max_force;
    lb_(3 * i + 2) = -config_.max_torque;
    ub_(3 * i + 2) = config_.max_torque;
  }

  // —— 通用线性不等式约束 Acon·U （报告 5.5.5.3.3）——
  // 第一类：摩擦圆多边形约束（常量右端 μ m g），作用于每步控制力 (Fx,Fy)。
  //   对全向轮各向同性摩擦圆 |F| ≤ μmg，用内接半平面族线性化：
  //   cos(φ_k)·Fx + sin(φ_k)·Fy ≤ μmg，k=0..sides-1。
  // 第二类：速度盒约束 |vx|,|vy| ≤ vmax、|ω| ≤ ωmax，作用于预测状态速度分量；
  //   速度来自 x_{i+1} = Sx·x0 + Su·U，故右端含 x0 偏移，每周期更新。
  const int sides = std::max(3, config_.friction_polygon_sides);
  friction_rows_ = n_ * sides;
  velocity_rows_ = n_ * 3;  // 每步 vx, vy, ω
  nc_ = friction_rows_ + velocity_rows_;

  Acon_ = Eigen::MatrixXd::Zero(nc_, nv_);
  con_const_upper_ = Eigen::VectorXd::Zero(nc_);
  con_const_lower_ = Eigen::VectorXd::Zero(nc_);
  vel_select_Su_ = Eigen::MatrixXd::Zero(velocity_rows_, nv_);
  vel_select_Sx_ = Eigen::MatrixXd::Zero(velocity_rows_, 6);

  const double friction_limit =
    config_.friction_coefficient * config_.mass * config_.gravity;
  // 摩擦行。
  for (int i = 0; i < n_; ++i) {
    for (int k = 0; k < sides; ++k) {
      const int row = i * sides + k;
      const double phi = 2.0 * M_PI * static_cast<double>(k) / static_cast<double>(sides);
      Acon_(row, 3 * i + 0) = std::cos(phi);  // Fx 系数
      Acon_(row, 3 * i + 1) = std::sin(phi);  // Fy 系数
      con_const_upper_(row) = friction_limit;
      con_const_lower_(row) = -std::numeric_limits<double>::infinity();
    }
  }
  // 速度行：取预测状态 x_{i+1} 的 vx(行3)、vy(行4)、ω(行5)。
  for (int i = 0; i < n_; ++i) {
    const int base = friction_rows_ + i * 3;
    const int sx_vx = 6 * i + 3;
    const int sx_vy = 6 * i + 4;
    const int sx_w = 6 * i + 5;
    Acon_.row(base + 0) = Su_.row(sx_vx);
    Acon_.row(base + 1) = Su_.row(sx_vy);
    Acon_.row(base + 2) = Su_.row(sx_w);
    vel_select_Su_.row(i * 3 + 0) = Su_.row(sx_vx);
    vel_select_Su_.row(i * 3 + 1) = Su_.row(sx_vy);
    vel_select_Su_.row(i * 3 + 2) = Su_.row(sx_w);
    vel_select_Sx_.row(i * 3 + 0) = Sx_.row(sx_vx);
    vel_select_Sx_.row(i * 3 + 1) = Sx_.row(sx_vy);
    vel_select_Sx_.row(i * 3 + 2) = Sx_.row(sx_w);
    con_const_upper_(base + 0) = config_.max_velocity;
    con_const_lower_(base + 0) = -config_.max_velocity;
    con_const_upper_(base + 1) = config_.max_velocity;
    con_const_lower_(base + 1) = -config_.max_velocity;
    con_const_upper_(base + 2) = config_.max_omega;
    con_const_lower_(base + 2) = -config_.max_omega;
  }
}

double Se2MpcController::project_onto_trajectory(
  const std::vector<TrajectoryPoint> & trajectory, const Pose2D & pose,
  double time_hint) const
{
  if (trajectory.empty()) {
    return 0.0;
  }
  // 粗采样找最近点，再在邻域做一次细化（报告 5.5.5.4.2：投影自身位置作为轨迹起点）。
  double best_t = trajectory.front().t;
  double best_d2 = std::numeric_limits<double>::max();
  for (const auto & p : trajectory) {
    const double dx = p.x - pose.x;
    const double dy = p.y - pose.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best_t = p.t;
    }
  }
  (void)time_hint;
  return best_t;
}

TrajectoryPoint Se2MpcController::sample_reference(
  const std::vector<TrajectoryPoint> & trajectory, double t) const
{
  TrajectoryPoint out;
  if (trajectory.empty()) {
    return out;
  }
  if (t <= trajectory.front().t) {
    return trajectory.front();
  }
  if (t >= trajectory.back().t) {
    return trajectory.back();
  }
  // 线性插值到时刻 t。
  std::size_t hi = 1;
  while (hi < trajectory.size() && trajectory[hi].t < t) {
    ++hi;
  }
  const auto & a = trajectory[hi - 1];
  const auto & b = trajectory[hi];
  const double span = std::max(1.0e-9, b.t - a.t);
  const double s = std::clamp((t - a.t) / span, 0.0, 1.0);
  out.t = t;
  out.x = a.x + s * (b.x - a.x);
  out.y = a.y + s * (b.y - a.y);
  out.vx = a.vx + s * (b.vx - a.vx);
  out.vy = a.vy + s * (b.vy - a.vy);
  out.ax = a.ax + s * (b.ax - a.ax);
  out.ay = a.ay + s * (b.ay - a.ay);
  // 航向取速度切线方向（报告 5.5.5.4.2：航向角由速度切线方向得到）。
  const double speed = std::hypot(out.vx, out.vy);
  out.yaw = speed > kSpeedEps ? std::atan2(out.vy, out.vx) : a.yaw;
  return out;
}

Eigen::VectorXd Se2MpcController::build_reference_vector(
  const std::vector<TrajectoryPoint> & trajectory,
  const std::vector<double> & sample_times, double yaw_anchor) const
{
  // 参考状态序列 Xref = [x_ref_1; ...; x_ref_N]，对应预测状态 x_1..x_N。
  Eigen::VectorXd xref = Eigen::VectorXd::Zero(6 * n_);
  double prev_yaw = yaw_anchor;  // 解缠基准：从当前航向开始累积
  for (int i = 0; i < n_ && i < static_cast<int>(sample_times.size()); ++i) {
    const auto ref = sample_reference(trajectory, sample_times[i]);
    // 把参考航向解缠到上一参考航向附近，避免 ±π 跳变造成虚假大误差。
    const double yaw_unwrapped = prev_yaw + normalize_angle(ref.yaw - prev_yaw);
    prev_yaw = yaw_unwrapped;
    xref(6 * i + 0) = ref.x;
    xref(6 * i + 1) = ref.y;
    xref(6 * i + 2) = yaw_unwrapped;
    xref(6 * i + 3) = ref.vx;
    xref(6 * i + 4) = ref.vy;
    // 角速度参考：由相邻参考航向差分估计（首步用 0）。
    xref(6 * i + 5) = 0.0;
  }
  return xref;
}

bool Se2MpcController::solve_qp(
  const Eigen::Matrix<double, 6, 1> & x0, const Eigen::VectorXd & xref,
  Eigen::VectorXd & u_out, double & cost_out) const
{
  using namespace qpOASES;

  // 梯度向量 g = 2 Suᵀ Q̄ (Sx x0 - Xref)（报告 5.5.5.3.2 展开整理）。
  const Eigen::VectorXd offset = Sx_ * x0 - xref;
  const Eigen::VectorXd g = 2.0 * (Su_.transpose() * (Qbar_ * offset));

  // 速度约束右端随 x0 变化：lb_v ≤ vel_Su·U + vel_Sx·x0 ≤ ub_v
  // 即 (lb_v - vel_Sx·x0) ≤ vel_Su·U ≤ (ub_v - vel_Sx·x0)。
  const Eigen::VectorXd vel_off = vel_select_Sx_ * x0;
  Eigen::VectorXd con_lower = con_const_lower_;
  Eigen::VectorXd con_upper = con_const_upper_;
  for (int r = 0; r < velocity_rows_; ++r) {
    const int row = friction_rows_ + r;
    con_lower(row) = con_const_lower_(row) - vel_off(r);
    con_upper(row) = con_const_upper_(row) - vel_off(r);
  }

  // qpOASES 使用行主序 real_t 数组；Eigen 默认列主序，需转置后取数据。
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H_row = H_;
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_row = Acon_;

  QProblem qp(nv_, nc_, HST_SEMIDEF);
  Options options;
  options.setToMPC();
  options.printLevel = PL_NONE;
  qp.setOptions(options);

  int nWSR = config_.max_working_set_recalculations;
  const returnValue ret = qp.init(
    H_row.data(), g.data(), A_row.data(),
    lb_.data(), ub_.data(), con_lower.data(), con_upper.data(), nWSR);

  if (ret != SUCCESSFUL_RETURN && ret != RET_MAX_NWSR_REACHED) {
    return false;
  }

  Eigen::VectorXd sol(nv_);
  if (qp.getPrimalSolution(sol.data()) != SUCCESSFUL_RETURN) {
    return false;
  }
  u_out = sol;
  cost_out = qp.getObjVal();
  // qpOASES 在不可解时可能返回 inf/NaN 目标值。
  if (!std::isfinite(cost_out)) {
    return false;
  }
  return true;
}

Twist2D Se2MpcController::compute(
  const Pose2D & pose, const Twist2D & world_velocity,
  const std::vector<TrajectoryPoint> & trajectory, double current_time_from_start)
{
  debug_ = Se2MpcDebug{};
  Twist2D zero_cmd;
  if (trajectory.size() < 2) {
    return zero_cmd;
  }

  // —— 1) 定位投影：当前位置在轨迹上的最近投影点，得到起始时刻 t0 ——
  const double t0 = project_onto_trajectory(trajectory, pose, current_time_from_start);
  debug_.projection_time = t0;

  // 当前状态 x0（世界系）。
  Eigen::Matrix<double, 6, 1> x0;
  x0 << pose.x, pose.y, pose.yaw, world_velocity.vx, world_velocity.vy, world_velocity.wz;

  // 到达判定：投影点接近终点且参考速度很小，则进入到达保持，输出零速。
  const auto nearest_ref = sample_reference(trajectory, t0);
  debug_.nearest_position_error = std::hypot(nearest_ref.x - pose.x, nearest_ref.y - pose.y);
  const double end_t = trajectory.back().t;
  const auto end_ref = trajectory.back();
  const double dist_to_end = std::hypot(end_ref.x - pose.x, end_ref.y - pose.y);
  const double end_ref_speed = std::hypot(end_ref.vx, end_ref.vy);
  if (t0 >= end_t - 1.0e-6 &&
    dist_to_end < config_.arrival_position_tolerance &&
    end_ref_speed < config_.arrival_speed_tolerance)
  {
    debug_.valid = true;
    debug_.stopped_by_arrival = true;
    debug_.reference_at_lookahead = end_ref;
    return zero_cmd;
  }

  // —— 2) 首次期望轨迹：以 t0 起、dt 为步长向后取 N 点（报告 5.5.5.4.2）——
  std::vector<double> first_times(n_);
  for (int i = 0; i < n_; ++i) {
    // 预测状态 x_{i+1} 对应时刻 t0 + (i+1)*dt。
    first_times[i] = t0 + static_cast<double>(i + 1) * config_.dt;
  }
  debug_.horizon_duration = static_cast<double>(n_) * config_.dt;

  const Eigen::VectorXd xref_first = build_reference_vector(trajectory, first_times, pose.yaw);

  Eigen::VectorXd u_first;
  double cost_first = 0.0;
  debug_.first_qp_success = solve_qp(x0, xref_first, u_first, cost_first);
  debug_.first_qp_cost = cost_first;
  if (!debug_.first_qp_success) {
    return zero_cmd;  // 首次 QP 失败，交由上层回退
  }

  // —— 3) 二次期望轨迹（报告 5.5.5.4.3）——
  // 根据首次规划速度与对应时刻参考速度的法向夹角余弦缩小时间步长，
  // 在横向误差较大（夹角大、余弦小）时主动“缩减”参考轨迹，牺牲纵向速度保横向精度。
  Eigen::VectorXd u_chosen = u_first;
  double chosen_cost = cost_first;
  double min_cos = 1.0;

  if (config_.secondary_pass_enabled) {
    // 重建首次规划预测状态序列 X_first = Sx x0 + Su u_first。
    const Eigen::VectorXd x_first = Sx_ * x0 + Su_ * u_first;
    std::vector<double> second_times(n_);
    double accum_t = t0;
    for (int i = 0; i < n_; ++i) {
      const double vpx = x_first(6 * i + 3);
      const double vpy = x_first(6 * i + 4);
      const double vrx = xref_first(6 * i + 3);
      const double vry = xref_first(6 * i + 4);
      const double plan_speed = std::hypot(vpx, vpy);
      const double ref_speed = std::hypot(vrx, vry);
      double cos_angle = 1.0;
      if (plan_speed > kSpeedEps && ref_speed > kSpeedEps) {
        cos_angle = (vpx * vrx + vpy * vry) / (plan_speed * ref_speed);
        cos_angle = std::clamp(cos_angle, -1.0, 1.0);
      }
      min_cos = std::min(min_cos, cos_angle);
      // 余弦越小（夹角越大）缩放越强；下限 secondary_min_scale 防止时间步过小。
      const double scale = std::clamp(cos_angle, config_.secondary_min_scale, 1.0);
      accum_t += config_.dt * scale;
      second_times[i] = accum_t;
    }
    debug_.secondary_min_cos = min_cos;

    const Eigen::VectorXd xref_second =
      build_reference_vector(trajectory, second_times, pose.yaw);
    Eigen::VectorXd u_second;
    double cost_second = 0.0;
    debug_.second_qp_success = solve_qp(x0, xref_second, u_second, cost_second);
    debug_.second_qp_cost = cost_second;
    if (debug_.second_qp_success) {
      u_chosen = u_second;
      chosen_cost = cost_second;
      debug_.used_secondary = true;
    }
  }
  (void)chosen_cost;

  // —— 4) 控制命令输出（报告 5.5.5.4.4）——
  // 取距起始时间 10ms 前瞻处的规划速度，作为对计算/通信/下位机 PID 延迟的补偿。
  const Eigen::VectorXd x_chosen = Sx_ * x0 + Su_ * u_chosen;
  // 预测状态 x_1（时刻 dt）的速度；起点 x0 的速度为当前世界速度。
  const double v1x = x_chosen(3);
  const double v1y = x_chosen(4);
  const double w1 = x_chosen(5);
  const double s = std::clamp(config_.command_lookahead_time / std::max(config_.dt, 1.0e-9), 0.0, 1.0);
  double vx_world = (1.0 - s) * world_velocity.vx + s * v1x;
  double vy_world = (1.0 - s) * world_velocity.vy + s * v1y;
  const double wz = (1.0 - s) * world_velocity.wz + s * w1;

  // 世界系速度旋转回机体系：v_body = R(yaw)ᵀ v_world。
  const double cy = std::cos(pose.yaw);
  const double sy = std::sin(pose.yaw);
  Twist2D cmd;
  cmd.vx = cy * vx_world + sy * vy_world;
  cmd.vy = -sy * vx_world + cy * vy_world;
  cmd.wz = wz;

  // 启动速度下限（报告 5.5.5.6 变通方法，默认关闭）。
  if (config_.startup_speed_floor > 1.0e-9) {
    const double speed = std::hypot(cmd.vx, cmd.vy);
    if (speed > kSpeedEps && speed < config_.startup_speed_floor) {
      const double k = config_.startup_speed_floor / speed;
      cmd.vx *= k;
      cmd.vy *= k;
    }
  }

  debug_.valid = true;
  debug_.command_speed = std::hypot(cmd.vx, cmd.vy);
  debug_.command_omega = cmd.wz;
  debug_.reference_at_lookahead = sample_reference(trajectory, t0 + config_.command_lookahead_time);
  return cmd;
}

}  // namespace astra_nav
