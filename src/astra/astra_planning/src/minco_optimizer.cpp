#include "astra_planning/minco_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "astra_planning/minco_adapter.hpp"
#include "astra_third_party/gcopter/lbfgs.hpp"
#include "astra_third_party/gcopter/minco.hpp"

namespace astra_nav
{

namespace
{

// 虚拟时间 tau -> 真实段时间 T 的 C2 连续映射，保证 T 恒正（GCOPTER 标准做法）。
inline double tau_to_time(double tau)
{
  return tau > 0.0 ? ((0.5 * tau + 1.0) * tau + 1.0)
                   : 1.0 / ((0.5 * tau - 1.0) * tau + 1.0);
}

// 真实段时间 T -> 虚拟时间 tau 的反映射，用于把初始猜测写入优化变量。
inline double time_to_tau(double time)
{
  return time > 1.0 ? (std::sqrt(2.0 * time - 1.0) - 1.0)
                    : (1.0 - std::sqrt(2.0 / time - 1.0));
}

// dT/dtau，链式法则用：dCost/dtau = dCost/dT * dT/dtau。
inline double grad_time_to_tau(double tau)
{
  if (tau > 0.0) {
    return tau + 1.0;
  }
  const double den = (0.5 * tau - 1.0) * tau + 1.0;
  return (1.0 - tau) / (den * den);
}

// 越界距离斜坡的正则化尺度，对齐 HWSentryNav26 minco_optimizer 的 EPS。
constexpr double kCostRampEps = 1.0e-9;

// 把点夹回代价图足迹，对齐 HWSentryNav26 GridGeometry::clamp_to_footprint。
inline Eigen::Vector2d clamp_to_footprint(const Grid2D & cost_map, const Eigen::Vector2d & point)
{
  const Eigen::Vector2d origin(cost_map.origin.x, cost_map.origin.y);
  const Eigen::Vector2d footprint_max = origin + cost_map.resolution *
    Eigen::Vector2d(cost_map.width, cost_map.height);
  return point.cwiseMax(origin).cwiseMin(footprint_max);
}

// 优化上下文：在 L-BFGS 回调之间传递所有不变量与中间缓存。
struct OptimizeContext
{
  const MincoOptimizerConfig * config{nullptr};
  // 障碍罚直接采样与前端 A* 同源的 0-255 连续代价场，不再走 ESDF 距离场。
  const Grid2D * cost_map{nullptr};
  MincoOptimizeStage stage{MincoOptimizeStage::PreOptimization};
  int piece_count{0};
  Eigen::Matrix<double, 2, 3> head;
  Eigen::Matrix<double, 2, 3> tail;
  minco::MINCO_S3NU * minco{nullptr};
  int iterations{0};
};

// 把优化变量 x 拆为内部路标点 P(2 x (N-1)) 与虚拟时间 tau(N)。
void decode_variables(
  const Eigen::VectorXd & x, int piece_count,
  Eigen::MatrixXd & inner_points, Eigen::VectorXd & times, Eigen::VectorXd & tau)
{
  const int inner = piece_count - 1;
  inner_points.resize(2, inner);
  for (int i = 0; i < inner; ++i) {
    inner_points(0, i) = x(2 * i);
    inner_points(1, i) = x(2 * i + 1);
  }
  tau.resize(piece_count);
  times.resize(piece_count);
  const int tau_offset = 2 * inner;
  for (int i = 0; i < piece_count; ++i) {
    tau(i) = x(tau_offset + i);
    times(i) = tau_to_time(tau(i));
  }
}

// 沿单条轨迹段，在采样点上累加障碍/速度/加速度软约束的代价，
// 并把梯度分摊到该段多项式系数（gradC）与该段时间（gradT）。
// 这是把“离散采样点的 gradPos/gradVel/gradAcc 汇总到控制点/时间”的具体实现。
double accumulate_penalty(
  OptimizeContext & ctx, const Eigen::MatrixX2d & coeffs,
  const Eigen::VectorXd & times, Eigen::MatrixX2d & gradC, Eigen::VectorXd & gradT)
{
  const auto & cfg = *ctx.config;
  const int samples = std::max(2, cfg.samples_per_piece);
  double cost = 0.0;

  for (int piece = 0; piece < ctx.piece_count; ++piece) {
    const Eigen::Matrix<double, 6, 2> c = coeffs.block<6, 2>(6 * piece, 0);
    const double dt = times(piece);
    const double step = dt / samples;

    for (int j = 0; j <= samples; ++j) {
      const double s = step * j;
      // 辛普森积分权重。
      double omega = (j == 0 || j == samples) ? 1.0 : (j % 2 == 1 ? 4.0 : 2.0);
      omega *= step / 3.0;

      const double s2 = s * s;
      const double s3 = s2 * s;
      const double s4 = s2 * s2;
      const double s5 = s4 * s;
      Eigen::Matrix<double, 6, 1> beta0, beta1, beta2;
      beta0 << 1.0, s, s2, s3, s4, s5;
      beta1 << 0.0, 1.0, 2.0 * s, 3.0 * s2, 4.0 * s3, 5.0 * s4;
      beta2 << 0.0, 0.0, 2.0, 6.0 * s, 12.0 * s2, 20.0 * s3;

      const Eigen::Vector2d pos = c.transpose() * beta0;
      const Eigen::Vector2d vel = c.transpose() * beta1;
      const Eigen::Vector2d acc = c.transpose() * beta2;

      Eigen::Vector2d grad_pos = Eigen::Vector2d::Zero();
      Eigen::Vector2d grad_vel = Eigen::Vector2d::Zero();
      Eigen::Vector2d grad_acc = Eigen::Vector2d::Zero();
      double point_cost = 0.0;
      // dCost/dt 的“显式”部分（采样点时间随 s=step*j 移动带来的链式项）。
      double grad_pt = 0.0;

      // ---- 障碍软约束（直接复制 HWSentryNav26 minco_optimizer 的采样点障碍罚）----
      // 罚 = w * 0.5 * value²，value = 归一化连续代价场采样值（0-1），梯度为解析双线性梯度。
      // 越界时在被复制的边界值之上叠加距离斜坡，而不是替换为常数 1 + d/res ——
      // 后者会在 footprint 边界让自由区的罚从 ≈0 阶跃到满值。
      if (ctx.cost_map != nullptr && cfg.weight_obstacle > 0.0) {
        const CostSample cost_sample =
          sample_cost_map_clamped(*ctx.cost_map, Point2D{pos.x(), pos.y()});
        double value = cost_sample.value / 255.0;
        Eigen::Vector2d value_gradient(
          cost_sample.gradient.x / 255.0, cost_sample.gradient.y / 255.0);
        const Eigen::Vector2d clamped = clamp_to_footprint(*ctx.cost_map, pos);
        const Eigen::Vector2d delta = pos - clamped;
        const double outside_squared = delta.squaredNorm();
        if (outside_squared > 0.0) {
          // 斜坡取 d²/(2·res·reg)（d<reg）与 d−reg/2 的 C1 拼接：边界处
          // 值与梯度都为 0，避免 sqrt 在 d→0 处的无界导数与常数偏置。
          const double regularization = kCostRampEps;
          const double distance = std::sqrt(outside_squared);
          const double resolution = ctx.cost_map->resolution;
          if (distance < regularization) {
            value += 0.5 * outside_squared / (resolution * regularization);
            value_gradient += delta / (resolution * regularization);
          } else {
            value += (distance - 0.5 * regularization) / resolution;
            value_gradient += delta / (distance * resolution);
          }
        }
        const double obstacle = cfg.weight_obstacle * 0.5 * value * value;
        cost += omega * obstacle;
        grad_pos += omega * cfg.weight_obstacle * value * value_gradient;
        point_cost += omega * obstacle;
      }

      // ---- 速度软约束 ----
      const double vel_sq = vel.squaredNorm();
      const double vmax_sq = cfg.max_velocity * cfg.max_velocity;
      if (vel_sq > vmax_sq) {
        const double viol = vel_sq - vmax_sq;  // 用平方差，避免 sqrt 的奇异
        const double w = cfg.weight_velocity;
        cost += omega * w * viol * viol;
        grad_vel += omega * w * 2.0 * viol * 2.0 * vel;
        point_cost += omega * w * viol * viol;
      }

      // ---- 加速度软约束 ----
      const double acc_sq = acc.squaredNorm();
      const double amax_sq = cfg.max_acceleration * cfg.max_acceleration;
      if (acc_sq > amax_sq) {
        const double viol = acc_sq - amax_sq;
        const double w = cfg.weight_acceleration;
        cost += omega * w * viol * viol;
        grad_acc += omega * w * 2.0 * viol * 2.0 * acc;
        point_cost += omega * w * viol * viol;
      }

      // ---- 把采样点梯度分摊到该段系数 gradC ----
      // pos = beta0^T C, vel = beta1^T C, acc = beta2^T C
      // dCost/dC += beta0 * grad_pos^T + beta1 * grad_vel^T + beta2 * grad_acc^T
      gradC.block<6, 2>(6 * piece, 0) +=
        beta0 * grad_pos.transpose() + beta1 * grad_vel.transpose() +
        beta2 * grad_acc.transpose();

      // ---- 时间梯度 gradT ----
      // 采样位置 s = (j/samples)*T，故 ds/dT = j/samples。
      // 链式： dCost/dT += (grad_pos·vel + grad_vel·acc) * ds/dT * omega权重内含
      // 另外辛普森权重 omega 也含 step=T/samples，对 T 的依赖通过 point_cost/T 体现。
      const double ds_dT = static_cast<double>(j) / samples;
      grad_pt += grad_pos.dot(vel) * ds_dT + grad_vel.dot(acc) * ds_dT;
      // omega 正比于 step=T/samples，所以 point_cost 对 T 的偏导含 point_cost/T 一项。
      if (dt > 1.0e-9) {
        grad_pt += point_cost / dt;
      }
      gradT(piece) += grad_pt;
    }
  }
  return cost;
}

// 段时间均匀性约束（仅 PRE 阶段）：让每段时间贴近平均段时间，
// 防止控制点被推出狭窄区域后段长突变。对应报告 5.5.4.2 末段的时间约束。
double accumulate_uniform_time(
  const MincoOptimizerConfig & cfg, const Eigen::VectorXd & times, Eigen::VectorXd & gradT)
{
  const int n = static_cast<int>(times.size());
  if (n < 2) {
    return 0.0;
  }
  const double mean = times.mean();
  const double upper = cfg.uniform_time_upper_ratio * mean;
  const double lower = cfg.uniform_time_lower_ratio * mean;
  double cost = 0.0;
  for (int i = 0; i < n; ++i) {
    if (times(i) > upper) {
      const double v = times(i) - upper;
      cost += cfg.weight_uniform_time * v * v;
      gradT(i) += cfg.weight_uniform_time * 2.0 * v;
    } else if (times(i) < lower) {
      const double v = lower - times(i);
      cost += cfg.weight_uniform_time * v * v;
      gradT(i) += -cfg.weight_uniform_time * 2.0 * v;
    }
  }
  return cost;
}

double lbfgs_cost(void * ptr, const Eigen::VectorXd & x, Eigen::VectorXd & grad)
{
  auto & ctx = *static_cast<OptimizeContext *>(ptr);
  ctx.iterations += 1;
  const auto & cfg = *ctx.config;

  Eigen::MatrixXd inner_points;
  Eigen::VectorXd times;
  Eigen::VectorXd tau;
  decode_variables(x, ctx.piece_count, inner_points, times, tau);

  // 用当前变量构造 MINCO，得到多项式系数。
  ctx.minco->setParameters(inner_points, times);

  // 平滑能量及其梯度。
  double energy = 0.0;
  ctx.minco->getEnergy(energy);
  Eigen::MatrixX2d gradC;
  ctx.minco->getEnergyPartialGradByCoeffs(gradC);
  Eigen::VectorXd gradT;
  ctx.minco->getEnergyPartialGradByTimes(gradT);
  gradC *= cfg.weight_energy;
  gradT *= cfg.weight_energy;
  double cost = cfg.weight_energy * energy;

  // 总时间项：cost += w_time * sum(T)，dCost/dT += w_time。
  cost += cfg.weight_time * times.sum();
  for (int i = 0; i < ctx.piece_count; ++i) {
    gradT(i) += cfg.weight_time;
  }

  // 障碍/速度/加速度软约束（沿轨迹采样积分）。
  const Eigen::MatrixX2d & coeffs = ctx.minco->getCoeffs();
  cost += accumulate_penalty(ctx, coeffs, times, gradC, gradT);

  // 段时间均匀性（仅 PRE 阶段）。
  if (ctx.stage == MincoOptimizeStage::PreOptimization) {
    cost += accumulate_uniform_time(cfg, times, gradT);
  }

  // 把系数/时间梯度经 MINCO 反传到内部路标点与段时间。
  Eigen::Matrix2Xd grad_by_points;
  Eigen::VectorXd grad_by_times;
  ctx.minco->propogateGrad(gradC, gradT, grad_by_points, grad_by_times);

  // 写回梯度向量：先内部点，再虚拟时间。
  grad.setZero(x.size());
  const int inner = ctx.piece_count - 1;
  for (int i = 0; i < inner; ++i) {
    grad(2 * i) = grad_by_points(0, i);
    grad(2 * i + 1) = grad_by_points(1, i);
  }
  const int tau_offset = 2 * inner;
  for (int i = 0; i < ctx.piece_count; ++i) {
    grad(tau_offset + i) = grad_by_times(i) * grad_time_to_tau(tau(i));
  }

  if (!std::isfinite(cost)) {
    return std::numeric_limits<double>::infinity();
  }
  return cost;
}

}  // namespace

MincoOptimizer::MincoOptimizer(const MincoOptimizerConfig & config)
: config_(config)
{
}

MincoOptimizerResult MincoOptimizer::optimize(
  const std::vector<Point2D> & waypoints, const std::vector<double> & times,
  const MincoBoundaryState & head, const MincoBoundaryState & tail,
  const Grid2D & cost_map, MincoOptimizeStage stage) const
{
  MincoOptimizerResult result;
  const int piece_count = static_cast<int>(waypoints.size()) - 1;
  if (piece_count < 1 || static_cast<int>(times.size()) != static_cast<int>(waypoints.size())) {
    return result;
  }

  // 边界状态矩阵（位置/速度/加速度）。
  Eigen::Matrix<double, 2, 3> head_state;
  head_state << head.position.x, head.velocity.x, head.acceleration.x,
    head.position.y, head.velocity.y, head.acceleration.y;
  Eigen::Matrix<double, 2, 3> tail_state;
  tail_state << tail.position.x, tail.velocity.x, tail.acceleration.x,
    tail.position.y, tail.velocity.y, tail.acceleration.y;

  minco::MINCO_S3NU minco;
  minco.setConditions(head_state, tail_state, piece_count);

  // 初始优化变量：内部路标点 + 由初始段时间反映射得到的虚拟时间。
  const int inner = piece_count - 1;
  Eigen::VectorXd x(2 * inner + piece_count);
  for (int i = 0; i < inner; ++i) {
    x(2 * i) = waypoints[static_cast<std::size_t>(i + 1)].x;
    x(2 * i + 1) = waypoints[static_cast<std::size_t>(i + 1)].y;
  }
  const int tau_offset = 2 * inner;
  for (int i = 0; i < piece_count; ++i) {
    const double seg = std::max(
      times[static_cast<std::size_t>(i + 1)] - times[static_cast<std::size_t>(i)], 1.0e-3);
    x(tau_offset + i) = time_to_tau(seg);
  }

  OptimizeContext ctx;
  ctx.config = &config_;
  ctx.cost_map = &cost_map;
  ctx.stage = stage;
  ctx.piece_count = piece_count;
  ctx.head = head_state;
  ctx.tail = tail_state;
  ctx.minco = &minco;

  Eigen::VectorXd grad_init(x.size());
  result.initial_cost = lbfgs_cost(&ctx, x, grad_init);

  lbfgs::lbfgs_parameter_t params;
  params.mem_size = std::max(3, config_.memory_size);
  params.past = std::max(0, config_.past);
  params.delta = std::max(0.0, config_.delta);
  params.g_epsilon = std::max(0.0, config_.g_epsilon);
  params.max_iterations = std::max(1, config_.max_iterations);
  params.min_step = 1.0e-12;
  params.max_linesearch = 64;

  ctx.iterations = 0;
  double final_cost = result.initial_cost;
  result.status = lbfgs::lbfgs_optimize(x, final_cost, lbfgs_cost, nullptr, nullptr, &ctx, params);
  result.iterations = ctx.iterations;
  result.final_cost = final_cost;

  // 用优化结果构造最终轨迹（经适配层，保持外部接口一致）。
  Eigen::MatrixXd inner_points;
  Eigen::VectorXd opt_times;
  Eigen::VectorXd tau;
  decode_variables(x, piece_count, inner_points, opt_times, tau);

  std::vector<Point2D> opt_waypoints;
  opt_waypoints.reserve(piece_count + 1);
  opt_waypoints.push_back(head.position);
  for (int i = 0; i < inner; ++i) {
    opt_waypoints.push_back({inner_points(0, i), inner_points(1, i)});
  }
  opt_waypoints.push_back(tail.position);

  std::vector<double> cumulative(piece_count + 1, 0.0);
  for (int i = 0; i < piece_count; ++i) {
    cumulative[static_cast<std::size_t>(i + 1)] = cumulative[static_cast<std::size_t>(i)] + opt_times(i);
  }

  result.trajectory = MincoTrajectory::fit(opt_waypoints, cumulative, head, tail);
  const bool accepted =
    result.status == lbfgs::LBFGS_CONVERGENCE || result.status == lbfgs::LBFGS_STOP ||
    result.status == lbfgs::LBFGSERR_MAXIMUMITERATION;
  result.optimized = accepted && !result.trajectory.empty() && std::isfinite(final_cost);
  return result;
}

}  // namespace astra_nav
