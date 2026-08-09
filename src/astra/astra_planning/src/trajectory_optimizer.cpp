#include "astra_planning/trajectory_optimizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace astra_nav
{

namespace
{

// 继承速度在首段切向上的前向投影，逆向分量取 0。
// 对应上游 SpeedProfileOptimizer::optimize 里的
// std::max(0.0, current_velocity_map.dot(tangent))（speed_profile_optimizer.cpp:573）。
double projected_start_speed(const Point2D & velocity, const Point2D & from, const Point2D & to)
{
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double dn = std::hypot(dx, dy);
  if (dn < 1.0e-6) {
    return 0.0;
  }
  return std::max(0.0, (velocity.x * dx + velocity.y * dy) / dn);
}

}  // namespace

TrajectoryOptimizer::TrajectoryOptimizer(const TrajectoryOptimizerConfig & config)
: config_(config)
{
}

MincoTrajectory TrajectoryOptimizer::optimize(
  const std::vector<Point2D> & coarse_path, const Grid2D & cost_map, const EsdfMap & esdf) const
{
  return optimize_with_stats(coarse_path, cost_map, esdf).trajectory;
}

TrajectoryOptimizationResult TrajectoryOptimizer::optimize_with_stats(
  const std::vector<Point2D> & coarse_path, const Grid2D & cost_map, const EsdfMap & esdf) const
{
  return optimize_with_stats(coarse_path, cost_map, esdf, {});
}

TrajectoryOptimizationResult TrajectoryOptimizer::optimize_with_stats(
  const std::vector<Point2D> & coarse_path, const Grid2D & cost_map, const EsdfMap & esdf,
  const TrajectoryBoundaryCondition & boundary_condition) const
{
  TrajectoryOptimizationResult output;
  output.stats.input_points = static_cast<int>(coarse_path.size());
  if (coarse_path.size() < 2) {
    return output;
  }

  // 新后端：基于 MINCO 控制点与分段时间梯度的两步优化（报告 5.5.4）。
  if (config_.use_minco_backend) {
    return optimize_minco_backend(coarse_path, cost_map, esdf, boundary_condition);
  }

  const auto start_time = std::chrono::steady_clock::now();
  auto waypoints = resample_polyline(coarse_path, config_.waypoint_spacing);
  output.stats.resampled_points = static_cast<int>(waypoints.size());
  auto pre = optimize_waypoints(waypoints, esdf, config_.pre_iterations, 0.7);
  auto fine = optimize_waypoints(pre.points, esdf, config_.fine_iterations, 1.0);
  const double legacy_start_speed = boundary_condition.inherit_start_state &&
    fine.points.size() >= 2
    ? projected_start_speed(boundary_condition.start_velocity, fine.points[0], fine.points[1])
    : 0.0;
  auto times = cumulative_times(
    fine.points, config_.max_velocity, config_.max_acceleration, legacy_start_speed);
  if (config_.initial_time_scale > 1.0e-3 && config_.initial_time_scale < 1.0 - 1.0e-6) {
    scale_times(times, config_.initial_time_scale);
  }
  output.trajectory = fit_minco(fine.points, times, boundary_condition);
  output.stats.quality = evaluate_quality(output.trajectory, esdf);
  if (config_.dynamic_time_scaling_enabled && !output.trajectory.empty()) {
    const int max_scaling_iterations = std::max(0, config_.dynamic_time_scaling_max_iterations);
    for (int i = 0; i < max_scaling_iterations; ++i) {
      const double velocity_ratio = config_.max_velocity > 1.0e-6 ?
        output.stats.quality.max_velocity / config_.max_velocity :
        1.0;
      const double acceleration_ratio = config_.max_acceleration > 1.0e-6 ?
        std::sqrt(output.stats.quality.max_acceleration / config_.max_acceleration) :
        1.0;
      const double constraint_ratio = std::max(velocity_ratio, acceleration_ratio);
      if (constraint_ratio <= 1.0 + 1.0e-3) {
        break;
      }
      const double required_scale =
        constraint_ratio * std::max(1.0, config_.dynamic_time_scaling_safety_ratio);

      scale_times(times, required_scale);
      output.stats.time_scaling_factor *= required_scale;
      output.stats.time_scaling_iterations += 1;
      output.trajectory = fit_minco(fine.points, times, boundary_condition);
      output.stats.quality = evaluate_quality(output.trajectory, esdf);
      if (output.trajectory.empty()) {
        break;
      }
    }
  }
  output.stats.pre_lbfgs_status = pre.status;
  output.stats.pre_lbfgs_iterations = pre.iterations;
  output.stats.pre_initial_cost = pre.initial_cost;
  output.stats.pre_final_cost = pre.final_cost;
  output.stats.pre_optimized = pre.optimized;
  output.stats.fine_lbfgs_status = fine.status;
  output.stats.fine_lbfgs_iterations = fine.iterations;
  output.stats.fine_initial_cost = fine.initial_cost;
  output.stats.fine_final_cost = fine.final_cost;
  output.stats.fine_optimized = fine.optimized;
  output.stats.start_state_inherited = boundary_condition.inherit_start_state;

  const auto end_time = std::chrono::steady_clock::now();
  output.stats.optimize_time_ms =
    std::chrono::duration<double, std::milli>(end_time - start_time).count();
  return output;
}

MincoOptimizerConfig TrajectoryOptimizer::make_minco_config() const
{
  MincoOptimizerConfig cfg;
  cfg.max_velocity = config_.max_velocity;
  cfg.max_acceleration = config_.max_acceleration;
  cfg.weight_energy = config_.minco_weight_energy;
  cfg.weight_time = config_.minco_weight_time;
  cfg.weight_obstacle = config_.minco_weight_obstacle;
  cfg.weight_velocity = config_.minco_weight_velocity;
  cfg.weight_acceleration = config_.minco_weight_acceleration;
  cfg.weight_uniform_time = config_.minco_weight_uniform_time;
  cfg.samples_per_piece = config_.minco_samples_per_piece;
  cfg.memory_size = config_.lbfgs_memory_size;
  cfg.past = config_.lbfgs_past;
  cfg.delta = config_.lbfgs_delta;
  cfg.g_epsilon = config_.minco_g_epsilon;  // 修复:此前 g_epsilon 从未从上层配置传入,一直用结构体默认值。
  return cfg;
}

TrajectoryOptimizationResult TrajectoryOptimizer::optimize_minco_backend(
  const std::vector<Point2D> & coarse_path, const Grid2D & cost_map, const EsdfMap & esdf,
  const TrajectoryBoundaryCondition & boundary_condition) const
{
  TrajectoryOptimizationResult output;
  output.stats.input_points = static_cast<int>(coarse_path.size());
  output.stats.used_minco_backend = true;
  const auto start_time = std::chrono::steady_clock::now();

  // 1) 把前端路径按设定间距重采样为初始路标点，并按梯形加减速估计初始分段时间。
  auto waypoints = resample_polyline(coarse_path, config_.waypoint_spacing);
  if (waypoints.size() < 2) {
    return output;
  }
  output.stats.resampled_points = static_cast<int>(waypoints.size());
  // 起点速度取继承速度在首段切向上的【前向投影】（上游 SpeedProfileOptimizer 用
  // current_velocity_map.dot(tangent) 并对负值取 0）。不传起点速度的话，5 Hz 重规划
  // 每一轮都会把剖面重排成"从静止起步"，机器人永远跑在加速段起点上。
  const double start_speed = boundary_condition.inherit_start_state
    ? projected_start_speed(boundary_condition.start_velocity, waypoints[0], waypoints[1])
    : 0.0;
  auto times = cumulative_times_with_turning(
    waypoints, config_.max_velocity, config_.max_acceleration, config_.turn_time_weight,
    start_speed);

  // 2) 边界状态：起点可继承上一轨迹的速度/加速度（重规划连续性），终点静止。
  MincoBoundaryState head;
  head.position = waypoints.front();
  if (boundary_condition.inherit_start_state) {
    head.velocity = boundary_condition.start_velocity;
    head.acceleration = boundary_condition.start_acceleration;
    // 规整继承边界：剔除逆向纵向分量（避免冲过拐弯点甩圈）+ 钳制速度/加速度幅值到上限。
    regularize_inherited_boundary(head, waypoints[0], waypoints[1]);
  } else {
    head.velocity = {};
    head.acceleration = {};
  }
  MincoBoundaryState tail;
  tail.position = waypoints.back();
  tail.velocity = {};        // 终点速度为零，保证到点停稳（修复旧实现终点非零速问题）
  tail.acceleration = {};

  // 3) 两步优化：PRE 得稳定形状，FINELY 得良好动力学。
  auto pre_config = make_minco_config();
  pre_config.max_iterations = config_.minco_pre_iterations;
  MincoOptimizer pre_optimizer(pre_config);

  auto pre = pre_optimizer.optimize(
    waypoints, times, head, tail, cost_map, MincoOptimizeStage::PreOptimization);
  output.stats.pre_lbfgs_status = pre.status;
  output.stats.pre_lbfgs_iterations = pre.iterations;
  output.stats.pre_initial_cost = pre.initial_cost;
  output.stats.pre_final_cost = pre.final_cost;
  output.stats.pre_optimized = pre.optimized;

  MincoTrajectory pre_trajectory = pre.trajectory;
  if (pre_trajectory.empty()) {
    // PRE 失败：直接用初始路标点拟合一条 MINCO 兜底，保证有可用输出。
    output.trajectory = MincoTrajectory::fit(waypoints, times, head, tail);
    output.stats.quality = evaluate_quality(output.trajectory, esdf);
    const auto end_time = std::chrono::steady_clock::now();
    output.stats.optimize_time_ms =
      std::chrono::duration<double, std::milli>(end_time - start_time).count();
    output.stats.start_state_inherited = boundary_condition.inherit_start_state;
    return output;
  }

  // 以 PRE 结果的等分采样作为 FINELY 的初值路标点与时间。
  const int piece_count = std::max(1, output.stats.resampled_points - 1);
  std::vector<Point2D> fine_waypoints;
  std::vector<double> fine_times;
  fine_waypoints.reserve(piece_count + 1);
  fine_times.reserve(piece_count + 1);
  const double duration = pre_trajectory.duration();
  for (int i = 0; i <= piece_count; ++i) {
    const double t = duration * static_cast<double>(i) / static_cast<double>(piece_count);
    const auto sample = pre_trajectory.evaluate(t);
    fine_waypoints.push_back({sample.x, sample.y});
    fine_times.push_back(t);
  }

  auto fine_config = make_minco_config();
  fine_config.max_iterations = config_.minco_fine_iterations;
  MincoOptimizer fine_optimizer(fine_config);
  auto fine = fine_optimizer.optimize(
    fine_waypoints, fine_times, head, tail, cost_map, MincoOptimizeStage::FinelyOptimization);
  output.stats.fine_lbfgs_status = fine.status;
  output.stats.fine_lbfgs_iterations = fine.iterations;
  output.stats.fine_initial_cost = fine.initial_cost;
  output.stats.fine_final_cost = fine.final_cost;
  output.stats.fine_optimized = fine.optimized;

  // FINELY 成功则采用其结果，否则回退到 PRE 结果。
  output.trajectory = !fine.trajectory.empty() ? fine.trajectory : pre_trajectory;

  // 回环检测与自动恢复（报告 5.5.4.2：PRE 得稳定优秀的形状，FINELY 动力学好但形状可能变差）。
  // 若所选轨迹中段甩圈/绕圈，而 PRE 形状无环，则回退到 PRE，消除中段大转角。
  if (config_.loop_detection_enabled) {
    double lx = 0.0;
    double ly = 0.0;
    const double chosen_turn = detect_loop(output.trajectory, lx, ly);
    const double thr = config_.loop_detection_angle_threshold;
    if (chosen_turn > thr && !pre_trajectory.empty()) {
      double plx = 0.0;
      double ply = 0.0;
      const double pre_turn = detect_loop(pre_trajectory, plx, ply);
      if (pre_turn <= thr) {
        output.trajectory = pre_trajectory;
        output.stats.loop_recovered = true;
      }
    }
  }

  output.stats.quality = evaluate_quality(output.trajectory, esdf);
  output.stats.start_state_inherited = boundary_condition.inherit_start_state;

  // 双向时间缩放：把巡航速度收敛到 max_velocity（受 max_acceleration 约束）。
  // MINCO 两步优化只定“形状”，速度由这一步对时间整体缩放决定，
  // 使 max_velocity / max_acceleration 成为真正生效、可纯参数调节的速度上限。
  apply_dynamic_time_scaling(output, boundary_condition, esdf);

  // 最终回环诊断（时间缩放之后）：写入统计，供 planner 日志定位甩圈位置与转角。
  if (config_.loop_detection_enabled) {
    double lx = 0.0;
    double ly = 0.0;
    const double final_turn = detect_loop(output.trajectory, lx, ly);
    output.stats.loop_max_window_angle = final_turn;
    output.stats.loop_detected = final_turn > config_.loop_detection_angle_threshold;
    output.stats.loop_position_x = lx;
    output.stats.loop_position_y = ly;
  }

  const auto end_time = std::chrono::steady_clock::now();
  output.stats.optimize_time_ms =
    std::chrono::duration<double, std::milli>(end_time - start_time).count();
  return output;
}

void TrajectoryOptimizer::apply_dynamic_time_scaling(
  TrajectoryOptimizationResult & output,
  const TrajectoryBoundaryCondition & boundary_condition, const EsdfMap & esdf) const
{
  if (!config_.dynamic_time_scaling_enabled || output.trajectory.empty()) {
    return;
  }
  const double vmax = config_.max_velocity;
  const double amax = config_.max_acceleration;
  const double duration = output.trajectory.duration();
  if (vmax <= 1.0e-6 || duration <= 1.0e-6) {
    return;
  }

  // 把当前轨迹按分段数等时间采样为空间路标点，保留形状，仅改时间标尺。
  const int n = std::max(2, output.stats.resampled_points);
  std::vector<Point2D> points;
  std::vector<double> times;
  points.reserve(n);
  times.reserve(n);
  for (int i = 0; i < n; ++i) {
    const double t = duration * static_cast<double>(i) / static_cast<double>(n - 1);
    const auto s = output.trajectory.evaluate(t);
    points.push_back({s.x, s.y});
    times.push_back(t);
  }

  // 边界与 MINCO 后端一致：起点可继承上一轨迹状态，终点零速（到点停稳）。
  MincoBoundaryState head;
  head.position = points.front();
  if (boundary_condition.inherit_start_state) {
    head.velocity = boundary_condition.start_velocity;
    head.acceleration = boundary_condition.start_acceleration;
    // 与主优化一致：规整继承边界（剔除逆向分量 + 钳制幅值）。日志显示高速甩圈/加速度
    // 爆炸均发生在时间缩放阶段，若此处不规整，会把主优化已规整掉的乱继承重新塞回去。
    if (points.size() >= 2) {
      regularize_inherited_boundary(head, points[0], points[1]);
    }
  }
  MincoBoundaryState tail;
  tail.position = points.back();  // velocity/acceleration 默认零

  double total_scale = 1.0;
  const int max_iter = std::max(1, config_.dynamic_time_scaling_max_iterations);
  for (int iter = 0; iter < max_iter; ++iter) {
    const double v_actual = output.stats.quality.max_velocity;
    const double a_actual = output.stats.quality.max_acceleration;
    if (v_actual <= 1.0e-6) {
      break;
    }
    // 期望缩放使峰值速度命中 vmax：scale<1 压缩时间（加速），scale>1 拉伸时间（减速）。
    double scale = v_actual / vmax;
    // 加速度约束：时间缩放 s 使加速度按 1/s² 变化，要求 a_actual/s² ≤ amax，
    // 即 s ≥ sqrt(a_actual/amax)。加速时（s<1）以此为下限，避免加速度超限。
    if (amax > 1.0e-6) {
      const double accel_floor = std::sqrt(std::max(a_actual, 0.0) / amax);
      scale = std::max(scale, accel_floor);
    }
    if (std::abs(scale - 1.0) < 1.0e-2) {
      break;  // 已收敛到速度/加速度上限附近
    }
    // 减速方向留安全余量，避免反复横跳。
    if (scale > 1.0) {
      scale *= std::max(1.0, config_.dynamic_time_scaling_safety_ratio);
    }
    scale_times(times, scale);
    const auto scaled = MincoTrajectory::fit(points, times, head, tail);
    if (scaled.empty()) {
      break;
    }
    // 若该次缩放重拟合引入了回环（而缩放前无环），撤销本次缩放并停止。
    // 时间压缩会放大起点继承速度/加速度的过冲，可能在折角处甩圈。
    if (config_.loop_detection_enabled) {
      double sx = 0.0;
      double sy = 0.0;
      double px = 0.0;
      double py = 0.0;
      const double thr = config_.loop_detection_angle_threshold;
      if (detect_loop(scaled, sx, sy) > thr && detect_loop(output.trajectory, px, py) <= thr) {
        scale_times(times, 1.0 / scale);  // 还原 times，保持与已采纳轨迹一致
        output.stats.loop_recovered = true;
        break;
      }
    }
    output.trajectory = scaled;
    output.stats.quality = evaluate_quality(scaled, esdf);
    total_scale *= scale;
    output.stats.time_scaling_iterations += 1;
  }
  output.stats.time_scaling_factor = total_scale;
}

// 规整继承的起点边界：①方向感知——剔除与新路径方向相反的纵向分量（冲过拐弯点后甩圈）；
// ②幅值钳制——把继承的速度/加速度钳到动力学上限。机器人剧烈机动时继承的瞬时加速度
// 可达十几 m/s²，直接作为 MINCO 起点硬边界会逼出 20+ m/s² 的不可行轨迹（时间缩放无法
// 降低固定的起点边界值），进而“机动→继承爆炸→更猛机动”正反馈，导致目标附近来回过冲。
void TrajectoryOptimizer::regularize_inherited_boundary(
  MincoBoundaryState & head, const Point2D & from, const Point2D & to) const
{
  if (!config_.directional_start_inheritance) {
    return;
  }
  double dx = to.x - from.x;
  double dy = to.y - from.y;
  const double dn = std::hypot(dx, dy);
  if (dn >= 1.0e-6) {
    dx /= dn;
    dy /= dn;
    const double v_along = head.velocity.x * dx + head.velocity.y * dy;
    if (v_along < 0.0) {
      head.velocity.x -= v_along * dx;
      head.velocity.y -= v_along * dy;
    }
    const double a_along = head.acceleration.x * dx + head.acceleration.y * dy;
    if (a_along < 0.0) {
      head.acceleration.x -= a_along * dx;
      head.acceleration.y -= a_along * dy;
    }
  }
  // 幅值钳制到动力学上限，保证起点边界条件可行、打断继承正反馈。
  const double vmag = std::hypot(head.velocity.x, head.velocity.y);
  if (config_.max_velocity > 1.0e-6 && vmag > config_.max_velocity) {
    const double k = config_.max_velocity / vmag;
    head.velocity.x *= k;
    head.velocity.y *= k;
  }
  const double amag = std::hypot(head.acceleration.x, head.acceleration.y);
  if (config_.max_acceleration > 1.0e-6 && amag > config_.max_acceleration) {
    const double k = config_.max_acceleration / amag;
    head.acceleration.x *= k;
    head.acceleration.y *= k;
  }
}

// 沿弧长滑窗累计【不解缠】航向变化，返回最大窗口净转角（弧度）。
// 不解缠是关键：一个完整的圈航向累计接近 ±2π，而正常折角即便累计也远小于阈值。
double TrajectoryOptimizer::detect_loop(
  const MincoTrajectory & trajectory, double & out_x, double & out_y) const
{
  out_x = 0.0;
  out_y = 0.0;
  if (trajectory.empty()) {
    return 0.0;
  }
  // 以固定弧长间隔重采样航向序列。
  const auto samples = trajectory.sample(0.05);
  if (samples.size() < 3) {
    return 0.0;
  }
  // 逐段航向（由相邻采样点位移决定），以及每段弧长。
  std::vector<double> seg_heading;
  std::vector<double> seg_len;
  std::vector<Point2D> seg_mid;
  seg_heading.reserve(samples.size());
  seg_len.reserve(samples.size());
  seg_mid.reserve(samples.size());
  for (std::size_t i = 1; i < samples.size(); ++i) {
    const double dx = samples[i].x - samples[i - 1].x;
    const double dy = samples[i].y - samples[i - 1].y;
    const double len = std::hypot(dx, dy);
    if (len < 1.0e-6) {
      continue;
    }
    seg_heading.push_back(std::atan2(dy, dx));
    seg_len.push_back(len);
    seg_mid.push_back({0.5 * (samples[i].x + samples[i - 1].x),
                       0.5 * (samples[i].y + samples[i - 1].y)});
  }
  if (seg_heading.size() < 2) {
    return 0.0;
  }
  // 相邻段的航向增量（解缠到 [-π,π]，表示该点真实转角），再沿弧长滑窗累计【带符号】。
  // 滑窗带符号累计 = 窗口内净转角；一个圈净转 ~±2π，正常折角窗口内净转远小。
  const double window = std::max(0.1, config_.loop_detection_window);
  std::vector<double> dtheta(seg_heading.size(), 0.0);
  for (std::size_t i = 1; i < seg_heading.size(); ++i) {
    dtheta[i] = normalize_angle(seg_heading[i] - seg_heading[i - 1]);
  }
  double max_abs = 0.0;
  for (std::size_t i = 1; i < dtheta.size(); ++i) {
    double acc = 0.0;
    double arc = 0.0;
    for (std::size_t j = i; j < dtheta.size() && arc < window; ++j) {
      acc += dtheta[j];
      arc += seg_len[j];
      if (std::abs(acc) > max_abs) {
        max_abs = std::abs(acc);
        out_x = seg_mid[j].x;
        out_y = seg_mid[j].y;
      }
    }
  }
  return max_abs;
}


WaypointLbfgsResult TrajectoryOptimizer::optimize_waypoints(
  const std::vector<Point2D> & input, const EsdfMap & esdf, int iterations,
  double obstacle_gain) const
{
  WaypointLbfgsResult fallback;
  fallback.points = input;
  if (input.size() <= 2 || !esdf.valid()) {
    return fallback;
  }

  WaypointLbfgsConfig lbfgs_config;
  lbfgs_config.reference_weight = config_.lbfgs_reference_weight;
  lbfgs_config.smooth_weight = config_.lbfgs_smooth_weight;
  lbfgs_config.obstacle_weight = config_.lbfgs_obstacle_weight * obstacle_gain;
  lbfgs_config.obstacle_margin = config_.obstacle_cost_radius;
  lbfgs_config.max_iterations = std::max(1, iterations);
  lbfgs_config.memory_size = config_.lbfgs_memory_size;
  lbfgs_config.past = config_.lbfgs_past;
  lbfgs_config.delta = config_.lbfgs_delta;

  WaypointLbfgsOptimizer optimizer(lbfgs_config);
  const auto result = optimizer.optimize(input, esdf);
  return result.optimized ? result : fallback;
}

MincoTrajectory TrajectoryOptimizer::fit_minco(
  const std::vector<Point2D> & points, const std::vector<double> & times,
  const TrajectoryBoundaryCondition & boundary_condition) const
{
  if (!boundary_condition.inherit_start_state || points.size() < 2 || times.size() != points.size()) {
    return MincoTrajectory::fit(points, times);
  }

  MincoBoundaryState head;
  head.position = points.front();
  head.velocity = boundary_condition.start_velocity;
  head.acceleration = boundary_condition.start_acceleration;

  MincoBoundaryState tail;
  tail.position = points.back();
  const double tail_dt = std::max(times.back() - times[times.size() - 2], 0.05);
  tail.velocity = {
    (points.back().x - points[points.size() - 2].x) / tail_dt,
    (points.back().y - points[points.size() - 2].y) / tail_dt};
  tail.acceleration = {};

  return MincoTrajectory::fit(points, times, head, tail);
}

void TrajectoryOptimizer::scale_times(std::vector<double> & times, double scale) const
{
  if (times.size() < 2 || scale <= 1.0e-6) {
    return;
  }
  for (std::size_t i = 1; i < times.size(); ++i) {
    times[i] *= scale;
  }
}

TrajectoryQuality TrajectoryOptimizer::evaluate_quality(
  const MincoTrajectory & trajectory, const EsdfMap & esdf) const
{
  TrajectoryQuality quality;
  if (trajectory.empty()) {
    return quality;
  }

  quality.min_esdf_distance = esdf.valid() ? 1.0e9 : 0.0;
  const auto samples = trajectory.sample(0.05);
  for (const auto & point : samples) {
    quality.max_velocity = std::max(quality.max_velocity, std::hypot(point.vx, point.vy));
    quality.max_acceleration = std::max(quality.max_acceleration, std::hypot(point.ax, point.ay));
    if (esdf.valid()) {
      quality.min_esdf_distance =
        std::min(quality.min_esdf_distance, esdf.query_distance(point.x, point.y));
    }
  }
  if (quality.min_esdf_distance > 1.0e8) {
    quality.min_esdf_distance = 0.0;
  }
  quality.velocity_violation = std::max(0.0, quality.max_velocity - config_.max_velocity);
  quality.acceleration_violation =
    std::max(0.0, quality.max_acceleration - config_.max_acceleration);
  return quality;
}

}  // namespace astra_nav
