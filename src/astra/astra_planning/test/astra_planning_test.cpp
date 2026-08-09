#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

#include "astra_common/common.hpp"
#include "astra_mapping/esdf_map.hpp"
#include "astra_mapping/obstacle_extractor.hpp"
#include "astra_planning/minco_trajectory.hpp"
#include "astra_planning/replan_strategy.hpp"
#include "astra_planning/spatial_grid_astar.hpp"
#include "astra_planning/trajectory_optimizer.hpp"
#include "astra_planning/waypoint_lbfgs_optimizer.hpp"

namespace
{

// 代价场改成 0-255 连续值后，“障碍本体”必须写成 >= kOccupiedCostThreshold 的值，
// 否则 EsdfMap / SpatialGridAstar / Grid2D::occupied 都不会把它当成硬占用。
constexpr std::uint8_t kObstacle = 255;

// 前端 A*（SpatialGridAstar）+ 后端轨迹优化的贯通冒烟测试。
void test_frontend_and_trajectory()
{
  astra_nav::Grid2D grid;
  grid.width = 40;
  grid.height = 30;
  grid.resolution = 0.1;
  grid.origin = {-2.0, -1.5};
  grid.data.assign(static_cast<std::size_t>(grid.width * grid.height), 0);
  for (int y = 0; y < grid.height; ++y) {
    if (y >= 13 && y <= 17) {
      continue;
    }
    grid.at(20, y) = kObstacle;
  }

  astra_nav::InflationParams inflation;
  inflation.full_cost_radius_m = 0.1;
  inflation.cutoff_radius_m = 0.3;
  const auto cost_map = astra_nav::ObstacleExtractor::inflate(grid, inflation);

  astra_nav::SpatialGridAstar astar(cost_map, {});
  const auto route = astar.plan({-1.5, 0.0}, {1.5, 0.0});
  assert(route.success);
  // A* 逐格展开，路径点数量必然远多于 2。
  assert(route.route.world_points.size() > 2);
  const auto resampled = astra_nav::resample_polyline(route.route.world_points, 0.35);
  assert(resampled.size() > 2);

  astra_nav::EsdfMap esdf(grid);
  const auto query = esdf.query_distance_and_gradient(0.0, 0.0);
  assert(std::isfinite(query.first));

  astra_nav::TrajectoryOptimizer optimizer({});
  const auto optimization =
    optimizer.optimize_with_stats(route.route.world_points, cost_map, esdf);
  const auto & trajectory = optimization.trajectory;
  assert(!trajectory.empty());
  assert(optimization.stats.input_points == static_cast<int>(route.route.world_points.size()));
  assert(optimization.stats.resampled_points >= 2);
  assert(optimization.stats.optimize_time_ms >= 0.0);
  assert(optimization.stats.quality.max_velocity > 0.0);
  assert(optimization.stats.quality.max_acceleration >= 0.0);
  assert(optimization.stats.quality.velocity_violation >= 0.0);
  assert(optimization.stats.quality.acceleration_violation >= 0.0);
  const auto samples = trajectory.sample(0.05);
  assert(samples.size() > 3);
  // 默认启用 MINCO 后端：确认走的是新后端路径。
  assert(optimization.stats.used_minco_backend);

  // 旧后端（路点级 L-BFGS + 事后时间缩放）仍可用，作为对照与回退。
  // 用严格动力学上限触发事后时间缩放，验证旧路径完整保留。
  astra_nav::TrajectoryOptimizerConfig legacy_config;
  legacy_config.use_minco_backend = false;
  legacy_config.max_velocity = 0.18;
  legacy_config.max_acceleration = 0.35;
  legacy_config.dynamic_time_scaling_enabled = true;
  legacy_config.dynamic_time_scaling_max_iterations = 6;
  legacy_config.initial_time_scale = 0.35;
  astra_nav::TrajectoryOptimizer legacy_optimizer(legacy_config);
  const auto legacy_optimization =
    legacy_optimizer.optimize_with_stats(route.route.world_points, cost_map, esdf);
  assert(!legacy_optimization.trajectory.empty());
  assert(!legacy_optimization.stats.used_minco_backend);
  assert(legacy_optimization.stats.time_scaling_iterations > 0);
  assert(legacy_optimization.stats.time_scaling_factor > 1.0);
  assert(legacy_optimization.stats.quality.max_velocity <= legacy_config.max_velocity * 1.08);
  assert(
    legacy_optimization.stats.quality.max_acceleration <= legacy_config.max_acceleration * 1.12);
}

// 验证报告 5.5.4 的核心：MINCO 控制点/分段时间两步优化能把前端绕障路径
// 优化为平滑、动力学可行、远离障碍的轨迹。
void test_minco_backend_optimization()
{
  // 中间一块障碍，前端 A* 绕行，后端两步优化收紧。
  astra_nav::Grid2D grid;
  grid.width = 80;
  grid.height = 60;
  grid.resolution = 0.1;
  grid.origin = {-4.0, -3.0};
  grid.data.assign(static_cast<std::size_t>(grid.width * grid.height), 0);
  for (int x = 37; x <= 43; ++x) {
    for (int y = 25; y <= 35; ++y) {
      grid.at(x, y) = kObstacle;
    }
  }
  astra_nav::EsdfMap esdf(grid);

  astra_nav::InflationParams inflation;
  inflation.full_cost_radius_m = 0.4;
  inflation.cutoff_radius_m = 0.8;
  const auto cost_map = astra_nav::ObstacleExtractor::inflate(grid, inflation);

  astra_nav::SpatialGridAstar astar(cost_map, {});
  const auto route = astar.plan({-2.5, 0.0}, {2.5, 0.0});
  assert(route.success);

  astra_nav::TrajectoryOptimizerConfig config;
  config.use_minco_backend = true;
  config.max_velocity = 2.0;
  config.max_acceleration = 3.0;
  config.obstacle_cost_radius = 0.4;
  config.minco_weight_time = 8.0;
  config.minco_weight_obstacle = 2000.0;
  astra_nav::TrajectoryOptimizer optimizer(config);
  const auto result = optimizer.optimize_with_stats(route.route.world_points, cost_map, esdf);

  assert(result.stats.used_minco_backend);
  assert(!result.trajectory.empty());
  assert(result.stats.pre_optimized);

  // 框架级不变量（与具体参数无关）：
  const auto trajectory_samples = result.trajectory.sample(0.05);
  assert(trajectory_samples.size() > 3);

  // 1) 优化确实降低了代价（PRE 阶段）。
  assert(result.stats.pre_final_cost <= result.stats.pre_initial_cost);

  // 2) 轨迹严格过起点与终点。
  const auto start = result.trajectory.evaluate(0.0);
  const auto end = result.trajectory.evaluate(result.trajectory.duration());
  assert(std::hypot(
      start.x - route.route.world_points.front().x,
      start.y - route.route.world_points.front().y) < 0.05);
  assert(std::hypot(
      end.x - route.route.world_points.back().x,
      end.y - route.route.world_points.back().y) < 0.05);

  // 3) 终点速度接近零（修复旧实现终点非零速问题，保证到点停稳）。
  assert(std::hypot(end.vx, end.vy) < 0.05);

  // 4) 优化后轨迹最小 ESDF 明显优于“穿障直线”，且不深入障碍内部。
  double min_esdf = 1.0e9;
  double max_vel = 0.0;
  for (const auto & p : trajectory_samples) {
    min_esdf = std::min(min_esdf, esdf.query_distance(p.x, p.y));
    max_vel = std::max(max_vel, std::hypot(p.vx, p.vy));
  }
  // 直线穿障会得到约 -0.3m 的负距离；优化后应为正且接近安全距离量级。
  assert(min_esdf > 0.0);
  // 5) 速度软约束生效：最大速度不应大幅超过上限。
  assert(max_vel <= config.max_velocity * 1.25);
}

void test_minco_continuity()
{
  const std::vector<astra_nav::Point2D> points{{0.0, 0.0}, {0.8, 0.35}, {1.6, 0.0}};
  const auto times = astra_nav::cumulative_times(points, 1.2, 1.4);
  const auto trajectory = astra_nav::MincoTrajectory::fit(points, times);
  assert(!trajectory.empty());

  const auto start = trajectory.evaluate(0.0);
  const auto end = trajectory.evaluate(trajectory.duration());
  assert(std::abs(start.x - points.front().x) < 1e-6);
  assert(std::abs(start.y - points.front().y) < 1e-6);
  assert(std::abs(end.x - points.back().x) < 1e-6);
  assert(std::abs(end.y - points.back().y) < 1e-6);

  const double joint_t = times[1];
  const auto left = trajectory.evaluate(joint_t - 1e-5);
  const auto right = trajectory.evaluate(joint_t + 1e-5);
  assert(std::hypot(left.x - right.x, left.y - right.y) < 1e-3);
  assert(std::hypot(left.vx - right.vx, left.vy - right.vy) < 1e-3);
  assert(std::hypot(left.ax - right.ax, left.ay - right.ay) < 1e-2);

  astra_nav::MincoBoundaryState head;
  head.position = points.front();
  head.velocity = {0.35, 0.12};
  head.acceleration = {0.02, -0.01};
  astra_nav::MincoBoundaryState tail;
  tail.position = points.back();
  tail.velocity = {0.0, 0.0};
  tail.acceleration = {0.0, 0.0};
  const auto inherited = astra_nav::MincoTrajectory::fit(points, times, head, tail);
  const auto inherited_start = inherited.evaluate(0.0);
  assert(!inherited.empty());
  assert(std::abs(inherited_start.vx - head.velocity.x) < 1e-6);
  assert(std::abs(inherited_start.vy - head.velocity.y) < 1e-6);
  assert(std::abs(inherited_start.ax - head.acceleration.x) < 1e-6);
  assert(std::abs(inherited_start.ay - head.acceleration.y) < 1e-6);
}

void test_waypoint_lbfgs_optimizer()
{
  astra_nav::Grid2D grid;
  grid.width = 60;
  grid.height = 40;
  grid.resolution = 0.1;
  grid.origin = {-3.0, -2.0};
  grid.data.assign(static_cast<std::size_t>(grid.width * grid.height), 0);
  for (int x = 29; x <= 31; ++x) {
    for (int y = 18; y <= 22; ++y) {
      grid.at(x, y) = kObstacle;
    }
  }
  astra_nav::EsdfMap esdf(grid);
  const std::vector<astra_nav::Point2D> points{
    {-1.2, 0.0}, {-0.45, 0.02}, {-0.05, 0.05}, {0.45, 0.02}, {1.2, 0.0}};

  double before = 100.0;
  for (const auto & point : points) {
    before = std::min(before, esdf.query_distance(point.x, point.y));
  }

  astra_nav::WaypointLbfgsConfig config;
  config.reference_weight = 0.03;
  config.smooth_weight = 0.2;
  config.obstacle_weight = 12.0;
  config.obstacle_margin = 0.55;
  config.max_iterations = 80;
  astra_nav::WaypointLbfgsOptimizer optimizer(config);
  const auto result = optimizer.optimize(points, esdf);

  double after = 100.0;
  for (const auto & point : result.points) {
    after = std::min(after, esdf.query_distance(point.x, point.y));
  }

  if (!(after > before + 0.05)) {
    std::cerr << "L-BFGS 路点优化距离未按预期提升，优化前最小距离：" << before
              << "，优化后最小距离：" << after << "，初始代价：" << result.initial_cost
              << "，最终代价：" << result.final_cost << "，状态：" << result.status
              << "，迭代次数：" << result.iterations << std::endl;
  }
  assert(result.optimized);
  assert(result.final_cost <= result.initial_cost);
  assert(after > before + 0.05);
}

// 验证报告 5.5.4.4 流程图下半“模式选择”：完全/部分/仅优化三种重规划模式的状态机。
// 只验证“在对的条件下进对的分支”这一与参数无关的框架不变量，不验证轨迹性能。
void test_replan_strategy()
{
  // 空旷地图：构造一条上一轨迹用于映射与碰撞检测。
  astra_nav::Grid2D grid;
  grid.width = 100;
  grid.height = 60;
  grid.resolution = 0.1;
  grid.origin = {-1.0, -3.0};
  grid.data.assign(static_cast<std::size_t>(grid.width * grid.height), 0);
  astra_nav::EsdfMap esdf(grid);

  const std::vector<astra_nav::Point2D> wp{{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}, {6.0, 0.0}};
  const auto times = astra_nav::cumulative_times(wp, 1.5, 1.6);
  const auto traj = astra_nav::MincoTrajectory::fit(wp, times);
  assert(!traj.empty());

  astra_nav::ReplanDecisionConfig config;
  config.goal_change_threshold = 0.5;
  config.localization_deviation_threshold = 0.6;
  config.collision_clearance = 0.05;
  config.good_tracking_threshold = 0.25;
  config.partial_enabled = true;
  config.optimize_only_enabled = true;
  astra_nav::ReplanStrategy strategy(config);

  const astra_nav::Point2D goal{6.0, 0.0};
  astra_nav::MincoTrajectory empty_traj;

  // 1) 首次规划（无上一轨迹）：必为完全重规划。
  const auto first = strategy.decide(empty_traj, false, {0.0, 0.0}, goal, goal, false, esdf);
  assert(first.mode == astra_nav::ReplanMode::FullReplan);

  // 2) 目标大幅跳变（上一目标距当前目标 > 阈值）：完全重规划。
  const astra_nav::Point2D far_last_goal{0.0, 0.0};  // 距当前目标 6m，远超 0.5m
  const auto jumped = strategy.decide(traj, true, {0.2, 0.0}, goal, far_last_goal, true, esdf);
  assert(jumped.mode == astra_nav::ReplanMode::FullReplan);
  assert(jumped.goal_change > config.goal_change_threshold);
  assert(!jumped.goal_stable);

  // 3) 机器人在轨迹上、跟踪良好、目标不跳变、前方无碰撞：仅优化。
  const auto on_track = strategy.decide(traj, true, {2.0, 0.02}, goal, goal, true, esdf);
  assert(on_track.mode == astra_nav::ReplanMode::OptimizeOnly);
  assert(on_track.trajectory_collision_free);
  assert(on_track.goal_stable);
  assert(on_track.tracking_good);
  // 映射点应落在轨迹上当前位置附近（x≈2.0）。
  assert(std::abs(on_track.mapped_position.x - 2.0) < 0.2);

  // 4) 定位中等偏离（0.25m < dev < 0.6m，超过跟踪良好阈值但未到完全重规划阈值）：部分重规划。
  const auto partial = strategy.decide(traj, true, {3.0, 0.4}, goal, goal, true, esdf);
  assert(partial.mode == astra_nav::ReplanMode::PartialReplan);
  assert(partial.localization_deviation > config.good_tracking_threshold);
  assert(partial.localization_deviation <= config.localization_deviation_threshold);

  // 5) 定位严重偏离（> 0.6m）：完全重规划。
  const auto deviated = strategy.decide(traj, true, {3.0, 1.5}, goal, goal, true, esdf);
  assert(deviated.mode == astra_nav::ReplanMode::FullReplan);
  assert(deviated.localization_deviation > config.localization_deviation_threshold);

  // 6) 前方轨迹被障碍挡住（无碰撞条件不满足）：即使跟踪良好也不能仅优化，应为部分重规划。
  astra_nav::Grid2D blocked_grid = grid;
  for (int x = 48; x <= 52; ++x) {        // 世界 x≈3.8~4.2，落在轨迹前方
    for (int y = 25; y <= 35; ++y) {      // 世界 y≈-0.5~0.5，盖住轨迹线
      blocked_grid.at(x, y) = kObstacle;
    }
  }
  astra_nav::EsdfMap blocked_esdf(blocked_grid);
  const auto blocked = strategy.decide(traj, true, {2.0, 0.02}, goal, goal, true, blocked_esdf);
  assert(!blocked.trajectory_collision_free);
  assert(blocked.mode == astra_nav::ReplanMode::PartialReplan);

  // 7) 三模式互斥、且都能被触发（覆盖性）：上面已分别触发 Full/Partial/OptimizeOnly。
}

}  // namespace

int main()
{
  test_frontend_and_trajectory();
  test_minco_backend_optimization();
  test_minco_continuity();
  test_waypoint_lbfgs_optimizer();
  test_replan_strategy();
  std::cout << "astra_planning 测试通过。" << std::endl;
  return 0;
}
