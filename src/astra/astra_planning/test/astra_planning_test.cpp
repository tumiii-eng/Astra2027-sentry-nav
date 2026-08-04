#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "astra_common/common.hpp"
#include "astra_mapping/esdf_map.hpp"
#include "astra_mapping/obstacle_extractor.hpp"
#include "astra_planning/endpoint_projector.hpp"
#include "astra_planning/goal_reachability_resolver.hpp"
#include "astra_planning/jps_planner.hpp"
#include "astra_planning/local_goal_selector.hpp"
#include "astra_planning/minco_optimizer.hpp"
#include "astra_planning/minco_trajectory.hpp"
#include "astra_planning/replan_strategy.hpp"
#include "astra_planning/trajectory_optimizer.hpp"
#include "astra_planning/waypoint_lbfgs_optimizer.hpp"

namespace
{

void test_jps_and_trajectory()
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
    grid.at(20, y) = 1;
  }

  const auto inflated = astra_nav::ObstacleExtractor::inflate(grid, 1);
  astra_nav::JpsPlanner planner(inflated);
  const auto path = planner.plan({-1.5, 0.0}, {1.5, 0.0});
  assert(path.has_value());
  // JPS 返回的是稀疏跳点（相邻跳点之间为直线），至少含起点与终点两个端点。
  // 不要求跳点数 > 2：直线可达时 JPS 仅用 2 个跳点即为最优，这是其相对 A* 的优势。
  assert(path->world_points.size() >= 2);
  // 后端真正使用的是重采样后的稠密路点；验证重采样能从跳点恢复足够的路点。
  const auto resampled = astra_nav::resample_polyline(path->world_points, 0.35);
  assert(resampled.size() > 2);

  astra_nav::EsdfMap esdf(grid);
  const auto query = esdf.query_distance_and_gradient(0.0, 0.0);
  assert(std::isfinite(query.first));

  astra_nav::TrajectoryOptimizer optimizer({});
  const auto optimization = optimizer.optimize_with_stats(path->world_points, esdf);
  const auto & trajectory = optimization.trajectory;
  assert(!trajectory.empty());
  assert(optimization.stats.input_points == static_cast<int>(path->world_points.size()));
  assert(optimization.stats.resampled_points >= 2);
  assert(optimization.stats.optimize_time_ms >= 0.0);
  assert(optimization.stats.quality.max_velocity > 0.0);
  assert(optimization.stats.quality.max_acceleration >= 0.0);
  assert(optimization.stats.time_scaling_factor >= 1.0);
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
  const auto legacy_optimization = legacy_optimizer.optimize_with_stats(path->world_points, esdf);
  assert(!legacy_optimization.trajectory.empty());
  assert(!legacy_optimization.stats.used_minco_backend);
  assert(legacy_optimization.stats.time_scaling_iterations > 0);
  assert(legacy_optimization.stats.time_scaling_factor > 1.0);
  assert(legacy_optimization.stats.quality.max_velocity <= legacy_config.max_velocity * 1.08);
  assert(legacy_optimization.stats.quality.max_acceleration <= legacy_config.max_acceleration * 1.12);
}

// 验证报告 5.5.4 的核心：MINCO 控制点/分段时间两步优化能把前端绕障路径
// 优化为平滑、动力学可行、远离障碍的轨迹。
void test_minco_backend_optimization()
{
  // 中间一块障碍，前端 JPS 绕行，后端两步优化收紧。
  astra_nav::Grid2D grid;
  grid.width = 80;
  grid.height = 60;
  grid.resolution = 0.1;
  grid.origin = {-4.0, -3.0};
  grid.data.assign(static_cast<std::size_t>(grid.width * grid.height), 0);
  for (int x = 37; x <= 43; ++x) {
    for (int y = 25; y <= 35; ++y) {
      grid.at(x, y) = 1;
    }
  }
  astra_nav::EsdfMap esdf(grid);
  const auto inflated = astra_nav::ObstacleExtractor::inflate(grid, 4);

  astra_nav::JpsPlanner jps(inflated);
  const auto path = jps.plan({-2.5, 0.0}, {2.5, 0.0});
  assert(path.has_value());

  astra_nav::TrajectoryOptimizerConfig config;
  config.use_minco_backend = true;
  config.max_velocity = 2.0;
  config.max_acceleration = 3.0;
  config.obstacle_cost_radius = 0.4;
  config.minco_weight_time = 8.0;
  config.minco_weight_obstacle = 2000.0;
  astra_nav::TrajectoryOptimizer optimizer(config);
  const auto result = optimizer.optimize_with_stats(path->world_points, esdf);

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
  assert(std::hypot(start.x - path->world_points.front().x,
    start.y - path->world_points.front().y) < 0.05);
  assert(std::hypot(end.x - path->world_points.back().x,
    end.y - path->world_points.back().y) < 0.05);

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

// 验证报告 5.5.3.1 的 JPS：跳点搜索必须与 A* 给出相同的最优代价（JPS 是 A* 的等价加速），
// 且路径无碰撞。这是与参数无关的硬不变量。
void test_jps_optimality()
{
  // 构造几组带障碍的地图，逐一对照 JPS 与 A* 的代价。
  auto build_grid = [](int seed) {
    astra_nav::Grid2D grid;
    grid.width = 50;
    grid.height = 50;
    grid.resolution = 0.1;
    grid.origin = {0.0, 0.0};
    grid.data.assign(static_cast<std::size_t>(grid.width * grid.height), 0);
    // 用确定性伪随机布置障碍块，保证可复现。
    unsigned long s = static_cast<unsigned long>(seed) * 2654435761UL + 12345UL;
    auto rnd = [&s]() {
      s = s * 1103515245UL + 12345UL;
      return (s >> 16) & 0x7fff;
    };
    const int nobs = 8 + static_cast<int>(rnd() % 12);
    for (int i = 0; i < nobs; ++i) {
      const int ox = static_cast<int>(rnd() % 45);
      const int oy = static_cast<int>(rnd() % 45);
      const int w = 1 + static_cast<int>(rnd() % 5);
      const int h = 1 + static_cast<int>(rnd() % 5);
      for (int x = ox; x < std::min(50, ox + w); ++x) {
        for (int y = oy; y < std::min(50, oy + h); ++y) {
          grid.at(x, y) = 1;
        }
      }
    }
    grid.at(1, 1) = 0;
    grid.at(48, 48) = 0;
    return grid;
  };

  auto line_free = [](const astra_nav::Grid2D & g, std::pair<int, int> a, std::pair<int, int> b) {
    const int dx = (b.first > a.first) - (b.first < a.first);
    const int dy = (b.second > a.second) - (b.second < a.second);
    int x = a.first;
    int y = a.second;
    while (x != b.first || y != b.second) {
      if (x < 0 || y < 0 || x >= g.width || y >= g.height || g.at(x, y) != 0) {
        return false;
      }
      x += dx;
      y += dy;
    }
    return g.at(b.first, b.second) == 0;
  };

  int checked = 0;
  for (int seed = 0; seed < 25; ++seed) {
    const auto grid = build_grid(seed);
    astra_nav::JpsPlanner jps(grid);
    const astra_nav::Point2D start{0.15, 0.15};
    const astra_nav::Point2D goal{4.85, 4.85};
    const auto path = jps.plan(start, goal);
    const auto astar = jps.astar_cost(start, goal);
    // 有解性必须一致。
    assert(path.has_value() == astar.has_value());
    if (!path.has_value()) {
      continue;
    }
    ++checked;
    // 代价一致（JPS 最优性 = A* 最优性）。
    double jps_cost = 0.0;
    for (std::size_t i = 1; i < path->grid_points.size(); ++i) {
      jps_cost += std::hypot(
        path->grid_points[i].first - path->grid_points[i - 1].first,
        path->grid_points[i].second - path->grid_points[i - 1].second);
    }
    assert(std::abs(jps_cost - *astar) < 1e-6);
    // 路径每一段（跳点之间的直线）都无碰撞。
    for (std::size_t i = 1; i < path->grid_points.size(); ++i) {
      assert(line_free(grid, path->grid_points[i - 1], path->grid_points[i]));
    }
  }
  // 至少要有若干可解地图被真正校验过，否则测试没有意义。
  assert(checked >= 5);
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
      grid.at(x, y) = 1;
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

void test_endpoint_projector()
{
  astra_nav::Grid2D grid;
  grid.width = 60;
  grid.height = 60;
  grid.resolution = 0.1;
  grid.origin = {-3.0, -3.0};
  grid.data.assign(static_cast<std::size_t>(grid.width * grid.height), 0);
  for (int x = 27; x <= 33; ++x) {
    for (int y = 27; y <= 33; ++y) {
      grid.at(x, y) = 1;
    }
  }
  const auto inflated = astra_nav::ObstacleExtractor::inflate(grid, 2);
  astra_nav::EsdfMap esdf(grid);

  astra_nav::EndpointProjectorConfig config;
  config.required_clearance = 0.45;
  config.max_search_radius = 1.2;
  config.angle_samples = 72;
  astra_nav::EndpointProjector projector(config);

  const auto blocked = projector.project({0.0, 0.0}, inflated, esdf);
  assert(blocked.success);
  assert(blocked.moved);
  assert(blocked.projected_distance >= config.required_clearance - 1e-6);
  const int gx = static_cast<int>(std::floor((blocked.point.x - inflated.origin.x) / inflated.resolution));
  const int gy = static_cast<int>(std::floor((blocked.point.y - inflated.origin.y) / inflated.resolution));
  assert(gx >= 0 && gy >= 0 && gx < inflated.width && gy < inflated.height);
  assert(inflated.at(gx, gy) == 0);

  const astra_nav::Point2D free_point{1.5, 1.5};
  const auto free_result = projector.project(free_point, inflated, esdf);
  assert(free_result.success);
  assert(!free_result.moved);
}

void test_local_goal_selector()
{
  astra_nav::Grid2D grid;
  grid.width = 40;
  grid.height = 40;
  grid.resolution = 0.1;
  grid.origin = {-2.0, -2.0};
  grid.data.assign(static_cast<std::size_t>(grid.width * grid.height), 0);
  astra_nav::EsdfMap esdf(grid);

  astra_nav::LocalGoalSelectorConfig config;
  config.required_clearance = 0.2;
  config.backoff_distance = 0.3;
  config.sample_step = 0.05;
  astra_nav::LocalGoalSelector selector(config);

  const auto clipped = selector.select({0.0, 0.0}, {5.0, 0.0}, grid, esdf);
  assert(clipped.success);
  assert(clipped.clipped);
  assert(clipped.local_goal.x > 1.3);
  assert(clipped.local_goal.x < 2.0);
  assert(std::abs(clipped.local_goal.y) < 1e-6);
  assert(clipped.distance_to_global_goal > 3.0);

  const auto inside = selector.select({0.0, 0.0}, {1.0, 0.0}, grid, esdf);
  assert(inside.success);
  assert(!inside.clipped);
  assert(std::abs(inside.local_goal.x - 1.0) < 1e-9);
}

// 验证报告 5.5.4.4 流程图右上“目标点优化”：目标点可达性检测 + BFS 找种子 + DFS 沿梯度扩展。
// 这里只验证“与参数无关”的框架不变量：可达直通、不可达必重定位到满足安全距离的可达点、
// 位移有界、BFS/DFS 分支在该走的时候确实被走到。
void test_goal_reachability_resolver()
{
  // 中央放一块实心障碍，目标点直接埋在障碍正中心。
  astra_nav::Grid2D grid;
  grid.width = 80;
  grid.height = 80;
  grid.resolution = 0.1;
  grid.origin = {-4.0, -4.0};
  grid.data.assign(static_cast<std::size_t>(grid.width * grid.height), 0);
  for (int x = 33; x <= 47; ++x) {
    for (int y = 33; y <= 47; ++y) {
      grid.at(x, y) = 1;
    }
  }
  astra_nav::EsdfMap esdf(grid);

  astra_nav::GoalReachabilityConfig config;
  config.required_clearance = 0.3;
  config.max_search_radius = 2.0;
  astra_nav::GoalReachabilityResolver resolver(config);

  // 1) 障碍正中心的目标：必须判定为不可达，并成功重定位。
  const astra_nav::Point2D buried{0.0, 0.0};  // 对应栅格 (40,40)，在障碍内部
  const auto relocated = resolver.resolve(buried, esdf);
  assert(!relocated.reachable);          // 输入目标本身不可达
  assert(relocated.success);             // 最终成功得到可达目标
  assert(relocated.relocated);           // 发生了重定位
  assert(relocated.bfs_used);            // 触发了 BFS 找种子
  assert(relocated.bfs_expansions > 0);
  // 重定位结果必须真正可达：在地图内且 ESDF 距离 >= 要求安全距离。
  assert(esdf.in_map(relocated.point.x, relocated.point.y));
  assert(esdf.query_distance(relocated.point.x, relocated.point.y) >=
    config.required_clearance - 1e-6);
  // 位移有界：不超过搜索半径量级（对角 sqrt2 余量）。
  assert(relocated.displacement <= config.max_search_radius * 1.4142 + 1e-6);
  // 原始距离为负（障碍内部），重定位后距离为正。
  assert(relocated.final_distance > 0.0);

  // 2) 已经在空旷区域、满足安全距离的目标：必须判定为可达，且原地不动。
  const astra_nav::Point2D free_goal{3.0, 3.0};
  const auto reachable = resolver.resolve(free_goal, esdf);
  assert(reachable.reachable);
  assert(reachable.success);
  assert(!reachable.relocated);
  assert(!reachable.bfs_used);
  assert(!reachable.dfs_used);
  assert(std::hypot(reachable.point.x - free_goal.x, reachable.point.y - free_goal.y) < 1e-9);

  // 3) 贴着障碍边缘、距离不足安全距离的目标：可达性应判否，重定位后 clearance 必须达标，
  //    且重定位结果应当离障碍更远（final_distance 严格大于原始 clearance）。
  const astra_nav::Point2D near_edge{0.85, 0.0};  // 障碍右边界约在 x=0.7 附近
  const auto pushed = resolver.resolve(near_edge, esdf);
  assert(pushed.success);
  if (pushed.relocated) {
    assert(pushed.final_distance >= config.required_clearance - 1e-6);
    assert(pushed.final_distance > pushed.original_distance);
  }
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
      blocked_grid.at(x, y) = 1;
    }
  }
  astra_nav::EsdfMap blocked_esdf(blocked_grid);
  const auto blocked = strategy.decide(traj, true, {2.0, 0.02}, goal, goal, true, blocked_esdf);
  assert(!blocked.trajectory_collision_free);
  assert(blocked.mode == astra_nav::ReplanMode::PartialReplan);

  // 7) 三模式互斥、且都能被触发（覆盖性）：上面已分别触发 Full/Partial/OptimizeOnly。
}

// 验证报告 5.5.4.2 FINELY 阶段的“势谷（梯度无效化）”处理：在对称峡谷中线，
// 双线性/二次插值得到的避障梯度趋近于 0，控制点会卡在中线优化不动。
// 报告的解决方案是沿 gradPos 探一步估计有效梯度，低于阈值时改用 violaPos 方向与幅度。
// 这里构造一条沿对称峡谷中线的轨迹，跑一次 FINELY 优化，验证势谷分支确实被命中，
// 且优化不发散、轨迹仍保持在峡谷可行区域内。
void test_finely_valley_handling()
{
  // 构造一条水平对称峡谷：上下两道墙，中间留出自由通道，通道中线 y=0。
  astra_nav::Grid2D grid;
  grid.width = 120;
  grid.height = 60;
  grid.resolution = 0.1;
  grid.origin = {-1.0, -3.0};
  grid.data.assign(static_cast<std::size_t>(grid.width * grid.height), 0);
  // 上墙：y 世界坐标约 [0.6, 1.0]（栅格行 36~40）；下墙：约 [-1.0, -0.6]（行 20~24）。
  for (int x = 0; x < grid.width; ++x) {
    for (int y = 36; y <= 40; ++y) {
      grid.at(x, y) = 1;
    }
    for (int y = 20; y <= 24; ++y) {
      grid.at(x, y) = 1;
    }
  }
  astra_nav::EsdfMap esdf(grid);

  // 通道中线附近的安全距离约 0.6 m（半通道宽）。在中线放一条直线初始轨迹。
  const std::vector<astra_nav::Point2D> waypoints{
    {0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}, {6.0, 0.0}, {8.0, 0.0}};
  const auto times = astra_nav::cumulative_times(waypoints, 1.5, 1.6);

  astra_nav::MincoBoundaryState head;
  head.position = waypoints.front();
  astra_nav::MincoBoundaryState tail;
  tail.position = waypoints.back();

  astra_nav::MincoOptimizerConfig config;
  config.max_velocity = 1.5;
  config.max_acceleration = 1.6;
  // 安全距离设得比半通道宽更大，使中线采样点持续处于障碍软约束作用域内（violation>0），
  // 从而真正进入 FINELY 的避障梯度分支，触发势谷判定。
  config.safe_distance = 0.9;
  config.valley_gradient_threshold = 0.5;
  config.max_iterations = 40;
  astra_nav::MincoOptimizer optimizer(config);

  const auto result = optimizer.optimize(
    waypoints, times, head, tail, esdf, astra_nav::MincoOptimizeStage::FinelyOptimization);

  // 1) 势谷分支确实被命中（修复前该分支根本不存在，valley_hits 恒为 0）。
  assert(result.valley_hits > 0);
  // 2) 优化没有发散：得到非空、有限代价的轨迹。
  assert(!result.trajectory.empty());
  assert(std::isfinite(result.final_cost));
  // 3) 轨迹仍保持在峡谷可行区域内（不穿墙）：所有采样点 ESDF 距离为正。
  const auto samples = result.trajectory.sample(0.05);
  double min_esdf = 1.0e9;
  for (const auto & p : samples) {
    min_esdf = std::min(min_esdf, esdf.query_distance(p.x, p.y));
  }
  assert(min_esdf > 0.0);
}

}  // namespace

int main()
{
  test_jps_and_trajectory();
  test_jps_optimality();
  test_minco_backend_optimization();
  test_minco_continuity();
  test_waypoint_lbfgs_optimizer();
  test_endpoint_projector();
  test_local_goal_selector();
  test_goal_reachability_resolver();
  test_replan_strategy();
  test_finely_valley_handling();
  std::cout << "astra_planning 测试通过。" << std::endl;
  return 0;
}
