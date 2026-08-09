#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

#include "astra_common/common.hpp"
#include "astra_mapping/esdf_map.hpp"
#include "astra_mapping/obstacle_extractor.hpp"
#include "astra_planning/endpoint_nudge.hpp"
#include "astra_planning/replan_strategy.hpp"
#include "astra_planning/spatial_grid_astar.hpp"
#include "astra_planning/trajectory_optimizer.hpp"

namespace astra_nav
{

// L-BFGS 退出状态码转中文说明。达迭代上限(-1008)按报告 5.5.4.1 是长轨迹的正常终止方式,
// 非错误;短/易轨迹经梯度或代价停滞判据正常收敛。
static const char * lbfgs_status_text(int status)
{
  switch (status) {
    case 0:      return "收敛(梯度达标)";        // LBFGS_CONVERGENCE
    case 1:      return "停止(代价停滞达标)";    // LBFGS_STOP
    case -1008:  return "达迭代上限正常退出";    // LBFGSERR_MAXIMUMITERATION
    default:     return "其它";
  }
}

class PlannerNode : public rclcpp::Node
{
public:
  PlannerNode()
  : Node("planner_node"), optimizer_(optimizer_config_)
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/astra/odom");
    obstacle_grid_topic_ =
      declare_parameter<std::string>("obstacle_grid_topic", "/astra/obstacle_grid");
    // 动态障碍层（可为空字符串=不订阅，此时只用静态层）。两层分开进来才能各自膨胀。
    dynamic_grid_topic_ = declare_parameter<std::string>("dynamic_grid_topic", "");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
    path_topic_ = declare_parameter<std::string>("path_topic", "/astra/trajectory");
    map_frame_ = declare_parameter<std::string>("map_frame", "odom");
    // 连续代价场膨胀参数（对齐 HWSentryNav26 map_server global_map.inflation）。
    // full_cost_radius_m 是硬阻断半径，应覆盖车体半径；cutoff_radius_m 之内为软代价，
    // 只加权边代价、为下游梯度优化器提供远处可见的连续约束，不阻断连通性。
    inflation_params_.full_cost_radius_m = declare_parameter("inflation.full_cost_radius_m", 0.1);
    inflation_params_.cutoff_radius_m = declare_parameter("inflation.cutoff_radius_m", 0.3);
    inflation_params_.decay_rate_per_m = declare_parameter("inflation.decay_rate_per_m", 24.0);
    // 动态障碍膨胀参数独立于静态（对齐 HWSentryNav26 map_server local_map.inflation）。
    // 动态障碍位置有观测噪声、且会移动，需要更大的满代价半径与更缓的衰减；静态先验
    // 地图是精确的，用更紧的膨胀以免切断窄走廊连通性。
    dynamic_inflation_params_.full_cost_radius_m =
      declare_parameter("dynamic_inflation.full_cost_radius_m", 0.2);
    dynamic_inflation_params_.cutoff_radius_m =
      declare_parameter("dynamic_inflation.cutoff_radius_m", 0.4);
    dynamic_inflation_params_.decay_rate_per_m =
      declare_parameter("dynamic_inflation.decay_rate_per_m", 16.0);
    replan_rate_hz_ = declare_parameter("replan_rate_hz", 5.0);
    // 端点 BFS 重定位半径（对齐 HWSentryNav26 path_planner endpoint.nudge_max_distance）。
    endpoint_nudge_max_distance_ = declare_parameter("endpoint_nudge_max_distance", 1.0);
    // 空间 A* 参数（对齐 HWSentryNav26 path_planner spatial_a_star）。
    astar_obstacle_weight_ = declare_parameter("spatial_a_star.obstacle_weight", 1.0);
    astar_max_expansions_ = declare_parameter("spatial_a_star.max_expansions", 1000000);
    replan_start_position_inheritance_enabled_ =
      declare_parameter("replan_start_position_inheritance_enabled", true);
    replan_start_max_position_error_ = declare_parameter("replan_start_max_position_error", 0.18);
    replan_start_transition_enabled_ = declare_parameter("replan_start_transition_enabled", true);
    replan_start_transition_max_position_error_ =
      declare_parameter("replan_start_transition_max_position_error", 0.36);
    short_path_hold_distance_ = declare_parameter("short_path_hold_distance", 0.25);
    start_projection_escape_enabled_ = declare_parameter("start_projection_escape_enabled", true);
    start_projection_escape_min_distance_ =
      declare_parameter("start_projection_escape_min_distance", 0.28);
    start_projection_escape_speed_ = declare_parameter("start_projection_escape_speed", 0.25);

    // 重规划三策略参数（报告 5.5.4.4）。
    replan_strategy_enabled_ = declare_parameter("replan_strategy_enabled", true);
    replan_goal_change_threshold_ = declare_parameter("replan_goal_change_threshold", 0.5);
    replan_optimize_only_enabled_ = declare_parameter("replan_optimize_only_enabled", true);
    replan_partial_enabled_ = declare_parameter("replan_partial_enabled", true);
    replan_collision_clearance_ = declare_parameter("replan_collision_clearance", 0.05);
    replan_localization_deviation_threshold_ =
      declare_parameter("replan_localization_deviation_threshold", 0.6);
    replan_good_tracking_threshold_ = declare_parameter("replan_good_tracking_threshold", 0.25);

    // BUG-4：把前端真正搜索用的代价图发布出来（RViz 里 /astra/global_obstacle_grid 不是搜索图）。
    publish_inflated_grid_ = declare_parameter("publish_inflated_grid", false);
    inflated_grid_topic_ =
      declare_parameter<std::string>("inflated_grid_topic", "/astra/planner_inflated_grid");

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 5);
    if (publish_inflated_grid_) {
      inflated_grid_pub_ =
        create_publisher<nav_msgs::msg::OccupancyGrid>(inflated_grid_topic_, 1);
    }
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        pose_.x = msg->pose.pose.position.x;
        pose_.y = msg->pose.pose.position.y;
        pose_.yaw = yaw_from_quaternion(msg->pose.pose.orientation);
        has_odom_ = true;
      });
    grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      obstacle_grid_topic_, 5, [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        latest_grid_ = *msg;
        has_grid_ = true;
      });
    if (!dynamic_grid_topic_.empty()) {
      dynamic_grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        dynamic_grid_topic_, 5, [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          latest_dynamic_grid_ = *msg;
          has_dynamic_grid_ = true;
        });
    }
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 5, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_.x = msg->pose.position.x;
        goal_.y = msg->pose.position.y;
        has_goal_ = true;
      });

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(replan_rate_hz_, 1e-3))),
      std::bind(&PlannerNode::on_timer, this));

    RCLCPP_INFO(get_logger(), "规划节点已启动，等待里程计、障碍图和目标点。");
  }

private:
  TrajectoryOptimizerConfig make_optimizer_config()
  {
    TrajectoryOptimizerConfig config;
    config.max_velocity = declare_parameter("max_velocity", 1.5);
    config.max_acceleration = declare_parameter("max_acceleration", 1.6);
    config.waypoint_spacing = declare_parameter("waypoint_spacing", 0.35);
    config.obstacle_cost_radius = declare_parameter("obstacle_cost_radius", 0.55);
    config.dynamic_time_scaling_enabled =
      declare_parameter("dynamic_time_scaling_enabled", true);
    config.dynamic_time_scaling_max_iterations =
      declare_parameter("dynamic_time_scaling_max_iterations", 4);
    config.dynamic_time_scaling_safety_ratio =
      declare_parameter("dynamic_time_scaling_safety_ratio", 1.03);
    // 含转角的时间分配权重（报告 5.5.3.2 思路一的 k_turn）：影响初始时间分配，折角处更慢。
    config.turn_time_weight = declare_parameter("turn_time_weight", 0.25);
    // —— MINCO 两步后端优化权重（报告 5.5.4.2）——
    // 这些权重决定轨迹“形状”；轨迹“速度上限”由 max_velocity / max_acceleration
    // 经时间缩放生效。全部暴露为参数，调速度/形状无需改代码。
    config.minco_weight_energy = declare_parameter("minco_weight_energy", 1.0);
    config.minco_weight_time = declare_parameter("minco_weight_time", 16.0);
    config.minco_weight_obstacle = declare_parameter("minco_weight_obstacle", 2000.0);
    config.minco_weight_velocity = declare_parameter("minco_weight_velocity", 200.0);
    config.minco_weight_acceleration = declare_parameter("minco_weight_acceleration", 200.0);
    config.minco_weight_uniform_time = declare_parameter("minco_weight_uniform_time", 120.0);
    config.minco_pre_iterations = declare_parameter("minco_pre_iterations", 60);
    config.minco_fine_iterations = declare_parameter("minco_fine_iterations", 60);
    // L-BFGS 退出判据(暴露为参数,便于按报告 5.5.4.1 调整)。g_epsilon 门控梯度收敛退出;
    // delta 门控代价停滞退出 + line search 早停(报告参考 DDR-OPT 的"放松最终条件")。
    config.minco_g_epsilon = declare_parameter("minco_g_epsilon", 1.0e-5);
    config.lbfgs_delta = declare_parameter("lbfgs_delta", 1.0e-5);
    // 回环检测（轨迹中段甩圈/绕圈自诊断与自动恢复），参数可纯 yaml 调。
    config.loop_detection_enabled = declare_parameter("loop_detection_enabled", true);
    config.loop_detection_window = declare_parameter("loop_detection_window", 1.2);
    config.loop_detection_angle_threshold =
      declare_parameter("loop_detection_angle_threshold", 4.71);
    // 方向感知起点继承（剔除冲过拐弯点后的逆向继承速度，避免甩圈），可纯 yaml 调。
    config.directional_start_inheritance =
      declare_parameter("directional_start_inheritance", true);
    return config;
  }

  void on_timer()
  {
    if (!has_odom_ || !has_grid_ || !has_goal_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "规划节点等待输入：里程计=%s，障碍图=%s，目标点=%s。",
        has_odom_ ? "已收到" : "未收到",
        has_grid_ ? "已收到" : "未收到",
        has_goal_ ? "已收到" : "未收到");
      return;
    }

    // 静态层与动态层各按自己的参数膨胀一次，再逐格取 max 合并成一张硬代价场。
    // 对齐 HWSentryNav26 obstacle_semantics 的 hard_cost = global_static->merge(dynamic_union)，
    // merge 就是逐格 std::max。
    const auto grid = occupancy_msg_to_grid(latest_grid_);
    auto inflated = ObstacleExtractor::inflate(grid, inflation_params_);
    if (has_dynamic_grid_) {
      const auto dynamic_grid = occupancy_msg_to_grid(latest_dynamic_grid_);
      if (dynamic_grid.width == inflated.width && dynamic_grid.height == inflated.height) {
        const auto dynamic_inflated =
          ObstacleExtractor::inflate(dynamic_grid, dynamic_inflation_params_);
        for (std::size_t i = 0; i < inflated.data.size(); ++i) {
          inflated.data[i] = std::max(inflated.data[i], dynamic_inflated.data[i]);
        }
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "动态障碍层几何与静态层不一致，本轮忽略动态层：静态=%dx%d，动态=%dx%d。",
          inflated.width, inflated.height, dynamic_grid.width, dynamic_grid.height);
      }
    }
    if (publish_inflated_grid_ && inflated_grid_pub_) {
      publish_grid_as_occupancy(inflated);
    }
    // ESDF 只用于重规划模式选择与轨迹质量统计；障碍源取合并后的硬占用格。
    EsdfMap esdf(inflated);
    const auto planning_stamp = now();
    const Point2D current_position{pose_.x, pose_.y};

    // 重规划模式选择（报告 5.5.4.4 流程图下半“模式选择”）：根据是否有上一轨迹、
    // 目标是否大幅跳变、定位是否严重偏离、前方轨迹是否无碰撞、跟踪是否良好，
    // 在“完全重规划 / 部分重规划 / 仅优化”三种模式之间选择。
    const auto replan_decision = decide_replan_mode(current_position, esdf);
    const bool allow_inheritance =
      !replan_strategy_enabled_ || replan_decision.mode != ReplanMode::FullReplan;
    log_replan_mode(replan_decision);

    const auto replan_start = select_replan_start(
      current_position, planning_stamp, esdf, allow_inheritance);
    Point2D plan_start = replan_start.point;
    // 起点端点门控（对齐 HWSentryNav26 path_planner::plan 的起点检查 + nudge）：
    // 只做“地图足迹内 + BFS 推到最近硬通行格”，不再按 ESDF 安全距离做射线投影。
    // 硬安全边界已经由连续代价场的 full_cost_radius_m 承担，端点只需满足
    // A* 的前置条件 grid_cell_traversable，其余贴障惩罚交给软代价与优化器。
    const auto nudged_start = nudge_point_to_free(inflated, plan_start, endpoint_nudge_max_distance_);
    if (!nudged_start) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "规划起点无法推到可行格，暂不规划：起点=(%.3f, %.3f)，搜索半径=%.3f m。",
        plan_start.x, plan_start.y, endpoint_nudge_max_distance_);
      return;
    }
    const double start_nudge_displacement = distance_2d(plan_start, *nudged_start);
    const bool start_nudged = start_nudge_displacement > 1.0e-6;
    if (start_nudged) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "规划起点已推到可行格：来源=%s，位移=%.3f m，(%.3f, %.3f)->(%.3f, %.3f)。",
        replan_start_kind_name(replan_start.kind), start_nudge_displacement,
        plan_start.x, plan_start.y, nudged_start->x, nudged_start->y);
    }
    plan_start = *nudged_start;
    if (should_publish_start_escape(current_position, plan_start, start_nudged)) {
      publish_start_escape_path(current_position, plan_start, planning_stamp, "规划起点脱困位移较大");
      return;
    }

    // 目标端点门控（对齐 HWSentryNav26 path_planner::plan 的终点检查 + nudge）：
    // 与起点同一套 BFS，不再有 BFS 种子 + DFS 梯度爬升的可达性重定位，
    // 也不再把全局目标裁剪成局部子目标——A* 在连续代价场上直接搜到全局目标，
    // 搜不通才是真的不通，此时如实报错而不是先自行缩短目标。
    Point2D plan_goal = goal_;
    const auto nudged_goal = nudge_point_to_free(inflated, plan_goal, endpoint_nudge_max_distance_);
    if (!nudged_goal) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "规划目标点无法推到可行格，暂不规划：目标=(%.3f, %.3f)，搜索半径=%.3f m。",
        plan_goal.x, plan_goal.y, endpoint_nudge_max_distance_);
      return;
    }
    if (distance_2d(plan_goal, *nudged_goal) > 1.0e-6) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "规划目标点已推到可行格：位移=%.3f m，(%.3f, %.3f)->(%.3f, %.3f)。",
        distance_2d(plan_goal, *nudged_goal),
        plan_goal.x, plan_goal.y, nudged_goal->x, nudged_goal->y);
    }
    plan_goal = *nudged_goal;

    if (distance_2d(plan_start, plan_goal) <= short_path_hold_distance_) {
      publish_hold_path(plan_goal, planning_stamp, "规划起点与目标点距离过近");
      return;
    }

    // 三种重规划模式构造优化器输入路点的方式不同（报告 5.5.4.4）：
    //   - 仅优化：不重新搜索，直接取上一轨迹从映射点到末端的前方部分等间距重采样；
    //   - 完全/部分重规划：空间 A* 前端搜索（完全=当前位置->目标，部分=映射点->目标）。
    // 三者随后都汇入相同的 MINCO 两步优化与发布流程。
    std::vector<Point2D> planning_points;
    const bool optimize_only =
      replan_strategy_enabled_ && replan_decision.mode == ReplanMode::OptimizeOnly &&
      has_previous_trajectory_ && !previous_trajectory_.empty();
    if (optimize_only) {
      planning_points = resample_previous_forward(replan_decision.mapped_t, plan_goal);
      if (planning_points.size() < 2) {
        // 仅优化无法构造足够路点时，退化到重新搜索兜底。
        planning_points.clear();
      }
    }

    if (planning_points.empty()) {
      SpatialGridAstar::Params astar_params;
      astar_params.obstacle_weight = astar_obstacle_weight_;
      astar_params.max_expansions = astar_max_expansions_;
      SpatialGridAstar astar(inflated, astar_params);
      const auto astar_result = astar.plan(plan_start, plan_goal);
      if (!astar_result.success) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "空间 A* 未找到可行路径，保持上一条轨迹：原因=%s，扩展=%d，open峰值=%zu。",
          astar_result.error.c_str(), astar_result.route.expansions, astar_result.route.open_peak);
        return;
      }
      planning_points =
        restore_planning_endpoints(astar_result.route.world_points, plan_start, plan_goal);
      if (planning_points.size() < 2) {
        if (distance_2d(plan_start, plan_goal) <= 2.0 * short_path_hold_distance_) {
          publish_hold_path(plan_goal, planning_stamp, "空间 A* 返回近距离短路径");
          return;
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "空间 A* 路径点数量不足，暂不发布轨迹。");
        return;
      }
    }

    const auto publish_stamp = planning_stamp;
    const auto boundary_condition = make_boundary_condition(publish_stamp);
    const auto result =
      optimizer_.optimize_with_stats(planning_points, inflated, esdf, boundary_condition);
    const auto samples = result.trajectory.sample(0.08);
    if (samples.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "轨迹优化输出为空，暂不发布轨迹。");
      return;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "轨迹优化统计：输入点=%d，重采样点=%d，耗时=%.2f ms，预优化=%s(%d)/%d次，精优化=%s(%d)/%d次，时间缩放=%d次/%.3f倍，最小障碍距离=%.3f m，最大速度=%.3f m/s，速度超限=%.3f m/s，最大加速度=%.3f m/s^2，加速度超限=%.3f m/s^2。",
      result.stats.input_points, result.stats.resampled_points, result.stats.optimize_time_ms,
      lbfgs_status_text(result.stats.pre_lbfgs_status),
      result.stats.pre_lbfgs_status, result.stats.pre_lbfgs_iterations,
      lbfgs_status_text(result.stats.fine_lbfgs_status),
      result.stats.fine_lbfgs_status, result.stats.fine_lbfgs_iterations,
      result.stats.time_scaling_iterations, result.stats.time_scaling_factor,
      result.stats.quality.min_esdf_distance, result.stats.quality.max_velocity,
      result.stats.quality.velocity_violation, result.stats.quality.max_acceleration,
      result.stats.quality.acceleration_violation);
    if (result.stats.start_state_inherited) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "重规划起点继承上一轨迹状态：速度=(%.3f, %.3f) m/s，加速度=(%.3f, %.3f) m/s^2。",
        boundary_condition.start_velocity.x, boundary_condition.start_velocity.y,
        boundary_condition.start_acceleration.x, boundary_condition.start_acceleration.y);
    }

    // 回环诊断（轨迹中段甩圈/绕圈自诊断与自动恢复）：
    // 把最大滑窗净转角、是否仍含环、是否已自动恢复、回环大致位置打出来，
    // 便于定位“中段突然绕一圈”的根因与确认修复效果。
    if (result.stats.loop_recovered || result.stats.loop_detected ||
      result.stats.loop_max_window_angle > 3.14159)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "轨迹回环诊断：最大滑窗净转角=%.1f 度，仍含回环=%s，已自动恢复=%s，回环位置=(%.3f, %.3f)，"
        "当前重规划模式继承起点=%s，时间缩放=%d次。",
        result.stats.loop_max_window_angle * 180.0 / M_PI,
        result.stats.loop_detected ? "是" : "否",
        result.stats.loop_recovered ? "是" : "否",
        result.stats.loop_position_x, result.stats.loop_position_y,
        result.stats.start_state_inherited ? "是" : "否",
        result.stats.time_scaling_iterations);
    }

    log_replan_continuity(result.trajectory, publish_stamp);

    // 发布前回环截断安全网（报告 5.5.4 轨迹应单调前进，不应自绕）：
    // MINCO 在比安全间隙更窄的缝隙里偶尔把控制点对挤翻折出一个圈，且 PRE/FINELY 两步
    // 都含环、无法靠回退消除。这里在【发布】这一步兜底：扫描即将下发的采样轨迹，若某段
    // 出现绕圈，就在环开始前把轨迹截断再发。机器人只执行近端干净段，远端靠 5Hz 重规划
    // 自然延伸；不碰优化器内部，风险最低。仅当本轮诊断到残留环时才扫描，干净轨迹零改动。
    std::vector<TrajectoryPoint> published = samples;
    if (result.stats.loop_detected) {
      const std::size_t keep = loop_free_prefix_length(published);
      if (keep < published.size()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "发布前回环截断：原采样点=%zu，截断保留=%zu（在环开始前截断，机器人只执行干净前缀）。",
          published.size(), keep);
        published.resize(keep);
      }
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = publish_stamp;
    path_msg.header.frame_id = map_frame_;
    path_msg.poses.reserve(published.size());
    for (const auto & sample : published) {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header = path_msg.header;
      pose_msg.header.stamp = stamp_with_offset(publish_stamp, sample.t);
      pose_msg.pose.position.x = sample.x;
      pose_msg.pose.position.y = sample.y;
      pose_msg.pose.position.z = 0.0;
      pose_msg.pose.orientation = quaternion_from_yaw(sample.yaw);
      path_msg.poses.push_back(pose_msg);
    }
    path_pub_->publish(path_msg);
    previous_trajectory_ = result.trajectory;
    previous_trajectory_stamp_ = publish_stamp;
    has_previous_trajectory_ = true;
    // 记录本轮所用全局目标，供下一轮判断目标是否大幅跳变（报告 5.5.4.4 完全重规划触发条件）。
    last_planned_goal_ = goal_;
    has_last_planned_goal_ = true;
  }

  // 发布前回环截断：扫描采样轨迹，返回“环开始前”应保留的采样点数（无环则返回全长）。
  // 与优化器 detect_loop 同口径：沿弧长滑窗累计【不解缠】航向变化，窗口净转角超阈值即判环，
  // 在该窗口起点截断。机器人只执行干净前缀，远端靠 5Hz 重规划自然延伸。
  std::size_t loop_free_prefix_length(const std::vector<TrajectoryPoint> & samples) const
  {
    const std::size_t n = samples.size();
    if (n < 3) {
      return n;
    }
    const double window = std::max(0.1, optimizer_config_.loop_detection_window);
    const double thr = optimizer_config_.loop_detection_angle_threshold;
    // 逐段航向、弧长，以及该段起点在 samples 中的下标。
    std::vector<double> seg_heading;
    std::vector<double> seg_len;
    std::vector<std::size_t> seg_start_idx;
    seg_heading.reserve(n);
    seg_len.reserve(n);
    seg_start_idx.reserve(n);
    for (std::size_t i = 1; i < n; ++i) {
      const double dx = samples[i].x - samples[i - 1].x;
      const double dy = samples[i].y - samples[i - 1].y;
      const double len = std::hypot(dx, dy);
      if (len < 1.0e-6) {
        continue;
      }
      seg_heading.push_back(std::atan2(dy, dx));
      seg_len.push_back(len);
      seg_start_idx.push_back(i - 1);
    }
    if (seg_heading.size() < 2) {
      return n;
    }
    std::vector<double> dtheta(seg_heading.size(), 0.0);
    for (std::size_t i = 1; i < seg_heading.size(); ++i) {
      dtheta[i] = normalize_angle(seg_heading[i] - seg_heading[i - 1]);
    }
    // 找到最早触发回环的滑窗，在其起点截断。
    for (std::size_t i = 1; i < dtheta.size(); ++i) {
      double acc = 0.0;
      double arc = 0.0;
      for (std::size_t j = i; j < dtheta.size() && arc < window; ++j) {
        acc += dtheta[j];
        arc += seg_len[j];
        if (std::abs(acc) > thr) {
          // 环从窗口起点 i 开始：保留 [0, seg_start_idx[i]]，至少保留 2 点。
          return std::max<std::size_t>(2, seg_start_idx[i] + 1);
        }
      }
    }
    return n;
  }

  void publish_hold_path(
    const Point2D & hold_point, const rclcpp::Time & publish_stamp, const char * reason)
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = publish_stamp;
    path_msg.header.frame_id = map_frame_;
    const std::vector<double> sample_times{0.0, 0.25, 0.50};
    path_msg.poses.reserve(sample_times.size());
    for (const double sample_t : sample_times) {
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header = path_msg.header;
      pose_msg.header.stamp = stamp_with_offset(publish_stamp, sample_t);
      pose_msg.pose.position.x = hold_point.x;
      pose_msg.pose.position.y = hold_point.y;
      pose_msg.pose.position.z = 0.0;
      pose_msg.pose.orientation = quaternion_from_yaw(pose_.yaw);
      path_msg.poses.push_back(pose_msg);
    }
    path_pub_->publish(path_msg);
    has_previous_trajectory_ = false;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "发布短保持轨迹：原因=%s，保持点=(%.3f, %.3f)，轨迹点数=%zu。",
      reason, hold_point.x, hold_point.y, path_msg.poses.size());
  }

  bool should_publish_start_escape(
    const Point2D & current_position, const Point2D & projected_start, bool projection_moved) const
  {
    if (!start_projection_escape_enabled_ || !projection_moved) {
      return false;
    }
    return distance_2d(current_position, projected_start) >= start_projection_escape_min_distance_;
  }

  void publish_start_escape_path(
    const Point2D & current_position, const Point2D & projected_start,
    const rclcpp::Time & publish_stamp, const char * reason)
  {
    const double escape_distance = distance_2d(current_position, projected_start);
    const double escape_speed = std::max(0.05, start_projection_escape_speed_);
    const double duration = std::clamp(escape_distance / escape_speed, 0.6, 2.5);
    constexpr int sample_count = 6;

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = publish_stamp;
    path_msg.header.frame_id = map_frame_;
    path_msg.poses.reserve(sample_count);
    const double yaw = pose_.yaw;
    for (int i = 0; i < sample_count; ++i) {
      const double ratio = static_cast<double>(i) / static_cast<double>(sample_count - 1);
      const double sample_t = ratio * duration;
      geometry_msgs::msg::PoseStamped pose_msg;
      pose_msg.header = path_msg.header;
      pose_msg.header.stamp = stamp_with_offset(publish_stamp, sample_t);
      pose_msg.pose.position.x =
        current_position.x + (projected_start.x - current_position.x) * ratio;
      pose_msg.pose.position.y =
        current_position.y + (projected_start.y - current_position.y) * ratio;
      pose_msg.pose.position.z = 0.0;
      pose_msg.pose.orientation = quaternion_from_yaw(yaw);
      path_msg.poses.push_back(pose_msg);
    }

    path_pub_->publish(path_msg);
    has_previous_trajectory_ = false;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "发布起点清障脱困轨迹：原因=%s，当前位置=(%.3f, %.3f)，安全起点=(%.3f, %.3f)，距离=%.3f m，时长=%.3f s，期望速度=%.3f m/s。",
      reason, current_position.x, current_position.y, projected_start.x, projected_start.y,
      escape_distance, duration, escape_distance / duration);
  }

  enum class ReplanStartKind
  {
    CurrentPose,
    Inherited,
    Transition
  };

  struct ReplanStartSelection
  {
    Point2D point;
    ReplanStartKind kind{ReplanStartKind::CurrentPose};
    double clearance{std::numeric_limits<double>::infinity()};
  };

  const char * replan_start_kind_name(ReplanStartKind kind) const
  {
    switch (kind) {
      case ReplanStartKind::Inherited:
        return "继承点";
      case ReplanStartKind::Transition:
        return "过渡点";
      case ReplanStartKind::CurrentPose:
      default:
        return "当前位姿";
    }
  }

  // 计算本轮重规划模式决策（报告 5.5.4.4 模式选择）。
  ReplanDecision decide_replan_mode(const Point2D & current_position, const EsdfMap & esdf) const
  {
    ReplanDecisionConfig decision_config;
    decision_config.goal_change_threshold = replan_goal_change_threshold_;
    decision_config.localization_deviation_threshold = replan_localization_deviation_threshold_;
    decision_config.collision_clearance = replan_collision_clearance_;
    decision_config.good_tracking_threshold = replan_good_tracking_threshold_;
    decision_config.partial_enabled = replan_partial_enabled_;
    decision_config.optimize_only_enabled = replan_optimize_only_enabled_;
    ReplanStrategy strategy(decision_config);
    return strategy.decide(
      previous_trajectory_, has_previous_trajectory_, current_position, goal_, last_planned_goal_,
      has_last_planned_goal_, esdf);
  }

  void log_replan_mode(const ReplanDecision & decision)
  {
    if (!replan_strategy_enabled_) {
      return;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "重规划模式=%s：目标跳变=%.3f m，定位偏离=%.3f m，前方最小障碍距离=%.3f m，"
      "无碰撞=%s，目标稳定=%s，跟踪良好=%s，映射点时间=%.3f s。",
      ReplanStrategy::mode_name(decision.mode), decision.goal_change,
      decision.localization_deviation, decision.min_trajectory_clearance,
      decision.trajectory_collision_free ? "是" : "否", decision.goal_stable ? "是" : "否",
      decision.tracking_good ? "是" : "否", decision.mapped_t);
  }

  ReplanStartSelection select_replan_start(
    const Point2D & current_position, const rclcpp::Time & stamp, const EsdfMap & esdf,
    bool allow_inheritance = true)
  {
    ReplanStartSelection selection;
    selection.point = current_position;
    selection.clearance =
      esdf.valid() ? esdf.query_distance(current_position.x, current_position.y) :
      std::numeric_limits<double>::infinity();
    // 完全重规划模式（allow_inheritance=false）：起点强制使用当前位置，不继承上一轨迹，
    // 对应报告“定位严重偏离/目标大幅跳变”时从当前位置重新搜索。
    if (!allow_inheritance || !replan_start_position_inheritance_enabled_ ||
      !has_previous_trajectory_ || previous_trajectory_.empty())
    {
      return selection;
    }

    const auto inherited = previous_state_at(stamp);
    const Point2D inherited_position{inherited.x, inherited.y};
    const double position_error = distance_2d(current_position, inherited_position);
    const double inherited_clearance =
      esdf.valid() ? esdf.query_distance(inherited_position.x, inherited_position.y) :
      std::numeric_limits<double>::infinity();
    if (position_error > replan_start_max_position_error_) {
      const double transition_limit = std::max(
        replan_start_max_position_error_, replan_start_transition_max_position_error_);
      if (replan_start_transition_enabled_ && position_error <= transition_limit &&
        replan_start_max_position_error_ > 1.0e-6)
      {
        const auto transition = make_bounded_transition_start(
          current_position, inherited_position, position_error);
        const double transition_clearance =
          esdf.valid() ? esdf.query_distance(transition.x, transition.y) :
          std::numeric_limits<double>::infinity();
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "重规划起点使用过渡继承：当前位置=(%.3f, %.3f)，继承点=(%.3f, %.3f)，过渡点=(%.3f, %.3f)，原偏差=%.3f m，新起点距机器人=%.3f m。",
          current_position.x, current_position.y, inherited_position.x, inherited_position.y,
          transition.x, transition.y, position_error, distance_2d(current_position, transition));
        // 过渡点贴障不再在此处判否：硬通行性由端点 BFS 统一裁决，
        // 软安全余量由连续代价场和优化器承担。
        selection.point = transition;
        selection.kind = ReplanStartKind::Transition;
        selection.clearance = transition_clearance;
        return selection;
      }
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "重规划起点位置继承被拒绝：当前位置=(%.3f, %.3f)，继承点=(%.3f, %.3f)，偏差=%.3f m，超过阈值 %.3f m。",
        current_position.x, current_position.y, inherited_position.x, inherited_position.y,
        position_error, replan_start_max_position_error_);
      return selection;
    }
    if (inherited_clearance < optimizer_config_.obstacle_cost_radius) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "重规划起点继承点贴近障碍，交由端点 BFS 处理：距离=%.3f m，参考安全距离=%.3f m。",
        inherited_clearance, optimizer_config_.obstacle_cost_radius);
      selection.point = inherited_position;
      selection.kind = ReplanStartKind::Inherited;
      selection.clearance = inherited_clearance;
      return selection;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "重规划起点位置继承上一轨迹期望点：当前位置=(%.3f, %.3f)，继承点=(%.3f, %.3f)，偏差=%.3f m。",
      current_position.x, current_position.y, inherited_position.x, inherited_position.y,
      position_error);
    selection.point = inherited_position;
    selection.kind = ReplanStartKind::Inherited;
    selection.clearance = inherited_clearance;
    return selection;
  }

  Point2D make_bounded_transition_start(
    const Point2D & current_position, const Point2D & inherited_position,
    double position_error) const
  {
    if (position_error < 1.0e-9) {
      return inherited_position;
    }

    const double allowed_robot_error =
      std::clamp(replan_start_max_position_error_, 0.0, position_error);
    const double ratio_from_current = allowed_robot_error / position_error;
    return {
      current_position.x + (inherited_position.x - current_position.x) * ratio_from_current,
      current_position.y + (inherited_position.y - current_position.y) * ratio_from_current};
  }

  std::vector<Point2D> restore_planning_endpoints(
    const std::vector<Point2D> & search_points, const Point2D & plan_start,
    const Point2D & plan_goal)
  {
    auto planning_points = search_points;
    if (planning_points.size() < 2) {
      return planning_points;
    }

    const double start_shift = distance_2d(planning_points.front(), plan_start);
    const double goal_shift = distance_2d(planning_points.back(), plan_goal);
    planning_points.front() = plan_start;
    planning_points.back() = plan_goal;

    if (start_shift > 1.0e-4 || goal_shift > 1.0e-4) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "前端路径端点已恢复为连续规划端点：起点修正=%.3f m，目标修正=%.3f m。",
        start_shift, goal_shift);
    }
    return planning_points;
  }

  // 仅优化模式：不重跑前端搜索，直接把上一轨迹从映射点到末端的前方部分按设定间距重采样，
  // 末点替换为本轮（重定位后的）目标点，作为 MINCO 两步优化的输入路点。
  // 对应报告 5.5.4.4“仅优化”分支：从重映射位置开始，按相同等时间间隔重新采样轨迹。
  std::vector<Point2D> resample_previous_forward(double mapped_t, const Point2D & plan_goal) const
  {
    std::vector<Point2D> points;
    if (!has_previous_trajectory_ || previous_trajectory_.empty()) {
      return points;
    }
    const double duration = previous_trajectory_.duration();
    const double from_t = std::clamp(mapped_t, 0.0, duration);
    if (duration - from_t < 1.0e-3) {
      return points;
    }
    // 先按时间均匀采样前方轨迹，再按空间间距重采样，保证路点疏密与其他模式一致。
    std::vector<Point2D> forward;
    constexpr double kSampleDt = 0.05;
    for (double t = from_t; t <= duration + 1.0e-9; t += kSampleDt) {
      const auto p = previous_trajectory_.evaluate(std::min(t, duration));
      forward.push_back({p.x, p.y});
    }
    if (forward.size() < 2) {
      return points;
    }
    points = resample_polyline(forward, optimizer_config_.waypoint_spacing);
    if (points.size() >= 2) {
      points.back() = plan_goal;  // 末点对齐当前目标，保证终点一致
    }
    return points;
  }

  TrajectoryBoundaryCondition make_boundary_condition(const rclcpp::Time & stamp) const
  {
    TrajectoryBoundaryCondition condition;
    if (!has_previous_trajectory_ || previous_trajectory_.empty()) {
      return condition;
    }

    const auto inherited = previous_state_at(stamp);
    condition.inherit_start_state = true;
    condition.start_velocity = {inherited.vx, inherited.vy};
    condition.start_acceleration = {inherited.ax, inherited.ay};
    return condition;
  }

  double elapsed_since_previous_trajectory(const rclcpp::Time & stamp) const
  {
    double elapsed = (stamp - previous_trajectory_stamp_).seconds();
    if (!std::isfinite(elapsed) || elapsed < 0.0) {
      elapsed = 0.0;
    }
    return elapsed;
  }

  TrajectoryPoint previous_state_at(const rclcpp::Time & stamp) const
  {
    const double elapsed = elapsed_since_previous_trajectory(stamp);
    return previous_trajectory_.evaluate(std::min(elapsed, previous_trajectory_.duration()));
  }

  void log_replan_continuity(const MincoTrajectory & trajectory, const rclcpp::Time & stamp)
  {
    if (!has_previous_trajectory_ || previous_trajectory_.empty() || trajectory.empty()) {
      return;
    }

    const double elapsed = elapsed_since_previous_trajectory(stamp);
    const double previous_t = std::min(elapsed, previous_trajectory_.duration());
    const auto expected = previous_state_at(stamp);
    const auto current = trajectory.evaluate(0.0);

    const double position_jump = std::hypot(current.x - expected.x, current.y - expected.y);
    const double velocity_jump = std::hypot(current.vx - expected.vx, current.vy - expected.vy);
    const double acceleration_jump = std::hypot(current.ax - expected.ax, current.ay - expected.ay);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "重规划连续性：间隔=%.3f s，上一轨迹采样时刻=%.3f s，位置跳变=%.3f m，速度跳变=%.3f m/s，加速度跳变=%.3f m/s^2。",
      elapsed, previous_t, position_jump, velocity_jump, acceleration_jump);
  }

  builtin_interfaces::msg::Time stamp_with_offset(
    const rclcpp::Time & base_stamp, double offset_seconds) const
  {
    const auto offset_ns = static_cast<int64_t>(
      std::llround(std::max(0.0, offset_seconds) * 1.0e9));
    const int64_t total_ns = base_stamp.nanoseconds() + offset_ns;
    builtin_interfaces::msg::Time stamp;
    stamp.sec = static_cast<int32_t>(total_ns / 1000000000LL);
    stamp.nanosec = static_cast<uint32_t>(total_ns % 1000000000LL);
    return stamp;
  }

  Grid2D occupancy_msg_to_grid(const nav_msgs::msg::OccupancyGrid & msg) const
  {
    Grid2D grid;
    grid.width = static_cast<int>(msg.info.width);
    grid.height = static_cast<int>(msg.info.height);
    grid.resolution = msg.info.resolution;
    grid.origin = {msg.info.origin.position.x, msg.info.origin.position.y};
    grid.data.assign(static_cast<std::size_t>(grid.width * grid.height), 0);
    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        const auto idx = static_cast<std::size_t>(y * grid.width + x);
        // 输入是 0-100 语义的 OccupancyGrid；障碍格作为膨胀源写入满代价 255，
        // 其余留 0，由后续连续膨胀填充软代价（Grid2D 为 0-255 代价场）。
        grid.at(x, y) = msg.data[idx] > 50 ? 255 : 0;
      }
    }
    return grid;
  }

  // BUG-4：把内部 Grid2D（前端搜索用的连续代价场）发布为 OccupancyGrid，几何沿用最新输入图 info。
  void publish_grid_as_occupancy(const Grid2D & grid)
  {
    nav_msgs::msg::OccupancyGrid out;
    out.header.stamp = now();
    out.header.frame_id = map_frame_;
    out.info = latest_grid_.info;
    out.data.assign(static_cast<std::size_t>(grid.width * grid.height), 0);
    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        const auto idx = static_cast<std::size_t>(y * grid.width + x);
        // 0-255 连续代价场按比例映射到 OccupancyGrid 的 0-100，便于 RViz 直接观察衰减带。
        out.data[idx] = static_cast<std::int8_t>(
          std::lround(static_cast<double>(grid.at(x, y)) * 100.0 / 255.0));
      }
    }
    inflated_grid_pub_->publish(out);
  }

  std::string odom_topic_;
  std::string obstacle_grid_topic_;
  std::string dynamic_grid_topic_;
  std::string goal_topic_;
  std::string path_topic_;
  std::string map_frame_;
  InflationParams inflation_params_{};
  InflationParams dynamic_inflation_params_{};
  double replan_rate_hz_{5.0};
  double endpoint_nudge_max_distance_{1.0};
  double astar_obstacle_weight_{1.0};
  int astar_max_expansions_{1000000};
  bool replan_start_position_inheritance_enabled_{true};
  double replan_start_max_position_error_{0.18};
  bool replan_start_transition_enabled_{true};
  double replan_start_transition_max_position_error_{0.36};
  double short_path_hold_distance_{0.25};
  bool start_projection_escape_enabled_{true};
  double start_projection_escape_min_distance_{0.28};
  double start_projection_escape_speed_{0.25};
  bool replan_strategy_enabled_{true};
  double replan_goal_change_threshold_{0.5};
  bool replan_optimize_only_enabled_{true};
  bool replan_partial_enabled_{true};
  double replan_collision_clearance_{0.05};
  double replan_localization_deviation_threshold_{0.6};
  double replan_good_tracking_threshold_{0.25};
  bool publish_inflated_grid_{false};
  std::string inflated_grid_topic_;
  bool has_odom_{false};
  bool has_grid_{false};
  bool has_dynamic_grid_{false};
  bool has_goal_{false};
  Pose2D pose_;
  Point2D goal_;
  // 上一轮规划所用的全局目标，用于检测目标突变以触发全局重规划。
  Point2D last_planned_goal_;
  bool has_last_planned_goal_{false};
  nav_msgs::msg::OccupancyGrid latest_grid_;
  nav_msgs::msg::OccupancyGrid latest_dynamic_grid_;
  TrajectoryOptimizerConfig optimizer_config_{make_optimizer_config()};
  TrajectoryOptimizer optimizer_;
  bool has_previous_trajectory_{false};
  MincoTrajectory previous_trajectory_;
  rclcpp::Time previous_trajectory_stamp_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr inflated_grid_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr dynamic_grid_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

}  // namespace astra_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<astra_nav::PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
