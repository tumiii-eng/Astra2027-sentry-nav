#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "astra_common/common.hpp"
#include "astra_control/preview_controller.hpp"
#include "astra_control/se2_mpc_preview.hpp"
#include "astra_control/se2_mpc_controller.hpp"
#include "astra_control/chassis_kinematics.hpp"
#include "astra_control/high_rate_state_estimator.hpp"
#include "astra_planning/minco_trajectory.hpp"

namespace astra_nav
{

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode()
  : Node("controller_node"), controller_(make_controller_config()),
    mpc_preview_(make_mpc_preview_config()), mpc_controller_(make_mpc_config()),
    chassis_(make_chassis_config()), estimator_(make_estimator_config())
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/astra/odom");
    path_topic_ = declare_parameter<std::string>("path_topic", "/astra/trajectory");
    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    control_rate_hz_ = declare_parameter("control_rate_hz", 50.0);
    // 控制器类型：preview=预瞄原型（默认，兼容旧行为）；mpc=报告 5.5.5 的 SE(2) MPC-QP。
    controller_type_ = declare_parameter<std::string>("controller_type", "preview");
    tracking_log_period_ms_ = declare_parameter("tracking_log_period_ms", 1000);
    tracking_error_warn_threshold_ = declare_parameter("tracking_error_warn_threshold", 0.35);
    tracking_velocity_error_warn_threshold_ =
      declare_parameter("tracking_velocity_error_warn_threshold", 0.35);
    linear_command_accel_limit_ = declare_parameter("linear_command_accel_limit", 1.2);
    angular_command_accel_limit_ = declare_parameter("angular_command_accel_limit", 2.0);
    startup_velocity_error_grace_time_ =
      declare_parameter("startup_velocity_error_grace_time", 0.6);
    mpc_preview_enabled_ = declare_parameter("mpc_preview_enabled", true);
    mpc_preview_error_warn_threshold_ =
      declare_parameter("mpc_preview_error_warn_threshold", 0.35);

    // —— 高频积分定位估计（报告 5.5.5.4.1）——
    // 默认关闭，保持“直接用原始里程计位姿”的现有行为；开启后用 IMU 航向角速度
    // 与底盘轮速换算的体速度，从有延迟的里程计锚点向前积分到控制时刻。
    state_estimation_enabled_ = declare_parameter("state_estimation_enabled", false);
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/astra/imu");
    wheel_joint_state_topic_ =
      declare_parameter<std::string>("wheel_joint_state_topic", "/astra/joint_states");
    // 里程计 twist 坐标系约定：true=机体系（默认，与既有 MPC 路径一致），false=世界系。
    odom_twist_in_body_frame_ = declare_parameter("odom_twist_in_body_frame", true);
    estimation_log_period_ms_ = declare_parameter("estimation_log_period_ms", 2000);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 50, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        pose_.x = msg->pose.pose.position.x;
        pose_.y = msg->pose.pose.position.y;
        pose_.yaw = yaw_from_quaternion(msg->pose.pose.orientation);
        odom_twist_.vx = msg->twist.twist.linear.x;
        odom_twist_.vy = msg->twist.twist.linear.y;
        odom_twist_.wz = msg->twist.twist.angular.z;
        has_odom_ = true;
        if (state_estimation_enabled_) {
          // 设积分锚点：估计器内部统一用世界系速度积分。
          const Twist2D world_velocity = odom_twist_to_world(odom_twist_, pose_.yaw);
          estimator_.set_anchor(
            pose_, world_velocity, rclcpp::Time(msg->header.stamp).seconds());
        }
      });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      path_topic_, 5, [this](nav_msgs::msg::Path::SharedPtr msg) {
        update_trajectory_from_path(*msg);
      });

    if (state_estimation_enabled_) {
      // IMU：取航向角速度（陀螺不打滑），作为高频航向积分输入（报告 IMU 200Hz）。
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 100, [this](sensor_msgs::msg::Imu::SharedPtr msg) {
          MotionSample sample;
          sample.stamp = rclcpp::Time(msg->header.stamp).seconds();
          sample.yaw_rate = msg->angular_velocity.z;
          sample.has_yaw_rate = true;
          estimator_.add_motion_sample(sample);
        });
      // 轮速：裸轮速经底盘正运动学换算为体速度（麦轮/全向轮由参数切换）。
      wheel_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        wheel_joint_state_topic_, 50, [this](sensor_msgs::msg::JointState::SharedPtr msg) {
          ChassisWheelSpeeds wheels;
          if (!extract_wheel_speeds(*msg, wheels)) {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 5000,
              "轮速关节名匹配失败，无法换算体速度，请核对 chassis_wheel_joint_names 参数。");
            return;
          }
          const Twist2D body = chassis_.forward(wheels);
          MotionSample sample;
          sample.stamp = rclcpp::Time(msg->header.stamp).seconds();
          sample.body_vx = body.vx;
          sample.body_vy = body.vy;
          sample.has_body_velocity = true;
          estimator_.add_motion_sample(sample);
        });
    }

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(control_rate_hz_, 1e-3))),
      std::bind(&ControllerNode::on_timer, this));

    use_mpc_ = controller_type_ == "mpc";
    RCLCPP_INFO(
      get_logger(),
      "控制节点已启动，控制器类型=%s，高频积分定位=%s，等待里程计和规划轨迹。",
      use_mpc_ ? "MPC（报告5.5.5 SE(2) QP）" : "预瞄原型",
      state_estimation_enabled_ ? "已开启（报告5.5.5.4.1）" : "关闭");
  }

private:
  PreviewControllerConfig make_controller_config()
  {
    PreviewControllerConfig config;
    config.lookahead_time = declare_parameter("lookahead_time", 0.18);
    config.max_linear_velocity = declare_parameter("max_linear_velocity", 1.8);
    config.max_angular_velocity = declare_parameter("max_angular_velocity", 2.5);
    config.position_kp = declare_parameter("position_kp", 1.4);
    config.heading_kp = declare_parameter("heading_kp", 2.2);
    config.velocity_damping = declare_parameter("velocity_damping", 0.6);
    config.arrival_position_tolerance = declare_parameter("arrival_position_tolerance", 0.18);
    config.arrival_reference_speed_tolerance =
      declare_parameter("arrival_reference_speed_tolerance", 0.05);
    config.path_heading_tracking_enabled = declare_parameter("path_heading_tracking_enabled", false);
    config.heading_startup_ramp_time = declare_parameter("heading_startup_ramp_time", 0.8);
    config.heading_error_deadband = declare_parameter("heading_error_deadband", 0.03);
    config.heading_control_min_reference_speed =
      declare_parameter("heading_control_min_reference_speed", 0.05);
    return config;
  }

  Se2MpcPreviewConfig make_mpc_preview_config()
  {
    Se2MpcPreviewConfig config;
    config.steps = declare_parameter("mpc_preview_steps", 12);
    config.dt = declare_parameter("mpc_preview_dt", 0.05);
    return config;
  }

  // 报告 5.5.5 SE(2) MPC-QP 控制器配置。物理量与权重均为待标定默认值。
  Se2MpcConfig make_mpc_config()
  {
    Se2MpcConfig config;
    config.horizon = declare_parameter("mpc_horizon", 40);
    config.dt = declare_parameter("mpc_dt", 0.05);
    config.mass = declare_parameter("mpc_mass", 15.0);
    config.inertia = declare_parameter("mpc_inertia", 0.6);
    config.weight_position = declare_parameter("mpc_weight_position", 12.0);
    config.weight_yaw = declare_parameter("mpc_weight_yaw", 2.0);
    config.weight_velocity = declare_parameter("mpc_weight_velocity", 1.0);
    config.weight_omega = declare_parameter("mpc_weight_omega", 0.2);
    config.weight_force = declare_parameter("mpc_weight_force", 0.02);
    config.weight_torque = declare_parameter("mpc_weight_torque", 0.05);
    config.max_velocity = declare_parameter("mpc_max_velocity", 2.0);
    config.max_omega = declare_parameter("mpc_max_omega", 2.5);
    config.max_force = declare_parameter("mpc_max_force", 60.0);
    config.max_torque = declare_parameter("mpc_max_torque", 20.0);
    config.friction_coefficient = declare_parameter("mpc_friction_coefficient", 0.8);
    config.gravity = declare_parameter("mpc_gravity", 9.81);
    config.friction_polygon_sides = declare_parameter("mpc_friction_polygon_sides", 8);
    config.secondary_min_scale = declare_parameter("mpc_secondary_min_scale", 0.3);
    config.secondary_pass_enabled = declare_parameter("mpc_secondary_pass_enabled", true);
    config.command_lookahead_time = declare_parameter("mpc_command_lookahead_time", 0.01);
    config.startup_speed_floor = declare_parameter("mpc_startup_speed_floor", 0.0);
    config.arrival_position_tolerance = declare_parameter("mpc_arrival_position_tolerance", 0.18);
    config.arrival_speed_tolerance = declare_parameter("mpc_arrival_speed_tolerance", 0.05);
    config.max_working_set_recalculations = declare_parameter("mpc_max_nwsr", 200);
    return config;
  }

  // 底盘正运动学配置（报告 5.5.5.4.1 轮速换算层）。麦轮/全向轮由 chassis_type 切换。
  ChassisKinematicsConfig make_chassis_config()
  {
    ChassisKinematicsConfig config;
    const std::string type = declare_parameter<std::string>("chassis_type", "mecanum");
    config.type = type == "omni" ? ChassisType::Omni : ChassisType::Mecanum;
    config.wheel_radius = declare_parameter("chassis_wheel_radius", 0.0762);
    config.half_wheel_base_x = declare_parameter("chassis_half_wheel_base_x", 0.2);
    config.half_wheel_base_y = declare_parameter("chassis_half_wheel_base_y", 0.2);
    config.wheel_mount_radius = declare_parameter("chassis_wheel_mount_radius", 0.3);
    const std::vector<double> sign = declare_parameter<std::vector<double>>(
      "chassis_wheel_direction_sign", std::vector<double>{1.0, 1.0, 1.0, 1.0});
    for (std::size_t i = 0; i < 4 && i < sign.size(); ++i) {
      config.wheel_direction_sign[i] = sign[i];
    }
    // 轮速关节名顺序 [FL, FR, RL, RR]，用于从 JointState 按名取值（顺序与底盘运动学一致）。
    chassis_wheel_joint_names_ = declare_parameter<std::vector<std::string>>(
      "chassis_wheel_joint_names",
      std::vector<std::string>{
        "front_left_wheel_joint", "front_right_wheel_joint",
        "back_left_wheel_joint", "back_right_wheel_joint"});
    return config;
  }

  // 高频积分定位估计器配置（报告 5.5.5.4.1，含轻量打滑闸门+锚点兜底）。
  HighRateStateEstimatorConfig make_estimator_config()
  {
    HighRateStateEstimatorConfig config;
    config.max_extrapolation_time = declare_parameter("estimation_max_extrapolation_time", 0.2);
    config.max_buffer_time = declare_parameter("estimation_max_buffer_time", 0.5);
    config.use_imu_yaw_rate = declare_parameter("estimation_use_imu_yaw_rate", true);
    config.use_body_velocity = declare_parameter("estimation_use_body_velocity", true);
    config.enable_slip_gate = declare_parameter("estimation_enable_slip_gate", true);
    config.max_body_acceleration = declare_parameter("estimation_max_body_acceleration", 12.0);
    return config;
  }

  // 里程计 twist → 世界系速度。odom_twist_in_body_frame_=true 时按航向旋到世界系。
  Twist2D odom_twist_to_world(const Twist2D & odom_twist, double yaw) const
  {
    Twist2D world;
    world.wz = odom_twist.wz;
    if (odom_twist_in_body_frame_) {
      const double cy = std::cos(yaw);
      const double sy = std::sin(yaw);
      world.vx = cy * odom_twist.vx - sy * odom_twist.vy;
      world.vy = sy * odom_twist.vx + cy * odom_twist.vy;
    } else {
      world.vx = odom_twist.vx;
      world.vy = odom_twist.vy;
    }
    return world;
  }

  // 从 JointState 按 chassis_wheel_joint_names_ 顺序提取四轮角速度。
  bool extract_wheel_speeds(
    const sensor_msgs::msg::JointState & msg, ChassisWheelSpeeds & wheels) const
  {
    if (chassis_wheel_joint_names_.size() < 4 || msg.velocity.empty()) {
      return false;
    }
    double values[4] = {0.0, 0.0, 0.0, 0.0};
    for (int w = 0; w < 4; ++w) {
      bool found = false;
      for (std::size_t i = 0; i < msg.name.size() && i < msg.velocity.size(); ++i) {
        if (msg.name[i] == chassis_wheel_joint_names_[w]) {
          values[w] = msg.velocity[i];
          found = true;
          break;
        }
      }
      if (!found) {
        return false;
      }
    }
    wheels.fl = values[0];
    wheels.fr = values[1];
    wheels.rl = values[2];
    wheels.rr = values[3];
    return true;
  }

  void update_trajectory_from_path(const nav_msgs::msg::Path & path)
  {
    if (path.poses.size() < 2) {
      trajectory_.clear();
      RCLCPP_WARN(get_logger(), "收到的轨迹点过少，控制器将输出零速度。");
      return;
    }

    const bool used_path_timing = build_timed_trajectory_from_path(path, trajectory_);
    if (!used_path_timing) {
      trajectory_ = refit_trajectory_from_path(path);
    }
    trajectory_start_time_ = now();
    has_trajectory_ = !trajectory_.empty();
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "控制器已接收新轨迹，采样点数量：%zu，时间来源：%s。",
      trajectory_.size(), used_path_timing ? "规划轨迹时间戳" : "控制器重拟合");
  }

  bool build_timed_trajectory_from_path(
    const nav_msgs::msg::Path & path, std::vector<TrajectoryPoint> & output) const
  {
    output.clear();
    output.reserve(path.poses.size());
    const rclcpp::Time first_stamp(path.poses.front().header.stamp);
    bool has_increasing_time = false;
    double last_t = -1.0;
    for (const auto & pose : path.poses) {
      const rclcpp::Time pose_stamp(pose.header.stamp);
      const double t = (pose_stamp - first_stamp).seconds();
      if (!std::isfinite(t) || t + 1.0e-6 < last_t) {
        output.clear();
        return false;
      }
      if (t > 1.0e-6) {
        has_increasing_time = true;
      }
      TrajectoryPoint point;
      point.t = std::max(0.0, t);
      point.x = pose.pose.position.x;
      point.y = pose.pose.position.y;
      point.yaw = yaw_from_quaternion(pose.pose.orientation);
      output.push_back(point);
      last_t = point.t;
    }
    if (!has_increasing_time) {
      output.clear();
      return false;
    }

    fill_timed_trajectory_derivatives(output);
    return true;
  }

  std::vector<TrajectoryPoint> refit_trajectory_from_path(const nav_msgs::msg::Path & path) const
  {
    std::vector<Point2D> points;
    points.reserve(path.poses.size());
    for (const auto & pose : path.poses) {
      points.push_back({pose.pose.position.x, pose.pose.position.y});
    }
    const auto times = cumulative_times(points, 1.5, 1.6);
    return MincoTrajectory::fit(points, times).sample(0.02);
  }

  void fill_timed_trajectory_derivatives(std::vector<TrajectoryPoint> & samples) const
  {
    if (samples.size() < 2) {
      return;
    }

    for (std::size_t i = 0; i < samples.size(); ++i) {
      const std::size_t prev = i == 0 ? i : i - 1;
      const std::size_t next = i + 1 < samples.size() ? i + 1 : i;
      if (prev == next) {
        continue;
      }
      const double dt = std::max(1.0e-3, samples[next].t - samples[prev].t);
      samples[i].vx = (samples[next].x - samples[prev].x) / dt;
      samples[i].vy = (samples[next].y - samples[prev].y) / dt;
    }

    for (std::size_t i = 0; i < samples.size(); ++i) {
      const std::size_t prev = i == 0 ? i : i - 1;
      const std::size_t next = i + 1 < samples.size() ? i + 1 : i;
      if (prev == next) {
        continue;
      }
      const double dt = std::max(1.0e-3, samples[next].t - samples[prev].t);
      samples[i].ax = (samples[next].vx - samples[prev].vx) / dt;
      samples[i].ay = (samples[next].vy - samples[prev].vy) / dt;
    }
  }

  void on_timer()
  {
    geometry_msgs::msg::Twist cmd;
    if (!has_odom_ || !has_trajectory_) {
      // 没有里程计或轨迹时【不发布】任何速度命令。
      // 原因：下游 fake_vel_transform 只要收到 cmd_vel（哪怕零速）就会叠加小陀螺自旋
      // （init_spin_speed），导致发目标点前机器人就持续自旋、odom/TF 抖动。保持静默即可
      // 让 fake_vel_transform 判定控制器超时而不自旋，机器人在发目标前完全静止。
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "控制器等待输入：里程计=%s，轨迹=%s（未发布速度，保持静止）。",
        has_odom_ ? "已收到" : "未收到",
        has_trajectory_ ? "已收到" : "未收到");
      return;
    }

    const double elapsed = std::max(0.0, (now() - trajectory_start_time_).seconds());

    // —— 高频积分定位估计（报告 5.5.5.4.1）——
    // 开启时：以有延迟的里程计为锚点，借 IMU 航向角速度+底盘体速度积分到当前控制时刻，
    //   得到最新位姿与世界系速度；关闭时：直接用原始里程计位姿与（旋转得到的）世界系速度。
    Pose2D control_pose = pose_;
    Twist2D world_velocity = odom_twist_to_world(odom_twist_, pose_.yaw);
    if (state_estimation_enabled_ && estimator_.has_anchor()) {
      const auto est = estimator_.estimate(now().seconds());
      control_pose = est.pose;
      world_velocity = est.world_velocity;
      log_estimation(est);
    }

    // 计算期望速度命令。两种控制器：
    //   preview：预瞄原型，吃世界系位姿与轨迹；
    //   mpc：报告 5.5.5 SE(2) MPC-QP，吃世界系位姿+世界系速度，返回机体系命令；
    //        QP 失败时回退到预瞄控制器保证不输出垃圾命令。
    Twist2D desired_twist;
    bool mpc_used = false;
    if (use_mpc_) {
      const auto mpc_cmd =
        mpc_controller_.compute(control_pose, world_velocity, trajectory_, elapsed);
      if (mpc_controller_.last_valid()) {
        desired_twist = mpc_cmd;
        mpc_used = true;
      } else {
        // QP 求解失败：回退到预瞄控制器（同样传入真实世界速度用于速度阻尼刹车）。
        desired_twist = controller_.compute(control_pose, world_velocity, trajectory_, elapsed);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "MPC 求解失败，本周期回退到预瞄控制器输出。");
      }
      log_mpc_solver(mpc_controller_.last_debug());
    } else {
      desired_twist = controller_.compute(control_pose, world_velocity, trajectory_, elapsed);
    }

    const auto current_time = now();
    const double control_dt = has_last_control_time_ ?
      std::max(1.0e-3, (current_time - last_control_time_).seconds()) :
      1.0 / std::max(control_rate_hz_, 1.0e-3);
    const auto twist = apply_command_rate_limit(desired_twist, control_dt);
    cmd.linear.x = twist.vx;
    cmd.linear.y = twist.vy;
    cmd.angular.z = twist.wz;
    cmd_pub_->publish(cmd);

    const double command_linear_delta = has_last_cmd_ ?
      std::hypot(twist.vx - last_cmd_.vx, twist.vy - last_cmd_.vy) : 0.0;
    const double command_angular_delta = has_last_cmd_ ? std::abs(twist.wz - last_cmd_.wz) : 0.0;
    // 预瞄控制器的跟踪指标依赖其内部 debug，仅在 preview 模式下输出。
    if (!mpc_used) {
      log_tracking_metrics(
        elapsed, desired_twist, twist, command_linear_delta, command_angular_delta);
    }
    if (mpc_preview_enabled_ && !use_mpc_) {
      const auto preview = mpc_preview_.predict(control_pose, twist, trajectory_, elapsed);
      const auto & debug = controller_.last_debug();
      log_mpc_preview(preview, debug.valid && debug.stopped_by_arrival_tolerance);
    }
    last_cmd_ = twist;
    has_last_cmd_ = true;
    last_control_time_ = current_time;
    has_last_control_time_ = true;
  }

  Twist2D apply_command_rate_limit(const Twist2D & desired, double dt) const
  {
    const Twist2D previous = has_last_cmd_ ? last_cmd_ : Twist2D{};
    Twist2D limited = desired;

    if (linear_command_accel_limit_ > 1.0e-6) {
      const double max_delta = linear_command_accel_limit_ * std::max(dt, 1.0e-3);
      const double delta_x = desired.vx - previous.vx;
      const double delta_y = desired.vy - previous.vy;
      const double delta_norm = std::hypot(delta_x, delta_y);
      if (delta_norm > max_delta) {
        const double scale = max_delta / std::max(delta_norm, 1.0e-9);
        limited.vx = previous.vx + delta_x * scale;
        limited.vy = previous.vy + delta_y * scale;
      }
    }

    if (angular_command_accel_limit_ > 1.0e-6) {
      const double max_delta = angular_command_accel_limit_ * std::max(dt, 1.0e-3);
      const double delta = std::clamp(desired.wz - previous.wz, -max_delta, max_delta);
      limited.wz = previous.wz + delta;
    }
    return limited;
  }

  void log_tracking_metrics(
    double elapsed, const Twist2D & desired_command, const Twist2D & output_command,
    double command_linear_delta, double command_angular_delta)
  {
    const auto & debug = controller_.last_debug();
    if (!debug.valid || trajectory_.empty()) {
      return;
    }

    const double duration = trajectory_.back().t;
    const double remaining_time = std::max(0.0, duration - elapsed);
    const double odom_linear_speed = std::hypot(odom_twist_.vx, odom_twist_.vy);
    const double odom_angular_speed = std::abs(odom_twist_.wz);
    const double desired_linear_speed = std::hypot(desired_command.vx, desired_command.vy);
    const double desired_angular_speed = std::abs(desired_command.wz);
    const double output_linear_speed = std::hypot(output_command.vx, output_command.vy);
    const double output_angular_speed = std::abs(output_command.wz);
    const double velocity_error = std::abs(debug.nearest_reference_speed - odom_linear_speed);
    const char * phase = elapsed <= duration ? "跟踪中" : "末端保持";
    if (debug.stopped_by_arrival_tolerance) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), tracking_log_period_ms_,
        "控制器进入近距离保持：阶段=%s，轨迹时刻=%.3f s，最近误差=%.3f m，前瞻误差=%.3f m，参考线速度=%.3f m/s，实际线速度=%.3f m/s，输出零速度，剩余时间=%.3f s。",
        phase, debug.current_time, debug.nearest_position_error, debug.target_position_error,
        debug.nearest_reference_speed, odom_linear_speed, remaining_time);
      return;
    }
    const double command_linear_gap =
      std::hypot(desired_command.vx - output_command.vx, desired_command.vy - output_command.vy);
    const double command_angular_gap = std::abs(desired_command.wz - output_command.wz);
    const bool startup_ramp_active =
      elapsed <= startup_velocity_error_grace_time_ &&
      (command_linear_gap > 0.03 || command_angular_gap > 0.03);
    if (startup_ramp_active && velocity_error > tracking_velocity_error_warn_threshold_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), tracking_log_period_ms_,
        "控制器启动渐入：阶段=%s，轨迹时刻=%.3f s，最近参考时刻=%.3f s，最近误差=%.3f m，前瞻误差=%.3f m，参考线速度=%.3f m/s，实际线速度=%.3f m/s，速度误差=%.3f m/s，期望指令=(%.3f m/s, %.3f rad/s)，输出指令=(%.3f m/s, %.3f rad/s)，命令变化=(%.3f m/s, %.3f rad/s)，剩余时间=%.3f s。",
        phase, debug.current_time, debug.nearest_reference_time, debug.nearest_position_error,
        debug.target_position_error, debug.nearest_reference_speed, odom_linear_speed,
        velocity_error, desired_linear_speed, desired_angular_speed, output_linear_speed,
        output_angular_speed, command_linear_delta, command_angular_delta, remaining_time);
      return;
    }
    if (debug.nearest_position_error > tracking_error_warn_threshold_ ||
      velocity_error > tracking_velocity_error_warn_threshold_)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), tracking_log_period_ms_,
        "控制跟踪误差偏大：阶段=%s，轨迹时刻=%.3f s，最近参考时刻=%.3f s，最近误差=%.3f m，前瞻误差=%.3f m，参考线速度=%.3f m/s，实际线速度=%.3f m/s，速度误差=%.3f m/s，生效航向误差=%.3f rad，原始航向误差=%.3f rad，航向权重=%.2f，指令线速度=%.3f m/s，指令角速度=%.3f rad/s，命令变化=(%.3f m/s, %.3f rad/s)，剩余时间=%.3f s。",
        phase, debug.current_time, debug.nearest_reference_time, debug.nearest_position_error,
        debug.target_position_error, debug.nearest_reference_speed, odom_linear_speed,
        velocity_error, debug.target_heading_error, debug.raw_target_heading_error,
        debug.heading_control_weight, output_linear_speed, output_angular_speed,
        command_linear_delta, command_angular_delta, remaining_time);
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), tracking_log_period_ms_,
      "控制跟踪指标：阶段=%s，轨迹时刻=%.3f s，最近参考时刻=%.3f s，前瞻参考时刻=%.3f s，最近误差=%.3f m，前瞻误差=%.3f m，参考线速度=%.3f m/s，实际线速度=%.3f m/s，速度误差=%.3f m/s，实际角速度=%.3f rad/s，生效航向误差=%.3f rad，原始航向误差=%.3f rad，航向权重=%.2f，指令线速度=%.3f m/s，指令角速度=%.3f rad/s，命令变化=(%.3f m/s, %.3f rad/s)，剩余时间=%.3f s。",
      phase, debug.current_time, debug.nearest_reference_time, debug.target_reference_time,
      debug.nearest_position_error, debug.target_position_error, debug.nearest_reference_speed,
      odom_linear_speed, velocity_error, odom_angular_speed, debug.target_heading_error,
      debug.raw_target_heading_error, debug.heading_control_weight, output_linear_speed,
      output_angular_speed, command_linear_delta, command_angular_delta, remaining_time);
  }

  void log_mpc_preview(const Se2MpcPreviewResult & preview, bool near_hold_active)
  {
    if (!preview.valid) {
      return;
    }

    if (near_hold_active) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), tracking_log_period_ms_,
        "MPC近距离保持预测：步数=%d，时域=%.3f s，保持误差=%.3f m，末端预测=(%.3f, %.3f)，末端参考=(%.3f, %.3f)，该误差处于近距离停车容差内。",
        preview.steps, preview.horizon_duration, preview.final_position_error,
        preview.final_prediction.x, preview.final_prediction.y,
        preview.final_reference.x, preview.final_reference.y);
      return;
    }

    if (preview.max_position_error > mpc_preview_error_warn_threshold_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), tracking_log_period_ms_,
        "MPC前置预测误差偏大：步数=%d，时域=%.3f s，均值误差=%.3f m，最大误差=%.3f m，末端误差=%.3f m，末端预测=(%.3f, %.3f)，末端参考=(%.3f, %.3f)。",
        preview.steps, preview.horizon_duration, preview.mean_position_error,
        preview.max_position_error, preview.final_position_error,
        preview.final_prediction.x, preview.final_prediction.y,
        preview.final_reference.x, preview.final_reference.y);
      return;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), tracking_log_period_ms_,
      "MPC前置预测指标：步数=%d，时域=%.3f s，均值误差=%.3f m，最大误差=%.3f m，末端误差=%.3f m，末端预测=(%.3f, %.3f)，末端参考=(%.3f, %.3f)。",
      preview.steps, preview.horizon_duration, preview.mean_position_error,
      preview.max_position_error, preview.final_position_error,
      preview.final_prediction.x, preview.final_prediction.y,
      preview.final_reference.x, preview.final_reference.y);
  }

  void log_mpc_solver(const Se2MpcDebug & debug)
  {
    if (debug.stopped_by_arrival) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), tracking_log_period_ms_,
        "MPC到达保持：投影时刻=%.3f s，最近误差=%.3f m，输出零速。",
        debug.projection_time, debug.nearest_position_error);
      return;
    }
    if (!debug.valid) {
      return;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), tracking_log_period_ms_,
      "MPC求解指标：首次QP=%s，二次QP=%s，采用二次=%s，投影时刻=%.3f s，时域=%.3f s，"
      "最近误差=%.3f m，最小方向余弦=%.3f，首次代价=%.3f，二次代价=%.3f，"
      "输出线速度=%.3f m/s，角速度=%.3f rad/s。",
      debug.first_qp_success ? "成功" : "失败",
      debug.second_qp_success ? "成功" : "失败",
      debug.used_secondary ? "是" : "否",
      debug.projection_time, debug.horizon_duration, debug.nearest_position_error,
      debug.secondary_min_cos, debug.first_qp_cost, debug.second_qp_cost,
      debug.command_speed, debug.command_omega);
  }

  // 高频积分定位估计日志（报告 5.5.5.4.1）：观测锚点时延、积分样本数与打滑拒绝数。
  void log_estimation(const EstimatedState & est)
  {
    if (est.extrapolated) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), estimation_log_period_ms_,
        "高频积分定位：锚点时延=%.1f ms，积分样本=%d，打滑拒绝=%d，"
        "估计位姿=(%.3f, %.3f, %.3f rad)，世界系速度=(%.3f, %.3f) m/s，角速度=%.3f rad/s。",
        est.anchor_age * 1000.0, est.integrated_samples, est.rejected_slip_samples,
        est.pose.x, est.pose.y, est.pose.yaw,
        est.world_velocity.vx, est.world_velocity.vy, est.world_velocity.wz);
      return;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), estimation_log_period_ms_,
      "高频积分定位：锚点时延=%.1f ms 超出外推上限或无高频样本，直接采用里程计锚点位姿。",
      est.anchor_age * 1000.0);
  }

  std::string odom_topic_;
  std::string path_topic_;
  std::string cmd_vel_topic_;
  std::string controller_type_{"preview"};
  bool use_mpc_{false};
  double control_rate_hz_{50.0};
  int tracking_log_period_ms_{1000};
  double tracking_error_warn_threshold_{0.35};
  double tracking_velocity_error_warn_threshold_{0.35};
  double linear_command_accel_limit_{1.2};
  double angular_command_accel_limit_{2.0};
  double startup_velocity_error_grace_time_{0.6};
  bool mpc_preview_enabled_{true};
  double mpc_preview_error_warn_threshold_{0.35};
  // —— 高频积分定位估计（报告 5.5.5.4.1）——
  bool state_estimation_enabled_{false};
  std::string imu_topic_;
  std::string wheel_joint_state_topic_;
  bool odom_twist_in_body_frame_{true};
  int estimation_log_period_ms_{2000};
  std::vector<std::string> chassis_wheel_joint_names_;
  bool has_odom_{false};
  bool has_trajectory_{false};
  Pose2D pose_;
  Twist2D odom_twist_;
  Twist2D last_cmd_;
  bool has_last_cmd_{false};
  rclcpp::Time last_control_time_;
  bool has_last_control_time_{false};
  rclcpp::Time trajectory_start_time_;
  std::vector<TrajectoryPoint> trajectory_;
  PreviewController controller_;
  Se2MpcPreview mpc_preview_;
  Se2MpcController mpc_controller_;
  ChassisKinematics chassis_;
  HighRateStateEstimator estimator_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_sub_;
};

}  // namespace astra_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<astra_nav::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
