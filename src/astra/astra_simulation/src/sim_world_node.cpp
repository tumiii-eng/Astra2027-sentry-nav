#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "astra_common/common.hpp"
#include "astra_perception/pointcloud_utils.hpp"

using namespace std::chrono_literals;

namespace astra_nav
{

class SimWorldNode : public rclcpp::Node
{
public:
  SimWorldNode()
  : Node("sim_world_node")
  {
    scenario_name_ = declare_parameter<std::string>("scenario_name", "基准狭窄通道");
    obstacle_layout_ = declare_parameter<std::string>("obstacle_layout", "narrow_passage");
    pose_.x = declare_parameter("initial_x", 0.0);
    pose_.y = declare_parameter("initial_y", 0.0);
    pose_.yaw = declare_parameter("initial_yaw", 0.0);
    motion_enabled_ = declare_parameter("motion_enabled", true);
    update_rate_hz_ = declare_parameter("update_rate_hz", 20.0);
    point_rate_hz_ = declare_parameter("point_rate_hz", 10.0);
    goal_x_ = declare_parameter("goal_x", 5.0);
    goal_y_ = declare_parameter("goal_y", 2.4);
    max_range_ = declare_parameter("max_range", 8.0);
    obstacle_height_ = declare_parameter("obstacle_height", 0.8);

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/sim/odom", 10);
    point_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/points", 10);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 1);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_ = *msg;
      });

    build_obstacles();
    last_update_time_ = now();
    update_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / update_rate_hz_)),
      std::bind(&SimWorldNode::on_update, this));
    point_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / point_rate_hz_)),
      std::bind(&SimWorldNode::publish_points_and_goal, this));

    RCLCPP_INFO(
      get_logger(),
      "本地仿真节点已启动，场景=%s，障碍布局=%s，初始位姿=(%.2f, %.2f, %.2f)，目标=(%.2f, %.2f)，运动积分=%s。",
      scenario_name_.c_str(), obstacle_layout_.c_str(), pose_.x, pose_.y, pose_.yaw,
      goal_x_, goal_y_, motion_enabled_ ? "开启" : "关闭");
  }

private:
  void build_obstacles()
  {
    obstacles_.clear();
    if (obstacle_layout_ == "open_field") {
      return;
    }
    if (obstacle_layout_ == "narrow_passage") {
      add_narrow_passage();
      return;
    }
    if (obstacle_layout_ == "near_start") {
      add_narrow_passage();
      add_block(0.35, 0.0, 0.35, 0.35, obstacle_height_);
      return;
    }
    if (obstacle_layout_ == "goal_near_obstacle") {
      add_narrow_passage();
      add_block(goal_x_ - 0.35, goal_y_, 0.45, 0.55, obstacle_height_);
      return;
    }
    if (obstacle_layout_ == "cluttered") {
      add_narrow_passage();
      add_block(1.4, -0.45, 0.45, 0.45, obstacle_height_);
      add_block(3.2, 2.1, 0.55, 0.45, obstacle_height_);
      add_block(4.4, 0.35, 0.45, 0.65, obstacle_height_);
      return;
    }

    RCLCPP_WARN(
      get_logger(), "未知障碍布局：%s，回退为基准狭窄通道。", obstacle_layout_.c_str());
    add_narrow_passage();
  }

  void add_narrow_passage()
  {
    // 两条长墙构造一个狭窄通道，再加一个独立障碍块。
    for (double x = 1.0; x <= 4.2; x += 0.12) {
      obstacles_.push_back({x, 0.75, obstacle_height_});
      obstacles_.push_back({x, 1.45, obstacle_height_});
    }
    for (double y = -1.0; y <= 0.2; y += 0.12) {
      obstacles_.push_back({2.8, y, obstacle_height_});
    }
    for (double x = -1.4; x <= -0.6; x += 0.12) {
      for (double y = 1.2; y <= 2.0; y += 0.12) {
        obstacles_.push_back({x, y, obstacle_height_});
      }
    }
  }

  void add_block(double center_x, double center_y, double size_x, double size_y, double height)
  {
    const double half_x = 0.5 * std::max(0.0, size_x);
    const double half_y = 0.5 * std::max(0.0, size_y);
    for (double x = center_x - half_x; x <= center_x + half_x; x += 0.12) {
      for (double y = center_y - half_y; y <= center_y + half_y; y += 0.12) {
        obstacles_.push_back({x, y, height});
      }
    }
  }

  void on_update()
  {
    const auto current = now();
    const double dt = std::max(0.001, (current - last_update_time_).seconds());
    last_update_time_ = current;

    if (motion_enabled_) {
      const double cy = std::cos(pose_.yaw);
      const double sy = std::sin(pose_.yaw);
      const double vx_world = cy * last_cmd_.linear.x - sy * last_cmd_.linear.y;
      const double vy_world = sy * last_cmd_.linear.x + cy * last_cmd_.linear.y;
      pose_.x += vx_world * dt;
      pose_.y += vy_world * dt;
      pose_.yaw = normalize_angle(pose_.yaw + last_cmd_.angular.z * dt);
    }

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = pose_.x;
    odom.pose.pose.position.y = pose_.y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = quaternion_from_yaw(pose_.yaw);
    odom.twist.twist = last_cmd_;
    odom_pub_->publish(odom);
  }

  void publish_points_and_goal()
  {
    const auto stamp = now();
    std::vector<Point3D> visible;
    visible.reserve(obstacles_.size());
    for (const auto & p : obstacles_) {
      if (std::hypot(p.x - pose_.x, p.y - pose_.y) <= max_range_) {
        for (double z = 0.05; z <= p.z; z += 0.18) {
          visible.push_back({p.x, p.y, z});
        }
      }
    }
    point_pub_->publish(make_xyz_cloud(visible, "odom", stamp));

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp = stamp;
    goal.header.frame_id = "odom";
    goal.pose.position.x = goal_x_;
    goal.pose.position.y = goal_y_;
    goal.pose.orientation = quaternion_from_yaw(0.0);
    goal_pub_->publish(goal);
  }

  std::string scenario_name_;
  std::string obstacle_layout_;
  bool motion_enabled_{true};
  double update_rate_hz_{20.0};
  double point_rate_hz_{10.0};
  double goal_x_{5.0};
  double goal_y_{2.4};
  double max_range_{8.0};
  double obstacle_height_{0.8};
  Pose2D pose_;
  geometry_msgs::msg::Twist last_cmd_;
  std::vector<Point3D> obstacles_;
  rclcpp::Time last_update_time_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::TimerBase::SharedPtr point_timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
};

}  // namespace astra_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<astra_nav::SimWorldNode>());
  rclcpp::shutdown();
  return 0;
}
