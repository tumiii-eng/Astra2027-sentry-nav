#include <memory>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "astra_common/common.hpp"
#include "astra_mapping/obstacle_extractor.hpp"
#include "astra_perception/pointcloud_utils.hpp"
#include "astra_perception/rolling_occupancy_grid.hpp"

namespace astra_nav
{

class OccupancyNode : public rclcpp::Node
{
public:
  OccupancyNode()
  : Node("occupancy_node"), grid_(make_config()), extractor_(make_extractor_config())
  {
    point_topic_ = declare_parameter<std::string>("point_topic", "/points");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/astra/odom");
    obstacle_grid_topic_ = declare_parameter<std::string>("obstacle_grid_topic", "/astra/obstacle_grid");
    map_frame_ = declare_parameter<std::string>("map_frame", "odom");

    obstacle_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(obstacle_grid_topic_, 5);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_pose_.x = msg->pose.pose.position.x;
        latest_pose_.y = msg->pose.pose.position.y;
        latest_pose_.yaw = yaw_from_quaternion(msg->pose.pose.orientation);
        has_odom_ = true;
      });
    point_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      point_topic_, 10, [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        on_points(*msg);
      });

    RCLCPP_INFO(get_logger(), "三维占据图节点已启动，等待点云和里程计。");
  }

private:
  OccupancyConfig make_config()
  {
    OccupancyConfig config;
    config.resolution = declare_parameter("resolution", 0.1);
    config.size_x = declare_parameter("size_x", 10.0);
    config.size_y = declare_parameter("size_y", 10.0);
    config.size_z = declare_parameter("size_z", 1.2);
    config.z_min = declare_parameter("z_min", -0.15);
    config.log_odds_hit = declare_parameter("log_odds_hit", 0.85);
    config.log_odds_miss = declare_parameter("log_odds_miss", -0.35);
    config.occupied_threshold = declare_parameter("occupied_threshold", 0.6);
    config.fade_ticks = declare_parameter("fade_ticks", 35);
    config.max_points_per_update =
      static_cast<std::size_t>(declare_parameter("max_points_per_update", 2500));
    return config;
  }

  ObstacleExtractionConfig make_extractor_config()
  {
    ObstacleExtractionConfig config;
    config.height_obstacle_min = declare_parameter("height_obstacle_min", 0.18);
    config.column_occupancy_min = declare_parameter("column_occupancy_min", 0.18);
    return config;
  }

  void on_points(const sensor_msgs::msg::PointCloud2 & msg)
  {
    if (!has_odom_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "尚未收到里程计，暂不更新占据图。");
      return;
    }

    const auto points = read_xyz_points(msg, grid_.config().max_points_per_update);
    const Point3D robot{latest_pose_.x, latest_pose_.y, 0.0};
    const Point3D sensor{latest_pose_.x, latest_pose_.y, 0.35};
    grid_.update(points, sensor, robot);
    publish_obstacle_grid(msg.header.stamp);
  }

  void publish_obstacle_grid(const rclcpp::Time & stamp)
  {
    const auto obstacles = extractor_.extract(grid_);

    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.info.resolution = static_cast<float>(obstacles.resolution);
    msg.info.width = static_cast<std::uint32_t>(obstacles.width);
    msg.info.height = static_cast<std::uint32_t>(obstacles.height);
    msg.info.origin.position.x = obstacles.origin.x;
    msg.info.origin.position.y = obstacles.origin.y;
    msg.info.origin.orientation = quaternion_from_yaw(0.0);
    msg.data.resize(obstacles.data.size());
    for (std::size_t i = 0; i < obstacles.data.size(); ++i) {
      msg.data[i] = obstacles.data[i] != 0 ? 100 : 0;
    }
    obstacle_pub_->publish(msg);
  }

  std::string point_topic_;
  std::string odom_topic_;
  std::string obstacle_grid_topic_;
  std::string map_frame_;
  bool has_odom_{false};
  Pose2D latest_pose_;
  RollingOccupancyGrid3D grid_;
  ObstacleExtractor extractor_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr obstacle_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_sub_;
};

}  // namespace astra_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<astra_nav::OccupancyNode>());
  rclcpp::shutdown();
  return 0;
}

