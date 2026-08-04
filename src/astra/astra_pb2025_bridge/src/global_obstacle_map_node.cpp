// 全局障碍图节点（阶段1+2）
//
// 作用：给 Astra 规划器提供一张【覆盖全场】的障碍图，替代原来随机器人滚动的局部 occupancy_node。
//   - 静态层（阶段1）：订阅先验地图 /map（map 帧，含原生障碍 + 用户手动标注），作为全局障碍底图。
//   - 动态层（阶段2）：订阅实时 terrain_map（odom 帧，intensity=离地高度），把 intensity>阈值 的障碍点
//                      经 map<-odom TF 变换到 map 帧后叠加到底图上，带时间衰减（动态障碍会过期消失）。
//   - 膨胀：按机器人半径 + 安全裕度做一次膨胀，等价 nav2 inflation_layer。
// 输出：/astra/global_obstacle_grid（nav_msgs/OccupancyGrid，map 帧，全场尺寸=先验地图尺寸）。
//
// 注意：本节点是 pb2025<->Astra 的对接件，不改 Astra/pb2025 源码。

#include <cmath>
#include <cstring>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/utils.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace astra_nav
{

class GlobalObstacleMapNode : public rclcpp::Node
{
public:
  GlobalObstacleMapNode()
  : Node("global_obstacle_map_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    map_topic_ = declare_parameter<std::string>("map_topic", "map");
    terrain_topic_ = declare_parameter<std::string>("terrain_topic", "terrain_map");
    output_topic_ = declare_parameter<std::string>("output_topic", "/astra/global_obstacle_grid");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    // 障碍膨胀半径（米），≈机器人半径+安全裕度。
    inflation_radius_ = declare_parameter<double>("inflation_radius", 0.30);
    // 动态障碍判定：terrain_map 点 intensity（离地高度）超过此值算障碍（对齐 nav2 min_obstacle_intensity）。
    dyn_intensity_min_ = declare_parameter<double>("dynamic_intensity_min", 0.1);
    // 动态障碍保留时间（秒），超时的动态障碍格清除。
    dyn_decay_sec_ = declare_parameter<double>("dynamic_decay_sec", 1.5);
    // 是否启用动态层（阶段1可设 false 只用静态先验图）。
    enable_dynamic_ = declare_parameter<bool>("enable_dynamic", true);
    publish_rate_ = declare_parameter<double>("publish_rate_hz", 5.0);
    // odom 重投影：订阅 odom 帧里程计，查 map<-odom TF，输出 map 帧里程计供 Astra 全局规划。
    in_odom_topic_ = declare_parameter<std::string>("input_odom_topic", "odometry");
    out_odom_topic_ = declare_parameter<std::string>("output_odom_topic", "/astra/odom");

    // 先验静态地图：transient_local QoS 才能收到 map_server 的锁存发布。
    rclcpp::QoS map_qos(1);
    map_qos.transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos, [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        on_static_map(*msg);
      });

    if (enable_dynamic_) {
      terrain_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        terrain_topic_, 10, [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          on_terrain(*msg);
        });
    }

    pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, 5);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(out_odom_topic_, 20);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      in_odom_topic_, 50, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
        on_odom(*msg);
      });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_)),
      [this]() { publish_global_grid(); });

    RCLCPP_INFO(
      get_logger(),
      "全局障碍图节点已启动：静态源=%s 动态源=%s(启用=%s) 输出=%s 膨胀=%.2fm。",
      map_topic_.c_str(), terrain_topic_.c_str(), enable_dynamic_ ? "是" : "否",
      output_topic_.c_str(), inflation_radius_);
  }

private:
  // 把 odom 帧里程计重投影到 map 帧后转发（供 Astra 在 map 帧全局规划）。
  void on_odom(const nav_msgs::msg::Odometry & in)
  {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(map_frame_, in.header.frame_id, tf2::TimePointZero);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "odom 重投影等待 %s<-%s TF：%s", map_frame_.c_str(), in.header.frame_id.c_str(), e.what());
      return;
    }
    const double tx = tf.transform.translation.x, ty = tf.transform.translation.y;
    const double tqz = tf.transform.rotation.z, tqw = tf.transform.rotation.w;
    const double tyaw = std::atan2(2.0 * tqw * tqz, 1.0 - 2.0 * tqz * tqz);
    const double c = std::cos(tyaw), s = std::sin(tyaw);

    nav_msgs::msg::Odometry out = in;
    out.header.frame_id = map_frame_;
    const double px = in.pose.pose.position.x, py = in.pose.pose.position.y;
    out.pose.pose.position.x = c * px - s * py + tx;
    out.pose.pose.position.y = s * px + c * py + ty;
    // 朝向：叠加 map<-odom 的 yaw。
    const double iqz = in.pose.pose.orientation.z, iqw = in.pose.pose.orientation.w;
    const double iyaw = std::atan2(2.0 * iqw * iqz, 1.0 - 2.0 * iqz * iqz);
    const double oyaw = iyaw + tyaw;
    out.pose.pose.orientation.x = 0.0;
    out.pose.pose.orientation.y = 0.0;
    out.pose.pose.orientation.z = std::sin(oyaw / 2.0);
    out.pose.pose.orientation.w = std::cos(oyaw / 2.0);
    // twist 在机体系，不变。
    odom_pub_->publish(out);
  }

  void on_static_map(const nav_msgs::msg::OccupancyGrid & msg)
  {
    static_map_ = msg;
    has_static_ = true;
    dyn_stamp_.assign(msg.data.size(), 0.0);
    RCLCPP_INFO(
      get_logger(), "已接收先验静态地图：%ux%u @%.3fm 原点(%.2f,%.2f)。",
      msg.info.width, msg.info.height, msg.info.resolution,
      msg.info.origin.position.x, msg.info.origin.position.y);
  }

  void on_terrain(const sensor_msgs::msg::PointCloud2 & msg)
  {
    if (!has_static_) return;
    // 找 x/y/z/intensity 字段偏移。
    int ox = -1, oy = -1, oz = -1, oi = -1;
    for (const auto & f : msg.fields) {
      if (f.name == "x") ox = f.offset;
      else if (f.name == "y") oy = f.offset;
      else if (f.name == "z") oz = f.offset;
      else if (f.name == "intensity") oi = f.offset;
    }
    if (ox < 0 || oy < 0 || oi < 0 || msg.point_step == 0) return;

    // terrain_map 在 odom 帧，需变换到 map 帧。查 map<-source 的 TF。
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(
        map_frame_, msg.header.frame_id, tf2::TimePointZero);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "等待 %s<-%s 的 TF：%s", map_frame_.c_str(), msg.header.frame_id.c_str(), e.what());
      return;
    }
    const double tx = tf.transform.translation.x, ty = tf.transform.translation.y;
    const double qz = tf.transform.rotation.z, qw = tf.transform.rotation.w;
    const double yaw = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);
    const double cy = std::cos(yaw), sy = std::sin(yaw);

    const double now = this->now().seconds();
    const auto & info = static_map_.info;
    const std::size_t n = static_cast<std::size_t>(msg.width) * msg.height;
    for (std::size_t i = 0; i < n; ++i) {
      const std::size_t base = i * msg.point_step;
      if (base + msg.point_step > msg.data.size()) break;
      float x, y; float inten;
      std::memcpy(&x, &msg.data[base + ox], sizeof(float));
      std::memcpy(&y, &msg.data[base + oy], sizeof(float));
      std::memcpy(&inten, &msg.data[base + oi], sizeof(float));
      if (inten < dyn_intensity_min_) continue;  // 非障碍点（离地太低=可通行地面）
      // 变到 map 帧
      const double mx = cy * x - sy * y + tx;
      const double my = sy * x + cy * y + ty;
      const int gx = static_cast<int>((mx - info.origin.position.x) / info.resolution);
      const int gy = static_cast<int>((my - info.origin.position.y) / info.resolution);
      if (gx < 0 || gy < 0 || gx >= (int)info.width || gy >= (int)info.height) continue;
      dyn_stamp_[static_cast<std::size_t>(gy * info.width + gx)] = now;
    }
  }

  void publish_global_grid()
  {
    if (!has_static_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "尚未收到先验静态地图，暂不发布全局障碍图。");
      return;
    }
    const auto & info = static_map_.info;
    const int W = info.width, H = info.height;
    const double now = this->now().seconds();

    // 1) 原始占据：静态地图(>50 占据 或 未知<0 视为障碍以保守) + 未过期的动态障碍。
    std::vector<std::uint8_t> occ(static_cast<std::size_t>(W * H), 0);
    for (std::size_t i = 0; i < occ.size(); ++i) {
      const int8_t s = static_map_.data[i];
      bool blocked = (s > 50);  // 先验静态障碍(含手动标注)
      if (enable_dynamic_ && dyn_stamp_[i] > 0.0 && (now - dyn_stamp_[i]) <= dyn_decay_sec_) {
        blocked = true;  // 未过期的动态障碍
      }
      occ[i] = blocked ? 1 : 0;
    }

    // 2) 膨胀：对占据格做半径 inflation_radius_ 的膨胀（等价 inflation_layer 的硬占据部分）。
    const double res = std::max(static_cast<double>(info.resolution), 1e-3);
    const int r = static_cast<int>(std::ceil(inflation_radius_ / res));
    nav_msgs::msg::OccupancyGrid out;
    out.header.stamp = this->now();
    out.header.frame_id = map_frame_;
    out.info = info;
    out.data.assign(static_cast<std::size_t>(W * H), 0);
    if (r <= 0) {
      for (std::size_t i = 0; i < occ.size(); ++i) out.data[i] = occ[i] ? 100 : 0;
    } else {
      const int r2 = r * r;
      for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
          if (!occ[static_cast<std::size_t>(y * W + x)]) continue;
          for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
              if (dx * dx + dy * dy > r2) continue;
              const int nx = x + dx, ny = y + dy;
              if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
              out.data[static_cast<std::size_t>(ny * W + nx)] = 100;
            }
          }
        }
      }
    }
    pub_->publish(out);
  }

  // 静态先验地图（收到一次即锁存）。
  nav_msgs::msg::OccupancyGrid static_map_;
  bool has_static_{false};

  // 动态障碍：记录被实时障碍点命中的格子索引 + 命中时间，用于衰减。
  std::vector<double> dyn_stamp_;  // 与 static_map_ 同尺寸，存最近命中时间(秒)，0=从未命中

  std::string map_topic_, terrain_topic_, output_topic_, map_frame_;
  std::string in_odom_topic_, out_odom_topic_;
  double inflation_radius_{0.30};
  double dyn_intensity_min_{0.1};
  double dyn_decay_sec_{1.5};
  bool enable_dynamic_{true};
  double publish_rate_{5.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace astra_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<astra_nav::GlobalObstacleMapNode>());
  rclcpp::shutdown();
  return 0;
}
