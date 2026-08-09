// 全局障碍图节点（阶段1+2）
//
// 作用：给 Astra 规划器提供【覆盖全场】的障碍图，替代原来随机器人滚动的局部 occupancy_node。
// 本节点对应 HWSentryNav26 的 map_server：只产出“障碍源二值图”，膨胀成 0-255 连续代价场
// 的工作交给下游 planner（Astra 的传输通道是 int8 的 OccupancyGrid，装不下 0-255，
// 因此把两层分别发出、由 planner 各自膨胀再 max 合并，语义与上游
// hard_cost = global_static->merge(dynamic_union) 完全一致）。
//   - 静态层（阶段1）：订阅先验地图 /map（map 帧，含原生障碍 + 用户手动标注）-> output_topic。
//   - 动态层（阶段2）：订阅实时 terrain_map（odom 帧，intensity=离地高度），经 map<-odom TF
//     变换到 map 帧后按下述多级过滤判定动态障碍，带时间衰减 -> dynamic_output_topic。
//
// 动态层多级过滤（直接复用 HWSentryNav26 map_server 的判定链，逐级语义一致）：
//   1) intensity 双边门限：只有 [dynamic_intensity_min, dynamic_intensity_max] 内的点算障碍。
//      原实现只有下界，离地高度无上界 -> 天花板/穹顶等高处点也被当成地面障碍写进图里，
//      是“机器人飞到 z=5.7m”（BUG-5）的直接来源；上界沿用 pb2025 nav2 侧已验证的 2.0。
//   2) 统计离群点过滤（SOR）：均值+std_mul*标准差为阈值剔除孤立点，等价上游
//      remove_statistical_outliers（这里直接用 PCL 的 StatisticalOutlierRemoval，同一算法）。
//   3) 投影密度门限：按地图分辨率把“点/m²”换算成每格最小点数，单格点数不足不算障碍。
//   4) 连通域面积过滤：剔除不具备空间连续性的孤立小斑块（面积判定而非形态学开运算，
//      开运算会侵蚀细长的真实障碍物）。
// bypass_dynamic_obstacle 置真时动态层恒发空图，用于隔离调试。
//
// 注意：本节点是 pb2025<->Astra 的对接件，不改 Astra/pb2025 源码。

#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/utils.h>

#include <opencv2/opencv.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace astra_nav
{

namespace
{

// 物理量 -> 栅格数的换算，直接复制 HWSentryNav26 map_server/src/utils.cpp 的
// minimum_density_count / minimum_area_cells，计算方法逐字一致。
int minimum_density_count(const double density_per_m2, const double resolution)
{
  const double count = std::ceil(density_per_m2 * resolution * resolution);
  return std::max(1, static_cast<int>(count));
}

int minimum_area_cells(const double area_m2, const double resolution)
{
  if (area_m2 <= 0.0) {
    return 0;
  }
  return static_cast<int>(std::ceil(area_m2 / (resolution * resolution)));
}

}  // namespace

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
    dynamic_output_topic_ = declare_parameter<std::string>(
      "dynamic_output_topic", "/astra/dynamic_obstacle_grid");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    // 障碍膨胀半径（米）。0 表示不在本节点膨胀（推荐）：膨胀统一由 planner 按
    // 静态/动态两套参数各自做一次，避免双重膨胀切断窄走廊连通域（BUG-1）。
    inflation_radius_ = declare_parameter<double>("inflation_radius", 0.0);
    // 动态障碍 intensity（离地高度）双边门限，对齐 nav2 的
    // min_obstacle_intensity / max_obstacle_intensity。上界必须有，否则高处点会被
    // 当成地面障碍（BUG-5 飞车）。
    dyn_intensity_min_ = declare_parameter<double>("dynamic_intensity_min", 0.1);
    dyn_intensity_max_ = declare_parameter<double>("dynamic_intensity_max", 2.0);
    // 动态障碍保留时间（秒），超时的动态障碍格清除。
    dyn_decay_sec_ = declare_parameter<double>("dynamic_decay_sec", 1.5);
    // 是否启用动态层（阶段1可设 false 只用静态先验图）。
    enable_dynamic_ = declare_parameter<bool>("enable_dynamic", true);
    // 绕过动态障碍检测：动态层恒发空图，用于隔离调试（对齐上游 bypass_dynamic_obstacle）。
    bypass_dynamic_obstacle_ = declare_parameter<bool>("bypass_dynamic_obstacle", false);
    // SOR 统计离群点过滤（对齐上游 sor_num_neighbors / sor_std_mul）。
    sor_num_neighbors_ = declare_parameter<int>("sor_num_neighbors", 12);
    sor_std_mul_ = declare_parameter<double>("sor_std_mul", 2.0);
    // 投影密度与连通域面积门限（对齐上游同名参数）。
    min_projected_point_density_per_m2_ =
      declare_parameter<double>("min_projected_point_density_per_m2", 800.0);
    min_obstacle_cluster_area_m2_ =
      declare_parameter<double>("min_obstacle_cluster_area_m2", 0.0075);
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
    dynamic_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(dynamic_output_topic_, 5);
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
      "全局障碍图节点已启动：静态源=%s 动态源=%s(启用=%s 绕过=%s) 静态输出=%s 动态输出=%s "
      "膨胀=%.2fm intensity门限=[%.2f, %.2f] SOR=%d邻居/%.1fσ 密度=%.0f点每m² 连通域=%.4f m²。",
      map_topic_.c_str(), terrain_topic_.c_str(), enable_dynamic_ ? "是" : "否",
      bypass_dynamic_obstacle_ ? "是" : "否",
      output_topic_.c_str(), dynamic_output_topic_.c_str(), inflation_radius_,
      dyn_intensity_min_, dyn_intensity_max_, sor_num_neighbors_, sor_std_mul_,
      min_projected_point_density_per_m2_, min_obstacle_cluster_area_m2_);
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
    // 分辨率相关门限按上游做法在拿到地图分辨率后换算一次。
    const double resolution = std::max(static_cast<double>(msg.info.resolution), 1.0e-3);
    min_points_per_cell_ =
      minimum_density_count(min_projected_point_density_per_m2_, resolution);
    min_obstacle_cluster_cells_ =
      minimum_area_cells(min_obstacle_cluster_area_m2_, resolution);
    RCLCPP_INFO(
      get_logger(),
      "已接收先验静态地图：%ux%u @%.3fm 原点(%.2f,%.2f)；"
      "分辨率相关门限：单格最少点数=%d，连通域最小格数=%d。",
      msg.info.width, msg.info.height, msg.info.resolution,
      msg.info.origin.position.x, msg.info.origin.position.y,
      min_points_per_cell_, min_obstacle_cluster_cells_);
  }

  // SOR：均值 + std_mul*标准差 为阈值剔除孤立点。与上游
  // remove_statistical_outliers 同一算法，这里用 PCL 的实现（容器里现成的 PCL 1.12）。
  pcl::PointCloud<pcl::PointXYZ>::Ptr remove_statistical_outliers(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) const
  {
    auto filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    const int knn = std::max(1, sor_num_neighbors_);
    // 点数不足以做 K 近邻统计时按上游做法原样返回，不做过滤。
    if (cloud->empty() || cloud->size() < static_cast<std::size_t>(knn) + 1) {
      return cloud;
    }
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(knn);
    sor.setStddevMulThresh(sor_std_mul_);
    sor.filter(*filtered);
    return filtered;
  }

  // 逐格点数阈值只做单元格内的密度判定，无法区分“成片的障碍物”与“散落的噪声格”，
  // 因此再按连通域面积剔除不具备空间连续性的格子。直接复制上游 create_obstacle_mask。
  cv::Mat create_obstacle_mask(const pcl::PointCloud<pcl::PointXYZ> & dynamic_points) const
  {
    const auto & info = static_map_.info;
    const int W = static_cast<int>(info.width);
    const int H = static_cast<int>(info.height);
    cv::Mat counts = cv::Mat::zeros(H, W, CV_32SC1);
    cv::Mat mask = cv::Mat::zeros(H, W, CV_8UC1);
    for (const auto & pt : dynamic_points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) {
        continue;
      }
      const int gx = static_cast<int>(
        std::floor((pt.x - info.origin.position.x) / info.resolution));
      const int gy = static_cast<int>(
        std::floor((pt.y - info.origin.position.y) / info.resolution));
      if (gx < 0 || gy < 0 || gx >= W || gy >= H) {
        continue;
      }
      int & cell = counts.at<int>(gy, gx);
      if (cell < min_points_per_cell_) {
        ++cell;
      }
      if (cell >= min_points_per_cell_) {
        mask.at<std::uint8_t>(gy, gx) = 255;
      }
    }
    if (min_obstacle_cluster_cells_ <= 1) {
      return mask;
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int label_count =
      cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);
    // 面积过滤而非形态学开运算：开运算会侵蚀细长的真实障碍物，而面积判定只针对
    // 孤立小斑块，语义上正好对应“不具备空间连续性”。
    std::vector<std::uint8_t> label_kept(
      static_cast<std::size_t>(std::max(1, label_count)), 0);
    bool dropped_any = false;
    for (int label = 1; label < label_count; ++label) {
      const bool kept =
        stats.at<int>(label, cv::CC_STAT_AREA) >= min_obstacle_cluster_cells_;
      label_kept[static_cast<std::size_t>(label)] = kept ? 1 : 0;
      dropped_any = dropped_any || !kept;
    }
    if (!dropped_any) {
      return mask;
    }

    for (int y = 0; y < H; ++y) {
      const int * label_row = labels.ptr<int>(y);
      std::uint8_t * mask_row = mask.ptr<std::uint8_t>(y);
      for (int x = 0; x < W; ++x) {
        if (!label_kept[static_cast<std::size_t>(label_row[x])]) {
          mask_row[x] = 0;
        }
      }
    }
    return mask;
  }

  void on_terrain(const sensor_msgs::msg::PointCloud2 & msg)
  {
    if (!has_static_ || bypass_dynamic_obstacle_) return;
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

    const auto & info = static_map_.info;
    // 1) intensity 双边门限 + 变换到 map 帧。
    auto candidates = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    const std::size_t n = static_cast<std::size_t>(msg.width) * msg.height;
    candidates->points.reserve(n);
    std::size_t rejected_intensity = 0;
    for (std::size_t i = 0; i < n; ++i) {
      const std::size_t base = i * msg.point_step;
      if (base + msg.point_step > msg.data.size()) break;
      float x, y; float inten;
      std::memcpy(&x, &msg.data[base + ox], sizeof(float));
      std::memcpy(&y, &msg.data[base + oy], sizeof(float));
      std::memcpy(&inten, &msg.data[base + oi], sizeof(float));
      // 离地太低=可通行地面；离地太高=天花板/穹顶等非地面障碍，不能算进平面障碍图。
      if (inten < dyn_intensity_min_ || inten > dyn_intensity_max_) {
        ++rejected_intensity;
        continue;
      }
      // 变到 map 帧
      const double mx = cy * x - sy * y + tx;
      const double my = sy * x + cy * y + ty;
      candidates->points.emplace_back(
        static_cast<float>(mx), static_cast<float>(my), 0.0F);
    }
    candidates->width = static_cast<std::uint32_t>(candidates->points.size());
    candidates->height = 1;
    candidates->is_dense = true;

    // 2) SOR 剔除孤立点。
    const auto denoised = remove_statistical_outliers(candidates);
    // 3)+4) 投影密度门限 + 连通域面积过滤。
    const cv::Mat mask = create_obstacle_mask(*denoised);

    const double now = this->now().seconds();
    std::size_t marked = 0;
    for (int gy = 0; gy < static_cast<int>(info.height); ++gy) {
      const std::uint8_t * mask_row = mask.ptr<std::uint8_t>(gy);
      for (int gx = 0; gx < static_cast<int>(info.width); ++gx) {
        if (mask_row[gx] == 0) continue;
        dyn_stamp_[static_cast<std::size_t>(gy * static_cast<int>(info.width) + gx)] = now;
        ++marked;
      }
    }
    RCLCPP_DEBUG(
      get_logger(),
      "动态障碍过滤：输入=%zu 点，intensity 门限剔除=%zu，SOR 后=%zu 点，成图格数=%zu。",
      n, rejected_intensity, denoised->points.size(), marked);
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

    // 两层分开出图：静态=先验地图(>50 占据)，动态=未过期的动态障碍格。
    // 下游 planner 对两层各用一套膨胀参数（静态 global_map.inflation、动态
    // local_map.inflation），再按 max 合并成一张 0-255 硬代价场——与上游
    // hard_cost = global_static->merge(dynamic_union) 一致。
    std::vector<std::uint8_t> static_occ(static_cast<std::size_t>(W * H), 0);
    std::vector<std::uint8_t> dynamic_occ(static_cast<std::size_t>(W * H), 0);
    for (std::size_t i = 0; i < static_occ.size(); ++i) {
      static_occ[i] = static_map_.data[i] > 50 ? 1 : 0;  // 先验静态障碍(含手动标注)
      if (enable_dynamic_ && !bypass_dynamic_obstacle_ &&
        dyn_stamp_[i] > 0.0 && (now - dyn_stamp_[i]) <= dyn_decay_sec_)
      {
        dynamic_occ[i] = 1;
      }
    }

    pub_->publish(make_grid(static_occ, info));
    dynamic_pub_->publish(make_grid(dynamic_occ, info));
  }

  // 把二值占据向量封成 OccupancyGrid。inflation_radius_>0 时在本节点内做一次硬膨胀，
  // 默认 0（不膨胀），把膨胀交给 planner 的连续代价场统一处理。
  nav_msgs::msg::OccupancyGrid make_grid(
    const std::vector<std::uint8_t> & occ, const nav_msgs::msg::MapMetaData & info) const
  {
    const int W = info.width, H = info.height;
    const double res = std::max(static_cast<double>(info.resolution), 1e-3);
    const int r = static_cast<int>(std::ceil(inflation_radius_ / res));
    nav_msgs::msg::OccupancyGrid out;
    out.header.stamp = this->now();
    out.header.frame_id = map_frame_;
    out.info = info;
    out.data.assign(static_cast<std::size_t>(W * H), 0);
    if (r <= 0) {
      for (std::size_t i = 0; i < occ.size(); ++i) out.data[i] = occ[i] ? 100 : 0;
      return out;
    }
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
    return out;
  }

  // 静态先验地图（收到一次即锁存）。
  nav_msgs::msg::OccupancyGrid static_map_;
  bool has_static_{false};

  // 动态障碍：记录被实时障碍点命中的格子索引 + 命中时间，用于衰减。
  std::vector<double> dyn_stamp_;  // 与 static_map_ 同尺寸，存最近命中时间(秒)，0=从未命中

  std::string map_topic_, terrain_topic_, output_topic_, dynamic_output_topic_, map_frame_;
  std::string in_odom_topic_, out_odom_topic_;
  double inflation_radius_{0.0};
  double dyn_intensity_min_{0.1};
  double dyn_intensity_max_{2.0};
  double dyn_decay_sec_{1.5};
  bool enable_dynamic_{true};
  bool bypass_dynamic_obstacle_{false};
  int sor_num_neighbors_{12};
  double sor_std_mul_{2.0};
  double min_projected_point_density_per_m2_{800.0};
  double min_obstacle_cluster_area_m2_{0.0075};
  // 由地图分辨率换算得到的格数门限，收到静态地图时更新一次。
  int min_points_per_cell_{1};
  int min_obstacle_cluster_cells_{0};
  double publish_rate_{5.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dynamic_pub_;
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
