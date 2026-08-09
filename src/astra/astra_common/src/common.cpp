#include "astra_common/common.hpp"

#include <algorithm>

namespace astra_nav
{

double normalize_angle(double angle)
{
  // 用 remainder 把角度归一化到 (-pi, pi]，O(1) 且对极大输入也安全，
  // 避免旧的 while 循环在异常大角度输入下退化为大量迭代。
  // 注意：std::remainder 的结果落在闭区间 [-pi, pi]，且在 pi 的奇数倍处按
  // “四舍五入到偶数”可能返回 -pi（例如 remainder(3*pi, 2*pi) = -pi）。
  // 这与本函数约定的半开区间 (-pi, pi] 不一致，会让“掉头”这个奇异角在 -pi/+pi
  // 之间二义。这里把下边界 -pi 统一翻到 +pi，使返回值严格落在 (-pi, pi]，
  // 与注释、与下游 heading 误差处理保持一致。
  double wrapped = std::remainder(angle, 2.0 * M_PI);
  if (wrapped <= -M_PI) {
    wrapped = M_PI;
  }
  return wrapped;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  const double half = 0.5 * yaw;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(half);
  q.w = std::cos(half);
  return q;
}

double distance_2d(const Point2D & a, const Point2D & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double heading_between(const Point2D & a, const Point2D & b, double fallback)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  if (std::abs(dx) + std::abs(dy) < 1e-9) {
    return fallback;
  }
  return std::atan2(dy, dx);
}

std::vector<Point2D> resample_polyline(const std::vector<Point2D> & points, double spacing)
{
  if (points.empty()) {
    return {};
  }
  if (points.size() == 1) {
    return points;
  }

  spacing = std::max(spacing, 1e-3);
  std::vector<Point2D> output;
  output.push_back(points.front());

  double carry = 0.0;
  Point2D prev = points.front();
  for (std::size_t i = 1; i < points.size(); ++i) {
    const Point2D curr = points[i];
    const double dx = curr.x - prev.x;
    const double dy = curr.y - prev.y;
    const double length = std::hypot(dx, dy);
    if (length < 1e-9) {
      prev = curr;
      continue;
    }

    const double ux = dx / length;
    const double uy = dy / length;
    double dist = spacing - carry;
    while (dist <= length + 1e-9) {
      output.push_back({prev.x + ux * dist, prev.y + uy * dist});
      dist += spacing;
    }
    carry = length - (dist - spacing);
    prev = curr;
  }

  if (distance_2d(output.back(), points.back()) > 1e-6) {
    output.push_back(points.back());
  }
  return output;
}

namespace
{

// 前向-后向速度² 可达传播 + 梯形时间积分。
// 逐行对应上游 HWSentryNav26 speed_profile_optimizer.cpp 的 reachable_seed()（第 314-347 行）
// 与 make_profile()（第 351-370 行）。nodes 为单调不减的累计【等效路程】。
std::vector<double> integrate_reachable_times(
  const std::vector<double> & nodes, double max_velocity, double max_acceleration,
  double start_speed)
{
  std::vector<double> times;
  if (nodes.empty()) {
    return times;
  }
  times.reserve(nodes.size());
  times.push_back(0.0);
  if (nodes.size() == 1) {
    return times;
  }

  const double vmax = std::max(max_velocity, 1e-3);
  const double amax = std::max(max_acceleration, 1e-3);
  const double speed_squared_upper = vmax * vmax;

  // 起点实测速度允许暂时超出 nominal 包络（上游 limits.front().speed_squared_upper
  // 与 measured_initial_speed_squared 取 max），随后由终点零速反向裁成可达值。
  const double start = std::max(0.0, start_speed);
  std::vector<double> speed_squared(nodes.size(), 0.0);
  speed_squared[0] = start * start;

  // 前向：加速可达性 z[i+1] = min(z_upper, z[i] + 2·a·Δs)。
  for (std::size_t i = 0; i + 1 < nodes.size(); ++i) {
    const double ds = std::max(0.0, nodes[i + 1] - nodes[i]);
    speed_squared[i + 1] = std::min(speed_squared_upper, speed_squared[i] + 2.0 * amax * ds);
  }
  // 终点停稳。
  speed_squared.back() = 0.0;
  // 后向：刹车可达性 z[i] = min(z[i], z[i+1] + 2·a·Δs)。
  for (std::size_t reverse = nodes.size() - 1; reverse > 0; --reverse) {
    const std::size_t i = reverse - 1;
    const double ds = std::max(0.0, nodes[i + 1] - nodes[i]);
    speed_squared[i] = std::min(speed_squared[i], speed_squared[i + 1] + 2.0 * amax * ds);
  }

  // 时间：匀加速段 Δt = 2Δs / (v_i + v_{i+1})。
  for (std::size_t i = 0; i + 1 < nodes.size(); ++i) {
    const double ds = std::max(0.0, nodes[i + 1] - nodes[i]);
    const double v = std::sqrt(std::max(speed_squared[i], 0.0));
    const double v_next = std::sqrt(std::max(speed_squared[i + 1], 0.0));
    const double velocity_sum = v + v_next;
    double dt = 0.0;
    if (velocity_sum > 1e-6) {
      dt = 2.0 * ds / velocity_sum;
    } else {
      // 两端速度同时为零（例如整条路径只有一段）：该段就是静止到静止的梯形，
      // 用其精确时长兜底。上游此处直接判定剖面不可用并返回空。
      dt = 2.0 * std::sqrt(ds / amax);
    }
    // 时长下界只为保证 MINCO 分段时间严格为正。退化零长段给 0.05 s，
    // 正常段只给 1 ms —— 固定 0.05 s 的下界会在密路标点/高速时反而【拉长】总时长。
    times.push_back(times.back() + std::max(dt, ds < 1e-6 ? 0.05 : 1.0e-3));
  }
  return times;
}

}  // namespace

std::vector<double> cumulative_times(
  const std::vector<Point2D> & points, double max_velocity, double max_acceleration,
  double start_speed)
{
  std::vector<double> nodes;
  nodes.reserve(points.size());
  double arc_length = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (i > 0) {
      arc_length += distance_2d(points[i - 1], points[i]);
    }
    nodes.push_back(arc_length);
  }
  return integrate_reachable_times(nodes, max_velocity, max_acceleration, start_speed);
}

std::vector<double> cumulative_times_with_turning(
  const std::vector<Point2D> & points, double max_velocity, double max_acceleration,
  double turn_weight, double start_speed)
{
  if (points.empty()) {
    return {};
  }
  std::vector<double> nodes;
  nodes.reserve(points.size());
  nodes.push_back(0.0);
  if (points.size() == 1) {
    return {0.0};
  }

  const double k_turn = std::max(0.0, turn_weight);

  for (std::size_t i = 1; i < points.size(); ++i) {
    // s1：本段直线长度。
    const double s1 = distance_2d(points[i - 1], points[i]);
    // s2：本段终点处的转角变化量（与下一段的夹角）。末段无后继，转角记为 0。
    double s2 = 0.0;
    if (i + 1 < points.size()) {
      const double in_x = points[i].x - points[i - 1].x;
      const double in_y = points[i].y - points[i - 1].y;
      const double out_x = points[i + 1].x - points[i].x;
      const double out_y = points[i + 1].y - points[i].y;
      const double in_norm = std::hypot(in_x, in_y);
      const double out_norm = std::hypot(out_x, out_y);
      if (in_norm > 1e-9 && out_norm > 1e-9) {
        double cos_angle =
          (in_x * out_x + in_y * out_y) / (in_norm * out_norm);
        cos_angle = std::clamp(cos_angle, -1.0, 1.0);
        s2 = std::acos(cos_angle);  // 转角弧度，0=直行，pi=掉头
      }
    }
    // 等效路程：直线段 + 转角等效行程。
    const double s = s1 + k_turn * s2;
    nodes.push_back(nodes.back() + std::max(s, 0.0));
  }
  return integrate_reachable_times(nodes, max_velocity, max_acceleration, start_speed);
}

}  // namespace astra_nav

