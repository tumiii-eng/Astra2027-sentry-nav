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

std::vector<double> cumulative_times(
  const std::vector<Point2D> & points, double max_velocity, double max_acceleration)
{
  std::vector<double> times;
  if (points.empty()) {
    return times;
  }
  times.reserve(points.size());
  times.push_back(0.0);

  const double vmax = std::max(max_velocity, 1e-3);
  const double amax = std::max(max_acceleration, 1e-3);
  const double t_acc = vmax / amax;
  const double s_acc = 0.5 * amax * t_acc * t_acc;

  for (std::size_t i = 1; i < points.size(); ++i) {
    const double dist = distance_2d(points[i - 1], points[i]);
    double dt = 0.0;
    if (dist >= 2.0 * s_acc) {
      dt = 2.0 * t_acc + (dist - 2.0 * s_acc) / vmax;
    } else {
      dt = 2.0 * std::sqrt(std::max(dist, 0.0) / amax);
    }
    times.push_back(times.back() + std::max(dt, 0.05));
  }
  return times;
}

std::vector<double> cumulative_times_with_turning(
  const std::vector<Point2D> & points, double max_velocity, double max_acceleration,
  double turn_weight)
{
  std::vector<double> times;
  if (points.empty()) {
    return times;
  }
  times.reserve(points.size());
  times.push_back(0.0);
  if (points.size() == 1) {
    return times;
  }

  const double vmax = std::max(max_velocity, 1e-3);
  const double amax = std::max(max_acceleration, 1e-3);
  const double t_acc = vmax / amax;
  const double s_acc = 0.5 * amax * t_acc * t_acc;
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
    double dt = 0.0;
    if (s >= 2.0 * s_acc) {
      dt = 2.0 * t_acc + (s - 2.0 * s_acc) / vmax;
    } else {
      dt = 2.0 * std::sqrt(std::max(s, 0.0) / amax);
    }
    times.push_back(times.back() + std::max(dt, 0.05));
  }
  return times;
}

}  // namespace astra_nav

