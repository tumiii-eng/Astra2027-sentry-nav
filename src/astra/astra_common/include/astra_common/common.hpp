#pragma once

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/quaternion.hpp>

namespace astra_nav
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Twist2D
{
  double vx{0.0};
  double vy{0.0};
  double wz{0.0};
};

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Point3D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct TrajectoryPoint
{
  double t{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double vx{0.0};
  double vy{0.0};
  double ax{0.0};
  double ay{0.0};
};

double normalize_angle(double angle);
double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q);
geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw);
double distance_2d(const Point2D & a, const Point2D & b);
double heading_between(const Point2D & a, const Point2D & b, double fallback = 0.0);
std::vector<Point2D> resample_polyline(const std::vector<Point2D> & points, double spacing);
std::vector<double> cumulative_times(
  const std::vector<Point2D> & points, double max_velocity, double max_acceleration);

// 含转角的梯形加减速时间分配（报告 5.5.3.2 思路一）。
// 等效路程 s = k_line * s1 + k_turn * s2：
//   s1 为相邻两点连线长度（对应直线行驶速度），
//   s2 为以中间点为转角的转角变化量（对应旋转时的速度，k_turn 表征轮到车体中心距离）。
// 然后对等效路程按梯形加减速估计每段时间。折角处因 s2 增大而自动获得更长时间，
// 避免简单按直线距离分配导致折角处时间过紧、后端优化失败。
std::vector<double> cumulative_times_with_turning(
  const std::vector<Point2D> & points, double max_velocity, double max_acceleration,
  double turn_weight);

}  // namespace astra_nav

