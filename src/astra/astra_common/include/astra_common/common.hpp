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

// 沿【整条路径】做速度² 的前向-后向可达传播，再梯形积分出每个路标点的时间。
// 逐项对应上游 HWSentryNav26 speed_profile_optimizer.cpp 的 reachable_seed() + make_profile()：
//   前向  z[i+1] = min(vmax², z[i] + 2·amax·Δs)      —— 起点速度可达性
//   终点  z[n-1] = 0                                 —— 终点停稳
//   后向  z[i]   = min(z[i],  z[i+1] + 2·amax·Δs)    —— 刹车可达性
//   时间  t += 2Δs / (v[i] + v[i+1])                 —— 匀加速段的精确时长
// 关键点是速度【跨路标点连续】。旧实现给每一段单独安排"从静止加速再减速到静止"的梯形，
// 在 waypoint_spacing=0.35 m 下每段平均速度只有 vmax 的 ~25%，是机器人整体偏慢的根因。
// start_speed 为起点在路径切向上的前向投影速度（重规划继承用），对应上游
// measured_initial_speed_squared；允许暂时超出 vmax 包络，由终点零速反向裁成可达值。
std::vector<double> cumulative_times(
  const std::vector<Point2D> & points, double max_velocity, double max_acceleration,
  double start_speed = 0.0);

// 含转角的时间分配（报告 5.5.3.2 思路一）。
// 等效路程 s = k_line * s1 + k_turn * s2：
//   s1 为相邻两点连线长度（对应直线行驶速度），
//   s2 为以中间点为转角的转角变化量（对应旋转时的速度，k_turn 表征轮到车体中心距离）。
// 折角处因 s2 增大而自动获得更长时间，避免简单按直线距离分配导致折角处时间过紧、
// 后端优化失败。时间分配本身与 cumulative_times 用同一套前向-后向可达传播，
// 只是把"弧长"换成上述等效路程。
std::vector<double> cumulative_times_with_turning(
  const std::vector<Point2D> & points, double max_velocity, double max_acceleration,
  double turn_weight, double start_speed = 0.0);

}  // namespace astra_nav

