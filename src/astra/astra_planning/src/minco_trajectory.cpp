#include "astra_planning/minco_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace astra_nav
{

MincoTrajectory MincoTrajectory::fit(
  const std::vector<Point2D> & waypoints, const std::vector<double> & times)
{
  MincoTrajectory trajectory;
  if (!trajectory.adapter_.fit(waypoints, times)) {
    return {};
  }
  trajectory.duration_ = trajectory.adapter_.duration();
  return trajectory;
}

MincoTrajectory MincoTrajectory::fit(
  const std::vector<Point2D> & waypoints, const std::vector<double> & times,
  const MincoBoundaryState & head, const MincoBoundaryState & tail)
{
  MincoTrajectory trajectory;
  if (!trajectory.adapter_.fit(waypoints, times, head, tail)) {
    return {};
  }
  trajectory.duration_ = trajectory.adapter_.duration();
  return trajectory;
}

TrajectoryPoint MincoTrajectory::evaluate(double t) const
{
  return adapter_.evaluate(t);
}

std::vector<TrajectoryPoint> MincoTrajectory::sample(double dt) const
{
  std::vector<TrajectoryPoint> points;
  if (empty()) {
    return points;
  }
  dt = std::max(dt, 1e-3);
  const int count = std::max(2, static_cast<int>(std::ceil(duration_ / dt)) + 1);
  points.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const double t = std::min(duration_, i * dt);
    points.push_back(evaluate(t));
  }
  return points;
}

double MincoTrajectory::project(const Point2D & point, double dt) const
{
  if (empty()) {
    return 0.0;
  }
  dt = std::max(dt, 1e-3);

  // 第一步：粗采样找到平方距离最小的时间点。
  const int count = std::max(2, static_cast<int>(std::ceil(duration_ / dt)) + 1);
  double best_t = 0.0;
  double best_d2 = std::numeric_limits<double>::infinity();
  for (int i = 0; i < count; ++i) {
    const double t = std::min(duration_, i * dt);
    const auto p = evaluate(t);
    const double d2 = (p.x - point.x) * (p.x - point.x) + (p.y - point.y) * (p.y - point.y);
    if (d2 < best_d2) {
      best_d2 = d2;
      best_t = t;
    }
  }

  // 第二步：在最近粗采样点相邻的一个 dt 区间内做黄金分割细化，得到平滑的最近点参数。
  double lo = std::max(0.0, best_t - dt);
  double hi = std::min(duration_, best_t + dt);
  const double inv_phi = 0.6180339887498949;  // 1/phi
  auto dist2_at = [&](double t) {
    const auto p = evaluate(t);
    return (p.x - point.x) * (p.x - point.x) + (p.y - point.y) * (p.y - point.y);
  };
  double c = hi - (hi - lo) * inv_phi;
  double d = lo + (hi - lo) * inv_phi;
  double fc = dist2_at(c);
  double fd = dist2_at(d);
  for (int iter = 0; iter < 40 && (hi - lo) > 1.0e-4; ++iter) {
    if (fc < fd) {
      hi = d;
      d = c;
      fd = fc;
      c = hi - (hi - lo) * inv_phi;
      fc = dist2_at(c);
    } else {
      lo = c;
      c = d;
      fc = fd;
      d = lo + (hi - lo) * inv_phi;
      fd = dist2_at(d);
    }
  }
  return 0.5 * (lo + hi);
}

}  // namespace astra_nav
