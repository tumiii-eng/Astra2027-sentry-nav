#include "astra_planning/minco_adapter.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/Eigen>

namespace astra_nav
{

namespace
{

Point2D velocity_between(const Point2D & a, const Point2D & b, double dt)
{
  dt = std::max(dt, 0.05);
  return {(b.x - a.x) / dt, (b.y - a.y) / dt};
}

Eigen::Matrix<double, 2, 3> to_minco_state(const MincoBoundaryState & state)
{
  Eigen::Matrix<double, 2, 3> result;
  result << state.position.x, state.velocity.x, state.acceleration.x,
    state.position.y, state.velocity.y, state.acceleration.y;
  return result;
}

bool validate_times(const std::vector<double> & times)
{
  if (times.size() < 2 || std::abs(times.front()) > 1e-6) {
    return false;
  }
  for (std::size_t i = 1; i < times.size(); ++i) {
    if (!(times[i] > times[i - 1])) {
      return false;
    }
  }
  return true;
}

}  // namespace

bool MincoAdapter::fit(const std::vector<Point2D> & waypoints, const std::vector<double> & times)
{
  if (waypoints.size() < 2 || waypoints.size() != times.size() || !validate_times(times)) {
    trajectory_.clear();
    duration_ = 0.0;
    return false;
  }

  MincoBoundaryState head;
  head.position = waypoints.front();
  head.velocity = velocity_between(waypoints[0], waypoints[1], times[1] - times[0]);
  head.acceleration = {};

  MincoBoundaryState tail;
  tail.position = waypoints.back();
  tail.velocity = velocity_between(
    waypoints[waypoints.size() - 2], waypoints.back(), times.back() - times[times.size() - 2]);
  tail.acceleration = {};

  return fit(waypoints, times, head, tail);
}

bool MincoAdapter::fit(
  const std::vector<Point2D> & waypoints, const std::vector<double> & times,
  const MincoBoundaryState & head, const MincoBoundaryState & tail)
{
  if (waypoints.size() < 2 || waypoints.size() != times.size() || !validate_times(times)) {
    trajectory_.clear();
    duration_ = 0.0;
    return false;
  }

  const int piece_count = static_cast<int>(waypoints.size()) - 1;
  Eigen::MatrixXd inner_points(2, std::max(0, piece_count - 1));
  for (int i = 0; i < piece_count - 1; ++i) {
    inner_points(0, i) = waypoints[static_cast<std::size_t>(i + 1)].x;
    inner_points(1, i) = waypoints[static_cast<std::size_t>(i + 1)].y;
  }

  Eigen::VectorXd durations(piece_count);
  for (int i = 0; i < piece_count; ++i) {
    durations(i) = times[static_cast<std::size_t>(i + 1)] - times[static_cast<std::size_t>(i)];
  }

  minco::MINCO_S3NU minco;
  minco.setConditions(to_minco_state(head), to_minco_state(tail), piece_count);
  minco.setParameters(inner_points, durations);
  minco.getTrajectory(trajectory_);
  duration_ = times.back();
  return !empty();
}

TrajectoryPoint MincoAdapter::evaluate(double t) const
{
  TrajectoryPoint point;
  if (empty()) {
    return point;
  }

  point.t = std::clamp(t, 0.0, duration_);
  const auto position = trajectory_.getPos(point.t);
  const auto velocity = trajectory_.getVel(point.t);
  const auto acceleration = trajectory_.getAcc(point.t);

  point.x = position.x();
  point.y = position.y();
  point.vx = velocity.x();
  point.vy = velocity.y();
  point.ax = acceleration.x();
  point.ay = acceleration.y();
  point.yaw = velocity.norm() > 1e-6 ? std::atan2(point.vy, point.vx) : 0.0;
  return point;
}

}  // namespace astra_nav
