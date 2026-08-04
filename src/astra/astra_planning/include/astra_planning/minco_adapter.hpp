#pragma once

#include <vector>

#include "astra_common/common.hpp"
#include "astra_third_party/gcopter/minco.hpp"

namespace astra_nav
{

struct MincoBoundaryState
{
  Point2D position;
  Point2D velocity;
  Point2D acceleration;
};

class MincoAdapter
{
public:
  bool fit(const std::vector<Point2D> & waypoints, const std::vector<double> & times);
  bool fit(
    const std::vector<Point2D> & waypoints, const std::vector<double> & times,
    const MincoBoundaryState & head, const MincoBoundaryState & tail);

  bool empty() const { return trajectory_.getPieceNum() == 0; }
  double duration() const { return duration_; }
  TrajectoryPoint evaluate(double t) const;

private:
  ::Trajectory<5, 2> trajectory_;
  double duration_{0.0};
};

}  // namespace astra_nav
