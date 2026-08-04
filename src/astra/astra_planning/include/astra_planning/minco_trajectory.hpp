#pragma once

#include <vector>

#include "astra_common/common.hpp"
#include "astra_planning/minco_adapter.hpp"

namespace astra_nav
{

class MincoTrajectory
{
public:
  static MincoTrajectory fit(
    const std::vector<Point2D> & waypoints, const std::vector<double> & times);
  static MincoTrajectory fit(
    const std::vector<Point2D> & waypoints, const std::vector<double> & times,
    const MincoBoundaryState & head, const MincoBoundaryState & tail);

  TrajectoryPoint evaluate(double t) const;
  std::vector<TrajectoryPoint> sample(double dt) const;
  // 把世界坐标点投影到轨迹上，返回最近点对应的时间参数 t（报告 5.5.4.4 重规划
  // “获取当前位置在轨迹上的映射位置”）。采用粗采样定位 + 局部黄金分割细化，
  // 在 O(duration/dt) 内得到稳定的最近点参数。空轨迹返回 0。
  double project(const Point2D & point, double dt = 0.05) const;
  bool empty() const { return adapter_.empty(); }
  double duration() const { return duration_; }

private:
  MincoAdapter adapter_;
  double duration_{0.0};
};

}  // namespace astra_nav
