#pragma once

#include <vector>

#include "astra_mapping/esdf_map.hpp"

namespace astra_nav
{

struct WaypointLbfgsConfig
{
  double reference_weight{0.08};
  double smooth_weight{0.45};
  double obstacle_weight{3.0};
  double obstacle_margin{0.55};
  double invalid_distance_weight{1.0};
  int max_iterations{80};
  int memory_size{8};
  int past{3};
  double delta{1.0e-5};
  double gradient_epsilon{1.0e-5};
};

struct WaypointLbfgsResult
{
  std::vector<Point2D> points;
  double initial_cost{0.0};
  double final_cost{0.0};
  int status{0};
  int iterations{0};
  bool optimized{false};
};

class WaypointLbfgsOptimizer
{
public:
  explicit WaypointLbfgsOptimizer(const WaypointLbfgsConfig & config);

  WaypointLbfgsResult optimize(const std::vector<Point2D> & input, const EsdfMap & esdf) const;

private:
  WaypointLbfgsConfig config_;
};

}  // namespace astra_nav
