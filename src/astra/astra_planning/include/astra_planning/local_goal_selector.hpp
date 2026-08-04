#pragma once

#include "astra_mapping/esdf_map.hpp"

namespace astra_nav
{

struct LocalGoalSelectorConfig
{
  double required_clearance{0.55};
  double backoff_distance{0.35};
  double sample_step{0.1};
  double min_progress_distance{0.35};
  bool require_grid_free{true};
};

struct LocalGoalSelectionResult
{
  Point2D global_goal;
  Point2D local_goal;
  bool success{false};
  bool clipped{false};
  double progress_ratio{0.0};
  double distance_to_global_goal{0.0};
};

class LocalGoalSelector
{
public:
  explicit LocalGoalSelector(const LocalGoalSelectorConfig & config);

  LocalGoalSelectionResult select(
    const Point2D & start, const Point2D & global_goal, const Grid2D & traversable_grid,
    const EsdfMap & esdf) const;

private:
  bool is_valid(
    const Point2D & point, const Grid2D & traversable_grid, const EsdfMap & esdf) const;
  LocalGoalSelectionResult select_lateral_goal(
    const Point2D & start, const Point2D & global_goal, const Grid2D & traversable_grid,
    const EsdfMap & esdf, double total_distance) const;

  LocalGoalSelectorConfig config_;
};

}  // namespace astra_nav
