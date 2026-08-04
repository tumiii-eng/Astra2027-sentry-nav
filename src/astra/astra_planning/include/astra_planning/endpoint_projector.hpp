#pragma once

#include "astra_mapping/esdf_map.hpp"

namespace astra_nav
{

struct EndpointProjectorConfig
{
  double required_clearance{0.55};
  double clearance_hysteresis{0.0};
  double max_search_radius{1.2};
  int angle_samples{72};
  bool require_grid_free{true};
  bool allow_hysteresis_without_grid_free{false};
};

struct EndpointProjectionResult
{
  Point2D original;
  Point2D point;
  bool success{false};
  bool moved{false};
  double original_distance{0.0};
  double projected_distance{0.0};
  double displacement{0.0};
};

class EndpointProjector
{
public:
  explicit EndpointProjector(const EndpointProjectorConfig & config);

  EndpointProjectionResult project(
    const Point2D & point, const Grid2D & traversable_grid, const EsdfMap & esdf) const;

private:
  bool is_candidate_free(const Point2D & point, const Grid2D & traversable_grid, const EsdfMap & esdf)
    const;
  double candidate_score(const Point2D & point, const Point2D & original, double distance) const;

  EndpointProjectorConfig config_;
};

}  // namespace astra_nav
