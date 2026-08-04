#include "astra_planning/endpoint_projector.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace astra_nav
{

namespace
{

constexpr double kTwoPi = 2.0 * M_PI;

double norm2d(const Point2D & point)
{
  return std::hypot(point.x, point.y);
}

Point2D delta(const Point2D & a, const Point2D & b)
{
  return {a.x - b.x, a.y - b.y};
}

}  // namespace

EndpointProjector::EndpointProjector(const EndpointProjectorConfig & config)
: config_(config)
{
  config_.required_clearance = std::max(0.0, config_.required_clearance);
  config_.clearance_hysteresis = std::max(0.0, config_.clearance_hysteresis);
  config_.max_search_radius = std::max(0.0, config_.max_search_radius);
  config_.angle_samples = std::max(8, config_.angle_samples);
}

EndpointProjectionResult EndpointProjector::project(
  const Point2D & point, const Grid2D & traversable_grid, const EsdfMap & esdf) const
{
  EndpointProjectionResult result;
  result.original = point;
  result.point = point;
  result.original_distance = esdf.query_distance(point.x, point.y);
  result.projected_distance = result.original_distance;

  if (!esdf.valid() || traversable_grid.width <= 0 || traversable_grid.height <= 0) {
    return result;
  }

  const double accepted_clearance =
    std::max(0.0, config_.required_clearance - config_.clearance_hysteresis);
  const auto esdf_index = esdf.world_to_grid(point.x, point.y);
  const bool in_esdf_bounds = esdf.in_bounds(esdf_index.first, esdf_index.second);
  const bool candidate_free = is_candidate_free(point, traversable_grid, esdf);
  const bool esdf_hysteresis_accepted =
    config_.allow_hysteresis_without_grid_free && config_.clearance_hysteresis > 0.0 &&
    in_esdf_bounds && result.original_distance >= accepted_clearance;
  if ((candidate_free && result.original_distance >= accepted_clearance) ||
    esdf_hysteresis_accepted)
  {
    result.success = true;
    return result;
  }

  const double radial_step = std::max(0.5 * esdf.resolution(), 0.04);
  const int rings = std::max(1, static_cast<int>(std::ceil(config_.max_search_radius / radial_step)));
  // 安全距离比较容差：ESDF 升级为更精确的二次插值后，候选点距离可能恰好等于
  // required_clearance，浮点直接比较会误判为不达标。引入小容差保证边界稳定。
  constexpr double kClearanceEps = 1.0e-6;
  double best_score = std::numeric_limits<double>::infinity();
  double best_distance = -std::numeric_limits<double>::infinity();
  Point2D best_point = point;

  for (int ring = 1; ring <= rings; ++ring) {
    const double radius = std::min(config_.max_search_radius, ring * radial_step);
    const int samples = std::max(config_.angle_samples, static_cast<int>(std::ceil(kTwoPi * radius / radial_step)));
    for (int i = 0; i < samples; ++i) {
      const double angle = kTwoPi * static_cast<double>(i) / static_cast<double>(samples);
      const Point2D candidate{point.x + radius * std::cos(angle), point.y + radius * std::sin(angle)};
      if (!is_candidate_free(candidate, traversable_grid, esdf)) {
        continue;
      }

      const double distance = esdf.query_distance(candidate.x, candidate.y);
      if (distance < config_.required_clearance - kClearanceEps) {
        if (distance > best_distance) {
          best_distance = distance;
          best_point = candidate;
          best_score = candidate_score(candidate, point, distance);
        }
        continue;
      }

      const double score = candidate_score(candidate, point, distance);
      if (score < best_score) {
        best_score = score;
        best_distance = distance;
        best_point = candidate;
      }
    }
  }

  result.point = best_point;
  result.projected_distance = best_distance;
  result.displacement = norm2d(delta(result.point, result.original));
  result.moved = result.displacement > 1.0e-6;
  result.success =
    std::isfinite(best_score) && best_distance >= config_.required_clearance - kClearanceEps;
  return result;
}

bool EndpointProjector::is_candidate_free(
  const Point2D & point, const Grid2D & traversable_grid, const EsdfMap & esdf) const
{
  const auto esdf_index = esdf.world_to_grid(point.x, point.y);
  if (!esdf.in_bounds(esdf_index.first, esdf_index.second)) {
    return false;
  }
  if (!config_.require_grid_free) {
    return true;
  }

  const int gx = static_cast<int>(std::floor((point.x - traversable_grid.origin.x) / traversable_grid.resolution));
  const int gy = static_cast<int>(std::floor((point.y - traversable_grid.origin.y) / traversable_grid.resolution));
  if (gx < 0 || gy < 0 || gx >= traversable_grid.width || gy >= traversable_grid.height) {
    return false;
  }
  return traversable_grid.at(gx, gy) == 0;
}

double EndpointProjector::candidate_score(
  const Point2D & point, const Point2D & original, double distance) const
{
  const double displacement = norm2d(delta(point, original));
  const double clearance_surplus = std::max(0.0, distance - config_.required_clearance);
  return displacement - 0.05 * clearance_surplus;
}

}  // namespace astra_nav
