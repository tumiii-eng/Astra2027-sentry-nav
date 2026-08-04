#include "astra_planning/waypoint_lbfgs_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Eigen>

#include "astra_third_party/gcopter/lbfgs.hpp"

namespace astra_nav
{

namespace
{

struct ObjectiveContext
{
  const WaypointLbfgsConfig * config{nullptr};
  const EsdfMap * esdf{nullptr};
  const std::vector<Point2D> * reference{nullptr};
  int iterations{0};
};

std::vector<Point2D> decode_points(
  const Eigen::VectorXd & x, const std::vector<Point2D> & reference)
{
  auto points = reference;
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const int offset = static_cast<int>((i - 1) * 2);
    points[i].x = x(offset);
    points[i].y = x(offset + 1);
  }
  return points;
}

Eigen::VectorXd encode_inner_points(const std::vector<Point2D> & points)
{
  Eigen::VectorXd x(static_cast<int>(std::max<std::size_t>(0, points.size() - 2) * 2));
  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const int offset = static_cast<int>((i - 1) * 2);
    x(offset) = points[i].x;
    x(offset + 1) = points[i].y;
  }
  return x;
}

void add_gradient(
  Eigen::VectorXd & gradient, std::size_t point_index, double gx, double gy,
  std::size_t point_count)
{
  if (point_index == 0 || point_index + 1 >= point_count) {
    return;
  }
  const int offset = static_cast<int>((point_index - 1) * 2);
  gradient(offset) += gx;
  gradient(offset + 1) += gy;
}

Point2D fallback_clearance_gradient(const std::vector<Point2D> & points, std::size_t i)
{
  const Point2D prev = points[i - 1];
  const Point2D next = points[i + 1];
  const double tx = next.x - prev.x;
  const double ty = next.y - prev.y;
  const double norm = std::hypot(tx, ty);
  if (norm < 1e-9) {
    return {0.0, 1.0};
  }

  Point2D normal{-ty / norm, tx / norm};
  if (std::abs(normal.x) + std::abs(normal.y) < 1e-9) {
    return {0.0, 1.0};
  }

  if (normal.y < 0.0 || (std::abs(normal.y) < 1e-9 && normal.x < 0.0)) {
    normal.x = -normal.x;
    normal.y = -normal.y;
  }
  return normal;
}

double evaluate_waypoint_objective(
  const WaypointLbfgsConfig & config, const EsdfMap & esdf,
  const std::vector<Point2D> & reference, const Eigen::VectorXd & x,
  Eigen::VectorXd & gradient)
{
  const auto points = decode_points(x, reference);
  gradient.setZero(x.size());
  double cost = 0.0;

  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const double dx = points[i].x - reference[i].x;
    const double dy = points[i].y - reference[i].y;
    cost += 0.5 * config.reference_weight * (dx * dx + dy * dy);
    add_gradient(gradient, i, config.reference_weight * dx, config.reference_weight * dy, points.size());
  }

  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const double rx = points[i - 1].x - 2.0 * points[i].x + points[i + 1].x;
    const double ry = points[i - 1].y - 2.0 * points[i].y + points[i + 1].y;
    cost += 0.5 * config.smooth_weight * (rx * rx + ry * ry);
    add_gradient(gradient, i - 1, config.smooth_weight * rx, config.smooth_weight * ry, points.size());
    add_gradient(
      gradient, i, -2.0 * config.smooth_weight * rx, -2.0 * config.smooth_weight * ry,
      points.size());
    add_gradient(gradient, i + 1, config.smooth_weight * rx, config.smooth_weight * ry, points.size());
  }

  if (!esdf.valid()) {
    return cost;
  }

  for (std::size_t i = 1; i + 1 < points.size(); ++i) {
    const auto query = esdf.query_distance_and_gradient(points[i].x, points[i].y);
    const auto grid_index = esdf.world_to_grid(points[i].x, points[i].y);
    const bool in_map = esdf.in_bounds(grid_index.first, grid_index.second);
    if (!in_map) {
      const double dx = points[i].x - reference[i].x;
      const double dy = points[i].y - reference[i].y;
      const double penalty = config.obstacle_margin - query.first;
      cost += 0.5 * config.invalid_distance_weight * (penalty * penalty + dx * dx + dy * dy);
      add_gradient(
        gradient, i, config.invalid_distance_weight * dx, config.invalid_distance_weight * dy,
        points.size());
      continue;
    }

    const double violation = config.obstacle_margin - query.first;
    if (violation <= 0.0) {
      continue;
    }
    cost += 0.5 * config.obstacle_weight * violation * violation;
    Point2D clearance_gradient = query.second;
    if (std::hypot(clearance_gradient.x, clearance_gradient.y) < 1.0e-6) {
      clearance_gradient = fallback_clearance_gradient(points, i);
    }
    add_gradient(
      gradient, i, -config.obstacle_weight * violation * clearance_gradient.x,
      -config.obstacle_weight * violation * clearance_gradient.y, points.size());
  }

  if (!std::isfinite(cost)) {
    return std::numeric_limits<double>::infinity();
  }
  return cost;
}

double lbfgs_evaluate(void * ptr, const Eigen::VectorXd & x, Eigen::VectorXd & gradient)
{
  auto & context = *static_cast<ObjectiveContext *>(ptr);
  context.iterations += 1;
  return evaluate_waypoint_objective(*context.config, *context.esdf, *context.reference, x, gradient);
}

}  // namespace

WaypointLbfgsOptimizer::WaypointLbfgsOptimizer(const WaypointLbfgsConfig & config)
: config_(config)
{
}

WaypointLbfgsResult WaypointLbfgsOptimizer::optimize(
  const std::vector<Point2D> & input, const EsdfMap & esdf) const
{
  WaypointLbfgsResult result;
  result.points = input;
  if (input.size() <= 2) {
    return result;
  }

  Eigen::VectorXd x = encode_inner_points(input);
  Eigen::VectorXd gradient(x.size());
  result.initial_cost = evaluate_waypoint_objective(config_, esdf, input, x, gradient);

  ObjectiveContext context;
  context.config = &config_;
  context.esdf = &esdf;
  context.reference = &input;

  lbfgs::lbfgs_parameter_t params;
  params.mem_size = std::max(3, config_.memory_size);
  params.past = std::max(0, config_.past);
  params.delta = std::max(0.0, config_.delta);
  params.g_epsilon = std::max(0.0, config_.gradient_epsilon);
  params.max_iterations = std::max(1, config_.max_iterations);
  params.min_step = 1.0e-12;
  params.max_linesearch = 64;

  double final_cost = result.initial_cost;
  result.status = lbfgs::lbfgs_optimize(
    x, final_cost, lbfgs_evaluate, nullptr, nullptr, &context, params);
  result.iterations = context.iterations;
  result.final_cost = final_cost;

  const bool accepted_status =
    result.status == lbfgs::LBFGS_CONVERGENCE || result.status == lbfgs::LBFGS_STOP ||
    result.status == lbfgs::LBFGSERR_MAXIMUMITERATION;
  if (accepted_status && std::isfinite(result.final_cost) && result.final_cost <= result.initial_cost) {
    result.points = decode_points(x, input);
    result.optimized = true;
  }

  return result;
}

}  // namespace astra_nav
