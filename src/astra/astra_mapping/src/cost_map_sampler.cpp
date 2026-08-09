#include "astra_mapping/cost_map_sampler.hpp"

#include <algorithm>
#include <cmath>

namespace astra_nav
{

namespace
{

struct BilinearStencil
{
  std::array<std::pair<int, int>, 4> cells;
  std::array<double, 4> weights;
  std::array<Point2D, 4> weight_gradients;
};

// 越界格代价视为 255，与 HWSentryNav26 CostMap::raw_cost_at_cell 一致。
double raw_cost_at_cell(const Grid2D & cost_map, const std::pair<int, int> & cell)
{
  if (cell.first >= 0 && cell.first < cost_map.width &&
    cell.second >= 0 && cell.second < cost_map.height)
  {
    return static_cast<double>(cost_map.at(cell.first, cell.second));
  }
  return 255.0;
}

bool contains_map_point(const Grid2D & cost_map, const Point2D & position)
{
  const double max_x = cost_map.origin.x + cost_map.resolution * cost_map.width;
  const double max_y = cost_map.origin.y + cost_map.resolution * cost_map.height;
  return std::isfinite(position.x) && std::isfinite(position.y) &&
         position.x >= cost_map.origin.x && position.y >= cost_map.origin.y &&
         position.x < max_x && position.y < max_y;
}

BilinearStencil centered_bilinear_stencil(const Grid2D & cost_map, const Point2D & position)
{
  double qx = (position.x - cost_map.origin.x) / cost_map.resolution - 0.5;
  double qy = (position.y - cost_map.origin.y) / cost_map.resolution - 0.5;
  for (double * q : {&qx, &qy}) {
    const double nearest_integer = std::round(*q);
    if (std::abs(*q - nearest_integer) <= 1e-10) {*q = nearest_integer;}
  }
  const double unclamped_x = qx;
  const double unclamped_y = qy;
  qx = std::clamp(qx, 0.0, static_cast<double>(cost_map.width - 1));
  qy = std::clamp(qy, 0.0, static_cast<double>(cost_map.height - 1));
  const int x0 = static_cast<int>(std::floor(qx));
  const int y0 = static_cast<int>(std::floor(qy));
  const int x1 = std::min(x0 + 1, cost_map.width - 1);
  const int y1 = std::min(y0 + 1, cost_map.height - 1);
  const double tx = qx - static_cast<double>(x0);
  const double ty = qy - static_cast<double>(y0);
  const double inv_resolution = 1.0 / cost_map.resolution;
  BilinearStencil stencil{
    {{{x0, y0}, {x1, y0}, {x0, y1}, {x1, y1}}},
    {{
      (1.0 - tx) * (1.0 - ty), tx * (1.0 - ty),
      (1.0 - tx) * ty, tx * ty,
    }},
    {{
      {-(1.0 - ty) * inv_resolution, -(1.0 - tx) * inv_resolution},
      {(1.0 - ty) * inv_resolution, -tx * inv_resolution},
      {-ty * inv_resolution, (1.0 - tx) * inv_resolution},
      {ty * inv_resolution, tx * inv_resolution},
    }},
  };
  if (unclamped_x < 0.0 || unclamped_x > static_cast<double>(cost_map.width - 1)) {
    for (Point2D & gradient : stencil.weight_gradients) {gradient.x = 0.0;}
  }
  if (unclamped_y < 0.0 || unclamped_y > static_cast<double>(cost_map.height - 1)) {
    for (Point2D & gradient : stencil.weight_gradients) {gradient.y = 0.0;}
  }
  return stencil;
}

}  // namespace

std::optional<CostSample> sample_cost_map(const Grid2D & cost_map, const Point2D & position)
{
  if (cost_map.width <= 0 || cost_map.height <= 0 || !contains_map_point(cost_map, position)) {
    return std::nullopt;
  }
  return sample_cost_map_clamped(cost_map, position);
}

CostSample sample_cost_map_clamped(const Grid2D & cost_map, const Point2D & position)
{
  const BilinearStencil stencil = centered_bilinear_stencil(cost_map, position);
  CostSample sample{0.0, {0.0, 0.0}};
  for (std::size_t i = 0; i < stencil.cells.size(); ++i) {
    const double raw = raw_cost_at_cell(cost_map, stencil.cells[i]);
    sample.value += stencil.weights[i] * raw;
    sample.gradient.x += stencil.weight_gradients[i].x * raw;
    sample.gradient.y += stencil.weight_gradients[i].y * raw;
  }
  return sample;
}

}  // namespace astra_nav
