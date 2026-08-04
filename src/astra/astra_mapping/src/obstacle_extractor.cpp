#include "astra_mapping/obstacle_extractor.hpp"

#include <algorithm>
#include <cmath>

namespace astra_nav
{

ObstacleExtractor::ObstacleExtractor(const ObstacleExtractionConfig & config)
: config_(config)
{
}

Grid2D ObstacleExtractor::extract(const RollingOccupancyGrid3D & grid) const
{
  Grid2D output;
  output.width = grid.nx();
  output.height = grid.ny();
  output.resolution = grid.resolution();
  output.origin = {grid.origin().x, grid.origin().y};
  output.data.assign(static_cast<std::size_t>(output.width * output.height), 0);
  const auto mask = grid.occupied_mask();

  for (int ix = 0; ix < grid.nx(); ++ix) {
    for (int iy = 0; iy < grid.ny(); ++iy) {
      int first = -1;
      int last = -1;
      int count = 0;
      for (int iz = 0; iz < grid.nz(); ++iz) {
        const std::size_t idx = static_cast<std::size_t>((ix * grid.ny() + iy) * grid.nz() + iz);
        if (mask[idx]) {
          if (first < 0) {
            first = iz;
          }
          last = iz;
          ++count;
        }
      }
      if (count == 0) {
        continue;
      }
      const double height = (last - first + 1) * grid.resolution();
      const double occupancy_ratio = (count * grid.resolution()) / std::max(height, grid.resolution());
      if (height >= config_.height_obstacle_min && occupancy_ratio >= config_.column_occupancy_min) {
        output.at(ix, iy) = 1;
      }
    }
  }
  return output;
}

Grid2D ObstacleExtractor::inflate(const Grid2D & input, int radius_cells)
{
  if (radius_cells <= 0) {
    return input;
  }
  Grid2D output = input;
  std::fill(output.data.begin(), output.data.end(), 0);
  for (int x = 0; x < input.width; ++x) {
    for (int y = 0; y < input.height; ++y) {
      if (input.at(x, y) == 0) {
        continue;
      }
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
          if (dx * dx + dy * dy > radius_cells * radius_cells) {
            continue;
          }
          const int nx = x + dx;
          const int ny = y + dy;
          if (nx >= 0 && ny >= 0 && nx < input.width && ny < input.height) {
            output.at(nx, ny) = 1;
          }
        }
      }
    }
  }
  return output;
}

}  // namespace astra_nav

