#pragma once

#include <cstdint>
#include <vector>

#include "astra_perception/rolling_occupancy_grid.hpp"

namespace astra_nav
{

struct ObstacleExtractionConfig
{
  double height_obstacle_min{0.18};
  double column_occupancy_min{0.18};
};

struct Grid2D
{
  int width{0};
  int height{0};
  double resolution{0.1};
  Point2D origin;
  std::vector<std::uint8_t> data;

  std::uint8_t & at(int x, int y) { return data[static_cast<std::size_t>(y * width + x)]; }
  std::uint8_t at(int x, int y) const { return data[static_cast<std::size_t>(y * width + x)]; }
};

class ObstacleExtractor
{
public:
  explicit ObstacleExtractor(const ObstacleExtractionConfig & config);
  Grid2D extract(const RollingOccupancyGrid3D & grid) const;
  static Grid2D inflate(const Grid2D & input, int radius_cells);

private:
  ObstacleExtractionConfig config_;
};

}  // namespace astra_nav

