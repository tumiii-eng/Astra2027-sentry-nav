#include "astra_mapping/obstacle_extractor.hpp"

#include <algorithm>
#include <cmath>

#include "astra_mapping/map_utils.hpp"

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
        output.at(ix, iy) = 255;
      }
    }
  }
  return output;
}

Grid2D ObstacleExtractor::inflate(const Grid2D & input, const InflationParams & params)
{
  if (input.width <= 0 || input.height <= 0) {
    return input;
  }

  // 直接复用 HWSentryNav26 的膨胀实现（map_utils），本函数只做 Grid2D <-> cv::Mat 转接。
  map_utils::MapInflationParams inflation_params;
  inflation_params.full_cost_radius_m = params.full_cost_radius_m;
  inflation_params.cutoff_radius_m = params.cutoff_radius_m;
  inflation_params.decay_rate_per_m = params.decay_rate_per_m;
  inflation_params.resolution = input.resolution;

  Grid2D output = input;
  // Grid2D 行主序 (y * width + x) 与 cv::Mat 行主序一致，可零拷贝包装。
  const cv::Mat source(
    input.height, input.width, CV_8UC1, const_cast<std::uint8_t *>(input.data.data()));
  const cv::Mat inflated = map_utils::inflate_cost_map_bounded(source, inflation_params);
  std::copy(
    inflated.begin<std::uint8_t>(), inflated.end<std::uint8_t>(), output.data.begin());
  return output;
}

}  // namespace astra_nav

