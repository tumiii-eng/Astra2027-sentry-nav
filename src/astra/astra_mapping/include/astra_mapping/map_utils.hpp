#pragma once

// 本文件的膨胀实现直接复制自 HWSentryNav26 (map_server/include/map_server/utils.hpp,
// map_server/src/utils.cpp)，计算方法逐字一致，不做任何改写，只裁剪掉 Astra 不需要
// 的地形方向场部分。上游是已验证可靠的方案，故保持分毫不差的复用。

#include <cstddef>
#include <cstdint>

#include <opencv2/opencv.hpp>

namespace astra_nav::map_utils
{

struct MapInflationParams
{
  double full_cost_radius_m; // 满代价半径：该半径内障碍代价保持255（膨胀平台）
  double cutoff_radius_m;
  double decay_rate_per_m;
  double resolution = 0.0; // map resolution (m/px), must be set before calling inflation functions
};

int enclosing_radius_cells(double radius_m, double resolution);

cv::Mat inflate_cost_map(
  const cv::Mat & source,
  const MapInflationParams & params);

/// Inflate only connected-component neighborhoods whose support can reach a
/// non-zero output. Falls back to the full-map transform when the aggregate
/// neighborhood area is not smaller than the source map.
cv::Mat inflate_cost_map_bounded(
  const cv::Mat & source,
  const MapInflationParams & params);

}  // namespace astra_nav::map_utils
