#include "astra_mapping/map_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace astra_nav::map_utils
{

namespace
{

// 每个膨胀区域的固定调用开销，折算为等价的整图格数。经实测标定：在 560×300 的
// 地图上使该回退阈值与分区/整图路径的实际耗时交叉点（约 800 个连通域）吻合。
constexpr size_t PER_REGION_INFLATION_OVERHEAD_CELLS = 48;

void validate_resolution(const double resolution)
{
  if (!std::isfinite(resolution) || resolution <= 0.0) {
    throw std::invalid_argument("map resolution must be finite and positive");
  }
}

int checked_integer(const double value, const char * quantity)
{
  if (!std::isfinite(value) ||
    value < static_cast<double>(std::numeric_limits<int>::min()) ||
    value > static_cast<double>(std::numeric_limits<int>::max()))
  {
    throw std::overflow_error(std::string(quantity) + " exceeds the supported cell range");
  }
  return static_cast<int>(value);
}

int ceil_nonnegative(const double value, const char * quantity)
{
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string(quantity) + " must be finite and non-negative");
  }
  // Avoid promoting values that are only infinitesimally above an integer due to FP division.
  const double tolerance = 1e-9 * std::max(1.0, value);
  return checked_integer(std::ceil(value - tolerance), quantity);
}

void validate_common_inflation_params(const MapInflationParams & params)
{
  if (!std::isfinite(params.resolution) || params.resolution <= 0.0) {
    throw std::invalid_argument("map inflation resolution must be finite and positive");
  }
  if (!std::isfinite(params.full_cost_radius_m) || params.full_cost_radius_m < 0.0 ||
    !std::isfinite(params.cutoff_radius_m) ||
    params.cutoff_radius_m < params.full_cost_radius_m)
  {
    throw std::invalid_argument(
      "map inflation radii require 0 <= full_cost_radius_m <= cutoff_radius_m");
  }
  if (!std::isfinite(params.decay_rate_per_m) || params.decay_rate_per_m < 0.0) {
    throw std::invalid_argument(
      "map inflation decay_rate_per_m must be finite and non-negative");
  }
}

}  // namespace

int enclosing_radius_cells(const double radius_m, const double resolution)
{
  validate_resolution(resolution);
  return ceil_nonnegative(radius_m / resolution, "metric radius");
}

cv::Mat inflate_cost_map(
  const cv::Mat & source,
  const MapInflationParams & params)
{
  CV_Assert(source.type() == CV_8UC1);
  validate_common_inflation_params(params);
  const int h = source.rows;
  const int w = source.cols;

  // 二值化: 非零值视为障碍物源
  cv::Mat bin_mask;
  cv::threshold(source, bin_mask, 0, 1, cv::THRESH_BINARY);

  // 距离变换（像素单位）
  cv::Mat dist_px;
  cv::distanceTransform(1 - bin_mask, dist_px, cv::DIST_L2, cv::DIST_MASK_PRECISE);

  cv::Mat out = source.clone();
  for (int y = 0; y < h; y++) {
    const float * dist_row = dist_px.ptr<float>(y);
    uint8_t * out_row = out.ptr<uint8_t>(y);
    for (int x = 0; x < w; x++) {
      if (bin_mask.at<uint8_t>(y, x)) {continue;}

      const double distance_m = static_cast<double>(dist_row[x]) * params.resolution;
      if (distance_m <= params.full_cost_radius_m) {
        out_row[x] = 255;
      } else if (distance_m <= params.cutoff_radius_m) {
        const float v = 255.0f * static_cast<float>(std::exp(
              -params.decay_rate_per_m * (distance_m - params.full_cost_radius_m)));
        out_row[x] = static_cast<uint8_t>(std::clamp(v, 0.0f, 255.0f));
      }
    }
  }
  return out;
}

cv::Mat inflate_cost_map_bounded(
  const cv::Mat & source,
  const MapInflationParams & params)
{
  CV_Assert(source.type() == CV_8UC1);
  validate_common_inflation_params(params);

  const int cutoff_radius_px = std::min(
    enclosing_radius_cells(params.cutoff_radius_m, params.resolution),
    std::max(source.rows, source.cols));
  cv::Mat labels;
  cv::Mat stats;
  cv::Mat centroids;
  const int label_count = cv::connectedComponentsWithStats(
    source, labels, stats, centroids, 8, CV_32S);

  std::vector<cv::Rect> inflation_regions;
  inflation_regions.reserve(static_cast<size_t>(std::max(0, label_count - 1)));
  size_t estimated_cell_cost = 0;
  const size_t map_cell_count = static_cast<size_t>(source.rows) *
    static_cast<size_t>(source.cols);

  for (int label = 1; label < label_count; ++label) {
    const cv::Rect component_bounds(
      stats.at<int>(label, cv::CC_STAT_LEFT),
      stats.at<int>(label, cv::CC_STAT_TOP),
      stats.at<int>(label, cv::CC_STAT_WIDTH),
      stats.at<int>(label, cv::CC_STAT_HEIGHT));
    const int left = std::max(0, component_bounds.x - cutoff_radius_px);
    const int top = std::max(0, component_bounds.y - cutoff_radius_px);
    const int right = static_cast<int>(std::min<int64_t>(
        source.cols,
        static_cast<int64_t>(component_bounds.x) + component_bounds.width +
        cutoff_radius_px));
    const int bottom = static_cast<int>(std::min<int64_t>(
        source.rows,
        static_cast<int64_t>(component_bounds.y) + component_bounds.height +
        cutoff_radius_px));
    const cv::Rect clipped_region(left, top, right - left, bottom - top);
    if (clipped_region.empty()) {continue;}

    // 面积之和会重复计入重叠区域，因此本身已是保守上界；再加上每个区域固有的
    // 调用开销（阈值化 / 距离变换的启动成本），否则连通域很多时估计会偏低。
    estimated_cell_cost += static_cast<size_t>(clipped_region.area()) +
      PER_REGION_INFLATION_OVERHEAD_CELLS;
    if (estimated_cell_cost >= map_cell_count) {
      return inflate_cost_map(source, params);
    }
    inflation_regions.push_back(clipped_region);
  }

  cv::Mat result = source.clone();
  for (const cv::Rect & region : inflation_regions) {
    const cv::Mat inflated_region = inflate_cost_map(source(region), params);
    cv::Mat result_region = result(region);
    cv::max(result_region, inflated_region, result_region);
  }
  return result;
}

}  // namespace astra_nav::map_utils
