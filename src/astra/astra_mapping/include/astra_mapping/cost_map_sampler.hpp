#pragma once

// 双线性代价采样与解析梯度直接复制自 HWSentryNav26
// (nav_executor/src/common/environment/nav_map.cpp 的 centered_bilinear_stencil /
//  CostMap::sample_map / CostMap::sample_map_clamped)，公式逐字一致，
// 只把 Eigen 类型换成 Astra 的 Point2D、CostMap 换成 Grid2D。

#include <array>
#include <optional>

#include "astra_mapping/obstacle_extractor.hpp"

namespace astra_nav
{

// 代价采样结果：value 为插值代价（0-255 量纲），gradient 为 cost/m。
struct CostSample
{
  double value{0.0};
  Point2D gradient{0.0, 0.0};
};

// 格心对齐的双线性采样：位于地图足迹内返回采样值，否则返回 nullopt。
std::optional<CostSample> sample_cost_map(const Grid2D & cost_map, const Point2D & position);

// 与 sample_cost_map 同一模板，但处处有定义：足迹外复制边界值，
// 并把越界方向的梯度分量置零（需要跨边界连续惩罚的调用方自行叠加距离斜坡）。
CostSample sample_cost_map_clamped(const Grid2D & cost_map, const Point2D & position);

}  // namespace astra_nav
