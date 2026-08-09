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

// 统一硬占用阈值：格子代价 >= 该值视为不可通行；低于该值为“软代价”，
// 只加权边代价而不阻断搜索。与 HWSentryNav26 path_planner 的
// endpoint_handling.occupied_cost_threshold 保持一致。
constexpr std::uint8_t kOccupiedCostThreshold = 200;

// 连续代价场膨胀参数（物理距离，单位米）。障碍源周围 full_cost_radius_m 内
// 保持满代价 255，之后按 exp(-decay_rate_per_m * d) 衰减到 cutoff_radius_m 截断。
// 默认值取自 HWSentryNav26 map_server/config/params.yaml 的静态障碍膨胀参数
// (global_map.inflation)；resolution 由 Grid2D 提供，故此处不含该字段。
struct InflationParams
{
  double full_cost_radius_m{0.1};
  double cutoff_radius_m{0.3};
  double decay_rate_per_m{24.0};
};

// 代价栅格，取值 0-255 的连续代价场（不再是 0/1 二值）：
// 255 = 障碍本体或满代价平台，(0, 255) = 膨胀衰减区的软代价，0 = 自由。
struct Grid2D
{
  int width{0};
  int height{0};
  double resolution{0.1};
  Point2D origin;
  std::vector<std::uint8_t> data;

  std::uint8_t & at(int x, int y) { return data[static_cast<std::size_t>(y * width + x)]; }
  std::uint8_t at(int x, int y) const { return data[static_cast<std::size_t>(y * width + x)]; }

  // 硬占用判定：越界视为占用，与 HWSentryNav26 CostMap::raw_cost_at_cell 越界返回 255 等价。
  bool occupied(int x, int y) const
  {
    if (x < 0 || y < 0 || x >= width || y >= height) {
      return true;
    }
    return at(x, y) >= kOccupiedCostThreshold;
  }
};

class ObstacleExtractor
{
public:
  explicit ObstacleExtractor(const ObstacleExtractionConfig & config);
  Grid2D extract(const RollingOccupancyGrid3D & grid) const;
  // 连续代价场膨胀：以“代价非零”的格子为源做精确欧氏距离变换，
  // 再按满代价平台 + 指数衰减生成 0-255 连续代价场。源格代价保持原值。
  // 相比二值硬膨胀，连续场保证下游基于梯度的优化器（MINCO/L-BFGS）在障碍
  // 边界附近代价连续可导，且远处梯度非零，不会“看不见”约束。
  static Grid2D inflate(const Grid2D & input, const InflationParams & params);

private:
  ObstacleExtractionConfig config_;
};

}  // namespace astra_nav

