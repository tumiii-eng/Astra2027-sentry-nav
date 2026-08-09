#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "astra_common/common.hpp"
#include "astra_mapping/esdf_map.hpp"
#include "astra_mapping/obstacle_extractor.hpp"
#include "astra_perception/rolling_occupancy_grid.hpp"

namespace
{

// 验证障碍提取：竖直障碍柱应在二维障碍图中被标记。
void test_obstacle_extraction()
{
  astra_nav::OccupancyConfig config;
  config.resolution = 0.2;
  config.size_x = 4.0;
  config.size_y = 4.0;
  config.size_z = 1.0;
  config.fade_ticks = 100;
  astra_nav::RollingOccupancyGrid3D grid(config);

  std::vector<astra_nav::Point3D> points;
  for (double z = 0.0; z < 0.8; z += 0.2) {
    points.push_back({1.0, 0.0, z});
  }
  grid.update(points, {0.0, 0.0, 0.2}, {0.0, 0.0, 0.0});

  astra_nav::ObstacleExtractor extractor({0.2, 0.1});
  const auto obstacles = extractor.extract(grid);
  bool has_obstacle = false;
  for (const auto value : obstacles.data) {
    has_obstacle = has_obstacle || value != 0;
  }
  assert(has_obstacle);
}

// 验证报告 5.5.2 的核心诉求：
// 1) 距离变换精度（对照暴力最近障碍距离）；
// 2) 障碍内部为真实负距离；
// 3) 二次插值在“两侧等距峡谷中线”处仍给出非零、连续的垂直梯度（消除梯度无效化）。
void test_esdf_quadratic_interpolation()
{
  astra_nav::Grid2D grid;
  grid.width = 80;
  grid.height = 60;
  grid.resolution = 0.1;
  grid.origin = {-4.0, -3.0};
  grid.data.assign(static_cast<std::size_t>(grid.width * grid.height), 0);
  // 构造上下两壁、中间留 3 格宽通道的水平峡谷：墙在 y=27..28 和 y=32..33，
  // 自由通道为第 29、30、31 行，几何对称中线落在第 30 行中心，
  // 世界坐标 y = origin.y + (30+0.5)*res = -3.0 + 3.05 = 0.05。
  for (int x = 0; x < grid.width; ++x) {
    for (int y = 27; y <= 28; ++y) {
      grid.at(x, y) = 255;
    }
    for (int y = 32; y <= 33; ++y) {
      grid.at(x, y) = 255;
    }
  }
  astra_nav::EsdfMap esdf(grid);

  // 精度验证：在峡谷自由区采样若干点，对照暴力最近障碍格中心距离。
  auto brute_force_distance = [&](double wx, double wy) {
    double best = 1.0e9;
    for (int x = 0; x < grid.width; ++x) {
      for (int y = 0; y < grid.height; ++y) {
        // EsdfMap 以 kOccupiedCostThreshold 作为硬占用判据，暴力对照必须用同一判据。
        if (grid.at(x, y) < astra_nav::kOccupiedCostThreshold) {
          continue;
        }
        const auto w = esdf.grid_to_world(x, y);
        best = std::min(best, std::hypot(wx - w.x, wy - w.y));
      }
    }
    return best;
  };
  for (double wx = -2.0; wx <= 2.0; wx += 1.0) {
    const double wy = 0.05;  // 通道中线附近
    const auto query = esdf.query_distance_and_gradient(wx, wy);
    const double brute = brute_force_distance(wx, wy);
    // 二次插值与栅格暴力解会有半格量级偏差，给 0.6*分辨率 的容差。
    assert(std::abs(query.first - brute) < 0.6 * grid.resolution);
  }

  // 障碍内部应为负距离。
  const auto inside = esdf.query_distance(0.0, grid.origin.y + 27.5 * grid.resolution);
  assert(inside < 0.0);

  // 梯度无效化验证：在通道中线偏离中心一点点的位置，垂直方向(y)梯度应显著非零，
  // 且符号随偏移方向翻转——这是双线性插值在等距中线处无法保证的。
  const double mid_y = grid.origin.y + 30.5 * grid.resolution;  // 通道中线世界 y
  const auto upper = esdf.query_distance_and_gradient(0.0, mid_y + 0.02);
  const auto lower = esdf.query_distance_and_gradient(0.0, mid_y - 0.02);
  assert(std::abs(upper.second.y) > 1.0e-3);
  assert(std::abs(lower.second.y) > 1.0e-3);
  assert(upper.second.y * lower.second.y < 0.0);  // 两侧梯度符号相反

  // in_map 边界判断：地图内为真，远离地图为假。
  assert(esdf.in_map(0.0, 0.0));
  assert(!esdf.in_map(100.0, 100.0));
}

}  // namespace

int main()
{
  test_obstacle_extraction();
  test_esdf_quadratic_interpolation();
  std::cout << "astra_mapping 测试通过。" << std::endl;
  return 0;
}
