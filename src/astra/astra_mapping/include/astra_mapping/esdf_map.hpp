#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "astra_common/common.hpp"
#include "astra_mapping/obstacle_extractor.hpp"

namespace astra_nav
{

class EsdfMap
{
public:
  EsdfMap() = default;
  explicit EsdfMap(const Grid2D & obstacles);

  bool valid() const { return width_ > 0 && height_ > 0; }
  // 距离与梯度查询：距离场为有符号欧氏距离（自由区为正、障碍内部为负），
  // 采用二次（抛物线）插值，避免双线性插值在峡谷中线处出现的“梯度无效化”。
  double query_distance(double x, double y) const;
  std::pair<double, Point2D> query_distance_and_gradient(double x, double y) const;
  std::pair<int, int> world_to_grid(double x, double y) const;
  Point2D grid_to_world(int x, int y) const;
  bool in_bounds(int x, int y) const;
  // 判断世界坐标是否落在 ESDF 栅格范围内。越界查询不再返回 -1 魔数，
  // 调用方应先用本函数判断有效性，再决定是否采用查询结果。
  bool in_map(double x, double y) const;

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  Point2D origin() const { return origin_; }

private:
  double distance_at(int x, int y) const;
  // 一维平方距离变换（Felzenszwalb & Huttenlocher）。对长度为 n 的一行/一列
  // 输入平方代价 f，输出每个位置到最近“源”的最小平方距离 d。
  static void distance_transform_1d(
    const std::vector<double> & f, std::vector<double> & d, int n);
  // 对二值占据栅格做一次精确欧氏距离变换（输出单位：米）。
  // occupied=true 表示该格视为“源”，输出每格到最近源的欧氏距离。
  std::vector<float> euclidean_distance_transform(
    const std::vector<std::uint8_t> & occupied) const;

  int width_{0};
  int height_{0};
  double resolution_{0.1};
  Point2D origin_;
  std::vector<float> distance_;
};

}  // namespace astra_nav

