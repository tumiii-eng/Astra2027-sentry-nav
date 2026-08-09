#include "astra_mapping/esdf_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace astra_nav
{

EsdfMap::EsdfMap(const Grid2D & obstacles)
: width_(obstacles.width), height_(obstacles.height), resolution_(obstacles.resolution),
  origin_(obstacles.origin)
{
  if (width_ <= 0 || height_ <= 0) {
    width_ = 0;
    height_ = 0;
    return;
  }

  const std::size_t cell_count = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
  distance_.assign(cell_count, 0.0F);

  // 统计占据格数量，决定是否需要做距离变换。
  std::vector<std::uint8_t> occupied(cell_count, 0);
  std::vector<std::uint8_t> free_mask(cell_count, 0);
  std::size_t occupied_count = 0;
  // 0-255 连续代价场：只有达到硬占用阈值的格才是障碍源，膨胀衰减区仍属自由空间。
  for (std::size_t i = 0; i < cell_count; ++i) {
    if (obstacles.data[i] >= kOccupiedCostThreshold) {
      occupied[i] = 1;
      ++occupied_count;
    } else {
      free_mask[i] = 1;
    }
  }

  // 全空地图：所有格到障碍距离视为一个很大的正值，保持与历史行为一致。
  if (occupied_count == 0) {
    std::fill(distance_.begin(), distance_.end(), 999.0F);
    return;
  }
  // 全占据地图：所有格都在障碍内部，距离取 0。
  if (occupied_count == cell_count) {
    std::fill(distance_.begin(), distance_.end(), 0.0F);
    return;
  }

  // 正向距离：每个自由格到最近障碍格的欧氏距离（O(N) 精确变换）。
  const auto positive = euclidean_distance_transform(occupied);
  // 负向距离：每个障碍格到最近自由格的欧氏距离，用于构造障碍内部的真实负 SDF，
  // 使优化器在控制点不慎落入障碍时仍能得到指向外部的有效梯度。
  const auto negative = euclidean_distance_transform(free_mask);

  for (std::size_t i = 0; i < cell_count; ++i) {
    if (occupied[i] != 0) {
      // 障碍内部：负号距离。减去半个分辨率使障碍-自由边界处距离过零更连续。
      distance_[i] = -std::max(0.0F, negative[i] - 0.5F * static_cast<float>(resolution_));
    } else {
      distance_[i] = positive[i];
    }
  }
}

void EsdfMap::distance_transform_1d(
  const std::vector<double> & f, std::vector<double> & d, int n)
{
  // Felzenszwalb & Huttenlocher 一维平方距离变换，O(n)。
  // 计算下包络抛物线，求每个位置的最小 (q-v)^2 + f[v]。
  static thread_local std::vector<int> v;
  static thread_local std::vector<double> z;
  v.assign(static_cast<std::size_t>(n), 0);
  z.assign(static_cast<std::size_t>(n) + 1, 0.0);

  int k = 0;
  v[0] = 0;
  z[0] = -std::numeric_limits<double>::infinity();
  z[1] = std::numeric_limits<double>::infinity();
  const double inf = std::numeric_limits<double>::infinity();

  for (int q = 1; q < n; ++q) {
    // 跳过被障碍“源”覆盖不到的位置（f 为无穷大）。
    if (f[static_cast<std::size_t>(q)] == inf) {
      continue;
    }
    double s = 0.0;
    while (k >= 0) {
      const int vk = v[static_cast<std::size_t>(k)];
      if (f[static_cast<std::size_t>(vk)] == inf) {
        --k;
        continue;
      }
      s = ((f[static_cast<std::size_t>(q)] + static_cast<double>(q) * q) -
        (f[static_cast<std::size_t>(vk)] + static_cast<double>(vk) * vk)) /
        (2.0 * q - 2.0 * vk);
      if (s <= z[static_cast<std::size_t>(k)]) {
        --k;
      } else {
        break;
      }
    }
    ++k;
    v[static_cast<std::size_t>(k)] = q;
    z[static_cast<std::size_t>(k)] = s;
    z[static_cast<std::size_t>(k) + 1] = inf;
  }

  if (k < 0) {
    // 该行/列没有任何源，全部置为无穷大。
    std::fill(d.begin(), d.begin() + n, inf);
    return;
  }

  int j = 0;
  for (int q = 0; q < n; ++q) {
    while (z[static_cast<std::size_t>(j) + 1] < static_cast<double>(q)) {
      ++j;
    }
    const int vj = v[static_cast<std::size_t>(j)];
    d[static_cast<std::size_t>(q)] =
      static_cast<double>(q - vj) * (q - vj) + f[static_cast<std::size_t>(vj)];
  }
}

std::vector<float> EsdfMap::euclidean_distance_transform(
  const std::vector<std::uint8_t> & source) const
{
  const int w = width_;
  const int h = height_;
  const double inf = std::numeric_limits<double>::infinity();
  std::vector<double> grid(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), inf);

  // 初始化：源格平方距离为 0，其余为无穷大。
  for (std::size_t i = 0; i < source.size(); ++i) {
    if (source[i] != 0) {
      grid[i] = 0.0;
    }
  }

  std::vector<double> f(static_cast<std::size_t>(std::max(w, h)));
  std::vector<double> d(static_cast<std::size_t>(std::max(w, h)));

  // 第一遍：按列（沿 y 方向）做一维变换。
  for (int x = 0; x < w; ++x) {
    for (int y = 0; y < h; ++y) {
      f[static_cast<std::size_t>(y)] = grid[static_cast<std::size_t>(y) * w + x];
    }
    distance_transform_1d(f, d, h);
    for (int y = 0; y < h; ++y) {
      grid[static_cast<std::size_t>(y) * w + x] = d[static_cast<std::size_t>(y)];
    }
  }

  // 第二遍：按行（沿 x 方向）做一维变换。
  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      f[static_cast<std::size_t>(x)] = grid[static_cast<std::size_t>(y) * w + x];
    }
    distance_transform_1d(f, d, w);
    for (int x = 0; x < w; ++x) {
      grid[static_cast<std::size_t>(y) * w + x] = d[static_cast<std::size_t>(x)];
    }
  }

  // 平方距离开方并换算为米。
  std::vector<float> result(grid.size());
  for (std::size_t i = 0; i < grid.size(); ++i) {
    const double squared = grid[i];
    result[i] = std::isfinite(squared) ?
      static_cast<float>(std::sqrt(std::max(0.0, squared)) * resolution_) :
      999.0F;
  }
  return result;
}

double EsdfMap::query_distance(double x, double y) const
{
  return query_distance_and_gradient(x, y).first;
}

namespace
{

// 抛物线（二次）插值：给定三点 (-1, d_lo)、(0, d_mid)、(1, d_hi)，
// 在偏移 t∈[-0.5, 0.5] 处求拟合二次曲线的值与一阶导数。
// 相比双线性，二次插值在“两侧等距障碍”的峡谷中线处仍能给出非零、连续的梯度。
struct QuadraticSample
{
  double value{0.0};
  double derivative{0.0};
};

QuadraticSample quadratic_interpolate(double d_lo, double d_mid, double d_hi, double t)
{
  // f(t) = a t^2 + b t + c，其中 c=d_mid, b=(d_hi-d_lo)/2, a=(d_hi-2 d_mid+d_lo)/2
  const double a = 0.5 * (d_hi - 2.0 * d_mid + d_lo);
  const double b = 0.5 * (d_hi - d_lo);
  QuadraticSample sample;
  sample.value = a * t * t + b * t + d_mid;
  sample.derivative = 2.0 * a * t + b;
  return sample;
}

}  // namespace

std::pair<double, Point2D> EsdfMap::query_distance_and_gradient(double x, double y) const
{
  if (!valid()) {
    return {-1.0, {0.0, 0.0}};
  }
  // 连续坐标转栅格中心坐标系：格中心位于整数索引处。
  const double gx = (x - origin_.x) / resolution_ - 0.5;
  const double gy = (y - origin_.y) / resolution_ - 0.5;
  if (gx < 0.0 || gy < 0.0 || gx > width_ - 1 || gy > height_ - 1) {
    return {-1.0, {0.0, 0.0}};
  }

  // 取最近的中心格 (cx, cy)，并以它为中心做 3x3 抛物线插值。
  int cx = static_cast<int>(std::lround(gx));
  int cy = static_cast<int>(std::lround(gy));
  cx = std::clamp(cx, 1, width_ - 2);
  cy = std::clamp(cy, 1, height_ - 2);
  const double tx = gx - cx;  // ∈[-0.5, 0.5]（边缘截断时可能略微超出）
  const double ty = gy - cy;

  // 先沿 x 方向对三行各做二次插值，得到三个“行值”和对应的 x 方向导数。
  QuadraticSample row[3];
  for (int j = -1; j <= 1; ++j) {
    const int yy = cy + j;
    row[j + 1] = quadratic_interpolate(
      distance_at(cx - 1, yy), distance_at(cx, yy), distance_at(cx + 1, yy), tx);
  }

  // 再沿 y 方向对“行值”做二次插值，得到最终距离与 y 方向导数。
  const QuadraticSample col =
    quadratic_interpolate(row[0].value, row[1].value, row[2].value, ty);
  // x 方向导数：对三行的 x 导数再沿 y 做一次二次插值。
  const QuadraticSample col_dx =
    quadratic_interpolate(row[0].derivative, row[1].derivative, row[2].derivative, ty);

  const double value = col.value;
  // 链式法则：栅格坐标导数除以分辨率换算到世界坐标导数。
  const double grad_x = col_dx.value / resolution_;
  const double grad_y = col.derivative / resolution_;
  return {value, {grad_x, grad_y}};
}

std::pair<int, int> EsdfMap::world_to_grid(double x, double y) const
{
  return {
    static_cast<int>(std::floor((x - origin_.x) / resolution_)),
    static_cast<int>(std::floor((y - origin_.y) / resolution_))};
}

Point2D EsdfMap::grid_to_world(int x, int y) const
{
  return {
    origin_.x + (static_cast<double>(x) + 0.5) * resolution_,
    origin_.y + (static_cast<double>(y) + 0.5) * resolution_};
}

bool EsdfMap::in_bounds(int x, int y) const
{
  return x >= 0 && y >= 0 && x < width_ && y < height_;
}

bool EsdfMap::in_map(double x, double y) const
{
  if (!valid()) {
    return false;
  }
  const auto index = world_to_grid(x, y);
  return in_bounds(index.first, index.second);
}

double EsdfMap::distance_at(int x, int y) const
{
  return distance_[static_cast<std::size_t>(y * width_ + x)];
}

}  // namespace astra_nav
