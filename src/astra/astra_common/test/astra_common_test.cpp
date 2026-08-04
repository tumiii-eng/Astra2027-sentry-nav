#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "astra_common/common.hpp"

namespace
{

// 验证通用基础库：折线重采样、角度归一化、累积时间分配。
void test_resample_polyline()
{
  const std::vector<astra_nav::Point2D> line{{0.0, 0.0}, {1.0, 0.0}};
  const auto samples = astra_nav::resample_polyline(line, 0.25);
  assert(samples.size() >= 5);
  assert(std::abs(samples.front().x) < 1e-9);
  assert(std::abs(samples.back().x - 1.0) < 1e-9);
}

void test_normalize_angle()
{
  assert(std::abs(astra_nav::normalize_angle(0.0)) < 1e-12);
  assert(std::abs(astra_nav::normalize_angle(3.0 * M_PI) - M_PI) < 1e-9);
  assert(std::abs(astra_nav::normalize_angle(-3.0 * M_PI) + M_PI) < 1e-9 ||
    std::abs(astra_nav::normalize_angle(-3.0 * M_PI) - M_PI) < 1e-9);
}

void test_cumulative_times()
{
  const std::vector<astra_nav::Point2D> points{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  const auto times = astra_nav::cumulative_times(points, 1.0, 1.0);
  assert(times.size() == points.size());
  assert(std::abs(times.front()) < 1e-9);
  for (std::size_t i = 1; i < times.size(); ++i) {
    assert(times[i] > times[i - 1]);
  }
}

// 验证报告 5.5.3.2 思路一：含转角的时间分配应在折角处分配更多时间，
// 而对纯直线路径与纯距离法结果一致。
void test_cumulative_times_with_turning()
{
  const std::vector<astra_nav::Point2D> straight{{0, 0}, {1, 0}, {2, 0}, {3, 0}};
  const std::vector<astra_nav::Point2D> corner{{0, 0}, {1, 0}, {1, 1}, {1, 2}};
  const double vmax = 1.5;
  const double amax = 1.6;
  const double k = 0.25;

  const auto straight_plain = astra_nav::cumulative_times(straight, vmax, amax);
  const auto straight_turn = astra_nav::cumulative_times_with_turning(straight, vmax, amax, k);
  const auto corner_plain = astra_nav::cumulative_times(corner, vmax, amax);
  const auto corner_turn = astra_nav::cumulative_times_with_turning(corner, vmax, amax, k);

  // 纯距离法：直线与折角总时长相同（折角处时间被低估，正是要解决的问题）。
  assert(std::abs(straight_plain.back() - corner_plain.back()) < 1e-6);
  // 含转角法：直线路径不受影响（与纯距离法一致）。
  assert(std::abs(straight_plain.back() - straight_turn.back()) < 1e-6);
  // 含转角法：折角路径分配更多时间。
  assert(corner_turn.back() > straight_turn.back() + 1e-3);
  // 时间序列严格递增。
  for (std::size_t i = 1; i < corner_turn.size(); ++i) {
    assert(corner_turn[i] > corner_turn[i - 1]);
  }
}

}  // namespace

int main()
{
  test_resample_polyline();
  test_normalize_angle();
  test_cumulative_times();
  test_cumulative_times_with_turning();
  std::cout << "astra_common 测试通过。" << std::endl;
  return 0;
}
