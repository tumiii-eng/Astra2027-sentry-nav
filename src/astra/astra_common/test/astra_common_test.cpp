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
  // 速度【跨路标点连续】：这是与旧实现的本质区别。
  // 2 m 直线在 vmax=1 / amax=1 下的物理时长 = 加速 0.5 m(1 s) + 巡航 1 m(1 s) + 刹车 0.5 m(1 s)
  // = 3.0 s。旧实现给每段单独安排"静止到静止"的梯形，20 个 0.1 m 段共 20*2√0.1 ≈ 12.6 s。
  std::vector<astra_nav::Point2D> dense;
  for (int i = 0; i <= 20; ++i) {
    dense.push_back({0.1 * i, 0.0});
  }
  const auto dense_times = astra_nav::cumulative_times(dense, 1.0, 1.0);
  assert(std::abs(dense_times.back() - 3.0) < 0.05);
  // 路标点越密时长【不增反趋近物理值】：梯形积分把段内速度当线性处理，
  // 稀疏节点会高估时长，故 dense ≤ coarse（旧实现恰好相反）。
  assert(dense_times.back() <= times.back() + 1e-9);

  // 规划器实际使用的 waypoint_spacing=0.35 m + medium 档动力学：
  // 6 m 直线物理时长 = 2*(2.2/2.0) + (6 - 2*1.21)/2.2 ≈ 3.83 s。
  // 离散化误差在该间距下 <1%，取 5% 裕度。
  std::vector<astra_nav::Point2D> planner_like;
  for (int i = 0; i * 0.35 <= 6.0 + 1e-9; ++i) {
    planner_like.push_back({0.35 * i, 0.0});
  }
  const auto planner_times = astra_nav::cumulative_times(planner_like, 2.2, 2.0);
  const double span = planner_like.back().x;
  const double accel_distance = 2.2 * 2.2 / (2.0 * 2.0);
  const double expected = 2.0 * (2.2 / 2.0) + (span - 2.0 * accel_distance) / 2.2;
  assert(std::abs(planner_times.back() - expected) < 0.05 * expected);
  // 平均速度应达到 vmax 的 70% 以上（旧实现在该间距下只有 ~25%）。
  assert(span / planner_times.back() > 0.7 * 2.2);

  // 继承起点速度：起步已有速度时总时长应更短（省掉起步加速段）。
  const auto moving = astra_nav::cumulative_times(planner_like, 2.2, 2.0, 2.2);
  assert(moving.back() < planner_times.back() - 1e-3);
}

// 验证报告 5.5.3.2 思路一：含转角的时间分配应在折角处分配更多时间，
// 而对纯直线路径与纯弧长法结果一致。
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

  // 纯弧长法：直线与折角总弧长相同故总时长相同（折角处时间被低估，正是要解决的问题）。
  assert(std::abs(straight_plain.back() - corner_plain.back()) < 1e-6);
  // 含转角法：直线路径不受影响（与纯弧长法一致）。
  assert(std::abs(straight_plain.back() - straight_turn.back()) < 1e-6);
  // 含转角法：折角路径分配更多时间。
  assert(corner_turn.back() > straight_turn.back() + 1e-3);
  // 时间序列严格递增。
  for (std::size_t i = 1; i < corner_turn.size(); ++i) {
    assert(corner_turn[i] > corner_turn[i - 1]);
  }

  // 含转角法同样是全路径可达传播：折角只是加长了等效路程，不会把速度打回零。
  // 折角路径的平均等效速度仍应显著高于旧实现的分段梯形水平。
  double corner_equivalent_length = 0.0;
  for (std::size_t i = 1; i < corner.size(); ++i) {
    corner_equivalent_length += std::hypot(
      corner[i].x - corner[i - 1].x, corner[i].y - corner[i - 1].y);
  }
  assert(corner_equivalent_length / corner_turn.back() > 0.5 * vmax);
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
