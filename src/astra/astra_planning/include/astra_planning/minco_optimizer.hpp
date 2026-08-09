#pragma once

#include <vector>

#include <Eigen/Eigen>

#include "astra_common/common.hpp"
#include "astra_mapping/cost_map_sampler.hpp"
#include "astra_planning/minco_trajectory.hpp"

namespace astra_nav
{

// MINCO 后端优化阶段：对应报告 5.5.4.2 的两步优化策略（PRE 迭代少、FINELY 迭代多）。
// 障碍罚在两阶段完全同形——都直接采样连续代价场的解析双线性梯度。
// 原先 FINELY 阶段的“扣除切向 + 势谷探测”是为了绕开硬膨胀 ESDF 在峡谷中线上
// 梯度恒为零的缺陷；连续代价场本身处处可微且远处可见，按 HWSentryNav26 的做法不再需要。
enum class MincoOptimizeStage
{
  PreOptimization,
  FinelyOptimization
};

struct MincoOptimizerConfig
{
  // 动力学软约束上限。
  double max_velocity{1.5};
  double max_acceleration{1.6};
  // 各惩罚项权重。
  double weight_energy{1.0};         // 平滑（jerk 能量）
  double weight_time{32.0};          // 总时间，鼓励更快完成
  double weight_obstacle{1000.0};    // 障碍软约束
  double weight_velocity{200.0};     // 速度软约束
  double weight_acceleration{200.0}; // 加速度软约束
  // 段时间相对均值的均匀性约束（仅 PRE 阶段启用），对应报告 5.5.4.2 末段的时间约束。
  double weight_uniform_time{120.0};
  double uniform_time_upper_ratio{1.1};
  double uniform_time_lower_ratio{0.9};
  // 每段轨迹上的约束采样点数（数值积分分辨率，取偶数配合辛普森积分）。
  int samples_per_piece{16};
  // L-BFGS 参数。
  int max_iterations{60};
  int memory_size{16};
  int past{3};
  double delta{1.0e-5};
  double g_epsilon{1.0e-5};
};

struct MincoOptimizerResult
{
  MincoTrajectory trajectory;
  bool optimized{false};
  int status{0};
  int iterations{0};
  double initial_cost{0.0};
  double final_cost{0.0};
};

// 基于 MINCO 控制点（内部路标点）与分段时间的两步无约束优化器。
//
// 变量：内部路标点 P(2 x (N-1)) 与虚拟时间 tau(N)（经 C2 连续映射保证段时间恒正）。
// 代价：MINCO 平滑能量 + 总时间 + 障碍/速度/加速度软约束（沿轨迹采样积分）+ 段时间均匀约束。
// 梯度：解析的 dCost/dCoeffs、dCost/dTimes 经 MINCO 的 propogateGrad 汇总到 dCost/dP、dCost/dT，
//       再经时间反向映射得到 dCost/dtau，交给 L-BFGS。
//
// 这与报告 5.5.4.2 “把离散采样的 gradPos/gradVel/gradAcc 汇总到控制点 GradC/GradT 上” 一致。
class MincoOptimizer
{
public:
  explicit MincoOptimizer(const MincoOptimizerConfig & config);

  // 在给定初始路标点、初始分段时间与边界状态下，按指定阶段优化一次。
  // waypoints 含首末端点（首=起点、末=终点），times 为累积时间（首项为 0）。
  MincoOptimizerResult optimize(
    const std::vector<Point2D> & waypoints, const std::vector<double> & times,
    const MincoBoundaryState & head, const MincoBoundaryState & tail,
    const Grid2D & cost_map, MincoOptimizeStage stage) const;

  const MincoOptimizerConfig & config() const { return config_; }

private:
  MincoOptimizerConfig config_;
};

}  // namespace astra_nav
