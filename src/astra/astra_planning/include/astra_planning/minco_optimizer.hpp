#pragma once

#include <vector>

#include <Eigen/Eigen>

#include "astra_common/common.hpp"
#include "astra_mapping/esdf_map.hpp"
#include "astra_planning/minco_trajectory.hpp"

namespace astra_nav
{

// MINCO 后端优化阶段：对应报告 5.5.4.2 的两步优化策略。
// PRE_OPTIMIZATION：障碍梯度各向同性（直接用二次插值梯度），快速得到稳定形状；
// FINELY_OPTIMIZATION：考虑速度方向，扣除沿轨迹方向分量并按势谷阈值特殊处理，得到良好动力学特性。
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
  // 期望与障碍保持的安全距离（ESDF 距离低于该值开始受罚）。
  double safe_distance{0.55};
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
  // FINELY 阶段的势谷判定阈值（报告取定 0.5）。
  double valley_gradient_threshold{0.5};
  // 势谷处理：violaPos 大小 = valley_scale * sqrt(gradPos.norm())（报告 5.5.4.2）。
  double valley_scale{1.0};
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
  // FINELY 阶段势谷分支命中的采样点次数（可观测指标，报告 5.5.4.2 梯度无效化处理）。
  int valley_hits{0};
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
    const EsdfMap & esdf, MincoOptimizeStage stage) const;

  const MincoOptimizerConfig & config() const { return config_; }

private:
  MincoOptimizerConfig config_;
};

}  // namespace astra_nav
