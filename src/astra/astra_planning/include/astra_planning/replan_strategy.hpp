#pragma once

#include "astra_common/common.hpp"
#include "astra_mapping/esdf_map.hpp"
#include "astra_planning/minco_trajectory.hpp"

namespace astra_nav
{

// 重规划三策略（报告 5.5.4.4 流程图下半“模式选择”）。
//
// 报告把重规划分为三种模式：
//   - 完全重规划（FullReplan）：定位严重偏离轨迹，或目标点大幅跳变。
//       从当前位置到目标点重新做 JPS 前端搜索。
//   - 部分重规划（PartialReplan）：非完全、也非仅优化的中间情况。
//       取当前位置在上一轨迹上的映射点，记录映射点位置 s 和速度 v，
//       从 s 到目标点做 JPS，再做时间重采样。映射点之前已走过的一小段轨迹保留在前方，
//       使新轨迹与老轨迹在当前机器人位置上基本不跳变。
//   - 仅优化（OptimizeOnly）：轨迹无碰撞、目标不跳变、机器人跟踪良好。
//       不重新 JPS，只取当前位置映射点，从重映射位置开始等时间间隔重采样后送优化。
enum class ReplanMode
{
  FullReplan,
  PartialReplan,
  OptimizeOnly
};

struct ReplanDecisionConfig
{
  // 目标点跳变阈值（米）：超过则判定为“目标大幅跳变”，触发完全重规划。
  double goal_change_threshold{0.5};
  // 定位严重偏离阈值（米）：当前位置到上一轨迹最近映射点距离超过则触发完全重规划。
  double localization_deviation_threshold{0.6};
  // 轨迹碰撞安全距离（米）：上一轨迹前方采样点 ESDF 低于此值视为有碰撞风险。
  double collision_clearance{0.05};
  // 跟踪良好阈值（米）：当前位置到映射点距离小于此值视为机器人跟踪良好。
  double good_tracking_threshold{0.25};
  bool partial_enabled{true};
  bool optimize_only_enabled{true};
};

struct ReplanDecision
{
  ReplanMode mode{ReplanMode::FullReplan};
  // 部分重规划 / 仅优化模式下，当前位置在上一轨迹上的映射点参数 t 与状态。
  double mapped_t{0.0};
  Point2D mapped_position;
  Point2D mapped_velocity;
  // 触发判据可观测量。
  double goal_change{0.0};
  double localization_deviation{0.0};
  double min_trajectory_clearance{0.0};
  bool trajectory_collision_free{true};
  bool goal_stable{true};
  bool tracking_good{true};
};

class ReplanStrategy
{
public:
  explicit ReplanStrategy(const ReplanDecisionConfig & config);

  // 根据上一轨迹、当前位置、当前/上一目标和 ESDF，决定本轮重规划模式。
  //   has_previous：是否存在可继承的上一轨迹（首次规划必为完全重规划）。
  //   has_last_goal：是否存在上一轮的全局目标（用于判断目标跳变）。
  ReplanDecision decide(
    const MincoTrajectory & previous_trajectory, bool has_previous,
    const Point2D & current_position, const Point2D & current_goal,
    const Point2D & last_goal, bool has_last_goal, const EsdfMap & esdf) const;

  static const char * mode_name(ReplanMode mode);

private:
  // 检测上一轨迹从映射点到末端的前方部分是否无碰撞，并返回最小 clearance。
  bool forward_collision_free(
    const MincoTrajectory & trajectory, double from_t, const EsdfMap & esdf,
    double & min_clearance) const;

  ReplanDecisionConfig config_;
};

}  // namespace astra_nav
