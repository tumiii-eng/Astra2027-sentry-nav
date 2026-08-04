#pragma once

#include "astra_mapping/esdf_map.hpp"

namespace astra_nav
{

// 目标点可达性检测与重定位（对应报告 5.5.4.4 流程图右上“目标点优化”）。
//
// 报告原文流程：
//   1. 检查目标点的位置是否可达，如果不可达则重新规划目标点；
//   2. 通过 BFS 在目标点附近找到第一个 ESDF 值不为 0 的点；
//   3. 通过 DFS 向外扩展，根据梯度方向作为优先扩展方向，
//      直到找到第一个满足可达性的点。
//
// 这里“ESDF 值不为 0”按报告语义理解为：落入地图内、且不处于障碍内部（距离为正）的栅格，
// 即从“目标埋在障碍里/地图外”先回到一个有正向距离、能拿到有效梯度的种子点；
// 再沿 ESDF 梯度（指向远离障碍方向）做深度优先扩展，直到 clearance 满足安全距离。
struct GoalReachabilityConfig
{
  // 可达判定要求的最小安全距离（与规划 obstacle_cost_radius 对齐）。
  double required_clearance{0.55};
  // 重定位允许的最大搜索半径（米），超出则判定失败。
  double max_search_radius{1.5};
};

struct GoalReachabilityResult
{
  Point2D original;          // 输入目标点
  Point2D point;             // 输出目标点（可达则同输入，不可达则重定位结果）
  bool reachable{false};     // 输入目标点本身是否已可达
  bool success{false};       // 最终是否得到一个可达目标点
  bool relocated{false};     // 是否发生了重定位
  bool bfs_used{false};      // 是否触发了 BFS 找种子点
  bool dfs_used{false};      // 是否触发了 DFS 沿梯度扩展
  double displacement{0.0};  // 重定位位移（米）
  double original_distance{0.0};
  double final_distance{0.0};
  int bfs_expansions{0};     // BFS 扩展的栅格数（可观测指标）
  int dfs_expansions{0};     // DFS 扩展的栅格数（可观测指标）
};

class GoalReachabilityResolver
{
public:
  explicit GoalReachabilityResolver(const GoalReachabilityConfig & config);

  // 在给定 ESDF 上检测并（必要时）重定位目标点。
  GoalReachabilityResult resolve(const Point2D & goal, const EsdfMap & esdf) const;

private:
  // 可达性判定：在地图内且 ESDF 距离 >= 要求安全距离。
  bool is_reachable(const Point2D & point, const EsdfMap & esdf) const;
  // 报告第 2 步 BFS：从目标栅格四邻域广度优先扩展，找到第一个 ESDF 距离为正
  //（不在障碍内部、能拿到有效梯度）的栅格作为种子。
  bool bfs_find_seed(
    const Point2D & goal, const EsdfMap & esdf, Point2D & seed, int & expansions) const;
  // 报告第 3 步 DFS：从种子点沿 ESDF 梯度方向优先扩展，直到 clearance 满足可达。
  bool dfs_along_gradient(
    const Point2D & seed, const EsdfMap & esdf, Point2D & result, int & expansions) const;

  GoalReachabilityConfig config_;
};

}  // namespace astra_nav
