#pragma once

#include <vector>

#include "astra_mapping/esdf_map.hpp"
#include "astra_planning/minco_optimizer.hpp"
#include "astra_planning/minco_trajectory.hpp"
#include "astra_planning/waypoint_lbfgs_optimizer.hpp"

namespace astra_nav
{

struct TrajectoryOptimizerConfig
{
  double max_velocity{1.5};
  double max_acceleration{1.6};
  double waypoint_spacing{0.35};
  double obstacle_cost_radius{0.55};
  double obstacle_step{0.06};
  // 含转角的时间分配权重（报告 5.5.3.2 思路一的 k_turn）。0 表示退化为纯直线时间分配。
  double turn_time_weight{0.25};
  int pre_iterations{30};
  int fine_iterations{45};
  double lbfgs_reference_weight{0.08};
  double lbfgs_smooth_weight{0.45};
  double lbfgs_obstacle_weight{3.0};
  int lbfgs_memory_size{8};
  int lbfgs_past{3};
  double lbfgs_delta{1.0e-5};
  bool dynamic_time_scaling_enabled{true};
  int dynamic_time_scaling_max_iterations{4};
  double dynamic_time_scaling_safety_ratio{1.03};
  double initial_time_scale{1.0};

  // 是否启用基于 MINCO 控制点/分段时间梯度的两步后端优化（对应报告 5.5.4）。
  // 关闭时回退到旧的“路点级 L-BFGS + 事后拟合 MINCO”实现，便于对比与排查。
  bool use_minco_backend{true};
  // MINCO 两步后端优化参数（仅 use_minco_backend 为真时生效）。
  double minco_weight_energy{1.0};
  double minco_weight_time{16.0};
  double minco_weight_obstacle{2000.0};
  double minco_weight_velocity{200.0};
  double minco_weight_acceleration{200.0};
  double minco_weight_uniform_time{120.0};
  double minco_valley_gradient_threshold{0.5};
  int minco_samples_per_piece{16};
  int minco_pre_iterations{60};
  int minco_fine_iterations{60};
  // L-BFGS 梯度收敛阈值 ||g||_inf/max(1,||x||_inf)<g_epsilon，仅门控"梯度达标收敛"退出。
  // 长轨迹梯度受 weight_obstacle 主导量级较大，按报告 5.5.4.1 走迭代上限正常退出(-1008)，非错误。
  double minco_g_epsilon{1.0e-5};
  // 回环检测（轨迹中段“甩圈/绕圈”自诊断与自动恢复）：
  // 沿弧长滑窗累计【不解缠】航向变化，窗口内净转角超阈值即判为回环。
  // 正常折角单窗最多约 135°，阈值取 270° 不会误报。检测到后：
  //   - FINELY 结果有环而 PRE 结果无环 → 回退用 PRE（报告 5.5.4.2：PRE 形状稳定）；
  //   - 时间缩放重拟合引入环 → 撤销该次缩放。
  bool loop_detection_enabled{true};
  double loop_detection_window{1.2};          // 弧长滑窗长度（米）
  double loop_detection_angle_threshold{4.71}; // 净转角阈值（弧度，默认 270°）
  // 方向感知的起点继承（修复高速冲过拐弯点后“向前甩圈而非往回拉”）：
  // 机器人因速度过快冲过路点时，继承的速度方向与新前端路径相反（v·d<0），
  // 若直接作为 MINCO 起点硬约束，样条被迫先顺该速度冲出再绕回，形成回环。
  // 开启后只剔除与路径方向【相反】的纵向分量，保留顺路径与侧向分量，
  // 使新轨迹能干净地把机器人拉回，同时不破坏正常跟踪时的连续性。
  bool directional_start_inheritance{true};
};

struct TrajectoryBoundaryCondition
{
  bool inherit_start_state{false};
  Point2D start_velocity;
  Point2D start_acceleration;
};

struct TrajectoryQuality
{
  double max_velocity{0.0};
  double max_acceleration{0.0};
  double min_esdf_distance{0.0};
  double velocity_violation{0.0};
  double acceleration_violation{0.0};
};

struct TrajectoryOptimizationStats
{
  int input_points{0};
  int resampled_points{0};
  int pre_lbfgs_status{0};
  int pre_lbfgs_iterations{0};
  double pre_initial_cost{0.0};
  double pre_final_cost{0.0};
  bool pre_optimized{false};
  int fine_lbfgs_status{0};
  int fine_lbfgs_iterations{0};
  double fine_initial_cost{0.0};
  double fine_final_cost{0.0};
  bool fine_optimized{false};
  double optimize_time_ms{0.0};
  bool start_state_inherited{false};
  int time_scaling_iterations{0};
  double time_scaling_factor{1.0};
  bool used_minco_backend{false};
  // FINELY 阶段势谷分支命中的采样点次数（报告 5.5.4.2 梯度无效化处理，可观测指标）。
  int finely_valley_hits{0};
  // —— 回环检测诊断（轨迹中段甩圈/绕圈）——
  bool loop_detected{false};          // 最终输出轨迹是否仍含回环
  bool loop_recovered{false};         // 是否通过回退 PRE / 撤销缩放消除了回环
  double loop_max_window_angle{0.0};  // 检测到的最大滑窗净转角（弧度）
  double loop_position_x{0.0};        // 回环大致发生位置（窗口中点世界坐标）
  double loop_position_y{0.0};
  TrajectoryQuality quality;
};

struct TrajectoryOptimizationResult
{
  MincoTrajectory trajectory;
  TrajectoryOptimizationStats stats;
};

class TrajectoryOptimizer
{
public:
  explicit TrajectoryOptimizer(const TrajectoryOptimizerConfig & config);
  MincoTrajectory optimize(const std::vector<Point2D> & coarse_path, const EsdfMap & esdf) const;
  TrajectoryOptimizationResult optimize_with_stats(
    const std::vector<Point2D> & coarse_path, const EsdfMap & esdf) const;
  TrajectoryOptimizationResult optimize_with_stats(
    const std::vector<Point2D> & coarse_path, const EsdfMap & esdf,
    const TrajectoryBoundaryCondition & boundary_condition) const;

private:
  WaypointLbfgsResult optimize_waypoints(
    const std::vector<Point2D> & input, const EsdfMap & esdf, int iterations,
    double obstacle_gain) const;
  MincoTrajectory fit_minco(
    const std::vector<Point2D> & points, const std::vector<double> & times,
    const TrajectoryBoundaryCondition & boundary_condition) const;
  // 基于 MINCO 控制点/分段时间梯度的两步后端优化主流程（报告 5.5.4）。
  TrajectoryOptimizationResult optimize_minco_backend(
    const std::vector<Point2D> & waypoints, const EsdfMap & esdf,
    const TrajectoryBoundaryCondition & boundary_condition) const;
  // 对已优化轨迹做双向时间缩放，使巡航速度收敛到 max_velocity（受 max_acceleration 约束）。
  // 这一步把“轨迹形状”（由 MINCO 两步优化决定）与“轨迹速度”（由此处对时间整体缩放决定）
  // 解耦，使 max_velocity / max_acceleration 成为真正生效、可纯参数调节的速度上限。
  void apply_dynamic_time_scaling(
    TrajectoryOptimizationResult & output,
    const TrajectoryBoundaryCondition & boundary_condition, const EsdfMap & esdf) const;
  MincoOptimizerConfig make_minco_config() const;
  void scale_times(std::vector<double> & times, double scale) const;
  TrajectoryQuality evaluate_quality(const MincoTrajectory & trajectory, const EsdfMap & esdf) const;
  // 回环检测：沿弧长滑窗累计【不解缠】航向变化，返回窗口内最大净转角（弧度）。
  // out_x/out_y 返回该最大转角窗口的中点世界坐标，便于日志定位甩圈位置。
  // 空轨迹或过短返回 0。
  double detect_loop(
    const MincoTrajectory & trajectory, double & out_x, double & out_y) const;
  // 规整继承的起点边界：①方向感知——剔除与路径方向 from->to【相反】的纵向分量
  // （v·d<0，修复冲过拐弯点后甩圈）；②幅值钳制——把继承的速度/加速度钳到动力学上限，
  // 避免剧烈机动时继承的瞬时加速度逼出不可行轨迹（继承正反馈）。
  void regularize_inherited_boundary(
    MincoBoundaryState & head, const Point2D & from, const Point2D & to) const;

  TrajectoryOptimizerConfig config_;
};

}  // namespace astra_nav
