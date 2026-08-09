#pragma once

#include <cstdint>
#include <optional>
#include <vector>

#include "astra_common/common.hpp"

namespace astra_nav
{

// 轨迹按【累计弧长】的求值索引。
// 对应上游 HWSentryNav26 的 MincoTrajectory::{total_arc_length, eval_arc_length} 与
// PathSpeedProfile::eval_arc_length：上游把弧长当作跟随层唯一的进度坐标。
// Astra 的参考轨迹是时间参数化采样 std::vector<TrajectoryPoint>，故这里在采样折线上
// 建立弧长索引，把 s 映射回位置/切线/规划速度/时间参数。
struct RouteSample
{
  Point2D position;          // 对应上游 TrajSample::p
  double tangent_yaw{0.0};   // 对应上游 TrajSample::theta（几何切线航向）
  double profile_speed{0.0}; // 对应上游 speed_profile.eval_arc_length(s).velocity
  double time{0.0};          // 该弧长对应的轨迹时间参数（下游预瞄/MPC 仍按时间取参考点）
};

class RouteGeometry
{
public:
  RouteGeometry() = default;
  explicit RouteGeometry(const std::vector<TrajectoryPoint> & trajectory);

  bool empty() const { return samples_.size() < 2; }
  double total_arc_length() const { return arc_lengths_.empty() ? 0.0 : arc_lengths_.back(); }
  // 采样点里存在非零规划速度时才把速度剖面当作弱先验（对应上游 speed_profile 非空判定）。
  bool has_speed_profile() const { return has_speed_profile_; }
  RouteSample eval_arc_length(double arc_length) const;

private:
  std::vector<TrajectoryPoint> samples_;
  std::vector<double> arc_lengths_;   // 前缀弧长，size = samples_.size()，首元素 0
  std::vector<double> segment_yaws_;  // 每段弦方向，size = samples_.size() - 1
  bool has_speed_profile_{false};
};

// 与上游 RouteTrackerParams 一一对应（默认值取自上游 task_manager.yaml）。
struct RouteTrackerParams
{
  // 新路径只允许在起点附近建立初始进度 (m)。
  double initial_search_distance{0.5};
  // 与有向参考点的距离超过该值视为跟踪丢失 (m)。
  double max_tracking_error{2.0};
  // 状态掉帧后最多积分这么久的路径速度 (s)。
  double prediction_time_limit{0.2};

  // 候选进度假设的弧长间距 (m)；决定多假设网格的分辨率。
  double hypothesis_spacing{0.05};
  // 保留的竞争假设数量上限。
  int max_hypotheses{6};
  // 假设代价超过领先假设该比例裕度时被淘汰。
  double hypothesis_prune_ratio{3.0};

  // 统一 (s, nu) 状态估计器的残差标准差。
  double position_sigma{0.20};
  double velocity_sigma{0.30};
  double progress_sigma{0.15};
  double profile_speed_sigma{0.80};
  double speed_dynamics_sigma{0.30};
  // 实际路径速度状态的物理上界 (m/s)，取控制器的线速度上限
  // （运行时由 controller_node 用 max_linear_velocity / mpc_max_velocity 覆盖）。
  double max_path_speed{2.2};
};

enum class RouteTrackingStatus : std::uint8_t
{
  TRACKED = 0,
  LOST = 1,
};

struct RouteEstimate
{
  RouteTrackingStatus status{RouteTrackingStatus::LOST};
  double arc_length{0.0};
  double path_speed{0.0};
  double remaining_length{0.0};
  Point2D reference_position{};
  double tracking_error{0.0};
  // 弧长进度换算回的轨迹时间参数。下游预瞄/MPC 用它替代"墙钟经过时间"作为参考点索引。
  double reference_time{0.0};
  // s 处的几何切线航向与速度剖面速度。控制器的切/法向误差分解与前馈直接取用，
  // 对应上游 follow_problem::reference_frame 返回的 tangent 与 nominal_path_speed。
  double tangent_yaw{0.0};
  double profile_speed{0.0};
  int hypothesis_count{0};
};

// 有向路径进度观测器（逐行移植 HWSentryNav26 nav_executor::RouteTracker）。
//
// 进度是有向路径上的时序状态，不是每帧独立的最近点：
//   - 位置与地图系速度矢量共同观测 (s, nu)，速度剖面仅作为 nu 的弱先验；
//   - 维护多个竞争的弧长假设，错误分支随后续观测被淘汰，不会因单次错误最近点永久锁死；
//   - 前进跟随时进度单调不减，杜绝沿错误分支回退。
//
// 与上游的唯一实现差异：上游底盘非全向，只有前向标量速度，需用 chassis_velocity*(cosθ,sinθ)
// 重建地图系速度矢量；pb2025 是四全向轮底盘，地图系速度矢量本身可直接观测，故直接传入。
class RouteTracker
{
public:
  RouteTracker() = default;
  explicit RouteTracker(const RouteTrackerParams & params)
  : params_(params) {}

  void set_params(const RouteTrackerParams & params) { params_ = params; }

  // trajectory_revision：轨迹版本号。变化即视为"新路径"，清空假设集与进度下界
  // （对应上游 `path != path_` 的指针身份比较）。
  // world_velocity：机器人在【世界系】的真实速度。
  // stamp_seconds：本帧时间戳（ROS 时钟秒，受 use_sim_time 控制）。
  std::optional<RouteEstimate> update(
    const std::vector<TrajectoryPoint> & trajectory, std::uint64_t trajectory_revision,
    const Pose2D & pose, const Twist2D & world_velocity, double stamp_seconds);

  void reset();

private:
  // 一个有向进度假设：弧长、路径方向速度，以及累计代价（越小越可信）。
  struct Hypothesis
  {
    double arc_length{0.0};
    double path_speed{0.0};
    double cost{0.0};
  };

  RouteTrackerParams params_{};
  RouteGeometry geometry_{};
  std::uint64_t revision_{0};
  bool has_geometry_{false};
  std::vector<Hypothesis> hypotheses_;
  // 已上报进度的下界。分支切换不得让对外可见的进度回退。
  double reported_arc_length_{0.0};
  std::optional<double> last_stamp_;
};

}  // namespace astra_nav
