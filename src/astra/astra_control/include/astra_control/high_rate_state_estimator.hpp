#pragma once

#include <deque>

#include "astra_common/common.hpp"

namespace astra_nav
{

// 报告 5.5.5.4.1「定位同步与估计」：
//   上游 Point-LIO 里程计有延迟且频率较低（报告 100Hz，仿真约 20Hz），
//   而控制器需要在更高频率（50Hz~1kHz）拿到“当前时刻”的位姿。
//   做法是把有延迟的里程计位姿当作【锚点】，借助高频 IMU 陀螺仪（航向角速度）
//   与高频底盘体速度，从锚点【局部前向积分】到控制查询时刻，估计最新位姿。
//
// 本类是纯算法、与 ROS 无关，便于单元测试：
//   - set_anchor：写入最新的（有延迟的）里程计位姿与【世界系】速度作为积分起点；
//   - add_motion_sample：写入高频运动样本（IMU 航向角速度 / 底盘体速度）；
//   - estimate：给定控制时刻，回放锚点之后的高频样本做积分，给出最新状态估计。
//
// 坐标系约定：锚点速度与输出 world_velocity 均在【世界系】；
//   运动样本里的体速度在【机体系】，积分时按当前估计航向旋转到世界系。
//   体速度本身由上游 ChassisKinematics 从裸轮速换算（麦轮/全向轮差异锁在那一层，
//   不会泄漏到本估计器）。
//
// 抗打滑（报告把完整抗打滑置于 5.1 Batch-LIWO 轮速置信度融合层，本轮未做该层）：
//   这里实现一个【轻量打滑闸门】+【锁点兑底】作为近似——
//   锚点来自激光惯性里程计（不依赖轮速，打滑免疫），航向用 IMU 陀螺积分（陀螺不打滑），
//   唯一受打滑影响的是“用体速度积分位置”。当相邻体速度样本隐含加速度超过阈值时，
//   判定为打滑、拒绝该样本（沿用上一已接受体速度）；高频数据全缺失时退化为
//   用锚点世界速度匀速外推，即纯靠打滑免疫的 LIO 锚点兑底。
struct HighRateStateEstimatorConfig
{
  // 锚点距查询时刻超过该时长则不再外推（避免高频数据缺失时积分发散），直接返回锚点位姿。
  double max_extrapolation_time{0.2};
  // 高频样本缓冲保留时长，超过则丢弃旧样本，限制内存与计算量。
  double max_buffer_time{0.5};
  // 是否用 IMU 陀螺仪航向角速度积分航向；关闭时航向角速度恒取锚点角速度。
  bool use_imu_yaw_rate{true};
  // 是否用高频体速度积分位置；关闭时退化为用锚点世界速度匀速外推（打滑免疫兜底）。
  bool use_body_velocity{true};
  // 是否启用轻量打滑闸门。
  bool enable_slip_gate{true};
  // 打滑判据：相邻体速度样本隐含加速度（m/s²）超过此阈值则判为打滑并拒绝该样本。
  // 默认值偏保守，需结合实车底盘动力学标定。
  double max_body_acceleration{12.0};
};

// 高频运动样本：可来自 IMU（航向角速度）或底盘体速度反馈，按时间戳同步。
// 两类字段相互独立，单条样本可只携带其中一类（IMU 只给 yaw_rate，底盘只给体速度）。
struct MotionSample
{
  double stamp{0.0};            // 样本时间戳（秒，与里程计/控制查询同一时钟）
  double yaw_rate{0.0};         // 航向角速度（rad/s，机体 z 轴与世界 z 轴一致）
  bool has_yaw_rate{false};     // 本样本是否携带航向角速度
  double body_vx{0.0};          // 体速度 x（m/s，机体系）
  double body_vy{0.0};          // 体速度 y（m/s，机体系）
  bool has_body_velocity{false};// 本样本是否携带体速度
};

// 估计结果。
struct EstimatedState
{
  Pose2D pose;                  // 估计的世界系位姿（控制查询时刻）
  Twist2D world_velocity;       // 估计的世界系线速度与角速度（喂给 MPC 的 x0）
  bool extrapolated{false};     // 是否真正做了前向积分（false=直接返回锚点）
  double anchor_age{0.0};       // 查询时刻距锚点时间戳的时长（秒）
  int integrated_samples{0};    // 本次积分回放的高频样本数
  int rejected_slip_samples{0}; // 本次被打滑闸门拒绝的体速度样本数
};

class HighRateStateEstimator
{
public:
  explicit HighRateStateEstimator(const HighRateStateEstimatorConfig & config);

  // 写入/更新里程计锚点：世界系位姿、世界系速度、时间戳（秒）。
  void set_anchor(const Pose2D & pose, const Twist2D & world_velocity, double stamp);

  // 写入一条高频运动样本（IMU/底盘体速度）。内部按时间戳维持有序并按 max_buffer_time 修剪。
  void add_motion_sample(const MotionSample & sample);

  // 估计给定控制时刻（秒）的最新状态。无锚点时返回 extrapolated=false 且位姿为零。
  EstimatedState estimate(double query_time) const;

  bool has_anchor() const { return has_anchor_; }
  double anchor_stamp() const { return anchor_stamp_; }

  // 测试/调试用：当前缓冲中的高频样本数。
  std::size_t buffer_size() const { return samples_.size(); }

private:
  // 单步积分：按当前航向把体速度旋到世界系更新位置，用航向角速度更新航向。
  void integrate_step(
    double dt, double & x, double & y, double & yaw,
    double body_vx, double body_vy, double yaw_rate) const;

  HighRateStateEstimatorConfig config_;

  bool has_anchor_{false};
  Pose2D anchor_pose_;
  Twist2D anchor_world_velocity_;
  double anchor_stamp_{0.0};

  std::deque<MotionSample> samples_;  // 按 stamp 升序
};

}  // namespace astra_nav
