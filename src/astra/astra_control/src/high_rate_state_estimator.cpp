#include "astra_control/high_rate_state_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace astra_nav
{

HighRateStateEstimator::HighRateStateEstimator(const HighRateStateEstimatorConfig & config)
: config_(config)
{
}

void HighRateStateEstimator::set_anchor(
  const Pose2D & pose, const Twist2D & world_velocity, double stamp)
{
  anchor_pose_ = pose;
  anchor_world_velocity_ = world_velocity;
  anchor_stamp_ = stamp;
  has_anchor_ = true;
}

void HighRateStateEstimator::add_motion_sample(const MotionSample & sample)
{
  // 维持按时间戳升序：绝大多数情况下新样本最新，直接尾插；
  // IMU 与底盘体速度来自不同话题，偶有乱序时按位置插入。
  if (samples_.empty() || sample.stamp >= samples_.back().stamp) {
    samples_.push_back(sample);
  } else {
    auto it = std::upper_bound(
      samples_.begin(), samples_.end(), sample,
      [](const MotionSample & a, const MotionSample & b) { return a.stamp < b.stamp; });
    samples_.insert(it, sample);
  }

  // 按缓冲时长修剪旧样本，限制内存与积分回放开销。
  const double newest = samples_.back().stamp;
  while (!samples_.empty() && samples_.front().stamp < newest - config_.max_buffer_time) {
    samples_.pop_front();
  }
}

void HighRateStateEstimator::integrate_step(
  double dt, double & x, double & y, double & yaw,
  double body_vx, double body_vy, double yaw_rate) const
{
  // 位置：用体速度时按当前航向旋到世界系（全向底盘成立），否则用锚点世界速度。
  double vx_world;
  double vy_world;
  if (config_.use_body_velocity) {
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    vx_world = cy * body_vx - sy * body_vy;
    vy_world = sy * body_vx + cy * body_vy;
  } else {
    vx_world = anchor_world_velocity_.vx;
    vy_world = anchor_world_velocity_.vy;
  }
  x += vx_world * dt;
  y += vy_world * dt;

  // 航向：用 IMU 航向角速度积分，否则用锚点角速度。
  const double wz = config_.use_imu_yaw_rate ? yaw_rate : anchor_world_velocity_.wz;
  yaw += wz * dt;
}

EstimatedState HighRateStateEstimator::estimate(double query_time) const
{
  EstimatedState out;
  if (!has_anchor_) {
    return out;  // 无锚点：extrapolated=false，调用方应先判断 has_anchor()
  }

  const double age = query_time - anchor_stamp_;
  out.anchor_age = age;
  // 默认（不外推）直接返回锚点状态。
  out.pose = anchor_pose_;
  out.world_velocity = anchor_world_velocity_;

  // 查询时刻早于/等于锚点，或锚点过旧（高频数据可能缺失）：不外推，避免积分发散，
  // 退化为锚点兜底（锚点来自激光惯性里程计，打滑免疫）。
  if (age <= 1.0e-6 || age > config_.max_extrapolation_time) {
    return out;
  }

  // —— 局部前向积分（报告 5.5.5.4.1）——
  double x = anchor_pose_.x;
  double y = anchor_pose_.y;
  double yaw = anchor_pose_.yaw;
  double cur_w = anchor_world_velocity_.wz;
  // 把锚点世界系速度旋转到锚点机体系，作为体速度积分的初值（首个样本到来前沿用）。
  const double ca = std::cos(anchor_pose_.yaw);
  const double sa = std::sin(anchor_pose_.yaw);
  double cur_body_vx = ca * anchor_world_velocity_.vx + sa * anchor_world_velocity_.vy;
  double cur_body_vy = -sa * anchor_world_velocity_.vx + ca * anchor_world_velocity_.vy;

  double t = anchor_stamp_;
  double last_body_stamp = anchor_stamp_;  // 上一已接受体速度的时间戳（打滑闸门用）
  int count = 0;
  int rejected = 0;
  for (const auto & s : samples_) {
    if (s.stamp <= anchor_stamp_ + 1.0e-9) {
      continue;  // 锚点之前/同时的样本不参与外推
    }
    if (s.stamp > query_time) {
      break;  // 超过查询时刻的样本留待下一周期
    }
    const double dt = s.stamp - t;
    if (dt > 0.0) {
      integrate_step(dt, x, y, yaw, cur_body_vx, cur_body_vy, cur_w);
      t = s.stamp;
    }
    // 应用该样本携带的运动输入，更新后续积分使用的体速度/航向角速度。
    if (s.has_yaw_rate && config_.use_imu_yaw_rate) {
      cur_w = s.yaw_rate;  // IMU 陀螺不打滑，直接采用。
    }
    if (s.has_body_velocity && config_.use_body_velocity) {
      // —— 轻量打滑闸门 ——
      // 相邻体速度隐含加速度超阈值视为打滑，拒绝该样本（沿用上一已接受体速度）。
      bool accept = true;
      if (config_.enable_slip_gate) {
        const double dt_b = s.stamp - last_body_stamp;
        if (dt_b > 1.0e-6) {
          const double dvx = s.body_vx - cur_body_vx;
          const double dvy = s.body_vy - cur_body_vy;
          const double implied_accel = std::hypot(dvx, dvy) / dt_b;
          if (implied_accel > config_.max_body_acceleration) {
            accept = false;
            ++rejected;
          }
        }
      }
      if (accept) {
        cur_body_vx = s.body_vx;
        cur_body_vy = s.body_vy;
        last_body_stamp = s.stamp;
      }
    }
    ++count;
  }
  // 最后一段：从最近样本（或锚点）积分到查询时刻。
  const double dt_final = query_time - t;
  if (dt_final > 0.0) {
    integrate_step(dt_final, x, y, yaw, cur_body_vx, cur_body_vy, cur_w);
  }

  out.pose.x = x;
  out.pose.y = y;
  out.pose.yaw = normalize_angle(yaw);

  // 输出世界系速度：用体速度则按最终航向旋回世界系，否则沿用锚点世界速度。
  if (config_.use_body_velocity) {
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);
    out.world_velocity.vx = cy * cur_body_vx - sy * cur_body_vy;
    out.world_velocity.vy = sy * cur_body_vx + cy * cur_body_vy;
  } else {
    out.world_velocity.vx = anchor_world_velocity_.vx;
    out.world_velocity.vy = anchor_world_velocity_.vy;
  }
  out.world_velocity.wz = config_.use_imu_yaw_rate ? cur_w : anchor_world_velocity_.wz;

  out.extrapolated = true;
  out.integrated_samples = count;
  out.rejected_slip_samples = rejected;
  return out;
}

}  // namespace astra_nav
