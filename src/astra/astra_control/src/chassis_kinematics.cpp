#include "astra_control/chassis_kinematics.hpp"

#include <cmath>

namespace astra_nav
{

namespace
{
// 全向轮 45° 布置常用常数 √2/2。
constexpr double kSqrt2Over2 = 0.70710678118654752440;
}  // namespace

ChassisKinematics::ChassisKinematics(const ChassisKinematicsConfig & config)
: config_(config)
{
}

Twist2D ChassisKinematics::forward(const ChassisWheelSpeeds & wheels) const
{
  // 先把编码器原始轮速按逐轮极性修正到模型规范方向（sign²=1，保证往返一致）。
  const double fl = config_.wheel_direction_sign[0] * wheels.fl;
  const double fr = config_.wheel_direction_sign[1] * wheels.fr;
  const double rl = config_.wheel_direction_sign[2] * wheels.rl;
  const double rr = config_.wheel_direction_sign[3] * wheels.rr;
  return config_.type == ChassisType::Mecanum ? forward_mecanum(fl, fr, rl, rr)
                                              : forward_omni(fl, fr, rl, rr);
}

ChassisWheelSpeeds ChassisKinematics::inverse(const Twist2D & body_velocity) const
{
  ChassisWheelSpeeds w = config_.type == ChassisType::Mecanum ? inverse_mecanum(body_velocity)
                                                              : inverse_omni(body_velocity);
  // 规范方向 → 编码器原始方向。
  w.fl *= config_.wheel_direction_sign[0];
  w.fr *= config_.wheel_direction_sign[1];
  w.rl *= config_.wheel_direction_sign[2];
  w.rr *= config_.wheel_direction_sign[3];
  return w;
}

// —— 麦轮（标准 X 型辊子）——
// 轮序 [FL, FR, RL, RR]，体速度 vx 前向、vy 左向、wz 逆时针。
// 逆运动学矩阵各列正交，正运动学即其伪逆（最小二乘），故 forward∘inverse=I。
Twist2D ChassisKinematics::forward_mecanum(double fl, double fr, double rl, double rr) const
{
  const double r = config_.wheel_radius;
  const double k = config_.half_wheel_base_x + config_.half_wheel_base_y;  // lx+ly
  Twist2D v;
  v.vx = (r / 4.0) * (fl + fr + rl + rr);
  v.vy = (r / 4.0) * (-fl + fr + rl - rr);
  v.wz = (r / (4.0 * std::max(1.0e-6, k))) * (-fl + fr - rl + rr);
  return v;
}

ChassisWheelSpeeds ChassisKinematics::inverse_mecanum(const Twist2D & v) const
{
  const double r = std::max(1.0e-6, config_.wheel_radius);
  const double k = config_.half_wheel_base_x + config_.half_wheel_base_y;
  ChassisWheelSpeeds w;
  w.fl = (v.vx - v.vy - k * v.wz) / r;
  w.fr = (v.vx + v.vy + k * v.wz) / r;
  w.rl = (v.vx + v.vy - k * v.wz) / r;
  w.rr = (v.vx - v.vy + k * v.wz) / r;
  return w;
}

// —— 全向轮（四轮切向布置于 135°/45°/225°/315°，对应 FL/FR/RL/RR）——
// 每轮驱动方向沿安装圆切向；逆运动学 ω=(−sinβ·vx+cosβ·vy+L·wz)/r。
// 矩阵各列正交，正运动学为其伪逆，forward∘inverse=I。
Twist2D ChassisKinematics::forward_omni(double fl, double fr, double rl, double rr) const
{
  const double r = config_.wheel_radius;
  const double L = std::max(1.0e-6, config_.wheel_mount_radius);
  Twist2D v;
  v.vx = (r * kSqrt2Over2 / 2.0) * (-fl - fr + rl + rr);
  v.vy = (r * kSqrt2Over2 / 2.0) * (-fl + fr - rl + rr);
  v.wz = (r / (4.0 * L)) * (fl + fr + rl + rr);
  return v;
}

ChassisWheelSpeeds ChassisKinematics::inverse_omni(const Twist2D & v) const
{
  const double r = std::max(1.0e-6, config_.wheel_radius);
  const double L = config_.wheel_mount_radius;
  const double s = kSqrt2Over2;
  ChassisWheelSpeeds w;
  // β: FL=135°, FR=45°, RL=225°, RR=315°
  w.fl = (s * (-v.vx - v.vy) + L * v.wz) / r;
  w.fr = (s * (-v.vx + v.vy) + L * v.wz) / r;
  w.rl = (s * (v.vx - v.vy) + L * v.wz) / r;
  w.rr = (s * (v.vx + v.vy) + L * v.wz) / r;
  return w;
}

}  // namespace astra_nav
