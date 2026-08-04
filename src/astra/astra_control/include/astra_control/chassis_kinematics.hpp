#pragma once

#include <array>

#include "astra_common/common.hpp"

namespace astra_nav
{

// 底盘正/逆运动学（报告 5.5.5.4.1 用“轮速”做高频积分的换算层）。
//
// 设计动机：仿真底盘是麦轮(mecanum)，实车是全向轮(omni)。两者「裸轮速→体速度」
//   的运动学矩阵不同，但「体速度→世界位姿」的积分与轮型无关。把轮型相关的部分
//   全部锁在本类里，高频积分估计器只消费体速度，从而仿真/实车切换只需改参数。
//
// 约定：
//   - 四轮固定顺序 [前左 FL, 前右 FR, 后左 RL, 后右 RR]，轮速为角速度 (rad/s)。
//     上游（仿真 odom 发布端 / 实车下位机 / JointState 映射）负责按此顺序提供。
//   - 体速度 Twist2D 在机体系：vx 前向、vy 左向、wz 逆时针为正。
//   - 麦轮：标准 X 型辊子布局，几何参数 half_wheel_base_x(lx)、half_wheel_base_y(ly)。
//   - 全向轮：四轮切向布置于 45°/135°/225°/315°，安装半径 wheel_mount_radius(L)。
//   - 各平台编码器极性不同，提供 wheel_direction_sign 逐轮符号修正（默认 +1，需标定）。
//
// 注意：本类是纯算法、与 ROS 无关，便于单元测试（forward∘inverse 往返一致）。
//   报告把完整抗打滑置于 5.1 Batch-LIWO 轮速置信度融合层；本类不做置信度，
//   打滑抑制由 HighRateStateEstimator 的轻量闸门近似承担。
enum class ChassisType
{
  Mecanum,  // 四麦轮
  Omni      // 四全向轮（45° 切向布置）
};

struct ChassisKinematicsConfig
{
  ChassisType type{ChassisType::Mecanum};
  double wheel_radius{0.0762};        // 轮半径 r (m)（待标定）
  double half_wheel_base_x{0.2};      // 麦轮：前后轮距一半 lx (m)
  double half_wheel_base_y{0.2};      // 麦轮：左右轮距一半 ly (m)
  double wheel_mount_radius{0.3};     // 全向轮：安装半径 L (m)
  std::array<double, 4> wheel_direction_sign{{1.0, 1.0, 1.0, 1.0}};  // 逐轮极性修正
};

// 四轮角速度 (rad/s)，顺序 [FL, FR, RL, RR]。
struct ChassisWheelSpeeds
{
  double fl{0.0};
  double fr{0.0};
  double rl{0.0};
  double rr{0.0};
};

class ChassisKinematics
{
public:
  explicit ChassisKinematics(const ChassisKinematicsConfig & config);

  // 正运动学：裸轮速 → 机体系体速度。
  Twist2D forward(const ChassisWheelSpeeds & wheels) const;

  // 逆运动学：机体系体速度 → 裸轮速（主要用于单测往返验证与调试）。
  ChassisWheelSpeeds inverse(const Twist2D & body_velocity) const;

  const ChassisKinematicsConfig & config() const { return config_; }

private:
  Twist2D forward_mecanum(double fl, double fr, double rl, double rr) const;
  Twist2D forward_omni(double fl, double fr, double rl, double rr) const;
  ChassisWheelSpeeds inverse_mecanum(const Twist2D & v) const;
  ChassisWheelSpeeds inverse_omni(const Twist2D & v) const;

  ChassisKinematicsConfig config_;
};

}  // namespace astra_nav
