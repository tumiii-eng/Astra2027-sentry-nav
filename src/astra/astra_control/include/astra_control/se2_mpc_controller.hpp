#pragma once

#include <vector>

#include <Eigen/Dense>

#include "astra_common/common.hpp"

namespace astra_nav
{

// 报告 5.5.5 控制器：全向底盘 MPC，转化为稠密二次规划（QP）由 qpOASES 求解。
//
// 关键建模约定（保证报告所述“时不变的简单线性模型，矩阵静态”成立）：
//   - 状态量 x = [px, py, θ, vx, vy, ω]，位置/速度均在【世界系】；
//   - 控制量 u = [Fx, Fy, Mz]，力同样在【世界系】；
//   - 平移(px,py,vx,vy,Fx,Fy)与旋转(θ,ω,Mz)在世界系下完全解耦，故离散
//     状态转移矩阵 A 与控制矩阵 B 为常量，可在构造时一次性预构造 Hessian。
//   - 若改用机体系，则 ṗ = R(θ)v 会引入 θ 耦合，模型不再线性时不变，
//     与报告 5.5.5.4.2 第 2 点“预构造静态矩阵”矛盾。
//
// 注意：里程计 twist 习惯在机体系(base_link)，进入本控制器前需旋转到世界系；
//       输出 cmd_vel 习惯在机体系，需把世界系规划速度旋转回机体系。
struct Se2MpcConfig
{
  // —— 预测时域 —— （报告：预测步数 40 步时平均 1ms 内完成求解）
  int horizon{40};                 // 预测步数 N
  double dt{0.05};                 // 预测/控制步长（待标定）

  // —— 机器人物理参数（待用户标定）——
  double mass{15.0};               // 总质量 m
  double inertia{0.6};             // 绕 Z 轴转动惯量 Iz

  // —— 状态误差权重 Q（对角，正半定）——
  double weight_position{12.0};    // px, py
  double weight_yaw{2.0};          // θ
  double weight_velocity{1.0};     // vx, vy
  double weight_omega{0.2};        // ω

  // —— 控制输入权重 R（对角，正定）——
  double weight_force{0.02};       // Fx, Fy
  double weight_torque{0.05};      // Mz

  // —— 运动学/动力学约束（待标定）——
  double max_velocity{2.0};        // |vx|,|vy| 上限（速度盒约束）
  double max_omega{2.5};           // |ω| 上限
  double max_force{60.0};          // |Fx|,|Fy| 上限（力盒约束）
  double max_torque{20.0};         // |Mz| 上限
  double friction_coefficient{0.8};// 摩擦系数 μ（摩擦圆约束 |F| ≤ μ m g）
  double gravity{9.81};            // 重力加速度 g
  int friction_polygon_sides{8};   // 摩擦圆多边形内接边数（线性化）

  // —— 二次期望轨迹（报告 5.5.5.4.3 横向误差优先）——
  double secondary_min_scale{0.3}; // 时间步最小缩放比，越小越偏重横向精度
  bool secondary_pass_enabled{true};

  // —— 输出前瞻（报告 5.5.5.4.4：取 10ms 前瞻速度补偿延迟）——
  double command_lookahead_time{0.01};

  // —— 启动速度下限（报告 5.5.5.6 讨论的变通方法）——
  // 因为输出的是 10ms 前瞻速度而非力前馈，起步零速时前瞻速度过小，下位机 PID
  // 推力不足导致起步困难。报告的变通是在起点低速时设置速度指令下限。
  // 默认 0 表示纯报告 MPC 模型（不加下限），用户按需标定开启。
  double startup_speed_floor{0.0};

  // —— 到达判定 ——
  double arrival_position_tolerance{0.18};
  double arrival_speed_tolerance{0.05};

  // —— QP 求解器 ——
  int max_working_set_recalculations{200}; // qpOASES nWSR 上限
};

// 可观测调试量，便于多场景日志判断框架是否按报告工作。
struct Se2MpcDebug
{
  bool valid{false};               // 本周期是否产生有效 MPC 输出
  bool first_qp_success{false};    // 首次 QP 是否成功
  bool second_qp_success{false};   // 二次 QP 是否成功
  bool used_secondary{false};      // 最终是否采用二次轨迹结果
  bool stopped_by_arrival{false};  // 是否处于到达保持
  double projection_time{0.0};     // 当前位置在轨迹上的投影时刻 t0
  double horizon_duration{0.0};    // 预测时域总时长
  double secondary_min_cos{1.0};   // 二次轨迹中最小方向余弦（横向偏离指标）
  double first_qp_cost{0.0};       // 首次 QP 目标值
  double second_qp_cost{0.0};      // 二次 QP 目标值
  double nearest_position_error{0.0};
  double command_speed{0.0};       // 输出线速度大小（机体系）
  double command_omega{0.0};       // 输出角速度
  TrajectoryPoint reference_at_lookahead; // 前瞻处参考点
};

class Se2MpcController
{
public:
  explicit Se2MpcController(const Se2MpcConfig & config);

  // 计算一次控制输出。
  //   pose                 : 世界系位姿（px,py,θ）
  //   world_velocity       : 世界系线速度与角速度（vx,vy,ω），调用方负责机体系→世界系旋转
  //   trajectory           : 带时间戳与世界系速度的参考轨迹采样（按 t 递增）
  //   current_time_from_start : 轨迹起点到当前的经过时间（仅用于投影初值）
  // 返回机体系速度命令（vx,vy 在机体系，wz 同标量）。QP 失败时返回 valid=false。
  Twist2D compute(
    const Pose2D & pose, const Twist2D & world_velocity,
    const std::vector<TrajectoryPoint> & trajectory, double current_time_from_start);

  const Se2MpcDebug & last_debug() const { return debug_; }
  bool last_valid() const { return debug_.valid; }

private:
  // 预构造与模型相关的常量矩阵：A、B、Sx、Su、H、约束矩阵。
  void build_constant_matrices();

  // 在轨迹上找当前位置最近投影点，返回投影时刻 t0。
  double project_onto_trajectory(
    const std::vector<TrajectoryPoint> & trajectory, const Pose2D & pose,
    double time_hint) const;

  // 在给定时刻插值参考点（世界系位置/速度），航向取速度切线。
  TrajectoryPoint sample_reference(
    const std::vector<TrajectoryPoint> & trajectory, double t) const;

  // 按一组采样时刻构造参考状态序列 Xref（长度 6N）。
  // yaw_anchor 用于把参考航向解缠到当前航向附近：θ 是线性状态，
  // 若参考航向在 ±π 处跳变会让 QP 误判巨大航向误差。
  Eigen::VectorXd build_reference_vector(
    const std::vector<TrajectoryPoint> & trajectory,
    const std::vector<double> & sample_times, double yaw_anchor) const;

  // 求解一次稠密 QP，输入 x0 与 Xref，输出控制序列 U（长度 3N）。
  bool solve_qp(
    const Eigen::Matrix<double, 6, 1> & x0, const Eigen::VectorXd & xref,
    Eigen::VectorXd & u_out, double & cost_out) const;

  Se2MpcConfig config_;
  int n_;                // 预测步数 N（缓存）
  int nv_;               // QP 变量数 3N
  int nc_;               // QP 约束数

  // 常量矩阵（世界系 LTI 模型）。
  Eigen::Matrix<double, 6, 6> Ad_;
  Eigen::Matrix<double, 6, 3> Bd_;
  Eigen::MatrixXd Sx_;   // 6N × 6
  Eigen::MatrixXd Su_;   // 6N × 3N
  Eigen::MatrixXd Qbar_; // 6N × 6N（对角）
  Eigen::MatrixXd H_;    // 3N × 3N（常量 Hessian）
  Eigen::MatrixXd Acon_; // nc × 3N（常量约束矩阵）
  Eigen::VectorXd lb_;   // 3N 力/力矩下界
  Eigen::VectorXd ub_;   // 3N 力/力矩上界
  // 约束行的常量部分：摩擦行右端 = μmg（上界），速度行需叠加 Sx x0 偏移。
  Eigen::VectorXd con_const_upper_; // nc
  Eigen::VectorXd con_const_lower_; // nc
  int friction_rows_{0};            // 摩擦约束行数
  int velocity_rows_{0};            // 速度约束行数
  Eigen::MatrixXd vel_select_Su_;   // 速度约束对应的 Su 行（velocity_rows × 3N）
  Eigen::MatrixXd vel_select_Sx_;   // 速度约束对应的 Sx 行（velocity_rows × 6）

  Se2MpcDebug debug_;
};

}  // namespace astra_nav
