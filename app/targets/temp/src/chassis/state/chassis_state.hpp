#ifndef BALANCE_CHASSIS_CHASSIS_STATE_HPP
#define BALANCE_CHASSIS_CHASSIS_STATE_HPP

#include "speed_estimator.hpp"
#include "wbr/leg_kinematics.hpp"
#include "wbr/wbr_controller.hpp"

/**
 * @file  chassis/state/chassis_state.hpp
 * @brief 底盘状态估计器接口定义
 */

/**
 * @brief 单个关节反馈
 * @note  统一使用 SI 单位
 */
struct JointFeedback {
  rm::f32 pos_rad{0.0f};
  rm::f32 vel_rad_s{0.0f};
  rm::f32 torque_nm{0.0f};
};

/**
 * @brief 单腿两关节反馈（前/后）
 */
struct LegJointFeedback {
  JointFeedback front;
  JointFeedback back;
};

/**
 * @brief 左右轮角速度反馈
 */
struct WheelFeedback {
  rm::f32 left_rad_s{0.0f};
  rm::f32 right_rad_s{0.0f};
};

/**
 * @brief IMU 反馈（姿态 + 角速度 + 线加速度）
 */
struct ImuFeedback {
  rm::f32 roll_rad{0.0f};
  rm::f32 pitch_rad{0.0f};
  rm::f32 yaw_rad{0.0f};

  rm::f32 gyro_x_rad_s{0.0f};
  rm::f32 gyro_y_rad_s{0.0f};
  rm::f32 gyro_z_rad_s{0.0f};

  rm::f32 acc_x_mps2{0.0f};
  rm::f32 acc_y_mps2{0.0f};
  rm::f32 acc_z_mps2{0.0f};
};

/**
 * @brief 状态估计器统一输入
 * @note  仅保留传感器/反馈数据，不耦合电机对象
 */
struct ChassisStateEstimatorInput {
  LegJointFeedback left_leg;
  LegJointFeedback right_leg;
  WheelFeedback wheel;
  ImuFeedback imu;

  rm::f32 dt_s{0.002f};
  rm::f32 yaw_motor_rad{0.0f};
  rm::f32 s_ref_m{0.0f};
  bool use_external_s_ref{false};
  bool use_wheel_speed_direct{false};
};

/**
 * @brief 状态估计器配置
 * @note  几何参数、轮速换算参数和关节标定偏置统一收敛在此
 */
struct ChassisStateEstimatorConfig {
  rm::f32 leg_l1_m{0.215f};
  rm::f32 leg_l2_m{0.254f};

  rm::f32 wheel_radius_m{0.0575f};
  rm::f32 wheel_reduction_ratio{17.0f / 268.0f};
  rm::f32 max_valid_speed_mps{8.0f};

  rm::f32 left_phi1_offset_rad{
      PI - 1.6824f};
  rm::f32 left_phi4_offset_rad{-2.465204f };
  rm::f32 right_phi1_offset_rad{PI - 0.913293f };
  rm::f32 right_phi4_offset_rad{1.70642f };

  rm::f32 theta_dot_filter_cutoff_hz{8.0f};
};

/**
 * @brief 单腿运动学输入（已完成符号和零位标定）
 */
struct LegKinematicsJointInput {
  rm::f32 phi1_rad{0.0f};
  rm::f32 phi4_rad{0.0f};
  rm::f32 w_phi1_rad_s{0.0f};
  rm::f32 w_phi4_rad_s{0.0f};
};

/**
 * @brief 双腿运动学输入（已标定）
 */
struct CalibratedLegKinematicsInput {
  LegKinematicsJointInput left;
  LegKinematicsJointInput right;
};

/**
 * @brief 状态估计输出
 * @note  包含可直接喂给 WbrController 的 current 状态
 */
struct ChassisStateEstimatorOutput {
  wbr::CurrentState current{};

  rm::f32 wheel_speed_mps{0.0f};
  rm::f32 fused_speed_mps{0.0f};
  rm::f32 raw_wheel_speed_mps{0.0f};
  rm::f32 raw_accel_speed_mps{0.0f};
  rm::f32 current_speed_mps{0.0f};

  rm::f32 left_leg_length_m{0.0f};
  rm::f32 right_leg_length_m{0.0f};
  rm::f32 left_leg_angle_rad{0.0f};
  rm::f32 right_leg_angle_rad{0.0f};

  CalibratedLegKinematicsInput calibrated_leg_input{};
};

class ChassisStateEstimator {
 public:
  ChassisStateEstimator() = default;

  /**
   * @brief 初始化估计器
   * @param config 估计器配置
   */
  void Init(const ChassisStateEstimatorConfig &config);

  /**
   * @brief 重置估计输出状态
   */
  void Reset();

  /**
   * @brief 单次状态更新
   * @param input 传感器输入（含控制周期 dt_s）
   */
  void Update(const ChassisStateEstimatorInput &input);

  /**
   * @brief 获取完整估计输出
   */
  [[nodiscard]] const ChassisStateEstimatorOutput &GetOutput() const;

  /**
   * @brief 获取当前控制状态向量
   */
  [[nodiscard]] const wbr::CurrentState &GetCurrentState() const;

 private:
  [[nodiscard]] CalibratedLegKinematicsInput
  BuildCalibratedLegInput(const ChassisStateEstimatorInput &input) const;

  void UpdateLegState(const ChassisStateEstimatorInput &input, rm::f32 dt_s);
  void UpdateBodyState(const ChassisStateEstimatorInput &input);
  void UpdateSpeedState(const ChassisStateEstimatorInput &input, rm::f32 dt_s);

  [[nodiscard]] rm::f32 ConvertLeftPhi1(const JointFeedback &joint) const;
  [[nodiscard]] rm::f32 ConvertLeftPhi4(const JointFeedback &joint) const;
  [[nodiscard]] rm::f32 ConvertRightPhi1(const JointFeedback &joint) const;
  [[nodiscard]] rm::f32 ConvertRightPhi4(const JointFeedback &joint) const;

 private:
  ChassisStateEstimatorConfig config_{};
  ChassisStateEstimatorOutput output_{};
  SpeedEstimator speed_estimator_{};
  wbr::LegKinematics left_leg_{0.215f, 0.254f};
  wbr::LegKinematics right_leg_{0.215f, 0.254f};
  rm::modules::LowPassFilterConstDt<float> theta_ll_dot_filter_{};
  rm::modules::LowPassFilterConstDt<float> theta_lr_dot_filter_{};
};

#endif // BALANCE_CHASSIS_CHASSIS_STATE_HPP
