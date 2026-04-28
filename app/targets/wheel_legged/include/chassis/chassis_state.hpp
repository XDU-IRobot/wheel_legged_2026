#pragma once

#include <cmath>
#include <cstring>

#include <librm.hpp>

#include "leg_kinematics.hpp"
#include "lqr_controllers.hpp"
#include "../utils/kalman_filter.hpp"

/**
 * @file  targets/wheel_legged/include/chassis/chassis_state.hpp
 * @brief 底盘状态估计模块：传感器输入建模、腿部运动学标定与速度融合
 */

namespace chassis {

/**
 * @brief 单关节反馈
 */
struct JointFeedback {
  rm::f32 pos_rad{0.0f};
  rm::f32 vel_rad_s{0.0f};
  rm::f32 torque_nm{0.0f};
};

/**
 * @brief 单腿双关节反馈（前/后）
 */
struct LegJointFeedback {
  JointFeedback front{};
  JointFeedback back{};
};

/**
 * @brief 左右轮反馈
 */
struct WheelFeedback {
  rm::f32 left_rad_s{0.0f};
  rm::f32 right_rad_s{0.0f};
};

/**
 * @brief 惯导反馈
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
 * @brief 状态估计器输入
 */
struct ChassisStateEstimatorInput {
  LegJointFeedback left_leg{};
  LegJointFeedback right_leg{};
  WheelFeedback wheel{};
  ImuFeedback imu{};

  rm::f32 dt_s{0.002f};
  rm::f32 yaw_motor_rad{0.0f};
  rm::f32 s_ref_m{0.0f};
  bool use_external_s_ref{false};
  bool use_wheel_speed_direct{false};
};

/**
 * @brief 状态估计器配置参数
 */
struct ChassisStateEstimatorConfig {
  rm::f32 leg_l1_m{0.215f};
  rm::f32 leg_l2_m{0.254f};

  rm::f32 wheel_radius_m{0.0575f};
  rm::f32 wheel_reduction_ratio{17.0f / 268.0f};
  rm::f32 max_valid_speed_mps{8.0f};

  rm::f32 left_phi1_offset_rad{3.14159265358979323846f - 1.6824f};
  rm::f32 left_phi4_offset_rad{-2.465204f};
  rm::f32 right_phi1_offset_rad{3.14159265358979323846f - 0.913293f};
  rm::f32 right_phi4_offset_rad{1.70642f};

  rm::f32 theta_dot_filter_cutoff_hz{8.0f};
};

/**
 * @brief 单腿运动学输入（已完成符号/零位标定）
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
  LegKinematicsJointInput left{};
  LegKinematicsJointInput right{};
};

/**
 * @brief 状态估计输出
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

/**
 * @brief 速度融合输入
 */
struct SpeedEstimatorInput {
  rm::f32 wheel_speed_mps{0.0f};
  rm::f32 imu_acc_x_mps2{0.0f};
  rm::f32 imu_acc_y_mps2{0.0f};
  rm::f32 imu_pitch_rad{0.0f};
  rm::f32 dt_s{0.002f};
};

/**
 * @brief 速度估计器（轮速+惯导融合）
 */
class SpeedEstimator {
 public:
  SpeedEstimator() : kf_(2, 0, 2) {}

  /** @brief 初始化滤波器参数与内部状态 */
  void Init() {
    accel_x_filter_.set_cutoff_frequency(500.0f, 10.0f);
    accel_y_filter_.set_cutoff_frequency(500.0f, 10.0f);

    kf_.UseAutoAdjustment = 0;
    std::memset(kf_.xhat_data, 0, sizeof(rm::f32) * 2);
    std::memset(kf_.xhatminus_data, 0, sizeof(rm::f32) * 2);
    std::memset(kf_.MeasuredVector, 0, sizeof(rm::f32) * 2);
    std::memset(kf_.FilteredValue, 0, sizeof(rm::f32) * 2);

    constexpr rm::f32 dt = 0.002f;
    const rm::f32 f_mat[4] = {1.0f, dt, 0.0f, 1.0f};
    const rm::f32 q_mat[4] = {0.0005f, 0.0f, 0.0f, 0.04f};
    const rm::f32 r_mat[4] = {0.5f, 0.0f, 0.0f, 2.0f};
    const rm::f32 p_mat[4] = {10.0f, 0.0f, 0.0f, 10.0f};
    const rm::f32 h_mat[4] = {1.0f, 0.0f, 0.0f, 1.0f};

    std::memcpy(kf_.F_data, f_mat, sizeof(f_mat));
    std::memcpy(kf_.Q_data, q_mat, sizeof(q_mat));
    std::memcpy(kf_.R_data, r_mat, sizeof(r_mat));
    std::memcpy(kf_.P_data, p_mat, sizeof(p_mat));
    std::memcpy(kf_.H_data, h_mat, sizeof(h_mat));

    kf_.StateMinVariance[0] = 1e-5f;
    kf_.StateMinVariance[1] = 1e-5f;

    Reset();
  }

  /** @brief 重置速度估计器状态 */
  void Reset() {
    wheel_speed_mps_ = 0.0f;
    accel_bias_ = 0.0f;
    accel_bias_sum_ = 0.0f;
    accel_bias_count_ = 0;
    accel_speed_raw_ = 0.0f;
    current_speed_mps_ = 0.0f;
    kf_.xhat_data[0] = 0.0f;
    kf_.xhat_data[1] = 0.0f;
  }

  /** @brief 单步更新速度估计 */
  void Update(const SpeedEstimatorInput &input) {
    const rm::f32 dt_s = (input.dt_s > 1e-5f) ? input.dt_s : 0.002f;

    wheel_speed_mps_ = input.wheel_speed_mps;

    const rm::f32 acc_x = accel_x_filter_.apply(input.imu_acc_x_mps2);
    const rm::f32 acc_y = accel_y_filter_.apply(input.imu_acc_y_mps2);
    rm::f32 accel_forward = acc_x - acc_y * std::sin(input.imu_pitch_rad);

    if (accel_bias_count_ < 1500) {
      accel_bias_sum_ += accel_forward;
      ++accel_bias_count_;
      accel_bias_ = accel_bias_sum_ / static_cast<rm::f32>(accel_bias_count_);
    }

    accel_forward -= accel_bias_;

    if (std::fabs(wheel_speed_mps_) < 0.02f && std::fabs(accel_forward) < 0.5f) {
      accel_forward = 0.0f;
    }

    accel_speed_raw_ += accel_forward * dt_s;
    if (std::fabs(wheel_speed_mps_) < 0.02f && std::fabs(accel_forward) < 0.2f) {
      accel_speed_raw_ = 0.0f;
    }

    kf_.MeasuredVector[0] = wheel_speed_mps_;
    kf_.MeasuredVector[1] = accel_forward;
    kf_.Update();
    current_speed_mps_ = kf_.FilteredValue[0];

    if (std::fabs(wheel_speed_mps_) < 0.02f && std::fabs(accel_forward) < 0.2f) {
      current_speed_mps_ = 0.0f;
      kf_.xhat_data[0] = 0.0f;
    }
  }

  [[nodiscard]] rm::f32 GetRawWheelSpeed() const { return wheel_speed_mps_; }
  [[nodiscard]] rm::f32 GetRawAccelSpeed() const { return accel_speed_raw_; }
  [[nodiscard]] rm::f32 GetCurrentSpeed() const { return current_speed_mps_; }

 private:
  rm::modules::LowPassFilterConstDt<rm::f32> accel_x_filter_{};
  rm::modules::LowPassFilterConstDt<rm::f32> accel_y_filter_{};
  rm::f32 wheel_speed_mps_{0.0f};
  rm::f32 accel_bias_{0.0f};
  rm::f32 accel_bias_sum_{0.0f};
  int accel_bias_count_{0};
  rm::f32 accel_speed_raw_{0.0f};
  rm::f32 current_speed_mps_{0.0f};
  KalmanFilter kf_;
};

/**
 * @brief 底盘状态估计器
 */
class ChassisStateEstimator {
 public:
  /** @brief 初始化估计器 */
  void Init(const ChassisStateEstimatorConfig &config) {
    config_ = config;
    left_leg_ = wbr::LegKinematics(config_.leg_l1_m, config_.leg_l2_m);
    right_leg_ = wbr::LegKinematics(config_.leg_l1_m, config_.leg_l2_m);
    theta_ll_dot_filter_.set_cutoff_frequency(500.0f, config_.theta_dot_filter_cutoff_hz);
    theta_lr_dot_filter_.set_cutoff_frequency(500.0f, config_.theta_dot_filter_cutoff_hz);
    speed_estimator_.Init();
    Reset();
  }

  /** @brief 重置估计器输出 */
  void Reset() { output_ = {}; }

  /** @brief 单步更新状态估计 */
  void Update(const ChassisStateEstimatorInput &input) {
    rm::f32 dt_s = input.dt_s;
    if (dt_s <= 0.0f) {
      dt_s = 0.002f;
    }

    UpdateBodyState(input);
    UpdateLegState(input, dt_s);
    UpdateSpeedState(input, dt_s);

    if (input.use_external_s_ref) {
      output_.current.s = input.s_ref_m;
    }
  }

  [[nodiscard]] const ChassisStateEstimatorOutput &GetOutput() const { return output_; }
  [[nodiscard]] const wbr::CurrentState &GetCurrentState() const { return output_.current; }

 private:
  /** @brief 构建已标定的腿部运动学输入 */
  [[nodiscard]] CalibratedLegKinematicsInput BuildCalibratedLegInput(const ChassisStateEstimatorInput &input) const {
    CalibratedLegKinematicsInput leg_input{};

    leg_input.left.phi1_rad = ConvertLeftPhi1(input.left_leg.front);
    leg_input.left.phi4_rad = ConvertLeftPhi4(input.left_leg.back);
    leg_input.left.w_phi1_rad_s = input.left_leg.front.vel_rad_s;
    leg_input.left.w_phi4_rad_s = input.left_leg.back.vel_rad_s;

    leg_input.right.phi1_rad = ConvertRightPhi1(input.right_leg.front);
    leg_input.right.phi4_rad = ConvertRightPhi4(input.right_leg.back);
    leg_input.right.w_phi1_rad_s = -input.right_leg.front.vel_rad_s;
    leg_input.right.w_phi4_rad_s = -input.right_leg.back.vel_rad_s;

    return leg_input;
  }

  /** @brief 更新腿部几何状态与摆角状态 */
  void UpdateLegState(const ChassisStateEstimatorInput &input, const rm::f32 dt_s) {
    static constexpr rm::f32 kPiHalf = 1.57079632679489661923f;

    const rm::f32 prev_theta_ll = output_.current.theta_ll;
    const rm::f32 prev_theta_lr = output_.current.theta_lr;

    const CalibratedLegKinematicsInput leg_input = BuildCalibratedLegInput(input);
    output_.calibrated_leg_input = leg_input;

    left_leg_.SetPhi1(leg_input.left.phi1_rad);
    left_leg_.SetPhi4(leg_input.left.phi4_rad);
    left_leg_.SetWPhi1(leg_input.left.w_phi1_rad_s);
    left_leg_.SetWPhi4(leg_input.left.w_phi4_rad_s);
    left_leg_.Update(dt_s);

    right_leg_.SetPhi1(leg_input.right.phi1_rad);
    right_leg_.SetPhi4(leg_input.right.phi4_rad);
    right_leg_.SetWPhi1(leg_input.right.w_phi1_rad_s);
    right_leg_.SetWPhi4(leg_input.right.w_phi4_rad_s);
    right_leg_.Update(dt_s);

    output_.left_leg_length_m = left_leg_.l0();
    output_.right_leg_length_m = right_leg_.l0();
    output_.current.l_l = output_.left_leg_length_m;
    output_.current.l_r = output_.right_leg_length_m;

    const rm::f32 theta_ll = -kPiHalf - (-output_.current.theta_b - left_leg_.phi0());
    const rm::f32 theta_lr = -kPiHalf - (-output_.current.theta_b - right_leg_.phi0());

    output_.left_leg_angle_rad = theta_ll;
    output_.right_leg_angle_rad = theta_lr;
    output_.current.theta_ll = theta_ll;
    output_.current.theta_lr = theta_lr;

    const rm::f32 raw_theta_ll_dot = (theta_ll - prev_theta_ll) / dt_s;
    const rm::f32 raw_theta_lr_dot = (theta_lr - prev_theta_lr) / dt_s;
    output_.current.theta_ll_dot = theta_ll_dot_filter_.apply(raw_theta_ll_dot);
    output_.current.theta_lr_dot = theta_lr_dot_filter_.apply(raw_theta_lr_dot);
  }

  /** @brief 更新机体俯仰/偏航状态 */
  void UpdateBodyState(const ChassisStateEstimatorInput &input) {
    output_.current.theta_b = input.imu.pitch_rad;
    output_.current.theta_b_dot = input.imu.gyro_y_rad_s;
    output_.current.phi = input.yaw_motor_rad;
    output_.current.phi_dot = input.imu.gyro_z_rad_s;
  }

  /** @brief 更新速度与位移状态 */
  void UpdateSpeedState(const ChassisStateEstimatorInput &input, const rm::f32 dt_s) {
    const rm::f32 left_wheel_vel = input.wheel.left_rad_s * config_.wheel_reduction_ratio * config_.wheel_radius_m;
    const rm::f32 right_wheel_vel = input.wheel.right_rad_s * config_.wheel_reduction_ratio * config_.wheel_radius_m;

    const rm::f32 left_speed = left_wheel_vel +
                               output_.current.l_l * output_.current.theta_ll_dot * std::cos(output_.current.theta_ll) +
                               left_leg_.l0_dot() * std::sin(output_.current.theta_ll);
    const rm::f32 right_speed =
        right_wheel_vel + output_.current.l_r * output_.current.theta_lr_dot * std::cos(output_.current.theta_lr) +
        right_leg_.l0_dot() * std::sin(output_.current.theta_lr);

    output_.wheel_speed_mps = 0.5f * (left_speed + right_speed);

    if (input.use_wheel_speed_direct) {
      output_.raw_wheel_speed_mps = output_.wheel_speed_mps;
      output_.raw_accel_speed_mps = 0.0f;
      output_.current_speed_mps = output_.wheel_speed_mps;
      output_.fused_speed_mps = output_.wheel_speed_mps;
      output_.current.s_dot = output_.wheel_speed_mps;
      output_.current.s += output_.current.s_dot * dt_s;
      return;
    }

    SpeedEstimatorInput speed_input{};
    speed_input.wheel_speed_mps = output_.wheel_speed_mps;
    speed_input.imu_acc_x_mps2 = input.imu.acc_x_mps2;
    speed_input.imu_acc_y_mps2 = input.imu.acc_y_mps2;
    speed_input.imu_pitch_rad = input.imu.pitch_rad;
    speed_input.dt_s = dt_s;
    speed_estimator_.Update(speed_input);

    output_.raw_wheel_speed_mps = speed_estimator_.GetRawWheelSpeed();
    output_.raw_accel_speed_mps = speed_estimator_.GetRawAccelSpeed();
    output_.current_speed_mps = speed_estimator_.GetCurrentSpeed();
    output_.fused_speed_mps = output_.current_speed_mps;

    if (std::fabs(output_.fused_speed_mps) > config_.max_valid_speed_mps) {
      output_.fused_speed_mps = output_.wheel_speed_mps;
    }

    output_.current.s_dot = output_.fused_speed_mps;
    output_.current.s += output_.current.s_dot * dt_s;
  }

  /** @brief 左腿前关节角度标定 */
  [[nodiscard]] rm::f32 ConvertLeftPhi1(const JointFeedback &joint) const {
    return joint.pos_rad + config_.left_phi1_offset_rad;
  }
  /** @brief 左腿后关节角度标定 */
  [[nodiscard]] rm::f32 ConvertLeftPhi4(const JointFeedback &joint) const {
    return joint.pos_rad + config_.left_phi4_offset_rad;
  }
  /** @brief 右腿前关节角度标定 */
  [[nodiscard]] rm::f32 ConvertRightPhi1(const JointFeedback &joint) const {
    return -joint.pos_rad + config_.right_phi1_offset_rad;
  }
  /** @brief 右腿后关节角度标定 */
  [[nodiscard]] rm::f32 ConvertRightPhi4(const JointFeedback &joint) const {
    return -joint.pos_rad + config_.right_phi4_offset_rad;
  }

 private:
  ChassisStateEstimatorConfig config_{};
  ChassisStateEstimatorOutput output_{};
  SpeedEstimator speed_estimator_{};
  wbr::LegKinematics left_leg_{0.215f, 0.254f};
  wbr::LegKinematics right_leg_{0.215f, 0.254f};
  rm::modules::LowPassFilterConstDt<rm::f32> theta_ll_dot_filter_{};
  rm::modules::LowPassFilterConstDt<rm::f32> theta_lr_dot_filter_{};
};

}  // namespace chassis
