#pragma once

#include <cmath>
#include <cstring>

#include <librm.hpp>

#include "../params.hpp"
#include "leg.hpp"
#include "lqr.hpp"
#include "../utils/kalman.hpp"

/**
 * @file  targets/wheel_legged/include/chassis/state.hpp
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
  LegJointFeedback left_leg{};   ///< 左腿关节反馈
  LegJointFeedback right_leg{};  ///< 右腿关节反馈
  WheelFeedback wheel{};         ///< 左右轮反馈
  ImuFeedback imu{};             ///< 底盘惯导反馈

  rm::f32 dt_s{0.002f};               ///< 估计周期
  rm::f32 yaw_motor_rad{0.0f};        ///< 云台偏航电机角度，供底盘跟随使用
  rm::f32 pitch_motor_rad{0.0f};      ///< 云台俯仰电机角度，供调试使用
  rm::f32 s_ref_m{0.0f};              ///< 外部位移参考
  bool use_external_s_ref{false};     ///< 是否用外部位移参考覆盖积分位移
  bool use_wheel_speed_direct{true};  ///< 是否跳过速度融合并直接使用轮速
};

/**
 * @brief 状态估计器配置参数
 */
struct ChassisStateEstimatorConfig {
  rm::f32 leg_l1_m{wheel_legged::params::active::state_estimator::kLegL1M};  ///< 五连杆主动杆长度
  rm::f32 leg_l2_m{wheel_legged::params::active::state_estimator::kLegL2M};  ///< 五连杆从动杆长度

  rm::f32 wheel_radius_m{wheel_legged::params::active::state_estimator::kWheelRadiusM};  ///< 轮半径
  rm::f32 wheel_reduction_ratio{
      wheel_legged::params::active::state_estimator::kWheelReductionRatio};  ///< 轮电机到车轮的速度换算比例
  rm::f32 max_valid_speed_mps{wheel_legged::params::active::state_estimator::kMaxValidSpeedMps};  ///< 融合速度可信上限

  rm::f32 left_phi1_offset_rad{
      wheel_legged::params::active::state_estimator::kLeftPhi1OffsetRad};  ///< 左腿前关节零位偏移
  rm::f32 left_phi4_offset_rad{
      wheel_legged::params::active::state_estimator::kLeftPhi4OffsetRad};  ///< 左腿后关节零位偏移
  rm::f32 right_phi1_offset_rad{
      wheel_legged::params::active::state_estimator::kRightPhi1OffsetRad};  ///< 右腿前关节零位偏移
  rm::f32 right_phi4_offset_rad{
      wheel_legged::params::active::state_estimator::kRightPhi4OffsetRad};  ///< 右腿后关节零位偏移

  rm::f32 theta_dot_filter_cutoff_hz{
      wheel_legged::params::active::state_estimator::kThetaDotFilterCutoffHz};  ///< 腿摆角速度低通截止频率
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
  wbr::CurrentState current{};  ///< LQR 使用的当前状态向量

  rm::f32 wheel_speed_mps{0.0f};           ///< 由轮速和腿部运动学解算的车速
  rm::f32 filtered_wheel_speed_mps{0.0f};  ///< 低通滤波后的轮速
  rm::f32 fused_speed_mps{0.0f};           ///< 轮速/惯导融合后的车速
  rm::f32 raw_wheel_speed_mps{0.0f};       ///< 速度滤波器中的原始轮速
  rm::f32 raw_accel_speed_mps{0.0f};       ///< 加速度积分得到的原始速度
  rm::f32 current_speed_mps{0.0f};         ///< 速度滤波器当前输出

  rm::f32 left_leg_length_m{0.0f};    ///< 左腿长
  rm::f32 right_leg_length_m{0.0f};   ///< 右腿长
  rm::f32 left_leg_angle_rad{0.0f};   ///< 左腿摆角
  rm::f32 right_leg_angle_rad{0.0f};  ///< 右腿摆角

  CalibratedLegKinematicsInput calibrated_leg_input{};  ///< 供控制器复用的已标定腿部输入
};

/**
 * @brief 速度融合输入
 */
struct SpeedEstimatorInput {
  rm::f32 wheel_speed_mps{0.0f};  ///< 轮系速度观测
  rm::f32 imu_acc_x_mps2{0.0f};   ///< 惯导 x 轴加速度
  rm::f32 imu_pitch_rad{0.0f};    ///< 机体俯仰角，用于重力/姿态补偿
  rm::f32 dt_s{wheel_legged::params::active::state_estimator::kDefaultDtS};  ///< 估计周期
};

/**
 * @brief 速度估计器（轮速+惯导融合）
 */
class SpeedEstimator {
 public:
  SpeedEstimator() : kf_(2, 0, 2) {}

  /** @brief 初始化滤波器参数与内部状态 */
  void Init() {
    accel_x_filter_.set_cutoff_frequency(wheel_legged::params::active::state_estimator::kImuAccelFilterSampleHz,
                                         wheel_legged::params::active::state_estimator::kImuAccelFilterCutoffHz);

    kf_.UseAutoAdjustment = 0;
    std::memset(kf_.xhat_data, 0, sizeof(rm::f32) * 2);
    std::memset(kf_.xhatminus_data, 0, sizeof(rm::f32) * 2);
    std::memset(kf_.MeasuredVector, 0, sizeof(rm::f32) * 2);
    std::memset(kf_.FilteredValue, 0, sizeof(rm::f32) * 2);

    const rm::f32 f_mat[4] = {wheel_legged::params::active::state_estimator::kKalmanF[0],
                              wheel_legged::params::active::state_estimator::kKalmanF[1],
                              wheel_legged::params::active::state_estimator::kKalmanF[2],
                              wheel_legged::params::active::state_estimator::kKalmanF[3]};
    const rm::f32 q_mat[4] = {wheel_legged::params::active::state_estimator::kKalmanQ[0],
                              wheel_legged::params::active::state_estimator::kKalmanQ[1],
                              wheel_legged::params::active::state_estimator::kKalmanQ[2],
                              wheel_legged::params::active::state_estimator::kKalmanQ[3]};
    const rm::f32 r_mat[4] = {wheel_legged::params::active::state_estimator::kKalmanR[0],
                              wheel_legged::params::active::state_estimator::kKalmanR[1],
                              wheel_legged::params::active::state_estimator::kKalmanR[2],
                              wheel_legged::params::active::state_estimator::kKalmanR[3]};
    const rm::f32 p_mat[4] = {wheel_legged::params::active::state_estimator::kKalmanP[0],
                              wheel_legged::params::active::state_estimator::kKalmanP[1],
                              wheel_legged::params::active::state_estimator::kKalmanP[2],
                              wheel_legged::params::active::state_estimator::kKalmanP[3]};
    const rm::f32 h_mat[4] = {wheel_legged::params::active::state_estimator::kKalmanH[0],
                              wheel_legged::params::active::state_estimator::kKalmanH[1],
                              wheel_legged::params::active::state_estimator::kKalmanH[2],
                              wheel_legged::params::active::state_estimator::kKalmanH[3]};

    std::memcpy(kf_.F_data, f_mat, sizeof(f_mat));
    std::memcpy(kf_.Q_data, q_mat, sizeof(q_mat));
    std::memcpy(kf_.R_data, r_mat, sizeof(r_mat));
    std::memcpy(kf_.P_data, p_mat, sizeof(p_mat));
    std::memcpy(kf_.H_data, h_mat, sizeof(h_mat));

    kf_.StateMinVariance[0] = wheel_legged::params::active::state_estimator::kKalmanMinVariance;
    kf_.StateMinVariance[1] = wheel_legged::params::active::state_estimator::kKalmanMinVariance;

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
    const rm::f32 dt_s = (input.dt_s > wheel_legged::params::active::state_estimator::kKalmanMinVariance)
                             ? input.dt_s
                             : wheel_legged::params::active::state_estimator::kDefaultDtS;

    wheel_speed_mps_ = input.wheel_speed_mps;

    constexpr rm::f32 kGravityMps2 = 9.8f;
    const rm::f32 acc_x = accel_x_filter_.apply(input.imu_acc_x_mps2);
    rm::f32 accel_forward = acc_x - kGravityMps2 * std::sin(input.imu_pitch_rad);

    if (accel_bias_count_ < static_cast<int>(wheel_legged::params::active::state_estimator::kAccelBiasInitSamples)) {
      accel_bias_sum_ += accel_forward;
      ++accel_bias_count_;
      // accel_bias_ = accel_bias_sum_ / static_cast<rm::f32>(accel_bias_count_);
      accel_bias_ = 0.f;
    }

    accel_forward -= accel_bias_;

    if (std::fabs(wheel_speed_mps_) < wheel_legged::params::active::state_estimator::kAccelZeroWheelSpeedThresholdMps &&
        std::fabs(accel_forward) < wheel_legged::params::active::state_estimator::kAccelZeroHighThresholdMps2) {
      accel_forward = 0.0f;
    }

    accel_speed_raw_ += accel_forward * dt_s;
    if (std::fabs(wheel_speed_mps_) < wheel_legged::params::active::state_estimator::kAccelZeroWheelSpeedThresholdMps &&
        std::fabs(accel_forward) < wheel_legged::params::active::state_estimator::kAccelZeroLowThresholdMps2) {
      accel_speed_raw_ = 0.0f;
    }

    kf_.MeasuredVector[0] = wheel_speed_mps_;
    kf_.MeasuredVector[1] = accel_forward;
    kf_.Update();
    current_speed_mps_ = kf_.FilteredValue[0];

    if (std::fabs(wheel_speed_mps_) < wheel_legged::params::active::state_estimator::kAccelZeroWheelSpeedThresholdMps &&
        std::fabs(accel_forward) < wheel_legged::params::active::state_estimator::kAccelZeroLowThresholdMps2) {
      current_speed_mps_ = 0.0f;
      kf_.xhat_data[0] = 0.0f;
    }
  }

  [[nodiscard]] rm::f32 GetRawWheelSpeed() const { return wheel_speed_mps_; }
  [[nodiscard]] rm::f32 GetRawAccelSpeed() const { return accel_speed_raw_; }
  [[nodiscard]] rm::f32 GetCurrentSpeed() const { return current_speed_mps_; }

 private:
  rm::modules::LowPassFilterConstDt<rm::f32> accel_x_filter_{};
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
    left_leg_.SetL1(config_.leg_l1_m);
    left_leg_.SetL2(config_.leg_l2_m);
    right_leg_.SetL1(config_.leg_l1_m);
    right_leg_.SetL2(config_.leg_l2_m);
    theta_ll_dot_filter_.set_cutoff_frequency(wheel_legged::params::active::state_estimator::kImuAccelFilterSampleHz,
                                              config_.theta_dot_filter_cutoff_hz);
    theta_lr_dot_filter_.set_cutoff_frequency(wheel_legged::params::active::state_estimator::kImuAccelFilterSampleHz,
                                              config_.theta_dot_filter_cutoff_hz);
    wheel_speed_filter_.set_cutoff_frequency(wheel_legged::params::active::state_estimator::kWheelSpeedFilterSampleHz,
                                             wheel_legged::params::active::state_estimator::kWheelSpeedFilterCutoffHz);
    speed_estimator_.Init();
    Reset();
  }

  /** @brief 重置估计器输出 */
  void Reset() { output_ = {}; }

  /** @brief 单步更新状态估计 */
  void Update(const ChassisStateEstimatorInput &input) {
    rm::f32 dt_s = input.dt_s;
    if (dt_s <= 0.0f) {
      dt_s = wheel_legged::params::active::state_estimator::kDefaultDtS;
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
    static constexpr rm::f32 kPiHalf = wheel_legged::params::active::state_estimator::kThetaPiHalf;

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

    const rm::f32 raw_theta_ll_dot = input.imu.gyro_y_rad_s + left_leg_.phi0_dot();
    const rm::f32 raw_theta_lr_dot = input.imu.gyro_y_rad_s + right_leg_.phi0_dot();
    output_.current.theta_ll_dot = theta_ll_dot_filter_.apply(raw_theta_ll_dot);
    output_.current.theta_lr_dot = theta_lr_dot_filter_.apply(raw_theta_lr_dot);
  }

  /** @brief 更新机体俯仰/偏航状态 */
  void UpdateBodyState(const ChassisStateEstimatorInput &input) {
    output_.current.theta_b = input.imu.pitch_rad;
    output_.current.theta_b_dot = input.imu.gyro_y_rad_s;
    output_.current.phi = input.imu.yaw_rad;
    output_.current.phi_dot = input.imu.gyro_z_rad_s;
  }

  /** @brief 更新速度与位移状态 */
  void UpdateSpeedState(const ChassisStateEstimatorInput &input, const rm::f32 dt_s) {
    const rm::f32 left_wheel_vel = input.wheel.left_rad_s * config_.wheel_reduction_ratio * config_.wheel_radius_m;
    const rm::f32 right_wheel_vel = input.wheel.right_rad_s * config_.wheel_reduction_ratio * config_.wheel_radius_m;

    // const rm::f32 left_speed = left_wheel_vel +
    //                            output_.current.l_l * output_.current.theta_ll_dot *
    //                            std::cos(output_.current.theta_ll) + left_leg_.l0_dot() *
    //                            std::sin(output_.current.theta_ll);
    // const rm::f32 right_speed =
    //     right_wheel_vel + output_.current.l_r * output_.current.theta_lr_dot * std::cos(output_.current.theta_lr) +
    //     right_leg_.l0_dot() * std::sin(output_.current.theta_lr);
    const rm::f32 left_speed = left_wheel_vel;
    const rm::f32 right_speed = right_wheel_vel;

    output_.wheel_speed_mps = 0.5f * (left_speed + right_speed);
    output_.filtered_wheel_speed_mps = wheel_speed_filter_.apply(output_.wheel_speed_mps);

    if (input.use_wheel_speed_direct) {
      output_.raw_wheel_speed_mps = output_.filtered_wheel_speed_mps;
      output_.raw_accel_speed_mps = 0.0f;
      output_.current_speed_mps = output_.wheel_speed_mps;
      output_.fused_speed_mps = output_.wheel_speed_mps;
      output_.current.s_dot = output_.wheel_speed_mps;
      output_.current.s += output_.current.s_dot * dt_s;
      return;
    }

    SpeedEstimatorInput speed_input{};
    speed_input.wheel_speed_mps = output_.filtered_wheel_speed_mps;
    speed_input.imu_acc_x_mps2 = input.imu.acc_x_mps2;
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
    output_.current.s += output_.filtered_wheel_speed_mps * dt_s;
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
  wbr::LegKinematics left_leg_{wheel_legged::params::active::state_estimator::kLegL1M,
                               wheel_legged::params::active::state_estimator::kLegL2M};
  wbr::LegKinematics right_leg_{wheel_legged::params::active::state_estimator::kLegL1M,
                                wheel_legged::params::active::state_estimator::kLegL2M};
  rm::modules::LowPassFilterConstDt<rm::f32> theta_ll_dot_filter_{};
  rm::modules::LowPassFilterConstDt<rm::f32> theta_lr_dot_filter_{};
  rm::modules::LowPassFilterConstDt<rm::f32> wheel_speed_filter_{};
};

}  // namespace chassis
