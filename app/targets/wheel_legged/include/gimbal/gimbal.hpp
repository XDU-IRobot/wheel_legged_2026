#pragma once

#include <algorithm>
#include <cmath>

#include "librm.hpp"

#include "common/controllers/gimbal_2dof.hpp"
#include "../fsm_common.hpp"
#include "../wheel_legged_params.hpp"

/**
 * @file  targets/wheel_legged/include/gimbal/gimbal.hpp
 * @brief 云台双轴控制器与 DM 电机命令输出
 */

namespace gimbal {

/**
 * @brief 云台控制主类
 */
class Gimbal {
 public:
  using DmMitMotor = rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>;

  /**
   * @brief 单次云台控制更新输入
   */
  struct UpdateInput {
    DmMitMotor *yaw_motor{nullptr};        ///< 偏航 DM 电机对象
    DmMitMotor *pitch_motor{nullptr};      ///< 俯仰 DM 电机对象
    bool gimbal_enable{false};             ///< 是否使能云台输出
    bool align_to_chassis_forward{false};  ///< 是否对齐车体前方
    bool use_yaw_motor_feedback{false};    ///< 是否用偏航电机编码器作为偏航反馈
    wheel_legged::GimbalTarget target{};   ///< 云台角度目标
    float chassis_yaw_rad{0.0f};           ///< 车体偏航角
    float chassis_pitch_rad{0.0f};         ///< 车体俯仰角，用于俯仰重力补偿
    float yaw_motor_rad{0.0f};             ///< 偏航电机编码器角度
    float gimbal_imu_yaw_rad{0.0f};        ///< 云台惯导偏航角
    float gimbal_imu_pitch_rad{0.0f};      ///< 云台惯导俯仰角
    float dt_s{wheel_legged::params::active::gimbal::kDefaultDtS};  ///< 控制周期
  };

  /**
   * @brief 单次云台控制更新输出
   */
  struct UpdateOutput {
    bool gimbal_enabled{false};  ///< 本周期云台是否使能

    float yaw_target_rad{0.0f};     ///< 偏航目标角
    float yaw_pos_rad{0.0f};        ///< 偏航反馈角
    float yaw_vel_rad_s{0.0f};      ///< 偏航反馈角速度
    float yaw_cmd_torque_nm{0.0f};  ///< 偏航输出力矩

    float pitch_target_rad{0.0f};     ///< 俯仰目标角
    float pitch_pos_rad{0.0f};        ///< 俯仰反馈角
    float pitch_vel_rad_s{0.0f};      ///< 俯仰反馈角速度
    float pitch_cmd_torque_nm{0.0f};  ///< 俯仰输出力矩
  };

  /**
   * @brief 初始化 PID 与电机使能锁存状态
   */
  void Init() {
    motors_enabled_latched_ = false;
    last_use_yaw_motor_feedback_ = false;
    ConfigurePid();
    ClearPid();
    output_ = {};
  }

  /**
   * @brief 执行一次云台控制更新并下发 DM MIT 命令
   */
  void Update(const UpdateInput &input) {
    output_ = {};

    if (input.yaw_motor == nullptr || input.pitch_motor == nullptr) {
      motors_enabled_latched_ = false;
      last_use_yaw_motor_feedback_ = false;
      ClearPid();
      return;
    }

    output_.yaw_pos_rad = input.use_yaw_motor_feedback ? input.yaw_motor_rad : input.gimbal_imu_yaw_rad;
    output_.yaw_vel_rad_s = input.yaw_motor->vel();
    output_.pitch_pos_rad = -input.gimbal_imu_pitch_rad;
    output_.pitch_vel_rad_s = input.pitch_motor->vel();

    const float desired_yaw = input.align_to_chassis_forward ? input.chassis_yaw_rad : input.target.yaw_rad;
    output_.yaw_target_rad = desired_yaw;
    output_.pitch_target_rad = std::clamp(input.target.pitch_rad, wheel_legged::params::active::gimbal::kPitchMinRad,
                                          wheel_legged::params::active::gimbal::kPitchMaxRad);
    output_.gimbal_enabled = input.gimbal_enable;

    if (!input.gimbal_enable) {
      controller_.Enable(false);
      // 失能时仍发送零力矩 MIT 命令，保持 CAN 设备刷新。
      input.yaw_motor->SetMitCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      input.pitch_motor->SetMitCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      DisableMotorsIfNeeded(input);
      ClearPid();
      last_use_yaw_motor_feedback_ = false;
      return;
    }

    EnableMotorsIfNeeded(input);
    if (input.use_yaw_motor_feedback != last_use_yaw_motor_feedback_) {
      ClearPid();
    }
    last_use_yaw_motor_feedback_ = input.use_yaw_motor_feedback;

    const float dt_s = (input.dt_s > 1e-5f) ? input.dt_s : wheel_legged::params::active::gimbal::kDefaultDtS;
    controller_.Enable(true);
    controller_.SetTarget(output_.yaw_target_rad, output_.pitch_target_rad);
    controller_.Update(output_.yaw_pos_rad, output_.yaw_vel_rad_s, output_.pitch_pos_rad, output_.pitch_vel_rad_s,
                       dt_s);

    output_.yaw_cmd_torque_nm = std::clamp(controller_.output().yaw,
                                           -wheel_legged::params::active::gimbal::kDmTorqueLimitNm,
                                           wheel_legged::params::active::gimbal::kDmTorqueLimitNm);
    const float pitch_gravity_ff =
        wheel_legged::params::active::gimbal::kPitchGravityCompensationNm * std::cos(input.chassis_pitch_rad);
    output_.pitch_cmd_torque_nm =
        std::clamp(controller_.output().pitch + pitch_gravity_ff,
                   -wheel_legged::params::active::gimbal::kDmTorqueLimitNm,
                   wheel_legged::params::active::gimbal::kDmTorqueLimitNm);

    // 仅使用 MIT 力矩通道，位置/速度前馈由外层控制器显式置零。
    input.yaw_motor->SetMitCommand(0.0f, 0.0f, output_.yaw_cmd_torque_nm, 0.0f, 0.0f);
    input.pitch_motor->SetMitCommand(0.0f, 0.0f, output_.pitch_cmd_torque_nm, 0.0f, 0.0f);
    // input.yaw_motor->SetMitCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    // input.pitch_motor->SetMitCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  }

  /** @brief 获取最近一次云台控制输出 */
  [[nodiscard]] const UpdateOutput &GetOutput() const { return output_; }

 private:
  /** @brief 配置双环 PID 参数 */
  void ConfigurePid() {
    const auto &yaw_pos = wheel_legged::params::active::gimbal::kYawPositionPid;
    const auto &yaw_spd = wheel_legged::params::active::gimbal::kYawSpeedPid;
    const auto &pitch_pos = wheel_legged::params::active::gimbal::kPitchPositionPid;
    const auto &pitch_spd = wheel_legged::params::active::gimbal::kPitchSpeedPid;
    controller_.pid().yaw_position.SetKp(yaw_pos.kp).SetKi(yaw_pos.ki).SetKd(yaw_pos.kd).SetMaxOut(yaw_pos.max_out).SetMaxIout(yaw_pos.max_iout);
    controller_.pid().yaw_speed.SetKp(yaw_spd.kp).SetKi(yaw_spd.ki).SetKd(yaw_spd.kd).SetMaxOut(yaw_spd.max_out).SetMaxIout(yaw_spd.max_iout);
    controller_.pid().pitch_position.SetKp(pitch_pos.kp).SetKi(pitch_pos.ki).SetKd(pitch_pos.kd).SetMaxOut(pitch_pos.max_out).SetMaxIout(pitch_pos.max_iout);
    controller_.pid().pitch_speed.SetKp(pitch_spd.kp).SetKi(pitch_spd.ki).SetKd(pitch_spd.kd).SetMaxOut(pitch_spd.max_out).SetMaxIout(pitch_spd.max_iout);
  }

  /** @brief 清空双环 PID 积分与历史状态 */
  void ClearPid() {
    controller_.pid().yaw_position.Clear();
    controller_.pid().yaw_speed.Clear();
    controller_.pid().pitch_position.Clear();
    controller_.pid().pitch_speed.Clear();
  }

  /** @brief 首次使能云台 DM 电机 */
  void EnableMotorsIfNeeded(const UpdateInput &input) {
    if (motors_enabled_latched_) {
      return;
    }

    input.yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
    input.yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    input.pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
    input.pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);

    motors_enabled_latched_ = true;
    ClearPid();
  }

  /** @brief 失能云台 DM 电机 */
  void DisableMotorsIfNeeded(const UpdateInput &input) {
    if (!motors_enabled_latched_) {
      return;
    }

    input.yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    input.pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    motors_enabled_latched_ = false;
  }

  bool motors_enabled_latched_{false};
  bool last_use_yaw_motor_feedback_{false};

  Gimbal2Dof controller_{};

  UpdateOutput output_{};
};

}  // namespace gimbal
