#pragma once

#include <algorithm>
#include <cmath>

#include "librm.hpp"

#include "common/controllers/gimbal_2dof.hpp"
#include "../fsm_common.hpp"

namespace gimbal {

class Gimbal {
 public:
  using DmMitMotor = rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>;
  using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

  // DM MIT motor settings for wheel-legged gimbal motors.
  static inline const DmMitSettings kPitchMotorSettings{0x12, 0x11, 3.141593f, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
  static inline const DmMitSettings kYawMotorSettings{0x13, 0x03, 3.141593f, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};

  struct UpdateInput {
    DmMitMotor *yaw_motor{nullptr};
    DmMitMotor *pitch_motor{nullptr};
    bool gimbal_enable{false};
    bool align_to_chassis_forward{false};
    wheel_legged::GimbalTarget target{};
    float chassis_yaw_rad{0.0f};
    float chassis_pitch_rad{0.0f};
    float gimbal_imu_yaw_rad{0.0f};
    float gimbal_imu_pitch_rad{0.0f};
    float dt_s{kDefaultDtS};
  };

  struct UpdateOutput {
    bool gimbal_enabled{false};

    float yaw_target_rad{0.0f};
    float yaw_pos_rad{0.0f};
    float yaw_vel_rad_s{0.0f};
    float yaw_cmd_torque_nm{0.0f};

    float pitch_target_rad{0.0f};
    float pitch_pos_rad{0.0f};
    float pitch_vel_rad_s{0.0f};
    float pitch_cmd_torque_nm{0.0f};
  };

  void Init() {
    motors_enabled_latched_ = false;
    ConfigurePid();
    ClearPid();
    output_ = {};
  }

  void Update(const UpdateInput &input) {
    output_ = {};

    if (input.yaw_motor == nullptr || input.pitch_motor == nullptr) {
      motors_enabled_latched_ = false;
      ClearPid();
      return;
    }

    output_.yaw_pos_rad = input.gimbal_imu_yaw_rad;
    output_.yaw_vel_rad_s = input.yaw_motor->vel();
    output_.pitch_pos_rad = -input.gimbal_imu_pitch_rad;
    output_.pitch_vel_rad_s = input.pitch_motor->vel();

    const float desired_yaw = input.align_to_chassis_forward ? input.chassis_yaw_rad : input.target.yaw_rad;
    output_.yaw_target_rad = desired_yaw;
    output_.pitch_target_rad = std::clamp(input.target.pitch_rad, kPitchMinRad, kPitchMaxRad);
    output_.gimbal_enabled = input.gimbal_enable;

    if (!input.gimbal_enable) {
      controller_.Enable(false);
      // Keep CAN traffic active in disabled mode while commanding zero torque.
      input.yaw_motor->SetMitCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      input.pitch_motor->SetMitCommand(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      DisableMotorsIfNeeded(input);
      ClearPid();
      return;
    }

    EnableMotorsIfNeeded(input);

    const float dt_s = (input.dt_s > 1e-5f) ? input.dt_s : kDefaultDtS;
    controller_.Enable(true);
    controller_.SetTarget(output_.yaw_target_rad, output_.pitch_target_rad);
    controller_.Update(output_.yaw_pos_rad, output_.yaw_vel_rad_s, output_.pitch_pos_rad, output_.pitch_vel_rad_s,
                       dt_s);

    output_.yaw_cmd_torque_nm = std::clamp(controller_.output().yaw, -kDmTorqueLimitNm, kDmTorqueLimitNm);
    const float pitch_gravity_ff = kPitchGravityCompensationNm * std::cos(input.chassis_pitch_rad);
    output_.pitch_cmd_torque_nm =
        std::clamp(controller_.output().pitch + pitch_gravity_ff, -kDmTorqueLimitNm, kDmTorqueLimitNm);

    // Test mode: keep output torque at zero for now.
    input.yaw_motor->SetMitCommand(0.0f, 0.0f, output_.yaw_cmd_torque_nm, 0.0f, 0.0f);
    input.pitch_motor->SetMitCommand(0.0f, 0.0f, output_.pitch_cmd_torque_nm, 0.0f, 0.0f);
  }

  [[nodiscard]] const UpdateOutput &GetOutput() const { return output_; }

 private:
  static constexpr float kPi = 3.14159265358979323846f;
  static constexpr float kDefaultDtS = 0.002f;
  static constexpr float kPitchMinRad = -0.2f;
  static constexpr float kPitchMaxRad = 0.25f;
  static constexpr float kDmTorqueLimitNm = 10.0f;
  static constexpr float kPitchGravityCompensationNm = 1.3f;

  void ConfigurePid() {
    controller_.pid().yaw_position.SetKp(13.0f).SetKi(0.0f).SetKd(0.05f).SetMaxOut(10.0f).SetMaxIout(0.4f);
    controller_.pid().yaw_speed.SetKp(0.85f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(8.0f).SetMaxIout(0.0f);

    controller_.pid().pitch_position.SetKp(13.0f).SetKi(0.f).SetKd(0.05f).SetMaxOut(10.0f).SetMaxIout(0.4f);
    controller_.pid().pitch_speed.SetKp(0.85f).SetKi(0.0f).SetKd(0.0f).SetMaxOut(8.0f).SetMaxIout(0.0f);
  }

  void ClearPid() {
    controller_.pid().yaw_position.Clear();
    controller_.pid().yaw_speed.Clear();
    controller_.pid().pitch_position.Clear();
    controller_.pid().pitch_speed.Clear();
  }

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

  void DisableMotorsIfNeeded(const UpdateInput &input) {
    if (!motors_enabled_latched_) {
      return;
    }

    input.yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    input.pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    motors_enabled_latched_ = false;
  }

  bool motors_enabled_latched_{false};

  Gimbal2Dof controller_{};

  UpdateOutput output_{};
};

}  // namespace gimbal
