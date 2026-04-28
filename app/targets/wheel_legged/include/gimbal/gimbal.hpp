#pragma once

#include <algorithm>

#include "librm.hpp"

#include "../fsm_common.hpp"

namespace gimbal {

class Gimbal {
 public:
  using YawMotor = rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>;

  struct UpdateInput {
    YawMotor *yaw_motor{nullptr};
    bool gimbal_enable{false};
    bool align_to_chassis_forward{false};
    wheel_legged::GimbalTarget target{};
    float chassis_yaw_rad{0.0f};
  };

  struct UpdateOutput {
    bool yaw_enabled{false};
    float yaw_target_rad{0.0f};
    float yaw_pos_rad{0.0f};
    float yaw_vel_rad_s{0.0f};
    float yaw_cmd_torque_nm{0.0f};
  };

  void Init() {
    yaw_enabled_latched_ = false;
    output_ = {};
  }

  void Update(const UpdateInput &input) {
    output_ = {};

    if (input.yaw_motor == nullptr) {
      yaw_enabled_latched_ = false;
      return;
    }

    output_.yaw_pos_rad = input.yaw_motor->pos();
    output_.yaw_vel_rad_s = input.yaw_motor->vel();

    const float desired_yaw = input.align_to_chassis_forward ? input.chassis_yaw_rad : input.target.yaw_rad;
    output_.yaw_target_rad = std::clamp(desired_yaw, -kYawLimitRad, kYawLimitRad);
    output_.yaw_enabled = input.gimbal_enable;

    if (!input.gimbal_enable) {
      if (yaw_enabled_latched_) {
        input.yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
      }
      yaw_enabled_latched_ = false;
      output_.yaw_cmd_torque_nm = 0.0f;
      return;
    }

    if (!yaw_enabled_latched_) {
      input.yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
      input.yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
      yaw_enabled_latched_ = true;
    }

    input.yaw_motor->SetMitCommand(0.f, 0.0f, 0.0f, 0.f, 0.f);
    output_.yaw_cmd_torque_nm = 0.0f;
  }

  [[nodiscard]] const UpdateOutput &GetOutput() const { return output_; }

 private:
  static constexpr float kYawLimitRad = 3.0f;
  static constexpr float kYawKp = 30.0f;
  static constexpr float kYawKd = 1.0f;

  bool yaw_enabled_latched_{false};
  UpdateOutput output_{};
};

}  // namespace gimbal
