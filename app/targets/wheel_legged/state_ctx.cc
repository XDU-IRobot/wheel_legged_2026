/**
 * @file  targets/wheel_legged/state_ctx.cc
 * @brief 底盘跨周期状态、偏航跟随、定点锁定与速率斜坡实现
 */

#include "include/state_ctx.hpp"

#include <algorithm>
#include <cmath>

#include "include/params.hpp"

namespace wheel_legged::control_loop {

namespace {

constexpr float kPi = params::active::kPi;
constexpr float kYawFollowFixedTargetRad = params::active::control_loop::kYawFollowFixedTargetRad;
constexpr float kYawFollowSideOffsetRad = params::active::control_loop::kYawFollowSideOffsetRad;
constexpr float kGimbalStartupYawAlignErrorRad = params::active::control_loop::kGimbalStartupYawAlignErrorRad;
constexpr float kGimbalStartupYawAlignVelRadS = params::active::control_loop::kGimbalStartupYawAlignVelRadS;
constexpr float kYawFollowDriveReadyErrorRad = params::active::control_loop::kYawFollowDriveReadyErrorRad;
constexpr float kYawFollowDriveReadyVelRadS = params::active::control_loop::kYawFollowDriveReadyVelRadS;
constexpr SdotRampParams kSdotRampLowLeg = params::active::control_loop::kSdotRampLowLeg;
constexpr SdotRampParams kSdotRampMidLeg = params::active::control_loop::kSdotRampMidLeg;
constexpr SdotRampParams kSdotRampMidLegG = params::active::control_loop::kSdotRampMidLegG;
constexpr SdotRampParams kSdotRampHighLeg = params::active::control_loop::kSdotRampHighLeg;

}  // namespace

void ChassisStateContext::ResetOnModeChange(const float current_s, const float current_s_dot) {
  expected_s = current_s;
  filtered_s_dot = current_s_dot;
  filtered_yaw_dot = 0.0f;
  yaw_follow_pid.Clear();
  integrate_position = false;
  yaw_follow_target_initialized = false;
  landing_decel_active = false;
  landing_theta_bias = 0.0f;
  landing_stable_ticks = 0U;
  off_ground_duration_ticks = 0U;
}

void RampValueToTarget(const float target, float &value, const SdotRampParams &ramp_params) {
  // 方向翻转或减速时使用制动步长，加速时使用加速步长
  const bool direction_changed = (value * target) < 0.0f;
  const bool magnitude_reduced = std::fabs(target) < std::fabs(value);
  const float step = (direction_changed || magnitude_reduced) ? ramp_params.brake_step : ramp_params.accel_step;

  if (value < target) {
    value += step;
    if (value > target) value = target;
  } else if (value > target) {
    value -= step;
    if (value < target) value = target;
  }
}

void RampYawDotToTarget(const float target_yaw_dot, float &filtered_yaw_dot, const float ramp_step) {
  if (filtered_yaw_dot < target_yaw_dot) {
    filtered_yaw_dot += ramp_step;
    if (filtered_yaw_dot > target_yaw_dot) filtered_yaw_dot = target_yaw_dot;
  } else if (filtered_yaw_dot > target_yaw_dot) {
    filtered_yaw_dot -= ramp_step;
    if (filtered_yaw_dot < target_yaw_dot) filtered_yaw_dot = target_yaw_dot;
  }
}

YawFollowTargetSelection SelectNearestYawTarget(const float yaw_motor_rad, const float target_offset_rad) {
  // 两个候选目标角（相差 pi），选择离当前电机角最近的一个
  const float yaw_target_a_rad = rm::modules::Wrap(kYawFollowFixedTargetRad + target_offset_rad, -kPi, kPi);
  const float yaw_target_b_rad = rm::modules::Wrap(kYawFollowFixedTargetRad + kPi + target_offset_rad, -kPi, kPi);
  const float yaw_err_to_a = std::fabs(rm::modules::Wrap(yaw_target_a_rad - yaw_motor_rad, -kPi, kPi));
  const float yaw_err_to_b = std::fabs(rm::modules::Wrap(yaw_target_b_rad - yaw_motor_rad, -kPi, kPi));
  if (yaw_err_to_a <= yaw_err_to_b) {
    return {yaw_target_a_rad, 1.0f};
  }
  return {yaw_target_b_rad, -1.0f};
}

float SelectNearestYawCenterTarget(const float yaw_motor_rad) {
  return SelectNearestYawTarget(yaw_motor_rad, 0.0f).target_rad;
}

float YawFollowTargetOffset(const YawFollowAlignMode mode) {
  switch (mode) {
    case YawFollowAlignMode::kSidePositive:
      return kYawFollowSideOffsetRad;
    case YawFollowAlignMode::kSideNegative:
      return -kYawFollowSideOffsetRad;
    case YawFollowAlignMode::kForward:
    default:
      return 0.0f;
  }
}

float YawFollowDriveSign(const YawFollowAlignMode mode, const float target_drive_sign) {
  switch (mode) {
    case YawFollowAlignMode::kSideNegative:
      return -target_drive_sign;
    case YawFollowAlignMode::kForward:
    case YawFollowAlignMode::kSidePositive:
    default:
      return target_drive_sign;
  }
}

bool IsYawAtStartupTarget(const float yaw_target_rad, const float yaw_motor_rad, const float yaw_motor_vel_rad_s) {
  const float yaw_err_rad = std::fabs(rm::modules::Wrap(yaw_target_rad - yaw_motor_rad, -kPi, kPi));
  return yaw_err_rad <= kGimbalStartupYawAlignErrorRad &&
         std::fabs(yaw_motor_vel_rad_s) <= kGimbalStartupYawAlignVelRadS;
}

bool IsYawFollowDriveReady(const float yaw_target_rad, const float yaw_motor_rad, const float yaw_motor_vel_rad_s) {
  const float yaw_err_rad = std::fabs(rm::modules::Wrap(yaw_target_rad - yaw_motor_rad, -kPi, kPi));
  return yaw_err_rad <= kYawFollowDriveReadyErrorRad && std::fabs(yaw_motor_vel_rad_s) <= kYawFollowDriveReadyVelRadS;
}

SdotRampParams ResolveSdotRampParams(const chassis::Fsm::State mode) {
  switch (mode) {
    case chassis::Fsm::State::kLowLeg:
      return kSdotRampLowLeg;
    case chassis::Fsm::State::kHighLeg:
      return kSdotRampHighLeg;
    case chassis::Fsm::State::kMidLeg:
    case chassis::Fsm::State::kJumpPrep:
    case chassis::Fsm::State::kJumpPush:
    case chassis::Fsm::State::kJumpRecover:
    default:
      return kSdotRampMidLeg;
  }
}

SdotRampParams ResolveSdotRampParams(const chassis::Fsm::State mode, const bool mid_leg_g) {
  if (mode == chassis::Fsm::State::kMidLeg && mid_leg_g) {
    return kSdotRampMidLegG;
  }
  return ResolveSdotRampParams(mode);
}

}  // namespace wheel_legged::control_loop
