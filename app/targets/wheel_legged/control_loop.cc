#include "include/globals.hpp"
#include "include/actuators.hpp"

#include "main.h"
#include <algorithm>
#include <cmath>

/**
 * @file  targets/wheel_legged/control_loop.cc
 * @brief 主控制循环：输入采集、状态机更新、底盘解算、执行器输出与调试同步
 */
extern "C" {
volatile uint32_t wl_fm_tick_ms{0};
volatile uint8_t wl_fm_chassis_mode{0};
volatile uint8_t wl_fm_gimbal_mode{0};
volatile uint8_t wl_fm_chassis_state_changed{0};
volatile uint8_t wl_fm_gimbal_state_changed{0};

volatile uint8_t wl_fm_dr16_online{0};
volatile int32_t wl_fm_dr16_switch_l{0};
volatile int32_t wl_fm_dr16_switch_r{0};
volatile int16_t wl_fm_dr16_dial{0};

volatile uint8_t wl_fm_dr16_enable_request{0};
volatile uint8_t wl_fm_dr16_spin_request{0};
volatile uint8_t wl_fm_dr16_jump_trigger_edge{0};

volatile float wl_fm_chassis_leg_length_m{0.0f};
volatile float wl_fm_left_leg_length_m{0.0f};
volatile float wl_fm_right_leg_length_m{0.0f};
volatile float wl_fm_chassis_speed_mps{0.0f};
volatile float wl_fm_left_support_force_n{0.0f};
volatile float wl_fm_right_support_force_n{0.0f};
volatile uint8_t wl_fm_off_ground_in_mid_high_leg{0};

volatile float wl_fm_motor_lf_pos_rad{0.0f};
volatile float wl_fm_motor_lf_vel_rad_s{0.0f};
volatile float wl_fm_motor_lf_tau_nm{0.0f};
volatile float wl_fm_motor_lb_pos_rad{0.0f};
volatile float wl_fm_motor_lb_vel_rad_s{0.0f};
volatile float wl_fm_motor_lb_tau_nm{0.0f};
volatile float wl_fm_motor_rf_pos_rad{0.0f};
volatile float wl_fm_motor_rf_vel_rad_s{0.0f};
volatile float wl_fm_motor_rf_tau_nm{0.0f};
volatile float wl_fm_motor_rb_pos_rad{0.0f};
volatile float wl_fm_motor_rb_vel_rad_s{0.0f};
volatile float wl_fm_motor_rb_tau_nm{0.0f};
volatile float wl_fm_wheel_left_rad_s{0.0f};
volatile float wl_fm_wheel_right_rad_s{0.0f};
volatile float wl_fm_wheel_left_tau_nm{0.0f};
volatile float wl_fm_wheel_right_tau_nm{0.0f};
volatile float wl_fm_motor_lf_out_tau_nm{0.0f};
volatile float wl_fm_motor_lb_out_tau_nm{0.0f};
volatile float wl_fm_motor_rf_out_tau_nm{0.0f};
volatile float wl_fm_motor_rb_out_tau_nm{0.0f};

volatile float wl_fm_imu_roll_rad{0.0f};
volatile float wl_fm_imu_pitch_rad{0.0f};
volatile float wl_fm_imu_yaw_rad{0.0f};
volatile float wl_fm_imu_gyro_x_rad_s{0.0f};
volatile float wl_fm_imu_gyro_y_rad_s{0.0f};
volatile float wl_fm_imu_gyro_z_rad_s{0.0f};
volatile float wl_fm_imu_acc_x_mps2{0.0f};
volatile float wl_fm_imu_acc_y_mps2{0.0f};
volatile float wl_fm_imu_acc_z_mps2{0.0f};
volatile float wl_fm_gimbal_imu_pitch_rad{0.0f};
volatile float wl_fm_gimbal_imu_yaw_rad{0.0f};

volatile float wl_fm_model_s_m{0.0f};
volatile float wl_fm_model_s_dot_mps{0.0f};
volatile float wl_fm_target_s_m{0.0f};
volatile float wl_fm_target_s_dot_mps{0.0f};
volatile float wl_fm_model_phi_rad{0.0f};
volatile float wl_fm_model_phi_dot_rad_s{0.0f};
volatile float wl_fm_model_theta_ll_rad{0.0f};
volatile float wl_fm_model_theta_ll_dot_rad_s{0.0f};
volatile float wl_fm_model_theta_lr_rad{0.0f};
volatile float wl_fm_model_theta_lr_dot_rad_s{0.0f};
volatile float wl_fm_model_theta_b_rad{0.0f};
volatile float wl_fm_model_theta_b_dot_rad_s{0.0f};
volatile float wl_fm_model_l_l_m{0.0f};
volatile float wl_fm_model_l_r_m{0.0f};
volatile float wl_fm_yaw_target_rad{0.0f};
volatile float wl_fm_yaw_motor_pos_rad{0.0f};
volatile float wl_fm_yaw_motor_vel_rad_s{0.0f};
volatile float wl_fm_pitch_target_rad{0.0f};
volatile float wl_fm_pitch_motor_pos_rad{0.0f};
volatile float wl_fm_pitch_motor_vel_rad_s{0.0f};
}

namespace {

constexpr int16_t kWheelSpinThreshold = wheel_legged::params::active::control_loop::kWheelSpinThreshold;
constexpr int16_t kWheelActionThreshold = wheel_legged::params::active::control_loop::kWheelActionThreshold;
constexpr int16_t kWheelCenterThreshold = wheel_legged::params::active::control_loop::kWheelCenterThreshold;
constexpr float kControlLoopDtS = wheel_legged::params::active::control_loop::kControlLoopDtS;
constexpr int16_t kDr16AxisMaxAbs = wheel_legged::params::active::control_loop::kDr16AxisMaxAbs;
constexpr float kTargetForwardSpeedMaxMps = wheel_legged::params::active::control_loop::kTargetForwardSpeedMaxMps;
constexpr float kVxInputDeadbandNorm = wheel_legged::params::active::control_loop::kVxInputDeadbandNorm;
constexpr float kVyInputDeadbandNorm = wheel_legged::params::active::control_loop::kVyInputDeadbandNorm;
constexpr float kLockPointEnterSpeedThresholdMps =
    wheel_legged::params::active::control_loop::kLockPointEnterSpeedThresholdMps;
constexpr float kLockPointExitSpeedThresholdMps =
    wheel_legged::params::active::control_loop::kLockPointExitSpeedThresholdMps;
constexpr float kLockPointEnterInputThreshold =
    wheel_legged::params::active::control_loop::kLockPointEnterInputThreshold;
constexpr float kLockPointExitInputThreshold = wheel_legged::params::active::control_loop::kLockPointExitInputThreshold;
constexpr uint32_t kLockPointMinDwellTicks = wheel_legged::params::active::control_loop::kLockPointMinDwellTicks;
constexpr float kLockPointAlphaRiseStep = wheel_legged::params::active::control_loop::kLockPointAlphaRiseStep;
constexpr float kLockPointAlphaFallStep = wheel_legged::params::active::control_loop::kLockPointAlphaFallStep;
constexpr float kRcStickMax = wheel_legged::params::active::control_loop::kRcStickMax;
constexpr float kRcYawRateMaxRadS = wheel_legged::params::active::control_loop::kRcYawRateMaxRadS;
constexpr float kRcPitchRateMaxRadS = wheel_legged::params::active::control_loop::kRcPitchRateMaxRadS;
constexpr float kPi = wheel_legged::params::active::kPi;
constexpr float kPitchTargetMinRad = wheel_legged::params::active::control_loop::kPitchTargetMinRad;
constexpr float kPitchTargetMaxRad = wheel_legged::params::active::control_loop::kPitchTargetMaxRad;
constexpr float kYawFollowRampStepRadS = wheel_legged::params::active::control_loop::kYawFollowRampStepRadS;
constexpr float kSpinYawRampStepRadS = wheel_legged::params::active::control_loop::kSpinYawRampStepRadS;
constexpr float kSpinYawTargetOffsetRad = wheel_legged::params::active::control_loop::kSpinYawTargetOffsetRad;
constexpr float kYawFollowFixedTargetRad = wheel_legged::params::active::control_loop::kYawFollowFixedTargetRad;
constexpr float kYawFollowSideOffsetRad = wheel_legged::params::active::control_loop::kYawFollowSideOffsetRad;
constexpr float kGimbalStartupYawAlignErrorRad =
    wheel_legged::params::active::control_loop::kGimbalStartupYawAlignErrorRad;
constexpr float kGimbalStartupYawAlignVelRadS =
    wheel_legged::params::active::control_loop::kGimbalStartupYawAlignVelRadS;
constexpr uint32_t kGimbalStartupYawAlignStableTicks =
    wheel_legged::params::active::control_loop::kGimbalStartupYawAlignStableTicks;
constexpr float kYawFollowDriveReadyErrorRad = wheel_legged::params::active::control_loop::kYawFollowDriveReadyErrorRad;
constexpr float kYawFollowDriveReadyVelRadS = wheel_legged::params::active::control_loop::kYawFollowDriveReadyVelRadS;
constexpr uint32_t kYawFollowDriveReadyStableTicks =
    wheel_legged::params::active::control_loop::kYawFollowDriveReadyStableTicks;
constexpr float kExpectedThetaLlBiasRad = wheel_legged::params::active::control_loop::kExpectedThetaLlBiasRad;
constexpr float kExpectedThetaLrBiasRad = wheel_legged::params::active::control_loop::kExpectedThetaLrBiasRad;
constexpr float kExpectedThetaBBiasRad = wheel_legged::params::active::control_loop::kExpectedThetaBBiasRad;
chassis_runtime::Actuators g_actuators{};

using SdotRampParams = wheel_legged::params::active::control_loop::SdotRampParams;
constexpr SdotRampParams kSdotRampLowLeg = wheel_legged::params::active::control_loop::kSdotRampLowLeg;
constexpr SdotRampParams kSdotRampMidLeg = wheel_legged::params::active::control_loop::kSdotRampMidLeg;
constexpr SdotRampParams kSdotRampHighLeg = wheel_legged::params::active::control_loop::kSdotRampHighLeg;

enum class YawFollowAlignMode : uint8_t {
  kForward = 0,
  kSidePositive,
  kSideNegative,
};

struct YawFollowTargetSelection {
  float target_rad{0.0f};
  float drive_sign{1.0f};
};

struct Dr16RawInput {
  bool online{false};
  rm::device::DR16::SwitchPosition switch_l{rm::device::DR16::SwitchPosition::kUnknown};
  rm::device::DR16::SwitchPosition switch_r{rm::device::DR16::SwitchPosition::kUnknown};
  int16_t right_y{0};
  int16_t right_x{0};
  int16_t left_x{0};
  int16_t left_y{0};
  int16_t dial{0};
  float gimbal_imu_yaw_rad{0.0f};
  float gimbal_imu_pitch_rad{0.0f};
};

/**
 * @brief 将 DR16 右摇杆映射为底盘前进速度目标
 */
float MapDr16RightYToForwardSpeed(const int16_t right_y) {
  const float normalized = static_cast<float>(right_y) / static_cast<float>(kDr16AxisMaxAbs);
  return rm::modules::Clamp(normalized * kTargetForwardSpeedMaxMps, -kTargetForwardSpeedMaxMps,
                            kTargetForwardSpeedMaxMps);
}

float MapDr16RightXToSideSpeed(const int16_t right_x) {
  const float normalized = static_cast<float>(right_x) / static_cast<float>(kDr16AxisMaxAbs);
  return rm::modules::Clamp(normalized * kTargetForwardSpeedMaxMps, -kTargetForwardSpeedMaxMps,
                            kTargetForwardSpeedMaxMps);
}

/**
 * @brief 将 DR16 通道归一化到 [-1, 1]
 */
float NormalizeDr16Axis(const int16_t axis) {
  return rm::modules::Clamp(static_cast<float>(axis) / static_cast<float>(kDr16AxisMaxAbs), -1.0f, 1.0f);
}

/**
 * @brief 按模式相关斜率逼近期望速度
 */
void RampValueToTarget(const float target, float &value, const SdotRampParams &ramp_params) {
  const bool direction_changed = (value * target) < 0.0f;
  const bool magnitude_reduced = std::fabs(target) < std::fabs(value);
  const float step = (direction_changed || magnitude_reduced) ? ramp_params.brake_step : ramp_params.accel_step;

  if (value < target) {
    value += step;
    if (value > target) {
      value = target;
    }
  } else if (value > target) {
    value -= step;
    if (value < target) {
      value = target;
    }
  }
}

/**
 * @brief 限制底盘偏航跟随角速度变化率
 */
void RampYawDotToTarget(const float target_yaw_dot, float &filtered_yaw_dot, const float ramp_step) {
  if (filtered_yaw_dot < target_yaw_dot) {
    filtered_yaw_dot += ramp_step;
    if (filtered_yaw_dot > target_yaw_dot) {
      filtered_yaw_dot = target_yaw_dot;
    }
  } else if (filtered_yaw_dot > target_yaw_dot) {
    filtered_yaw_dot -= ramp_step;
    if (filtered_yaw_dot < target_yaw_dot) {
      filtered_yaw_dot = target_yaw_dot;
    }
  }
}

/**
 * @brief 更新定点锁定混合权重
 */
void UpdateLockPointBlend(const bool target_lock, float &alpha_lock) {
  if (target_lock) {
    alpha_lock = rm::modules::Clamp(alpha_lock + kLockPointAlphaRiseStep, 0.0f, 1.0f);
  } else {
    alpha_lock = rm::modules::Clamp(alpha_lock - kLockPointAlphaFallStep, 0.0f, 1.0f);
  }
}

/**
 * @brief 选择距离当前偏航电机角最近的车头对齐目标
 */
YawFollowTargetSelection SelectNearestYawTarget(const float yaw_motor_rad, const float target_offset_rad) {
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

/**
 * @brief 判断启动归中是否达到角度和速度稳定条件
 */
bool IsYawAtStartupTarget(const float yaw_target_rad, const float yaw_motor_rad, const float yaw_motor_vel_rad_s) {
  const float yaw_err_rad = std::fabs(rm::modules::Wrap(yaw_target_rad - yaw_motor_rad, -kPi, kPi));
  return yaw_err_rad <= kGimbalStartupYawAlignErrorRad &&
         std::fabs(yaw_motor_vel_rad_s) <= kGimbalStartupYawAlignVelRadS;
}

bool IsYawFollowDriveReady(const float yaw_target_rad, const float yaw_motor_rad, const float yaw_motor_vel_rad_s) {
  const float yaw_err_rad = std::fabs(rm::modules::Wrap(yaw_target_rad - yaw_motor_rad, -kPi, kPi));
  return yaw_err_rad <= kYawFollowDriveReadyErrorRad && std::fabs(yaw_motor_vel_rad_s) <= kYawFollowDriveReadyVelRadS;
}

/**
 * @brief 根据底盘模式选择纵向速度斜坡参数
 */
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

/**
 * @brief 控制循环输入快照
 * @note  将多源输入收敛为单周期稳定视图。
 */
struct InputSnapshot {
  bool input_valid{false};

  Dr16RawInput dr16{};
  chassis::ChassisStateEstimatorInput estimator_input{};
  wheel_legged::ModeRequest mode_request{};
};

/**
 * @brief 调试状态快照
 */
struct DebugSnapshot {
  uint32_t tick_ms{0};
  chassis::Fsm::State chassis_mode{chassis::Fsm::State::kDisabled};
  gimbal::Fsm::State gimbal_mode{gimbal::Fsm::State::kDisabled};
  bool chassis_state_changed{false};
  bool gimbal_state_changed{false};
  bool dr16_enable_request{false};
  bool dr16_spin_request{false};
  bool dr16_jump_trigger_edge{false};
};

/**
 * @brief DR16 语义状态（用于边沿/锁存逻辑）
 */
struct Dr16SemanticState {
  bool wheel_action_armed{true};
  bool gimbal_target_initialized{false};
  wheel_legged::GimbalTarget rc_target{};
};

wheel_legged::LegProfile ResolveLegProfile(const rm::device::DR16::SwitchPosition switch_r) {
  switch (switch_r) {
    case rm::device::DR16::SwitchPosition::kMid:
      return wheel_legged::LegProfile::kMid;
    case rm::device::DR16::SwitchPosition::kUp:
      return wheel_legged::LegProfile::kHigh;
    case rm::device::DR16::SwitchPosition::kDown:
    default:
      return wheel_legged::LegProfile::kLow;
  }
}

wheel_legged::DomainRequest ResolveDomainRequest(const rm::device::DR16::SwitchPosition switch_l) {
  switch (switch_l) {
    case rm::device::DR16::SwitchPosition::kUp:
      return wheel_legged::DomainRequest::kCombat;
    case rm::device::DR16::SwitchPosition::kMid:
      return wheel_legged::DomainRequest::kService;
    case rm::device::DR16::SwitchPosition::kDown:
    default:
      return wheel_legged::DomainRequest::kDisabled;
  }
}

/**
 * @brief 将 DR16 原始输入转换为语义控制输入
 */
void ApplyDr16Semantics(const Dr16RawInput &dr16, Dr16SemanticState &semantic_state, InputSnapshot &input) {
  input.input_valid = dr16.online;
  input.dr16 = dr16;
  const float yaw_motor_pos_rad = input.estimator_input.yaw_motor_rad;

  wheel_legged::ModeRequest request{};
  request.input_valid = dr16.online;
  request.domain_request = ResolveDomainRequest(dr16.switch_l);
  request.service_profile = wheel_legged::ServiceProfile::kChassisAndGimbalWithFire;
  request.leg_request = ResolveLegProfile(dr16.switch_r);

  request.spin_hold = dr16.dial >= kWheelSpinThreshold;

  // 拨轮回中后重新布防，负向越过阈值时只产生一次跳跃触发边沿。
  if (std::abs(dr16.dial) <= kWheelCenterThreshold) {
    semantic_state.wheel_action_armed = true;
  }
  const bool wheel_action_trigger = semantic_state.wheel_action_armed && (dr16.dial <= -kWheelActionThreshold);
  if (wheel_action_trigger) {
    semantic_state.wheel_action_armed = false;
  }
  request.jump_trigger = wheel_action_trigger;

  request.fire_request = false;
  request.target_source = wheel_legged::TargetSource::kRc;
  // 遥控器云台目标采用积分形式；输入失效或关闭时跟随当前电机角，避免下次使能突跳。
  if (!dr16.online || request.domain_request == wheel_legged::DomainRequest::kDisabled) {
    semantic_state.rc_target.yaw_rad = yaw_motor_pos_rad;
    semantic_state.rc_target.pitch_rad = 0.0f;
    semantic_state.gimbal_target_initialized = false;
  } else {
    if (!semantic_state.gimbal_target_initialized) {
      semantic_state.rc_target.yaw_rad = yaw_motor_pos_rad;
      semantic_state.rc_target.pitch_rad = 0.0f;
      semantic_state.gimbal_target_initialized = true;
    }
    const float yaw_delta = static_cast<float>(dr16.left_x) / kRcStickMax * kRcYawRateMaxRadS * kControlLoopDtS;
    const float pitch_delta = static_cast<float>(dr16.left_y) / kRcStickMax * kRcPitchRateMaxRadS * kControlLoopDtS;
    semantic_state.rc_target.yaw_rad = rm::modules::Wrap(semantic_state.rc_target.yaw_rad + yaw_delta, -kPi, kPi);
    semantic_state.rc_target.pitch_rad =
        std::clamp(semantic_state.rc_target.pitch_rad + pitch_delta, kPitchTargetMinRad, kPitchTargetMaxRad);
  }
  request.rc_target = semantic_state.rc_target;
  request.host_target_valid = false;
  request.fall_detected = false;
  request.fall_detected_hold_ms = 0U;
  request.upright_stable = true;
  request.tick_ms = 0U;

  input.mode_request = request;
}

/**
 * @brief 更新传感器快照与 DR16 语义输入
 */
void UpdateRawFeedbackAndInputSnapshot(SharedResources &g, InputSnapshot &input, Dr16SemanticState &semantic_state) {
  g_actuators.FillEstimatorInput(g, input.estimator_input);

  Dr16RawInput dr16{
      .online = (g.dr16.online_status() == rm::device::Device::kOk),
      .switch_l = g.dr16.switch_l(),
      .switch_r = g.dr16.switch_r(),
      .right_y = g.dr16.right_y(),
      .right_x = g.dr16.right_x(),
      .left_x = g.dr16.left_x(),
      .left_y = g.dr16.left_y(),
      .dial = g.dr16.dial(),
      .gimbal_imu_yaw_rad = g.gimbal_imu_feedback_rx.has_value() ? g.gimbal_imu_feedback_rx->yaw_rad() : 0.0f,
      .gimbal_imu_pitch_rad = g.gimbal_imu_feedback_rx.has_value() ? g.gimbal_imu_feedback_rx->pitch_rad() : 0.0f,
  };
  ApplyDr16Semantics(dr16, semantic_state, input);
}

/**
 * @brief 构建底盘状态机输入
 */
chassis::Fsm::Input BuildChassisFsmInput(const InputSnapshot &input, const uint32_t tick_ms) {
  chassis::Fsm::Input fsm_input{};
  fsm_input.request = input.mode_request;
  fsm_input.request.tick_ms = tick_ms;
  return fsm_input;
}

/**
 * @brief 构建云台状态机输入
 */
gimbal::Fsm::Input BuildGimbalFsmInput(const InputSnapshot &input, const chassis::Fsm::Output &chassis_output,
                                       const bool startup_align_complete) {
  gimbal::Fsm::Input fsm_input{};
  fsm_input.request = input.mode_request;
  fsm_input.chassis_recovery_active = chassis_output.mode == chassis::Fsm::State::kRecoveryFallCheck ||
                                      chassis_output.mode == chassis::Fsm::State::kRecoverySelfRight;
  fsm_input.startup_align_complete = startup_align_complete;
  return fsm_input;
}

/**
 * @brief 将本周期关键状态写入调试导出变量
 */
void UpdateDebugSnapshot(const uint32_t tick_ms, const InputSnapshot &input, const chassis::Fsm::Output &chassis_output,
                         const gimbal::Fsm::Output &gimbal_output,
                         const chassis::Chassis::UpdateOutput &chassis_control_output,
                         const gimbal::Gimbal::UpdateOutput &gimbal_control_output) {
  DebugSnapshot debug{};
  debug.tick_ms = tick_ms;
  debug.chassis_mode = chassis_output.mode;
  debug.gimbal_mode = gimbal_output.mode;
  debug.chassis_state_changed = chassis_output.state_changed;
  debug.gimbal_state_changed = gimbal_output.state_changed;
  debug.dr16_enable_request =
      input.mode_request.input_valid && input.mode_request.domain_request != wheel_legged::DomainRequest::kDisabled;
  debug.dr16_spin_request = input.mode_request.spin_hold;
  debug.dr16_jump_trigger_edge = input.mode_request.jump_trigger;

  wl_fm_tick_ms = debug.tick_ms;
  wl_fm_chassis_mode = static_cast<uint8_t>(debug.chassis_mode);
  wl_fm_gimbal_mode = static_cast<uint8_t>(debug.gimbal_mode);
  wl_fm_chassis_state_changed = static_cast<uint8_t>(debug.chassis_state_changed);
  wl_fm_gimbal_state_changed = static_cast<uint8_t>(debug.gimbal_state_changed);

  wl_fm_dr16_online = static_cast<uint8_t>(input.dr16.online);
  wl_fm_dr16_switch_l = static_cast<int32_t>(input.dr16.switch_l);
  wl_fm_dr16_switch_r = static_cast<int32_t>(input.dr16.switch_r);
  wl_fm_dr16_dial = input.dr16.dial;

  wl_fm_dr16_enable_request = static_cast<uint8_t>(debug.dr16_enable_request);
  wl_fm_dr16_spin_request = static_cast<uint8_t>(debug.dr16_spin_request);
  wl_fm_dr16_jump_trigger_edge = static_cast<uint8_t>(debug.dr16_jump_trigger_edge);

  wl_fm_chassis_leg_length_m = chassis_control_output.mean_leg_length_m;
  wl_fm_chassis_speed_mps = chassis_control_output.speed_mps;
  wl_fm_left_support_force_n = chassis_control_output.left_support_force_n;
  wl_fm_right_support_force_n = chassis_control_output.right_support_force_n;
  wl_fm_off_ground_in_mid_high_leg = static_cast<uint8_t>(chassis_control_output.off_ground_in_mid_high_leg);

  const auto &motor = input.estimator_input;
  wl_fm_motor_lf_pos_rad = motor.left_leg.front.pos_rad;
  wl_fm_motor_lf_vel_rad_s = motor.left_leg.front.vel_rad_s;
  wl_fm_motor_lf_tau_nm = motor.left_leg.front.torque_nm;
  wl_fm_motor_lb_pos_rad = motor.left_leg.back.pos_rad;
  wl_fm_motor_lb_vel_rad_s = motor.left_leg.back.vel_rad_s;
  wl_fm_motor_lb_tau_nm = motor.left_leg.back.torque_nm;
  wl_fm_motor_rf_pos_rad = motor.right_leg.front.pos_rad;
  wl_fm_motor_rf_vel_rad_s = motor.right_leg.front.vel_rad_s;
  wl_fm_motor_rf_tau_nm = motor.right_leg.front.torque_nm;
  wl_fm_motor_rb_pos_rad = motor.right_leg.back.pos_rad;
  wl_fm_motor_rb_vel_rad_s = motor.right_leg.back.vel_rad_s;
  wl_fm_motor_rb_tau_nm = motor.right_leg.back.torque_nm;
  wl_fm_wheel_left_rad_s = motor.wheel.left_rad_s;
  wl_fm_wheel_right_rad_s = motor.wheel.right_rad_s;
  wl_fm_wheel_left_tau_nm = chassis_control_output.lw_tau;
  wl_fm_wheel_right_tau_nm = chassis_control_output.rw_tau;
  wl_fm_motor_lf_out_tau_nm = chassis_control_output.lf_tau;
  wl_fm_motor_lb_out_tau_nm = chassis_control_output.lb_tau;
  wl_fm_motor_rf_out_tau_nm = chassis_control_output.rf_tau;
  wl_fm_motor_rb_out_tau_nm = chassis_control_output.rb_tau;

  wl_fm_imu_roll_rad = motor.imu.roll_rad;
  wl_fm_imu_pitch_rad = motor.imu.pitch_rad;
  wl_fm_imu_yaw_rad = motor.imu.yaw_rad;
  wl_fm_imu_gyro_x_rad_s = motor.imu.gyro_x_rad_s;
  wl_fm_imu_gyro_y_rad_s = motor.imu.gyro_y_rad_s;
  wl_fm_imu_gyro_z_rad_s = motor.imu.gyro_z_rad_s;
  wl_fm_imu_acc_x_mps2 = motor.imu.acc_x_mps2;
  wl_fm_imu_acc_y_mps2 = motor.imu.acc_y_mps2;
  wl_fm_imu_acc_z_mps2 = motor.imu.acc_z_mps2;

  const auto &x = chassis_control_output.current_state;
  wl_fm_model_s_m = x.s;
  wl_fm_model_s_dot_mps = x.s_dot;
  wl_fm_model_phi_rad = x.phi;
  wl_fm_model_phi_dot_rad_s = x.phi_dot;
  wl_fm_model_theta_ll_rad = x.theta_ll;
  wl_fm_model_theta_ll_dot_rad_s = x.theta_ll_dot;
  wl_fm_model_theta_lr_rad = x.theta_lr;
  wl_fm_model_theta_lr_dot_rad_s = x.theta_lr_dot;
  wl_fm_model_theta_b_rad = x.theta_b;
  wl_fm_model_theta_b_dot_rad_s = x.theta_b_dot;
  wl_fm_model_l_l_m = x.l_l;
  wl_fm_model_l_r_m = x.l_r;
  wl_fm_left_leg_length_m = x.l_l;
  wl_fm_right_leg_length_m = x.l_r;
  wl_fm_yaw_target_rad = gimbal_control_output.yaw_target_rad;
  wl_fm_yaw_motor_pos_rad = gimbal_control_output.yaw_pos_rad;
  wl_fm_yaw_motor_vel_rad_s = gimbal_control_output.yaw_vel_rad_s;
  wl_fm_pitch_target_rad = gimbal_control_output.pitch_target_rad;
  wl_fm_pitch_motor_pos_rad = gimbal_control_output.pitch_pos_rad;
  wl_fm_pitch_motor_vel_rad_s = gimbal_control_output.pitch_vel_rad_s;
}

}  // namespace

/**
 * @brief 500Hz 主控制循环
 */
void ControlLoop() {
  if (globals == nullptr) {
    return;
  }

  const uint32_t now_ms = HAL_GetTick();

  static InputSnapshot input{};
  static Dr16SemanticState dr16_semantic_state{};
  static chassis::Chassis::UpdateOutput chassis_control_output{};
  static gimbal::Gimbal::UpdateOutput gimbal_control_output{};
  static float filtered_s_dot = 0.0f;
  static float filtered_yaw_dot = 0.0f;
  static float expected_s = 0.0f;
  static bool lock_point_target = false;
  static float lock_point_alpha = 0.0f;
  static float lock_point_s_ref = 0.0f;
  static uint32_t lock_point_last_switch_tick = 0U;
  static rm::modules::PID yaw_follow_pid{wheel_legged::params::active::control_loop::kYawFollowPid.kp,
                                         wheel_legged::params::active::control_loop::kYawFollowPid.ki,
                                         wheel_legged::params::active::control_loop::kYawFollowPid.kd,
                                         wheel_legged::params::active::control_loop::kYawFollowPid.max_out,
                                         wheel_legged::params::active::control_loop::kYawFollowPid.max_iout};
  static bool yaw_follow_pid_initialized = false;
  static chassis::Fsm::State last_chassis_mode = chassis::Fsm::State::kDisabled;
  static bool gimbal_startup_align_complete = false;
  static bool gimbal_startup_align_was_active = false;
  static uint32_t gimbal_startup_align_stable_ticks = 0U;
  static float gimbal_startup_align_target_rad = kYawFollowFixedTargetRad;
  static YawFollowAlignMode yaw_follow_align_mode = YawFollowAlignMode::kForward;
  static YawFollowTargetSelection yaw_follow_target{};
  static bool yaw_follow_target_initialized = false;
  static bool yaw_follow_drive_ready = false;
  static uint32_t yaw_follow_drive_ready_stable_ticks = 0U;

  if (!yaw_follow_pid_initialized) {
    yaw_follow_pid.SetCircular(true).SetCircularCycle(2.0f * kPi);
    yaw_follow_pid_initialized = true;
  }

  // 1. 采集硬件反馈并将 DR16 原始输入折叠为整车语义请求。
  UpdateRawFeedbackAndInputSnapshot(*globals, input, dr16_semantic_state);
  input.mode_request.current_leg_length_m = chassis_control_output.mean_leg_length_m;
  input.mode_request.theta_ll_rad = chassis_control_output.current_state.theta_ll;
  input.mode_request.theta_lr_rad = chassis_control_output.current_state.theta_lr;
  input.mode_request.tick_ms = now_ms;

  // 2. 状态机先消费同一份语义请求，输出本周期底盘/云台控制意图。
  const chassis::Fsm::Input chassis_input = BuildChassisFsmInput(input, now_ms);
  const chassis::Fsm::Output chassis_output = globals->chassis_fsm.Update(chassis_input);

  const gimbal::Fsm::Input gimbal_input = BuildGimbalFsmInput(input, chassis_output, gimbal_startup_align_complete);
  const gimbal::Fsm::Output gimbal_output = globals->gimbal_fsm.Update(gimbal_input);
  const bool gimbal_startup_align_active = gimbal_output.mode == gimbal::Fsm::State::kStartupAlign;

  // 3. 进入启动归中时锁定最近的车头方向，归中完成前底盘暂缓输出。
  if (gimbal_startup_align_active && !gimbal_startup_align_was_active) {
    gimbal_startup_align_complete = false;
    gimbal_startup_align_stable_ticks = 0U;
    gimbal_startup_align_target_rad = SelectNearestYawCenterTarget(input.estimator_input.yaw_motor_rad);
  }

  // 4. 组装云台控制输入，云台控制器直接写入 yaw/pitch DM MIT 命令。
  gimbal::Gimbal::UpdateInput gimbal_update_input{};
  gimbal_update_input.yaw_motor = globals->yaw_motor.has_value() ? &(*globals->yaw_motor) : nullptr;
  gimbal_update_input.pitch_motor = globals->pitch_motor.has_value() ? &(*globals->pitch_motor) : nullptr;
  gimbal_update_input.gimbal_enable = gimbal_output.control.gimbal_enable;
  gimbal_update_input.align_to_chassis_forward = gimbal_output.control.align_to_chassis_forward;
  gimbal_update_input.target = gimbal_output.control.gimbal_target;
  gimbal_update_input.use_yaw_motor_feedback = gimbal_startup_align_active;
  if (gimbal_startup_align_active) {
    gimbal_update_input.align_to_chassis_forward = false;
    gimbal_update_input.target.yaw_rad = gimbal_startup_align_target_rad;
  }
  gimbal_update_input.chassis_yaw_rad = input.estimator_input.imu.yaw_rad;
  gimbal_update_input.chassis_pitch_rad = input.estimator_input.imu.pitch_rad;
  gimbal_update_input.yaw_motor_rad = input.estimator_input.yaw_motor_rad;
  gimbal_update_input.gimbal_imu_yaw_rad = input.dr16.gimbal_imu_yaw_rad;
  gimbal_update_input.gimbal_imu_pitch_rad = input.dr16.gimbal_imu_pitch_rad;
  gimbal_update_input.dt_s = kControlLoopDtS;
  globals->gimbal.Update(gimbal_update_input);
  gimbal_control_output = globals->gimbal.GetOutput();

  // 5. 云台启动归中闭环判稳；完成后把遥控器积分目标对齐到当前云台惯导角。
  if (!gimbal_output.control.gimbal_enable) {
    gimbal_startup_align_complete = false;
    gimbal_startup_align_was_active = false;
    gimbal_startup_align_stable_ticks = 0U;
  } else if (gimbal_startup_align_active) {
    const bool yaw_motor_ready = globals->yaw_motor.has_value();
    const float yaw_motor_vel_rad_s = yaw_motor_ready ? globals->yaw_motor->vel() : 0.0f;
    if (yaw_motor_ready && IsYawAtStartupTarget(gimbal_startup_align_target_rad, input.estimator_input.yaw_motor_rad,
                                                yaw_motor_vel_rad_s)) {
      ++gimbal_startup_align_stable_ticks;
    } else {
      gimbal_startup_align_stable_ticks = 0U;
    }

    if (gimbal_startup_align_stable_ticks >= kGimbalStartupYawAlignStableTicks) {
      gimbal_startup_align_complete = true;
      if (input.mode_request.target_source == wheel_legged::TargetSource::kRc) {
        dr16_semantic_state.rc_target.yaw_rad = input.dr16.gimbal_imu_yaw_rad;
      }
    }
    gimbal_startup_align_was_active = true;
  } else {
    gimbal_startup_align_was_active = false;
    gimbal_startup_align_stable_ticks = 0U;
  }

  const bool chassis_startup_ready = !gimbal_output.control.gimbal_enable || !gimbal_startup_align_active;
  const bool chassis_output_enable = chassis_output.control.enable_dm && chassis_startup_ready;

  // 6. 将状态机输出和传感器快照转换为底盘控制器输入。
  chassis::Chassis::UpdateInput chassis_update_input{};
  chassis_update_input.fsm_mode = chassis_output.mode;
  chassis_update_input.enable_output = chassis_output_enable;
  chassis_update_input.run_chassis_update = chassis_output.control.run_chassis_update;
  chassis_update_input.spin_enable = chassis_output.control.spin_enable;
  chassis_update_input.target_leg_length_m = chassis_output.control.target_leg_length_m;
  chassis_update_input.estimator_input = input.estimator_input;
  chassis_update_input.estimator_input.dt_s = kControlLoopDtS;

  const auto &current_state = chassis_control_output.current_state;
  const bool mode_changed = (chassis_output.mode != last_chassis_mode);
  // 状态切换时重置速度斜坡、定点锁定和偏航跟随，避免继承上一模式的控制残留。
  if (mode_changed) {
    expected_s = current_state.s;
    filtered_s_dot = current_state.s_dot;
    filtered_yaw_dot = 0.0f;
    yaw_follow_pid.Clear();
    lock_point_target = false;
    yaw_follow_target_initialized = false;
    lock_point_alpha = 0.0f;
    lock_point_s_ref = expected_s;
    lock_point_last_switch_tick = now_ms;
    last_chassis_mode = chassis_output.mode;
  }

  const float forward_input_norm = input.dr16.online ? NormalizeDr16Axis(input.dr16.right_y) : 0.0f;
  const float side_input_norm = input.dr16.online ? NormalizeDr16Axis(input.dr16.right_x) : 0.0f;
  const bool forward_input_active = std::fabs(forward_input_norm) > kVxInputDeadbandNorm;
  const bool side_input_active = std::fabs(side_input_norm) > kVyInputDeadbandNorm;

  YawFollowAlignMode requested_yaw_follow_align_mode = yaw_follow_align_mode;
  if (!input.dr16.online || input.mode_request.domain_request == wheel_legged::DomainRequest::kDisabled) {
    requested_yaw_follow_align_mode = YawFollowAlignMode::kForward;
  } else if (forward_input_active) {
    requested_yaw_follow_align_mode = YawFollowAlignMode::kForward;
  } else if (side_input_active) {
    requested_yaw_follow_align_mode =
        (side_input_norm > 0.0f) ? YawFollowAlignMode::kSidePositive : YawFollowAlignMode::kSideNegative;
  }

  const bool yaw_follow_mode_changed = requested_yaw_follow_align_mode != yaw_follow_align_mode;
  if (!input.dr16.online || input.mode_request.domain_request == wheel_legged::DomainRequest::kDisabled) {
    yaw_follow_align_mode = YawFollowAlignMode::kForward;
    yaw_follow_target_initialized = false;
    yaw_follow_drive_ready = false;
    yaw_follow_drive_ready_stable_ticks = 0U;
  } else if (yaw_follow_mode_changed || !yaw_follow_target_initialized) {
    yaw_follow_align_mode = requested_yaw_follow_align_mode;
    yaw_follow_target =
        SelectNearestYawTarget(input.estimator_input.yaw_motor_rad, YawFollowTargetOffset(yaw_follow_align_mode));
    yaw_follow_target_initialized = true;
    yaw_follow_drive_ready = false;
    yaw_follow_drive_ready_stable_ticks = 0U;
    filtered_yaw_dot = 0.0f;
    yaw_follow_pid.Clear();
  }

  if (!yaw_follow_target_initialized) {
    yaw_follow_target =
        SelectNearestYawTarget(input.estimator_input.yaw_motor_rad, YawFollowTargetOffset(yaw_follow_align_mode));
    yaw_follow_target_initialized = true;
  }

  const float yaw_follow_drive_sign = YawFollowDriveSign(yaw_follow_align_mode, yaw_follow_target.drive_sign);
  const bool spin_control_enabled = chassis_output.mode == chassis::Fsm::State::kSpin && chassis_output_enable &&
                                    chassis_output.control.run_chassis_update;

  const bool yaw_follow_control_enabled = chassis_output.mode != chassis::Fsm::State::kDisabled &&
                                          chassis_output_enable && chassis_output.control.run_chassis_update &&
                                          gimbal_output.control.gimbal_enable;
  if (!spin_control_enabled && !yaw_follow_drive_ready) {
    const bool yaw_motor_ready = globals->yaw_motor.has_value();
    const float yaw_motor_vel_rad_s = yaw_motor_ready ? globals->yaw_motor->vel() : 0.0f;
    if (yaw_follow_control_enabled && yaw_motor_ready &&
        IsYawFollowDriveReady(yaw_follow_target.target_rad, input.estimator_input.yaw_motor_rad, yaw_motor_vel_rad_s)) {
      ++yaw_follow_drive_ready_stable_ticks;
    } else {
      yaw_follow_drive_ready_stable_ticks = 0U;
    }
    if (yaw_follow_drive_ready_stable_ticks >= kYawFollowDriveReadyStableTicks) {
      yaw_follow_drive_ready = true;
    }
  } else if (spin_control_enabled) {
    yaw_follow_drive_ready = false;
    yaw_follow_drive_ready_stable_ticks = 0U;
  }

  float target_s_dot = 0.0f;
  if (spin_control_enabled) {
    target_s_dot = 0.0f;
  } else if (!yaw_follow_drive_ready) {
    target_s_dot = 0.0f;
  } else if (forward_input_active) {
    target_s_dot = input.dr16.online ? yaw_follow_drive_sign * MapDr16RightYToForwardSpeed(input.dr16.right_y) : 0.0f;
  } else if (side_input_active) {
    target_s_dot = input.dr16.online ? yaw_follow_drive_sign * MapDr16RightXToSideSpeed(input.dr16.right_x) : 0.0f;
  }
  if (!chassis_output_enable) {
    target_s_dot = 0.0f;
  }

  // 低速且无输入时进入定点锁定，将期望位移混合到锁定点以抑制慢漂。
  const bool lockpoint_enabled = chassis_output_enable && (chassis_output.mode != chassis::Fsm::State::kDisabled &&
                                                           chassis_output.mode != chassis::Fsm::State::kSpin);
  if (!lockpoint_enabled) {
    lock_point_target = false;
  } else {
    const float speed_abs = std::fabs(current_state.s_dot);
    const float input_abs = std::max(std::fabs(forward_input_norm), std::fabs(side_input_norm));
    const bool request_lock =
        (speed_abs < kLockPointEnterSpeedThresholdMps) && (input_abs < kLockPointEnterInputThreshold);
    const bool request_unlock =
        (speed_abs > kLockPointExitSpeedThresholdMps) || (input_abs > kLockPointExitInputThreshold);
    const uint32_t elapsed = now_ms - lock_point_last_switch_tick;

    if (!lock_point_target && request_lock && elapsed >= kLockPointMinDwellTicks) {
      lock_point_target = true;
      lock_point_s_ref = current_state.s;
      lock_point_last_switch_tick = now_ms;
    } else if (lock_point_target && request_unlock && elapsed >= kLockPointMinDwellTicks) {
      lock_point_target = false;
      lock_point_last_switch_tick = now_ms;
    }
  }

  UpdateLockPointBlend(lock_point_target, lock_point_alpha);
  if (lock_point_alpha < 0.02f) {
    lock_point_s_ref = current_state.s;
  }

  const SdotRampParams ramp_params = ResolveSdotRampParams(chassis_output.mode);
  RampValueToTarget(target_s_dot, filtered_s_dot, ramp_params);
  // LQR 期望量：纵向速度来自斜坡，纵向位置在定点锁定时逐步切到锁定参考。
  chassis_update_input.expected.s_dot = (1.0f - lock_point_alpha) * filtered_s_dot;
  expected_s = lock_point_alpha * lock_point_s_ref + (1.0f - lock_point_alpha) * current_state.s;
  chassis_update_input.expected.s = expected_s;
  wl_fm_target_s_dot_mps = chassis_update_input.expected.s_dot;
  wl_fm_target_s_m = chassis_update_input.expected.s;
  chassis_update_input.expected.phi = current_state.phi;
  chassis_update_input.expected.phi_dot = 0.0f;
  chassis_update_input.expected.theta_ll =
      (chassis_output.control.theta_leg_target_rad != 0.0f) ? chassis_output.control.theta_leg_target_rad
                                                             : kExpectedThetaLlBiasRad;
  chassis_update_input.expected.theta_lr =
      (chassis_output.control.theta_leg_target_rad != 0.0f) ? chassis_output.control.theta_leg_target_rad
                                                             : kExpectedThetaLrBiasRad;
  chassis_update_input.expected.theta_b = kExpectedThetaBBiasRad;

  const bool yaw_follow_enabled = yaw_follow_control_enabled && !spin_control_enabled;
  // 底盘偏航期望跟随云台偏航电机，保持车体尽量对准云台中心方向。
  if (spin_control_enabled) {
    const float yaw_motor_rad = input.estimator_input.yaw_motor_rad;
    const float yaw_target_rad = rm::modules::Wrap(yaw_motor_rad + kSpinYawTargetOffsetRad, -kPi, kPi);
    yaw_follow_pid.Update(yaw_target_rad, yaw_motor_rad, kControlLoopDtS);
    const float target_yaw_dot = -yaw_follow_pid.out();
    RampYawDotToTarget(target_yaw_dot, filtered_yaw_dot, kSpinYawRampStepRadS);
    chassis_update_input.expected.phi_dot = filtered_yaw_dot;
  } else if (!yaw_follow_enabled) {
    filtered_yaw_dot = 0.0f;
    yaw_follow_pid.Clear();
  } else {
    const float yaw_motor_rad = input.estimator_input.yaw_motor_rad;
    const float yaw_target_rad = yaw_follow_target.target_rad;
    yaw_follow_pid.Update(yaw_target_rad, yaw_motor_rad, kControlLoopDtS);
    const float target_yaw_dot = -yaw_follow_pid.out();
    RampYawDotToTarget(target_yaw_dot, filtered_yaw_dot, kYawFollowRampStepRadS);
    chassis_update_input.expected.phi_dot = filtered_yaw_dot;
  }

  // 7. 底盘控制器完成状态估计、LQR/补偿解算，执行器适配层负责实际电机下发。
  globals->chassis.Update(chassis_update_input);
  chassis_control_output = globals->chassis.GetOutput();
  g_actuators.ApplyChassisOutput(*globals, chassis_control_output, chassis_output_enable);

  // 8. 导出关键内部量，供上位机/调试器观察本周期数据流。
  if (globals->gimbal_imu_feedback_rx.has_value()) {
    wl_fm_gimbal_imu_pitch_rad = globals->gimbal_imu_feedback_rx->pitch_rad();
    wl_fm_gimbal_imu_yaw_rad = globals->gimbal_imu_feedback_rx->yaw_rad();
  } else {
    wl_fm_gimbal_imu_pitch_rad = 0.0f;
    wl_fm_gimbal_imu_yaw_rad = 0.0f;
  }

  UpdateDebugSnapshot(now_ms, input, chassis_output, gimbal_output, chassis_control_output, gimbal_control_output);
}
