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

constexpr int16_t kWheelSpinThreshold = 220;
constexpr int16_t kWheelActionThreshold = 320;
constexpr int16_t kWheelCenterThreshold = 80;
constexpr float kControlLoopDtS = 0.002f;
constexpr int16_t kDr16AxisMaxAbs = 660;
constexpr float kTargetForwardSpeedMaxMps = 1.8f;
constexpr float kVxInputDeadbandNorm = 0.1f;
constexpr float kLockPointEnterSpeedThresholdMps = 0.30f;
constexpr float kLockPointExitSpeedThresholdMps = 0.55f;
constexpr float kLockPointEnterInputThreshold = 0.08f;
constexpr float kLockPointExitInputThreshold = 0.12f;
constexpr uint32_t kLockPointMinDwellTicks = 100U;  // 200ms @500Hz
constexpr float kLockPointAlphaRiseStep = 0.015f;
constexpr float kLockPointAlphaFallStep = 0.018f;
constexpr float kRcStickMax = 660.0f;
constexpr float kRcYawRateMaxRadS = -2.5f;
constexpr float kRcPitchRateMaxRadS = 1.5f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kPitchTargetMinRad = -0.35;
constexpr float kPitchTargetMaxRad = 0.25f;
constexpr float kYawFollowRampStepRadS = 0.05f;
constexpr float kYawFollowFixedTargetRad = -1.72f;
constexpr float kGimbalStartupYawAlignErrorRad = 0.04f;
constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;
constexpr uint32_t kGimbalStartupYawAlignStableTicks = 50U;  // 100ms @500Hz
chassis_runtime::Actuators g_actuators{};

struct SdotRampParams {
  float accel_step;
  float brake_step;
};

constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.01f};
constexpr SdotRampParams kSdotRampMidLeg{0.006f, 0.006f};
constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};

struct Dr16RawInput {
  bool online{false};
  rm::device::DR16::SwitchPosition switch_l{rm::device::DR16::SwitchPosition::kUnknown};
  rm::device::DR16::SwitchPosition switch_r{rm::device::DR16::SwitchPosition::kUnknown};
  int16_t right_y{0};
  int16_t left_x{0};
  int16_t left_y{0};
  int16_t dial{0};
  float gimbal_imu_yaw_rad{0.0f};
  float gimbal_imu_pitch_rad{0.0f};
};

float MapDr16RightYToForwardSpeed(const int16_t right_y) {
  const float normalized = static_cast<float>(right_y) / static_cast<float>(kDr16AxisMaxAbs);
  return rm::modules::Clamp(normalized * kTargetForwardSpeedMaxMps, -kTargetForwardSpeedMaxMps,
                            kTargetForwardSpeedMaxMps);
}

float NormalizeDr16Axis(const int16_t axis) {
  return rm::modules::Clamp(static_cast<float>(axis) / static_cast<float>(kDr16AxisMaxAbs), -1.0f, 1.0f);
}

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

void RampYawDotToTarget(const float target_yaw_dot, float &filtered_yaw_dot) {
  if (filtered_yaw_dot < target_yaw_dot) {
    filtered_yaw_dot += kYawFollowRampStepRadS;
    if (filtered_yaw_dot > target_yaw_dot) {
      filtered_yaw_dot = target_yaw_dot;
    }
  } else if (filtered_yaw_dot > target_yaw_dot) {
    filtered_yaw_dot -= kYawFollowRampStepRadS;
    if (filtered_yaw_dot < target_yaw_dot) {
      filtered_yaw_dot = target_yaw_dot;
    }
  }
}

void UpdateLockPointBlend(const bool target_lock, float &alpha_lock) {
  if (target_lock) {
    alpha_lock = rm::modules::Clamp(alpha_lock + kLockPointAlphaRiseStep, 0.0f, 1.0f);
  } else {
    alpha_lock = rm::modules::Clamp(alpha_lock - kLockPointAlphaFallStep, 0.0f, 1.0f);
  }
}

float SelectNearestYawCenterTarget(const float yaw_motor_rad) {
  const float yaw_target_a_rad = kYawFollowFixedTargetRad;
  const float yaw_target_b_rad = rm::modules::Wrap(kYawFollowFixedTargetRad + kPi, -kPi, kPi);
  const float yaw_err_to_a = std::fabs(rm::modules::Wrap(yaw_target_a_rad - yaw_motor_rad, -kPi, kPi));
  const float yaw_err_to_b = std::fabs(rm::modules::Wrap(yaw_target_b_rad - yaw_motor_rad, -kPi, kPi));
  return (yaw_err_to_a <= yaw_err_to_b) ? yaw_target_a_rad : yaw_target_b_rad;
}

bool IsYawAtStartupTarget(const float yaw_target_rad, const float yaw_motor_rad, const float yaw_motor_vel_rad_s) {
  const float yaw_err_rad = std::fabs(rm::modules::Wrap(yaw_target_rad - yaw_motor_rad, -kPi, kPi));
  return yaw_err_rad <= kGimbalStartupYawAlignErrorRad &&
         std::fabs(yaw_motor_vel_rad_s) <= kGimbalStartupYawAlignVelRadS;
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
  static rm::modules::PID yaw_follow_pid{5.5f, 0.0f, 0.9f, 5.f, 0.f};
  static bool yaw_follow_pid_initialized = false;
  static chassis::Fsm::State last_chassis_mode = chassis::Fsm::State::kDisabled;
  static bool gimbal_startup_align_complete = false;
  static bool gimbal_startup_align_was_active = false;
  static uint32_t gimbal_startup_align_stable_ticks = 0U;
  static float gimbal_startup_align_target_rad = kYawFollowFixedTargetRad;

  if (!yaw_follow_pid_initialized) {
    yaw_follow_pid.SetCircular(true).SetCircularCycle(2.0f * kPi);
    yaw_follow_pid_initialized = true;
  }

  UpdateRawFeedbackAndInputSnapshot(*globals, input, dr16_semantic_state);
  input.mode_request.current_leg_length_m = chassis_control_output.mean_leg_length_m;
  input.mode_request.tick_ms = now_ms;

  const chassis::Fsm::Input chassis_input = BuildChassisFsmInput(input, now_ms);
  const chassis::Fsm::Output chassis_output = globals->chassis_fsm.Update(chassis_input);

  const gimbal::Fsm::Input gimbal_input =
      BuildGimbalFsmInput(input, chassis_output, gimbal_startup_align_complete);
  const gimbal::Fsm::Output gimbal_output = globals->gimbal_fsm.Update(gimbal_input);
  const bool gimbal_startup_align_active = gimbal_output.mode == gimbal::Fsm::State::kStartupAlign;

  if (gimbal_startup_align_active && !gimbal_startup_align_was_active) {
    gimbal_startup_align_complete = false;
    gimbal_startup_align_stable_ticks = 0U;
    gimbal_startup_align_target_rad = SelectNearestYawCenterTarget(input.estimator_input.yaw_motor_rad);
  }

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

  if (!gimbal_output.control.gimbal_enable) {
    gimbal_startup_align_complete = false;
    gimbal_startup_align_was_active = false;
    gimbal_startup_align_stable_ticks = 0U;
  } else if (gimbal_startup_align_active) {
    const bool yaw_motor_ready = globals->yaw_motor.has_value();
    const float yaw_motor_vel_rad_s = yaw_motor_ready ? globals->yaw_motor->vel() : 0.0f;
    if (yaw_motor_ready &&
        IsYawAtStartupTarget(gimbal_startup_align_target_rad, input.estimator_input.yaw_motor_rad,
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

  const bool chassis_startup_ready =
      !gimbal_output.control.gimbal_enable || !gimbal_startup_align_active;
  const bool chassis_output_enable = chassis_output.control.enable_dm && chassis_startup_ready;

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
  if (mode_changed) {
    expected_s = current_state.s;
    filtered_s_dot = current_state.s_dot;
    filtered_yaw_dot = 0.0f;
    yaw_follow_pid.Clear();
    lock_point_target = false;
    lock_point_alpha = 0.0f;
    lock_point_s_ref = expected_s;
    lock_point_last_switch_tick = now_ms;
    last_chassis_mode = chassis_output.mode;
  }

  const float input_norm = input.dr16.online ? NormalizeDr16Axis(input.dr16.right_y) : 0.0f;
  float target_s_dot = 0.0f;
  if (std::fabs(input_norm) > kVxInputDeadbandNorm) {
    target_s_dot = input.dr16.online ? MapDr16RightYToForwardSpeed(input.dr16.right_y) : 0.0f;
  }
  if (!chassis_output_enable) {
    target_s_dot = 0.0f;
  }

  const bool lockpoint_enabled = chassis_output_enable && (chassis_output.mode != chassis::Fsm::State::kDisabled &&
                                                          chassis_output.mode != chassis::Fsm::State::kSpin);
  if (!lockpoint_enabled) {
    lock_point_target = false;
  } else {
    const float speed_abs = std::fabs(current_state.s_dot);
    const float input_abs = std::fabs(input_norm);
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
  chassis_update_input.expected.s_dot = (1.0f - lock_point_alpha) * filtered_s_dot;
  expected_s = lock_point_alpha * lock_point_s_ref + (1.0f - lock_point_alpha) * current_state.s;
  chassis_update_input.expected.s = expected_s;
  chassis_update_input.expected.phi = current_state.phi;
  chassis_update_input.expected.phi_dot = 0.0f;

  const bool yaw_follow_enabled = chassis_output.mode != chassis::Fsm::State::kDisabled && chassis_output_enable &&
                                  chassis_output.control.run_chassis_update && gimbal_output.control.gimbal_enable;
  if (!yaw_follow_enabled) {
    filtered_yaw_dot = 0.0f;
    yaw_follow_pid.Clear();
  } else {
    const float yaw_motor_rad = input.estimator_input.yaw_motor_rad;
    const float yaw_target_rad = SelectNearestYawCenterTarget(yaw_motor_rad);
    yaw_follow_pid.Update(yaw_target_rad, yaw_motor_rad, kControlLoopDtS);
    const float target_yaw_dot = -yaw_follow_pid.out();
    RampYawDotToTarget(target_yaw_dot, filtered_yaw_dot);
    chassis_update_input.expected.phi_dot = filtered_yaw_dot;
  }

  globals->chassis.Update(chassis_update_input);
  chassis_control_output = globals->chassis.GetOutput();
  g_actuators.ApplyChassisOutput(*globals, chassis_control_output, chassis_output_enable);

  if (globals->gimbal_imu_feedback_rx.has_value()) {
    wl_fm_gimbal_imu_pitch_rad = globals->gimbal_imu_feedback_rx->pitch_rad();
    wl_fm_gimbal_imu_yaw_rad = globals->gimbal_imu_feedback_rx->yaw_rad();
  } else {
    wl_fm_gimbal_imu_pitch_rad = 0.0f;
    wl_fm_gimbal_imu_yaw_rad = 0.0f;
  }

  UpdateDebugSnapshot(now_ms, input, chassis_output, gimbal_output, chassis_control_output, gimbal_control_output);
}

