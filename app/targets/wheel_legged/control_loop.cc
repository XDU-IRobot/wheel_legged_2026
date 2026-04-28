#include "include/globals.hpp"
#include "include/actuators.hpp"

#include "main.h"
#include <cstddef>
#include <cmath>

/**
 * @file  targets/wheel_legged/control_loop.cc
 * @brief 主控制循环：输入采集、状态机更新、底盘解算、执行器输出与调试同步
 */

extern "C" {
volatile uint32_t wl_fm_tick_ms{0};
volatile float wl_fm_can_loop_freq_hz{0.0f};
volatile float wl_fm_joint_can_fps{0.0f};
volatile float wl_fm_wheel_can_fps{0.0f};
volatile float wl_fm_timer_period_us{0.0f};
volatile uint8_t wl_fm_chassis_mode{0};
volatile uint8_t wl_fm_gimbal_mode{0};
volatile uint8_t wl_fm_chassis_state_changed{0};
volatile uint8_t wl_fm_gimbal_state_changed{0};
volatile char wl_fm_chassis_mode_text[32]{"Disabled"};
volatile char wl_fm_gimbal_mode_text[32]{"Disabled"};

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
volatile float wl_fm_yaw_motor_pos_rad{0.0f};
volatile float wl_fm_yaw_motor_vel_rad_s{0.0f};
}

namespace {

constexpr int16_t kWheelSpinThreshold = 220;
constexpr int16_t kWheelActionThreshold = 320;
constexpr int16_t kWheelCenterThreshold = 80;
constexpr float kControlLoopDtS = 0.002f;
chassis_runtime::Actuators g_actuators{};

struct Dr16RawInput {
  bool online{false};
  rm::device::DR16::SwitchPosition switch_l{rm::device::DR16::SwitchPosition::kUnknown};
  rm::device::DR16::SwitchPosition switch_r{rm::device::DR16::SwitchPosition::kUnknown};
  int16_t dial{0};
};

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

const char *ToChassisModeText(const chassis::Fsm::State mode) {
  switch (mode) {
    case chassis::Fsm::State::kDisabled:
      return "Disabled";
    case chassis::Fsm::State::kLowLeg:
      return "LowLeg";
    case chassis::Fsm::State::kMidLeg:
      return "MidLeg";
    case chassis::Fsm::State::kHighLeg:
      return "HighLeg";
    case chassis::Fsm::State::kSpin:
      return "Spin";
    case chassis::Fsm::State::kJumpPrep:
      return "JumpPrep";
    case chassis::Fsm::State::kJumpPush:
      return "JumpPush";
    case chassis::Fsm::State::kJumpRecover:
      return "JumpRecover";
    case chassis::Fsm::State::kRecoveryFallCheck:
      return "RecoveryFallCheck";
    case chassis::Fsm::State::kRecoverySelfRight:
      return "RecoverySelfRight";
    default:
      return "Unknown";
  }
}

const char *ToGimbalModeText(const gimbal::Fsm::State mode) {
  switch (mode) {
    case gimbal::Fsm::State::kDisabled:
      return "Disabled";
    case gimbal::Fsm::State::kServiceWithFire:
      return "ServiceWithFire";
    case gimbal::Fsm::State::kServiceSafe:
      return "ServiceSafe";
    case gimbal::Fsm::State::kCombat:
      return "Combat";
    case gimbal::Fsm::State::kRecoveryAlign:
      return "RecoveryAlign";
    default:
      return "Unknown";
  }
}

void CopyTextToVolatile(volatile char *dst, size_t dst_size, const char *src) {
  if (dst_size == 0U) {
    return;
  }
  size_t i = 0U;
  for (; i + 1U < dst_size && src[i] != '\0'; ++i) {
    dst[i] = src[i];
  }
  dst[i] = '\0';
}

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
      .dial = g.dr16.dial(),
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
gimbal::Fsm::Input BuildGimbalFsmInput(const InputSnapshot &input, const chassis::Fsm::Output &chassis_output) {
  gimbal::Fsm::Input fsm_input{};
  fsm_input.request = input.mode_request;
  fsm_input.chassis_recovery_active = chassis_output.mode == chassis::Fsm::State::kRecoveryFallCheck ||
                                      chassis_output.mode == chassis::Fsm::State::kRecoverySelfRight;
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
  CopyTextToVolatile(wl_fm_chassis_mode_text, sizeof(wl_fm_chassis_mode_text), ToChassisModeText(debug.chassis_mode));
  CopyTextToVolatile(wl_fm_gimbal_mode_text, sizeof(wl_fm_gimbal_mode_text), ToGimbalModeText(debug.gimbal_mode));

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
  wl_fm_yaw_motor_pos_rad = gimbal_control_output.yaw_pos_rad;
  wl_fm_yaw_motor_vel_rad_s = gimbal_control_output.yaw_vel_rad_s;
}

}  // namespace

/**
 * @brief 500Hz 主控制循环
 */
void ControlLoop() {
  if (globals == nullptr) {
    return;
  }

  {
    static uint32_t invoke_count = 0;
    static uint32_t last_ms = 0;
    invoke_count++;
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t elapsed = now_ms - last_ms;
    if (elapsed >= 500) {
      wl_fm_timer_period_us = static_cast<float>(elapsed) * 1000.0f / static_cast<float>(invoke_count);
      invoke_count = 0;
      last_ms = now_ms;
    }
  }

  const uint32_t now_ms = HAL_GetTick();

  static InputSnapshot input{};
  static Dr16SemanticState dr16_semantic_state{};
  static chassis::Chassis::UpdateOutput chassis_control_output{};
  static gimbal::Gimbal::UpdateOutput gimbal_control_output{};
  UpdateRawFeedbackAndInputSnapshot(*globals, input, dr16_semantic_state);
  input.mode_request.current_leg_length_m = chassis_control_output.mean_leg_length_m;
  input.mode_request.tick_ms = now_ms;

  const chassis::Fsm::Input chassis_input = BuildChassisFsmInput(input, now_ms);
  const chassis::Fsm::Output chassis_output = globals->chassis_fsm.Update(chassis_input);

  const gimbal::Fsm::Input gimbal_input = BuildGimbalFsmInput(input, chassis_output);
  const gimbal::Fsm::Output gimbal_output = globals->gimbal_fsm.Update(gimbal_input);

  gimbal::Gimbal::UpdateInput gimbal_update_input{};
  gimbal_update_input.yaw_motor = globals->yaw_motor.has_value() ? &(*globals->yaw_motor) : nullptr;
  gimbal_update_input.gimbal_enable = gimbal_output.control.gimbal_enable;
  gimbal_update_input.align_to_chassis_forward = gimbal_output.control.align_to_chassis_forward;
  gimbal_update_input.target = gimbal_output.control.gimbal_target;
  gimbal_update_input.chassis_yaw_rad = input.estimator_input.imu.yaw_rad;
  globals->gimbal.Update(gimbal_update_input);
  gimbal_control_output = globals->gimbal.GetOutput();

  chassis::Chassis::UpdateInput chassis_update_input{};
  chassis_update_input.fsm_mode = chassis_output.mode;
  chassis_update_input.enable_output = chassis_output.control.enable_dm;
  chassis_update_input.run_chassis_update = chassis_output.control.run_chassis_update;
  chassis_update_input.spin_enable = chassis_output.control.spin_enable;
  chassis_update_input.target_leg_length_m = chassis_output.control.target_leg_length_m;
  chassis_update_input.estimator_input = input.estimator_input;
  chassis_update_input.estimator_input.dt_s = kControlLoopDtS;

  globals->chassis.Update(chassis_update_input);
  chassis_control_output = globals->chassis.GetOutput();
  g_actuators.ApplyChassisOutput(*globals, chassis_control_output, chassis_output.control.enable_dm);

  UpdateDebugSnapshot(now_ms, input, chassis_output, gimbal_output, chassis_control_output, gimbal_control_output);
}
