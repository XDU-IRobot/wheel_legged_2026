#include "include/globals.hpp"

#include "main.h"

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
}

namespace {

constexpr int16_t kDialJumpTriggerThreshold = 420;
constexpr int16_t kDialJumpReleaseThreshold = 260;
constexpr int16_t kDialSpinEnableThreshold = -360;
constexpr int16_t kDialSpinDisableThreshold = -220;

struct Dr16RawInput {
  bool online{false};
  rm::device::DR16::SwitchPosition switch_l{rm::device::DR16::SwitchPosition::kUnknown};
  rm::device::DR16::SwitchPosition switch_r{rm::device::DR16::SwitchPosition::kUnknown};
  int16_t dial{0};
};

struct InputSnapshot {
  bool input_valid{false};

  Dr16RawInput dr16{};

  bool chassis_enable_request{false};
  chassis::Fsm::LegLengthMode chassis_leg_length_mode{chassis::Fsm::LegLengthMode::kLow};
  bool chassis_spin_request{false};
  bool chassis_jump_request{false};
  bool chassis_fall_detected{false};
  bool chassis_upright_stable{true};
  float chassis_current_leg_length_m{0.0f};

  bool gimbal_enable_request{false};
  bool gimbal_safe_request{true};
  bool gimbal_host_target_external_valid{false};
  bool gimbal_host_target_valid{false};
  bool gimbal_fire_request{false};
};

struct DebugSnapshot {
  uint32_t tick_ms{0};
  chassis::Fsm::State chassis_mode{chassis::Fsm::State::kDisabled};
  gimbal::Fsm::State gimbal_mode{gimbal::Fsm::State::kDisabled};
  bool chassis_state_changed{false};
  bool gimbal_state_changed{false};
  bool dr16_chassis_enable_request{false};
  bool dr16_chassis_spin_request{false};
  bool dr16_chassis_jump_trigger_edge{false};
};

struct Dr16SemanticState {
  bool jump_latched{false};
  bool spin_latched{false};
};

chassis::Fsm::LegLengthMode ResolveLegLengthMode(
    const rm::device::DR16::SwitchPosition switch_r) {
  switch (switch_r) {
  case rm::device::DR16::SwitchPosition::kDown:
    return chassis::Fsm::LegLengthMode::kLow;
  case rm::device::DR16::SwitchPosition::kMid:
    return chassis::Fsm::LegLengthMode::kMid;
  case rm::device::DR16::SwitchPosition::kUp:
    return chassis::Fsm::LegLengthMode::kHigh;
  default:
    return chassis::Fsm::LegLengthMode::kLow;
  }
}

bool IsEnableSwitch(const rm::device::DR16::SwitchPosition switch_l) {
  return switch_l == rm::device::DR16::SwitchPosition::kMid ||
         switch_l == rm::device::DR16::SwitchPosition::kUp;
}

void ApplyDr16Semantics(const Dr16RawInput &dr16,
                        Dr16SemanticState &semantic_state,
                        InputSnapshot &input) {
  input.input_valid = dr16.online;
  input.dr16 = dr16;

  const bool chassis_enable = dr16.online && IsEnableSwitch(dr16.switch_l);
  input.chassis_enable_request = chassis_enable;
  input.chassis_leg_length_mode = ResolveLegLengthMode(dr16.switch_r);

  if (dr16.dial <= kDialSpinEnableThreshold) {
    semantic_state.spin_latched = true;
  } else if (dr16.dial >= kDialSpinDisableThreshold) {
    semantic_state.spin_latched = false;
  }
  input.chassis_spin_request = chassis_enable && semantic_state.spin_latched;

  const bool jump_level = dr16.dial >= kDialJumpTriggerThreshold;
  input.chassis_jump_request = chassis_enable && jump_level && !semantic_state.jump_latched;
  if (jump_level) {
    semantic_state.jump_latched = true;
  } else if (dr16.dial <= kDialJumpReleaseThreshold) {
    semantic_state.jump_latched = false;
  }

  input.gimbal_enable_request = chassis_enable;
  input.gimbal_safe_request = dr16.switch_r == rm::device::DR16::SwitchPosition::kDown;
  const bool host_mode_selected = dr16.switch_r == rm::device::DR16::SwitchPosition::kUp;
  input.gimbal_host_target_valid =
      host_mode_selected && input.gimbal_host_target_external_valid;
  input.gimbal_fire_request = false;
}

void UpdateRawFeedbackAndInputSnapshot(SharedResources &g,
                                       InputSnapshot &input,
                                       Dr16SemanticState &semantic_state) {
  Dr16RawInput dr16{
      .online = (g.dr16.online_status() == rm::device::Device::kOk),
      .switch_l = g.dr16.switch_l(),
      .switch_r = g.dr16.switch_r(),
      .dial = g.dr16.dial(),
  };
  ApplyDr16Semantics(dr16, semantic_state, input);
}

chassis::Fsm::Input BuildChassisFsmInput(const InputSnapshot &input,
                                         const uint32_t tick_ms) {
  chassis::Fsm::Input fsm_input{};
  fsm_input.input_valid = input.input_valid;
  fsm_input.force_enable = input.chassis_enable_request;
  fsm_input.leg_length_mode = input.chassis_leg_length_mode;
  fsm_input.spin_enable = input.chassis_spin_request;
  fsm_input.jump_trigger = input.chassis_jump_request;
  fsm_input.fall_detected = input.chassis_fall_detected;
  fsm_input.upright_stable = input.chassis_upright_stable;
  fsm_input.current_leg_length_m = input.chassis_current_leg_length_m;
  fsm_input.tick_ms = tick_ms;
  return fsm_input;
}

gimbal::Fsm::Input BuildGimbalFsmInput(const InputSnapshot &input,
                                       const chassis::Fsm::Output &chassis_output) {
  gimbal::Fsm::Input fsm_input{};
  fsm_input.input_valid = input.input_valid;
  fsm_input.enable_request = input.gimbal_enable_request;
  fsm_input.safe_request = input.gimbal_safe_request;
  fsm_input.host_target_valid = input.gimbal_host_target_valid;
  fsm_input.chassis_recovery_active =
      chassis_output.mode == chassis::Fsm::State::kRecoveryFallCheck ||
      chassis_output.mode == chassis::Fsm::State::kRecoverySelfRight;
  fsm_input.fire_request = input.gimbal_fire_request;
  return fsm_input;
}

void UpdateDebugSnapshot(const uint32_t tick_ms,
                         const InputSnapshot &input,
                         const chassis::Fsm::Output &chassis_output,
                         const gimbal::Fsm::Output &gimbal_output) {
  DebugSnapshot debug{};
  debug.tick_ms = tick_ms;
  debug.chassis_mode = chassis_output.mode;
  debug.gimbal_mode = gimbal_output.mode;
  debug.chassis_state_changed = chassis_output.state_changed;
  debug.gimbal_state_changed = gimbal_output.state_changed;
  debug.dr16_chassis_enable_request = input.chassis_enable_request;
  debug.dr16_chassis_spin_request = input.chassis_spin_request;
  debug.dr16_chassis_jump_trigger_edge = input.chassis_jump_request;

  wl_fm_tick_ms = debug.tick_ms;
  wl_fm_chassis_mode = static_cast<uint8_t>(debug.chassis_mode);
  wl_fm_gimbal_mode = static_cast<uint8_t>(debug.gimbal_mode);
  wl_fm_chassis_state_changed = static_cast<uint8_t>(debug.chassis_state_changed);
  wl_fm_gimbal_state_changed = static_cast<uint8_t>(debug.gimbal_state_changed);

  wl_fm_dr16_online = static_cast<uint8_t>(input.dr16.online);
  wl_fm_dr16_switch_l = static_cast<int32_t>(input.dr16.switch_l);
  wl_fm_dr16_switch_r = static_cast<int32_t>(input.dr16.switch_r);
  wl_fm_dr16_dial = input.dr16.dial;

  wl_fm_dr16_enable_request = static_cast<uint8_t>(debug.dr16_chassis_enable_request);
  wl_fm_dr16_spin_request = static_cast<uint8_t>(debug.dr16_chassis_spin_request);
  wl_fm_dr16_jump_trigger_edge =
      static_cast<uint8_t>(debug.dr16_chassis_jump_trigger_edge);
}

}  // namespace

void ControlLoop() {
  if (globals == nullptr) {
    return;
  }

  const uint32_t now_ms = HAL_GetTick();

  static InputSnapshot input{};
  static Dr16SemanticState dr16_semantic_state{};
  UpdateRawFeedbackAndInputSnapshot(*globals, input, dr16_semantic_state);

  const chassis::Fsm::Input chassis_input = BuildChassisFsmInput(input, now_ms);
  const chassis::Fsm::Output chassis_output = globals->chassis_fsm.Update(chassis_input);

  const gimbal::Fsm::Input gimbal_input = BuildGimbalFsmInput(input, chassis_output);
  const gimbal::Fsm::Output gimbal_output = globals->gimbal_fsm.Update(gimbal_input);

  UpdateDebugSnapshot(now_ms, input, chassis_output, gimbal_output);
}

