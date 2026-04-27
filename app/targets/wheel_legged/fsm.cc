#include "include/chassis/fsm.hpp"

#include "etl/fsm.h"
#include "etl/message.h"

namespace {

enum EtlStateId : etl::fsm_state_id_t {
  kDisabledState = 0,
  kStandbyState = 1,
  kBalanceState = 2,
  kSpinState = 3,
  kJumpPrepState = 4,
  kJumpPushState = 5,
  kJumpRecoverState = 6,
  kRecoveryFallCheckState = 7,
  kRecoverySelfRightState = 8,
  kStateCount = 9,
};

constexpr uint32_t kJumpPrepMs = 450U;
constexpr uint32_t kJumpPushMs = 1000U;
constexpr uint32_t kJumpRecoverMs = 450U;
constexpr float kJumpPushTargetLegLengthM = 0.35f;
constexpr float kJumpPushReachTolM = 0.01f;

constexpr EtlStateId ToEtlStateId(const chassis::Fsm::State mode) {
  switch (mode) {
    case chassis::Fsm::State::kDisabled:
      return kDisabledState;
    case chassis::Fsm::State::kStandby:
      return kStandbyState;
    case chassis::Fsm::State::kBalance:
      return kBalanceState;
    case chassis::Fsm::State::kSpin:
      return kSpinState;
    case chassis::Fsm::State::kJumpPrep:
      return kJumpPrepState;
    case chassis::Fsm::State::kJumpPush:
      return kJumpPushState;
    case chassis::Fsm::State::kJumpRecover:
      return kJumpRecoverState;
    case chassis::Fsm::State::kRecoveryFallCheck:
      return kRecoveryFallCheckState;
    case chassis::Fsm::State::kRecoverySelfRight:
      return kRecoverySelfRightState;
    default:
      return kDisabledState;
  }
}

chassis::Fsm::State ResolveDriveState(const bool spin_enable) {
  return spin_enable ? chassis::Fsm::State::kSpin : chassis::Fsm::State::kBalance;
}

bool IsJumpState(const chassis::Fsm::State mode) {
  return mode == chassis::Fsm::State::kJumpPrep || mode == chassis::Fsm::State::kJumpPush ||
         mode == chassis::Fsm::State::kJumpRecover;
}

float ResolveLegLengthTarget(const chassis::Fsm::LegLengthMode leg_length_mode) {
  switch (leg_length_mode) {
    case chassis::Fsm::LegLengthMode::kLow:
      return 0.18f;
    case chassis::Fsm::LegLengthMode::kMid:
      return 0.23f;
    case chassis::Fsm::LegLengthMode::kHigh:
      return 0.23f;
    default:
      return 0.18f;
  }
}

chassis::Fsm::Output::ControlOutput BuildControlOutput(const chassis::Fsm::State mode,
                                                       const chassis::Fsm::LegLengthMode leg_length_mode) {
  chassis::Fsm::Output::ControlOutput control{};
  switch (mode) {
    case chassis::Fsm::State::kDisabled:
      control.enable_dm = false;
      control.run_chassis_update = false;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = true;
      control.target_leg_length_m = 0.18f;
      control.jump_phase = 0U;
      break;

    case chassis::Fsm::State::kStandby:
      control.enable_dm = false;
      control.run_chassis_update = false;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = true;
      control.target_leg_length_m = 0.18f;
      control.jump_phase = 0U;
      break;

    case chassis::Fsm::State::kBalance:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.target_leg_length_m = ResolveLegLengthTarget(leg_length_mode);
      control.jump_phase = 0U;
      break;

    case chassis::Fsm::State::kSpin:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = true;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.target_leg_length_m = ResolveLegLengthTarget(chassis::Fsm::LegLengthMode::kLow);
      control.jump_phase = 0U;
      break;

    case chassis::Fsm::State::kJumpPrep:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.target_leg_length_m = 0.14f;
      control.jump_phase = 1U;
      break;

    case chassis::Fsm::State::kJumpPush:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.target_leg_length_m = 0.38f;
      control.jump_phase = 2U;
      break;

    case chassis::Fsm::State::kJumpRecover:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.target_leg_length_m = 0.2f;
      control.jump_phase = 3U;
      break;

    case chassis::Fsm::State::kRecoveryFallCheck:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = true;
      control.safe_output_required = false;
      control.target_leg_length_m = ResolveLegLengthTarget(leg_length_mode);
      control.jump_phase = 0U;
      break;

    case chassis::Fsm::State::kRecoverySelfRight:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = true;
      control.safe_output_required = false;
      control.target_leg_length_m = ResolveLegLengthTarget(chassis::Fsm::LegLengthMode::kLow);
      control.jump_phase = 0U;
      break;

    default:
      control.enable_dm = false;
      control.run_chassis_update = false;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = true;
      control.target_leg_length_m = ResolveLegLengthTarget(leg_length_mode);
      control.jump_phase = 0U;
      break;
  }

  return control;
}

struct ModeRequestEvent : public etl::message<1> {
  bool force_enable;
  bool spin_enable;
  bool jump_trigger_rise;
  bool fall_detected;
  bool upright_stable;
  float current_leg_length_m;
  uint32_t tick_ms;

  explicit ModeRequestEvent(const bool force_enable, const bool spin_enable, const bool jump_trigger_rise,
                            const bool fall_detected, const bool upright_stable, const float current_leg_length_m,
                            const uint32_t tick_ms)
      : force_enable(force_enable),
        spin_enable(spin_enable),
        jump_trigger_rise(jump_trigger_rise),
        fall_detected(fall_detected),
        upright_stable(upright_stable),
        current_leg_length_m(current_leg_length_m),
        tick_ms(tick_ms) {}
};

etl::fsm_state_id_t ResolveRequestedState(const ModeRequestEvent &event) {
  return ToEtlStateId(ResolveDriveState(event.spin_enable));
}

class FsmEtlContext;

class DisabledState : public etl::fsm_state<FsmEtlContext, DisabledState, kDisabledState, ModeRequestEvent> {
 public:
  etl::fsm_state_id_t on_event(const ModeRequestEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &);
  etl::fsm_state_id_t on_enter_state();
};

class StandbyState : public etl::fsm_state<FsmEtlContext, StandbyState, kStandbyState, ModeRequestEvent> {
 public:
  etl::fsm_state_id_t on_event(const ModeRequestEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &);
  etl::fsm_state_id_t on_enter_state();
};

class BalanceState : public etl::fsm_state<FsmEtlContext, BalanceState, kBalanceState, ModeRequestEvent> {
 public:
  etl::fsm_state_id_t on_event(const ModeRequestEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &);
  etl::fsm_state_id_t on_enter_state();
};

class SpinState : public etl::fsm_state<FsmEtlContext, SpinState, kSpinState, ModeRequestEvent> {
 public:
  etl::fsm_state_id_t on_event(const ModeRequestEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &);
  etl::fsm_state_id_t on_enter_state();
};

class JumpPrepState : public etl::fsm_state<FsmEtlContext, JumpPrepState, kJumpPrepState, ModeRequestEvent> {
 public:
  etl::fsm_state_id_t on_event(const ModeRequestEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &);
  etl::fsm_state_id_t on_enter_state();
};

class JumpPushState : public etl::fsm_state<FsmEtlContext, JumpPushState, kJumpPushState, ModeRequestEvent> {
 public:
  etl::fsm_state_id_t on_event(const ModeRequestEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &);
  etl::fsm_state_id_t on_enter_state();
};

class JumpRecoverState : public etl::fsm_state<FsmEtlContext, JumpRecoverState, kJumpRecoverState, ModeRequestEvent> {
 public:
  etl::fsm_state_id_t on_event(const ModeRequestEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &);
  etl::fsm_state_id_t on_enter_state();
};

class RecoveryFallCheckState
    : public etl::fsm_state<FsmEtlContext, RecoveryFallCheckState, kRecoveryFallCheckState, ModeRequestEvent> {
 public:
  etl::fsm_state_id_t on_event(const ModeRequestEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &);
  etl::fsm_state_id_t on_enter_state();
};

class RecoverySelfRightState
    : public etl::fsm_state<FsmEtlContext, RecoverySelfRightState, kRecoverySelfRightState, ModeRequestEvent> {
 public:
  etl::fsm_state_id_t on_event(const ModeRequestEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &);
  etl::fsm_state_id_t on_enter_state();
};

class FsmEtlContext : public etl::fsm {
 public:
  explicit FsmEtlContext(chassis::Fsm &owner) : etl::fsm(0x41), owner_(owner) {
    states_[0] = &disabled_;
    states_[1] = &standby_;
    states_[2] = &balance_;
    states_[3] = &spin_;
    states_[4] = &jump_prep_;
    states_[5] = &jump_push_;
    states_[6] = &jump_recover_;
    states_[7] = &recovery_fall_check_;
    states_[8] = &recovery_self_right_;
    set_states(states_, kStateCount);
  }

  void ApplyMode(chassis::Fsm::State mode) { owner_.Transit(mode); }
  void Dispatch(const ModeRequestEvent &event) {
    tick_ms_ = event.tick_ms;
    receive(event);
  }
  void MarkJumpPhaseEnter() { jump_phase_enter_ms_ = tick_ms_; }
  uint32_t JumpPhaseElapsedMs() const { return tick_ms_ - jump_phase_enter_ms_; }

 private:
  chassis::Fsm &owner_;
  DisabledState disabled_;
  StandbyState standby_;
  BalanceState balance_;
  SpinState spin_;
  JumpPrepState jump_prep_;
  JumpPushState jump_push_;
  JumpRecoverState jump_recover_;
  RecoveryFallCheckState recovery_fall_check_;
  RecoverySelfRightState recovery_self_right_;
  etl::ifsm_state *states_[kStateCount]{};
  uint32_t tick_ms_{0};
  uint32_t jump_phase_enter_ms_{0};
};

etl::fsm_state_id_t DisabledState::on_event(const ModeRequestEvent &event) {
  if (!event.force_enable) {
    return No_State_Change;
  }
  if (event.jump_trigger_rise) {
    return kJumpPrepState;
  }
  return ResolveRequestedState(event);
}

etl::fsm_state_id_t DisabledState::on_event_unknown(const etl::imessage &) { return No_State_Change; }

etl::fsm_state_id_t DisabledState::on_enter_state() {
  get_fsm_context().ApplyMode(chassis::Fsm::State::kDisabled);
  return No_State_Change;
}

etl::fsm_state_id_t StandbyState::on_event(const ModeRequestEvent &event) {
  if (!event.force_enable) {
    return kDisabledState;
  }
  if (event.jump_trigger_rise) {
    return kJumpPrepState;
  }
  return ResolveRequestedState(event);
}

etl::fsm_state_id_t StandbyState::on_event_unknown(const etl::imessage &) { return No_State_Change; }

etl::fsm_state_id_t StandbyState::on_enter_state() {
  get_fsm_context().ApplyMode(chassis::Fsm::State::kStandby);
  return No_State_Change;
}

etl::fsm_state_id_t BalanceState::on_event(const ModeRequestEvent &event) {
  if (!event.force_enable) {
    return kDisabledState;
  }
  if (event.jump_trigger_rise) {
    return kJumpPrepState;
  }
  return ResolveRequestedState(event);
}

etl::fsm_state_id_t BalanceState::on_event_unknown(const etl::imessage &) { return No_State_Change; }

etl::fsm_state_id_t BalanceState::on_enter_state() {
  get_fsm_context().ApplyMode(chassis::Fsm::State::kBalance);
  return No_State_Change;
}

etl::fsm_state_id_t SpinState::on_event(const ModeRequestEvent &event) {
  if (!event.force_enable) {
    return kDisabledState;
  }
  if (event.jump_trigger_rise) {
    return kJumpPrepState;
  }
  return ResolveRequestedState(event);
}

etl::fsm_state_id_t SpinState::on_event_unknown(const etl::imessage &) { return No_State_Change; }

etl::fsm_state_id_t SpinState::on_enter_state() {
  get_fsm_context().ApplyMode(chassis::Fsm::State::kSpin);
  return No_State_Change;
}

etl::fsm_state_id_t JumpPrepState::on_event(const ModeRequestEvent &event) {
  if (!event.force_enable) {
    return kDisabledState;
  }
  return get_fsm_context().JumpPhaseElapsedMs() >= kJumpPrepMs ? kJumpPushState : No_State_Change;
}

etl::fsm_state_id_t JumpPrepState::on_event_unknown(const etl::imessage &) { return No_State_Change; }

etl::fsm_state_id_t JumpPrepState::on_enter_state() {
  get_fsm_context().MarkJumpPhaseEnter();
  get_fsm_context().ApplyMode(chassis::Fsm::State::kJumpPrep);
  return No_State_Change;
}

etl::fsm_state_id_t JumpPushState::on_event(const ModeRequestEvent &event) {
  if (!event.force_enable) {
    return kDisabledState;
  }
  const bool leg_reached = event.current_leg_length_m >= (kJumpPushTargetLegLengthM - kJumpPushReachTolM);
  if (leg_reached || get_fsm_context().JumpPhaseElapsedMs() >= kJumpPushMs) {
    return kJumpRecoverState;
  }
  return No_State_Change;
}

etl::fsm_state_id_t JumpPushState::on_event_unknown(const etl::imessage &) { return No_State_Change; }

etl::fsm_state_id_t JumpPushState::on_enter_state() {
  get_fsm_context().MarkJumpPhaseEnter();
  get_fsm_context().ApplyMode(chassis::Fsm::State::kJumpPush);
  return No_State_Change;
}

etl::fsm_state_id_t JumpRecoverState::on_event(const ModeRequestEvent &event) {
  if (!event.force_enable) {
    return kDisabledState;
  }
  if (get_fsm_context().JumpPhaseElapsedMs() < kJumpRecoverMs) {
    return No_State_Change;
  }
  return ResolveRequestedState(event);
}

etl::fsm_state_id_t JumpRecoverState::on_event_unknown(const etl::imessage &) { return No_State_Change; }

etl::fsm_state_id_t JumpRecoverState::on_enter_state() {
  get_fsm_context().MarkJumpPhaseEnter();
  get_fsm_context().ApplyMode(chassis::Fsm::State::kJumpRecover);
  return No_State_Change;
}

etl::fsm_state_id_t RecoveryFallCheckState::on_event(const ModeRequestEvent &event) {
  if (!event.force_enable) {
    return kDisabledState;
  }
  return ResolveRequestedState(event);
}

etl::fsm_state_id_t RecoveryFallCheckState::on_event_unknown(const etl::imessage &) { return No_State_Change; }

etl::fsm_state_id_t RecoveryFallCheckState::on_enter_state() {
  get_fsm_context().ApplyMode(chassis::Fsm::State::kRecoveryFallCheck);
  return No_State_Change;
}

etl::fsm_state_id_t RecoverySelfRightState::on_event(const ModeRequestEvent &event) {
  if (!event.force_enable) {
    return kDisabledState;
  }
  return ResolveRequestedState(event);
}

etl::fsm_state_id_t RecoverySelfRightState::on_event_unknown(const etl::imessage &) { return No_State_Change; }

etl::fsm_state_id_t RecoverySelfRightState::on_enter_state() {
  get_fsm_context().ApplyMode(chassis::Fsm::State::kRecoverySelfRight);
  return No_State_Change;
}

}  // namespace

struct chassis::Fsm::EtlImpl {
  explicit EtlImpl(chassis::Fsm &owner) : context(owner) {}

  FsmEtlContext context;
  bool last_jump_trigger{false};
};

chassis::Fsm::~Fsm() {
  delete etl_impl_;
  etl_impl_ = nullptr;
}

void chassis::Fsm::Init() {
  if (!etl_impl_) {
    etl_impl_ = new EtlImpl(*this);
  }

  if (etl_impl_ && !etl_impl_->context.is_started()) {
    etl_impl_->context.start(true);
  }
}

void chassis::Fsm::Transit(const State new_mode) {
  output_.state_changed = (new_mode != mode_);
  mode_ = new_mode;
  output_.mode = mode_;
  output_.control = BuildControlOutput(mode_, leg_length_mode_);
}

chassis::Fsm::Output chassis::Fsm::Update(const Input &input) {
  if (!etl_impl_) {
    Init();
  }

  output_.state_changed = false;

  if (!input.input_valid) {
    Transit(State::kDisabled);
    etl_impl_->last_jump_trigger = false;
    return output_;
  }

  leg_length_mode_ = input.leg_length_mode;

  const bool jump_trigger_rise = input.jump_trigger && !etl_impl_->last_jump_trigger;
  etl_impl_->last_jump_trigger = input.jump_trigger;

  const bool should_dispatch = jump_trigger_rise || IsJumpState(mode_) ||
                               (ToEtlStateId(ResolveDriveState(input.spin_enable)) != ToEtlStateId(mode_)) ||
                               (mode_ == State::kDisabled && input.force_enable) ||
                               (mode_ != State::kDisabled && !input.force_enable);

  if (!should_dispatch) {
    output_.control = BuildControlOutput(mode_, leg_length_mode_);
    return output_;
  }

  etl_impl_->context.Dispatch(ModeRequestEvent{input.force_enable, input.spin_enable, jump_trigger_rise,
                                               input.fall_detected, input.upright_stable, input.current_leg_length_m,
                                               input.tick_ms});

  return output_;
}
