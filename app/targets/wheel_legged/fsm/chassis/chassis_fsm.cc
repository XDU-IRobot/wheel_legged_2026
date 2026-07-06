#include "chassis_fsm.hpp"

#include <cmath>

#include "../../include/params.hpp"

namespace wheel_legged::fsm {

namespace {

namespace chassis_params = wheel_legged::params::active::chassis_fsm;

constexpr etl::message_router_id_t kChassisFsmRouterId = 2U;

constexpr etl::fsm_state_id_t StateId(const ChassisState state) { return static_cast<etl::fsm_state_id_t>(state); }

bool IsDisableRequested(const ChassisFsmRequest &request) {
  return !request.input_valid || request.requested_state == ChassisState::kDisable;
}

bool IsStableState(const ChassisState state) {
  return state == ChassisState::kStandby || state == ChassisState::kNormal || state == ChassisState::kFly ||
         state == ChassisState::kUpstairs;
}

ChassisState ResolveStableState(ChassisFsm &machine, const ChassisFsmRequest &request) {
  if (machine.context().spin_lock_low) {
    if (IsStableState(request.requested_state) && request.requested_state != machine.context().spin_locked_request) {
      machine.context().spin_lock_low = false;
      return request.requested_state;
    }
    return ChassisState::kNormal;
  }
  if (IsStableState(request.requested_state)) {
    return request.requested_state;
  }
  return machine.context().previous_stable_state;
}

etl::fsm_state_id_t ResolveGlobalSafety(ChassisFsm &machine, const ChassisFsmRequest &request,
                                        const bool allow_fall_transition = true) {
  if (IsDisableRequested(request)) {
    const auto reason = request.input_valid ? TransitionReason::kModeRequested : TransitionReason::kInputLost;
    return machine.TransitionTo(ChassisState::kDisable, reason);
  }
  if (allow_fall_transition && (request.fall_detected || request.upstairs_recovery_required)) {
    const auto reason = request.fall_detected ? TransitionReason::kFallDetected : TransitionReason::kRecoveryRequested;
    return machine.TransitionTo(ChassisState::kFall, reason);
  }
  return etl::ifsm_state::No_State_Change;
}

bool SpinEntryAllowed(const ChassisFsmRequest &request) {
  return std::fabs(request.current_speed_mps) < chassis_params::kSpinEntrySpeedThresholdMps;
}

etl::fsm_state_id_t ResolveActionOrStable(ChassisFsm &machine, const ChassisFsmRequest &request,
                                          const bool jump_allowed) {
  if (jump_allowed && request.jump_trigger && request.jump_mode == JumpMode::kManual) {
    machine.context().jump_return_state =
        machine.state() == ChassisState::kFly ? ChassisState::kFly : ChassisState::kNormal;
    return machine.TransitionTo(ChassisState::kJump, TransitionReason::kModeRequested);
  }
  if ((request.special_spin_hold || request.requested_state == ChassisState::kSpecialSpin) &&
      SpinEntryAllowed(request)) {
    return machine.TransitionTo(ChassisState::kSpecialSpin, TransitionReason::kModeRequested);
  }
  if ((request.spin_hold || request.requested_state == ChassisState::kSpin) && SpinEntryAllowed(request)) {
    return machine.TransitionTo(ChassisState::kSpin, TransitionReason::kModeRequested);
  }
  return machine.TransitionTo(ResolveStableState(machine, request), TransitionReason::kModeRequested);
}

uint32_t PhaseElapsedMs(const ChassisFsm &machine, const ChassisFsmRequest &request) {
  return request.tick_ms - machine.context().phase_enter_tick_ms;
}

}  // namespace

ChassisFsm::ChassisFsm() : etl::fsm(kChassisFsmRouterId) {
  state_list_[StateId(ChassisState::kDisable)] = &disable_state_;
  state_list_[StateId(ChassisState::kStandby)] = &standby_state_;
  state_list_[StateId(ChassisState::kNormal)] = &normal_state_;
  state_list_[StateId(ChassisState::kFly)] = &fly_state_;
  state_list_[StateId(ChassisState::kUpstairs)] = &upstairs_state_;
  state_list_[StateId(ChassisState::kSpin)] = &spin_state_;
  state_list_[StateId(ChassisState::kSpecialSpin)] = &special_spin_state_;
  state_list_[StateId(ChassisState::kFall)] = &fall_state_;
  state_list_[StateId(ChassisState::kJump)] = &jump_state_;
  set_states(state_list_, static_cast<uint8_t>(ChassisState::kCount));
  Init();
}

void ChassisFsm::Init() {
  if (is_started()) {
    reset(true);
  }
  request_ = {};
  output_ = {};
  context_ = {};
  pending_transition_reason_ = TransitionReason::kInitialized;
  has_entered_state_ = false;
  start();
}

ChassisFsmOutput ChassisFsm::Update(const ChassisFsmRequest &request) {
  request_ = request;
  output_.state_changed = false;
  output_.transition_reason = TransitionReason::kNone;

  const ChassisUpdateEvent event{request_};
  receive(event);

  output_.state_elapsed_ms = request_.tick_ms - context_.state_enter_tick_ms;
  SyncPhaseOutput();
  return output_;
}

etl::fsm_state_id_t ChassisFsm::TransitionTo(const ChassisState state, const TransitionReason reason) {
  if (is_started() && get_state_id() == StateId(state)) {
    return etl::ifsm_state::No_State_Change;
  }
  pending_transition_reason_ = reason;
  return StateId(state);
}

void ChassisFsm::EnterState(const ChassisState state) {
  output_.state_changed = has_entered_state_ && output_.state != state;
  output_.state = state;
  output_.transition_reason = pending_transition_reason_;
  output_.state_elapsed_ms = 0U;
  context_.state_enter_tick_ms = request_.tick_ms;
  pending_transition_reason_ = TransitionReason::kNone;
  has_entered_state_ = true;

  if (state == ChassisState::kNormal || state == ChassisState::kFly || state == ChassisState::kUpstairs) {
    context_.previous_stable_state = state;
  }
  if (state == ChassisState::kDisable) {
    context_.spin_lock_low = false;
  } else if (state == ChassisState::kSpin || state == ChassisState::kSpecialSpin) {
    context_.spin_phase = SpinPhase::kRunning;
    context_.spin_lock_low = false;
    context_.spin_locked_request = context_.previous_stable_state;
    EnterPhase(request_.tick_ms);
  } else if (state == ChassisState::kJump) {
    context_.jump_phase = JumpPhase::kPrepare;
    context_.jump_mode = request_.jump_mode;
    context_.jump_push_reached_armed = true;
    EnterPhase(request_.tick_ms);
  } else if (state == ChassisState::kFall) {
    context_.fall_phase = FallPhase::kConfirm;
    EnterPhase(request_.tick_ms);
  } else if (state == ChassisState::kUpstairs) {
    context_.upstairs_mode = request_.upstairs_mode;
  }
  SyncPhaseOutput();
}

void ChassisFsm::EnterPhase(const uint32_t tick_ms) { context_.phase_enter_tick_ms = tick_ms; }

void ChassisFsm::SyncPhaseOutput() {
  output_.spin_phase = context_.spin_phase;
  output_.jump_phase = context_.jump_phase;
  output_.jump_mode = context_.jump_mode;
  output_.fall_phase = context_.fall_phase;
  output_.upstairs_mode = context_.upstairs_mode;
}

void ChassisFsm::WriteBaseOutput(const ChassisState state, const ChassisBehavior behavior,
                                 const wheel_legged::LegProfile leg_profile, const bool motor_enable,
                                 const bool wheel_torque_enable, const bool run_controller) {
  output_.state = state;
  output_.behavior = behavior;
  output_.leg_profile = leg_profile;
  output_.chassis_motor_enable = motor_enable;
  output_.wheel_torque_enable = wheel_torque_enable;
  output_.run_controller = run_controller;
  output_.upstairs_task_active = false;
  SyncPhaseOutput();
}

void ChassisFsm::WriteDisableOutput() {
  WriteBaseOutput(ChassisState::kDisable, ChassisBehavior::kSafeStop, wheel_legged::LegProfile::kLow, false, false,
                  false);
  output_.target_leg_length_m = chassis_params::kLowLegLengthM;
}

void ChassisFsm::WriteStandbyOutput() {
  WriteBaseOutput(ChassisState::kStandby, ChassisBehavior::kStandby, wheel_legged::LegProfile::kLow, true, false, true);
  output_.target_leg_length_m = chassis_params::kLowLegLengthM;
}

void ChassisFsm::WriteNormalOutput() {
  WriteBaseOutput(ChassisState::kNormal, ChassisBehavior::kBalance, wheel_legged::LegProfile::kLow, true, true, true);
  output_.target_leg_length_m = chassis_params::kLowLegLengthM;
}

void ChassisFsm::WriteFlyOutput() {
  WriteBaseOutput(ChassisState::kFly, ChassisBehavior::kBalance, wheel_legged::LegProfile::kMid, true, true, true);
  output_.target_leg_length_m = chassis_params::kMidLegLengthM;
}

void ChassisFsm::WriteUpstairsOutput() {
  WriteBaseOutput(ChassisState::kUpstairs, ChassisBehavior::kUpstairs, wheel_legged::LegProfile::kHigh, true, true,
                  true);
  output_.target_leg_length_m = !request_.upstairs_active ? chassis_params::kHighLegLengthM
                                : context_.upstairs_mode == UpstairsMode::kDouble
                                    ? chassis_params::kStairClimbStep2.high_leg_length_m
                                    : chassis_params::kStairClimb.high_leg_length_m;
  output_.upstairs_task_active = request_.upstairs_active;
}

void ChassisFsm::WriteSpinOutput() {
  WriteBaseOutput(ChassisState::kSpin, ChassisBehavior::kSpin, wheel_legged::LegProfile::kLow, true, true, true);
  output_.target_leg_length_m = chassis_params::kLowLegLengthM;
}

void ChassisFsm::WriteSpecialSpinOutput() {
  // SpecialSpin 目前只保留状态语义，执行行为安全降级为普通 Spin。
  WriteBaseOutput(ChassisState::kSpecialSpin, ChassisBehavior::kSpin, wheel_legged::LegProfile::kLow, true, true, true);
  output_.target_leg_length_m = chassis_params::kLowLegLengthM;
}

void ChassisFsm::WriteFallOutput() {
  WriteBaseOutput(ChassisState::kFall, ChassisBehavior::kFallRecovery, wheel_legged::LegProfile::kLow, true, false,
                  true);
  output_.target_leg_length_m = chassis_params::kLowLegLengthM;
}

void ChassisFsm::WriteJumpOutput() {
  const auto leg_profile = context_.jump_return_state == ChassisState::kFly ? wheel_legged::LegProfile::kMid
                                                                            : wheel_legged::LegProfile::kLow;
  WriteBaseOutput(ChassisState::kJump, ChassisBehavior::kJump, leg_profile, true, true, true);
  switch (context_.jump_phase) {
    case JumpPhase::kPrepare:
      output_.target_leg_length_m = chassis_params::kJumpLowPrepLegLengthM;
      break;
    case JumpPhase::kPush:
      output_.target_leg_length_m = chassis_params::kJumpLowPushLegLengthM;
      break;
    case JumpPhase::kRecover:
      output_.target_leg_length_m = chassis_params::kJumpLowRecoverLegLengthM;
      break;
  }
}

template <typename TDerived, ChassisState TState>
etl::fsm_state_id_t ChassisStateBase<TDerived, TState>::on_event(const DisableEvent &) {
  return machine().TransitionTo(ChassisState::kDisable, TransitionReason::kPowerDisabled);
}

template <typename TDerived, ChassisState TState>
etl::fsm_state_id_t ChassisStateBase<TDerived, TState>::on_event(const ResetEvent &) {
  return machine().TransitionTo(ChassisState::kDisable, TransitionReason::kInitialized);
}

template <typename TDerived, ChassisState TState>
etl::fsm_state_id_t ChassisStateBase<TDerived, TState>::on_event_unknown(const etl::imessage &) {
  return etl::ifsm_state::No_State_Change;
}

#define WHEEL_LEGGED_DEFINE_CHASSIS_ENTRY_EXIT(ClassName, StateValue, WriteMethod) \
  etl::fsm_state_id_t ClassName::on_enter_state() {                                \
    machine().EnterState(StateValue);                                              \
    machine().WriteMethod();                                                       \
    return etl::ifsm_state::No_State_Change;                                       \
  }                                                                                \
  void ClassName::on_exit_state() {}

WHEEL_LEGGED_DEFINE_CHASSIS_ENTRY_EXIT(ChassisDisableState, ChassisState::kDisable, WriteDisableOutput)
WHEEL_LEGGED_DEFINE_CHASSIS_ENTRY_EXIT(ChassisStandbyState, ChassisState::kStandby, WriteStandbyOutput)
WHEEL_LEGGED_DEFINE_CHASSIS_ENTRY_EXIT(ChassisNormalState, ChassisState::kNormal, WriteNormalOutput)
WHEEL_LEGGED_DEFINE_CHASSIS_ENTRY_EXIT(ChassisFlyState, ChassisState::kFly, WriteFlyOutput)
WHEEL_LEGGED_DEFINE_CHASSIS_ENTRY_EXIT(ChassisUpstairsState, ChassisState::kUpstairs, WriteUpstairsOutput)
WHEEL_LEGGED_DEFINE_CHASSIS_ENTRY_EXIT(ChassisSpinState, ChassisState::kSpin, WriteSpinOutput)
WHEEL_LEGGED_DEFINE_CHASSIS_ENTRY_EXIT(ChassisSpecialSpinState, ChassisState::kSpecialSpin, WriteSpecialSpinOutput)
WHEEL_LEGGED_DEFINE_CHASSIS_ENTRY_EXIT(ChassisFallState, ChassisState::kFall, WriteFallOutput)
WHEEL_LEGGED_DEFINE_CHASSIS_ENTRY_EXIT(ChassisJumpState, ChassisState::kJump, WriteJumpOutput)

#undef WHEEL_LEGGED_DEFINE_CHASSIS_ENTRY_EXIT

etl::fsm_state_id_t ChassisDisableState::on_event(const ChassisUpdateEvent &event) {
  const auto &request = *event.request;
  machine().WriteDisableOutput();
  if (IsDisableRequested(request)) {
    return etl::ifsm_state::No_State_Change;
  }
  if (request.fall_detected || request.upstairs_recovery_required) {
    const auto reason = request.fall_detected ? TransitionReason::kFallDetected : TransitionReason::kRecoveryRequested;
    return machine().TransitionTo(ChassisState::kFall, reason);
  }
  return machine().TransitionTo(ResolveStableState(machine(), request), TransitionReason::kModeRequested);
}

etl::fsm_state_id_t ChassisStandbyState::on_event(const ChassisUpdateEvent &event) {
  const auto &request = *event.request;
  machine().WriteStandbyOutput();
  const auto safety = ResolveGlobalSafety(machine(), request);
  if (safety != etl::ifsm_state::No_State_Change) {
    return safety;
  }
  if (request.requested_state == ChassisState::kStandby) {
    return etl::ifsm_state::No_State_Change;
  }
  return ResolveActionOrStable(machine(), request, false);
}

etl::fsm_state_id_t ChassisNormalState::on_event(const ChassisUpdateEvent &event) {
  const auto &request = *event.request;
  machine().WriteNormalOutput();
  const auto safety = ResolveGlobalSafety(machine(), request);
  if (safety != etl::ifsm_state::No_State_Change) {
    return safety;
  }
  return ResolveActionOrStable(machine(), request, true);
}

etl::fsm_state_id_t ChassisFlyState::on_event(const ChassisUpdateEvent &event) {
  const auto &request = *event.request;
  machine().WriteFlyOutput();
  const auto safety = ResolveGlobalSafety(machine(), request);
  if (safety != etl::ifsm_state::No_State_Change) {
    return safety;
  }
  return ResolveActionOrStable(machine(), request, true);
}

etl::fsm_state_id_t ChassisUpstairsState::on_event(const ChassisUpdateEvent &event) {
  const auto &request = *event.request;
  machine().context().upstairs_mode = request.upstairs_mode;
  machine().WriteUpstairsOutput();
  const auto safety = ResolveGlobalSafety(machine(), request);
  if (safety != etl::ifsm_state::No_State_Change) {
    return safety;
  }
  if (request.upstairs_finished) {
    return machine().TransitionTo(ChassisState::kNormal, TransitionReason::kTaskFinished);
  }
  if (request.upstairs_aborted) {
    return machine().TransitionTo(ChassisState::kNormal, TransitionReason::kTaskAborted);
  }
  if (!request.upstairs_active && request.requested_state != ChassisState::kUpstairs) {
    return ResolveActionOrStable(machine(), request, false);
  }
  return etl::ifsm_state::No_State_Change;
}

etl::fsm_state_id_t ChassisSpinState::on_event(const ChassisUpdateEvent &event) {
  const auto &request = *event.request;
  machine().WriteSpinOutput();
  const auto safety = ResolveGlobalSafety(machine(), request);
  if (safety != etl::ifsm_state::No_State_Change) {
    return safety;
  }
  if (request.requested_state == ChassisState::kStandby) {
    return machine().TransitionTo(ChassisState::kStandby, TransitionReason::kModeRequested);
  }
  if (request.special_spin_hold) {
    return machine().TransitionTo(ChassisState::kSpecialSpin, TransitionReason::kModeRequested);
  }
  if (request.spin_hold) {
    if (machine().context().spin_phase != SpinPhase::kRunning) {
      machine().context().spin_phase = SpinPhase::kRunning;
      machine().EnterPhase(request.tick_ms);
    }
    return etl::ifsm_state::No_State_Change;
  }
  if (machine().context().spin_phase == SpinPhase::kRunning) {
    machine().context().spin_phase = SpinPhase::kExitPending;
    machine().context().spin_lock_low = true;
    machine().EnterPhase(request.tick_ms);
    machine().WriteSpinOutput();
  }
  if (request.spin_exit_yaw_aligned) {
    return machine().TransitionTo(ResolveStableState(machine(), request), TransitionReason::kYawAligned);
  }
  if (PhaseElapsedMs(machine(), request) >= chassis_params::kSpinExitTimeoutMs) {
    return machine().TransitionTo(ResolveStableState(machine(), request), TransitionReason::kTimeout);
  }
  return etl::ifsm_state::No_State_Change;
}

etl::fsm_state_id_t ChassisSpecialSpinState::on_event(const ChassisUpdateEvent &event) {
  const auto &request = *event.request;
  machine().WriteSpecialSpinOutput();
  const auto safety = ResolveGlobalSafety(machine(), request);
  if (safety != etl::ifsm_state::No_State_Change) {
    return safety;
  }
  if (request.special_spin_hold) {
    if (machine().context().spin_phase != SpinPhase::kRunning) {
      machine().context().spin_phase = SpinPhase::kRunning;
      machine().EnterPhase(request.tick_ms);
    }
    return etl::ifsm_state::No_State_Change;
  }
  if (request.spin_hold) {
    return machine().TransitionTo(ChassisState::kSpin, TransitionReason::kModeRequested);
  }
  if (machine().context().spin_phase == SpinPhase::kRunning) {
    machine().context().spin_phase = SpinPhase::kExitPending;
    machine().context().spin_lock_low = true;
    machine().EnterPhase(request.tick_ms);
    machine().WriteSpecialSpinOutput();
  }
  if (request.spin_exit_yaw_aligned) {
    return machine().TransitionTo(ResolveStableState(machine(), request), TransitionReason::kYawAligned);
  }
  if (PhaseElapsedMs(machine(), request) >= chassis_params::kSpinExitTimeoutMs) {
    return machine().TransitionTo(ResolveStableState(machine(), request), TransitionReason::kTimeout);
  }
  return etl::ifsm_state::No_State_Change;
}

etl::fsm_state_id_t ChassisFallState::on_event(const ChassisUpdateEvent &event) {
  const auto &request = *event.request;
  machine().WriteFallOutput();
  const auto safety = ResolveGlobalSafety(machine(), request, false);
  if (safety != etl::ifsm_state::No_State_Change) {
    return safety;
  }

  switch (machine().context().fall_phase) {
    case FallPhase::kConfirm:
      if (request.upstairs_recovery_required || request.fall_hold_ms >= chassis_params::kRecoveryFallConfirmMs) {
        machine().context().fall_phase = FallPhase::kSelfRight;
        machine().EnterPhase(request.tick_ms);
        machine().WriteFallOutput();
      } else if (!request.fall_detected) {
        return machine().TransitionTo(ResolveStableState(machine(), request), TransitionReason::kTargetReached);
      }
      break;

    case FallPhase::kSelfRight:
      if (PhaseElapsedMs(machine(), request) >= chassis_params::kRecoverySelfRightTimeoutMs) {
        return machine().TransitionTo(ChassisState::kDisable, TransitionReason::kTimeout);
      }
      if (request.upright_stable) {
        machine().context().fall_phase = FallPhase::kStandup;
        machine().EnterPhase(request.tick_ms);
        machine().WriteFallOutput();
      }
      break;

    case FallPhase::kStandup:
      if (request.standup_complete) {
        return machine().TransitionTo(ResolveStableState(machine(), request), TransitionReason::kTargetReached);
      }
      if (PhaseElapsedMs(machine(), request) >= chassis_params::kRecoverySelfRightTimeoutMs) {
        return machine().TransitionTo(ChassisState::kDisable, TransitionReason::kTimeout);
      }
      break;
  }
  return etl::ifsm_state::No_State_Change;
}

etl::fsm_state_id_t ChassisJumpState::on_event(const ChassisUpdateEvent &event) {
  const auto &request = *event.request;
  machine().WriteJumpOutput();
  const auto safety = ResolveGlobalSafety(machine(), request);
  if (safety != etl::ifsm_state::No_State_Change) {
    return safety;
  }

  switch (machine().context().jump_phase) {
    case JumpPhase::kPrepare:
      if (PhaseElapsedMs(machine(), request) >= chassis_params::kJumpLowPrepMs) {
        machine().context().jump_phase = JumpPhase::kPush;
        machine().context().jump_push_reached_armed = true;
        machine().EnterPhase(request.tick_ms);
        machine().WriteJumpOutput();
      }
      break;

    case JumpPhase::kPush: {
      if (request.current_leg_length_m >= chassis_params::kJumpLowPushReachedLegLengthM) {
        if (machine().context().jump_push_reached_armed) {
          machine().context().jump_push_reached_tick_ms = request.tick_ms;
          machine().context().jump_push_reached_armed = false;
        }
      } else {
        machine().context().jump_push_reached_armed = true;
      }
      const bool reached_held =
          !machine().context().jump_push_reached_armed &&
          request.tick_ms - machine().context().jump_push_reached_tick_ms >= chassis_params::kJumpPushReachedHoldMs;
      if (reached_held || PhaseElapsedMs(machine(), request) >= chassis_params::kJumpLowPushMaxMs) {
        machine().context().jump_phase = JumpPhase::kRecover;
        machine().EnterPhase(request.tick_ms);
        machine().WriteJumpOutput();
      }
      break;
    }

    case JumpPhase::kRecover:
      if ((PhaseElapsedMs(machine(), request) >= chassis_params::kJumpLowRecoverMinMs && !request.off_ground) ||
          PhaseElapsedMs(machine(), request) >= chassis_params::kJumpLowRecoverMs) {
        const auto reason = request.off_ground ? TransitionReason::kTimeout : TransitionReason::kLanded;
        return machine().TransitionTo(machine().context().jump_return_state, reason);
      }
      break;
  }
  return etl::ifsm_state::No_State_Change;
}

template class ChassisStateBase<ChassisDisableState, ChassisState::kDisable>;
template class ChassisStateBase<ChassisStandbyState, ChassisState::kStandby>;
template class ChassisStateBase<ChassisNormalState, ChassisState::kNormal>;
template class ChassisStateBase<ChassisFlyState, ChassisState::kFly>;
template class ChassisStateBase<ChassisUpstairsState, ChassisState::kUpstairs>;
template class ChassisStateBase<ChassisSpinState, ChassisState::kSpin>;
template class ChassisStateBase<ChassisSpecialSpinState, ChassisState::kSpecialSpin>;
template class ChassisStateBase<ChassisFallState, ChassisState::kFall>;
template class ChassisStateBase<ChassisJumpState, ChassisState::kJump>;

}  // namespace wheel_legged::fsm
