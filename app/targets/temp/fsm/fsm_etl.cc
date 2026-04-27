#include "fsm_internal.hpp"

/**
 * @file  wl_infantry/fsm/fsm_etl.cc
 * @brief 主 HFSM 的 ETL 状态与上下文实现
 */

namespace wl_infantry::fsm_internal {

etl::fsm_state_id_t DisabledState::on_event(const TickEvent&) {
  if (get_fsm_context().ShouldDisable()) {
    return etl::ifsm_state::No_State_Change;
  }
  return ToEtlStateId(get_fsm_context().RequestedDriveState());
}

etl::fsm_state_id_t ServiceState::on_event(const TickEvent&) {
  if (get_fsm_context().ShouldDisable()) {
    return kDisabledState;
  }
  if (get_fsm_context().input().domain_request == Fsm::DomainRequest::kCombat) {
    return ToEtlStateId(get_fsm_context().RequestedDriveState());
  }
  if (get_fsm_context().input().service_profile == Fsm::ServiceProfile::kGimbalOnlyWithFire) {
    return kServiceGimbalOnlyWithFireState;
  }
  return etl::ifsm_state::No_State_Change;
}

etl::fsm_state_id_t ServiceGimbalOnlyWithFireState::on_event(const TickEvent&) {
  if (get_fsm_context().ShouldDisable()) {
    return kDisabledState;
  }
  const Fsm::State requested = get_fsm_context().RequestedDriveState();
  if (requested != Fsm::State::kServiceGimbalOnlyWithFire) {
    return ToEtlStateId(requested);
  }
  return etl::ifsm_state::Pass_To_Parent;
}

etl::fsm_state_id_t ServiceWithFireState::on_event(const TickEvent&) { return MotionCompositeEvent(get_fsm_context()); }

etl::fsm_state_id_t ServiceWithFireLowLegState::on_event(const TickEvent&) {
  return DriveLeafEvent(get_fsm_context(), Fsm::State::kServiceWithFireLowLeg);
}

etl::fsm_state_id_t ServiceWithFireMidLegState::on_event(const TickEvent&) {
  return MidHighLeafEvent(get_fsm_context(), Fsm::State::kServiceWithFireMidLeg);
}

etl::fsm_state_id_t ServiceWithFireHighLegState::on_event(const TickEvent&) {
  return MidHighLeafEvent(get_fsm_context(), Fsm::State::kServiceWithFireHighLeg);
}

etl::fsm_state_id_t ServiceWithFireSpinState::on_event(const TickEvent&) { return SpinLeafEvent(get_fsm_context()); }

etl::fsm_state_id_t ServiceWithFireJumpPrepState::on_event(const TickEvent&) {
  return JumpPrepLeafEvent(get_fsm_context());
}

etl::fsm_state_id_t ServiceWithFireJumpPushState::on_event(const TickEvent&) {
  return JumpPushLeafEvent(get_fsm_context());
}

etl::fsm_state_id_t ServiceWithFireJumpRecoverState::on_event(const TickEvent&) {
  return JumpRecoverLeafEvent(get_fsm_context());
}

etl::fsm_state_id_t ServiceSafeState::on_event(const TickEvent&) { return MotionCompositeEvent(get_fsm_context()); }

etl::fsm_state_id_t ServiceSafeLowLegState::on_event(const TickEvent&) {
  return DriveLeafEvent(get_fsm_context(), Fsm::State::kServiceSafeLowLeg);
}

etl::fsm_state_id_t ServiceSafeMidLegState::on_event(const TickEvent&) {
  return MidHighLeafEvent(get_fsm_context(), Fsm::State::kServiceSafeMidLeg);
}

etl::fsm_state_id_t ServiceSafeHighLegState::on_event(const TickEvent&) {
  return MidHighLeafEvent(get_fsm_context(), Fsm::State::kServiceSafeHighLeg);
}

etl::fsm_state_id_t ServiceSafeSpinState::on_event(const TickEvent&) { return SpinLeafEvent(get_fsm_context()); }

etl::fsm_state_id_t ServiceSafeJumpPrepState::on_event(const TickEvent&) {
  return JumpPrepLeafEvent(get_fsm_context());
}

etl::fsm_state_id_t ServiceSafeJumpPushState::on_event(const TickEvent&) {
  return JumpPushLeafEvent(get_fsm_context());
}

etl::fsm_state_id_t ServiceSafeJumpRecoverState::on_event(const TickEvent&) {
  return JumpRecoverLeafEvent(get_fsm_context());
}

etl::fsm_state_id_t CombatState::on_event(const TickEvent&) { return MotionCompositeEvent(get_fsm_context()); }

etl::fsm_state_id_t CombatLowLegState::on_event(const TickEvent&) {
  return DriveLeafEvent(get_fsm_context(), Fsm::State::kCombatLowLeg);
}

etl::fsm_state_id_t CombatMidLegState::on_event(const TickEvent&) {
  return MidHighLeafEvent(get_fsm_context(), Fsm::State::kCombatMidLeg);
}

etl::fsm_state_id_t CombatHighLegState::on_event(const TickEvent&) {
  return MidHighLeafEvent(get_fsm_context(), Fsm::State::kCombatHighLeg);
}

etl::fsm_state_id_t CombatSpinState::on_event(const TickEvent&) { return SpinLeafEvent(get_fsm_context()); }

etl::fsm_state_id_t CombatJumpPrepState::on_event(const TickEvent&) { return JumpPrepLeafEvent(get_fsm_context()); }

etl::fsm_state_id_t CombatJumpPushState::on_event(const TickEvent&) { return JumpPushLeafEvent(get_fsm_context()); }

etl::fsm_state_id_t CombatJumpRecoverState::on_event(const TickEvent&) {
  return JumpRecoverLeafEvent(get_fsm_context());
}

etl::fsm_state_id_t RecoveryState::on_event(const TickEvent&) {
  if (get_fsm_context().ShouldDisable()) {
    return kDisabledState;
  }
  return etl::ifsm_state::No_State_Change;
}

etl::fsm_state_id_t RecoveryFallCheckState::on_event(const TickEvent&) {
  if (get_fsm_context().ShouldDisable()) {
    return kDisabledState;
  }
  if (get_fsm_context().input().fall_detected_hold_ms >= kFallConfirmMs) {
    return kRecoverySelfRightState;
  }
  if (!get_fsm_context().input().fall_detected) {
    return ToEtlStateId(get_fsm_context().RequestedLowLegState());
  }
  return etl::ifsm_state::No_State_Change;
}

etl::fsm_state_id_t RecoverySelfRightState::on_event(const TickEvent&) {
  if (get_fsm_context().ShouldDisable()) {
    return kDisabledState;
  }
  if (get_fsm_context().ElapsedMs() >= kSelfRightTimeoutMs) {
    return kDisabledState;
  }
  if (get_fsm_context().input().upright_stable) {
    return ToEtlStateId(get_fsm_context().RequestedLowLegState());
  }
  return etl::ifsm_state::No_State_Change;
}

}  // namespace wl_infantry::fsm_internal

Fsm::EtlImpl::EtlImpl(Fsm& owner) : etl::hfsm(0x57), owner_(owner) {
  using namespace wl_infantry::fsm_internal;

  states_[kDisabledState] = &disabled_;
  states_[kServiceState] = &service_;
  states_[kServiceGimbalOnlyWithFireState] = &service_gimbal_only_with_fire_;
  states_[kServiceWithFireState] = &service_with_fire_;
  states_[kServiceWithFireLowLegState] = &service_with_fire_low_leg_;
  states_[kServiceWithFireMidLegState] = &service_with_fire_mid_leg_;
  states_[kServiceWithFireHighLegState] = &service_with_fire_high_leg_;
  states_[kServiceWithFireSpinState] = &service_with_fire_spin_;
  states_[kServiceWithFireJumpPrepState] = &service_with_fire_jump_prep_;
  states_[kServiceWithFireJumpPushState] = &service_with_fire_jump_push_;
  states_[kServiceWithFireJumpRecoverState] = &service_with_fire_jump_recover_;
  states_[kServiceSafeState] = &service_safe_;
  states_[kServiceSafeLowLegState] = &service_safe_low_leg_;
  states_[kServiceSafeMidLegState] = &service_safe_mid_leg_;
  states_[kServiceSafeHighLegState] = &service_safe_high_leg_;
  states_[kServiceSafeSpinState] = &service_safe_spin_;
  states_[kServiceSafeJumpPrepState] = &service_safe_jump_prep_;
  states_[kServiceSafeJumpPushState] = &service_safe_jump_push_;
  states_[kServiceSafeJumpRecoverState] = &service_safe_jump_recover_;
  states_[kCombatState] = &combat_;
  states_[kCombatLowLegState] = &combat_low_leg_;
  states_[kCombatMidLegState] = &combat_mid_leg_;
  states_[kCombatHighLegState] = &combat_high_leg_;
  states_[kCombatSpinState] = &combat_spin_;
  states_[kCombatJumpPrepState] = &combat_jump_prep_;
  states_[kCombatJumpPushState] = &combat_jump_push_;
  states_[kCombatJumpRecoverState] = &combat_jump_recover_;
  states_[kRecoveryState] = &recovery_;
  states_[kRecoveryFallCheckState] = &recovery_fall_check_;
  states_[kRecoverySelfRightState] = &recovery_self_right_;
  set_states(states_.data(), states_.size());

  etl::ifsm_state* service_children[] = {&service_gimbal_only_with_fire_, &service_with_fire_, &service_safe_};
  service_.set_child_states(service_children, 3U);

  etl::ifsm_state* service_with_fire_children[] = {&service_with_fire_low_leg_,     &service_with_fire_mid_leg_,
                                                   &service_with_fire_high_leg_,    &service_with_fire_spin_,
                                                   &service_with_fire_jump_prep_,   &service_with_fire_jump_push_,
                                                   &service_with_fire_jump_recover_};
  service_with_fire_.set_child_states(service_with_fire_children, 7U);

  etl::ifsm_state* service_safe_children[] = {
      &service_safe_low_leg_,   &service_safe_mid_leg_,   &service_safe_high_leg_,    &service_safe_spin_,
      &service_safe_jump_prep_, &service_safe_jump_push_, &service_safe_jump_recover_};
  service_safe_.set_child_states(service_safe_children, 7U);

  etl::ifsm_state* combat_children[] = {&combat_low_leg_,   &combat_mid_leg_,   &combat_high_leg_,    &combat_spin_,
                                        &combat_jump_prep_, &combat_jump_push_, &combat_jump_recover_};
  combat_.set_child_states(combat_children, 7U);

  etl::ifsm_state* recovery_children[] = {&recovery_fall_check_, &recovery_self_right_};
  recovery_.set_child_states(recovery_children, 2U);
}

void Fsm::EtlImpl::Dispatch(const Input& input) {
  input_ = input;
  receive(wl_infantry::fsm_internal::TickEvent(input_));
}

void Fsm::EtlImpl::EnterLeaf(State state) { owner_.Transit(state, input_.tick_ms); }

const Fsm::Input& Fsm::EtlImpl::input() const { return input_; }

bool Fsm::EtlImpl::ShouldDisable() const {
  return !input_.input_valid || input_.domain_request == DomainRequest::kDisabled;
}

Fsm::State Fsm::EtlImpl::RequestedDriveState() const {
  return wl_infantry::fsm_internal::ResolveRequestedDriveState(input_);
}

Fsm::State Fsm::EtlImpl::RequestedLowLegState() const {
  return wl_infantry::fsm_internal::ResolveRequestedLowLegState(input_);
}

uint32_t Fsm::EtlImpl::ElapsedMs() const { return input_.tick_ms - owner_.state_enter_tick_ms_; }

Fsm::State Fsm::EtlImpl::CurrentLeafState() const { return owner_.state_; }

void Fsm::EtlImpl::SetJumpExitLowState(State state) { owner_.jump_exit_low_state_ = state; }

Fsm::State Fsm::EtlImpl::JumpExitLowState() const { return owner_.jump_exit_low_state_; }
