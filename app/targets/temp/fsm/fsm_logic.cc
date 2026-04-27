#include "fsm_internal.hpp"

/**
 * @file  wl_infantry/fsm/fsm_logic.cc
 * @brief 主 HFSM 的纯逻辑辅助函数实现
 */

namespace wl_infantry::fsm_internal {

float TargetLegLengthForProfile(Fsm::LegProfile profile) {
  switch (profile) {
    case Fsm::LegProfile::kMid:
      return kLegLengthMidM;
    case Fsm::LegProfile::kHigh:
      return kLegLengthHighM;
    case Fsm::LegProfile::kLow:
    default:
      return kLegLengthLowM;
  }
}

Fsm::State ResolveRequestedDriveState(const Fsm::Input& input) {
  switch (input.domain_request) {
    case Fsm::DomainRequest::kService:
      if (input.service_profile == Fsm::ServiceProfile::kGimbalOnlyWithFire) {
        return Fsm::State::kServiceGimbalOnlyWithFire;
      }
      if (input.service_profile == Fsm::ServiceProfile::kChassisAndGimbalSafe) {
        switch (input.leg_request) {
          case Fsm::LegProfile::kMid:
            return Fsm::State::kServiceSafeMidLeg;
          case Fsm::LegProfile::kHigh:
            return Fsm::State::kServiceSafeHighLeg;
          case Fsm::LegProfile::kLow:
          default:
            return Fsm::State::kServiceSafeLowLeg;
        }
      }
      switch (input.leg_request) {
        case Fsm::LegProfile::kMid:
          return Fsm::State::kServiceWithFireMidLeg;
        case Fsm::LegProfile::kHigh:
          return Fsm::State::kServiceWithFireHighLeg;
        case Fsm::LegProfile::kLow:
        default:
          return Fsm::State::kServiceWithFireLowLeg;
      }

    case Fsm::DomainRequest::kCombat:
      switch (input.leg_request) {
        case Fsm::LegProfile::kMid:
          return Fsm::State::kCombatMidLeg;
        case Fsm::LegProfile::kHigh:
          return Fsm::State::kCombatHighLeg;
        case Fsm::LegProfile::kLow:
        default:
          return Fsm::State::kCombatLowLeg;
      }

    case Fsm::DomainRequest::kDisabled:
    default:
      return Fsm::State::kDisabled;
  }
}

Fsm::State ResolveRequestedLowLegState(const Fsm::Input& input) {
  switch (input.domain_request) {
    case Fsm::DomainRequest::kService:
      if (input.service_profile == Fsm::ServiceProfile::kGimbalOnlyWithFire) {
        return Fsm::State::kServiceGimbalOnlyWithFire;
      }
      return (input.service_profile == Fsm::ServiceProfile::kChassisAndGimbalSafe) ? Fsm::State::kServiceSafeLowLeg
                                                                                   : Fsm::State::kServiceWithFireLowLeg;
    case Fsm::DomainRequest::kCombat:
      return Fsm::State::kCombatLowLeg;
    case Fsm::DomainRequest::kDisabled:
    default:
      return Fsm::State::kDisabled;
  }
}

bool IsWheelFireEnabledState(Fsm::State state) {
  switch (state) {
    case Fsm::State::kServiceGimbalOnlyWithFire:
    case Fsm::State::kServiceWithFireLowLeg:
    case Fsm::State::kServiceWithFireMidLeg:
    case Fsm::State::kServiceWithFireHighLeg:
    case Fsm::State::kServiceWithFireSpin:
    case Fsm::State::kServiceWithFireJumpPrep:
    case Fsm::State::kServiceWithFireJumpPush:
    case Fsm::State::kServiceWithFireJumpRecover:
    case Fsm::State::kCombatLowLeg:
    case Fsm::State::kCombatMidLeg:
    case Fsm::State::kCombatHighLeg:
    case Fsm::State::kCombatSpin:
    case Fsm::State::kCombatJumpPrep:
    case Fsm::State::kCombatJumpPush:
    case Fsm::State::kCombatJumpRecover:
      return true;
    default:
      return false;
  }
}

bool IsChassisEnabledState(Fsm::State state) {
  return !(state == Fsm::State::kDisabled || state == Fsm::State::kServiceGimbalOnlyWithFire);
}

bool IsSpinState(Fsm::State state) {
  return state == Fsm::State::kServiceWithFireSpin || state == Fsm::State::kServiceSafeSpin ||
         state == Fsm::State::kCombatSpin;
}

bool IsJumpState(Fsm::State state) {
  switch (state) {
    case Fsm::State::kServiceWithFireJumpPrep:
    case Fsm::State::kServiceWithFireJumpPush:
    case Fsm::State::kServiceWithFireJumpRecover:
    case Fsm::State::kServiceSafeJumpPrep:
    case Fsm::State::kServiceSafeJumpPush:
    case Fsm::State::kServiceSafeJumpRecover:
    case Fsm::State::kCombatJumpPrep:
    case Fsm::State::kCombatJumpPush:
    case Fsm::State::kCombatJumpRecover:
      return true;
    default:
      return false;
  }
}

bool IsRecoveryState(Fsm::State state) {
  return state == Fsm::State::kRecoveryFallCheck || state == Fsm::State::kRecoverySelfRight;
}

Fsm::State JumpPrepStateForLowState(Fsm::State low_state) {
  switch (low_state) {
    case Fsm::State::kServiceWithFireLowLeg:
      return Fsm::State::kServiceWithFireJumpPrep;
    case Fsm::State::kServiceSafeLowLeg:
      return Fsm::State::kServiceSafeJumpPrep;
    case Fsm::State::kCombatLowLeg:
    default:
      return Fsm::State::kCombatJumpPrep;
  }
}

Fsm::State JumpPushStateForPrep(Fsm::State prep_state) {
  switch (prep_state) {
    case Fsm::State::kServiceWithFireJumpPrep:
      return Fsm::State::kServiceWithFireJumpPush;
    case Fsm::State::kServiceSafeJumpPrep:
      return Fsm::State::kServiceSafeJumpPush;
    case Fsm::State::kCombatJumpPrep:
    default:
      return Fsm::State::kCombatJumpPush;
  }
}

Fsm::State JumpRecoverStateForPush(Fsm::State push_state) {
  switch (push_state) {
    case Fsm::State::kServiceWithFireJumpPush:
      return Fsm::State::kServiceWithFireJumpRecover;
    case Fsm::State::kServiceSafeJumpPush:
      return Fsm::State::kServiceSafeJumpRecover;
    case Fsm::State::kCombatJumpPush:
    default:
      return Fsm::State::kCombatJumpRecover;
  }
}

Fsm::State SpinStateForDrive(Fsm::State drive_state) {
  switch (drive_state) {
    case Fsm::State::kServiceWithFireLowLeg:
    case Fsm::State::kServiceWithFireMidLeg:
    case Fsm::State::kServiceWithFireHighLeg:
      return Fsm::State::kServiceWithFireSpin;
    case Fsm::State::kServiceSafeLowLeg:
    case Fsm::State::kServiceSafeMidLeg:
    case Fsm::State::kServiceSafeHighLeg:
      return Fsm::State::kServiceSafeSpin;
    case Fsm::State::kCombatLowLeg:
    case Fsm::State::kCombatMidLeg:
    case Fsm::State::kCombatHighLeg:
    default:
      return Fsm::State::kCombatSpin;
  }
}

etl::fsm_state_id_t ToEtlStateId(Fsm::State state) {
  switch (state) {
    case Fsm::State::kDisabled:
      return kDisabledState;
    case Fsm::State::kServiceGimbalOnlyWithFire:
      return kServiceGimbalOnlyWithFireState;
    case Fsm::State::kServiceWithFireLowLeg:
      return kServiceWithFireLowLegState;
    case Fsm::State::kServiceWithFireMidLeg:
      return kServiceWithFireMidLegState;
    case Fsm::State::kServiceWithFireHighLeg:
      return kServiceWithFireHighLegState;
    case Fsm::State::kServiceWithFireSpin:
      return kServiceWithFireSpinState;
    case Fsm::State::kServiceWithFireJumpPrep:
      return kServiceWithFireJumpPrepState;
    case Fsm::State::kServiceWithFireJumpPush:
      return kServiceWithFireJumpPushState;
    case Fsm::State::kServiceWithFireJumpRecover:
      return kServiceWithFireJumpRecoverState;
    case Fsm::State::kServiceSafeLowLeg:
      return kServiceSafeLowLegState;
    case Fsm::State::kServiceSafeMidLeg:
      return kServiceSafeMidLegState;
    case Fsm::State::kServiceSafeHighLeg:
      return kServiceSafeHighLegState;
    case Fsm::State::kServiceSafeSpin:
      return kServiceSafeSpinState;
    case Fsm::State::kServiceSafeJumpPrep:
      return kServiceSafeJumpPrepState;
    case Fsm::State::kServiceSafeJumpPush:
      return kServiceSafeJumpPushState;
    case Fsm::State::kServiceSafeJumpRecover:
      return kServiceSafeJumpRecoverState;
    case Fsm::State::kCombatLowLeg:
      return kCombatLowLegState;
    case Fsm::State::kCombatMidLeg:
      return kCombatMidLegState;
    case Fsm::State::kCombatHighLeg:
      return kCombatHighLegState;
    case Fsm::State::kCombatSpin:
      return kCombatSpinState;
    case Fsm::State::kCombatJumpPrep:
      return kCombatJumpPrepState;
    case Fsm::State::kCombatJumpPush:
      return kCombatJumpPushState;
    case Fsm::State::kCombatJumpRecover:
      return kCombatJumpRecoverState;
    case Fsm::State::kRecoveryFallCheck:
      return kRecoveryFallCheckState;
    case Fsm::State::kRecoverySelfRight:
      return kRecoverySelfRightState;
    default:
      return kDisabledState;
  }
}

etl::fsm_state_id_t DriveLeafEvent(Fsm::EtlImpl& context, Fsm::State self_state) {
  if (context.ShouldDisable()) {
    return kDisabledState;
  }

  if (context.input().jump_trigger && context.input().leg_request == Fsm::LegProfile::kLow) {
    context.SetJumpExitLowState(context.RequestedLowLegState());
    return ToEtlStateId(JumpPrepStateForLowState(context.RequestedLowLegState()));
  }

  if (context.input().spin_hold) {
    return ToEtlStateId(SpinStateForDrive(self_state));
  }

  const Fsm::State requested = context.RequestedDriveState();
  if (requested != self_state) {
    return ToEtlStateId(requested);
  }

  return etl::ifsm_state::Pass_To_Parent;
}

etl::fsm_state_id_t MidHighLeafEvent(Fsm::EtlImpl& context, Fsm::State self_state) {
  if (context.ShouldDisable()) {
    return kDisabledState;
  }

  if (context.input().spin_hold) {
    return ToEtlStateId(SpinStateForDrive(self_state));
  }

  const Fsm::State requested = context.RequestedDriveState();
  if (requested != self_state) {
    return ToEtlStateId(requested);
  }

  return etl::ifsm_state::Pass_To_Parent;
}

etl::fsm_state_id_t SpinLeafEvent(Fsm::EtlImpl& context) {
  if (context.ShouldDisable()) {
    return kDisabledState;
  }

  if (!context.input().spin_hold) {
    return ToEtlStateId(context.RequestedDriveState());
  }

  return etl::ifsm_state::Pass_To_Parent;
}

etl::fsm_state_id_t JumpPrepLeafEvent(Fsm::EtlImpl& context) {
  if (context.ShouldDisable()) {
    return kDisabledState;
  }
  if (context.ElapsedMs() >= kJumpPrepDurationMs) {
    return ToEtlStateId(JumpPushStateForPrep(context.CurrentLeafState()));
  }
  return etl::ifsm_state::No_State_Change;
}

etl::fsm_state_id_t JumpPushLeafEvent(Fsm::EtlImpl& context) {
  if (context.ShouldDisable()) {
    return kDisabledState;
  }
  if (context.input().current_leg_length_m >= kJumpPushLegReachedM || context.ElapsedMs() >= kJumpPushMaxDurationMs) {
    return ToEtlStateId(JumpRecoverStateForPush(context.CurrentLeafState()));
  }
  return etl::ifsm_state::No_State_Change;
}

etl::fsm_state_id_t JumpRecoverLeafEvent(Fsm::EtlImpl& context) {
  if (context.ShouldDisable()) {
    return kDisabledState;
  }
  if (context.ElapsedMs() >= kJumpRecoverDurationMs) {
    return ToEtlStateId(context.JumpExitLowState());
  }
  return etl::ifsm_state::No_State_Change;
}

etl::fsm_state_id_t MotionCompositeEvent(Fsm::EtlImpl& context) {
  if (context.ShouldDisable()) {
    return kDisabledState;
  }
  if (!IsJumpState(context.CurrentLeafState()) && context.input().fall_detected) {
    return kRecoveryFallCheckState;
  }
  return etl::ifsm_state::Pass_To_Parent;
}

}  // namespace wl_infantry::fsm_internal
