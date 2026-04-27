#include "fsm.hpp"

/**
 * @file  wl_infantry/fsm/fsm.cc
 * @brief 整车统一状态机外观实现
 */

#include <memory>

#include "fsm_internal.hpp"

using namespace wl_infantry::fsm_internal;

Fsm::Fsm() : etl_impl_(std::make_unique<EtlImpl>(*this)) {}

Fsm::~Fsm() = default;

void Fsm::Init() {
  // 上电后默认进入失能态，等待有效输入和纯遥控器语义请求。
  state_ = State::kDisabled;
  output_ = {};
  output_.state = state_;
  state_enter_tick_ms_ = 0;
  jump_exit_low_state_ = State::kDisabled;
  etl_impl_->reset(false);
  etl_impl_->start(true);
}

void Fsm::Transit(State next_state, uint32_t tick_ms) {
  // 叶子状态切换时刷新进入时刻，供跳跃/自起身等时序状态使用。
  if (next_state == state_) {
    return;
  }
  state_ = next_state;
  state_enter_tick_ms_ = tick_ms;
}

Fsm::Output Fsm::BuildOutput(const Input& input) const {
  Output out{};
  out.state = state_;

  out.chassis.chassis_enable = IsChassisEnabledState(state_);
  out.chassis.spin_mode = IsSpinState(state_);
  out.chassis.recovery_mode = IsRecoveryState(state_);

  switch (state_) {
    case State::kServiceWithFireMidLeg:
    case State::kServiceSafeMidLeg:
    case State::kCombatMidLeg:
      out.chassis.leg_profile = LegProfile::kMid;
      out.chassis.target_leg_length_m = kLegLengthMidM;
      break;
    case State::kServiceWithFireHighLeg:
    case State::kServiceSafeHighLeg:
    case State::kCombatHighLeg:
      out.chassis.leg_profile = LegProfile::kHigh;
      out.chassis.target_leg_length_m = kLegLengthHighM;
      break;
    case State::kServiceWithFireSpin:
    case State::kServiceSafeSpin:
    case State::kCombatSpin:
      out.chassis.leg_profile = input.leg_request;
      out.chassis.target_leg_length_m = TargetLegLengthForProfile(input.leg_request);
      break;
    case State::kServiceWithFireJumpPrep:
    case State::kServiceSafeJumpPrep:
    case State::kCombatJumpPrep:
      out.chassis.leg_profile = LegProfile::kLow;
      out.chassis.target_leg_length_m = kJumpPrepTargetLegM;
      break;
    case State::kServiceWithFireJumpPush:
    case State::kServiceSafeJumpPush:
    case State::kCombatJumpPush:
      out.chassis.leg_profile = LegProfile::kLow;
      out.chassis.target_leg_length_m = kJumpPushTargetLegM;
      break;
    case State::kServiceWithFireJumpRecover:
    case State::kServiceSafeJumpRecover:
    case State::kCombatJumpRecover:
      out.chassis.leg_profile = LegProfile::kLow;
      out.chassis.target_leg_length_m = kJumpRecoverTargetLegM;
      break;
    default:
      out.chassis.leg_profile = LegProfile::kLow;
      out.chassis.target_leg_length_m = kLegLengthLowM;
      break;
  }

  out.gimbal.gimbal_enable = (state_ != State::kDisabled);
  if (!out.gimbal.gimbal_enable) {
    out.gimbal.fire_allowed = false;
    out.gimbal.shoot_request = false;
    out.gimbal.align_to_chassis_forward = false;
    out.gimbal.active_target_source = TargetSource::kRc;
    out.gimbal.gimbal_target = input.rc_target;
    return out;
  }

  const bool use_host_target = input.target_source == TargetSource::kHost && input.host_target_valid;
  out.gimbal.active_target_source = use_host_target ? TargetSource::kHost : TargetSource::kRc;
  out.gimbal.gimbal_target = use_host_target ? input.host_target : input.rc_target;
  out.gimbal.align_to_chassis_forward = IsRecoveryState(state_);
  const bool fire_allowed = IsWheelFireEnabledState(state_) && !IsRecoveryState(state_);
  out.gimbal.fire_allowed = fire_allowed;
  out.gimbal.shoot_request = fire_allowed && input.fire_request;
  return out;
}

Fsm::Output Fsm::Update(const Input& input) {
  const State prev_state = state_;
  etl_impl_->Dispatch(input);
  output_ = BuildOutput(input);
  output_.state_changed = (state_ != prev_state);
  return output_;
}
