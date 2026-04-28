#include "include/gimbal/fsm.hpp"

/**
 * @file  targets/wheel_legged/gimbal_fsm.cc
 * @brief 云台状态机实现
 */

namespace {

gimbal::Fsm::State ResolveMode(const gimbal::Fsm::Input &input) {
  const wheel_legged::ModeRequest &request = input.request;

  if (!request.input_valid || request.domain_request == wheel_legged::DomainRequest::kDisabled) {
    return gimbal::Fsm::State::kDisabled;
  }
  if (input.chassis_recovery_active) {
    return gimbal::Fsm::State::kRecoveryAlign;
  }
  if (request.domain_request == wheel_legged::DomainRequest::kCombat) {
    return gimbal::Fsm::State::kCombat;
  }
  return request.service_profile == wheel_legged::ServiceProfile::kChassisAndGimbalSafe
             ? gimbal::Fsm::State::kServiceSafe
             : gimbal::Fsm::State::kServiceWithFire;
}

gimbal::Fsm::Output::ControlOutput BuildControlOutput(const gimbal::Fsm::Input &input, const gimbal::Fsm::State mode) {
  gimbal::Fsm::Output::ControlOutput control{};
  const wheel_legged::ModeRequest &request = input.request;

  const bool use_host_target = request.target_source == wheel_legged::TargetSource::kHost && request.host_target_valid;
  control.active_target_source = use_host_target ? wheel_legged::TargetSource::kHost : wheel_legged::TargetSource::kRc;
  control.gimbal_target = use_host_target ? request.host_target : request.rc_target;

  switch (mode) {
    case gimbal::Fsm::State::kDisabled:
      control.gimbal_enable = false;
      control.fire_allowed = false;
      control.shoot_request = false;
      control.align_to_chassis_forward = false;
      break;

    case gimbal::Fsm::State::kServiceWithFire:
      control.gimbal_enable = true;
      control.fire_allowed = true;
      control.shoot_request = request.fire_request;
      control.align_to_chassis_forward = false;
      break;

    case gimbal::Fsm::State::kServiceSafe:
      control.gimbal_enable = true;
      control.fire_allowed = false;
      control.shoot_request = false;
      control.align_to_chassis_forward = false;
      break;

    case gimbal::Fsm::State::kCombat:
      control.gimbal_enable = true;
      control.fire_allowed = true;
      control.shoot_request = request.fire_request;
      control.align_to_chassis_forward = false;
      break;

    case gimbal::Fsm::State::kRecoveryAlign:
      control.gimbal_enable = true;
      control.fire_allowed = false;
      control.shoot_request = false;
      control.align_to_chassis_forward = true;
      break;
  }

  return control;
}

}  // namespace

void gimbal::Fsm::Init() {
  mode_ = State::kDisabled;
  output_ = {};
  output_.mode = mode_;
  output_.control = BuildControlOutput(Input{}, mode_);
}

void gimbal::Fsm::Transit(const State new_mode) {
  output_.state_changed = (new_mode != mode_);
  mode_ = new_mode;
  output_.mode = mode_;
}

gimbal::Fsm::Output gimbal::Fsm::Update(const Input &input) {
  output_.state_changed = false;

  const State requested = ResolveMode(input);
  if (requested != mode_) {
    Transit(requested);
  }

  output_.control = BuildControlOutput(input, mode_);
  return output_;
}
