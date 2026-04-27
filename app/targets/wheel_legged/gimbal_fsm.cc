#include "include/gimbal/fsm.hpp"

namespace {

gimbal::Fsm::Output::ControlOutput BuildControlOutput(const gimbal::Fsm::State mode, const bool host_target_valid,
                                                      const bool fire_request) {
  gimbal::Fsm::Output::ControlOutput control{};

  switch (mode) {
    case gimbal::Fsm::State::kDisabled:
      control.gimbal_enable = false;
      control.align_to_chassis_forward = false;
      control.fire_allowed = false;
      control.shoot_request = false;
      control.target_source = gimbal::Fsm::TargetSource::kRemote;
      break;

    case gimbal::Fsm::State::kSafe:
      control.gimbal_enable = true;
      control.align_to_chassis_forward = false;
      control.fire_allowed = false;
      control.shoot_request = false;
      control.target_source = gimbal::Fsm::TargetSource::kRemote;
      break;

    case gimbal::Fsm::State::kManualControl:
      control.gimbal_enable = true;
      control.align_to_chassis_forward = false;
      control.fire_allowed = true;
      control.shoot_request = fire_request;
      control.target_source = gimbal::Fsm::TargetSource::kRemote;
      break;

    case gimbal::Fsm::State::kHostControl:
      control.gimbal_enable = true;
      control.align_to_chassis_forward = false;
      control.fire_allowed = true;
      control.shoot_request = fire_request;
      control.target_source = host_target_valid ? gimbal::Fsm::TargetSource::kHost : gimbal::Fsm::TargetSource::kRemote;
      break;

    case gimbal::Fsm::State::kRecoveryAlign:
      control.gimbal_enable = true;
      control.align_to_chassis_forward = true;
      control.fire_allowed = false;
      control.shoot_request = false;
      control.target_source = gimbal::Fsm::TargetSource::kRemote;
      break;
  }

  return control;
}

gimbal::Fsm::State ResolveOperationalState(const gimbal::Fsm::Input &input) {
  if (!input.enable_request) {
    return gimbal::Fsm::State::kDisabled;
  }
  if (input.chassis_recovery_active) {
    return gimbal::Fsm::State::kRecoveryAlign;
  }
  if (input.safe_request) {
    return gimbal::Fsm::State::kSafe;
  }
  if (input.host_target_valid) {
    return gimbal::Fsm::State::kHostControl;
  }
  return gimbal::Fsm::State::kManualControl;
}

}  // namespace

void gimbal::Fsm::Init() { Transit(State::kDisabled); }

void gimbal::Fsm::Transit(const State new_mode) {
  output_.state_changed = (new_mode != mode_);
  mode_ = new_mode;
  output_.mode = mode_;
}

gimbal::Fsm::Output gimbal::Fsm::Update(const Input &input) {
  output_.state_changed = false;

  if (!input.input_valid) {
    Transit(State::kDisabled);
    output_.control = BuildControlOutput(mode_, false, false);
    return output_;
  }

  const State requested = ResolveOperationalState(input);
  if (requested != mode_) {
    Transit(requested);
  }

  if (mode_ == State::kDisabled && input.enable_request) {
    Transit(State::kSafe);
  }

  output_.control = BuildControlOutput(mode_, input.host_target_valid, input.fire_request);
  return output_;
}
