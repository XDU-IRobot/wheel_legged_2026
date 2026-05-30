#include "include/chassis/stair_task_coordinator.hpp"

namespace chassis {

bool StairTaskCoordinator::IsActive() const {
  return mode_ == wheel_legged::StairTaskMode::kArmed || mode_ == wheel_legged::StairTaskMode::kExecuting ||
         mode_ == wheel_legged::StairTaskMode::kBetweenSteps;
}

void StairTaskCoordinator::Reset() {
  mode_ = wheel_legged::StairTaskMode::kIdle;
  abort_reason_ = wheel_legged::StairAbortReason::kNone;
  requested_attempts_ = 0U;
  completed_attempts_ = 0U;
  continuous_ = false;
  hold_low_until_release_ = false;
  recovery_required_ = false;
  contact_released_ = true;
  output_ = {};
}

void StairTaskCoordinator::Arm(const uint8_t attempts, const bool continuous) {
  mode_ = wheel_legged::StairTaskMode::kArmed;
  abort_reason_ = wheel_legged::StairAbortReason::kNone;
  requested_attempts_ = attempts;
  completed_attempts_ = 0U;
  continuous_ = continuous;
  recovery_required_ = false;
  contact_released_ = true;
}

void StairTaskCoordinator::Abort(const wheel_legged::StairAbortReason reason, const bool recovery_required) {
  if (continuous_) {
    hold_low_until_release_ = true;
  }
  mode_ = wheel_legged::StairTaskMode::kAborted;
  abort_reason_ = reason;
  recovery_required_ = recovery_required;
}

const StairTaskCoordinator::Output &StairTaskCoordinator::Update(const Input &input) {
  output_.start_sequence = false;
  output_.cancel_sequence = false;
  output_.reset_sequence = false;

  if (input.request != wheel_legged::StairTaskRequest::kArmContinuous) {
    hold_low_until_release_ = false;
  }

  if (input.request == wheel_legged::StairTaskRequest::kCancel && IsActive()) {
    output_.cancel_sequence = mode_ == wheel_legged::StairTaskMode::kExecuting;
    Abort(wheel_legged::StairAbortReason::kCancelled, false);
  } else if ((input.request == wheel_legged::StairTaskRequest::kArmSingle ||
              input.request == wheel_legged::StairTaskRequest::kArmDouble) &&
             IsActive()) {
    output_.cancel_sequence = mode_ == wheel_legged::StairTaskMode::kExecuting;
    Abort(wheel_legged::StairAbortReason::kCancelled, false);
  } else if (input.request == wheel_legged::StairTaskRequest::kArmSingle) {
    Arm(1U, false);
    output_.reset_sequence = true;
  } else if (input.request == wheel_legged::StairTaskRequest::kArmDouble) {
    Arm(2U, false);
    output_.reset_sequence = true;
  } else if (input.request == wheel_legged::StairTaskRequest::kArmContinuous && !IsActive() &&
             !hold_low_until_release_) {
    Arm(0U, true);
    output_.reset_sequence = true;
  } else if (continuous_ && IsActive() && input.request != wheel_legged::StairTaskRequest::kArmContinuous) {
    output_.cancel_sequence = mode_ == wheel_legged::StairTaskMode::kExecuting;
    Abort(wheel_legged::StairAbortReason::kCancelled, false);
  }

  if (IsActive() && !input.output_enabled) {
    output_.cancel_sequence = mode_ == wheel_legged::StairTaskMode::kExecuting;
    Abort(wheel_legged::StairAbortReason::kOutputDisabled, false);
  } else if (IsActive() && !input.posture_valid) {
    output_.cancel_sequence = mode_ == wheel_legged::StairTaskMode::kExecuting;
    Abort(wheel_legged::StairAbortReason::kPostureInvalid, true);
  }

  switch (mode_) {
    case wheel_legged::StairTaskMode::kArmed:
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
      if (input.high_leg_ready && input.contact_detected) {
#else
      if (contact_released_ && input.high_leg_ready && input.contact_detected) {
#endif
        mode_ = wheel_legged::StairTaskMode::kExecuting;
        output_.start_sequence = true;
        contact_released_ = false;
      }
      break;
    case wheel_legged::StairTaskMode::kExecuting:
      if (input.sequence_aborted) {
        Abort(input.sequence_abort_reason,
              input.sequence_abort_reason == wheel_legged::StairAbortReason::kPostureInvalid);
      } else if (input.sequence_succeeded) {
        ++completed_attempts_;
        output_.reset_sequence = true;
        if (continuous_ || completed_attempts_ < requested_attempts_) {
          mode_ = wheel_legged::StairTaskMode::kBetweenSteps;
        } else {
          mode_ = wheel_legged::StairTaskMode::kSucceeded;
        }
      }
      break;
    case wheel_legged::StairTaskMode::kBetweenSteps:
      if (!input.contact_detected) {
        contact_released_ = true;
        mode_ = wheel_legged::StairTaskMode::kArmed;
      }
      break;
    case wheel_legged::StairTaskMode::kSucceeded:
    case wheel_legged::StairTaskMode::kAborted:
      mode_ = wheel_legged::StairTaskMode::kIdle;
      continuous_ = false;
      output_.reset_sequence = true;
      break;
    case wheel_legged::StairTaskMode::kIdle:
    default:
      break;
  }

  RefreshOutput();
  return output_;
}

void StairTaskCoordinator::RefreshOutput() {
  output_.mode = mode_;
  output_.abort_reason = abort_reason_;
  output_.requested_attempts = requested_attempts_;
  output_.completed_attempts = completed_attempts_;
  output_.task_active = IsActive();
  output_.request_high_leg =
      mode_ == wheel_legged::StairTaskMode::kArmed || mode_ == wheel_legged::StairTaskMode::kBetweenSteps;
  output_.force_low_leg = mode_ == wheel_legged::StairTaskMode::kSucceeded ||
                          mode_ == wheel_legged::StairTaskMode::kAborted || hold_low_until_release_;
  output_.recovery_required = recovery_required_;
}

}  // namespace chassis
