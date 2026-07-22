#include "include/chassis/stair_climb_sequence.hpp"

#include "include/params.hpp"

#include <cmath>

namespace chassis {

namespace {
const auto &kParams = wheel_legged::params::active::chassis_fsm::kStairClimb;
const auto &kParamsStep2 = wheel_legged::params::active::chassis_fsm::kStairClimbStep2;

bool IsRunningPhase(const wheel_legged::StairPhase phase) {
  return phase == wheel_legged::StairPhase::kHook || phase == wheel_legged::StairPhase::kRetract ||
         phase == wheel_legged::StairPhase::kSettle;
}
}  // namespace

void StairClimbSequence::Reset() {
  phase_ = wheel_legged::StairPhase::kIdle;
  abort_reason_ = wheel_legged::StairAbortReason::kNone;
  phase_enter_tick_ms_ = 0U;
  stable_start_tick_ms_ = 0U;
  stable_active_ = false;
  settle_theta_ramp_target_ = 0.0f;
  output_ = {};

}

void StairClimbSequence::EnterPhase(const wheel_legged::StairPhase phase, const uint32_t tick_ms) {
  phase_ = phase;
  phase_enter_tick_ms_ = tick_ms;
  stable_start_tick_ms_ = tick_ms;
  stable_active_ = false;
  if (phase == wheel_legged::StairPhase::kSettle) {
    const auto &p = use_step2_params_ ? kParamsStep2 : kParams;
    settle_theta_ramp_target_ = p.retract_theta_target_rad;
  }
}

void StairClimbSequence::Abort(const wheel_legged::StairAbortReason reason, const uint32_t tick_ms) {
  abort_reason_ = reason;
  EnterPhase(wheel_legged::StairPhase::kAborted, tick_ms);
}

bool StairClimbSequence::StableFor(const bool condition, const uint32_t duration_ms, const uint32_t tick_ms) {
  if (!condition) {
    stable_active_ = false;
    output_.stable_elapsed_ms = 0U;
    return false;
  }
  if (!stable_active_) {
    stable_active_ = true;
    stable_start_tick_ms_ = tick_ms;
  }
  output_.stable_elapsed_ms = tick_ms - stable_start_tick_ms_;
  return output_.stable_elapsed_ms >= duration_ms;
}

const StairClimbSequence::Output &StairClimbSequence::Update(const Input &input) {
  use_step2_params_ = input.use_step2_params;
  if (input.start && !IsRunningPhase(phase_)) {
    abort_reason_ = wheel_legged::StairAbortReason::kNone;
    EnterPhase(wheel_legged::StairPhase::kHook, input.tick_ms);
  }

  if (IsRunningPhase(phase_)) {
    if (!input.output_enabled) {
      Abort(wheel_legged::StairAbortReason::kOutputDisabled, input.tick_ms);
    } else if (!input.posture_valid) {
      Abort(wheel_legged::StairAbortReason::kPostureInvalid, input.tick_ms);
    } else if (input.cancel) {
      Abort(wheel_legged::StairAbortReason::kCancelled, input.tick_ms);
    } else {
      const auto &p = use_step2_params_ ? kParamsStep2 : kParams;
      const uint32_t elapsed_ms = input.tick_ms - phase_enter_tick_ms_;
      switch (phase_) {
        case wheel_legged::StairPhase::kHook: {
          const bool at_target =
              std::fabs(input.theta_ll_rad - p.hook_theta_target_rad) <= p.hook_theta_tolerance_rad &&
              std::fabs(input.theta_lr_rad - p.hook_theta_target_rad) <= p.hook_theta_tolerance_rad;
          if (StableFor(at_target, p.hook_stable_ms, input.tick_ms)) {
            EnterPhase(wheel_legged::StairPhase::kRetract, input.tick_ms);
          } else if (elapsed_ms >= p.hook_timeout_ms) {
            Abort(wheel_legged::StairAbortReason::kHookTimeout, input.tick_ms);
          }
          break;
        }
        case wheel_legged::StairPhase::kRetract: {
          const bool leg_ok = std::fabs(input.mean_leg_length_m - p.retract_leg_length_m) <= p.leg_length_tolerance_m;
          if (StableFor(leg_ok, p.retract_stable_ms, input.tick_ms)) {
            EnterPhase(wheel_legged::StairPhase::kSettle, input.tick_ms);
          } else if (elapsed_ms >= p.retract_timeout_ms) {
            Abort(wheel_legged::StairAbortReason::kRetractTimeout, input.tick_ms);
          }
          break;
        }
        case wheel_legged::StairPhase::kSettle: {
          if (settle_theta_ramp_target_ > p.settle_theta_target_rad) {
            settle_theta_ramp_target_ -= p.settle_theta_ramp_step_rad;
            if (settle_theta_ramp_target_ < p.settle_theta_target_rad) {
              settle_theta_ramp_target_ = p.settle_theta_target_rad;
            }
          } else if (settle_theta_ramp_target_ < p.settle_theta_target_rad) {
            settle_theta_ramp_target_ += p.settle_theta_ramp_step_rad;
            if (settle_theta_ramp_target_ > p.settle_theta_target_rad) {
              settle_theta_ramp_target_ = p.settle_theta_target_rad;
            }
          }
          const bool ramp_done = settle_theta_ramp_target_ == p.settle_theta_target_rad;
          const bool theta_ok =
              std::fabs(input.theta_ll_rad - p.settle_theta_target_rad) <= p.settle_theta_tolerance_rad &&
              std::fabs(input.theta_lr_rad - p.settle_theta_target_rad) <= p.settle_theta_tolerance_rad;
          if (StableFor(ramp_done && theta_ok, p.settle_stable_ms, input.tick_ms)) {
            EnterPhase(wheel_legged::StairPhase::kSucceeded, input.tick_ms);
          } else if (elapsed_ms >= p.settle_timeout_ms) {
            Abort(wheel_legged::StairAbortReason::kSettleTimeout, input.tick_ms);
          }
          break;
        }
        default:
          break;
      }
    }
  }

  UpdateOutput(input);
  return output_;
}

void StairClimbSequence::UpdateOutput(const Input &input) {
  const auto &p = use_step2_params_ ? kParamsStep2 : kParams;
  output_.phase = phase_;
  output_.abort_reason = abort_reason_;
  output_.phase_elapsed_ms = input.tick_ms - phase_enter_tick_ms_;
  output_.running = IsRunningPhase(phase_);
  output_.succeeded = phase_ == wheel_legged::StairPhase::kSucceeded;
  output_.aborted = phase_ == wheel_legged::StairPhase::kAborted;
  output_.controls_motion = output_.running || output_.succeeded || output_.aborted;
  output_.target = {};
  output_.target.disable_wheel_torque = output_.running;
  output_.target.use_stair_theta_controller = output_.controls_motion;

  switch (phase_) {
    case wheel_legged::StairPhase::kHook:
      output_.target.leg_length_m = p.hook_leg_length_m;
      output_.target.theta_ll_rad = p.hook_theta_target_rad;
      output_.target.theta_lr_rad = p.hook_theta_target_rad;
      break;
    case wheel_legged::StairPhase::kRetract:
      output_.target.leg_length_m = p.retract_leg_length_m;
      output_.target.theta_ll_rad = p.retract_theta_target_rad;
      output_.target.theta_lr_rad = p.retract_theta_target_rad;
      break;
    case wheel_legged::StairPhase::kSettle:
      output_.target.leg_length_m = p.settle_leg_length_m;
      output_.target.theta_ll_rad = settle_theta_ramp_target_;
      output_.target.theta_lr_rad = settle_theta_ramp_target_;
      break;
    case wheel_legged::StairPhase::kSucceeded:
      output_.target.leg_length_m = p.settle_leg_length_m;
      output_.target.theta_ll_rad = p.settle_theta_target_rad;
      output_.target.theta_lr_rad = p.settle_theta_target_rad;
      break;
    case wheel_legged::StairPhase::kAborted:
      output_.target.leg_length_m = p.settle_leg_length_m;
      output_.target.theta_ll_rad = p.settle_theta_target_rad;
      output_.target.theta_lr_rad = p.settle_theta_target_rad;
      break;
    case wheel_legged::StairPhase::kIdle:
      break;
    default:
      output_.controls_motion = false;
      break;
  }

  output_.theta_ll_error_rad = output_.target.theta_ll_rad - input.theta_ll_rad;
  output_.theta_lr_error_rad = output_.target.theta_lr_rad - input.theta_lr_rad;
  output_.leg_length_error_m = output_.target.leg_length_m - input.mean_leg_length_m;

}

}  // namespace chassis
