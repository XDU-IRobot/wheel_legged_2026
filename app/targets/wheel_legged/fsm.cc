#include "include/chassis/fsm.hpp"
#include "include/wheel_legged_params.hpp"
#include <cmath>

/**
 * @file  targets/wheel_legged/fsm.cc
 * @brief 底盘状态机实现
 */

namespace {

/**
 * @brief 判断当前模式是否属于跳跃流程
 */
bool IsJumpState(const chassis::Fsm::State state) {
  return state == chassis::Fsm::State::kJumpPrep || state == chassis::Fsm::State::kJumpPush ||
         state == chassis::Fsm::State::kJumpRecover;
}

bool IsStairClimbReadyToDone(const wheel_legged::ModeRequest &request) {
  const float leg_length_error =
      std::fabs(request.current_leg_length_m - wheel_legged::params::active::chassis_fsm::kStairClimbLegLengthM);
  const bool leg_length_near_target =
      leg_length_error <= wheel_legged::params::active::chassis_fsm::kStairClimbLegLengthNearTargetToleranceM;
  const bool left_theta_near_zero =
      std::fabs(request.theta_ll_rad) <= wheel_legged::params::active::chassis_fsm::kStairClimbThetaNearZeroThresholdRad;
  const bool right_theta_near_zero =
      std::fabs(request.theta_lr_rad) <= wheel_legged::params::active::chassis_fsm::kStairClimbThetaNearZeroThresholdRad;
  return leg_length_near_target && left_theta_near_zero && right_theta_near_zero;
}

/**
 * @brief 将语义腿长档位转换为物理目标腿长
 */
float LegLengthForProfile(const wheel_legged::LegProfile profile) {
  switch (profile) {
    case wheel_legged::LegProfile::kMid:
      return wheel_legged::params::active::chassis_fsm::kMidLegLengthM;
    case wheel_legged::LegProfile::kHigh:
      return wheel_legged::params::active::chassis_fsm::kHighLegLengthM;
    case wheel_legged::LegProfile::kLow:
    default:
      return wheel_legged::params::active::chassis_fsm::kLowLegLengthM;
  }
}

/**
 * @brief 规范化请求中的腿长档位
 */
wheel_legged::LegProfile ResolveRequestedLegProfile(const wheel_legged::ModeRequest &request) {
  switch (request.leg_request) {
    case wheel_legged::LegProfile::kMid:
      return wheel_legged::LegProfile::kMid;
    case wheel_legged::LegProfile::kHigh:
      return wheel_legged::LegProfile::kHigh;
    case wheel_legged::LegProfile::kLow:
    default:
      return wheel_legged::LegProfile::kLow;
  }
}

/**
 * @brief 将腿长档位映射为常规底盘模式
 */
chassis::Fsm::State ResolveRequestedNormalState(const wheel_legged::LegProfile profile) {
  switch (profile) {
    case wheel_legged::LegProfile::kMid:
      return chassis::Fsm::State::kMidLeg;
    case wheel_legged::LegProfile::kHigh:
      return chassis::Fsm::State::kHighLeg;
    case wheel_legged::LegProfile::kLow:
    default:
      return chassis::Fsm::State::kLowLeg;
  }
}

/**
 * @brief 根据状态机构造底盘控制动作
 */
chassis::Fsm::Output::ControlOutput BuildControlOutput(const chassis::Fsm::State state,
                                                       const wheel_legged::LegProfile requested_leg_profile) {
  chassis::Fsm::Output::ControlOutput control{};
  control.leg_profile = wheel_legged::LegProfile::kLow;
  control.target_leg_length_m = wheel_legged::params::active::chassis_fsm::kLowLegLengthM;

  switch (state) {
    case chassis::Fsm::State::kDisabled:
      control.enable_dm = false;
      control.run_chassis_update = false;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = true;
      control.jump_phase = 0U;
      break;

    case chassis::Fsm::State::kLowLeg:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.leg_profile = wheel_legged::LegProfile::kLow;
      control.target_leg_length_m = wheel_legged::params::active::chassis_fsm::kLowLegLengthM;
      control.jump_phase = 0U;
      break;

    case chassis::Fsm::State::kMidLeg:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.leg_profile = wheel_legged::LegProfile::kMid;
      control.target_leg_length_m = wheel_legged::params::active::chassis_fsm::kMidLegLengthM;
      control.jump_phase = 0U;
      break;

    case chassis::Fsm::State::kHighLeg:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.leg_profile = wheel_legged::LegProfile::kHigh;
      control.target_leg_length_m = wheel_legged::params::active::chassis_fsm::kHighLegLengthM;
      control.jump_phase = 0U;
      break;

    case chassis::Fsm::State::kSpin:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = true;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.leg_profile = requested_leg_profile;
      control.target_leg_length_m = LegLengthForProfile(requested_leg_profile);
      control.jump_phase = 0U;
      break;

    case chassis::Fsm::State::kJumpPrep:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.leg_profile = wheel_legged::LegProfile::kLow;
      control.target_leg_length_m = wheel_legged::params::active::chassis_fsm::kJumpPrepLegLengthM;
      control.jump_phase = 1U;
      break;

    case chassis::Fsm::State::kJumpPush:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.leg_profile = wheel_legged::LegProfile::kLow;
      control.target_leg_length_m = wheel_legged::params::active::chassis_fsm::kJumpPushLegLengthM;
      control.jump_phase = 2U;
      break;

    case chassis::Fsm::State::kJumpRecover:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.leg_profile = wheel_legged::LegProfile::kLow;
      control.target_leg_length_m = wheel_legged::params::active::chassis_fsm::kJumpRecoverLegLengthM;
      control.jump_phase = 3U;
      break;

    case chassis::Fsm::State::kRecoveryFallCheck:
    case chassis::Fsm::State::kRecoverySelfRight:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = true;
      control.safe_output_required = false;
      control.leg_profile = wheel_legged::LegProfile::kLow;
      control.target_leg_length_m = wheel_legged::params::active::chassis_fsm::kLowLegLengthM;
      control.jump_phase = 0U;
      break;

    case chassis::Fsm::State::kStairClimb:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.leg_profile = wheel_legged::LegProfile::kHigh;
      control.target_leg_length_m = wheel_legged::params::active::chassis_fsm::kStairClimbLegLengthM;
      control.theta_leg_target_rad = wheel_legged::params::active::chassis_fsm::kStairClimbThetaTargetRad;
      control.jump_phase = 0U;
      break;

    case chassis::Fsm::State::kStairClimbDone:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.leg_profile = wheel_legged::LegProfile::kHigh;
      control.target_leg_length_m = wheel_legged::params::active::chassis_fsm::kStairClimbLegLengthM;
      control.theta_leg_target_rad = 0.0f;
      control.jump_phase = 0U;
      break;
  }

  return control;
}

}  // namespace

chassis::Fsm::~Fsm() = default;

void chassis::Fsm::Init() {
  mode_ = State::kDisabled;
  requested_leg_profile_ = wheel_legged::LegProfile::kLow;
  state_enter_tick_ms_ = 0U;
  output_ = {};
  output_.mode = mode_;
  output_.control = BuildControlOutput(mode_, requested_leg_profile_);
}

void chassis::Fsm::Transit(const State new_mode) {
  output_.state_changed = (new_mode != mode_);
  mode_ = new_mode;
  output_.mode = mode_;
  output_.control = BuildControlOutput(mode_, requested_leg_profile_);
}

chassis::Fsm::Output chassis::Fsm::Update(const Input &input) {
  output_.state_changed = false;

  const wheel_legged::ModeRequest &request = input.request;
  requested_leg_profile_ = ResolveRequestedLegProfile(request);

  // 输入失效或请求关闭时立即退回 Disabled，并由执行器层输出零力矩。
  if (!request.input_valid || request.domain_request == wheel_legged::DomainRequest::kDisabled) {
    if (mode_ != State::kDisabled) {
      state_enter_tick_ms_ = request.tick_ms;
    }
    Transit(State::kDisabled);
    return output_;
  }

  const uint32_t elapsed_ms = request.tick_ms - state_enter_tick_ms_;
  const State requested_normal_state = ResolveRequestedNormalState(requested_leg_profile_);

  State next_mode = mode_;

  // 状态机只处理模式时序；具体力矩由 Chassis 根据输出的 ControlOutput 计算。
  switch (mode_) {
    case State::kDisabled:
      next_mode = requested_normal_state;
      break;

    case State::kLowLeg:
      if (request.fall_detected) {
        next_mode = State::kRecoveryFallCheck;
      } else if (request.jump_trigger && request.leg_request == wheel_legged::LegProfile::kLow) {
        next_mode = State::kJumpPrep;
      } else if (request.spin_hold) {
        next_mode = State::kSpin;
      } else {
        next_mode = requested_normal_state;
      }
      break;

    case State::kMidLeg:
      if (request.fall_detected) {
        next_mode = State::kRecoveryFallCheck;
      } else if (request.spin_hold) {
        next_mode = State::kSpin;
      } else {
        next_mode = requested_normal_state;
      }
      break;

    case State::kHighLeg:
      if (request.fall_detected) {
        next_mode = State::kRecoveryFallCheck;
      } else if (request.spin_hold) {
        next_mode = State::kSpin;
      } else if (request.theta_ll_rad > wheel_legged::params::active::chassis_fsm::kStairClimbThetaThresholdRad &&
                 request.theta_lr_rad > wheel_legged::params::active::chassis_fsm::kStairClimbThetaThresholdRad) {
        next_mode = State::kStairClimb;
      } else {
        next_mode = requested_normal_state;
      }
      break;

    case State::kSpin:
      if (request.fall_detected) {
        next_mode = State::kRecoveryFallCheck;
      } else if (!request.spin_hold) {
        next_mode = requested_normal_state;
      }
      break;

    case State::kJumpPrep:
      if (elapsed_ms >= wheel_legged::params::active::chassis_fsm::kJumpPrepMs) {
        next_mode = State::kJumpPush;
      }
      break;

    case State::kJumpPush:
      if (request.current_leg_length_m >= wheel_legged::params::active::chassis_fsm::kJumpPushReachedLegLengthM ||
          elapsed_ms >= wheel_legged::params::active::chassis_fsm::kJumpPushMaxMs) {
        next_mode = State::kJumpRecover;
      }
      break;

    case State::kJumpRecover:
      if (elapsed_ms >= wheel_legged::params::active::chassis_fsm::kJumpRecoverMs) {
        next_mode = State::kLowLeg;
      }
      break;

    case State::kRecoveryFallCheck:
      if (request.fall_detected_hold_ms >= wheel_legged::params::active::chassis_fsm::kRecoveryFallConfirmMs) {
        next_mode = State::kRecoverySelfRight;
      } else if (!request.fall_detected) {
        next_mode = State::kLowLeg;
      }
      break;

    case State::kRecoverySelfRight:
      if (elapsed_ms >= wheel_legged::params::active::chassis_fsm::kRecoverySelfRightTimeoutMs) {
        next_mode = State::kDisabled;
      } else if (request.upright_stable) {
        next_mode = State::kLowLeg;
      }
      break;

    case State::kStairClimb:
      if (request.fall_detected) {
        next_mode = State::kRecoveryFallCheck;
      } else if (IsStairClimbReadyToDone(request) ||
                 elapsed_ms >= wheel_legged::params::active::chassis_fsm::kStairClimbDurationMs) {
        next_mode = State::kStairClimbDone;
      }
      break;

    case State::kStairClimbDone:
      if (request.fall_detected) {
        next_mode = State::kRecoveryFallCheck;
      } else if (elapsed_ms >= wheel_legged::params::active::chassis_fsm::kStairClimbPitchStableMs) {
        next_mode = State::kHighLeg;
      }
      break;
  }

  if (next_mode != mode_) {
    state_enter_tick_ms_ = request.tick_ms;
  }

  Transit(next_mode);

  if (!IsJumpState(mode_) &&
      (mode_ == State::kLowLeg || mode_ == State::kMidLeg || mode_ == State::kHighLeg || mode_ == State::kSpin)) {
    output_.control = BuildControlOutput(mode_, requested_leg_profile_);
  }

  return output_;
}
