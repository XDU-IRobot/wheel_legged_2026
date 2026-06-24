#include "include/chassis/fsm.hpp"
#include "include/params.hpp"

/**
 * @file  targets/wheel_legged/chassis_fsm.cc
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
wheel_legged::LegProfile ResolveRequestedLegProfile(const wheel_legged::ChassisFsmInput &request) {
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
                                                       const wheel_legged::LegProfile requested_leg_profile,
                                                       const wheel_legged::LegProfile jump_leg_profile,
                                                       const bool stair_descend_retracted,
                                                       const bool stair_step2 = false) {
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
    case chassis::Fsm::State::kStandby:
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
      control.target_leg_length_m = stair_step2
                                        ? wheel_legged::params::active::chassis_fsm::kStairClimbStep2.high_leg_length_m
                                        : wheel_legged::params::active::chassis_fsm::kHighLegLengthM;
      control.jump_phase = 0U;
      break;

    case chassis::Fsm::State::kSpin:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = true;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.leg_profile = wheel_legged::LegProfile::kLow;
      control.target_leg_length_m = wheel_legged::params::active::chassis_fsm::kLowLegLengthM;
      control.jump_phase = 0U;
      break;

    case chassis::Fsm::State::kSpinExitPending:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = true;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.leg_profile = wheel_legged::LegProfile::kLow;
      control.target_leg_length_m = wheel_legged::params::active::chassis_fsm::kLowLegLengthM;
      control.jump_phase = 0U;
      break;

    case chassis::Fsm::State::kJumpPrep:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.leg_profile = jump_leg_profile;
      control.target_leg_length_m = wheel_legged::params::active::chassis_fsm::kJumpLowPrepLegLengthM;
      control.jump_phase = 1U;
      break;

    case chassis::Fsm::State::kJumpPush:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.leg_profile = jump_leg_profile;
      control.target_leg_length_m = wheel_legged::params::active::chassis_fsm::kJumpLowPushLegLengthM;
      control.jump_phase = 2U;
      break;

    case chassis::Fsm::State::kJumpRecover:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.leg_profile = jump_leg_profile;
      control.target_leg_length_m = wheel_legged::params::active::chassis_fsm::kJumpLowRecoverLegLengthM;
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

    case chassis::Fsm::State::kStairTask:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.leg_profile = wheel_legged::LegProfile::kHigh;
      control.target_leg_length_m = stair_step2
                                        ? wheel_legged::params::active::chassis_fsm::kStairClimbStep2.high_leg_length_m
                                        : wheel_legged::params::active::chassis_fsm::kStairClimb.high_leg_length_m;
      control.jump_phase = 0U;
      break;

    case chassis::Fsm::State::kStairDescend:
      control.enable_dm = true;
      control.run_chassis_update = true;
      control.spin_enable = false;
      control.recovery_enable = false;
      control.safe_output_required = false;
      control.leg_profile = wheel_legged::LegProfile::kLow;
      control.target_leg_length_m = stair_descend_retracted
                                        ? wheel_legged::params::active::chassis_fsm::kLowLegLengthM
                                        : wheel_legged::params::active::chassis_fsm::kStairDescendLegLengthM;
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
  stair_descend_retracted_ = false;
  jump_push_reached_armed_ = true;
  jump_push_reached_tick_ms_ = 0U;
  state_enter_tick_ms_ = 0U;
  output_ = {};
  output_.mode = mode_;
  output_.control =
      BuildControlOutput(mode_, requested_leg_profile_, jump_leg_profile_, stair_descend_retracted_, stair_step2_);
}

void chassis::Fsm::Transit(const State new_mode) {
  output_.state_changed = (new_mode != mode_);
  mode_ = new_mode;
  output_.mode = mode_;
  output_.control =
      BuildControlOutput(mode_, requested_leg_profile_, jump_leg_profile_, stair_descend_retracted_, stair_step2_);
}

chassis::Fsm::Output chassis::Fsm::Update(const Input &input) {
  output_.state_changed = false;

  const wheel_legged::ChassisFsmInput &request = input.request;
  stair_step2_ = request.stair_step2;
  const wheel_legged::LegProfile raw_leg_profile = ResolveRequestedLegProfile(request);

  // 小陀螺锁低腿长：检测手动切档解锁
  if (spin_lock_low_) {
    if (raw_leg_profile != prev_leg_request_) {
      spin_lock_low_ = false;
    } else {
      requested_leg_profile_ = wheel_legged::LegProfile::kLow;
    }
  }
  if (!spin_lock_low_) {
    requested_leg_profile_ = raw_leg_profile;
  }
  prev_leg_request_ = raw_leg_profile;

  // 输入失效或请求关闭时立即退回 Disabled，并由执行器层输出零力矩。
  if (!request.input_valid || request.domain_request == wheel_legged::DomainRequest::kDisabled) {
    if (mode_ != State::kDisabled) {
      state_enter_tick_ms_ = request.tick_ms;
      spin_lock_low_ = false;
    }
    Transit(State::kDisabled);
    return output_;
  }

  const uint32_t elapsed_ms = request.tick_ms - state_enter_tick_ms_;
  const State requested_normal_state =
      request.standby ? State::kStandby : ResolveRequestedNormalState(requested_leg_profile_);
  const State requested_stable_state =
      (!request.standby && request.stair_descend_active) ? State::kStairDescend : requested_normal_state;

  State next_mode = mode_;

  // 状态机只处理模式时序；具体力矩由 Chassis 根据输出的 ControlOutput 计算。
  switch (mode_) {
    case State::kDisabled:
      next_mode = request.stair_task_active ? State::kStairTask : requested_stable_state;
      break;

    case State::kStandby:
      if (request.fall_detected) {
        next_mode = State::kRecoveryFallCheck;
      } else if (!request.standby) {
        next_mode = requested_stable_state;
      }
      break;

    case State::kLowLeg:
      if (request.fall_detected) {
        next_mode = State::kRecoveryFallCheck;
      } else if (request.standby) {
        next_mode = State::kStandby;
      } else if (request.stair_task_active) {
        next_mode = State::kStairTask;
      } else if (request.jump_trigger) {
        jump_leg_profile_ = wheel_legged::LegProfile::kLow;
        next_mode = State::kJumpPrep;
      } else if (request.spin_hold && std::fabs(request.current_s_dot) <
                                          wheel_legged::params::active::chassis_fsm::kSpinEntrySpeedThresholdMps) {
        next_mode = State::kSpin;
      } else {
        next_mode = requested_stable_state;
      }
      break;

    case State::kMidLeg:
      if (request.fall_detected) {
        next_mode = State::kRecoveryFallCheck;
      } else if (request.standby) {
        next_mode = State::kStandby;
      } else if (request.stair_task_active) {
        next_mode = State::kStairTask;
      } else if (request.jump_trigger) {
        jump_leg_profile_ = wheel_legged::LegProfile::kMid;
        next_mode = State::kJumpPrep;
      } else if (request.spin_hold && std::fabs(request.current_s_dot) <
                                          wheel_legged::params::active::chassis_fsm::kSpinEntrySpeedThresholdMps) {
        next_mode = State::kSpin;
      } else {
        next_mode = requested_stable_state;
      }
      break;

    case State::kHighLeg:
      if (request.fall_detected) {
        next_mode = State::kRecoveryFallCheck;
      } else if (request.standby) {
        next_mode = State::kStandby;
      } else if (request.stair_task_active) {
        next_mode = State::kStairTask;
      } else if (request.spin_hold && std::fabs(request.current_s_dot) <
                                          wheel_legged::params::active::chassis_fsm::kSpinEntrySpeedThresholdMps) {
        next_mode = State::kSpin;
      } else {
        next_mode = requested_stable_state;
      }
      break;

    case State::kSpin:
      if (request.fall_detected) {
        next_mode = State::kRecoveryFallCheck;
      } else if (request.standby) {
        next_mode = State::kStandby;
      } else if (!request.spin_hold) {
        spin_lock_low_ = true;
        next_mode = State::kSpinExitPending;
      }
      break;

    case State::kSpinExitPending:
      if (request.fall_detected) {
        next_mode = State::kRecoveryFallCheck;
      } else if (request.standby) {
        next_mode = State::kStandby;
      } else if (request.spin_hold) {
        next_mode = State::kSpin;
      } else if (request.spin_exit_yaw_aligned ||
                 elapsed_ms >= wheel_legged::params::active::chassis_fsm::kSpinExitTimeoutMs) {
        next_mode = requested_stable_state;
      }
      break;

    case State::kJumpPrep: {
      if (elapsed_ms >= wheel_legged::params::active::chassis_fsm::kJumpLowPrepMs) {
        next_mode = State::kJumpPush;
      }
      break;
    }

    case State::kJumpPush: {
      if (request.current_leg_length_m >= wheel_legged::params::active::chassis_fsm::kJumpLowPushReachedLegLengthM) {
        if (jump_push_reached_armed_) {
          jump_push_reached_tick_ms_ = request.tick_ms;
          jump_push_reached_armed_ = false;
        }
      } else {
        jump_push_reached_armed_ = true;
      }
      const bool reached_hold_elapsed =
          !jump_push_reached_armed_ && (request.tick_ms - jump_push_reached_tick_ms_ >=
                                        wheel_legged::params::active::chassis_fsm::kJumpPushReachedHoldMs);
      if (reached_hold_elapsed || elapsed_ms >= wheel_legged::params::active::chassis_fsm::kJumpLowPushMaxMs) {
        next_mode = State::kJumpRecover;
      }
      break;
    }

    case State::kJumpRecover: {
      // 最低维持时间结束后，落地（非离地）时退出回收阶段，超时作为保底
      if (elapsed_ms >= wheel_legged::params::active::chassis_fsm::kJumpLowRecoverMinMs && !request.off_ground) {
        next_mode = requested_stable_state;
      } else if (elapsed_ms >= wheel_legged::params::active::chassis_fsm::kJumpLowRecoverMs) {
        next_mode = requested_stable_state;
      }
      break;
    }

    case State::kRecoveryFallCheck:
      if (request.fall_detected_hold_ms >= wheel_legged::params::active::chassis_fsm::kRecoveryFallConfirmMs) {
        next_mode = State::kRecoverySelfRight;
      } else if (!request.fall_detected) {
        next_mode = requested_stable_state;
      }
      break;

    case State::kRecoverySelfRight:
      if (elapsed_ms >= wheel_legged::params::active::chassis_fsm::kRecoverySelfRightTimeoutMs) {
        next_mode = State::kDisabled;
      } else if (request.upright_stable) {
        next_mode = requested_normal_state;
      }
      break;

    case State::kStairTask:
      if (request.fall_detected || request.stair_task_recovery_required) {
        next_mode = State::kRecoveryFallCheck;
      } else if (request.standby) {
        next_mode = State::kStandby;
      } else if (request.spin_hold) {
        next_mode = State::kSpin;
      } else if (!request.stair_task_active) {
        next_mode = requested_stable_state;
      }
      break;

    case State::kStairDescend:
      if (request.fall_detected) {
        next_mode = State::kRecoveryFallCheck;
      } else if (request.standby) {
        next_mode = State::kStandby;
      } else if (request.stair_task_active) {
        next_mode = State::kStairTask;
      } else if (request.spin_hold) {
        next_mode = State::kSpin;
      } else if (!request.stair_descend_active) {
        next_mode = requested_normal_state;
      }
      break;
  }

  if (next_mode == State::kStairDescend) {
    if (mode_ != State::kStairDescend) {
      stair_descend_retracted_ = false;
    }
    if (request.theta_b_rad > wheel_legged::params::active::chassis_fsm::kStairDescendThetaBTriggerRad) {
      stair_descend_retracted_ = true;
    }
  } else {
    stair_descend_retracted_ = false;
  }

  if (next_mode != mode_) {
    state_enter_tick_ms_ = request.tick_ms;
  }

  Transit(next_mode);

  if (!IsJumpState(mode_) &&
      (mode_ == State::kLowLeg || mode_ == State::kMidLeg || mode_ == State::kHighLeg || mode_ == State::kSpin ||
       mode_ == State::kStandby || mode_ == State::kStairTask || mode_ == State::kStairDescend)) {
    output_.control =
        BuildControlOutput(mode_, requested_leg_profile_, jump_leg_profile_, stair_descend_retracted_, stair_step2_);
  }

  return output_;
}
