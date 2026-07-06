#pragma once

#include <cstdint>

#include "../common.hpp"
#include "../../include/fsm_common.hpp"

namespace wheel_legged::fsm {

/** @brief 底盘顶层状态。动作内部阶段不再占用顶层状态枚举。 */
enum class ChassisState : uint8_t {
  kDisable = 0,
  kStandby,
  kNormal,
  kFly,
  kUpstairs,
  kSpin,
  kSpecialSpin,
  kFall,
  kJump,
  kCount,
};

enum class ChassisBehavior : uint8_t {
  kSafeStop = 0,
  kStandby,
  kBalance,
  kUpstairs,
  kSpin,
  kSpecialSpin,
  kFallRecovery,
  kJump,
};

enum class SpinPhase : uint8_t {
  kRunning = 0,
  kExitPending,
};

enum class JumpPhase : uint8_t {
  kPrepare = 0,
  kPush,
  kRecover,
};

enum class JumpMode : uint8_t {
  kManual = 0,
  kAutomatic,
};

enum class FallPhase : uint8_t {
  kConfirm = 0,
  kSelfRight,
  kStandup,
};

enum class UpstairsMode : uint8_t {
  kSingle = 0,
  kDouble,
  kContinuous,
};

/**
 * @brief 新底盘状态机单周期请求。
 * @note  输入层负责完成 DR16/键鼠仲裁，FSM 不解析任何原始键位。
 */
struct ChassisFsmRequest {
  bool input_valid{false};
  ChassisState requested_state{ChassisState::kDisable};

  bool spin_hold{false};
  bool special_spin_hold{false};

  bool jump_trigger{false};
  JumpMode jump_mode{JumpMode::kManual};

  UpstairsMode upstairs_mode{UpstairsMode::kSingle};
  bool upstairs_active{false};
  bool upstairs_finished{false};
  bool upstairs_aborted{false};
  bool upstairs_recovery_required{false};

  bool fall_detected{false};
  uint32_t fall_hold_ms{0U};
  bool upright_stable{false};
  bool standup_complete{false};

  bool spin_exit_yaw_aligned{false};
  bool off_ground{false};

  float current_speed_mps{0.0f};
  float current_leg_length_m{0.0f};
  uint32_t tick_ms{0U};
};

/** @brief 新底盘状态机统一输出。 */
struct ChassisFsmOutput {
  ChassisState state{ChassisState::kDisable};
  bool state_changed{false};

  SpinPhase spin_phase{SpinPhase::kRunning};
  JumpPhase jump_phase{JumpPhase::kPrepare};
  JumpMode jump_mode{JumpMode::kManual};
  FallPhase fall_phase{FallPhase::kConfirm};
  UpstairsMode upstairs_mode{UpstairsMode::kSingle};

  bool chassis_motor_enable{false};
  bool wheel_torque_enable{false};
  bool run_controller{false};

  wheel_legged::LegProfile leg_profile{wheel_legged::LegProfile::kLow};
  ChassisBehavior behavior{ChassisBehavior::kSafeStop};
  float target_leg_length_m{0.0F};
  bool upstairs_task_active{false};

  uint32_t state_elapsed_ms{0U};
  TransitionReason transition_reason{TransitionReason::kNone};
};

/** @brief 仅由底盘 FSM 持有的跨周期状态。 */
struct ChassisFsmContext {
  uint32_t state_enter_tick_ms{0U};
  ChassisState previous_stable_state{ChassisState::kNormal};

  SpinPhase spin_phase{SpinPhase::kRunning};
  JumpPhase jump_phase{JumpPhase::kPrepare};
  JumpMode jump_mode{JumpMode::kManual};
  ChassisState jump_return_state{ChassisState::kNormal};
  FallPhase fall_phase{FallPhase::kConfirm};
  UpstairsMode upstairs_mode{UpstairsMode::kSingle};

  uint32_t phase_enter_tick_ms{0U};
  uint32_t jump_push_reached_tick_ms{0U};
  bool jump_push_reached_armed{true};
  bool spin_lock_low{false};
  ChassisState spin_locked_request{ChassisState::kNormal};
};

}  // namespace wheel_legged::fsm
