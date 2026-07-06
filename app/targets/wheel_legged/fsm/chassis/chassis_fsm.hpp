#pragma once

#include <etl/fsm.h>

#include "chassis_states.hpp"
#include "chassis_types.hpp"

namespace wheel_legged::fsm {

/**
 * @brief 基于 ETL 的底盘顶层状态机。
 *
 * 顶层只保留 Disable、Standby、Normal、Fly、Upstairs、Spin、SpecialSpin、Fall、Jump；
 * 小陀螺退出、跳跃和倒地恢复的时序由内部 phase 表达，不再膨胀顶层状态枚举。
 */
class ChassisFsm final : public etl::fsm {
 public:
  ChassisFsm();

  void Init();
  ChassisFsmOutput Update(const ChassisFsmRequest &request);

  [[nodiscard]] const ChassisFsmOutput &output() const { return output_; }
  [[nodiscard]] ChassisState state() const { return output_.state; }
  [[nodiscard]] const ChassisFsmRequest &request() const { return request_; }
  [[nodiscard]] ChassisFsmContext &context() { return context_; }
  [[nodiscard]] const ChassisFsmContext &context() const { return context_; }

  etl::fsm_state_id_t TransitionTo(ChassisState state, TransitionReason reason);
  void EnterState(ChassisState state);
  void EnterPhase(uint32_t tick_ms);
  void SyncPhaseOutput();

  void WriteDisableOutput();
  void WriteStandbyOutput();
  void WriteNormalOutput();
  void WriteFlyOutput();
  void WriteUpstairsOutput();
  void WriteSpinOutput();
  void WriteSpecialSpinOutput();
  void WriteFallOutput();
  void WriteJumpOutput();

 private:
  void WriteBaseOutput(ChassisState state, ChassisBehavior behavior, wheel_legged::LegProfile leg_profile,
                       bool motor_enable, bool wheel_torque_enable, bool run_controller);

  ChassisDisableState disable_state_{};
  ChassisStandbyState standby_state_{};
  ChassisNormalState normal_state_{};
  ChassisFlyState fly_state_{};
  ChassisUpstairsState upstairs_state_{};
  ChassisSpinState spin_state_{};
  ChassisSpecialSpinState special_spin_state_{};
  ChassisFallState fall_state_{};
  ChassisJumpState jump_state_{};
  etl::ifsm_state *state_list_[static_cast<uint8_t>(ChassisState::kCount)]{};

  ChassisFsmRequest request_{};
  ChassisFsmOutput output_{};
  ChassisFsmContext context_{};
  TransitionReason pending_transition_reason_{TransitionReason::kInitialized};
  bool has_entered_state_{false};
};

}  // namespace wheel_legged::fsm
