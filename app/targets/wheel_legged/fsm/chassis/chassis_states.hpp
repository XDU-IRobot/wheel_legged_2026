#pragma once

#include <etl/fsm.h>

#include "../common_events.hpp"
#include "chassis_events.hpp"

namespace wheel_legged::fsm {

class ChassisFsm;

/** @brief 底盘状态公共基类，统一处理紧急失能、复位和未知事件。 */
template <typename TDerived, ChassisState TState>
class ChassisStateBase : public etl::fsm_state<ChassisFsm, TDerived, static_cast<etl::fsm_state_id_t>(TState),
                                               ChassisUpdateEvent, DisableEvent, ResetEvent> {
 public:
  etl::fsm_state_id_t on_event(const DisableEvent &event);
  etl::fsm_state_id_t on_event(const ResetEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &event);

 protected:
  [[nodiscard]] ChassisFsm &machine() const { return this->get_fsm_context(); }
};

#define WHEEL_LEGGED_DECLARE_CHASSIS_STATE(ClassName, StateValue)          \
  class ClassName final : public ChassisStateBase<ClassName, StateValue> { \
   public:                                                                 \
    using Base = ChassisStateBase<ClassName, StateValue>;                  \
    using Base::on_event;                                                  \
    using Base::on_event_unknown;                                          \
    etl::fsm_state_id_t on_enter_state() override;                         \
    void on_exit_state() override;                                         \
    etl::fsm_state_id_t on_event(const ChassisUpdateEvent &event);         \
  }

WHEEL_LEGGED_DECLARE_CHASSIS_STATE(ChassisDisableState, ChassisState::kDisable);
WHEEL_LEGGED_DECLARE_CHASSIS_STATE(ChassisStandbyState, ChassisState::kStandby);
WHEEL_LEGGED_DECLARE_CHASSIS_STATE(ChassisNormalState, ChassisState::kNormal);
WHEEL_LEGGED_DECLARE_CHASSIS_STATE(ChassisFlyState, ChassisState::kFly);
WHEEL_LEGGED_DECLARE_CHASSIS_STATE(ChassisUpstairsState, ChassisState::kUpstairs);
WHEEL_LEGGED_DECLARE_CHASSIS_STATE(ChassisSpinState, ChassisState::kSpin);
WHEEL_LEGGED_DECLARE_CHASSIS_STATE(ChassisSpecialSpinState, ChassisState::kSpecialSpin);
WHEEL_LEGGED_DECLARE_CHASSIS_STATE(ChassisFallState, ChassisState::kFall);
WHEEL_LEGGED_DECLARE_CHASSIS_STATE(ChassisJumpState, ChassisState::kJump);

#undef WHEEL_LEGGED_DECLARE_CHASSIS_STATE

}  // namespace wheel_legged::fsm
