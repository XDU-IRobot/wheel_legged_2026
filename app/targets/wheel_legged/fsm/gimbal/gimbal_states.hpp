#pragma once

#include <etl/fsm.h>

#include "../common_events.hpp"
#include "gimbal_events.hpp"

namespace wheel_legged::fsm {

class GimbalFsm;

/**
 * @brief 所有云台状态的 CRTP 公共基类。
 *
 * TState 直接映射到 ETL state id；基类统一处理全局 Disable、Reset 和未知事件，
 * 具体状态只需实现周期更新、进入和退出逻辑。
 */
template <typename TDerived, GimbalState TState>
class GimbalStateBase : public etl::fsm_state<GimbalFsm, TDerived, static_cast<etl::fsm_state_id_t>(TState),
                                              GimbalUpdateEvent, DisableEvent, ResetEvent> {
 public:
  etl::fsm_state_id_t on_event(const DisableEvent &event);
  etl::fsm_state_id_t on_event(const ResetEvent &event);
  etl::fsm_state_id_t on_event_unknown(const etl::imessage &event);

 protected:
  /** @brief 获取拥有当前状态对象的 GimbalFsm 上下文。 */
  [[nodiscard]] GimbalFsm &machine() const { return this->get_fsm_context(); }
};

// 各状态结构完全一致，用宏只消除 ETL 所需的样板声明；状态行为仍在 gimbal_fsm.cc 中分别实现。
#define WHEEL_LEGGED_DECLARE_GIMBAL_STATE(ClassName, StateValue)          \
  class ClassName final : public GimbalStateBase<ClassName, StateValue> { \
   public:                                                                \
    using Base = GimbalStateBase<ClassName, StateValue>;                  \
    using Base::on_event;                                                 \
    using Base::on_event_unknown;                                         \
    etl::fsm_state_id_t on_enter_state() override;                        \
    void on_exit_state() override;                                        \
    etl::fsm_state_id_t on_event(const GimbalUpdateEvent &event);         \
  }

WHEEL_LEGGED_DECLARE_GIMBAL_STATE(GimbalDisableState, GimbalState::kDisable);
WHEEL_LEGGED_DECLARE_GIMBAL_STATE(GimbalStartupAlignState, GimbalState::kStartupAlign);
WHEEL_LEGGED_DECLARE_GIMBAL_STATE(GimbalManualState, GimbalState::kManual);
WHEEL_LEGGED_DECLARE_GIMBAL_STATE(GimbalAimbotState, GimbalState::kAimbot);
WHEEL_LEGGED_DECLARE_GIMBAL_STATE(GimbalRecoveryAlignState, GimbalState::kRecoveryAlign);
WHEEL_LEGGED_DECLARE_GIMBAL_STATE(GimbalRecoveryYawCenteringState, GimbalState::kRecoveryYawCentering);
WHEEL_LEGGED_DECLARE_GIMBAL_STATE(GimbalFfVerifyState, GimbalState::kFfVerify);
WHEEL_LEGGED_DECLARE_GIMBAL_STATE(GimbalIdentState, GimbalState::kIdent);

#undef WHEEL_LEGGED_DECLARE_GIMBAL_STATE

}  // namespace wheel_legged::fsm
