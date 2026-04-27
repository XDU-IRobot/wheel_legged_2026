#pragma once

/**
 * @file  wl_infantry/fsm/fsm_internal.hpp
 * @brief `Fsm` 的 ETL HFSM 私有实现声明
 */

#include <array>
#include <cstdint>

#include <etl/hfsm.h>

#include "fsm.hpp"

namespace wl_infantry::fsm_internal {

/**
 * @brief 主状态机使用的目标腿长常量
 */
inline constexpr float kLegLengthLowM = 0.18f;
inline constexpr float kLegLengthMidM = 0.24f;
inline constexpr float kLegLengthHighM = 0.30f;

inline constexpr float kJumpPrepTargetLegM = 0.13f;
inline constexpr float kJumpPushTargetLegM = 0.36f;
inline constexpr float kJumpRecoverTargetLegM = 0.20f;

inline constexpr uint32_t kJumpPrepDurationMs = 450U;
inline constexpr uint32_t kJumpPushMaxDurationMs = 1000U;
inline constexpr uint32_t kJumpRecoverDurationMs = 450U;

inline constexpr float kJumpPushLegReachedM = 0.30f;

inline constexpr uint32_t kFallConfirmMs = 220U;
inline constexpr uint32_t kSelfRightTimeoutMs = 2200U;

/**
 * @brief ETL HFSM 内部状态编号
 * @note  这些编号只在内部使用；外部仍然观察 `Fsm::State`。
 */
enum EtlStateId : etl::fsm_state_id_t {
  kDisabledState = 0,
  kServiceState,
  kServiceGimbalOnlyWithFireState,
  kServiceWithFireState,
  kServiceWithFireLowLegState,
  kServiceWithFireMidLegState,
  kServiceWithFireHighLegState,
  kServiceWithFireSpinState,
  kServiceWithFireJumpPrepState,
  kServiceWithFireJumpPushState,
  kServiceWithFireJumpRecoverState,
  kServiceSafeState,
  kServiceSafeLowLegState,
  kServiceSafeMidLegState,
  kServiceSafeHighLegState,
  kServiceSafeSpinState,
  kServiceSafeJumpPrepState,
  kServiceSafeJumpPushState,
  kServiceSafeJumpRecoverState,
  kCombatState,
  kCombatLowLegState,
  kCombatMidLegState,
  kCombatHighLegState,
  kCombatSpinState,
  kCombatJumpPrepState,
  kCombatJumpPushState,
  kCombatJumpRecoverState,
  kRecoveryState,
  kRecoveryFallCheckState,
  kRecoverySelfRightState,
  kStateCount,
};

/**
 * @brief 主 HFSM 每个控制拍接收的统一输入事件
 */
struct TickEvent : public etl::message<1> {
  explicit TickEvent(const Fsm::Input& input_) : input(input_) {}
  Fsm::Input input{};
};

float TargetLegLengthForProfile(Fsm::LegProfile profile);
Fsm::State ResolveRequestedDriveState(const Fsm::Input& input);
Fsm::State ResolveRequestedLowLegState(const Fsm::Input& input);
bool IsWheelFireEnabledState(Fsm::State state);
bool IsChassisEnabledState(Fsm::State state);
bool IsSpinState(Fsm::State state);
bool IsJumpState(Fsm::State state);
bool IsRecoveryState(Fsm::State state);
Fsm::State JumpPrepStateForLowState(Fsm::State low_state);
Fsm::State JumpPushStateForPrep(Fsm::State prep_state);
Fsm::State JumpRecoverStateForPush(Fsm::State push_state);
Fsm::State SpinStateForDrive(Fsm::State drive_state);
etl::fsm_state_id_t ToEtlStateId(Fsm::State state);

etl::fsm_state_id_t DriveLeafEvent(Fsm::EtlImpl& context, Fsm::State self_state);
etl::fsm_state_id_t MidHighLeafEvent(Fsm::EtlImpl& context, Fsm::State self_state);
etl::fsm_state_id_t SpinLeafEvent(Fsm::EtlImpl& context);
etl::fsm_state_id_t JumpPrepLeafEvent(Fsm::EtlImpl& context);
etl::fsm_state_id_t JumpPushLeafEvent(Fsm::EtlImpl& context);
etl::fsm_state_id_t JumpRecoverLeafEvent(Fsm::EtlImpl& context);
etl::fsm_state_id_t MotionCompositeEvent(Fsm::EtlImpl& context);

template <typename TDerived, etl::fsm_state_id_t STATE_ID>
class CompositeStateBase : public etl::fsm_state<Fsm::EtlImpl, TDerived, STATE_ID, TickEvent> {
 public:
  /**
   * @brief 组合状态默认不直接改变对外叶子态，只承接公共规则。
   */
  etl::fsm_state_id_t on_enter_state() override { return etl::ifsm_state::No_State_Change; }

  etl::fsm_state_id_t on_event_unknown(const etl::imessage&) { return etl::ifsm_state::No_State_Change; }
};

template <typename TDerived, etl::fsm_state_id_t STATE_ID, Fsm::State PUBLIC_STATE>
class LeafStateBase : public etl::fsm_state<Fsm::EtlImpl, TDerived, STATE_ID, TickEvent> {
 public:
  /**
   * @brief 叶子状态进入时，同步刷新外层 `Fsm::state_`
   */
  etl::fsm_state_id_t on_enter_state() override {
    this->get_fsm_context().EnterLeaf(PUBLIC_STATE);
    return etl::ifsm_state::No_State_Change;
  }

  etl::fsm_state_id_t on_event_unknown(const etl::imessage&) { return etl::ifsm_state::No_State_Change; }
};

class DisabledState : public LeafStateBase<DisabledState, kDisabledState, Fsm::State::kDisabled> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class ServiceState : public CompositeStateBase<ServiceState, kServiceState> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class ServiceGimbalOnlyWithFireState
    : public LeafStateBase<ServiceGimbalOnlyWithFireState, kServiceGimbalOnlyWithFireState,
                           Fsm::State::kServiceGimbalOnlyWithFire> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class ServiceWithFireState : public CompositeStateBase<ServiceWithFireState, kServiceWithFireState> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class ServiceWithFireLowLegState : public LeafStateBase<ServiceWithFireLowLegState, kServiceWithFireLowLegState,
                                                        Fsm::State::kServiceWithFireLowLeg> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class ServiceWithFireMidLegState : public LeafStateBase<ServiceWithFireMidLegState, kServiceWithFireMidLegState,
                                                        Fsm::State::kServiceWithFireMidLeg> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class ServiceWithFireHighLegState : public LeafStateBase<ServiceWithFireHighLegState, kServiceWithFireHighLegState,
                                                         Fsm::State::kServiceWithFireHighLeg> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class ServiceWithFireSpinState
    : public LeafStateBase<ServiceWithFireSpinState, kServiceWithFireSpinState, Fsm::State::kServiceWithFireSpin> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class ServiceWithFireJumpPrepState : public LeafStateBase<ServiceWithFireJumpPrepState, kServiceWithFireJumpPrepState,
                                                          Fsm::State::kServiceWithFireJumpPrep> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class ServiceWithFireJumpPushState : public LeafStateBase<ServiceWithFireJumpPushState, kServiceWithFireJumpPushState,
                                                          Fsm::State::kServiceWithFireJumpPush> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class ServiceWithFireJumpRecoverState
    : public LeafStateBase<ServiceWithFireJumpRecoverState, kServiceWithFireJumpRecoverState,
                           Fsm::State::kServiceWithFireJumpRecover> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class ServiceSafeState : public CompositeStateBase<ServiceSafeState, kServiceSafeState> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class ServiceSafeLowLegState
    : public LeafStateBase<ServiceSafeLowLegState, kServiceSafeLowLegState, Fsm::State::kServiceSafeLowLeg> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class ServiceSafeMidLegState
    : public LeafStateBase<ServiceSafeMidLegState, kServiceSafeMidLegState, Fsm::State::kServiceSafeMidLeg> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class ServiceSafeHighLegState
    : public LeafStateBase<ServiceSafeHighLegState, kServiceSafeHighLegState, Fsm::State::kServiceSafeHighLeg> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class ServiceSafeSpinState
    : public LeafStateBase<ServiceSafeSpinState, kServiceSafeSpinState, Fsm::State::kServiceSafeSpin> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class ServiceSafeJumpPrepState
    : public LeafStateBase<ServiceSafeJumpPrepState, kServiceSafeJumpPrepState, Fsm::State::kServiceSafeJumpPrep> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class ServiceSafeJumpPushState
    : public LeafStateBase<ServiceSafeJumpPushState, kServiceSafeJumpPushState, Fsm::State::kServiceSafeJumpPush> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class ServiceSafeJumpRecoverState : public LeafStateBase<ServiceSafeJumpRecoverState, kServiceSafeJumpRecoverState,
                                                         Fsm::State::kServiceSafeJumpRecover> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class CombatState : public CompositeStateBase<CombatState, kCombatState> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class CombatLowLegState : public LeafStateBase<CombatLowLegState, kCombatLowLegState, Fsm::State::kCombatLowLeg> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class CombatMidLegState : public LeafStateBase<CombatMidLegState, kCombatMidLegState, Fsm::State::kCombatMidLeg> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class CombatHighLegState : public LeafStateBase<CombatHighLegState, kCombatHighLegState, Fsm::State::kCombatHighLeg> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class CombatSpinState : public LeafStateBase<CombatSpinState, kCombatSpinState, Fsm::State::kCombatSpin> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class CombatJumpPrepState
    : public LeafStateBase<CombatJumpPrepState, kCombatJumpPrepState, Fsm::State::kCombatJumpPrep> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class CombatJumpPushState
    : public LeafStateBase<CombatJumpPushState, kCombatJumpPushState, Fsm::State::kCombatJumpPush> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class CombatJumpRecoverState
    : public LeafStateBase<CombatJumpRecoverState, kCombatJumpRecoverState, Fsm::State::kCombatJumpRecover> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class RecoveryState : public CompositeStateBase<RecoveryState, kRecoveryState> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class RecoveryFallCheckState
    : public LeafStateBase<RecoveryFallCheckState, kRecoveryFallCheckState, Fsm::State::kRecoveryFallCheck> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

class RecoverySelfRightState
    : public LeafStateBase<RecoverySelfRightState, kRecoverySelfRightState, Fsm::State::kRecoverySelfRight> {
 public:
  etl::fsm_state_id_t on_event(const TickEvent& event);
};

}  // namespace wl_infantry::fsm_internal

/**
 * @brief `Fsm` 的 ETL HFSM 上下文
 * @note  该类持有所有 ETL 状态对象，并向外层 `Fsm` 同步当前叶子状态。
 */
struct Fsm::EtlImpl : public etl::hfsm {
  explicit EtlImpl(Fsm& owner);

  void Dispatch(const Input& input);
  void EnterLeaf(State state);

  [[nodiscard]] const Input& input() const;
  [[nodiscard]] bool ShouldDisable() const;
  [[nodiscard]] State RequestedDriveState() const;
  [[nodiscard]] State RequestedLowLegState() const;
  [[nodiscard]] uint32_t ElapsedMs() const;
  [[nodiscard]] State CurrentLeafState() const;
  void SetJumpExitLowState(State state);
  [[nodiscard]] State JumpExitLowState() const;

 private:
  Fsm& owner_;
  Input input_{};
  std::array<etl::ifsm_state*, wl_infantry::fsm_internal::kStateCount> states_{};

  wl_infantry::fsm_internal::DisabledState disabled_{};
  wl_infantry::fsm_internal::ServiceState service_{};
  wl_infantry::fsm_internal::ServiceGimbalOnlyWithFireState service_gimbal_only_with_fire_{};
  wl_infantry::fsm_internal::ServiceWithFireState service_with_fire_{};
  wl_infantry::fsm_internal::ServiceWithFireLowLegState service_with_fire_low_leg_{};
  wl_infantry::fsm_internal::ServiceWithFireMidLegState service_with_fire_mid_leg_{};
  wl_infantry::fsm_internal::ServiceWithFireHighLegState service_with_fire_high_leg_{};
  wl_infantry::fsm_internal::ServiceWithFireSpinState service_with_fire_spin_{};
  wl_infantry::fsm_internal::ServiceWithFireJumpPrepState service_with_fire_jump_prep_{};
  wl_infantry::fsm_internal::ServiceWithFireJumpPushState service_with_fire_jump_push_{};
  wl_infantry::fsm_internal::ServiceWithFireJumpRecoverState service_with_fire_jump_recover_{};
  wl_infantry::fsm_internal::ServiceSafeState service_safe_{};
  wl_infantry::fsm_internal::ServiceSafeLowLegState service_safe_low_leg_{};
  wl_infantry::fsm_internal::ServiceSafeMidLegState service_safe_mid_leg_{};
  wl_infantry::fsm_internal::ServiceSafeHighLegState service_safe_high_leg_{};
  wl_infantry::fsm_internal::ServiceSafeSpinState service_safe_spin_{};
  wl_infantry::fsm_internal::ServiceSafeJumpPrepState service_safe_jump_prep_{};
  wl_infantry::fsm_internal::ServiceSafeJumpPushState service_safe_jump_push_{};
  wl_infantry::fsm_internal::ServiceSafeJumpRecoverState service_safe_jump_recover_{};
  wl_infantry::fsm_internal::CombatState combat_{};
  wl_infantry::fsm_internal::CombatLowLegState combat_low_leg_{};
  wl_infantry::fsm_internal::CombatMidLegState combat_mid_leg_{};
  wl_infantry::fsm_internal::CombatHighLegState combat_high_leg_{};
  wl_infantry::fsm_internal::CombatSpinState combat_spin_{};
  wl_infantry::fsm_internal::CombatJumpPrepState combat_jump_prep_{};
  wl_infantry::fsm_internal::CombatJumpPushState combat_jump_push_{};
  wl_infantry::fsm_internal::CombatJumpRecoverState combat_jump_recover_{};
  wl_infantry::fsm_internal::RecoveryState recovery_{};
  wl_infantry::fsm_internal::RecoveryFallCheckState recovery_fall_check_{};
  wl_infantry::fsm_internal::RecoverySelfRightState recovery_self_right_{};
};
