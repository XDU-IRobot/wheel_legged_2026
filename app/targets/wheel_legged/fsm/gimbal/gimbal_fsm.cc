#include "gimbal_fsm.hpp"

namespace wheel_legged::fsm {

namespace {

// 云台 FSM 使用独立 router id；当前事件均同步发送，不依赖消息队列。
constexpr etl::message_router_id_t kGimbalFsmRouterId = 1U;

// GimbalState 的连续枚举值就是 ETL 状态编号。
constexpr etl::fsm_state_id_t StateId(const GimbalState state) { return static_cast<etl::fsm_state_id_t>(state); }

// 输入失联与上层显式请求 Disable 具有相同的安全结果，但保留不同迁移原因用于诊断。
bool IsDisableRequested(const GimbalFsmRequest &request) {
  return !request.input_valid || request.requested_state == GimbalState::kDisable;
}

// 启动对中完成后只允许进入两个常规操作状态；其他请求回退到 Manual。
GimbalState ResolveOperatorState(const GimbalFsmRequest &request) {
  return request.requested_state == GimbalState::kAimbot ? GimbalState::kAimbot : GimbalState::kManual;
}

/**
 * @brief 处理所有运行态共享的安全/恢复迁移。
 * @return ETL 目标状态；没有更高优先级请求时返回 No_State_Change。
 *
 * 顺序不可随意调整：Disable > yaw 回中 > 倒地对齐。
 */
etl::fsm_state_id_t ResolveSafetyOrRecovery(GimbalFsm &machine, const GimbalFsmRequest &request) {
  if (IsDisableRequested(request)) {
    const auto reason = request.input_valid ? TransitionReason::kModeRequested : TransitionReason::kInputLost;
    return machine.TransitionTo(GimbalState::kDisable, reason);
  }
  if (request.yaw_centering_required) {
    return machine.TransitionTo(GimbalState::kRecoveryYawCentering, TransitionReason::kRecoveryRequested);
  }
  if (request.chassis_fall_active) {
    return machine.TransitionTo(GimbalState::kRecoveryAlign, TransitionReason::kFallDetected);
  }
  return etl::ifsm_state::No_State_Change;
}

// 辨识和前馈验证是显式测试状态，只有上层明确请求时才进入。
etl::fsm_state_id_t ResolveRequestedTest(GimbalFsm &machine, const GimbalFsmRequest &request) {
  if (request.requested_state == GimbalState::kIdent) {
    return machine.TransitionTo(GimbalState::kIdent, TransitionReason::kModeRequested);
  }
  if (request.requested_state == GimbalState::kFfVerify) {
    return machine.TransitionTo(GimbalState::kFfVerify, TransitionReason::kModeRequested);
  }
  return etl::ifsm_state::No_State_Change;
}

}  // namespace

GimbalFsm::GimbalFsm() : etl::fsm(kGimbalFsmRouterId) {
  // set_states() 要求数组下标与每个状态类声明的 state id 完全一致。
  state_list_[StateId(GimbalState::kDisable)] = &disable_state_;
  state_list_[StateId(GimbalState::kStartupAlign)] = &startup_align_state_;
  state_list_[StateId(GimbalState::kManual)] = &manual_state_;
  state_list_[StateId(GimbalState::kAimbot)] = &aimbot_state_;
  state_list_[StateId(GimbalState::kRecoveryAlign)] = &recovery_align_state_;
  state_list_[StateId(GimbalState::kRecoveryYawCentering)] = &recovery_yaw_centering_state_;
  state_list_[StateId(GimbalState::kFfVerify)] = &ff_verify_state_;
  state_list_[StateId(GimbalState::kIdent)] = &ident_state_;
  set_states(state_list_, static_cast<uint8_t>(GimbalState::kCount));
  Init();
}

void GimbalFsm::Init() {
  // reset(true) 会正常执行当前状态的退出流程，随后重新从 state_list_[0] 启动。
  if (is_started()) {
    reset(true);
  }
  request_ = {};
  output_ = {};
  context_ = {};
  pending_transition_reason_ = TransitionReason::kInitialized;
  has_entered_state_ = false;
  start();
}

GimbalFsmOutput GimbalFsm::Update(const GimbalFsmRequest &request) {
  // 先复制输入，保证事件处理期间所有状态读取的是同一帧快照。
  request_ = request;
  output_.state_changed = false;
  output_.transition_reason = TransitionReason::kNone;

  const GimbalUpdateEvent event{request_};
  // ETL receive() 在本调用内同步执行当前状态逻辑，并立即完成可能发生的状态切换。
  receive(event);

  // uint32_t 无符号减法天然支持系统 tick 单次回绕。
  output_.state_elapsed_ms = request_.tick_ms - context_.state_enter_tick_ms;
  return output_;
}

etl::fsm_state_id_t GimbalFsm::TransitionTo(const GimbalState state, const TransitionReason reason) {
  // 避免重复进入同一状态，也避免覆盖一次并未真正发生的迁移原因。
  if (is_started() && get_state_id() == StateId(state)) {
    return etl::ifsm_state::No_State_Change;
  }
  pending_transition_reason_ = reason;
  return StateId(state);
}

void GimbalFsm::EnterState(const GimbalState state) {
  // 构造函数首次进入 Disable 不算运行期状态变化。
  output_.state_changed = has_entered_state_ && output_.state != state;
  output_.state = state;
  output_.transition_reason = pending_transition_reason_;
  output_.state_elapsed_ms = 0U;
  context_.state_enter_tick_ms = request_.tick_ms;
  pending_transition_reason_ = TransitionReason::kNone;
  has_entered_state_ = true;
}

void GimbalFsm::WriteBaseOutput(const GimbalState state, const bool gimbal_enable, const bool align_to_chassis_forward,
                                const bool use_yaw_motor_feedback, const wheel_legged::TargetSource source,
                                const wheel_legged::GimbalTarget target,
                                const wheel_legged::GimbalTestProfile test_profile) {
  output_.state = state;
  output_.gimbal_enable = gimbal_enable;
  output_.align_to_chassis_forward = align_to_chassis_forward;
  output_.use_yaw_motor_feedback = use_yaw_motor_feedback;
  output_.active_target_source = source;
  output_.target = target;
  output_.test_profile = test_profile;
}

void GimbalFsm::WriteDisabledOutput() {
  WriteBaseOutput(GimbalState::kDisable, false, false, false, wheel_legged::TargetSource::kRc, request_.rc_target,
                  wheel_legged::GimbalTestProfile::kNormal);
}

void GimbalFsm::WriteStartupAlignOutput() {
  WriteBaseOutput(GimbalState::kStartupAlign, true, false, true, wheel_legged::TargetSource::kRc, request_.rc_target,
                  wheel_legged::GimbalTestProfile::kNormal);
}

void GimbalFsm::WriteManualOutput() {
  WriteBaseOutput(GimbalState::kManual, true, false, false, wheel_legged::TargetSource::kRc, request_.rc_target,
                  wheel_legged::GimbalTestProfile::kNormal);
}

void GimbalFsm::WriteAimbotOutput() {
  // 上层可以偏好 Host，但目标失效时必须无缝回退 RC，不能沿用过期自瞄数据。
  const bool use_host =
      request_.preferred_target_source == wheel_legged::TargetSource::kHost && request_.host_target_valid;
  const auto source = use_host ? wheel_legged::TargetSource::kHost : wheel_legged::TargetSource::kRc;
  const auto target = use_host ? request_.host_target : request_.rc_target;
  WriteBaseOutput(GimbalState::kAimbot, true, false, false, source, target, wheel_legged::GimbalTestProfile::kNormal);
}

void GimbalFsm::WriteRecoveryAlignOutput() {
  WriteBaseOutput(GimbalState::kRecoveryAlign, true, true, false, wheel_legged::TargetSource::kRc, request_.rc_target,
                  wheel_legged::GimbalTestProfile::kNormal);
}

void GimbalFsm::WriteRecoveryYawCenteringOutput() {
  WriteBaseOutput(GimbalState::kRecoveryYawCentering, true, false, true, wheel_legged::TargetSource::kRc,
                  request_.rc_target, wheel_legged::GimbalTestProfile::kNormal);
}

void GimbalFsm::WriteFfVerifyOutput() {
  WriteBaseOutput(GimbalState::kFfVerify, true, false, true, wheel_legged::TargetSource::kRc, request_.rc_target,
                  wheel_legged::GimbalTestProfile::kFfVerify);
}

void GimbalFsm::WriteIdentOutput() {
  WriteBaseOutput(GimbalState::kIdent, true, false, true, wheel_legged::TargetSource::kRc, request_.rc_target,
                  wheel_legged::GimbalTestProfile::kIdent);
}

template <typename TDerived, GimbalState TState>
etl::fsm_state_id_t GimbalStateBase<TDerived, TState>::on_event(const DisableEvent &) {
  // 独立 DisableEvent 供未来故障管理或紧急停机路径直接使用。
  return machine().TransitionTo(GimbalState::kDisable, TransitionReason::kPowerDisabled);
}

template <typename TDerived, GimbalState TState>
etl::fsm_state_id_t GimbalStateBase<TDerived, TState>::on_event(const ResetEvent &) {
  return machine().TransitionTo(GimbalState::kDisable, TransitionReason::kInitialized);
}

template <typename TDerived, GimbalState TState>
etl::fsm_state_id_t GimbalStateBase<TDerived, TState>::on_event_unknown(const etl::imessage &) {
  return etl::ifsm_state::No_State_Change;
}

#define WHEEL_LEGGED_DEFINE_GIMBAL_ENTRY_EXIT(ClassName, StateValue, WriteMethod) \
  etl::fsm_state_id_t ClassName::on_enter_state() {                               \
    machine().EnterState(StateValue);                                             \
    machine().WriteMethod();                                                      \
    return etl::ifsm_state::No_State_Change;                                      \
  }                                                                               \
  void ClassName::on_exit_state() {}

WHEEL_LEGGED_DEFINE_GIMBAL_ENTRY_EXIT(GimbalDisableState, GimbalState::kDisable, WriteDisabledOutput)
WHEEL_LEGGED_DEFINE_GIMBAL_ENTRY_EXIT(GimbalStartupAlignState, GimbalState::kStartupAlign, WriteStartupAlignOutput)
WHEEL_LEGGED_DEFINE_GIMBAL_ENTRY_EXIT(GimbalManualState, GimbalState::kManual, WriteManualOutput)
WHEEL_LEGGED_DEFINE_GIMBAL_ENTRY_EXIT(GimbalAimbotState, GimbalState::kAimbot, WriteAimbotOutput)
WHEEL_LEGGED_DEFINE_GIMBAL_ENTRY_EXIT(GimbalRecoveryAlignState, GimbalState::kRecoveryAlign, WriteRecoveryAlignOutput)
WHEEL_LEGGED_DEFINE_GIMBAL_ENTRY_EXIT(GimbalRecoveryYawCenteringState, GimbalState::kRecoveryYawCentering,
                                      WriteRecoveryYawCenteringOutput)
WHEEL_LEGGED_DEFINE_GIMBAL_ENTRY_EXIT(GimbalFfVerifyState, GimbalState::kFfVerify, WriteFfVerifyOutput)
WHEEL_LEGGED_DEFINE_GIMBAL_ENTRY_EXIT(GimbalIdentState, GimbalState::kIdent, WriteIdentOutput)

#undef WHEEL_LEGGED_DEFINE_GIMBAL_ENTRY_EXIT

// Disable 是唯一安全初态。输入恢复后先处理恢复/测试请求，普通操作必须经过 StartupAlign。
etl::fsm_state_id_t GimbalDisableState::on_event(const GimbalUpdateEvent &event) {
  const auto &request = *event.request;
  machine().WriteDisabledOutput();
  if (IsDisableRequested(request)) {
    return etl::ifsm_state::No_State_Change;
  }
  if (request.yaw_centering_required) {
    return machine().TransitionTo(GimbalState::kRecoveryYawCentering, TransitionReason::kRecoveryRequested);
  }
  if (request.chassis_fall_active) {
    return machine().TransitionTo(GimbalState::kRecoveryAlign, TransitionReason::kFallDetected);
  }
  const auto test_state = ResolveRequestedTest(machine(), request);
  if (test_state != etl::ifsm_state::No_State_Change) {
    return test_state;
  }
  return machine().TransitionTo(GimbalState::kStartupAlign, TransitionReason::kModeRequested);
}

// StartupAlign 建立可靠的初始反馈基准；完成前不会进入 Manual 或 Aimbot。
etl::fsm_state_id_t GimbalStartupAlignState::on_event(const GimbalUpdateEvent &event) {
  const auto &request = *event.request;
  machine().WriteStartupAlignOutput();
  const auto priority_state = ResolveSafetyOrRecovery(machine(), request);
  if (priority_state != etl::ifsm_state::No_State_Change) {
    return priority_state;
  }
  const auto test_state = ResolveRequestedTest(machine(), request);
  if (test_state != etl::ifsm_state::No_State_Change) {
    return test_state;
  }
  if (!request.startup_align_complete) {
    return etl::ifsm_state::No_State_Change;
  }
  return machine().TransitionTo(ResolveOperatorState(request), TransitionReason::kTargetReached);
}

// Manual 始终使用 RC 目标，但安全、恢复和测试请求仍可抢占。
etl::fsm_state_id_t GimbalManualState::on_event(const GimbalUpdateEvent &event) {
  const auto &request = *event.request;
  machine().WriteManualOutput();
  const auto priority_state = ResolveSafetyOrRecovery(machine(), request);
  if (priority_state != etl::ifsm_state::No_State_Change) {
    return priority_state;
  }
  const auto test_state = ResolveRequestedTest(machine(), request);
  if (test_state != etl::ifsm_state::No_State_Change) {
    return test_state;
  }
  if (request.requested_state == GimbalState::kAimbot) {
    return machine().TransitionTo(GimbalState::kAimbot, TransitionReason::kModeRequested);
  }
  return etl::ifsm_state::No_State_Change;
}

// Aimbot 每周期重新仲裁目标源，Host 数据失效后本周期立即回退 RC。
etl::fsm_state_id_t GimbalAimbotState::on_event(const GimbalUpdateEvent &event) {
  const auto &request = *event.request;
  machine().WriteAimbotOutput();
  const auto priority_state = ResolveSafetyOrRecovery(machine(), request);
  if (priority_state != etl::ifsm_state::No_State_Change) {
    return priority_state;
  }
  const auto test_state = ResolveRequestedTest(machine(), request);
  if (test_state != etl::ifsm_state::No_State_Change) {
    return test_state;
  }
  if (request.requested_state != GimbalState::kAimbot) {
    return machine().TransitionTo(GimbalState::kManual, TransitionReason::kModeRequested);
  }
  return etl::ifsm_state::No_State_Change;
}

// 底盘倒地期间保持云台对齐正前方；恢复结束后重新执行启动对中。
etl::fsm_state_id_t GimbalRecoveryAlignState::on_event(const GimbalUpdateEvent &event) {
  const auto &request = *event.request;
  machine().WriteRecoveryAlignOutput();
  if (IsDisableRequested(request)) {
    const auto reason = request.input_valid ? TransitionReason::kModeRequested : TransitionReason::kInputLost;
    return machine().TransitionTo(GimbalState::kDisable, reason);
  }
  if (request.yaw_centering_required) {
    return machine().TransitionTo(GimbalState::kRecoveryYawCentering, TransitionReason::kRecoveryRequested);
  }
  if (!request.chassis_fall_active) {
    return machine().TransitionTo(GimbalState::kStartupAlign, TransitionReason::kTargetReached);
  }
  return etl::ifsm_state::No_State_Change;
}

// yaw 回中优先于普通倒地对齐；回中完成后再依据底盘状态选择后续恢复路径。
etl::fsm_state_id_t GimbalRecoveryYawCenteringState::on_event(const GimbalUpdateEvent &event) {
  const auto &request = *event.request;
  machine().WriteRecoveryYawCenteringOutput();
  if (IsDisableRequested(request)) {
    const auto reason = request.input_valid ? TransitionReason::kModeRequested : TransitionReason::kInputLost;
    return machine().TransitionTo(GimbalState::kDisable, reason);
  }
  if (request.yaw_centering_required) {
    return etl::ifsm_state::No_State_Change;
  }
  if (request.chassis_fall_active) {
    return machine().TransitionTo(GimbalState::kRecoveryAlign, TransitionReason::kTargetReached);
  }
  return machine().TransitionTo(GimbalState::kStartupAlign, TransitionReason::kTargetReached);
}

// 测试态退出时统一经过 StartupAlign，避免测试控制器留下的目标或反馈模式直接带入操作态。
etl::fsm_state_id_t GimbalFfVerifyState::on_event(const GimbalUpdateEvent &event) {
  const auto &request = *event.request;
  machine().WriteFfVerifyOutput();
  const auto priority_state = ResolveSafetyOrRecovery(machine(), request);
  if (priority_state != etl::ifsm_state::No_State_Change) {
    return priority_state;
  }
  if (request.requested_state == GimbalState::kFfVerify) {
    return etl::ifsm_state::No_State_Change;
  }
  if (request.requested_state == GimbalState::kIdent) {
    return machine().TransitionTo(GimbalState::kIdent, TransitionReason::kModeRequested);
  }
  return machine().TransitionTo(GimbalState::kStartupAlign, TransitionReason::kModeRequested);
}

etl::fsm_state_id_t GimbalIdentState::on_event(const GimbalUpdateEvent &event) {
  const auto &request = *event.request;
  machine().WriteIdentOutput();
  const auto priority_state = ResolveSafetyOrRecovery(machine(), request);
  if (priority_state != etl::ifsm_state::No_State_Change) {
    return priority_state;
  }
  if (request.requested_state == GimbalState::kIdent) {
    return etl::ifsm_state::No_State_Change;
  }
  if (request.requested_state == GimbalState::kFfVerify) {
    return machine().TransitionTo(GimbalState::kFfVerify, TransitionReason::kModeRequested);
  }
  return machine().TransitionTo(GimbalState::kStartupAlign, TransitionReason::kModeRequested);
}

// 模板实现位于本 .cc 中，显式实例化确保每个具体状态都生成公共事件处理代码。
template class GimbalStateBase<GimbalDisableState, GimbalState::kDisable>;
template class GimbalStateBase<GimbalStartupAlignState, GimbalState::kStartupAlign>;
template class GimbalStateBase<GimbalManualState, GimbalState::kManual>;
template class GimbalStateBase<GimbalAimbotState, GimbalState::kAimbot>;
template class GimbalStateBase<GimbalRecoveryAlignState, GimbalState::kRecoveryAlign>;
template class GimbalStateBase<GimbalRecoveryYawCenteringState, GimbalState::kRecoveryYawCentering>;
template class GimbalStateBase<GimbalFfVerifyState, GimbalState::kFfVerify>;
template class GimbalStateBase<GimbalIdentState, GimbalState::kIdent>;

}  // namespace wheel_legged::fsm
