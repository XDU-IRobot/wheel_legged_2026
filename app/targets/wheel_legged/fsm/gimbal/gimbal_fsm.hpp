#pragma once

#include <etl/fsm.h>

#include "gimbal_states.hpp"
#include "gimbal_types.hpp"

namespace wheel_legged::fsm {

/**
 * @brief 基于 ETL 的云台状态机。
 *
 * 调用约定：控制循环每周期构造一次完整 GimbalFsmRequest 并调用 Update()；状态机同步完成
 * 迁移并返回 GimbalFsmOutput。对象内部不使用动态内存，也不持有外部输入指针。
 *
 * 状态迁移的公共优先级为：
 * 1. 输入失效或显式 Disable；
 * 2. 恢复 yaw 回中；
 * 3. 底盘倒地对齐；
 * 4. Ident/FfVerify 测试状态；
 * 5. Manual/Aimbot 操作状态。
 */
class GimbalFsm final : public etl::fsm {
 public:
  GimbalFsm();

  /** @brief 清空请求、输出和跨周期上下文，并从 Disable 重新启动。 */
  void Init();

  /** @brief 同步处理一帧请求，返回迁移完成后的输出。 */
  GimbalFsmOutput Update(const GimbalFsmRequest &request);

  [[nodiscard]] const GimbalFsmOutput &output() const { return output_; }
  [[nodiscard]] GimbalState state() const { return output_.state; }
  [[nodiscard]] const GimbalFsmRequest &request() const { return request_; }
  [[nodiscard]] GimbalFsmContext &context() { return context_; }

  /** @brief 记录迁移原因并返回 ETL 目标 state id；请求当前状态时返回 No_State_Change。 */
  etl::fsm_state_id_t TransitionTo(GimbalState state, TransitionReason reason);

  /** @brief 状态 on_enter 的公共处理：更新时间、状态变化标志和迁移原因。 */
  void EnterState(GimbalState state);

  // 各状态的输出策略集中在 FSM 中，状态类只负责选择迁移和调用对应策略。
  void WriteDisabledOutput();
  void WriteStartupAlignOutput();
  void WriteManualOutput();
  void WriteAimbotOutput();
  void WriteRecoveryAlignOutput();
  void WriteRecoveryYawCenteringOutput();
  void WriteFfVerifyOutput();
  void WriteIdentOutput();

 private:
  /** @brief 写入所有状态共享的输出字段，避免旧值跨状态残留。 */
  void WriteBaseOutput(GimbalState state, bool gimbal_enable, bool align_to_chassis_forward,
                       bool use_yaw_motor_feedback, wheel_legged::TargetSource source,
                       wheel_legged::GimbalTarget target, wheel_legged::GimbalTestProfile test_profile);

  // ETL 要求状态对象在 FSM 生命周期内保持有效，因此全部按值静态持有。
  GimbalDisableState disable_state_{};
  GimbalStartupAlignState startup_align_state_{};
  GimbalManualState manual_state_{};
  GimbalAimbotState aimbot_state_{};
  GimbalRecoveryAlignState recovery_align_state_{};
  GimbalRecoveryYawCenteringState recovery_yaw_centering_state_{};
  GimbalFfVerifyState ff_verify_state_{};
  GimbalIdentState ident_state_{};
  etl::ifsm_state *state_list_[static_cast<uint8_t>(GimbalState::kCount)]{};  ///< state id 到状态对象的映射。

  GimbalFsmRequest request_{};  ///< 当前周期输入快照。
  GimbalFsmOutput output_{};    ///< 当前周期输出及最近状态信息。
  GimbalFsmContext context_{};  ///< 仅由 FSM 修改的跨周期上下文。
  TransitionReason pending_transition_reason_{TransitionReason::kInitialized};  ///< 下一次 on_enter 消费的原因。
  bool has_entered_state_{false};  ///< 区分首次启动进入与真实状态切换。
};

}  // namespace wheel_legged::fsm
