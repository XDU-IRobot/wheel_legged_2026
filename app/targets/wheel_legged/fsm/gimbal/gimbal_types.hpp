#pragma once

#include <cstdint>

#include "../common.hpp"
#include "../../include/fsm_common.hpp"

namespace wheel_legged::fsm {

/**
 * @brief 云台顶层状态。
 *
 * 状态只描述云台本身的控制行为，不携带摩擦轮、拨盘等发射机构权限。
 * 枚举值同时作为 ETL FSM 的 state id，必须从 0 开始连续排列，kCount 必须放在最后。
 */
enum class GimbalState : uint8_t {
  kDisable = 0,           ///< 云台电机无力，所有控制输出关闭。
  kStartupAlign,          ///< 上电或恢复后的启动对中，使用电机反馈完成初始姿态建立。
  kManual,                ///< 遥控器或键鼠直接控制云台目标角。
  kAimbot,                ///< 自瞄状态，优先采用有效的上位机目标，失效时回退到 RC 目标。
  kRecoveryAlign,         ///< 底盘倒地时云台对齐底盘正前方，为底盘自起提供稳定姿态。
  kRecoveryYawCentering,  ///< 底盘要求先回正 yaw 时，仅执行 yaw 电机回中流程。
  kFfVerify,              ///< 云台动力学前馈验证模式。
  kIdent,                 ///< 云台系统辨识模式。
  kCount,                 ///< 状态数量，不是可进入的状态。
};

/**
 * @brief 云台状态机单周期输入快照。
 *
 * 上层先完成输入设备仲裁和语义转换，再构造本结构。FSM 不读取 DR16、键鼠或图传原始数据。
 */
struct GimbalFsmRequest {
  bool input_valid{false};  ///< 当前控制输入是否在线可信；false 无条件进入 Disable。
  GimbalState requested_state{GimbalState::kDisable};  ///< 操作层请求的常规或测试状态。

  ///< Aimbot 状态期望采用的目标源；只有 host_target_valid 时 Host 才会真正生效。
  wheel_legged::TargetSource preferred_target_source{wheel_legged::TargetSource::kRc};
  wheel_legged::GimbalTarget rc_target{};    ///< 遥控器或键鼠积分得到的目标角。
  wheel_legged::GimbalTarget host_target{};  ///< 上位机自瞄目标角。
  bool host_target_valid{false};             ///< 上位机目标是否新鲜、在线且可使用。

  bool chassis_fall_active{false};  ///< 底盘正在倒地/自起流程中，请求云台进入 RecoveryAlign。
  bool yaw_centering_required{false};  ///< 底盘恢复前要求云台先完成 yaw 回中，优先级高于倒地对齐。
  bool startup_align_complete{false};  ///< 启动对中目标是否到达，由云台控制层判定。
  uint32_t tick_ms{0U};                ///< 单调递增毫秒时间戳，用于计算状态持续时间。
};

/** @brief 云台状态机统一输出；执行层只消费本结构，不需要了解状态迁移细节。 */
struct GimbalFsmOutput {
  GimbalState state{GimbalState::kDisable};  ///< 本周期最终状态。
  bool state_changed{false};                 ///< 本周期是否实际发生状态切换。

  bool gimbal_enable{false};             ///< 是否允许云台电机输出力矩。
  bool align_to_chassis_forward{false};  ///< 是否忽略普通目标，转而对齐底盘正前方。
  bool use_yaw_motor_feedback{false};    ///< yaw 闭环是否使用电机编码器反馈而非正常姿态反馈。

  wheel_legged::TargetSource active_target_source{wheel_legged::TargetSource::kRc};  ///< 实际选中的目标源。
  wheel_legged::GimbalTarget target{};  ///< 经过目标源仲裁后的最终目标角。
  wheel_legged::GimbalTestProfile test_profile{wheel_legged::GimbalTestProfile::kNormal};  ///< 测试控制算法。

  uint32_t state_elapsed_ms{0U};                                ///< 当前状态已持续的时间。
  TransitionReason transition_reason{TransitionReason::kNone};  ///< 本次状态切换原因；未切换时为 kNone。
};

/** @brief 仅由云台 FSM 持有的跨周期状态。 */
struct GimbalFsmContext {
  uint32_t state_enter_tick_ms{0U};                ///< 当前状态进入时刻。
  GimbalState resume_state{GimbalState::kManual};  ///< 预留：恢复流程结束后需要返回的操作状态。
};

}  // namespace wheel_legged::fsm
