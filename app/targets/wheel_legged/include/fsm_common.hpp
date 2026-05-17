#pragma once

#include <cstdint>

/**
 * @file  targets/wheel_legged/include/fsm_common.hpp
 * @brief 底盘与云台状态机共享的语义请求数据结构
 */

namespace wheel_legged {

/**
 * @brief 整车工作域请求
 */
enum class DomainRequest : uint8_t {
  kDisabled = 0,  ///< 关闭输出
  kService,       ///< 调试/维护域
  kCombat,        ///< 战斗域
};

/**
 * @brief 维护域下的安全策略
 */
enum class ServiceProfile : uint8_t {
  kChassisAndGimbalWithFire = 0,  ///< 底盘与云台均可工作，允许发射链路
  kChassisAndGimbalSafe,          ///< 底盘与云台工作，禁止发射链路
};

/**
 * @brief 腿长档位请求
 */
enum class LegProfile : uint8_t {
  kLow = 0,  ///< 低腿长
  kMid,      ///< 中腿长
  kHigh,     ///< 高腿长
};

/**
 * @brief 云台目标来源
 */
enum class TargetSource : uint8_t {
  kRc = 0,  ///< 遥控器积分目标
  kHost,    ///< 上位机视觉/自瞄目标
};

/**
 * @brief 战斗域下的子模式
 * @note  仅当 domain_request == kCombat 时有效。
 *        kAutoAimNoMove / kAutoAimWithMove 优先使用上位机自瞄目标，
 *        通信异常时自动降级为遥控器积分目标。
 */
enum class CombatProfile : uint8_t {
  kNormal = 0,       ///< 正常战斗：底盘有力，RC目标
  kAutoAimNoMove,    ///< 自瞄不移动：底盘无力（零输出），上位机目标
  kAutoAimWithMove,  ///< 自瞄+移动：底盘有力，上位机目标
};

/**
 * @brief 云台角度目标
 */
struct GimbalTarget {
  float yaw_rad{0.0f};    ///< 偏航目标角
  float pitch_rad{0.0f};  ///< 俯仰目标角
};

// ── 旧版统一语义请求（Phase 3 将替换为以下两个独立结构体）──
// @deprecated 请使用 ChassisFsmInput / GimbalFsmInput

/**
 * @brief 控制环生成的统一语义请求
 * @note  该结构是底盘 FSM 与云台 FSM 的共同输入，避免各模块直接解析遥控器原始量。
 */
struct ModeRequest {
  bool input_valid{false};                                 ///< 输入源是否在线且可信
  DomainRequest domain_request{DomainRequest::kDisabled};  ///< 整车工作域
  ServiceProfile service_profile{
      ServiceProfile::kChassisAndGimbalSafe};  ///< 维护域策略 (@todo 目前硬编码为 Safe，未来接入 DR16)
  LegProfile leg_request{LegProfile::kLow};    ///< 腿长档位请求

  bool spin_hold{false};             ///< 小陀螺保持请求
  bool jump_trigger{false};          ///< 跳跃边沿触发请求
  bool reset_yaw_request{false};     ///< R 键重置底盘正方向
  bool auto_jump_triggered{false};   ///< 自动跳跃触发（DYP 测距）
  float current_leg_length_m{0.0f};  ///< 当前平均腿长反馈 (回灌自上周期底盘输出)
  float theta_ll_rad{0.0f};          ///< 当前左腿摆角 (回灌自上周期底盘输出)
  float theta_lr_rad{0.0f};          ///< 当前右腿摆角 (回灌自上周期底盘输出)

  CombatProfile combat_profile{CombatProfile::kNormal};  ///< 战斗域子模式
  TargetSource target_source{TargetSource::kRc};         ///< 当前目标来源偏好
  GimbalTarget rc_target{};                              ///< 遥控器积分得到的目标
  GimbalTarget host_target{};                            ///< 上位机目标 (NUC 自瞄 CAN 反馈)
  bool host_target_valid{false};                         ///< 上位机目标是否有效 (NUC 启动且在线)

  bool fall_detected{false};          ///< 是否检测到倒地 (@todo 未接入 IMU 姿态判断，始终 false)
  uint32_t fall_detected_hold_ms{0};  ///< 倒地持续时间 (@todo 未接入)
  bool upright_stable{false};         ///< 是否已恢复稳定直立 (@todo 未接入，始终 true)

  uint32_t tick_ms{0};  ///< 当前系统时间戳
};

// ── 新版独立 FSM 输入（Phase 3 启用）──────────────────────────────

/**
 * @brief 底盘状态机专用输入
 * @note  仅包含底盘 FSM 状态决策所需的字段，不含云台相关字段。
 *
 * 状态切换依据：
 * - kDisabled → kLowLeg/kMidLeg/kHighLeg: domain_request != kDisabled 且
 *   combat_profile == kNormal（或 AutoAimWithMove）
 * - kDisabled → (底盘无力): domain_request == kCombat 且 combat_profile == kAutoAimNoMove
 *   此时底盘保持零输出，云台使用上位机自瞄
 * - kLowLeg/kMidLeg/kHighLeg 之间切换: leg_request 字段
 * - → kSpin: spin_hold == true 且不在跳台阶流程中
 * - → kJumpPrep: jump_trigger == true 且 domain_request != kCombat（战斗域下拨轮用于发射）
 * - → kRecoveryFallCheck: fall_detected == true（俯仰或横滚越界）
 * - → kStairClimb: 当前为 kHighLeg 且 theta_ll/lr > 阈值
 * - kJumpPrep → kJumpPush: 计时到达 kJumpPrepMs
 * - kJumpPush → kJumpRecover: 腿长到达 kJumpPushReachedLegLengthM 或超时
 * - kJumpRecover → kLowLeg: 计时到达 kJumpRecoverMs
 * - kRecoveryFallCheck → kRecoverySelfRight: 倒地持续 kRecoveryFallConfirmMs
 * - kRecoveryFallCheck → kLowLeg: 倒地恢复（未持续到确认时间）
 * - kRecoverySelfRight → kLowLeg: upright_stable == true
 * - kRecoverySelfRight → kDisabled: 超时 kRecoverySelfRightTimeoutMs
 * - kStairClimb → kStairClimbDone: 腿到位 + kStairClimbDurationMs 到
 * - kStairClimbDone → kHighLeg: pitch 稳定 kStairClimbPitchStableMs
 */
struct ChassisFsmInput {
  bool input_valid{false};                                 ///< 输入源是否在线且可信
  DomainRequest domain_request{DomainRequest::kDisabled};  ///< 整车工作域
  LegProfile leg_request{LegProfile::kLow};                ///< 腿长档位请求
  CombatProfile combat_profile{CombatProfile::kNormal};    ///< 战斗域子模式
  bool spin_hold{false};                                   ///< 小陀螺保持请求
  bool spin_exit_yaw_aligned{false};                       ///< 小陀螺退出：yaw 已对齐目标方向
  bool jump_trigger{false};                                ///< 跳跃边沿触发请求
  float current_leg_length_m{0.0f};                        ///< 当前平均腿长反馈
  float theta_ll_rad{0.0f};                                ///< 当前左腿摆角
  float theta_lr_rad{0.0f};                                ///< 当前右腿摆角
  bool fall_detected{false};                               ///< 是否检测到倒地
  uint32_t fall_detected_hold_ms{0};                       ///< 倒地持续时间
  bool upright_stable{false};                              ///< 是否已恢复稳定直立
  bool off_ground{false};                                  ///< 是否离地（支撑力过低）
  bool stair_climb_ready_for_done{false};                  ///< 上台阶回摆到位，可以进入 kStairClimbDone
  uint32_t tick_ms{0};                                     ///< 当前系统时间戳
};

/**
 * @brief 云台状态机专用输入
 * @note  仅包含云台 FSM 状态决策所需的字段。
 *
 * 状态切换依据：
 * - kDisabled → kStartupAlign: input_valid 首次变为 true（domain != kDisabled）
 * - kStartupAlign → kServiceWithFire/kServiceSafe/kCombat: startup_align_complete == true
 *   - domain_request == kCombat → kCombat
 *   - domain_request == kService 且 service_profile == kChassisAndGimbalWithFire → kServiceWithFire
 *   - domain_request == kService 且 service_profile == kChassisAndGimbalSafe → kServiceSafe
 * - kCombat / kService / kStartupAlign -> kDisabled: input_valid == false or domain == kDisabled
 * - -> kRecoveryAlign: chassis_recovery_active == true (chassis in recovery)
 * - kRecoveryAlign -> normal mode: chassis_recovery_active == false
 *
 * Gimbal target selection logic:
 *   - combat_profile == kAutoAim* prefers host_target
 *     host_target_valid -> TargetSource::kHost, otherwise falls back to TargetSource::kRc
 *   - combat_profile == kNormal uses TargetSource::kRc
 * - fire_allowed: true when kCombat or kServiceWithFire
 * - align_to_chassis_forward: true when kRecoveryAlign or kStartupAlign
 */
struct GimbalFsmInput {
  bool input_valid{false};                                                ///< 输入源是否在线且可信
  DomainRequest domain_request{DomainRequest::kDisabled};                 ///< 整车工作域
  ServiceProfile service_profile{ServiceProfile::kChassisAndGimbalSafe};  ///< 维护域策略
  CombatProfile combat_profile{CombatProfile::kNormal};                   ///< 战斗域子模式
  TargetSource target_source{TargetSource::kRc};                          ///< 当前目标来源偏好
  GimbalTarget rc_target{};                                               ///< 遥控器积分目标
  GimbalTarget host_target{};                                             ///< 上位机目标 (NUC 自瞄 CAN 反馈)
  bool host_target_valid{false};        ///< 上位机目标是否有效 (NUC 启动且在线)
  bool chassis_recovery_active{false};  ///< 底盘是否处于恢复流程
  bool startup_align_complete{false};   ///< 启动偏航归中是否完成
};

}  // namespace wheel_legged
