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
 * @brief 云台角度目标
 */
struct GimbalTarget {
  float yaw_rad{0.0f};    ///< 偏航目标角
  float pitch_rad{0.0f};  ///< 俯仰目标角
};

/**
 * @brief 控制环生成的统一语义请求
 * @note  该结构是底盘 FSM 与云台 FSM 的共同输入，避免各模块直接解析遥控器原始量。
 */
struct ModeRequest {
  bool input_valid{false};                                                    ///< 输入源是否在线且可信
  DomainRequest domain_request{DomainRequest::kDisabled};                     ///< 整车工作域
  ServiceProfile service_profile{ServiceProfile::kChassisAndGimbalWithFire};  ///< 维护域策略
  LegProfile leg_request{LegProfile::kLow};                                   ///< 腿长档位请求

  bool spin_hold{false};             ///< 小陀螺保持请求
  bool jump_trigger{false};          ///< 跳跃边沿触发请求
  bool fire_request{false};          ///< 发射请求
  float current_leg_length_m{0.0f};  ///< 当前平均腿长反馈

  TargetSource target_source{TargetSource::kRc};  ///< 当前目标来源偏好
  GimbalTarget rc_target{};                       ///< 遥控器积分得到的目标
  GimbalTarget host_target{};                     ///< 上位机目标
  bool host_target_valid{false};                  ///< 上位机目标是否有效

  bool fall_detected{false};          ///< 是否检测到倒地
  uint32_t fall_detected_hold_ms{0};  ///< 倒地持续时间
  bool upright_stable{false};         ///< 是否已恢复稳定直立

  uint32_t tick_ms{0};  ///< 当前系统时间戳
};

}  // namespace wheel_legged
