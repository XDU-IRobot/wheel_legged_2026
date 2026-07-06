#pragma once

#include <cstdint>

namespace wheel_legged::fsm {

/** @brief 状态迁移原因，用于调试、UI 和故障回溯。 */
enum class TransitionReason : uint8_t {
  kNone = 0,
  kInitialized,
  kModeRequested,
  kInputLost,
  kPowerDisabled,
  kFallDetected,
  kRecoveryRequested,
  kTaskFinished,
  kTaskCancelled,
  kTaskAborted,
  kReleased,
  kTargetReached,
  kYawAligned,
  kLanded,
  kTimeout,
};

/**
 * @brief 发射机构权限。
 * @note  发射权限与底盘/云台状态正交，不再通过 Combat 或 Service 状态隐式表达。
 */
struct WeaponPermission {
  bool friction_enabled{false};
  bool loader_enabled{false};
  bool fire_allowed{false};
};

}  // namespace wheel_legged::fsm
