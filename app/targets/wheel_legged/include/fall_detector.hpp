#pragma once

#include <cstdint>

#include "params.hpp"
#include "posture_observer.hpp"

namespace wheel_legged {

/// @brief 倒地方向（每周期重分类，支持倒地期间方向切换）
enum class FallDirection : uint8_t {
  kUnknown = 0,
  kFront = 1,  ///< 前趴
  kBack = 2,   ///< 后躺
  kLeft = 3,   ///< 左侧躺
  kRight = 4,  ///< 右侧躺
};

/// @brief 倒地原因
enum class FallCause : uint8_t {
  kNone = 0,
  kTiltExceeded = 1,   ///< 机身倾斜超限（|ux| 或 |uy| 超过进入阈值）
  kLegOutOfRange = 2,  ///< 腿摆角越界（pitch/roll 正常，但 theta 超限）
  kSensorInvalid = 3,  ///< 传感器数据无效
};

/// @brief 腿状态上下文
struct LegSafetyContext {
  float theta_ll_rad{0.0f};
  float theta_lr_rad{0.0f};
  float left_leg_length_m{0.0f};
  float right_leg_length_m{0.0f};
};

/// @brief 倒地检测器输出
struct FallDetection {
  // 原始信号
  bool body_raw_upright{true};

  // 滞回确认
  bool body_upright_confirmed{true};
  bool fall_candidate{false};
  bool fall_confirmed{false};

  // 腿
  bool leg_fall_candidate{false};  ///< 腿摆角超限（pitch/roll 正常但 theta 越界）
  bool leg_configuration_safe{true};

  // 传感器
  bool sensor_valid{false};

  // 方向与原因（每周期更新）
  FallDirection direction{FallDirection::kUnknown};
  FallCause cause{FallCause::kNone};

  // 计时
  uint32_t condition_hold_ms{0};
  uint32_t upright_hold_ms{0};
};

/// @brief 基于 up_body + 腿状态的滞回倒地检测器
///
/// 倒地候选 = 机身倾斜超限 OR 腿摆角越界（机身直立但腿不在安全范围）。
/// 腿越界进来的路径保留现有 theta 恢复逻辑，不改动机身倾斜自起逻辑。
class FallDetector {
 public:
  struct Config {
    ::wheel_legged::params::FallDetectorParams params;
  };

  void Init(const Config& config) { config_ = config; }

  FallDetection Update(const PostureObservation& obs, const LegSafetyContext& legs, uint32_t timestamp_ms);

  void Reset();

 private:
  Config config_{};

  bool prev_fall_candidate_{false};
  uint32_t fall_candidate_start_ms_{0};

  bool prev_upright_candidate_{false};
  uint32_t upright_start_ms_{0};

  static FallDirection ClassifyDirection(float ux, float uy, float threshold);
  static bool CheckLegSafe(const LegSafetyContext& legs);
};

}  // namespace wheel_legged
