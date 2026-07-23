#include "include/fall_detector.hpp"

#include <algorithm>
#include <cmath>

namespace wheel_legged {

namespace {
inline bool ThetaInCircularRange(const float theta, const float min, const float max) {
  constexpr float kTwoPi = 2.0f * static_cast<float>(M_PI);
  float a = std::fmod(theta - min, kTwoPi);
  if (a < 0) a += kTwoPi;
  return a <= (max - min);
}
}  // namespace

FallDirection FallDetector::ClassifyDirection(const float ux, const float uy, const float threshold) {
  if (ux < -threshold) return FallDirection::kFront;
  if (ux > threshold) return FallDirection::kBack;
  if (uy < -threshold) return FallDirection::kLeft;
  if (uy > threshold) return FallDirection::kRight;
  return std::abs(ux) >= std::abs(uy) ? (ux < 0 ? FallDirection::kFront : FallDirection::kBack)
                                       : (uy < 0 ? FallDirection::kLeft : FallDirection::kRight);
}

bool FallDetector::CheckLegSafe(const LegSafetyContext& legs) {
  return ThetaInCircularRange(legs.theta_ll_rad, wheel_legged::params::active::chassis::kPostureThetaLegMinRad,
                              wheel_legged::params::active::chassis::kPostureThetaLegMaxRad) &&
         ThetaInCircularRange(legs.theta_lr_rad, wheel_legged::params::active::chassis::kPostureThetaLegMinRad,
                              wheel_legged::params::active::chassis::kPostureThetaLegMaxRad);
}

FallDetection FallDetector::Update(const PostureObservation& obs, const LegSafetyContext& legs,
                                   const uint32_t timestamp_ms) {
  FallDetection out{};

  out.sensor_valid = obs.sensor_valid;

  if (!obs.sensor_valid) {
    out.direction = locked_direction_;
    out.cause = FallCause::kSensorInvalid;
    return out;
  }

  // ── 1. 机身倾斜倒地候选（ux/uy 超阈值判断）──
  const float ux_abs = std::abs(obs.up_body_x);
  const float uy_abs = std::abs(obs.up_body_y);
  const bool raw_upright =
      ux_abs <= config_.params.upright_exit_uxy_abs && uy_abs <= config_.params.upright_exit_uxy_abs;
  const bool tilt_fall_candidate =
      ux_abs > config_.params.fall_enter_uxy_abs || uy_abs > config_.params.fall_enter_uxy_abs;
  // ── 2. 腿摆角越界倒地候选 ──
  // 当 pitch/roll 在正常范围但腿摆角超出安全区间时，也视为倒地
  // 这对应 chassis.cc 中 pitch_roll_valid_theta_invalid 分支的场景
  out.leg_configuration_safe = CheckLegSafe(legs);
  out.leg_fall_candidate = obs.sensor_valid && !out.leg_configuration_safe;

  // 综合倒地候选：机身倾斜或腿越界
  const bool fall_candidate = tilt_fall_candidate || out.leg_fall_candidate;

  // ── 3. 倒地计时 ──
  if (fall_candidate) {
    if (!prev_fall_candidate_) {
      fall_candidate_start_ms_ = timestamp_ms;
    }
    out.condition_hold_ms = timestamp_ms - fall_candidate_start_ms_;
  } else {
    out.condition_hold_ms = 0;
  }

  // ── 4. 倒地确认 ──
  out.fall_confirmed = fall_candidate && out.condition_hold_ms >= config_.params.fall_confirm_ms;

  // ── 5. 方向分类（仅在上升沿锁定）──
  if (out.fall_confirmed && !direction_locked_) {
    // 方向：纯腿越界标 Unknown，有机身倾斜则按 up_body 判定
    if (!tilt_fall_candidate && out.leg_fall_candidate) {
      out.direction = FallDirection::kUnknown;
    } else {
      out.direction = ClassifyDirection(obs.up_body_x, obs.up_body_y,
                                        config_.params.direction_threshold);
    }
    // 原因：腿越界优先（腿和机身倾斜同时存在时，腿是决定性因素）
    out.cause = out.leg_fall_candidate ? FallCause::kLegOutOfRange : FallCause::kTiltExceeded;
    locked_direction_ = out.direction;
    locked_cause_ = out.cause;
    direction_locked_ = true;
  } else if (out.fall_confirmed) {
    out.direction = locked_direction_;
    out.cause = locked_cause_;
  } else {
    direction_locked_ = false;
    locked_direction_ = FallDirection::kUnknown;
    locked_cause_ = FallCause::kNone;
    out.direction = FallDirection::kUnknown;
    out.cause = FallCause::kNone;
  }

  // ── 6. 直立确认（退出滞回）──
  const bool upright_candidate = raw_upright && out.leg_configuration_safe;

  if (upright_candidate) {
    if (!prev_upright_candidate_) {
      upright_start_ms_ = timestamp_ms;
    }
    out.upright_hold_ms = timestamp_ms - upright_start_ms_;
  } else {
    out.upright_hold_ms = 0;
  }

  out.body_raw_upright = raw_upright;
  out.body_upright_confirmed = out.upright_hold_ms >= config_.params.upright_confirm_ms;
  out.fall_candidate = fall_candidate;

  // ── 7. 更新持久状态 ──
  prev_fall_candidate_ = fall_candidate;
  prev_upright_candidate_ = upright_candidate;

  return out;
}

void FallDetector::Reset() {
  prev_fall_candidate_ = false;
  fall_candidate_start_ms_ = 0;
  prev_upright_candidate_ = false;
  upright_start_ms_ = 0;
  locked_direction_ = FallDirection::kUnknown;
  locked_cause_ = FallCause::kNone;
  direction_locked_ = false;
}

}  // namespace wheel_legged
