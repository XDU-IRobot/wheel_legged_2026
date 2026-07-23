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

FallDirection FallDetector::ClassifyDirection(const float ux, const float uy, const float uz,
                                              const float dominance_ratio, const float inverted_cos) {
  if (uz < inverted_cos) {
    return FallDirection::kInverted;
  }
  const float ax = std::abs(ux);
  const float ay = std::abs(uy);
  if (ax > ay && ax > dominance_ratio * std::max(ax, ay)) {
    return ux < 0.0f ? FallDirection::kFront : FallDirection::kBack;
  }
  if (ay > ax && ay > dominance_ratio * std::max(ax, ay)) {
    return uy < 0.0f ? FallDirection::kLeft : FallDirection::kRight;
  }
  return FallDirection::kDiagonal;
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

  // ── 1. 机身倾斜倒地候选 ──
  const bool raw_upright = obs.up_body_z >= config_.params.upright_exit_cos;
  const bool tilt_fall_candidate = obs.up_body_z < config_.params.fall_enter_cos;
  const bool severe_candidate = obs.up_body_z < config_.params.severe_fall_cos;

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
  out.severe_fall = severe_candidate;
  if (severe_candidate) {
    out.fall_confirmed = out.condition_hold_ms >= config_.params.severe_confirm_ms;
  } else {
    out.fall_confirmed = fall_candidate && out.condition_hold_ms >= config_.params.fall_confirm_ms;
  }

  // ── 5. 方向分类（仅在上升沿锁定）──
  if (out.fall_confirmed && !direction_locked_) {
    if (!tilt_fall_candidate && out.leg_fall_candidate) {
      // 腿越界但机身直立 → 方向暂标 Unknown，由现有 theta 恢复逻辑处理
      out.direction = FallDirection::kUnknown;
      out.cause = FallCause::kLegOutOfRange;
    } else {
      out.direction = ClassifyDirection(obs.up_body_x, obs.up_body_y, obs.up_body_z,
                                        config_.params.direction_dominance_ratio, config_.params.inverted_tilt_cos);
      out.cause = severe_candidate ? FallCause::kSevereTilt : FallCause::kTiltExceeded;
    }
    locked_direction_ = out.direction;
    direction_locked_ = true;
  } else if (out.fall_confirmed) {
    out.direction = locked_direction_;
    // 保持锁定时的 cause
  } else {
    direction_locked_ = false;
    locked_direction_ = FallDirection::kUnknown;
    out.direction = FallDirection::kUnknown;
    out.cause = FallCause::kNone;
  }

  // ── 6. 直立确认（退出滞回）──
  const bool upright_candidate =
      raw_upright && out.leg_configuration_safe && (obs.gyro_norm_rad_s < config_.params.upright_gyro_max_rad_s);

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
  direction_locked_ = false;
}

}  // namespace wheel_legged
