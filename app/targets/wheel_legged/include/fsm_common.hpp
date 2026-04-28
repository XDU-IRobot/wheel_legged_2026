#pragma once

#include <cstdint>

namespace wheel_legged {

enum class DomainRequest : uint8_t {
  kDisabled = 0,
  kService,
  kCombat,
};

enum class ServiceProfile : uint8_t {
  kChassisAndGimbalWithFire = 0,
  kChassisAndGimbalSafe,
};

enum class LegProfile : uint8_t {
  kLow = 0,
  kMid,
  kHigh,
};

enum class TargetSource : uint8_t {
  kRc = 0,
  kHost,
};

struct GimbalTarget {
  float yaw_rad{0.0f};
  float pitch_rad{0.0f};
};

struct ModeRequest {
  bool input_valid{false};
  DomainRequest domain_request{DomainRequest::kDisabled};
  ServiceProfile service_profile{ServiceProfile::kChassisAndGimbalWithFire};
  LegProfile leg_request{LegProfile::kLow};

  bool spin_hold{false};
  bool jump_trigger{false};
  bool fire_request{false};
  float current_leg_length_m{0.0f};

  TargetSource target_source{TargetSource::kRc};
  GimbalTarget rc_target{};
  GimbalTarget host_target{};
  bool host_target_valid{false};

  bool fall_detected{false};
  uint32_t fall_detected_hold_ms{0};
  bool upright_stable{false};

  uint32_t tick_ms{0};
};

}  // namespace wheel_legged
