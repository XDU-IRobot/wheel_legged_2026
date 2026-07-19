#pragma once

#include <cstdint>

namespace wheel_legged {

/** Which physical pair of ToF sensors is powered and polled. */
enum class TofMode : std::uint8_t {
  kAutoJump = 0,
  kStairDescend = 1,
};

}  // namespace wheel_legged
