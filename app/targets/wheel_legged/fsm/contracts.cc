#include "chassis/chassis_fsm.hpp"
#include "common_events.hpp"
#include "gimbal/gimbal_fsm.hpp"

#include <type_traits>

namespace wheel_legged::fsm {

static_assert(static_cast<uint8_t>(ChassisState::kCount) == 9U);
static_assert(static_cast<uint8_t>(GimbalState::kCount) == 8U);
static_assert(std::is_trivially_copyable_v<ChassisFsmRequest>);
static_assert(std::is_trivially_copyable_v<ChassisFsmOutput>);
static_assert(std::is_trivially_copyable_v<GimbalFsmRequest>);
static_assert(std::is_trivially_copyable_v<GimbalFsmOutput>);

}  // namespace wheel_legged::fsm
