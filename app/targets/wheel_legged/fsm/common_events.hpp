#pragma once

#include <etl/message.h>

namespace wheel_legged::fsm {

/** @brief 任意状态立即进入安全失能状态。 */
struct DisableEvent final : etl::message<3> {};

/** @brief 复位状态机上下文并返回初始状态。 */
struct ResetEvent final : etl::message<4> {};

}  // namespace wheel_legged::fsm
