#pragma once

#include <etl/message.h>

#include "chassis_types.hpp"

namespace wheel_legged::fsm {

/** @brief 500Hz 底盘状态机更新事件；同步处理，不进入消息队列。 */
struct ChassisUpdateEvent final : etl::message<1> {
  explicit ChassisUpdateEvent(const ChassisFsmRequest &request_value) : request(&request_value) {}

  const ChassisFsmRequest *request;
};

}  // namespace wheel_legged::fsm
