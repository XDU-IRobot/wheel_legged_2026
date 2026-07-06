#pragma once

#include <etl/message.h>

#include "gimbal_types.hpp"

namespace wheel_legged::fsm {

/**
 * @brief 500Hz 云台状态机更新事件。
 *
 * 事件由 GimbalFsm::Update 在栈上创建并同步分发，因此这里只保存请求的只读指针，
 * 不复制整份输入，也不能把该指针保存到本周期之外。
 */
struct GimbalUpdateEvent final : etl::message<2> {
  explicit GimbalUpdateEvent(const GimbalFsmRequest &request_value) : request(&request_value) {}

  const GimbalFsmRequest *request;  ///< 仅在本次 receive() 调用期间有效。
};

}  // namespace wheel_legged::fsm
