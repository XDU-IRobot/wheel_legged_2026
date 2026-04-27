#pragma once

#include "task/messages/control_messages.hpp"

#include "cmsis_os.h"

#include <memory>

/**
 * @file  task/runtime/task_context.hpp
 * @brief 任务运行时共享上下文定义
 */

class Fsm;

namespace tasking {

/**
 * @brief 队列写入（最新值优先）
 * @note  若队列已满，先弹出一个旧消息再写入新消息。
 */
template <typename T>
inline bool QueuePutLatest(osMessageQueueId_t queue, const T &msg) {
  if (queue == nullptr) {
    return false;
  }

  osStatus_t status = osMessageQueuePut(queue, &msg, 0U, 0U);
  if (status == osErrorResource) {
    T dropped{};
    (void)osMessageQueueGet(queue, &dropped, nullptr, 0U);
    status = osMessageQueuePut(queue, &msg, 0U, 0U);
  }
  return status == osOK;
}

/**
 * @brief 队列无阻塞读取
 */
template <typename T>
inline bool QueueTryGet(osMessageQueueId_t queue, T &msg) {
  if (queue == nullptr) {
    return false;
  }
  return osMessageQueueGet(queue, &msg, nullptr, 0U) == osOK;
}

/**
 * @brief 四任务运行时上下文
 * @note  仅保存资源对象和队列句柄，不保存跨任务共享快照。
 */
struct TaskContext {
  struct Queues {
    osMessageQueueId_t comm_to_fsm{};
    osMessageQueueId_t comm_to_compute{};
    osMessageQueueId_t fsm_to_compute{};
    osMessageQueueId_t compute_to_fsm{};
    osMessageQueueId_t motor_to_compute{};
    osMessageQueueId_t compute_to_motor{};
  } queues{};

  struct MotorRuntime {
    // DM 当前使能锁存，避免重复发送 enable/disable 指令。
    bool dm_enabled{false};
  } motor_runtime{};

  std::unique_ptr<Fsm> fsm{};
};

/**
 * @brief 获取全局任务上下文实例
 */
TaskContext &GetTaskContext();

} // namespace tasking


