#include "cmsis_os.h"
#include "task/api/task_c_api.h"
#include "task/runtime/task_context.hpp"

namespace tasking {

static TaskContext g_task_ctx;
static bool g_queues_initialized = false;

TaskContext &GetTaskContext() { return g_task_ctx; }

namespace {

void InitQueues(TaskContext &ctx) {
  if (ctx.queues.comm_to_fsm == nullptr) {
    ctx.queues.comm_to_fsm = osMessageQueueNew(1U, sizeof(CommInputMsg), nullptr);
  }
  if (ctx.queues.comm_to_compute == nullptr) {
    ctx.queues.comm_to_compute =
        osMessageQueueNew(1U, sizeof(CommInputMsg), nullptr);
  }
  if (ctx.queues.fsm_to_compute == nullptr) {
    ctx.queues.fsm_to_compute =
        osMessageQueueNew(1U, sizeof(FsmOutputMsg), nullptr);
  }
  if (ctx.queues.compute_to_fsm == nullptr) {
    ctx.queues.compute_to_fsm =
        osMessageQueueNew(1U, sizeof(StateEstimateMsg), nullptr);
  }
  if (ctx.queues.motor_to_compute == nullptr) {
    ctx.queues.motor_to_compute =
        osMessageQueueNew(1U, sizeof(MotorFeedbackMsg), nullptr);
  }
  if (ctx.queues.compute_to_motor == nullptr) {
    ctx.queues.compute_to_motor =
        osMessageQueueNew(1U, sizeof(TorqueCmd6), nullptr);
  }
}

void EnsureQueuesInitialized() {
  if (g_queues_initialized) {
    return;
  }

  TaskContext &ctx = GetTaskContext();
  InitQueues(ctx);
  g_queues_initialized = true;
}

} // namespace
} // namespace tasking

extern "C" void TaskQueuesInit() {
  tasking::EnsureQueuesInitialized();
}




