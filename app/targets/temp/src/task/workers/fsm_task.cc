#include "task/runtime/task_context.hpp"
#include "task/runtime/task_debug.hpp"
#include "task/api/task_entrypoints.hpp"
#include "task/api/task_c_api.h"

#include "fsm/fsm.hpp"
#include "cmsis_os.h"

namespace tasking {

namespace {

constexpr uint32_t kCommInputTimeoutMs = 30U;

}

/**
 * @brief 状态机任务初始化
 */
void FsmTaskInit(TaskContext &ctx) {
  if (!ctx.fsm) {
    ctx.fsm = std::make_unique<Fsm>();
    ctx.fsm->Init();
  }
}

/**
 * @brief 状态机任务单步更新
 * @note  消费通信输入并输出统一模式动作。
 */
void FsmTaskUpdate(TaskContext &ctx) {
  if (!ctx.fsm) {
    return;
  }

  // 缓存最近一次通信输入，避免因单拍未取到新消息而误回退无力态。
  static bool has_comm_latched = false;
  static CommInputMsg comm_latched{};
  static bool has_state_latched = false;
  static StateEstimateMsg state_latched{};

  CommInputMsg comm_rx{};
  if (QueueTryGet(ctx.queues.comm_to_fsm, comm_rx)) {
    comm_latched = comm_rx;
    has_comm_latched = true;
  }

  StateEstimateMsg state_rx{};
  if (QueueTryGet(ctx.queues.compute_to_fsm, state_rx)) {
    state_latched = state_rx;
    has_state_latched = true;
  }

  const uint32_t now_ms = osKernelGetTickCount();
  const bool comm_fresh = has_comm_latched && ((now_ms - comm_latched.h.tick_ms) <= kCommInputTimeoutMs);

  Fsm::Input input{};
  input.input_valid = comm_fresh && comm_latched.link_ok;
  input.force_enable = comm_fresh && comm_latched.link_ok && comm_latched.force_enable;
  input.leg_length_mode =
      (comm_fresh && comm_latched.link_ok) ? comm_latched.leg_length_mode : Fsm::LegLengthMode::kLow;
  input.small_gyro_enable = comm_fresh && comm_latched.link_ok && comm_latched.small_gyro_enable;
  input.jump_trigger = comm_fresh && comm_latched.link_ok && comm_latched.jump_trigger;
  input.current_leg_length_m =
      has_state_latched ? 0.5f * (state_latched.current.l_l + state_latched.current.l_r) : 0.0f;
  input.tick_ms = now_ms;

  const Fsm::Output out = ctx.fsm->Update(input);

  static uint16_t seq = 0;
  FsmOutputMsg msg{};
  msg.h.tick_ms = osKernelGetTickCount();
  msg.h.seq = ++seq;
  msg.h.source_id = 2U;
  msg.mode = out.mode;
  msg.state_changed = out.state_changed;
  msg.control = out.control;

  (void)QueuePutLatest(ctx.queues.fsm_to_compute, msg);
  DebugUpdateFsm(input, out, msg);
}

}  // namespace tasking

extern "C" void FsmTaskInitC() { tasking::FsmTaskInit(tasking::GetTaskContext()); }

extern "C" void FsmTaskUpdateC() { tasking::FsmTaskUpdate(tasking::GetTaskContext()); }
