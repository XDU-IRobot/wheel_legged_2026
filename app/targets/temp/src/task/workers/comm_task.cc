/**
 * @file  task/workers/comm_task.cc
 * @brief 通信任务实现
 */

#include "task/api/task_c_api.h"
#include "task/api/task_entrypoints.hpp"
#include "task/runtime/task_context.hpp"
#include "task/runtime/task_debug.hpp"

#include "communication/communication.h"
#include <fdcan.h>
#include <librm.hpp>

namespace tasking {

namespace {

std::unique_ptr<rm::hal::stm32::FdCan> g_comm_can{};
std::unique_ptr<communication::Communication> g_comm_rx{};
std::shared_ptr<rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>> g_comm_dm_motor{};
bool g_comm_can_started{false};

struct CommCanSnapshot {
  communication::Command cmd{};
  bool link_ok{false};
  bool valid{false};
};

CommCanSnapshot SampleCommCanSnapshot() {
  CommCanSnapshot snapshot{};
  if (!g_comm_rx) {
    return snapshot;
  }

  snapshot.cmd = g_comm_rx->getCommand();
  snapshot.link_ok = g_comm_rx->online_status() == rm::device::Device::kOk;
  snapshot.valid = true;
  return snapshot;
}

void MapChassisCommandToCommInput(const CommCanSnapshot &snapshot, CommInputMsg &msg) {
  msg.link_ok = snapshot.link_ok;
  msg.remote_mid_force = false;
  msg.force_enable = false;
  msg.small_gyro_enable = false;
  msg.jump_trigger = false;

  switch (snapshot.cmd.chassis.state) {
    case communication::ChassisState::kUnable:
      break;
    case communication::ChassisState::kFollow:
      msg.force_enable = true;
      break;
    case communication::ChassisState::kRotate:
      msg.force_enable = true;
      msg.small_gyro_enable = true;
      break;
    case communication::ChassisState::kJump:
      msg.force_enable = true;
      msg.jump_trigger = true;
      break;
    default:
      break;
  }

  switch (snapshot.cmd.chassis.leg_length) {
    case communication::LegLength::kLow:
      msg.leg_length_mode = Fsm::LegLengthMode::kLow;
      break;
    case communication::LegLength::kNormal:
      msg.leg_length_mode = Fsm::LegLengthMode::kMid;
      break;
    case communication::LegLength::kHigh:
      msg.leg_length_mode = Fsm::LegLengthMode::kHigh;
      break;
    default:
      msg.leg_length_mode = Fsm::LegLengthMode::kLow;
      break;
  }

  // 仅保留前向输入通道，横向平移输入不再下发到控制链路。
  msg.vx_cmd = static_cast<float>(snapshot.cmd.chassis.move_y) / 127.f;
}

}  // namespace

/**
 * @brief 通信任务初始化
 * @note  当前使用云台/底盘 CAN 协议输入，无额外串口遥控器资源初始化。
 */
void CommTaskInit(TaskContext &ctx) {
  (void)ctx;

  if (!g_comm_can) {
    g_comm_can = std::make_unique<rm::hal::stm32::FdCan>(hfdcan2);
    g_comm_can->SetFilter(0, 0);
  }

  if (!g_comm_rx && g_comm_can) {
    g_comm_rx = std::make_unique<communication::Communication>(*g_comm_can);
  }

  if (!g_comm_dm_motor && g_comm_can) {
    g_comm_dm_motor = std::make_shared<rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>>(
        *g_comm_can, rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{
                         0x13, 0x03, 3.141593f, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}});
  }

  if (g_comm_can && !g_comm_can_started) {
    // 先完成设备注册，再启动中断接收，避免回调访问注册表竞争。
    g_comm_can->Begin();
    g_comm_can_started = true;
  }
}

/**
 * @brief 通信任务单步更新
 * @note  当前从云台/底盘 CAN 协议采样输入，向 FSM/扭矩任务输出统一消息。
 */
f32 yaw_pos;
void CommTaskUpdate(TaskContext &ctx) {
  static uint16_t seq = 0;

  CommInputMsg msg{};
  msg.h.tick_ms = osKernelGetTickCount();
  msg.h.seq = ++seq;
  msg.h.source_id = 1U;

  const CommCanSnapshot snapshot = SampleCommCanSnapshot();
  if (snapshot.valid) {
    MapChassisCommandToCommInput(snapshot, msg);
  }

  if (g_comm_dm_motor) {
    // Attempt to update motor status and read position
    // (In polling based update, the feedback is automatically updated via CAN
    // interrupt, just read it)
    msg.yaw_cmd = g_comm_dm_motor->pos();
  }

  (void)QueuePutLatest(ctx.queues.comm_to_fsm, msg);
  (void)QueuePutLatest(ctx.queues.comm_to_compute, msg);
  DebugUpdateComm(msg);
}

}  // namespace tasking

extern "C" void CommTaskInitC() { tasking::CommTaskInit(tasking::GetTaskContext()); }

extern "C" void CommTaskUpdateC() { tasking::CommTaskUpdate(tasking::GetTaskContext()); }
