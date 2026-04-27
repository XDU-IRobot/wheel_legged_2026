/**
 * @file  task/workers/motor_output_task.cc
 * @brief 电机输出任务实现
 */

#include "task/runtime/task_context.hpp"
#include "task/runtime/task_debug.hpp"
#include "task/api/task_entrypoints.hpp"
#include "task/api/task_c_api.h"

#include "fdcan.h"

#include <chrono>
#include <librm.hpp>

namespace tasking {

namespace {

constexpr uint32_t kTorqueMsgTimeoutTicks = 4U;
constexpr double kJointCanTxLimitHz = 7000.0;
constexpr double kWheelCanTxLimitHz = 2000.0;
constexpr uint32_t kJointCanProcessBudgetPerUpdate = 8U;
constexpr uint32_t kWheelCanProcessBudgetPerUpdate = 4U;
constexpr auto kJointCanProcessStep = std::chrono::duration<double>(1.0 / kJointCanTxLimitHz);
constexpr auto kWheelCanProcessStep = std::chrono::duration<double>(1.0 / kWheelCanTxLimitHz);

inline int16_t ClampToI16(float value) {
  if (value > 16000.0f) {
    return 16000;
  }
  if (value < -16000.0f) {
    return -16000;
  }
  return static_cast<int16_t>(value);
}

struct DmJoints {
  std::shared_ptr<rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>> lf{};
  std::shared_ptr<rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>> lb{};
  std::shared_ptr<rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>> rf{};
  std::shared_ptr<rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>> rb{};

  [[nodiscard]] bool Valid() const { return lf && lb && rf && rb; }
};

std::unique_ptr<rm::hal::ThrottledCan<>> g_joint_can{};
std::unique_ptr<rm::hal::ThrottledCan<>> g_wheel_can{};
DmJoints g_dm_joints{};
std::shared_ptr<rm::device::M3508> g_left_wheel{};
std::shared_ptr<rm::device::M3508> g_right_wheel{};
bool g_joint_can_started{false};
bool g_wheel_can_started{false};

void InitMotorsIfNeeded() {
  if (!g_joint_can) {
    g_joint_can = std::make_unique<rm::hal::ThrottledCan<>>(kJointCanTxLimitHz, hfdcan1);
    g_joint_can->SetFilter(0, 0);
  }

  if (!g_wheel_can) {
    g_wheel_can = std::make_unique<rm::hal::ThrottledCan<>>(kWheelCanTxLimitHz, hfdcan3);
    g_wheel_can->SetFilter(0, 0);
  }

  if (!g_dm_joints.Valid()) {
    g_dm_joints.lf = std::make_shared<rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>>(
        *g_joint_can,
        rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{
            0x17, 0x07, 3.141593f, 45.f, 54.f, {0, 500}, {0, 10}});
    g_dm_joints.lb = std::make_shared<rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>>(
        *g_joint_can,
        rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{
            0x14, 0x04, 3.141593f, 45.f, 54.f, {0, 500}, {0, 10}});
    g_dm_joints.rf = std::make_shared<rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>>(
        *g_joint_can,
        rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{
            0x16, 0x06, 3.141593f, 45.f, 54.f, {0, 500}, {0, 10}});
    g_dm_joints.rb = std::make_shared<rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>>(
        *g_joint_can,
        rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{
            0x15, 0x05, 3.141593f, 45.f, 54.f, {0, 500}, {0, 10}});
  }

  if (!g_left_wheel) {
    g_left_wheel = std::make_shared<rm::device::M3508>(*g_wheel_can, 0x06);
  }
  if (!g_right_wheel) {
    g_right_wheel = std::make_shared<rm::device::M3508>(*g_wheel_can, 0x05);
  }

  if (g_joint_can && !g_joint_can_started) {
    // 先完成设备注册，再启动中断接收，避免回调访问注册表竞争。
    g_joint_can->Begin();
    g_joint_can_started = true;
  }

  if (g_wheel_can && !g_wheel_can_started) {
    g_wheel_can->Begin();
    g_wheel_can_started = true;
  }
}


void SendDmEnable(bool &dm_enabled) {
  if (dm_enabled) {
    return;
  }

  if (!g_dm_joints.Valid()) {
    return;
  }
  g_dm_joints.lf->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  g_dm_joints.lf->SendInstruction(rm::device::DmMotorInstructions::kEnable);
  g_dm_joints.lb->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  g_dm_joints.lb->SendInstruction(rm::device::DmMotorInstructions::kEnable);
  g_dm_joints.rf->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  g_dm_joints.rf->SendInstruction(rm::device::DmMotorInstructions::kEnable);
  g_dm_joints.rb->SendInstruction(rm::device::DmMotorInstructions::kClearError);
  g_dm_joints.rb->SendInstruction(rm::device::DmMotorInstructions::kEnable);
  dm_enabled = true;
}

void SendDmDisable(bool &dm_enabled) {
  if (!dm_enabled) {
    return;
  }

  if (!g_dm_joints.Valid()) {
    return;
  }

  g_dm_joints.lf->SendInstruction(rm::device::DmMotorInstructions::kDisable);
  g_dm_joints.lb->SendInstruction(rm::device::DmMotorInstructions::kDisable);
  g_dm_joints.rf->SendInstruction(rm::device::DmMotorInstructions::kDisable);
  g_dm_joints.rb->SendInstruction(rm::device::DmMotorInstructions::kDisable);
  dm_enabled = false;
}

void SendDmMitCommand(float lf_tau, float lb_tau, float rf_tau, float rb_tau) {
  if (!g_dm_joints.Valid()) {
    return;
  }

  g_dm_joints.lb->SetMitCommand(0.0f, 0.0f, lb_tau, 0.0f, 0.0f);
  g_dm_joints.lf->SetMitCommand(0.0f, 0.0f, lf_tau, 0.0f, 0.0f);
  g_dm_joints.rb->SetMitCommand(0.0f, 0.0f, rb_tau, 0.0f, 0.0f);
  g_dm_joints.rf->SetMitCommand(0.0f, 0.0f, rf_tau, 0.0f, 0.0f);
}


void SendWheelCommand(float lw_tau, float rw_tau) {
  if (!g_left_wheel || !g_right_wheel || !g_wheel_can) {
    return;
  }

  g_left_wheel->SetCurrent(ClampToI16(lw_tau));
  g_right_wheel->SetCurrent(ClampToI16(rw_tau));
  rm::device::DjiMotorBase::SendCommand(*g_wheel_can);
}

void SendSafeZero(bool &dm_enabled) {
  SendDmDisable(dm_enabled);
  SendDmMitCommand(0.0f, 0.0f, 0.0f, 0.0f);
  SendWheelCommand(0.0f, 0.0f);
}

void PumpCanTx() {
  if (g_joint_can) {
    auto now = rm::hal::ThrottledCan<>::clock::now();
    for (uint32_t i = 0; i < kJointCanProcessBudgetPerUpdate; ++i) {
      if (g_joint_can->queue().empty()) {
        break;
      }
      (void)g_joint_can->Process(now);
      now += std::chrono::duration_cast<rm::hal::ThrottledCan<>::duration>(kJointCanProcessStep);
    }
  }

  if (g_wheel_can) {
    auto now = rm::hal::ThrottledCan<>::clock::now();
    for (uint32_t i = 0; i < kWheelCanProcessBudgetPerUpdate; ++i) {
      if (g_wheel_can->queue().empty()) {
        break;
      }
      (void)g_wheel_can->Process(now);
      now += std::chrono::duration_cast<rm::hal::ThrottledCan<>::duration>(kWheelCanProcessStep);
    }
  }
}

void PublishMotorFeedback(TaskContext &ctx) {
  if (!g_dm_joints.Valid() || !g_left_wheel || !g_right_wheel) {
    return;
  }

  static uint16_t seq = 0;
  MotorFeedbackMsg msg{};
  msg.h.tick_ms = osKernelGetTickCount();
  msg.h.seq = ++seq;
  msg.h.source_id = 4U;

  msg.left_leg.front = {g_dm_joints.lf->pos(), g_dm_joints.lf->vel(), g_dm_joints.lf->tau()};
  msg.left_leg.back = {g_dm_joints.lb->pos(), g_dm_joints.lb->vel(), g_dm_joints.lb->tau()};
  msg.right_leg.front = {g_dm_joints.rf->pos(), g_dm_joints.rf->vel(), g_dm_joints.rf->tau()};
  msg.right_leg.back = {g_dm_joints.rb->pos(), g_dm_joints.rb->vel(), g_dm_joints.rb->tau()};

  msg.wheel.left_rad_s = -static_cast<float>(g_left_wheel->rpm()) * PI / 30.0f;
  msg.wheel.right_rad_s = static_cast<float>(g_right_wheel->rpm()) * PI / 30.0f;
  msg.valid = true;

  (void)QueuePutLatest(ctx.queues.motor_to_compute, msg);
  DebugUpdateMotor(msg);
}

} // namespace

/**
 * @brief 电机输出任务初始化
 */
void MotorOutputTaskInit(TaskContext &ctx) {
  (void)ctx;
  InitMotorsIfNeeded();
  ctx.motor_runtime.dm_enabled = false;
}

/**
 * @brief 电机输出任务单步更新
 * @note  仅执行下发，不做模型计算；无效力矩时回零输出。
 */
void MotorOutputTaskUpdate(TaskContext &ctx) {
  TorqueCmd6 torque_rx{};
  if (!QueueTryGet(ctx.queues.compute_to_motor, torque_rx)) {
    SendSafeZero(ctx.motor_runtime.dm_enabled);
    PumpCanTx();
    DebugUpdateMotor(false, false, torque_rx, ctx.motor_runtime.dm_enabled);
    return;
  }

  const uint32_t now_ticks = osKernelGetTickCount();
  if ((now_ticks - torque_rx.h.tick_ms) > kTorqueMsgTimeoutTicks) {
    SendSafeZero(ctx.motor_runtime.dm_enabled);
    PumpCanTx();
    DebugUpdateMotor(true, true, torque_rx, ctx.motor_runtime.dm_enabled);
    return;
  }

  if (torque_rx.enable_dm) {
    SendDmEnable(ctx.motor_runtime.dm_enabled);
  } else {
    SendDmDisable(ctx.motor_runtime.dm_enabled);
  }

  if (torque_rx.valid) {
    // 有力状态：下发计算任务输出的目标力矩。
    SendDmMitCommand(torque_rx.lf_tau, torque_rx.lb_tau, torque_rx.rf_tau,
                     torque_rx.rb_tau);
    SendWheelCommand(torque_rx.lw_tau * 2436.5f, torque_rx.rw_tau * 2436.5f);
  } else {
    // 其他状态：保持零力矩输出。
    SendDmMitCommand(0.0f, 0.0f, 0.0f, 0.0f);
    SendWheelCommand(0.0f, 0.0f);
  }
  PumpCanTx();
  PublishMotorFeedback(ctx);
  DebugUpdateMotor(true, false, torque_rx, ctx.motor_runtime.dm_enabled);
}

} // namespace tasking

extern "C" void MotorTaskInitC() {
  tasking::MotorOutputTaskInit(tasking::GetTaskContext());
}

extern "C" void MotorTaskUpdateC() {
  tasking::MotorOutputTaskUpdate(tasking::GetTaskContext());
}



