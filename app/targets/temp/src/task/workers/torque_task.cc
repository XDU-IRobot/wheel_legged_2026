/**
 * @file  task/workers/torque_task.cc
 * @brief 力矩计算任务实现
 */

#include "task/api/task_c_api.h"
#include "task/api/task_entrypoints.hpp"
#include "task/runtime/task_context.hpp"
#include "task/runtime/task_debug.hpp"

#include "chassis/chassis.hpp"
#include "cmsis_os.h"
#include "usart.h"
#include <cmath>

#include <librm.hpp>
namespace tasking {

namespace {

std::unique_ptr<Chassis> g_compute_chassis{};
std::unique_ptr<rm::hal::stm32::Uart> g_imu_uart{};
std::unique_ptr<rm::device::HipnucImu> g_imu{};
std::unique_ptr<rm::modules::PID> turn_pid{};

// 小陀螺平移增益：原来的 2.1f 偏大，这里调小便于细调轨迹。
constexpr f32 kSmallGyroTranslationGain = 1.f;
// 小陀螺目标自转角速度（rad/s）。
constexpr f32 kSmallGyroTargetYawDotRadS = 6.0f;
// 普通/跳跃模式前向目标增益：最终目标保持 vx_cmd * 2.1f。
constexpr f32 kNormalTranslationGain = 2.1f;

// 前向速度目标斜坡参数（2ms/step）：低/中/高腿长三套。
struct SdotRampParams {
  f32 accel_step;
  f32 brake_step;
};

constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.01f};
constexpr SdotRampParams kSdotRampMidLeg{0.006f, 0.006f};
constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};
// 输入死区：过滤极小遥控量，避免零点抖动带来微小速度指令。
constexpr f32 kVxCmdDeadband = 0.1f;
constexpr f32 kControlDtS = 0.002f;
// 速度/位移切换：使用滞回 + 最小驻留时间，避免模式抖动。
constexpr f32 kLockPointEnterSpeedThresholdMps = 0.30f;
constexpr f32 kLockPointExitSpeedThresholdMps = 0.55f;
constexpr f32 kLockPointEnterInputThreshold = 0.08f;
constexpr f32 kLockPointExitInputThreshold = 0.12f;
constexpr uint32_t kLockPointMinDwellTicks = 100U; // 约 200 ms
// alpha=0 纯速度，alpha=1 纯位移。
constexpr f32 kLockPointAlphaRiseStep = 0.015f;
constexpr f32 kLockPointAlphaFallStep = 0.018f;

bool IsSmallGyroMode(const Fsm::State mode) {
  return mode == Fsm::State::kSmallGyro;
}

void RampValueToTarget(const f32 target, f32 &value,
                       const SdotRampParams &ramp_params) {
  const bool direction_changed = (value * target) < 0.0f;
  const bool magnitude_reduced = std::abs(target) < std::abs(value);
  const f32 kStep = (direction_changed || magnitude_reduced)
                        ? ramp_params.brake_step
                        : ramp_params.accel_step;

  if (value < target) {
    value += kStep;
    if (value > target) {
      value = target;
    }
  } else if (value > target) {
    value -= kStep;
    if (value < target) {
      value = target;
    }
  }
}

void RampYawDotToTarget(const f32 target_yaw_dot, bool &is_ramping,
                        f32 &filtered_yaw_dot) {
  if (filtered_yaw_dot < target_yaw_dot) {
    filtered_yaw_dot += 0.05f;
    if (filtered_yaw_dot >= target_yaw_dot) {
      filtered_yaw_dot = target_yaw_dot;
      is_ramping = false;
    }
  } else if (filtered_yaw_dot > target_yaw_dot) {
    filtered_yaw_dot -= 0.05f;
    if (filtered_yaw_dot <= target_yaw_dot) {
      filtered_yaw_dot = target_yaw_dot;
      is_ramping = false;
    }
  } else {
    filtered_yaw_dot = target_yaw_dot;
  }
}

void UpdateLockPointBlend(const bool target_lock, f32 &alpha_lock) {
  if (target_lock) {
    alpha_lock = Clamp(alpha_lock + kLockPointAlphaRiseStep, 0.0f, 1.0f);
  } else {
    alpha_lock = Clamp(alpha_lock - kLockPointAlphaFallStep, 0.0f, 1.0f);
  }
}

void BuildSmallGyroExpected(const CommInputMsg &comm_msg,
                            const Chassis &chassis, rm::modules::PID &yaw_pid,
                            bool &is_ramping, f32 &filtered_yaw_dot,
                            f32 &expected_s, wbr::ExpectedState &expected) {
  const auto &current_state = chassis.GetOutput().current_state;
  const f32 current_s = current_state.s;

  // 小陀螺平移仅保留 vx_cmd，并按云台朝向投影到底盘一维 s 方向。
  const f32 gimbal_heading = -comm_msg.yaw_cmd;
  const f32 vx_gimbal = comm_msg.vx_cmd;
  const bool has_vx_motion_cmd = std::abs(vx_gimbal) > kVxCmdDeadband;
  const f32 s_dot_cmd =
      has_vx_motion_cmd ? (vx_gimbal * std::cos(gimbal_heading)) : 0.0f;

  // 小陀螺模式不启用控位移，只做速度控制。
  expected.s_dot = kSmallGyroTranslationGain * s_dot_cmd;
  expected.phi = current_state.phi;
  expected.phi_dot = 0.0f;
  expected.theta_ll = 0.01f;
  expected.theta_lr = -0.0f;

  const f32 ratio = comm_msg.yaw_cmd / PI;
  const int k = std::lround(ratio);
  const f32 yaw_target = static_cast<f32>(k) * PI;
  yaw_pid.Update(yaw_target, comm_msg.yaw_cmd);
  RampYawDotToTarget(kSmallGyroTargetYawDotRadS, is_ramping, filtered_yaw_dot);

  expected_s = current_s;
  expected.s = expected_s;
  expected.phi_dot = filtered_yaw_dot;
}

void BuildNormalExpected(const CommInputMsg &comm_msg, const Chassis &chassis,
                         rm::modules::PID &yaw_pid, bool &is_ramping,
                         f32 &filtered_yaw_dot, f32 &filtered_s_dot,
                         f32 &expected_s, const f32 lock_point_alpha,
                         const f32 lock_point_s_ref,
                         const SdotRampParams &ramp_params,
                         wbr::ExpectedState &expected) {
  const auto &current_state = chassis.GetOutput().current_state;
  const f32 current_s = current_state.s;

  const bool has_vx_motion_cmd = std::abs(comm_msg.vx_cmd) > kVxCmdDeadband;
  const f32 raw_target_s_dot =
      has_vx_motion_cmd ? (comm_msg.vx_cmd * kNormalTranslationGain) : 0.0f;
  f32 target_s_dot = raw_target_s_dot;
  const f32 ratio = comm_msg.yaw_cmd / PI;
  const int k = std::lround(ratio);
  const f32 yaw_target = static_cast<f32>(k) * PI;
  if ((k % 2) != 0) {
    target_s_dot = -target_s_dot;
  }
  RampValueToTarget(target_s_dot, filtered_s_dot, ramp_params);
  // 无扰切换：alpha 连续变化，避免 s_dot 在速度/位移模式间突变。
  expected.s_dot = (1.0f - lock_point_alpha) * filtered_s_dot;

  yaw_pid.Update(yaw_target, comm_msg.yaw_cmd);
  const f32 target_yaw_dot = -yaw_pid.out();
  if (is_ramping) {
    RampYawDotToTarget(target_yaw_dot, is_ramping, filtered_yaw_dot);
    expected.phi = current_state.phi;
  } else {
    filtered_yaw_dot = target_yaw_dot;
    expected.phi = -yaw_target;
  }

  expected.phi_dot = filtered_yaw_dot;
  // 位移目标从 current_s 平滑过渡到锁点参考，避免参考量跳变。
  expected_s = lock_point_alpha * lock_point_s_ref +
               (1.0f - lock_point_alpha) * current_s;
  expected.s = expected_s;
  expected.theta_ll = 0.0f;
  expected.theta_lr = 0.0f;
}

void BuildJumpExpected(const CommInputMsg &comm_msg, const Chassis &chassis,
                       rm::modules::PID &yaw_pid, bool &is_ramping,
                       f32 &filtered_yaw_dot, f32 &filtered_s_dot,
                       f32 &expected_s, const f32 lock_point_alpha,
                       const f32 lock_point_s_ref,
                       const SdotRampParams &ramp_params,
                       wbr::ExpectedState &expected) {
  BuildNormalExpected(comm_msg, chassis, yaw_pid, is_ramping, filtered_yaw_dot,
                      filtered_s_dot, expected_s, lock_point_alpha,
                      lock_point_s_ref, ramp_params, expected);
}

void BuildExpectedFromFsmMode(const Fsm::State fsm_mode,
                              const CommInputMsg &comm_msg,
                              const Chassis &chassis, rm::modules::PID &yaw_pid,
                              bool &is_ramping, f32 &filtered_yaw_dot,
                              f32 &filtered_s_dot, f32 &expected_s,
                              const f32 lock_point_alpha,
                              const f32 lock_point_s_ref,
                              wbr::ExpectedState &expected) {
  switch (fsm_mode) {
  case Fsm::State::kNoForce:
    expected = {};
    expected.s = chassis.GetOutput().current_state.s;
    expected.s_dot = chassis.GetOutput().current_state.s_dot;
    expected.phi = chassis.GetOutput().current_state.phi;
    expected.phi_dot = chassis.GetOutput().current_state.phi_dot;
    expected.theta_ll = chassis.GetOutput().current_state.theta_ll;
    expected.theta_ll_dot = chassis.GetOutput().current_state.theta_ll_dot;
    expected.theta_lr = chassis.GetOutput().current_state.theta_lr;
    expected.theta_lr_dot = chassis.GetOutput().current_state.theta_lr_dot;
    expected.theta_b = chassis.GetOutput().current_state.theta_b;
    expected.theta_b_dot = chassis.GetOutput().current_state.theta_b_dot;
    expected_s = expected.s;
    filtered_yaw_dot = 0.0f;
    filtered_s_dot = expected.s_dot;
    return;

  case Fsm::State::kSmallGyro:
    // 小陀螺不走 Normal 速度斜坡，切回 Normal 时用当前实测速度作初值。
    filtered_s_dot = chassis.GetOutput().current_state.s_dot;
    BuildSmallGyroExpected(comm_msg, chassis, yaw_pid, is_ramping,
                           filtered_yaw_dot, expected_s, expected);
    return;

  case Fsm::State::kNormalLowLeg:
    BuildNormalExpected(comm_msg, chassis, yaw_pid, is_ramping,
                        filtered_yaw_dot, filtered_s_dot, expected_s,
                        lock_point_alpha, lock_point_s_ref,
                        kSdotRampLowLeg, expected);
    return;

  case Fsm::State::kNormalMidLeg:
    BuildNormalExpected(comm_msg, chassis, yaw_pid, is_ramping,
                        filtered_yaw_dot, filtered_s_dot, expected_s,
                        lock_point_alpha, lock_point_s_ref,
                        kSdotRampMidLeg, expected);
    return;

  case Fsm::State::kNormalHighLeg:
    BuildNormalExpected(comm_msg, chassis, yaw_pid, is_ramping,
                        filtered_yaw_dot, filtered_s_dot, expected_s,
                        lock_point_alpha, lock_point_s_ref,
                        kSdotRampHighLeg, expected);
    return;

  case Fsm::State::kJumpRetract1:
    BuildNormalExpected(comm_msg, chassis, yaw_pid, is_ramping,
                        filtered_yaw_dot, filtered_s_dot, expected_s,
                        lock_point_alpha, lock_point_s_ref,
                        kSdotRampMidLeg, expected);
    return;

  case Fsm::State::kJumpExtend:
  case Fsm::State::kJumpRetract2:
    BuildJumpExpected(comm_msg, chassis, yaw_pid, is_ramping, filtered_yaw_dot,
                      filtered_s_dot, expected_s, lock_point_alpha,
                      lock_point_s_ref, kSdotRampMidLeg, expected);
    return;

  default:
    expected.s = expected_s;
    expected.s_dot = 0.0f;
    expected.phi = 0.0f;
    expected.phi_dot = 0.0f;
    expected.theta_ll = 0.0f;
    expected.theta_lr = 0.0f;
    filtered_yaw_dot = 0.0f;
    filtered_s_dot = 0.0f;
    return;
  }
}

} // namespace

/**
 * @brief 力矩计算任务初始化
 */
void TorqueComputeTaskInit(TaskContext &ctx) {
  (void)ctx;
  if (!g_compute_chassis) {
    g_compute_chassis = std::make_unique<Chassis>();
    g_compute_chassis->Init();
  }

  if (!g_imu_uart) {
    g_imu_uart = std::make_unique<rm::hal::stm32::Uart>(
        huart10, 518, rm::hal::stm32::UartMode::kNormal,
        rm::hal::stm32::UartMode::kDma);
  }
  if (!g_imu && g_imu_uart) {
    g_imu = std::make_unique<rm::device::HipnucImu>(*g_imu_uart);
    g_imu->Begin();
  }
  turn_pid = std::make_unique<rm::modules::PID>(9.2f, 0.0f, 200.f, 5.f, 0.f);
}

/**
 * @brief 力矩计算任务单步更新
 * @note  负责状态估计/控制器调用，并输出六电机目标力矩。
 */
void TorqueComputeTaskUpdate(TaskContext &ctx) {
  if (!g_compute_chassis || !g_imu) {
    return;
  }

  static f32 filtered_yaw_dot = 0.0f;
  static f32 filtered_s_dot = 0.0f;
  static f32 expected_s = 0.0f;

  // 输入采用“读队列，读不到就沿用上次快照”的策略，避免控制量突变为默认值。
  static bool has_comm = false;
  static CommInputMsg comm_msg{};
  CommInputMsg comm_rx{};
  if (QueueTryGet(ctx.queues.comm_to_compute, comm_rx)) {
    comm_msg = comm_rx;
    has_comm = true;
  }

  static bool has_fsm = false;
  static FsmOutputMsg fsm_msg{};
  FsmOutputMsg fsm_rx{};
  if (QueueTryGet(ctx.queues.fsm_to_compute, fsm_rx)) {
    fsm_msg = fsm_rx;
    has_fsm = true;
  }

  static bool has_motor_feedback = false;
  static MotorFeedbackMsg motor_msg{};
  MotorFeedbackMsg motor_rx{};
  if (QueueTryGet(ctx.queues.motor_to_compute, motor_rx)) {
    motor_msg = motor_rx;
    has_motor_feedback = true;
  }

  const Fsm::State fsm_mode_raw = has_fsm ? fsm_msg.mode : Fsm::State::kNoForce;

  TorqueCmd6 torque_msg{};
  static uint16_t seq = 0;
  torque_msg.h.tick_ms = osKernelGetTickCount();
  torque_msg.h.seq = ++seq;
  torque_msg.h.source_id = 3U;
  torque_msg.enable_dm = has_fsm && fsm_msg.control.enable_dm;
  torque_msg.jump_phase = has_fsm ? fsm_msg.control.jump_phase : 0U;
  const float target_leg_length =
      has_fsm ? fsm_msg.control.target_leg_length_m : 0.18f;

  // 采集本周期 IMU 快照，避免调试值与控制输入不一致。
  const float imu_roll_rad = g_imu->roll();
  const float imu_pitch_rad = -g_imu->pitch();
  const float imu_gyro_y_rad_s = -g_imu->gyro_x();
  const float imu_gyro_z_rad_s = g_imu->gyro_z();
  const float imu_acc_x_mps2 = g_imu->acc_x();
  const float imu_acc_y_mps2 = g_imu->acc_y();
  const float imu_acc_z_mps2 = g_imu->acc_z();
  // const float imu_yaw_motor_rad = g_imu->yaw();
  const float imu_yaw_motor_rad = -comm_msg.yaw_cmd;

  static Fsm::State last_fsm_mode = Fsm::State::kNoForce;
  static bool is_ramping = false;
  static bool lock_point_target = false;
  static f32 lock_point_alpha = 0.0f;
  static f32 lock_point_s_ref = 0.0f;
  static uint32_t lock_point_last_switch_tick = 0U;

  if (has_fsm && fsm_mode_raw != last_fsm_mode) {
    expected_s = g_compute_chassis->GetOutput().current_state.s;
    filtered_s_dot = g_compute_chassis->GetOutput().current_state.s_dot;
    lock_point_target = false;
    lock_point_alpha = 0.0f;
    lock_point_s_ref = expected_s;
    lock_point_last_switch_tick = osKernelGetTickCount();

    const bool cross_small_gyro =
        IsSmallGyroMode(last_fsm_mode) != IsSmallGyroMode(fsm_mode_raw);
    if (fsm_mode_raw == Fsm::State::kNoForce) {
      // 进入失能态时清掉旋转过渡状态。
      filtered_yaw_dot = 0.0f;
      is_ramping = false;
    } else if (cross_small_gyro) {
      // 小陀螺进出都走斜坡，避免角速度瞬变。
      is_ramping = true;
    } else {
      filtered_yaw_dot = 0.0f;
      is_ramping = false;
    }

    turn_pid->Clear();
    last_fsm_mode = fsm_mode_raw;
  }

  const auto publish_zero_torque = [&](const bool debug_run_compute_flag) {
    torque_msg.enable_dm = false;
    torque_msg.jump_phase = 0U;
    torque_msg.valid = false;
    torque_msg.lf_tau = 0.0f;
    torque_msg.lb_tau = 0.0f;
    torque_msg.rf_tau = 0.0f;
    torque_msg.rb_tau = 0.0f;
    torque_msg.lw_tau = 0.0f;
    torque_msg.rw_tau = 0.0f;
    (void)QueuePutLatest(ctx.queues.compute_to_motor, torque_msg);
    DebugUpdateTorque(torque_msg, has_comm, has_fsm, has_motor_feedback,
                      debug_run_compute_flag, target_leg_length, nullptr,
                      nullptr, 0.0f, true, imu_roll_rad, imu_pitch_rad,
                      imu_gyro_y_rad_s, imu_gyro_z_rad_s, imu_acc_x_mps2,
                      imu_acc_y_mps2, imu_acc_z_mps2, imu_yaw_motor_rad, 0.0f,
                      0.0f, 0.0f, 0.0f, 0.0f);
  };

  // 仅在状态机允许时运行模型；不允许时直接下发“无效 + 零力矩”。
  const bool run_compute = has_fsm &&
                           (fsm_msg.control.run_chassis_update ||
                            fsm_mode_raw == Fsm::State::kNoForce);
  if (!run_compute) {
    publish_zero_torque(false);
    return;
  }

  // 组装控制期望：完全基于 FSM 状态分发。
  wbr::ExpectedState expected{};

  if (has_comm && comm_msg.link_ok) {
    const Fsm::State fsm_mode = fsm_mode_raw;

    const auto &current_state = g_compute_chassis->GetOutput().current_state;
    const f32 speed_abs = std::abs(current_state.s_dot);
    const f32 input_abs = std::abs(comm_msg.vx_cmd);
    const bool lock_point_mode_enabled =
        (fsm_mode != Fsm::State::kNoForce &&
         fsm_mode != Fsm::State::kSmallGyro);

    if (!lock_point_mode_enabled) {
      lock_point_target = false;
    } else {
      const bool request_lock =
          (speed_abs < kLockPointEnterSpeedThresholdMps) &&
          (input_abs < kLockPointEnterInputThreshold);
      const bool request_unlock =
          (speed_abs > kLockPointExitSpeedThresholdMps) ||
          (input_abs > kLockPointExitInputThreshold);
      const uint32_t now_ticks = osKernelGetTickCount();
      const uint32_t elapsed = now_ticks - lock_point_last_switch_tick;
      if (!lock_point_target && request_lock &&
          elapsed >= kLockPointMinDwellTicks) {
        lock_point_target = true;
        lock_point_s_ref = current_state.s;
        lock_point_last_switch_tick = now_ticks;
      } else if (lock_point_target && request_unlock &&
                 elapsed >= kLockPointMinDwellTicks) {
        lock_point_target = false;
        lock_point_last_switch_tick = now_ticks;
      }
    }

    UpdateLockPointBlend(lock_point_target, lock_point_alpha);
    if (lock_point_alpha < 0.02f) {
      // 退出位移模式后让参考贴回当前状态，避免下次进入时带入旧误差。
      lock_point_s_ref = current_state.s;
    }

    BuildExpectedFromFsmMode(fsm_mode, comm_msg, *g_compute_chassis, *turn_pid,
                             is_ramping, filtered_yaw_dot, filtered_s_dot,
                             expected_s, lock_point_alpha, lock_point_s_ref,
                             expected);
    last_fsm_mode = fsm_mode;
  } else {
    lock_point_target = false;
    lock_point_alpha = 0.0f;
    expected_s = g_compute_chassis->GetOutput().current_state.s;
    lock_point_s_ref = expected_s;
    expected.s = expected_s;
    expected.s_dot = 0.0f;
    filtered_s_dot = 0.0f;
    expected.phi = 0.0f;
    expected.theta_ll = 0.0f;
    expected.theta_lr = 0.0f;
  }

  expected.theta_ll_dot = 0.0f;
  expected.theta_lr_dot = 0.0f;
  expected.theta_b = 0.0f;
  expected.theta_b_dot = 0.0f;

  Chassis::UpdateInput update_input{};
  update_input.estimator_input.left_leg = motor_msg.left_leg;
  update_input.estimator_input.right_leg = motor_msg.right_leg;
  update_input.estimator_input.wheel = motor_msg.wheel;
  update_input.estimator_input.imu.roll_rad = imu_roll_rad;
  update_input.estimator_input.imu.pitch_rad = imu_pitch_rad;
  update_input.estimator_input.imu.gyro_y_rad_s = imu_gyro_y_rad_s;
  update_input.estimator_input.imu.gyro_z_rad_s = imu_gyro_z_rad_s;
  update_input.estimator_input.imu.acc_x_mps2 = imu_acc_x_mps2;
  update_input.estimator_input.imu.acc_y_mps2 = imu_acc_y_mps2;
  update_input.estimator_input.imu.acc_z_mps2 = imu_acc_z_mps2;
  update_input.estimator_input.yaw_motor_rad = imu_yaw_motor_rad;
  // 任务周期 2ms，这里与调度周期保持一致。
  update_input.estimator_input.dt_s = kControlDtS;
  update_input.expected = expected;
  update_input.fsm_mode = fsm_mode_raw;
  update_input.target_leg_length_m = target_leg_length;

  // 运行底盘一步更新，拿到六电机目标力矩与当前状态估计。
  g_compute_chassis->Update(update_input);
  const Chassis::UpdateOutput &chassis_output = g_compute_chassis->GetOutput();

  StateEstimateMsg state_msg{};
  state_msg.h.tick_ms = torque_msg.h.tick_ms;
  state_msg.h.seq = torque_msg.h.seq;
  state_msg.h.source_id = 3U;
  state_msg.current = chassis_output.current_state;
  state_msg.wheel_speed_mps = chassis_output.raw_wheel_speed_mps;
  state_msg.fused_speed_mps = chassis_output.current_speed_mps;
  (void)QueuePutLatest(ctx.queues.compute_to_fsm, state_msg);

  torque_msg.valid = true;
  torque_msg.lf_tau = chassis_output.lf_tau;
  torque_msg.lb_tau = chassis_output.lb_tau;
  torque_msg.rf_tau = chassis_output.rf_tau;
  torque_msg.rb_tau = chassis_output.rb_tau;
  torque_msg.lw_tau = chassis_output.lw_tau;
  torque_msg.rw_tau = chassis_output.rw_tau;

  if (fsm_mode_raw == Fsm::State::kNoForce) {
    torque_msg.enable_dm = false;
    torque_msg.valid = true;
    torque_msg.lf_tau = 0.0f;
    torque_msg.lb_tau = 0.0f;
    torque_msg.rf_tau = 0.0f;
    torque_msg.rb_tau = 0.0f;
    torque_msg.lw_tau = 0.0f;
    torque_msg.rw_tau = 0.0f;
  }

  // 结果通过 latest 队列发送到电机任务，并同步写入调试快照。
  (void)QueuePutLatest(ctx.queues.compute_to_motor, torque_msg);
  DebugUpdateTorque(torque_msg, has_comm, has_fsm, has_motor_feedback, true,
                    target_leg_length, &chassis_output.current_state, &expected,
                    0.0f, true, imu_roll_rad, imu_pitch_rad, imu_gyro_y_rad_s,
                    imu_gyro_z_rad_s, imu_acc_x_mps2, imu_acc_y_mps2,
                    imu_acc_z_mps2, imu_yaw_motor_rad, chassis_output.left_Fn,
                    chassis_output.right_Fn, chassis_output.raw_wheel_speed_mps,
                    chassis_output.raw_accel_speed_mps,
                    chassis_output.current_speed_mps);
}

} // namespace tasking

extern "C" void TorqueTaskInitC() {
  tasking::TorqueComputeTaskInit(tasking::GetTaskContext());
}

extern "C" void TorqueTaskUpdateC() {
  tasking::TorqueComputeTaskUpdate(tasking::GetTaskContext());
}
