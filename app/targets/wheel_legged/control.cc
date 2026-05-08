#include "include/globals.hpp"
#include "include/actuators.hpp"
#include "include/debug.hpp"

#include "main.h"
#include <algorithm>
#include <cmath>

#include "include/input.hpp"
#include "include/state_ctx.hpp"

/**
 * @file  targets/wheel_legged/control.cc
 * @brief 500Hz 主控制循环：输入采集、状态机更新、底盘解算、执行器输出与调试同步
 */

DebugSnapshot wl_debug __attribute__((section(".sram4")));

namespace {

namespace ns = wheel_legged::params::active;
using namespace wheel_legged::control_loop;

// ── 控制循环参数（常量化，避免逻辑中散落魔数）──
constexpr float kControlLoopDtS = ns::control_loop::kControlLoopDtS;
constexpr float kPi = ns::kPi;
constexpr float kTargetForwardSpeedMaxMps = ns::control_loop::kTargetForwardSpeedMaxMps;
constexpr float kVxInputDeadbandNorm = ns::control_loop::kVxInputDeadbandNorm;
constexpr float kVyInputDeadbandNorm = ns::control_loop::kVyInputDeadbandNorm;
constexpr float kYawFollowRampStepRadS = ns::control_loop::kYawFollowRampStepRadS;
constexpr float kPositionFreezeSpeedThresholdMps = ns::control_loop::kPositionFreezeSpeedThresholdMps;
constexpr float kSpinYawRampStepRadS = ns::control_loop::kSpinYawRampStepRadS;
constexpr float kSpinTargetYawDotRadS = ns::control_loop::kSpinTargetYawDotRadS;
constexpr float kSpinTranslationGain = ns::control_loop::kSpinTranslationGain;
constexpr float kSpinThetaLlBiasRad = ns::control_loop::kSpinThetaLlBiasRad;
constexpr float kExpectedThetaLlBiasRad = ns::control_loop::kExpectedThetaLlBiasRad;
constexpr float kExpectedThetaLrBiasRad = ns::control_loop::kExpectedThetaLrBiasRad;
constexpr float kExpectedThetaBBiasRad = ns::control_loop::kExpectedThetaBBiasRad;
constexpr uint32_t kGimbalStartupYawAlignStableTicks = ns::control_loop::kGimbalStartupYawAlignStableTicks;
constexpr uint32_t kYawFollowDriveReadyStableTicks = ns::control_loop::kYawFollowDriveReadyStableTicks;
constexpr float kYawFollowFixedTargetRad = ns::control_loop::kYawFollowFixedTargetRad;

chassis_runtime::Actuators g_actuators{};

}  // namespace

/**
 * @brief 500Hz 主控制循环
 * @note  由 TimerTaskScheduler 绑定到 htim13，按固定周期调用。
 *        整个循环分为 9 个阶段，数据单向流动：采集 → 语义 → FSM → 控制 → 执行。
 *        状态机 (FSM) 决定"做什么"，控制器决定"怎么做"，执行器负责"下发"。
 */
void ControlLoop() {
  if (globals == nullptr) return;

  const uint32_t now_ms = HAL_GetTick();

  // ── 跨周期状态（static 保持值语义，避免堆分配）──
  static InputSnapshot input{};
  static Dr16SemanticState dr16_state{};
  static TcSemanticState tc_state{};
  static ChassisStateContext ctx{};
  static chassis::Chassis::UpdateOutput chassis_control_output{};
  static gimbal::Gimbal::UpdateOutput gimbal_control_output{};

  // ── 一次性初始化 ──
  if (!ctx.yaw_follow_pid_initialized) {
    const auto &pid = ns::control_loop::kYawFollowPid;
    ctx.yaw_follow_pid.SetKp(pid.kp).SetKi(pid.ki).SetKd(pid.kd).SetMaxOut(pid.max_out).SetMaxIout(pid.max_iout);
    ctx.yaw_follow_pid.SetCircular(true).SetCircularCycle(2.0f * kPi);
    ctx.yaw_follow_pid_initialized = true;
  }

  // 云台 C 板通信断开时强制退出中腿长保持
  if (!(globals->gimbal_rx.has_value() && globals->gimbal_rx->frame_count() > 0)) {
    tc_state.mid_leg_hold = false;
  }

  // ═══════════════════════════════════════════════════════════════════════
  // 阶段 1：硬件反馈采集 + DR16 语义折叠
  // ═══════════════════════════════════════════════════════════════════════
  UpdateRawFeedbackAndInputSnapshot(*globals, g_actuators, input, dr16_state, tc_state);
  wl_debug.tc_mid_leg_hold = tc_state.mid_leg_hold ? 1 : 0;

  // ═══════════════════════════════════════════════════════════════════════
  // 阶段 2：状态机决策
  // ═══════════════════════════════════════════════════════════════════════
  const chassis::Fsm::Input chassis_input = BuildChassisFsmInput(input, now_ms, chassis_control_output);
  const chassis::Fsm::Output chassis_output = globals->chassis_fsm.Update(chassis_input);

  const gimbal::Fsm::Input gimbal_input = BuildGimbalFsmInput(input, chassis_output, ctx.gimbal_startup_align_complete);
  const gimbal::Fsm::Output gimbal_output = globals->gimbal_fsm.Update(gimbal_input);
  const bool gimbal_startup_align_active = gimbal_output.mode == gimbal::Fsm::State::kStartupAlign;

  // ═══════════════════════════════════════════════════════════════════════
  // 阶段 3：云台启动归中初始化
  // ═══════════════════════════════════════════════════════════════════════
  if (gimbal_startup_align_active && !ctx.gimbal_startup_align_was_active) {
    ctx.gimbal_startup_align_complete = false;
    ctx.gimbal_startup_align_stable_ticks = 0U;
    // 选择最近的车头方向作为归中目标（两个候选角，相差 π）
    ctx.gimbal_startup_align_target_rad = SelectNearestYawCenterTarget(input.estimator_input.yaw_motor_rad);
  }

  // ═══════════════════════════════════════════════════════════════════════
  // 阶段 4：云台控制
  // ═══════════════════════════════════════════════════════════════════════
  gimbal::Gimbal::UpdateInput gimbal_update_input{};
  gimbal_update_input.yaw_motor = globals->yaw_motor.has_value() ? &(*globals->yaw_motor) : nullptr;
  gimbal_update_input.pitch_motor = globals->pitch_motor.has_value() ? &(*globals->pitch_motor) : nullptr;
  gimbal_update_input.gimbal_enable = gimbal_output.control.gimbal_enable;

  // 底盘姿态异常时强制关闭云台输出
  const bool chassis_posture_invalid = !chassis_control_output.posture_valid;
  if (chassis_posture_invalid) {
    gimbal_update_input.gimbal_enable = false;
  }

  gimbal_update_input.align_to_chassis_forward = gimbal_output.control.align_to_chassis_forward;
  gimbal_update_input.target = gimbal_output.control.gimbal_target;
  gimbal_update_input.use_yaw_motor_feedback = gimbal_startup_align_active;
  gimbal_update_input.aimbot_mode = gimbal_output.control.active_target_source == wheel_legged::TargetSource::kHost;
  if (gimbal_startup_align_active) {
    // 归中阶段：强制对齐车头方向，覆盖 FSM 下发的目标
    gimbal_update_input.align_to_chassis_forward = false;
    gimbal_update_input.target.yaw_rad = ctx.gimbal_startup_align_target_rad;
  }
  gimbal_update_input.chassis_yaw_rad = input.estimator_input.imu.yaw_rad;
  gimbal_update_input.chassis_pitch_rad = input.estimator_input.imu.pitch_rad;
  gimbal_update_input.yaw_motor_rad = input.estimator_input.yaw_motor_rad;
  gimbal_update_input.gimbal_imu_yaw_rad = input.gimbal_imu_yaw_rad;
  gimbal_update_input.gimbal_imu_pitch_rad = input.gimbal_imu_pitch_rad;
  gimbal_update_input.gimbal_imu_gyro_z_rad_s = input.gimbal_imu_gyro_z_rad_s;
  gimbal_update_input.gimbal_imu_gyro_x_rad_s = -input.gimbal_imu_gyro_x_rad_s;
  gimbal_update_input.dt_s = kControlLoopDtS;
  globals->gimbal.Update(gimbal_update_input);
  gimbal_control_output = globals->gimbal.GetOutput();
  g_actuators.ApplyGimbalOutput(*globals, gimbal_control_output);

  // ═══════════════════════════════════════════════════════════════════════
  // 阶段 5：发射机构控制（变体分支）
  // ═══════════════════════════════════════════════════════════════════════
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  // Hero：三摩擦轮 + DM 拨盘，ShootController 内置 5 状态机自行下发
  {
    const bool shooter_enter = (gimbal_output.mode == gimbal::Fsm::State::kCombat);
    const bool fire_trigger = input.dr16.dial < ns::shoot::kFireDialThreshold;
    globals->shoot_controller.Update(shooter_enter, fire_trigger);
    wl_debug.booster_raw_pos_rad = globals->shoot_controller.booster_pos();
    wl_debug.fw_raw_rpm_1 = globals->fw_motor_1.has_value() ? static_cast<float>(globals->fw_motor_1->rpm()) : 0.0f;
    wl_debug.fw_raw_rpm_2 = globals->fw_motor_2.has_value() ? static_cast<float>(globals->fw_motor_2->rpm()) : 0.0f;
    wl_debug.fw_raw_rpm_3 = globals->fw_motor_3.has_value() ? static_cast<float>(globals->fw_motor_3->rpm()) : 0.0f;
    rm::device::DjiMotorBase::SendCommand(*globals->gimbal_can);
  }
#else
  // Infantry3/4：双摩擦轮 + M3508 拨盘，通过 ShootOutput 解耦
  {
    const bool in_combat = (gimbal_output.mode == gimbal::Fsm::State::kCombat);
    if (in_combat && !globals->shoot.enabled()) {
      globals->shoot.Enable();
    } else if (!in_combat && globals->shoot.enabled()) {
      globals->shoot.Disable();
    }
    wl_debug.shoot_enabled = globals->shoot.enabled() ? 1U : 0U;
    const float fric_left_rpm = globals->fric_left.has_value() ? static_cast<float>(globals->fric_left->rpm()) : 0.0f;
    const float fric_right_rpm =
        globals->fric_right.has_value() ? static_cast<float>(globals->fric_right->rpm()) : 0.0f;
    const float dial_encoder = globals->dial.has_value() ? -static_cast<float>(globals->dial->encoder()) : 0.0f;
    const float dial_rpm = globals->dial.has_value() ? -static_cast<float>(globals->dial->rpm()) : 0.0f;
    const auto shoot_output =
        globals->shoot.Update(fric_left_rpm, fric_right_rpm, dial_encoder, dial_rpm, kControlLoopDtS, input.dr16.dial,
                              input.tc_remote.left_button, in_combat);
    g_actuators.ApplyShootOutput(*globals, shoot_output);
  }
#endif

  // ═══════════════════════════════════════════════════════════════════════
  // 阶段 6：云台启动归中判稳
  // ═══════════════════════════════════════════════════════════════════════
  if (!gimbal_output.control.gimbal_enable || chassis_posture_invalid) {
    ctx.gimbal_startup_align_complete = false;
    ctx.gimbal_startup_align_was_active = false;
    ctx.gimbal_startup_align_stable_ticks = 0U;
  } else if (gimbal_startup_align_active) {
    const bool yaw_motor_ready = globals->yaw_motor.has_value();
    const float yaw_motor_vel_rad_s = yaw_motor_ready ? globals->yaw_motor->vel() : 0.0f;
    if (yaw_motor_ready && IsYawAtStartupTarget(ctx.gimbal_startup_align_target_rad,
                                                input.estimator_input.yaw_motor_rad, yaw_motor_vel_rad_s)) {
      ++ctx.gimbal_startup_align_stable_ticks;
    } else {
      ctx.gimbal_startup_align_stable_ticks = 0U;
    }
    if (ctx.gimbal_startup_align_stable_ticks >= kGimbalStartupYawAlignStableTicks) {
      ctx.gimbal_startup_align_complete = true;
      // 归中完成后，将 RC 积分目标对齐到当前云台惯导角（消除归中过程的目标漂移）
      if (gimbal_input.request.target_source == wheel_legged::TargetSource::kRc) {
        dr16_state.rc_target.yaw_rad = input.gimbal_imu_yaw_rad;
      }
    }
    ctx.gimbal_startup_align_was_active = true;
  } else {
    ctx.gimbal_startup_align_was_active = false;
    ctx.gimbal_startup_align_stable_ticks = 0U;
  }

  // ── 底盘输出使能条件 ──
  const bool chassis_startup_ready = !gimbal_output.control.gimbal_enable || !gimbal_startup_align_active;
  const bool auto_aim_no_move = chassis_input.request.combat_profile == wheel_legged::CombatProfile::kAutoAimNoMove;
  const bool chassis_output_enable =
      auto_aim_no_move ? false
                       : (chassis_posture_invalid ? chassis_output.control.enable_dm
                                                  : chassis_output.control.enable_dm && chassis_startup_ready);

  // ═══════════════════════════════════════════════════════════════════════
  // 阶段 7：底盘控制
  // ═══════════════════════════════════════════════════════════════════════

  // ── 7a. 底盘控制器输入组装 ──
  chassis::Chassis::UpdateInput chassis_update_input{};
  chassis_update_input.fsm_mode = chassis_output.mode;
  chassis_update_input.enable_output = chassis_output_enable;
  chassis_update_input.run_chassis_update = chassis_output.control.run_chassis_update;
  chassis_update_input.spin_enable = chassis_output.control.spin_enable;
  chassis_update_input.target_leg_length_m = chassis_output.control.target_leg_length_m;
  chassis_update_input.estimator_input = input.estimator_input;
  chassis_update_input.estimator_input.dt_s = kControlLoopDtS;

  // ── 7b. 模式切换处理 ──
  const auto &current_state = chassis_control_output.current_state;
  const bool mode_changed = (chassis_output.mode != ctx.last_chassis_mode);
  const bool last_was_spin = (ctx.last_chassis_mode == chassis::Fsm::State::kSpin);
  const bool now_is_spin = (chassis_output.mode == chassis::Fsm::State::kSpin);
  const bool cross_spin = (last_was_spin != now_is_spin);
  if (mode_changed) {
    ctx.ResetOnModeChange(current_state.s, current_state.s_dot);
    if (cross_spin) {
      // 进出小陀螺时继承当前车体角速度，避免阶梯跳变
      ctx.filtered_yaw_dot = chassis_control_output.current_state.phi_dot;
    }
    ctx.last_chassis_mode = chassis_output.mode;
  }

  // ── 7c. 驾驶输入解析 ──
  const bool dr16_online = input.dr16.online;
  const bool tc_remote_active = input.tc_remote.valid;
  const bool has_drive_input = dr16_online || tc_remote_active;

  const auto drive = ResolveDriveInput(input.dr16, input.tc_remote);
  const float forward_input_norm = drive.forward;
  const float side_input_norm = drive.side;
  const bool forward_input_active = std::fabs(forward_input_norm) > kVxInputDeadbandNorm;
  const bool side_input_active = std::fabs(side_input_norm) > kVyInputDeadbandNorm;

  // ── 7d. 偏航跟随模式选择 ──
  YawFollowAlignMode requested_yaw_follow_align_mode = ctx.yaw_follow_align_mode;
  if (!has_drive_input || chassis_input.request.domain_request == wheel_legged::DomainRequest::kDisabled) {
    requested_yaw_follow_align_mode = YawFollowAlignMode::kForward;
  } else if (forward_input_active) {
    requested_yaw_follow_align_mode = YawFollowAlignMode::kForward;
  } else if (side_input_active) {
    requested_yaw_follow_align_mode =
        (side_input_norm > 0.0f) ? YawFollowAlignMode::kSidePositive : YawFollowAlignMode::kSideNegative;
  }

  // ── 7e. 偏航目标更新 ──
  const bool yaw_follow_mode_changed = requested_yaw_follow_align_mode != ctx.yaw_follow_align_mode;
  if (!has_drive_input || chassis_input.request.domain_request == wheel_legged::DomainRequest::kDisabled) {
    ctx.yaw_follow_align_mode = YawFollowAlignMode::kForward;
    ctx.yaw_follow_target_initialized = false;
    ctx.yaw_follow_drive_ready = false;
    ctx.yaw_follow_drive_ready_stable_ticks = 0U;
  } else if (yaw_follow_mode_changed || !ctx.yaw_follow_target_initialized) {
    ctx.yaw_follow_align_mode = requested_yaw_follow_align_mode;
    ctx.yaw_follow_target =
        SelectNearestYawTarget(input.estimator_input.yaw_motor_rad, YawFollowTargetOffset(ctx.yaw_follow_align_mode));
    ctx.yaw_follow_target_initialized = true;
    ctx.yaw_follow_drive_ready = false;
    ctx.yaw_follow_drive_ready_stable_ticks = 0U;
    ctx.filtered_yaw_dot = 0.0f;
    ctx.yaw_follow_pid.Clear();
  }

  if (!ctx.yaw_follow_target_initialized) {
    ctx.yaw_follow_target =
        SelectNearestYawTarget(input.estimator_input.yaw_motor_rad, YawFollowTargetOffset(ctx.yaw_follow_align_mode));
    ctx.yaw_follow_target_initialized = true;
  }

  // ── 7f. 纵向速度目标计算 ──
  const float yaw_follow_drive_sign = YawFollowDriveSign(ctx.yaw_follow_align_mode, ctx.yaw_follow_target.drive_sign);
  const bool spin_control_enabled = chassis_output.mode == chassis::Fsm::State::kSpin && chassis_output_enable &&
                                    chassis_output.control.run_chassis_update;

  // ── 7g. 偏航就绪判稳 ──
  const bool yaw_follow_control_enabled = chassis_output.mode != chassis::Fsm::State::kDisabled &&
                                          chassis_output_enable && chassis_output.control.run_chassis_update &&
                                          gimbal_output.control.gimbal_enable;
  if (!spin_control_enabled && !ctx.yaw_follow_drive_ready) {
    const bool yaw_motor_ready = globals->yaw_motor.has_value();
    const float yaw_motor_vel_rad_s = yaw_motor_ready ? globals->yaw_motor->vel() : 0.0f;
    if (yaw_follow_control_enabled && yaw_motor_ready &&
        IsYawFollowDriveReady(ctx.yaw_follow_target.target_rad, input.estimator_input.yaw_motor_rad,
                              yaw_motor_vel_rad_s)) {
      ++ctx.yaw_follow_drive_ready_stable_ticks;
    } else {
      ctx.yaw_follow_drive_ready_stable_ticks = 0U;
    }
    if (ctx.yaw_follow_drive_ready_stable_ticks >= kYawFollowDriveReadyStableTicks) {
      ctx.yaw_follow_drive_ready = true;
    }
  } else if (spin_control_enabled) {
    ctx.yaw_follow_drive_ready = false;
    ctx.yaw_follow_drive_ready_stable_ticks = 0U;
  }

  // ── 7h. 目标纵向速度 ──
  float target_s_dot = 0.0f;
  float spin_target_s_dot = 0.0f;
  if (spin_control_enabled) {
    // 小陀螺：将云台系前进指令投影到车体系
    const float gimbal_heading = -input.gimbal_imu_yaw_rad;
    const float vx_gimbal = forward_input_norm;
    const bool has_vx_motion_cmd = std::fabs(vx_gimbal) > kVxInputDeadbandNorm;
    const float s_dot_cmd = has_vx_motion_cmd ? (vx_gimbal * std::cos(gimbal_heading)) : 0.0f;
    spin_target_s_dot = kSpinTranslationGain * s_dot_cmd;
    target_s_dot = 0.0f;
  } else if (!ctx.yaw_follow_drive_ready) {
    target_s_dot = 0.0f;
  } else if (forward_input_active) {
    target_s_dot = yaw_follow_drive_sign * kTargetForwardSpeedMaxMps * forward_input_norm;
  } else if (side_input_active) {
    target_s_dot = yaw_follow_drive_sign * kTargetForwardSpeedMaxMps * side_input_norm;
  }
  if (!chassis_output_enable) {
    target_s_dot = 0.0f;
  }

  // ── 7i. 纵向位置 I 项管理（PI 风格：正常行驶仅 P=速度控制；摇杆归中后 I=位移锚定）──
  // 正常行驶：expected_s 跟随 current_s，位移误差恒为零，仅速度 P 控制生效。
  // 摇杆归中：expected_s 沿速度斜坡积分，平滑构建减速轨迹；
  //          filtered_s_dot 归零后 expected_s 冻结为位置锚点（I 项自然生效）。
  const bool can_hold_position = chassis_output_enable && chassis_output.mode != chassis::Fsm::State::kDisabled &&
                                 chassis_output.mode != chassis::Fsm::State::kSpin;
  const bool driver_command_active = forward_input_active || side_input_active;
  if (!can_hold_position || driver_command_active) {
    ctx.integrate_position = false;
    ctx.expected_s = current_state.s;
  } else if (ctx.filtered_s_dot != 0.0f) {
    ctx.integrate_position = true;
    ctx.expected_s += ctx.filtered_s_dot * kControlLoopDtS;
  } else if (ctx.integrate_position && std::fabs(current_state.s_dot) < kPositionFreezeSpeedThresholdMps) {
    // 速度斜坡刚走完，snap 锚点到实际停住的位置，然后冻结
    ctx.expected_s = current_state.s;
    ctx.integrate_position = false;
  }

  // ── 7j. 纵向速度斜坡 ──
  if (spin_control_enabled) {
    ctx.filtered_s_dot = current_state.s_dot;
  } else {
    const SdotRampParams ramp_params = ResolveSdotRampParams(chassis_output.mode);
    RampValueToTarget(target_s_dot, ctx.filtered_s_dot, ramp_params);
  }

  // ── 7k. 期望状态填充（腿摆角偏置 + 偏航角速度）──
  chassis_update_input.expected.s_dot = chassis_control_output.off_ground_in_mid_high_leg
                                            ? current_state.s_dot
                                            : (spin_control_enabled ? spin_target_s_dot : ctx.filtered_s_dot);
  chassis_update_input.expected.s = ctx.expected_s;
  wl_debug.expected_s_dot_mps = chassis_update_input.expected.s_dot;
  wl_debug.expected_s_m = chassis_update_input.expected.s;
  wl_debug.filtered_s_dot_mps = ctx.filtered_s_dot;
  chassis_update_input.expected.phi = current_state.phi;
  chassis_update_input.expected.phi_dot = 0.0f;
  if (spin_control_enabled) {
    chassis_update_input.expected.theta_ll = kSpinThetaLlBiasRad;
    chassis_update_input.expected.theta_lr = 0.0f;
  } else {
    chassis_update_input.expected.theta_ll = kExpectedThetaLlBiasRad;
    chassis_update_input.expected.theta_lr = kExpectedThetaLrBiasRad;
  }
  chassis_update_input.expected.theta_b = kExpectedThetaBBiasRad;

  // ── 7l. 偏航角速度控制 ──
  const bool yaw_follow_enabled = yaw_follow_control_enabled && !spin_control_enabled;
  if (spin_control_enabled) {
    RampYawDotToTarget(kSpinTargetYawDotRadS, ctx.filtered_yaw_dot, kSpinYawRampStepRadS);
    chassis_update_input.expected.phi_dot = ctx.filtered_yaw_dot;
  } else if (!yaw_follow_enabled) {
    ctx.filtered_yaw_dot = 0.0f;
    ctx.yaw_follow_pid.Clear();
  } else {
    const float yaw_motor_rad = input.estimator_input.yaw_motor_rad;
    const float yaw_target_rad = ctx.yaw_follow_target.target_rad;
    ctx.yaw_follow_pid.Update(yaw_target_rad, yaw_motor_rad, kControlLoopDtS);
    const float target_yaw_dot = -ctx.yaw_follow_pid.out();
    RampYawDotToTarget(target_yaw_dot, ctx.filtered_yaw_dot, kYawFollowRampStepRadS);
    chassis_update_input.expected.phi_dot = ctx.filtered_yaw_dot;
  }

  // ── 7m. 底盘控制器执行 ──
  globals->chassis.Update(chassis_update_input);
  chassis_control_output = globals->chassis.GetOutput();
  g_actuators.ApplyChassisOutput(*globals, chassis_control_output, chassis_output_enable);

  // ═══════════════════════════════════════════════════════════════════════
  // 阶段 8：自瞄通信 — 云台 IMU 欧拉角→CAN 转发
  // ═══════════════════════════════════════════════════════════════════════
  if (globals->aimbot.has_value() && globals->gimbal_rx.has_value() && globals->gimbal_rx->frame_count() > 0) {
    constexpr float kRadToDeg = 180.f / kPi;
    const float yaw_deg = globals->gimbal_rx->euler_yaw_rad() * kRadToDeg;
    const float pitch_deg = globals->gimbal_rx->euler_pitch_rad() * kRadToDeg;
    const float roll_deg = globals->gimbal_rx->euler_roll_rad() * kRadToDeg;

    uint8_t aimbot_mode = 1;
    switch (chassis_input.request.combat_profile) {
      case wheel_legged::CombatProfile::kAutoAimNoMove:
        aimbot_mode = 1;
        break;
      case wheel_legged::CombatProfile::kAutoAimWithMove:
        aimbot_mode = 1;
        break;
      case wheel_legged::CombatProfile::kNormal:
      default:
        aimbot_mode = 1;
        break;
    }

    const bool referee_online =
        globals->referee.has_value() && globals->referee->online_status() == rm::device::Device::kOk;
    const uint8_t robot_id = referee_online ? globals->referee->data().robot_status.robot_id : ns::aimbot::kRobotId;
    const float referee_bullet_speed = globals->referee->data().shoot_data.initial_speed;
    const float bullet_speed =
        (referee_online && referee_bullet_speed > 0.0f) ? referee_bullet_speed : ns::aimbot::kBulletSpeedMps;
    const uint16_t imu_count = static_cast<uint16_t>(globals->gimbal_rx->frame_count() & 0xFU);
    globals->aimbot->UpdateControl(yaw_deg, pitch_deg, roll_deg, 0.0f, robot_id, aimbot_mode, imu_count, bullet_speed);

    // 自瞄 TX 调试
    wl_debug.aimbot_tx_mode = aimbot_mode;
    wl_debug.aimbot_tx_robot_id = robot_id;
  } else {
    wl_debug.aimbot_tx_mode = 0U;
    wl_debug.aimbot_tx_robot_id = 0U;
  }

  // ═══════════════════════════════════════════════════════════════════════
  // 阶段 9：调试快照导出
  // ═══════════════════════════════════════════════════════════════════════
  if (globals->gimbal_rx.has_value()) {
    wl_debug.gimbal_imu_pitch_rad = globals->gimbal_rx->pitch_rad();
    wl_debug.gimbal_imu_yaw_rad = globals->gimbal_rx->yaw_rad();
    wl_debug.gimbal_imu_gyro_x_rad_s = globals->gimbal_rx->gyro_x_rad_s();
    wl_debug.gimbal_imu_gyro_z_rad_s = globals->gimbal_rx->gyro_z_rad_s();
  } else {
    wl_debug.gimbal_imu_pitch_rad = 0.0f;
    wl_debug.gimbal_imu_yaw_rad = 0.0f;
    wl_debug.gimbal_imu_gyro_x_rad_s = 0.0f;
    wl_debug.gimbal_imu_gyro_z_rad_s = 0.0f;
  }
  // ── 云台 IMU 欧拉角（Frame C: 0x112，原始为度，转为弧度存储）──
  if (globals->gimbal_rx.has_value()) {
    constexpr float kDegToRad = kPi / 180.0f;
    wl_debug.gimbal_euler_yaw_rad = globals->gimbal_rx->euler_yaw_rad() * kDegToRad;
    wl_debug.gimbal_euler_pitch_rad = globals->gimbal_rx->euler_pitch_rad() * kDegToRad;
    wl_debug.gimbal_euler_roll_rad = globals->gimbal_rx->euler_roll_rad() * kDegToRad;
  } else {
    wl_debug.gimbal_euler_yaw_rad = 0.0f;
    wl_debug.gimbal_euler_pitch_rad = 0.0f;
    wl_debug.gimbal_euler_roll_rad = 0.0f;
  }
  if (globals->dyp_rx.has_value()) {
    wl_debug.dyp_distance_raw_left = globals->dyp_rx->distance_raw_left();
    wl_debug.dyp_distance_raw_right = globals->dyp_rx->distance_raw_right();
    wl_debug.dyp_result_left = globals->dyp_rx->last_result_left();
    wl_debug.dyp_result_right = globals->dyp_rx->last_result_right();
    wl_debug.dyp_frame_count = globals->dyp_rx->frame_count();
  }
  wl_debug.yaw_motor_status = globals->yaw_motor.has_value() ? globals->yaw_motor->status() : 0;
  wl_debug.pitch_motor_status = globals->pitch_motor.has_value() ? globals->pitch_motor->status() : 0;

  // ── 裁判系统调试 ──
  if (globals->referee.has_value() && globals->referee->online_status() == rm::device::Device::kOk) {
    wl_debug.referee_online = 1U;
    wl_debug.referee_robot_id = globals->referee->data().robot_status.robot_id;
    wl_debug.referee_bullet_speed_mps = globals->referee->data().shoot_data.initial_speed;
  } else {
    wl_debug.referee_online = 0U;
    wl_debug.referee_robot_id = 0U;
    wl_debug.referee_bullet_speed_mps = 0.0f;
  }

  // ── 自瞄 RX 调试（NUC 反馈，原始为度，转为弧度存储）──
  if (globals->aimbot.has_value()) {
    constexpr float kDegToRad = kPi / 180.0f;
    wl_debug.aimbot_rx_state = globals->aimbot->aimbot_state();
    wl_debug.aimbot_rx_target = globals->aimbot->aimbot_target();
    wl_debug.aimbot_rx_nuc_start_flag = globals->aimbot->nuc_start_flag();
    wl_debug.aimbot_rx_yaw_rad = globals->aimbot->yaw() * kDegToRad;
    wl_debug.aimbot_rx_pitch_rad = globals->aimbot->pitch() * kDegToRad;
  } else {
    wl_debug.aimbot_rx_state = 0U;
    wl_debug.aimbot_rx_target = 0U;
    wl_debug.aimbot_rx_nuc_start_flag = 0U;
    wl_debug.aimbot_rx_yaw_rad = 0.0f;
    wl_debug.aimbot_rx_pitch_rad = 0.0f;
  }

  UpdateDebugSnapshot(now_ms, input, chassis_output, gimbal_output, chassis_control_output, gimbal_control_output);
}
