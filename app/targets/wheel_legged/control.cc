#include "include/globals.hpp"
#include "include/actuators.hpp"
#include "include/chassis/stair_climb_sequence.hpp"
#include "include/chassis/stair_task_coordinator.hpp"
#include "include/debug.hpp"
#include "main.h"
#include <algorithm>
#include <cmath>
#include "ui.hpp"
#include "include/input.hpp"
#include "include/state_ctx.hpp"
float debug_mid_target_theta;
f32 yaw_motor_tau, pitch_motor_tau;
float yaw_motor_pos, pitch_motor_pos;
float yaw_motor_vel, pitch_motor_vel;
bool init_flag;
int times = 0;
/**
 * @file  targets/wheel_legged/control.cc
 * @brief 500Hz 主控制循环：输入采集、状态机更新、底盘解算、执行器输出与调试同步
 */
float debug_pitch_motor_raw_pos_rad;
DebugSnapshot wl_debug __attribute__((section(".sram4")));

namespace {

namespace ns = wheel_legged::params::active;
using namespace wheel_legged::control_loop;

// ── 控制循环参数（常量化，避免逻辑中散落魔数）──
constexpr float kControlLoopDtS = ns::control_loop::kControlLoopDtS;
constexpr float kPi = ns::kPi;
constexpr float kTargetForwardSpeedMaxMps = ns::control_loop::kTargetForwardSpeedMaxMps;
constexpr float kTargetForwardSpeedMaxNoScMps = ns::control_loop::kTargetForwardSpeedMaxNoScMps;
constexpr float kTargetForwardSpeedMaxHighLegMps = ns::control_loop::kTargetForwardSpeedMaxHighLegMps;
constexpr float kTargetForwardSpeedMaxMidLegMps = ns::control_loop::kTargetForwardSpeedMaxMidLegMps;
constexpr float kTargetSpeedBiasLowLegMps = ns::control_loop::kTargetSpeedBiasLowLegMps;
constexpr float kTargetSpeedBiasMidLegMps = ns::control_loop::kTargetSpeedBiasMidLegMps;
constexpr float kTargetSpeedBiasMidLegFMps = ns::control_loop::kTargetSpeedBiasMidLegFMps;
constexpr float kTargetSpeedBiasHighLegMps = ns::control_loop::kTargetSpeedBiasHighLegMps;
constexpr float kVxInputDeadbandNorm = ns::control_loop::kVxInputDeadbandNorm;
constexpr float kVyInputDeadbandNorm = ns::control_loop::kVyInputDeadbandNorm;
constexpr float kYawFollowRampStepRadS = ns::control_loop::kYawFollowRampStepRadS;
constexpr float kYawFollowRampStepRadNoScS = ns::control_loop::kYawFollowRampStepRadNoScS;
constexpr float kSpinYawRampStepRadS = ns::control_loop::kSpinYawRampStepRadS;
constexpr float kSpinExitYawRampStepRadS = ns::control_loop::kSpinExitYawRampStepRadS;
constexpr float kSpinTargetYawDotRadS1 = ns::control_loop::kSpinTargetYawDotRadS1;
constexpr float kSpinTargetYawDotRadS2 = ns::control_loop::kSpinTargetYawDotRadS2;
constexpr float kSpinTargetYawDotRadS3 = ns::control_loop::kSpinTargetYawDotRadS3;
constexpr float kSpinTargetYawDotRadS4 = ns::control_loop::kSpinTargetYawDotRadS4;
constexpr float kSpinTargetYawDotRadNoScS1 = ns::control_loop::kSpinTargetYawDotRadNoScS1;
constexpr float kSpinTargetYawDotRadNoScS2 = ns::control_loop::kSpinTargetYawDotRadNoScS2;
constexpr float kSpinTargetYawDotRadNoScS3 = ns::control_loop::kSpinTargetYawDotRadNoScS3;
constexpr float kSpinTargetYawDotRadNoScS4 = ns::control_loop::kSpinTargetYawDotRadNoScS4;
constexpr float kSpinExitYawAlignThresholdRad = ns::control_loop::kSpinExitYawAlignThresholdRad;
constexpr float kSpinTranslationGain = ns::control_loop::kSpinTranslationGain;
constexpr float kSpinThetaLlBiasRad = ns::control_loop::kSpinThetaLlBiasRad;
constexpr float kExpectedThetaLlBiasRadLowLeg = ns::control_loop::kExpectedThetaLlBiasRadLowLeg;
constexpr float kExpectedThetaLrBiasRadLowLeg = ns::control_loop::kExpectedThetaLrBiasRadLowLeg;
constexpr float kExpectedThetaLlBiasRadMidLeg = ns::control_loop::kExpectedThetaLlBiasRadMidLeg;
constexpr float kExpectedThetaLrBiasRadMidLeg = ns::control_loop::kExpectedThetaLrBiasRadMidLeg;
constexpr float kExpectedThetaLlBiasRadHighLeg = ns::control_loop::kExpectedThetaLlBiasRadHighLeg;
constexpr float kExpectedThetaLrBiasRadHighLeg = ns::control_loop::kExpectedThetaLrBiasRadHighLeg;
constexpr float kSpinThetaLrBiasRad = ns::control_loop::kSpinThetaLrBiasRad;
constexpr float kSpinLegLengthBiasM = ns::control_loop::kSpinLegLengthBiasM;
constexpr float kSpinThetaBBiasRad = ns::control_loop::kSpinThetaBBiasRad;
constexpr float kJumpThetaLlBiasRad = ns::control_loop::kJumpThetaLlBiasRad;
constexpr float kJumpThetaLrBiasRad = ns::control_loop::kJumpThetaLrBiasRad;
constexpr float kExpectedThetaBBiasRad = ns::control_loop::kExpectedThetaBBiasRad;
constexpr float kLandingDecelThetaGain = ns::control_loop::kLandingDecelThetaGain;
constexpr float kLandingDecelThetaMaxRad = ns::control_loop::kLandingDecelThetaMaxRad;

constexpr float kLandingDecelThetaRampStepRad = ns::control_loop::kLandingDecelThetaRampStepRad;
constexpr std::uint32_t kLandingDecelOffGroundMinMs = ns::control_loop::kLandingDecelOffGroundMinMs;
constexpr std::uint32_t kLandingDecelStableDurationMs = ns::control_loop::kLandingDecelStableDurationMs;
constexpr uint32_t kGimbalStartupYawAlignStableTicks = ns::control_loop::kGimbalStartupYawAlignStableTicks;
constexpr uint32_t kYawFollowDriveReadyStableTicks = ns::control_loop::kYawFollowDriveReadyStableTicks;
constexpr float kYawFollowFixedTargetRad = ns::control_loop::kYawFollowFixedTargetRad;

chassis_runtime::Actuators g_actuators{};
chassis::StairTaskCoordinator g_stair_task_coordinator{};
chassis::StairClimbSequence g_stair_sequence{};

/// 根据底盘功率限制返回小陀螺目标自旋角速度 [rad/s]
/// ≤55W / 55-65W / 65-75W / >75W 四档
/// kBuckBoost(bit2) 或 kShortCircuit(bit3) 置位时使用无超电档位
constexpr uint8_t kScFatalMask = (1U << 2) | (1U << 3);
float ResolveSpinTargetYawDot(uint16_t chassis_power_limit, uint8_t supercap_error_code) {
  const bool has_supercap = (supercap_error_code & kScFatalMask) == 0;
  if (chassis_power_limit <= 55) return has_supercap ? kSpinTargetYawDotRadS1 : kSpinTargetYawDotRadNoScS1;
  if (chassis_power_limit <= 65) return has_supercap ? kSpinTargetYawDotRadS2 : kSpinTargetYawDotRadNoScS2;
  if (chassis_power_limit <= 75) return has_supercap ? kSpinTargetYawDotRadS3 : kSpinTargetYawDotRadNoScS3;
  return has_supercap ? kSpinTargetYawDotRadS4 : kSpinTargetYawDotRadNoScS4;
}

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
    tc_state.stair_descend_hold = false;
  }

  // ═══════════════════════════════════════════════════════════════════════
  // 阶段 1：硬件反馈采集 + DR16 语义折叠
  // ═══════════════════════════════════════════════════════════════════════
  UpdateRawFeedbackAndInputSnapshot(*globals, g_actuators, input, dr16_state, tc_state);
  globals->ui_refresh_key = tc_state.e_ui_refresh;
  input.ui_refresh_key = tc_state.e_ui_refresh;

  wl_debug.motor_reenable_chassis_trig = 0U;
  wl_debug.motor_reenable_gimbal_trig = 0U;

  // ═══════════════════════════════════════════════════════════════════════
  // 阶段 2：状态机决策
  // ═══════════════════════════════════════════════════════════════════════
  const auto &stair_params = ns::chassis_fsm::kStairClimb;
  const bool stair_output_enabled = input.mode_request.input_valid &&
                                    input.mode_request.domain_request != wheel_legged::DomainRequest::kDisabled &&
                                    !input.mode_request.standby;
  const bool stair_contact_detected =
      chassis_control_output.current_state.theta_ll > stair_params.contact_theta_threshold_rad &&
      chassis_control_output.current_state.theta_lr > stair_params.contact_theta_threshold_rad;
  const bool stair_high_leg_ready = std::fabs(chassis_control_output.mean_leg_length_m -
                                              stair_params.high_leg_length_m) <= stair_params.leg_length_tolerance_m;
  const auto &previous_sequence_output = g_stair_sequence.output();
  const auto &stair_task_output = g_stair_task_coordinator.Update({
      .request = input.mode_request.stair_task_request,
      .contact_detected = stair_contact_detected,
      .high_leg_ready = stair_high_leg_ready,
      .posture_valid = chassis_control_output.posture_valid,
      .output_enabled = stair_output_enabled,
      .sequence_succeeded = previous_sequence_output.succeeded,
      .sequence_aborted = previous_sequence_output.aborted,
      .sequence_abort_reason = previous_sequence_output.abort_reason,
  });
  if (stair_task_output.reset_sequence) {
    g_stair_sequence.Reset();
  }
  const auto &stair_sequence_output = g_stair_sequence.Update({
      .start = stair_task_output.start_sequence,
      .cancel = stair_task_output.cancel_sequence,
      .output_enabled = stair_output_enabled,
      .posture_valid = chassis_control_output.posture_valid,
      .mean_leg_length_m = chassis_control_output.mean_leg_length_m,
      .theta_ll_rad = chassis_control_output.current_state.theta_ll,
      .theta_lr_rad = chassis_control_output.current_state.theta_lr,
      .theta_ll_dot_rad_s = chassis_control_output.current_state.theta_ll_dot,
      .theta_lr_dot_rad_s = chassis_control_output.current_state.theta_lr_dot,
      .theta_b_rad = chassis_control_output.current_state.theta_b,
      .theta_b_dot_rad_s = chassis_control_output.current_state.theta_b_dot,
      .roll_rad = input.estimator_input.imu.roll_rad,
      .tick_ms = now_ms,
  });

  auto chassis_input = BuildChassisFsmInput(input, now_ms, chassis_control_output);
  if (stair_task_output.request_high_leg) {
    chassis_input.request.leg_request = wheel_legged::LegProfile::kHigh;
  }
  if (stair_task_output.force_low_leg) {
    chassis_input.request.leg_request = wheel_legged::LegProfile::kLow;
  }
  chassis_input.request.stair_task_active = stair_task_output.task_active;
  chassis_input.request.stair_task_recovery_required = stair_task_output.recovery_required;
  {
    const float nearest_forward = SelectNearestYawCenterTarget(input.estimator_input.yaw_motor_rad);
    const float yaw_err = rm::modules::Wrap(nearest_forward - input.estimator_input.yaw_motor_rad, -kPi, kPi);
    chassis_input.request.spin_exit_yaw_aligned = std::fabs(yaw_err) < kSpinExitYawAlignThresholdRad;
  }
  const chassis::Fsm::Output chassis_output = globals->chassis_fsm.Update(chassis_input);

  // ── recovery→正常过渡：清除中腿长保持，落地后保持低腿长 ──
  {
    static chassis::Fsm::State prev_chassis_mode_for_recovery = chassis::Fsm::State::kDisabled;
    const bool is_recovery = (chassis_output.mode == chassis::Fsm::State::kRecoveryFallCheck ||
                              chassis_output.mode == chassis::Fsm::State::kRecoverySelfRight);
    const bool was_recovery = (prev_chassis_mode_for_recovery == chassis::Fsm::State::kRecoveryFallCheck ||
                               prev_chassis_mode_for_recovery == chassis::Fsm::State::kRecoverySelfRight);
    if (was_recovery && !is_recovery) {
      tc_state.mid_leg_hold = false;
      tc_state.stair_descend_hold = false;
      tc_state.auto_small_jump_enabled = false;
    }
    prev_chassis_mode_for_recovery = chassis_output.mode;
  }

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
  gimbal_update_input.aimbot_is_rune =
      (chassis_input.request.combat_profile == wheel_legged::CombatProfile::kAutoAimFuSmall ||
       chassis_input.request.combat_profile == wheel_legged::CombatProfile::kAutoAimFuBig);
  if (gimbal_update_input.aimbot_mode && globals->aimbot.has_value()) {
    constexpr float kDegToRad = ns::kPi / 180.0f;
    gimbal_update_input.aimbot_yaw_vel = globals->aimbot->yaw_vel() * kDegToRad;
    gimbal_update_input.aimbot_pitch_vel = globals->aimbot->pitch_vel() * kDegToRad;
    gimbal_update_input.aimbot_yaw_acc = globals->aimbot->yaw_acc() * kDegToRad;
    gimbal_update_input.aimbot_pitch_acc = globals->aimbot->pitch_acc() * kDegToRad;
  }
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
  gimbal_update_input.ident = &globals->gimbal_ident;
  gimbal_update_input.test_profile = gimbal_output.control.gimbal_test_profile;
  globals->gimbal.Update(gimbal_update_input);
  gimbal_control_output = globals->gimbal.GetOutput();
  g_actuators.ApplyGimbalOutput(*globals, gimbal_control_output);
  wl_debug.gimbal_motors_enabled_latched = g_actuators.gimbal_motors_enabled_latched() ? 1U : 0U;

  // 云台电机心跳检测：全部 offline→online 边沿触发重使能
  {
    static bool prev_all_ok = false;
    const bool yaw_ok =
        globals->yaw_motor.has_value() && globals->yaw_motor->online_status() == rm::device::Device::kOk;
    const bool pitch_ok =
        globals->pitch_motor.has_value() && globals->pitch_motor->online_status() == rm::device::Device::kOk;
    const bool cur_all_ok = yaw_ok && pitch_ok;

    wl_debug.yaw_motor_online = yaw_ok ? 1U : 0U;
    wl_debug.pitch_motor_online = pitch_ok ? 1U : 0U;

    if (cur_all_ok && !prev_all_ok && gimbal_control_output.gimbal_enabled) {
      g_actuators.ResetGimbalMotorsLatch();
      ctx.gimbal_startup_align_complete = false;
      ctx.gimbal_startup_align_was_active = false;
      ctx.gimbal_startup_align_stable_ticks = 0U;
      wl_debug.motor_reenable_gimbal_trig = 1U;
    }
    prev_all_ok = cur_all_ok;
  }

  // 辨识模式串口数据发送
  if (gimbal_control_output.ident_data_pending && gimbal_control_output.ident_tx_data != nullptr) {
    globals->no_dtcm->ident_uart.Write(reinterpret_cast<const rm::u8 *>(gimbal_control_output.ident_tx_data),
                                       gimbal_control_output.ident_tx_len, 5);
  }

  // 超级电容 TX：每周期从裁判系统读取功率上限和缓冲能量，下发给超级电容
  if (globals->supercap.has_value() && globals->referee.has_value()) {
    rm::device::GkSupercap::TxData supercap_tx{};
    supercap_tx.enable_dcdc = 1;
    supercap_tx.system_restart = 0;
    supercap_tx.resv0 = 0;
    supercap_tx.feedback_referee_power_limit = globals->referee->data().robot_status.chassis_power_limit;
    supercap_tx.feedback_referee_energy_buffer = globals->referee->data().power_heat_data.buffer_energy;
    globals->supercap->Update(supercap_tx);
  }

  // ═══════════════════════════════════════════════════════════════════════
  // 阶段 5：发射机构控制（变体分支）
  // ═══════════════════════════════════════════════════════════════════════
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  // Hero：三摩擦轮 + DM 拨盘，ShootController 内置 5 状态机自行下发
  {
    const bool shooter_enter = (gimbal_output.mode == gimbal::Fsm::State::kCombat);
    const bool manual_fire = input.dr16.dial < ns::shoot::kFireDialThreshold || input.tc_remote.left_button;
    // const bool fire_flag = (globals->aimbot->aimbot_state() == 0 && manual_fire) ||
    const bool fire_flag =
        (manual_fire) || (gimbal_output.control.active_target_source == wheel_legged::TargetSource::kHost &&
                          (manual_fire || (globals->aimbot->aimbot_state() >> 1) & 1));
    globals->shoot_controller.Update(shooter_enter, fire_flag, tc_state.fric_speed_target_rpm,
                                     globals->referee->data().robot_status.shooter_barrel_heat_limit -
                                         globals->referee->data().power_heat_data.shooter_42mm_barrel_heat);
    wl_debug.booster_raw_pos_rad = globals->shoot_controller.booster_pos();
    wl_debug.fw_raw_rpm_1 = globals->fw_motor_1.has_value() ? static_cast<float>(globals->fw_motor_1->rpm()) : 0.0f;
    wl_debug.fw_raw_rpm_2 = globals->fw_motor_2.has_value() ? static_cast<float>(globals->fw_motor_2->rpm()) : 0.0f;
    wl_debug.fw_raw_rpm_3 = globals->fw_motor_3.has_value() ? static_cast<float>(globals->fw_motor_3->rpm()) : 0.0f;
    // rm::device::DjiMotorBase::SendCommand(*globals->gimbal_can);
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
    wl_debug.fric_left_rpm = fric_left_rpm;
    wl_debug.fric_right_rpm = -fric_right_rpm;
    wl_debug.shoot_fric_ready = globals->shoot.fric_ready() ? 1U : 0U;
    if (globals->dial.has_value()) {
      globals->shoot.dial_encoder_counter().Update(globals->dial->encoder());
    }
    const float dial_encoder = globals->dial.has_value() ? -static_cast<float>(globals->shoot.dial_linear_pos()) : 0.0f;
    wl_debug.dial_encoder_raw = static_cast<float>(globals->shoot.dial_linear_pos());
    const float dial_rpm = globals->dial.has_value() ? -static_cast<float>(globals->dial->rpm()) : 0.0f;
    const bool manual_fire =
        input.dr16.dial < wheel_legged::params::active::shoot::kDialFireThreshold || input.tc_remote.left_button;
    const bool fire_flag =
        manual_fire || (gimbal_output.control.active_target_source == wheel_legged::TargetSource::kHost &&
                        globals->aimbot->aimbot_state() >> 1 & 1);

    wl_debug.shoot_manual_fire = manual_fire ? 1U : 0U;

    const bool ref_online =
        globals->referee.has_value() && globals->referee->online_status() == rm::device::Device::kOk;
    const uint16_t heat_limit =
        ref_online ? globals->referee->data().robot_status.shooter_barrel_heat_limit : ns::shoot::kDefaultHeatLimit;
    const uint16_t cooling_rate = ref_online ? globals->referee->data().robot_status.shooter_barrel_cooling_value
                                             : ns::shoot::kDefaultCoolingRate;
    globals->shoot.SetHeatParams(heat_limit, cooling_rate);

    const bool single_shot = input.mode_request.combat_profile == wheel_legged::CombatProfile::kAutoAimFuSmall ||
                             input.mode_request.combat_profile == wheel_legged::CombatProfile::kAutoAimFuBig;
    wl_debug.shoot_single_shot_mode = single_shot ? 1U : 0U;
    const uint16_t ref_barrel_heat =
        ref_online ? globals->referee->data().power_heat_data.shooter_17mm_1_barrel_heat : 0U;
    const auto shoot_output =
        globals->shoot.Update(fric_left_rpm, fric_right_rpm, dial_encoder, dial_rpm, kControlLoopDtS, fire_flag,
                              in_combat, tc_state.fric_speed_target_rpm, single_shot, ref_barrel_heat);
    g_actuators.ApplyShootOutput(*globals, shoot_output);
    wl_debug.shot_count = globals->shoot.shot_count();
    wl_debug.shoot_dial_current = shoot_output.dial_current;
    wl_debug.shoot_local_heat = globals->shoot.current_heat();
    wl_debug.shoot_heat_limit = globals->shoot.heat_limit();
    wl_debug.shoot_cooling_rate = globals->shoot.cooling_rate();
    wl_debug.shoot_heat_suppressed = globals->shoot.heat_over_limit() ? 1U : 0U;
    wl_debug.shoot_mode = globals->shoot.shoot_mode();
    wl_debug.shoot_single_complete = globals->shoot.single_complete() ? 1U : 0U;
    wl_debug.shoot_fire_flag = fire_flag ? 1U : 0U;
    wl_debug.shoot_effective_fire = (fire_flag && !globals->shoot.heat_over_limit()) ? 1U : 0U;
    wl_debug.shoot_loader_pos_error = globals->shoot.loader_pos_error();
    wl_debug.shoot_loader_pos_target = globals->shoot.loader_pos_target();
    wl_debug.shoot_loader_pos_feedback = dial_encoder;
    wl_debug.shoot_loader_pos_pid_out = globals->shoot.loader_pos_pid_out();
    wl_debug.shoot_loader_spd_target = globals->shoot.loader_spd_target();
    wl_debug.shoot_loader_spd_feedback = dial_rpm;
    wl_debug.shoot_loader_spd_pid_out = globals->shoot.loader_spd_pid_out();
  }
#endif

  wl_debug.fric_speed_target_rpm = tc_state.fric_speed_target_rpm;

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
  const bool chassis_output_enable = chassis_posture_invalid
                                         ? chassis_output.control.enable_dm
                                         : chassis_output.control.enable_dm && chassis_startup_ready;

  // ═══════════════════════════════════════════════════════════════════════
  // 阶段 7：底盘控制
  // ═══════════════════════════════════════════════════════════════════════

  // ── 7a. 底盘控制器输入组装 ──
  chassis::Chassis::UpdateInput chassis_update_input{};
  chassis_update_input.fsm_mode = chassis_output.mode;
  chassis_update_input.enable_output = chassis_output_enable;
  chassis_update_input.run_chassis_update = chassis_output.control.run_chassis_update;
  chassis_update_input.spin_enable = chassis_output.control.spin_enable;
  chassis_update_input.recovery_manual_mode = chassis_input.request.recovery_manual_mode;
  chassis_update_input.manual_left_leg_speed = chassis_input.request.manual_left_leg_speed;
  chassis_update_input.manual_right_leg_speed = chassis_input.request.manual_right_leg_speed;
  chassis_update_input.motion_target.leg_length_m = chassis_output.control.target_leg_length_m;
  if (stair_sequence_output.controls_motion && chassis_output.mode == chassis::Fsm::State::kStairTask) {
    chassis_update_input.motion_target = stair_sequence_output.target;
  }
  chassis_update_input.keyboard_active = input.tc_remote.valid && !input.tc_remote.tc_from_dr16;
  chassis_update_input.estimator_input = input.estimator_input;
  chassis_update_input.estimator_input.dt_s = kControlLoopDtS;

  // ── 7b. 模式切换处理 ──
  const auto &current_state = chassis_control_output.current_state;
  const bool mode_changed = (chassis_output.mode != ctx.last_chassis_mode);
  const auto is_spin_like = [](chassis::Fsm::State s) {
    return s == chassis::Fsm::State::kSpin || s == chassis::Fsm::State::kSpinExitPending;
  };
  const bool last_was_spin = is_spin_like(ctx.last_chassis_mode);
  const bool now_is_spin = is_spin_like(chassis_output.mode);
  const bool now_is_standby = chassis_output.mode == chassis::Fsm::State::kStandby;
  const bool cross_spin = (last_was_spin != now_is_spin);
  if (mode_changed) {
    ctx.ResetOnModeChange(current_state.s, current_state.s_dot);
    if (cross_spin) {
      if (!now_is_spin) {
        // 退出小陀螺：偏航已对齐正前方，不继承 phi_dot，直接交给 yaw follow PID 控制
      } else {
        // 进入小陀螺：继承当前车体角速度做平滑起步，强制对齐正前方
        ctx.filtered_yaw_dot = chassis_control_output.current_state.phi_dot;
        ctx.yaw_follow_align_mode = YawFollowAlignMode::kForward;
      }
    }
    ctx.last_chassis_mode = chassis_output.mode;
  }

  // ── 7c. 驾驶输入解析 ──
  const bool dr16_online = input.dr16.online;
  const bool tc_remote_active = input.tc_remote.valid;
  const bool has_drive_input = dr16_online || tc_remote_active;

  const auto drive = ResolveDriveInput(input.dr16, input.tc_remote, tc_state.dr16_parallel);
  const float forward_input_norm = drive.forward;
  const float side_input_norm = drive.side;
  const bool forward_input_active = std::fabs(forward_input_norm) > kVxInputDeadbandNorm;
  const bool side_input_active = std::fabs(side_input_norm) > kVyInputDeadbandNorm;

  // ── 7d. 偏航跟随模式选择（非自旋模式专用，自旋期间侧向指令由 7h 全向投影独立处理）──
  YawFollowAlignMode requested_yaw_follow_align_mode = ctx.yaw_follow_align_mode;
  if (!has_drive_input || chassis_input.request.domain_request == wheel_legged::DomainRequest::kDisabled) {
    requested_yaw_follow_align_mode = YawFollowAlignMode::kForward;
  } else if (now_is_spin) {
    // 自旋期间冻结 yaw follow 模式，不处理方向性输入
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
    if (!ctx.spin_exit_recovery) {
      ctx.filtered_yaw_dot = 0.0f;
    }
    ctx.yaw_follow_pid.Clear();
  }

  // R 键重置底盘正方向（静止时也生效，目标为固定偏置角）
  wl_debug.reset_yaw_request = input.mode_request.reset_yaw_request ? 1 : 0;
  if (input.mode_request.reset_yaw_request) {
    ctx.yaw_follow_target = {kYawFollowFixedTargetRad, 1.0f};
    ctx.yaw_follow_align_mode = YawFollowAlignMode::kForward;
    ctx.yaw_follow_target_initialized = true;
    ctx.yaw_follow_drive_ready = false;
    ctx.yaw_follow_drive_ready_stable_ticks = 0U;
    ctx.filtered_yaw_dot = 0.0f;
    ctx.yaw_follow_pid.Clear();
  }

  // R 键云台转 180°：翻转底盘驱动方向，抑制偏航跟随直到云台旋转完成
  if (input.mode_request.flip_180_request) {
    ctx.yaw_follow_target.drive_sign = -ctx.yaw_follow_target.drive_sign;
    ctx.yaw_follow_pid.Clear();
    ctx.flip_180_in_progress = true;
    ctx.flip_180_ticks = 0U;
  }

  if (!ctx.yaw_follow_target_initialized) {
    ctx.yaw_follow_target =
        SelectNearestYawTarget(input.estimator_input.yaw_motor_rad, YawFollowTargetOffset(ctx.yaw_follow_align_mode));
    ctx.yaw_follow_target_initialized = true;
  }

  // ── 7f. 纵向速度目标计算 ──
  const float yaw_follow_drive_sign = YawFollowDriveSign(ctx.yaw_follow_align_mode, ctx.yaw_follow_target.drive_sign);
  const bool spin_control_enabled = (chassis_output.mode == chassis::Fsm::State::kSpin ||
                                     chassis_output.mode == chassis::Fsm::State::kSpinExitPending) &&
                                    chassis_output_enable && chassis_output.control.run_chassis_update;
  const bool jump_control_enabled =
      (chassis_output.mode == chassis::Fsm::State::kJumpPrep || chassis_output.mode == chassis::Fsm::State::kJumpPush ||
       chassis_output.mode == chassis::Fsm::State::kJumpRecover) &&
      chassis_output_enable && chassis_output.control.run_chassis_update;

  // ── 7g. 偏航就绪判稳 ──
  const bool yaw_follow_control_enabled = chassis_output.mode != chassis::Fsm::State::kDisabled && !now_is_standby &&
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
  const uint8_t sc_err = globals->supercap.has_value() ? globals->supercap->rx_data_.error_code : 0xFFU;
  const bool has_supercap = (sc_err & kScFatalMask) == 0;
  const float default_speed_max = has_supercap ? kTargetForwardSpeedMaxMps : kTargetForwardSpeedMaxNoScMps;
  const float forward_speed_base =
      (chassis_output.mode == chassis::Fsm::State::kHighLeg || chassis_output.mode == chassis::Fsm::State::kStairTask)
          ? kTargetForwardSpeedMaxHighLegMps
      : (chassis_output.mode == chassis::Fsm::State::kMidLeg && input.mode_request.mid_leg_f)
          ? kTargetForwardSpeedMaxMidLegMps
          : default_speed_max;
  const float forward_speed_bias =
      (chassis_output.mode == chassis::Fsm::State::kLowLeg) ? kTargetSpeedBiasLowLegMps
      : (chassis_output.mode == chassis::Fsm::State::kMidLeg && input.mode_request.mid_leg_f)
          ? kTargetSpeedBiasMidLegFMps
      : (chassis_output.mode == chassis::Fsm::State::kMidLeg) ? kTargetSpeedBiasMidLegMps
      : (chassis_output.mode == chassis::Fsm::State::kHighLeg || chassis_output.mode == chassis::Fsm::State::kStairTask)
          ? kTargetSpeedBiasHighLegMps
          : 0.0f;
  const float forward_max_speed = forward_speed_base;
  float target_s_dot = 0.0f;
  float spin_target_s_dot = 0.0f;
  if (spin_control_enabled) {
    // 小陀螺平移：把云台系速度指令投影到底盘当前纵向轴，底盘自旋一圈后的平均位移沿云台指令方向。
    const float vx_gimbal = side_input_active ? forward_max_speed * side_input_norm : 0.0f;
    const float vy_gimbal = forward_input_active ? forward_max_speed * forward_input_norm : 0.0f;

    const float spin_phase_rad =
        rm::modules::Wrap(input.estimator_input.yaw_motor_rad - kYawFollowFixedTargetRad, -kPi, kPi);
    spin_target_s_dot =
        kSpinTranslationGain * (-vx_gimbal * std::cos(spin_phase_rad) + vy_gimbal * std::sin(spin_phase_rad));
    target_s_dot = 0.0f;
  } else if (!ctx.yaw_follow_drive_ready) {
    target_s_dot = 0.0f;
  } else if (forward_input_active) {
    target_s_dot = yaw_follow_drive_sign * forward_max_speed * forward_input_norm;
  } else if (side_input_active) {
    target_s_dot = yaw_follow_drive_sign * forward_max_speed * side_input_norm;
  }
  target_s_dot += forward_speed_bias;
  if (!chassis_output_enable || now_is_standby) {
    target_s_dot = 0.0f;
  }

  // // ── 摩擦圆限速（转向优先）──
  // // 当目标偏航速率与目标速度的归一化矢量和超出单位圆时，限制速度以保证转向。
  // if (target_s_dot != 0.0f && chassis_output_enable) {
  //
  //   const float yaw_dot = spin_control_enabled ? kSpinTargetYawDotRadS : ctx.filtered_yaw_dot;
  //   const float speed_norm = std::fabs(target_s_dot) / forward_max_speed;
  //   const float yaw_norm = std::fabs(yaw_dot) / ns::control_loop::kMaxSafeYawRateRadS;
  //   const float sum_sq = speed_norm * speed_norm + yaw_norm * yaw_norm;
  //   flag =0;
  //   if (sum_sq > 1.0f) {
  //     flag = 1;
  //     const float max_speed = forward_max_speed * std::sqrt(1.0f - yaw_norm * yaw_norm);
  //     target_s_dot = std::copysign(max_speed, target_s_dot);
  //   }
  // }

  // ── 7i. 纵向位置 I 项管理（PI 风格：正常行驶仅 P=速度控制；摇杆归中后 I=位移锚定）──
  // 优先级从高到低：
  //   1. 不可保持 / 驾驶员推杆 / 斜坡未走完 → 复位计数器，跟随 current_s（无位置误差）
  //   2. 已锚定 → 保持冻结（跳过后续判断）
  //   3. 斜坡归零但车速超阈值 → 递增超时计数器，超时强冻，否则继续跟随
  //   4. 斜坡归零 且 车速低于阈值 → 正常冻结锚点
  const bool can_hold_position = chassis_output_enable && chassis_output.mode != chassis::Fsm::State::kDisabled &&
                                 !now_is_standby && chassis_output.mode != chassis::Fsm::State::kSpin;
  const bool driver_command_active = forward_input_active || side_input_active;
  if (!can_hold_position || driver_command_active || ctx.filtered_s_dot != 0.0f) {
    ctx.position_hold_timeout_ticks = 0U;
    ctx.integrate_position = false;
    ctx.position_frozen_by_timeout = false;
    ctx.expected_s = current_state.s;
  } else if (ctx.integrate_position) {
    // 已锚定（正常冻结或超时强冻）：保持不动
  } else if (std::fabs(current_state.s_dot) > ns::control_loop::kPositionFreezeSpeedThresholdMps) {
    // 斜坡归零但车速仍超标：等待物理静止或超时兜底
    ctx.position_hold_timeout_ticks++;
    if (ctx.position_hold_timeout_ticks >= ns::control_loop::kPositionHoldTimeoutTicks) {
      ctx.integrate_position = true;  // 超时强冻
      ctx.position_frozen_by_timeout = true;
    } else {
      ctx.expected_s = current_state.s;  // 继续跟随，等待静止
    }
  } else {
    ctx.position_hold_timeout_ticks = 0U;
    ctx.integrate_position = true;  // 正常冻结：车速已低于阈值
    ctx.position_frozen_by_timeout = false;
  }

  // ── 7j. 纵向速度斜坡 ──
  if (spin_control_enabled) {
    ctx.filtered_s_dot = current_state.s_dot;
  } else {
    const SdotRampParams ramp_params = ResolveSdotRampParams(chassis_output.mode, input.mode_request.mid_leg_f);
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
  wl_debug.position_frozen_by_timeout = ctx.position_frozen_by_timeout ? 1U : 0U;
  chassis_update_input.expected.phi = current_state.phi;
  chassis_update_input.expected.phi_dot = 0.0f;

  // ── 落地减速：中腿长离地→落地边沿触发，theta_bias = k * s_dot 辅助减速 ──
  // const bool is_mid_leg = chassis_output.mode == chassis::Fsm::State::kMidLeg;
  // const bool off_ground = chassis_control_output.off_ground_in_mid_high_leg;
  //
  // // 离地持续时间计数（防单帧误判）—— 先判边沿再更新计数器
  // const uint32_t off_ground_min_ticks = kLandingDecelOffGroundMinMs / 2U;  // 2ms/tick
  // const bool landing_edge = is_mid_leg && ctx.prev_off_ground_in_mid_leg && !off_ground &&
  //                           ctx.off_ground_duration_ticks >= off_ground_min_ticks;
  // ctx.prev_off_ground_in_mid_leg = off_ground;
  //
  // if (off_ground && is_mid_leg) {
  //   ctx.off_ground_duration_ticks++;
  // } else {
  //   ctx.off_ground_duration_ticks = 0U;
  // }
  //
  // if (landing_edge) {
  //   ctx.landing_decel_active = true;
  //   ctx.landing_stable_ticks = 0U;
  // }
  //
  // if (ctx.landing_decel_active) {
  //   const bool speed_low = std::fabs(current_state.s_dot) < ns::control_loop::kPositionFreezeSpeedThresholdMps;
  //   if (speed_low) {
  //     ctx.landing_stable_ticks++;
  //   } else {
  //     ctx.landing_stable_ticks = 0U;
  //   }
  //
  //   const uint32_t stable_ticks_needed = kLandingDecelStableDurationMs / 2U;  // 500Hz → 2ms/tick
  //   if (ctx.landing_stable_ticks >= stable_ticks_needed) {
  //     ctx.landing_decel_active = false;
  //   }
  // }
  //
  // {
  //   const float target_bias = ctx.landing_decel_active ? std::clamp(kLandingDecelThetaGain * current_state.s_dot,
  //                                                                   -kLandingDecelThetaMaxRad,
  //                                                                   kLandingDecelThetaMaxRad)
  //                                                      : 0.0f;
  //   const SdotRampParams theta_ramp{kLandingDecelThetaRampStepRad, kLandingDecelThetaRampStepRad};
  //   RampValueToTarget(target_bias, ctx.landing_theta_bias, theta_ramp);
  // }

  if (spin_control_enabled) {
    chassis_update_input.expected.theta_ll = kSpinThetaLlBiasRad;
    chassis_update_input.expected.theta_lr = kSpinThetaLrBiasRad;
    chassis_update_input.expected.theta_b = kSpinThetaBBiasRad;
  } else if (jump_control_enabled) {
    chassis_update_input.expected.theta_ll = kJumpThetaLlBiasRad;
    chassis_update_input.expected.theta_lr = kJumpThetaLrBiasRad;
    chassis_update_input.expected.theta_b = 0.0f;
  } else if (stair_sequence_output.controls_motion && chassis_output.mode == chassis::Fsm::State::kStairTask) {
    chassis_update_input.expected.theta_ll = stair_sequence_output.target.theta_ll_rad;
    chassis_update_input.expected.theta_lr = stair_sequence_output.target.theta_lr_rad;
    chassis_update_input.expected.theta_b = stair_sequence_output.target.theta_b_rad;
  } else if (chassis_output.mode == chassis::Fsm::State::kHighLeg ||
             chassis_output.mode == chassis::Fsm::State::kStairTask) {
    chassis_update_input.expected.theta_ll = kExpectedThetaLlBiasRadHighLeg;
    chassis_update_input.expected.theta_lr = kExpectedThetaLrBiasRadHighLeg;
  } else if (chassis_output.mode == chassis::Fsm::State::kMidLeg) {
    if (chassis_control_output.mid_leg_dip_active) {
      chassis_update_input.expected.theta_ll = kExpectedThetaLlBiasRadLowLeg;
      chassis_update_input.expected.theta_lr = kExpectedThetaLrBiasRadLowLeg;
    } else {
      chassis_update_input.expected.theta_ll = kExpectedThetaLlBiasRadMidLeg;
      chassis_update_input.expected.theta_lr = kExpectedThetaLrBiasRadMidLeg;
    }
  } else {
    chassis_update_input.expected.theta_ll = kExpectedThetaLlBiasRadLowLeg;
    chassis_update_input.expected.theta_lr = kExpectedThetaLrBiasRadLowLeg;
  }
  if (!spin_control_enabled &&
      !(stair_sequence_output.controls_motion && chassis_output.mode == chassis::Fsm::State::kStairTask)) {
    chassis_update_input.expected.theta_b = kExpectedThetaBBiasRad;
  }

  // ── 7l. 偏航角速度控制 ──
  const bool yaw_follow_enabled = yaw_follow_control_enabled && !spin_control_enabled;
  if (spin_control_enabled) {
    ctx.spin_exit_recovery = false;
    ctx.flip_180_in_progress = false;
    const bool ref_online = globals->referee.has_value() && globals->referee->online_status() == rm::device::Device::kOk;
    const uint16_t power_limit = ref_online ? globals->referee->data().robot_status.chassis_power_limit : 0U;
    const float spin_target = ResolveSpinTargetYawDot(power_limit, sc_err);
    RampYawDotToTarget(input.mode_request.spin_dir * spin_target, ctx.filtered_yaw_dot, kSpinYawRampStepRadS);
    chassis_update_input.expected.phi_dot = ctx.filtered_yaw_dot;
  } else if (ctx.flip_180_in_progress) {
    // R 键云台 180° 旋转中：抑制偏航跟随，等待云台旋转完成
    ctx.filtered_yaw_dot = 0.0f;
    ctx.yaw_follow_pid.Clear();
    chassis_update_input.expected.phi_dot = 0.0f;
    ++ctx.flip_180_ticks;
    // 最短等待 100 tick (200ms)，之后检测偏航电机速度归零
    constexpr uint32_t kFlip180MinTicks = 100U;
    if (ctx.flip_180_ticks >= kFlip180MinTicks) {
      const bool yaw_motor_ready = globals->yaw_motor.has_value();
      const float yaw_motor_vel_rad_s = yaw_motor_ready ? globals->yaw_motor->vel() : 0.0f;
      if (yaw_motor_ready && std::fabs(yaw_motor_vel_rad_s) < ns::control_loop::kGimbalStartupYawAlignVelRadS) {
        ctx.flip_180_in_progress = false;
        // 云台旋转完成，基于当前偏航电机位置重新选择跟随目标
        ctx.yaw_follow_target = SelectNearestYawTarget(input.estimator_input.yaw_motor_rad,
                                                       YawFollowTargetOffset(ctx.yaw_follow_align_mode));
        ctx.yaw_follow_target_initialized = true;
      }
    }
  } else if (!yaw_follow_enabled) {
    ctx.spin_exit_recovery = false;
    ctx.flip_180_in_progress = false;
    ctx.filtered_yaw_dot = 0.0f;
    ctx.yaw_follow_pid.Clear();
  } else {
    const float yaw_motor_rad = input.estimator_input.yaw_motor_rad;
    const float yaw_target_rad = ctx.yaw_follow_target.target_rad;
    // 当 error ≈ ±π 时，Wrap 总是返回 -π，PID 会走长路径。
    // 通过 ±2π unwrap 强制走短路径。
    const float raw_err = yaw_target_rad - yaw_motor_rad;
    float adj_target = yaw_target_rad;
    if (const float wrapped_err = rm::modules::Wrap(raw_err, -kPi, kPi);
        std::fabs(std::fabs(wrapped_err) - kPi) < 0.5f) {
      if (raw_err > 0.0f) {
        adj_target += 2.0f * kPi;
      }
    }
    ctx.yaw_follow_pid.UpdateExtDiff(adj_target, yaw_motor_rad, ctx.filtered_yaw_dot, kControlLoopDtS);
    const float target_yaw_dot = -ctx.yaw_follow_pid.out();
    const float yaw_follow_step = has_supercap ? kYawFollowRampStepRadS : kYawFollowRampStepRadNoScS;
    const float ramp_step = ctx.spin_exit_recovery ? kSpinExitYawRampStepRadS : yaw_follow_step;
    RampYawDotToTarget(target_yaw_dot, ctx.filtered_yaw_dot, ramp_step);
    chassis_update_input.expected.phi_dot = ctx.filtered_yaw_dot;
    // 退出小陀螺快速恢复：车体角速度降到 1 rad/s 以下切回正常斜坡
    if (ctx.spin_exit_recovery && std::fabs(ctx.filtered_yaw_dot) < 1.0f) {
      ctx.spin_exit_recovery = false;
    }
  }

  // ── 7m. 底盘控制器执行 ──
  globals->chassis.Update(chassis_update_input);
  chassis_control_output = globals->chassis.GetOutput();

  // ── 中腿长下压退出：下降沿清除中腿长保持，走斜坡到低腿长 ──
  {
    static bool prev_dip_active = false;
    if (prev_dip_active && !chassis_control_output.mid_leg_dip_active) {
      tc_state.mid_leg_hold = false;
      tc_state.auto_small_jump_enabled = false;
    }
    prev_dip_active = chassis_control_output.mid_leg_dip_active;
  }

  // ── LQR 状态误差调试 ──
  wl_debug.lqr_err_s = chassis_control_output.current_state.s - chassis_update_input.expected.s;
  wl_debug.lqr_err_s_dot = chassis_control_output.current_state.s_dot - chassis_update_input.expected.s_dot;
  wl_debug.lqr_err_phi =
      rm::modules::Wrap(chassis_control_output.current_state.phi - chassis_update_input.expected.phi, -kPi, kPi);
  wl_debug.lqr_err_phi_dot = chassis_control_output.current_state.phi_dot - chassis_update_input.expected.phi_dot;
  wl_debug.lqr_err_theta_ll = chassis_control_output.current_state.theta_ll - chassis_update_input.expected.theta_ll;
  wl_debug.lqr_err_theta_ll_dot =
      chassis_control_output.current_state.theta_ll_dot - chassis_update_input.expected.theta_ll_dot;
  wl_debug.lqr_err_theta_lr = chassis_control_output.current_state.theta_lr - chassis_update_input.expected.theta_lr;
  wl_debug.lqr_err_theta_lr_dot =
      chassis_control_output.current_state.theta_lr_dot - chassis_update_input.expected.theta_lr_dot;
  wl_debug.lqr_err_theta_b = chassis_control_output.current_state.theta_b - chassis_update_input.expected.theta_b;
  wl_debug.lqr_err_theta_b_dot =
      chassis_control_output.current_state.theta_b_dot - chassis_update_input.expected.theta_b_dot;
  wl_debug.expected_theta_ll_rad = chassis_update_input.expected.theta_ll;
  wl_debug.expected_theta_lr_rad = chassis_update_input.expected.theta_lr;

  g_actuators.ApplyChassisOutput(*globals, chassis_control_output, chassis_output_enable);
  wl_debug.dm_enabled_latched = g_actuators.dm_enabled_latched() ? 1U : 0U;

  // 底盘电机心跳检测：全部 offline→online 边沿触发重使能
  {
    static bool prev_all_ok = false;
    const bool lf_ok = globals->dm_lf.has_value() && globals->dm_lf->online_status() == rm::device::Device::kOk;
    const bool lb_ok = globals->dm_lb.has_value() && globals->dm_lb->online_status() == rm::device::Device::kOk;
    const bool rf_ok = globals->dm_rf.has_value() && globals->dm_rf->online_status() == rm::device::Device::kOk;
    const bool rb_ok = globals->dm_rb.has_value() && globals->dm_rb->online_status() == rm::device::Device::kOk;
    const bool cur_all_ok = lf_ok && lb_ok && rf_ok && rb_ok;

    wl_debug.dm_lf_online = lf_ok ? 1U : 0U;
    wl_debug.dm_lb_online = lb_ok ? 1U : 0U;
    wl_debug.dm_rf_online = rf_ok ? 1U : 0U;
    wl_debug.dm_rb_online = rb_ok ? 1U : 0U;

    if (cur_all_ok && !prev_all_ok && chassis_output_enable) {
      g_actuators.ResetDmMotorsLatch();
      wl_debug.motor_reenable_chassis_trig = 1U;
    }
    prev_all_ok = cur_all_ok;
  }

  // ═══════════════════════════════════════════════════════════════════════
  // 阶段 8：自瞄通信 — 云台 IMU 欧拉角→CAN 转发
  // ═══════════════════════════════════════════════════════════════════════
  if (globals->aimbot.has_value() && globals->gimbal_rx.has_value() && globals->gimbal_rx->frame_count() > 0) {
    constexpr float kRadToDeg = 180.f / kPi;
    const float yaw_deg = globals->gimbal_rx->euler_yaw_rad() * kRadToDeg;
    const float pitch_deg = globals->gimbal_rx->euler_pitch_rad() * kRadToDeg;
    // todo: hero和infantry不同？？
    const float roll_deg = -globals->gimbal_rx->euler_roll_rad() * kRadToDeg;
    // const float roll_deg = globals->gimbal_rx->euler_roll_rad() * kRadToDeg;

    uint8_t aimbot_mode = 1;
    switch (chassis_input.request.combat_profile) {
      case wheel_legged::CombatProfile::kAutoAimAmmo:
        aimbot_mode = 1;
        break;
      case wheel_legged::CombatProfile::kAutoAimFuSmall:
        aimbot_mode = 2;
        break;
      case wheel_legged::CombatProfile::kAutoAimFuBig:
        aimbot_mode = 3;
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
        (referee_online && referee_bullet_speed >= ns::aimbot::kBulletBoundarySpeedMps)
            ? referee_bullet_speed
            : ((referee_online && referee_bullet_speed > 0.0f) ? ns::aimbot::kBulletDefaultSpeedMps
                                                               : ns::aimbot::kBulletSpeedMps);
    const uint16_t imu_count = static_cast<uint16_t>(globals->gimbal_rx->frame_count() & 0xFU);
    globals->aimbot->UpdateControl(yaw_deg, pitch_deg, roll_deg, robot_id, aimbot_mode, imu_count, bullet_speed);

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
  } else {
    wl_debug.gimbal_euler_yaw_rad = 0.0f;
    wl_debug.gimbal_euler_pitch_rad = 0.0f;
  }
  {
    // DYP-A22 受控输出：每 50ms (25 tick @ 500Hz) 触发一次测量
    static uint32_t dyp_trigger_tick = 0U;
    if (++dyp_trigger_tick >= 100U) {
      dyp_trigger_tick = 0U;
      if (globals->dyp_left.has_value()) globals->dyp_left->TriggerOnce();
      if (globals->dyp_right.has_value()) globals->dyp_right->TriggerOnce();
    }
    const auto left = globals->dyp_left.has_value() ? globals->dyp_left->distance_raw() : 0U;
    const auto right = globals->dyp_right.has_value() ? globals->dyp_right->distance_raw() : 0U;
    wl_debug.dyp_distance_raw_left = left;
    wl_debug.dyp_distance_raw_right = right;
    wl_debug.dyp_distance_filtered_left = left;
    wl_debug.dyp_distance_filtered_right = right;
    wl_debug.dyp_distance_filtered_avg = static_cast<uint16_t>((static_cast<uint32_t>(left) + right) / 2U);
    wl_debug.dyp_result_left = globals->dyp_left.has_value() && globals->dyp_left->data_valid() ? 1U : 0U;
    wl_debug.dyp_result_right = globals->dyp_right.has_value() && globals->dyp_right->data_valid() ? 1U : 0U;
    wl_debug.dyp_frame_count = (globals->dyp_left.has_value() ? globals->dyp_left->frame_count() : 0U) +
                               (globals->dyp_right.has_value() ? globals->dyp_right->frame_count() : 0U);
  }
  wl_debug.yaw_motor_status = globals->yaw_motor.has_value() ? globals->yaw_motor->status() : 0;
  wl_debug.pitch_motor_status = globals->pitch_motor.has_value() ? globals->pitch_motor->status() : 0;

  // ── 裁判系统调试 ──
  if (globals->referee.has_value() && globals->referee->online_status() == rm::device::Device::kOk) {
    wl_debug.referee_online = 1U;
    wl_debug.referee_robot_id = globals->referee->data().robot_status.robot_id;
    wl_debug.referee_bullet_speed_mps = globals->referee->data().shoot_data.initial_speed;
    wl_debug.referee_barrel_heat = globals->referee->data().power_heat_data.shooter_17mm_1_barrel_heat;
    wl_debug.referee_power_chassis =
        static_cast<uint8_t>(globals->referee->data().robot_status.power_management_chassis_output);
    wl_debug.referee_power_gimbal =
        static_cast<uint8_t>(globals->referee->data().robot_status.power_management_gimbal_output);
  } else {
    wl_debug.referee_online = 0U;
    wl_debug.referee_robot_id = 0U;
    wl_debug.referee_bullet_speed_mps = 0.0f;
    wl_debug.referee_barrel_heat = 0U;
    wl_debug.referee_power_chassis = 0U;
    wl_debug.referee_power_gimbal = 0U;
  }
  // ── 超级电容调试 ──
  if (globals->supercap.has_value()) {
    wl_debug.supercap_enable_dcdc = 1U;
    wl_debug.supercap_error_code = globals->supercap->rx_data_.error_code;
    wl_debug.supercap_chassis_power = globals->supercap->rx_data_.chassis_power;
    wl_debug.supercap_chassis_power_limit = globals->supercap->rx_data_.chassis_power_limit;
    wl_debug.supercap_cap_energy = globals->supercap->rx_data_.cap_energy;
  } else {
    wl_debug.supercap_enable_dcdc = 0U;
    wl_debug.supercap_error_code = 0U;
    wl_debug.supercap_chassis_power = 0.0f;
    wl_debug.supercap_chassis_power_limit = 0U;
    wl_debug.supercap_cap_energy = 0U;
  }

  // ── 自瞄 RX 调试（NUC 反馈，原始为度，转为弧度存储）──
  if (globals->aimbot.has_value()) {
    constexpr float kDegToRad = kPi / 180.0f;
    wl_debug.aimbot_rx_state = globals->aimbot->aimbot_state();
    wl_debug.aimbot_rx_target = globals->aimbot->aimbot_id();
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
  yaw_motor_pos = globals->yaw_motor->pos();
  yaw_motor_vel = globals->yaw_motor->vel();
  yaw_motor_tau = globals->yaw_motor->tau();
  pitch_motor_pos = globals->pitch_motor->pos();
  pitch_motor_vel = globals->pitch_motor->vel();
  pitch_motor_tau = globals->pitch_motor->tau();

  {
    const bool referee_online =
        globals->referee.has_value() && globals->referee->online_status() == rm::device::Device::kOk;
    ui_snapshot.referee_online = referee_online;
    ui_snapshot.referee_robot_id = globals->referee.has_value() ? globals->referee->data().robot_status.robot_id : 0U;

    if (globals->gimbal_rx.has_value()) {
      ui_snapshot.gimbal_pitch_rad = globals->gimbal_rx->pitch_rad();
      ui_snapshot.gimbal_yaw_rad = globals->gimbal_rx->yaw_rad();
    } else {
      ui_snapshot.gimbal_pitch_rad = 0.0f;
      ui_snapshot.gimbal_yaw_rad = 0.0f;
    }
    ui_snapshot.yaw_motor_raw_pos_rad = input.estimator_input.yaw_motor_rad;

    const auto &ui_chassis_state = chassis_control_output.current_state;
    ui_snapshot.left_leg_length_m = ui_chassis_state.l_l;
    ui_snapshot.right_leg_length_m = ui_chassis_state.l_r;
    ui_snapshot.left_leg_theta_rad = ui_chassis_state.theta_ll;
    ui_snapshot.right_leg_theta_rad = ui_chassis_state.theta_lr;
    ui_snapshot.leg_view_flip =
        (ctx.yaw_follow_align_mode == YawFollowAlignMode::kForward && ctx.yaw_follow_target.drive_sign < 0.0f) ||
        (ctx.yaw_follow_align_mode == YawFollowAlignMode::kSidePositive);

    ui_snapshot.chassis_fsm_state = static_cast<uint8_t>(chassis_output.mode);
    ui_snapshot.domain_request = static_cast<uint8_t>(input.mode_request.domain_request);
    ui_snapshot.combat_profile = static_cast<uint8_t>(input.mode_request.combat_profile);
    ui_snapshot.aim_mode = static_cast<uint8_t>(tc_state.aim_mode);
    ui_snapshot.auto_aim_hold = tc_state.auto_aim_hold;
    ui_snapshot.standby = chassis_output.mode == chassis::Fsm::State::kStandby;
    ui_snapshot.spin_active = chassis_output.mode == chassis::Fsm::State::kSpin ||
                              chassis_output.mode == chassis::Fsm::State::kSpinExitPending;
    ui_snapshot.cross_active = input.mode_request.mid_leg_f;

    ui_snapshot.supercap_cap_energy =
        globals->supercap.has_value() ? static_cast<float>(globals->supercap->rx_data_.cap_energy) : 0.0f;
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
    ui_snapshot.fw_raw_rpm_1 = globals->fw_motor_1.has_value() ? static_cast<float>(globals->fw_motor_1->rpm()) : 0.0f;
    ui_snapshot.fw_raw_rpm_2 = globals->fw_motor_2.has_value() ? static_cast<float>(globals->fw_motor_2->rpm()) : 0.0f;
    ui_snapshot.fw_raw_rpm_3 = globals->fw_motor_3.has_value() ? static_cast<float>(globals->fw_motor_3->rpm()) : 0.0f;
    ui_snapshot.fric_left_rpm = 0.0f;
    ui_snapshot.fric_right_rpm = 0.0f;
#else
    ui_snapshot.fric_left_rpm = globals->fric_left.has_value() ? static_cast<float>(globals->fric_left->rpm()) : 0.0f;
    ui_snapshot.fric_right_rpm =
        globals->fric_right.has_value() ? static_cast<float>(globals->fric_right->rpm()) : 0.0f;
    ui_snapshot.fw_raw_rpm_1 = 0.0f;
    ui_snapshot.fw_raw_rpm_2 = 0.0f;
    ui_snapshot.fw_raw_rpm_3 = 0.0f;
#endif
    if (referee_online) {
      ui_snapshot.bullet_speed_mps = globals->referee->data().shoot_data.initial_speed;
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
      ui_snapshot.projectile_allowance = globals->referee->data().projectile_allowance.projectile_allowance_42mm;
#else
      ui_snapshot.projectile_allowance = globals->referee->data().projectile_allowance.projectile_allowance_17mm;
#endif
    } else {
      ui_snapshot.bullet_speed_mps = 0.0f;
      ui_snapshot.projectile_allowance = 0U;
    }

    if (globals->gimbal_rx.has_value()) {
      const auto &hp = globals->gimbal_rx->robot_hp();
      ui_snapshot.hero_1_HP = hp.hero_1_HP;
      ui_snapshot.engineer_2_HP = hp.engineer_2_HP;
      ui_snapshot.standard_3_HP = hp.standard_3_HP;
      ui_snapshot.standard_4_HP = hp.standard_4_HP;
      ui_snapshot.sentry_7_HP = hp.sentry_7_HP;
    } else {
      ui_snapshot.hero_1_HP = 0;
      ui_snapshot.engineer_2_HP = 0;
      ui_snapshot.standard_3_HP = 0;
      ui_snapshot.standard_4_HP = 0;
      ui_snapshot.sentry_7_HP = 0;
    }

    if (globals->subReferee.has_value()) {
      const auto &gold = globals->subReferee->data().enemy_gold_coin_RFID;
      ui_snapshot.enemy_gold_remaining = gold.enemy_gold_remaining;
      ui_snapshot.enemy_gold_total = gold.enemy_gold_total;

      const auto &allow = globals->subReferee->data().enemy_robot_projectile_allowance;
      ui_snapshot.enemy_hero_1_allowance = allow.hero_1_projectile_allowance;
      ui_snapshot.enemy_standard_3_allowance = allow.standard_3_projectile_allowance;
      ui_snapshot.enemy_standard_4_allowance = allow.standard_4_projectile_allowance;
      ui_snapshot.enemy_drone_6_allowance = allow.drone_6_projectile_allowance;
      ui_snapshot.enemy_sentry_7_allowance = allow.sentry_7_projectile_allowance;
    } else {
      ui_snapshot.enemy_gold_remaining = 0;
      ui_snapshot.enemy_gold_total = 0;
      ui_snapshot.enemy_hero_1_allowance = 0;
      ui_snapshot.enemy_standard_3_allowance = 0;
      ui_snapshot.enemy_standard_4_allowance = 0;
      ui_snapshot.enemy_drone_6_allowance = 0;
      ui_snapshot.enemy_sentry_7_allowance = 0;
    }

    if (globals->aimbot.has_value()) {
      ui_snapshot.aimbot_id = globals->aimbot->aimbot_id();
    } else {
      ui_snapshot.aimbot_id = 0;
    }

    {
      uint16_t target_hp = 0;
      uint16_t target_allowance = 0;
      if (globals->subReferee.has_value()) {
        const auto &hp = globals->subReferee->data().enemy_robot_HP;
        const auto &allow = globals->subReferee->data().enemy_robot_projectile_allowance;
        switch (ui_snapshot.aimbot_id) {
          case 1:
            target_hp = hp.hero_1_HP;
            target_allowance = allow.hero_1_projectile_allowance;
            break;
          case 2:
            target_hp = hp.engineer_2_HP;
            target_allowance = 0;
            break;
          case 3:
            target_hp = hp.standard_3_HP;
            target_allowance = allow.standard_3_projectile_allowance;
            break;
          case 4:
            target_hp = hp.standard_4_HP;
            target_allowance = allow.standard_4_projectile_allowance;
            break;
          case 5:
            target_hp = hp.sentry_7_HP;
            target_allowance = allow.sentry_7_projectile_allowance;
            break;
          default:
            break;
        }
      }
      ui_snapshot.aimbot_target_hp = target_hp;
      ui_snapshot.aimbot_target_allowance = target_allowance;
    }
  }

  UpdateDebugSnapshot(now_ms, input, chassis_output, gimbal_output, chassis_control_output, gimbal_control_output,
                      stair_task_output, stair_sequence_output);

  if (!init_flag) {
    ui_init();
    init_flag = true;
  }
  times++;
  if (times % 17 == 0) {
    schedule.schedule();
  }
}
