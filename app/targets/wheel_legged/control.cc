#include "include/globals.hpp"
#include "include/actuators.hpp"
#include "include/ai/policy_runner.hpp"
#include "include/chassis/stair_climb_sequence.hpp"
#include "include/chassis/stair_task_coordinator.hpp"
#include "include/debug.hpp"
#include "tools/theta_bias_generated.hpp"
#include "main.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include "ui/UIWheelLegged.hpp"
#include "ui/UIEnemyInfo.hpp"
#include "ui/TaskScheduler.hpp"
#include "ui/ui_snapshot.hpp"
#include "include/input.hpp"
#include "include/state_ctx.hpp"
bool init_flag;
uint32_t times = 0;

namespace {
float LookupThetaBiasSingle(float leg_length, const float table[][2]) {
  constexpr int N = 50;
  const float t_min = table[0][0];
  const float t_max = table[N - 1][0];
  if (leg_length <= t_min) return table[0][1];
  if (leg_length >= t_max) return table[N - 1][1];
  const float step = (t_max - t_min) / static_cast<float>(N - 1);
  const float idx_f = (leg_length - t_min) / step;
  const int idx = static_cast<int>(idx_f);
  const float alpha = idx_f - static_cast<float>(idx);
  return table[idx][1] * (1.0f - alpha) + table[idx + 1][1] * alpha;
}
}  // namespace

// ── UI task scheduler & task objects (drone_gb_new style) ──
static auto schedule = rm::device::UITaskScheduler(30);

// Static labels (one-shot add)
static auto UI_label_leg = rm::device::UITask(UIWheelLeggedLabelLeg_add);
static auto UI_label_ad = rm::device::UITask(UIWheelLeggedLabelAD_add);
static auto UI_status_label_st1 = rm::device::UITask(UIWheelLeggedStatusLabel_add_st1);
static auto UI_status_label_st2 = rm::device::UITask(UIWheelLeggedStatusLabel_add_st2);
static auto UI_status_label_st3 = rm::device::UITask(UIWheelLeggedStatusLabel_add_st3);

// Crosshair
static auto UI_crosshair_add = rm::device::UITask(UIWheelLeggedCrosshair_add);
static auto UI_crosshair_edit = rm::device::UITask(UIWheelLeggedCrosshair_edit, 1.5f);

// Gimbal data (hero variant)
static auto UI_label_py = rm::device::UITask(UIWheelLeggedLabelPY_add);
static auto UI_gimbal_data_add = rm::device::UITask(UIWheelLeggedGimbalData_add);
static auto UI_gimbal_data_edit = rm::device::UITask(UIWheelLeggedGimbalData_edit, 1.5f);

// Supercap energy bar
static auto UI_supercap_box = rm::device::UITask(UIWheelLeggedSupercapBox_add);
static auto UI_supercap_add = rm::device::UITask(UIWheelLeggedSupercap_add);
static auto UI_supercap_edit = rm::device::UITask(UIWheelLeggedSupercap_edit, 1.0f);

// Leg length indicator (L M H box)
static auto UI_leg_box_add = rm::device::UITask(UIWheelLeggedLegBox_add);
static auto UI_leg_box_edit = rm::device::UITask(UIWheelLeggedLegBox_edit, 3.0f);

// Leg pose lines + yaw arc
static auto UI_leg_pose_add = rm::device::UITask(UIWheelLeggedLegPose_add);
static auto UI_leg_pose_edit = rm::device::UITask(UIWheelLeggedLegPose_edit, 2.0f);

// Friction RPM
static auto UI_fric_rpm_add = rm::device::UITask(UIWheelLeggedFricRPM_add);
static auto UI_fric_rpm_edit = rm::device::UITask(UIWheelLeggedFricRPM_edit, 0.5f);

// Bullet data
static auto UI_bullet_add = rm::device::UITask(UIWheelLeggedBulletData_add);
static auto UI_bullet_edit = rm::device::UITask(UIWheelLeggedBulletData_edit, 3.0f);
// State indicator (infantry variant)
static auto UI_state_indicator_add = rm::device::UITask(UIWheelLeggedStateIndicator_add);
static auto UI_state_indicator_edit = rm::device::UITask(UIWheelLeggedStateIndicator_edit, 2.f);

// Aimbot box
static auto UI_aimbot_box_add = rm::device::UITask(UIWheelLeggedAimbotBox_add);
static auto UI_aimbot_box_edit = rm::device::UITask(UIWheelLeggedAimbotBox_edit, 5.f);

// Shooter disabled warning X
static auto UI_shooter_x_add = rm::device::UITask(UIWheelLeggedShooterX_add);
static auto UI_shooter_x_edit = rm::device::UITask(UIWheelLeggedShooterX_edit, 3.f);

// Enemy info — red team
static auto UI_enemy_header_red = rm::device::UITask(UIEnemyHeaderRed_add);
static auto UI_enemy_hp_red_add = rm::device::UITask(UIEnemyHPRed_add);
static auto UI_enemy_hp_red_edit = rm::device::UITask(UIEnemyHPRed_edit, 3.f);
static auto UI_enemy_allowance_red_add = rm::device::UITask(UIEnemyAllowanceRed_add);
static auto UI_enemy_allowance_red_edit = rm::device::UITask(UIEnemyAllowanceRed_edit, 3.f);

// Enemy info — blue team
static auto UI_enemy_header_blue = rm::device::UITask(UIEnemyHeaderBlue_add);
static auto UI_enemy_hp_blue_add = rm::device::UITask(UIEnemyHPBlue_add);
static auto UI_enemy_hp_blue_edit = rm::device::UITask(UIEnemyHPBlue_edit, 3.f);
static auto UI_enemy_allowance_blue_add = rm::device::UITask(UIEnemyAllowanceBlue_add);
static auto UI_enemy_allowance_blue_edit = rm::device::UITask(UIEnemyAllowanceBlue_edit, 3.f);

// Gold coin
static auto UI_gold_coin_add = rm::device::UITask(UIGoldCoin_add);
static auto UI_gold_coin_edit = rm::device::UITask(UIGoldCoin_edit, 0.5f);

void static_UI_add() {
  schedule.addTaskStatic(&UI_label_leg);
  schedule.addTaskStatic(&UI_label_ad);
  schedule.addTaskStatic(&UI_crosshair_add);
  schedule.addTask(&UI_crosshair_edit);

#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  schedule.addTaskStatic(&UI_label_py);
  schedule.addTaskStatic(&UI_gimbal_data_add);
  schedule.addTask(&UI_gimbal_data_edit);
#else
  schedule.addTaskStatic(&UI_status_label_st1);
  schedule.addTaskStatic(&UI_status_label_st2);
  schedule.addTaskStatic(&UI_status_label_st3);
  schedule.addTaskStatic(&UI_state_indicator_add);
  schedule.addTask(&UI_state_indicator_edit);
#endif

  schedule.addTaskStatic(&UI_supercap_box);
  schedule.addTaskStatic(&UI_supercap_add);
  schedule.addTask(&UI_supercap_edit);
  schedule.addTaskStatic(&UI_leg_box_add);
  schedule.addTask(&UI_leg_box_edit);
  schedule.addTaskStatic(&UI_leg_pose_add);
  schedule.addTask(&UI_leg_pose_edit);

  schedule.addTaskStatic(&UI_fric_rpm_add);
  schedule.addTask(&UI_fric_rpm_edit);

  schedule.addTaskStatic(&UI_bullet_add);
  schedule.addTask(&UI_bullet_edit);

  schedule.addTaskStatic(&UI_aimbot_box_add);
  schedule.addTask(&UI_aimbot_box_edit);

  schedule.addTaskStatic(&UI_shooter_x_add);
  schedule.addTask(&UI_shooter_x_edit);

  schedule.addTaskStatic(&UI_enemy_header_red);
  schedule.addTaskStatic(&UI_enemy_hp_red_add);
  schedule.addTask(&UI_enemy_hp_red_edit);
  schedule.addTaskStatic(&UI_enemy_allowance_red_add);
  schedule.addTask(&UI_enemy_allowance_red_edit);

  schedule.addTaskStatic(&UI_enemy_header_blue);
  schedule.addTaskStatic(&UI_enemy_hp_blue_add);
  schedule.addTask(&UI_enemy_hp_blue_edit);
  schedule.addTaskStatic(&UI_enemy_allowance_blue_add);
  schedule.addTask(&UI_enemy_allowance_blue_edit);

  schedule.addTaskStatic(&UI_gold_coin_add);
  schedule.addTask(&UI_gold_coin_edit);
}
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
constexpr float kLargeTurnThresholdRad = ns::control_loop::kLargeTurnThresholdRad;
constexpr float kSafeTurnSpeedMps = ns::control_loop::kSafeTurnSpeedMps;
constexpr float kLargeTurnThetaThresholdRad = ns::control_loop::kLargeTurnThetaThresholdRad;
constexpr float kLargeTurnRecoveryAccelScale = ns::control_loop::kLargeTurnRecoveryAccelScale;
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
constexpr float kSpinThetaLrBiasRad = ns::control_loop::kSpinThetaLrBiasRad;
constexpr float kSpinLegLengthBiasM = ns::control_loop::kSpinLegLengthBiasM;
constexpr float kSpinThetaBBiasRad = ns::control_loop::kSpinThetaBBiasRad;
constexpr float kJumpThetaLlBiasRad = ns::control_loop::kJumpThetaLlBiasRad;
constexpr float kJumpThetaLrBiasRad = ns::control_loop::kJumpThetaLrBiasRad;
constexpr float kExpectedThetaBBiasRad = ns::control_loop::kExpectedThetaBBiasRad;
constexpr uint32_t kGimbalStartupYawAlignStableTicks = ns::control_loop::kGimbalStartupYawAlignStableTicks;
constexpr uint32_t kYawFollowDriveReadyStableTicks = ns::control_loop::kYawFollowDriveReadyStableTicks;
constexpr float kYawFollowFixedTargetRad = ns::control_loop::kYawFollowFixedTargetRad;
constexpr float kYawTargetRampStepRad = ns::control_loop::kYawTargetRampStepRad;

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

float ClampPolicyAction(float value, std::uint32_t index) {
  const float limit = (index == 2U || index == 5U) ? 1.3262599469496021f : 3.0f;
  return std::clamp(value, -limit, limit);
}

wheel_legged::ai::PolicyInput BuildRealPolicyInput(const wheel_legged::control_loop::InputSnapshot &input,
                                                   const chassis::Chassis::UpdateOutput &chassis_output,
                                                   float vx_cmd_mps, float yaw_rate_cmd_rad_s) {
  wheel_legged::ai::PolicyInput policy_input{};
  static bool history_initialized = false;
  static float history[135]{};
  static float wheel_pos_left_rad = 0.0f;
  static float wheel_pos_right_rad = 0.0f;

  const auto &imu = input.estimator_input.imu;
  const auto &x = chassis_output.current_state;
  const auto &last_output = wheel_legged::ai::GetPolicyTestOutput();

  constexpr float kPolicyDtS = 0.02f;
  constexpr float kDefaultHeightCmdM = 0.23f;

  wheel_pos_left_rad += input.estimator_input.wheel.left_rad_s * kPolicyDtS;
  wheel_pos_right_rad += input.estimator_input.wheel.right_rad_s * kPolicyDtS;

  const float sin_roll = std::sin(imu.roll_rad);
  const float cos_roll = std::cos(imu.roll_rad);
  const float sin_pitch = std::sin(imu.pitch_rad);
  const float cos_pitch = std::cos(imu.pitch_rad);

  // 注意：MCU 机体坐标系与网络训练坐标系绕 Z 差 180°
  //   MCU +X(前) = 网络 -X, MCU +Y(左) = 网络 -Y, MCU +Z = 网络 +Z
  //   因此 gyro_x/y 取反, projected_gravity x/y 取反, vx 取反, 左右腿/轮交换
  float obs[27]{};
  obs[0] = -imu.gyro_x_rad_s * 0.25f;
  obs[1] = -imu.gyro_y_rad_s * 0.25f;
  obs[2] = imu.gyro_z_rad_s * 0.25f;
  obs[3] = -sin_pitch;
  obs[4] = sin_roll * cos_pitch;
  obs[5] = -cos_roll * cos_pitch;
  // obs[6] = -vx_cmd_mps * 2.0f;
  // obs[7] = yaw_rate_cmd_rad_s * 0.25f;
  // obs[8] = kDefaultHeightCmdM * 5.0f;
  obs[6] = -vx_cmd_mps * 0.0f;
  obs[7] = yaw_rate_cmd_rad_s * 0.f;
  obs[8] = kDefaultHeightCmdM * 0.0f;
  // 左右腿交换: MCU左→网络右, MCU右→网络左，且摆角/摆角速度取反
  obs[9] = -x.theta_lr;
  obs[10] = -x.theta_ll;
  obs[11] = -x.theta_lr_dot * 0.05f;
  obs[12] = -x.theta_ll_dot * 0.05f;
  obs[13] = x.l_r * 5.0f;
  obs[14] = x.l_l * 5.0f;
  obs[15] = chassis_output.right_l0_dot_mps * 0.25f;
  obs[16] = chassis_output.left_l0_dot_mps * 0.25f;
  // 左右轮交换
  obs[17] = wheel_pos_right_rad;
  obs[18] = wheel_pos_left_rad;
  obs[19] = -input.estimator_input.wheel.right_rad_s * 0.05f;
  obs[20] = -input.estimator_input.wheel.left_rad_s * 0.05f;
  // last_action 是网络自身的上一帧输出，不需要交换
  for (std::uint32_t i = 0; i < 6U; ++i) {
    obs[21U + i] = ClampPolicyAction(last_output.actions[i], i);
  }

  std::memcpy(policy_input.observations, obs, sizeof(obs));

  if (!history_initialized) {
    for (std::uint32_t frame = 0; frame < 5U; ++frame) {
      std::memcpy(&history[frame * 27U], obs, sizeof(obs));
    }
    history_initialized = true;
  } else {
    std::memmove(history, &history[27], sizeof(float) * 27U * 4U);
    std::memcpy(&history[27U * 4U], obs, sizeof(obs));
  }
  std::memcpy(policy_input.observation_history, history, sizeof(history));

  return policy_input;
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

  // DWT CYCCNT runs at the Cortex-M7 core clock.  Measure the real interval
  // between control-loop invocations instead of assuming that the timer ISR
  // always executes at exactly 500 Hz.  The unsigned subtraction is safe
  // across the 32-bit CYCCNT wrap as long as adjacent calls are less than one
  // wrap period apart.
  static uint32_t last_control_cycles = 0U;
  const uint32_t now_control_cycles = DWT->CYCCNT;
  const uint32_t control_exec_start_cycles = now_control_cycles;
  float measured_control_dt_s = kControlLoopDtS;
  bool measured_control_dt_valid = false;
  if (last_control_cycles != 0U && SystemCoreClock != 0U) {
    const uint32_t delta_cycles = now_control_cycles - last_control_cycles;
    measured_control_dt_s = static_cast<float>(delta_cycles) / static_cast<float>(SystemCoreClock);
    measured_control_dt_valid = measured_control_dt_s >= 0.0005f && measured_control_dt_s <= 0.020f;
    if (measured_control_dt_valid) {
      constexpr uint32_t kExpectedControlDtUs = 2000U;
      static uint32_t control_dt_min_us = 0xFFFFFFFFU;
      static uint32_t control_dt_max_us = 0U;
      static uint32_t control_jitter_max_us = 0U;

      const uint32_t control_dt_us =
          static_cast<uint32_t>((static_cast<uint64_t>(delta_cycles) * 1000000ULL) / SystemCoreClock);
      const uint32_t jitter_us = control_dt_us >= kExpectedControlDtUs ? control_dt_us - kExpectedControlDtUs
                                                                       : kExpectedControlDtUs - control_dt_us;
      if (control_dt_us < control_dt_min_us) control_dt_min_us = control_dt_us;
      if (control_dt_us > control_dt_max_us) control_dt_max_us = control_dt_us;
      if (jitter_us > control_jitter_max_us) control_jitter_max_us = jitter_us;
      wl_debug.control_dt_us = static_cast<uint16_t>(std::min<uint32_t>(control_dt_us, 0xFFFFU));
      wl_debug.control_dt_min_us = static_cast<uint16_t>(std::min<uint32_t>(control_dt_min_us, 0xFFFFU));
      wl_debug.control_dt_max_us = static_cast<uint16_t>(std::min<uint32_t>(control_dt_max_us, 0xFFFFU));
      wl_debug.control_jitter_max_us = static_cast<uint16_t>(std::min<uint32_t>(control_jitter_max_us, 0xFFFFU));
      if (jitter_us > 100U) ++wl_debug.control_jitter_over_100us_count;
    }
  }
  last_control_cycles = now_control_cycles;

  // ── 跨周期状态（static 保持值语义，避免堆分配）──
  static InputSnapshot input{};
  static Dr16SemanticState dr16_state{};
  static TcSemanticState tc_state{};
  static ChassisStateContext ctx{};
  static uint32_t fall_start_ms = 0;
  static bool was_posture_invalid = false;
  static chassis::Chassis::UpdateOutput chassis_control_output{};
  static gimbal::Gimbal::UpdateOutput gimbal_control_output{};
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  static int hero_remaining_ammo = ns::control_loop::kInitialAmmoCount;
#endif

  // ── 一次性初始化 ──
  (void)0;

  // 云台 C 板通信断开时强制退出中腿长保持
  if (!(globals->gimbal_rx.has_value() && globals->gimbal_rx->frame_count() > 0)) {
    tc_state.mid_leg_hold = false;
  }

  // ═══════════════════════════════════════════════════════════════════════
  // 阶段 1：硬件反馈采集 + DR16 语义折叠
  // ═══════════════════════════════════════════════════════════════════════
  UpdateRawFeedbackAndInputSnapshot(*globals, g_actuators, input, dr16_state, tc_state);
  globals->ui_refresh_key = tc_state.e_ui_refresh;
  input.ui_refresh_key = tc_state.e_ui_refresh;

  // ═══════════════════════════════════════════════════════════════════════
  // 阶段 2：状态机决策
  // ═══════════════════════════════════════════════════════════════════════
  const bool stair_step2 = g_stair_task_coordinator.output().completed_attempts > 0U;
  const auto &stair_params = stair_step2 ? ns::chassis_fsm::kStairClimbStep2 : ns::chassis_fsm::kStairClimb;
  const bool stair_output_enabled = input.mode_request.input_valid &&
                                    input.mode_request.domain_request != wheel_legged::DomainRequest::kDisabled &&
                                    !input.mode_request.standby;
  const bool stair_contact_detected =
      chassis_control_output.current_state.theta_ll > stair_params.contact_theta_threshold_rad &&
      chassis_control_output.current_state.theta_lr > stair_params.contact_theta_threshold_rad;
  const bool stair_high_leg_ready = std::fabs(chassis_control_output.mean_leg_length_m -
                                              stair_params.high_leg_length_m) <= stair_params.leg_length_tolerance_m;
  const auto &previous_sequence_output = g_stair_sequence.output();
  // 延迟的台阶请求（来自 C/V/B 键 yaw 对齐完成后注入）
  const auto deferred_req = tc_state.deferred_stair_request;
  if (deferred_req != wheel_legged::StairTaskRequest::kNone) {
    tc_state.deferred_stair_request = wheel_legged::StairTaskRequest::kNone;
  }
  const auto effective_stair_request = [&]() {
    if (deferred_req != wheel_legged::StairTaskRequest::kNone) return deferred_req;
    if (input.mode_request.spin_hold) return wheel_legged::StairTaskRequest::kCancel;
    return input.mode_request.stair_task_request;
  }();
  const auto &stair_task_output = g_stair_task_coordinator.Update({
      .request = effective_stair_request,
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
      .use_step2_params = stair_task_output.completed_attempts > 0U,
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

  auto chassis_input = BuildChassisFsmInput(input, now_ms, chassis_control_output, fall_start_ms, was_posture_invalid);
  if (stair_task_output.request_high_leg) {
    chassis_input.request.leg_request = wheel_legged::LegProfile::kHigh;
  }
  if (stair_task_output.force_low_leg) {
    chassis_input.request.leg_request = wheel_legged::LegProfile::kLow;
  }
  chassis_input.request.stair_task_active = stair_task_output.task_active;
  chassis_input.request.stair_step2 = stair_task_output.completed_attempts > 0U;
  chassis_input.request.stair_task_recovery_required = stair_task_output.recovery_required;
  {
    const float nearest_forward = SelectNearestYawCenterTarget(input.estimator_input.yaw_motor_rad);
    const float yaw_err = rm::modules::Wrap(nearest_forward - input.estimator_input.yaw_motor_rad, -kPi, kPi);
    chassis_input.request.spin_exit_yaw_aligned = std::fabs(yaw_err) < kSpinExitYawAlignThresholdRad;
  }
  // 裁判系统电源管理：底盘输出为 0 时强制切到 Disabled
  if (globals->referee.has_value() && globals->referee->online_status() == rm::device::Device::kOk &&
      globals->referee->data().robot_status.power_management_chassis_output == 0) {
    chassis_input.request.domain_request = wheel_legged::DomainRequest::kDisabled;
  }
  // 只有 Q 键 / Ctrl+Q 或 DR16 拨杆能从 Disabled 切到其他状态（DR16 优先）
  {
    static uint8_t prev_domain_state = 0U;
    const bool q_just_pressed = (prev_domain_state == 0U && tc_state.domain_state != 0U);
    const bool dr16_drives_domain = input.dr16.online;
    if (globals->chassis_fsm.mode() == chassis::Fsm::State::kDisabled &&
        chassis_input.request.domain_request != wheel_legged::DomainRequest::kDisabled && !q_just_pressed &&
        !dr16_drives_domain) {
      chassis_input.request.domain_request = wheel_legged::DomainRequest::kDisabled;
    }
    prev_domain_state = tc_state.domain_state;
  }
  // C/V/B 键重置正方向：始终先对齐 yaw 再切换腿长
  // 暂缓期间必须阻止 FSM 切换模式，只覆盖 target_leg_length_m 不够——
  // fsm_mode 在 chassis.cc 里还会影响腿 PID、力控、轮力矩等多个路径。
  const auto leg_profile_from_state = [](chassis::Fsm::State s) -> wheel_legged::LegProfile {
    if (s == chassis::Fsm::State::kMidLeg) return wheel_legged::LegProfile::kMid;
    if (s == chassis::Fsm::State::kHighLeg || s == chassis::Fsm::State::kStairTask)
      return wheel_legged::LegProfile::kHigh;
    return wheel_legged::LegProfile::kLow;
  };

  if (ctx.defer_leg_change) {
    ctx.pending_leg_profile = chassis_input.request.leg_request;
    const float yaw_err =
        std::fabs(rm::modules::Wrap(ctx.defer_yaw_target_rad - input.estimator_input.yaw_motor_rad, -kPi, kPi));
    if (yaw_err < kSpinExitYawAlignThresholdRad) {
      chassis_input.request.leg_request = ctx.pending_leg_profile;
      ctx.defer_leg_change = false;
    } else {
      chassis_input.request.leg_request = leg_profile_from_state(globals->chassis_fsm.mode());
    }
  } else if (input.mode_request.reset_yaw_request) {
    ctx.defer_leg_change = true;
    ctx.pending_leg_profile = chassis_input.request.leg_request;
    chassis_input.request.leg_request = leg_profile_from_state(globals->chassis_fsm.mode());
  }

  // yaw 对齐未完成时保持 FSM 当前状态，完成后再执行 C/V/B 的 pending 动作
  if (ctx.defer_leg_change) {
    chassis_input.request.stair_task_active = (globals->chassis_fsm.mode() == chassis::Fsm::State::kStairTask);
  } else if (tc_state.pending_action != TcSemanticState::PendingAction::kNone) {
    // yaw 对齐完成（或本就不需要转），执行之前暂缓的动作
    switch (tc_state.pending_action) {
      case TcSemanticState::PendingAction::kC:
        tc_state.mid_leg_hold = !tc_state.mid_leg_hold;
        tc_state.mid_leg_f = false;
        tc_state.deferred_stair_request = wheel_legged::StairTaskRequest::kCancel;
        break;
      case TcSemanticState::PendingAction::kV:
        tc_state.mid_leg_hold = false;
        tc_state.mid_leg_f = false;
        tc_state.deferred_stair_request = wheel_legged::StairTaskRequest::kArmSingle;
        break;
      case TcSemanticState::PendingAction::kB:
        tc_state.mid_leg_hold = false;
        tc_state.mid_leg_f = false;
        tc_state.deferred_stair_request = wheel_legged::StairTaskRequest::kArmDouble;
        break;
      default:
        break;
    }
    tc_state.pending_action = TcSemanticState::PendingAction::kNone;
  }

  chassis::Fsm::Output chassis_output = globals->chassis_fsm.Update(chassis_input);

  // Z arms one automatic jump only. It is cleared after that jump leaves all jump phases.
  const auto is_jump_state = [](const chassis::Fsm::State state) {
    return state == chassis::Fsm::State::kJumpPrep || state == chassis::Fsm::State::kJumpPush ||
           state == chassis::Fsm::State::kJumpRecover;
  };
  if (input.auto_jump_triggered && is_jump_state(chassis_output.mode)) {
    tc_state.auto_jump_in_progress = true;
  }
  if (tc_state.auto_jump_in_progress && !is_jump_state(chassis_output.mode)) {
    tc_state.auto_jump_enabled = false;
    tc_state.auto_jump_in_progress = false;
    tc_state.auto_jump_tof_armed = true;
    input.auto_jump_enabled = false;
  }

  // ── recovery→正常过渡：清除中腿长保持，落地后保持低腿长 ──
  {
    static chassis::Fsm::State prev_chassis_mode_for_recovery = chassis::Fsm::State::kDisabled;
    const bool is_recovery = (chassis_output.mode == chassis::Fsm::State::kRecoveryFallCheck ||
                              chassis_output.mode == chassis::Fsm::State::kRecoverySelfRight);
    const bool was_recovery = (prev_chassis_mode_for_recovery == chassis::Fsm::State::kRecoveryFallCheck ||
                               prev_chassis_mode_for_recovery == chassis::Fsm::State::kRecoverySelfRight);
    if (was_recovery && !is_recovery) {
      tc_state.mid_leg_hold = false;
    }
    prev_chassis_mode_for_recovery = chassis_output.mode;
  }

  gimbal::Fsm::Input gimbal_input = BuildGimbalFsmInput(input, chassis_output, ctx.gimbal_startup_align_complete);
  gimbal_input.request.yaw_centering_for_recovery = chassis_control_output.pitch_roll_valid_theta_invalid;
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

  // 底盘姿态异常时强制关闭云台输出（恢复归中阶段除外）
  const bool chassis_posture_invalid = !chassis_control_output.posture_valid;
  const bool recovery_yaw_centering_active = gimbal_output.mode == gimbal::Fsm::State::kRecoveryYawCentering;
  if (chassis_posture_invalid && !recovery_yaw_centering_active) {
    gimbal_update_input.gimbal_enable = false;
  }

  gimbal_update_input.align_to_chassis_forward = gimbal_output.control.align_to_chassis_forward;
  gimbal_update_input.target = gimbal_output.control.gimbal_target;
  gimbal_update_input.use_yaw_motor_feedback = gimbal_startup_align_active;
  gimbal_update_input.aimbot_mode = gimbal_output.control.active_target_source == wheel_legged::TargetSource::kHost;
  gimbal_update_input.aimbot_is_rune =
      (chassis_input.request.combat_profile == wheel_legged::CombatProfile::kAutoAimFuSmall ||
       chassis_input.request.combat_profile == wheel_legged::CombatProfile::kAutoAimFuBig);
  const bool chassis_spin_mode = chassis_output.mode == chassis::Fsm::State::kSpin;
  const bool chassis_spin_exit_pending = chassis_output.mode == chassis::Fsm::State::kSpinExitPending;
  gimbal_update_input.spin_hold = chassis_spin_mode || chassis_spin_exit_pending;
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
  if (recovery_yaw_centering_active) {
    // 恢复归中：yaw 到最近车头方向，pitch 到上限
    gimbal_update_input.align_to_chassis_forward = false;
    gimbal_update_input.target.yaw_rad = SelectNearestYawCenterTarget(input.estimator_input.yaw_motor_rad);
    gimbal_update_input.target.pitch_rad = wheel_legged::params::active::gimbal::kPitchMaxRad;
    gimbal_update_input.use_yaw_motor_feedback = true;
  }
  if (ctx.recovery_yaw_centering_was_active && !recovery_yaw_centering_active) {
    // 恢复归中退出：同步所有目标源到当前云台惯导角，消除目标跳变
    dr16_state.rc_target.yaw_rad = input.gimbal_imu_yaw_rad;
    dr16_state.rc_target.pitch_rad = -input.gimbal_imu_pitch_rad;
    // 直接覆盖当周期目标，用上一周期 gimbal 实际输出位置，避免本周期目标跳变
    gimbal_update_input.target.yaw_rad = gimbal_control_output.yaw_pos_rad;
    gimbal_update_input.target.pitch_rad = gimbal_control_output.pitch_pos_rad;
    globals->gimbal.ResetFf();
  }
  ctx.recovery_yaw_centering_was_active = recovery_yaw_centering_active;
  gimbal_update_input.chassis_yaw_rad = input.estimator_input.imu.yaw_rad;
  if (chassis_spin_mode) {
    const bool ref_online =
        globals->referee.has_value() && globals->referee->online_status() == rm::device::Device::kOk;
    const uint16_t power_limit = ref_online ? globals->referee->data().robot_status.chassis_power_limit : 0U;
    const uint8_t sc_err = globals->supercap.has_value() ? globals->supercap->rx_data().error_code : 0xFFU;
    const float spin_target = ResolveSpinTargetYawDot(power_limit, sc_err);
    gimbal_update_input.chassis_yaw_rate_rad_s = input.mode_request.spin_dir * spin_target;
  } else {
    gimbal_update_input.chassis_yaw_rate_rad_s = input.estimator_input.imu.gyro_z_rad_s;
  }
  gimbal_update_input.chassis_pitch_rad = input.estimator_input.imu.pitch_rad;
  gimbal_update_input.yaw_motor_rad = input.estimator_input.yaw_motor_rad;
  gimbal_update_input.gimbal_imu_yaw_rad = input.gimbal_imu_yaw_rad;
  gimbal_update_input.gimbal_imu_pitch_rad = input.gimbal_imu_pitch_rad;
  gimbal_update_input.gimbal_imu_gyro_z_rad_s = input.gimbal_imu_gyro_z_rad_s;
  gimbal_update_input.gimbal_imu_gyro_x_rad_s = -input.gimbal_imu_gyro_x_rad_s;
  gimbal_update_input.dt_s = kControlLoopDtS;
  gimbal_update_input.measured_dt_s = measured_control_dt_s;
  gimbal_update_input.measured_dt_valid = measured_control_dt_valid;
  gimbal_update_input.ident = &globals->gimbal_ident;
  gimbal_update_input.test_profile = gimbal_output.control.gimbal_test_profile;
  globals->gimbal.Update(gimbal_update_input);
  gimbal_control_output = globals->gimbal.GetOutput();
  g_actuators.ApplyGimbalOutput(*globals, gimbal_control_output);
  wl_debug.gimbal_motors_enabled_latched = g_actuators.gimbal_motors_enabled_latched() ? 1U : 0U;

  // 云台电机在线状态
  {
    const bool yaw_ok =
        globals->yaw_motor.has_value() && globals->yaw_motor->online_status() == rm::device::Device::kOk;
    const bool pitch_ok =
        globals->pitch_motor.has_value() && globals->pitch_motor->online_status() == rm::device::Device::kOk;
    wl_debug.yaw_motor_online = yaw_ok ? 1U : 0U;
    wl_debug.pitch_motor_online = pitch_ok ? 1U : 0U;
  }

  // 辨识模式串口数据发送
  if (gimbal_control_output.ident_data_pending && gimbal_control_output.ident_tx_data != nullptr) {
    // Do not block the 500 Hz ISR while a full CSV line is shifted out.
    if (!globals->no_dtcm->ident_uart.IsTxBusy()) {
      const size_t tx_len = std::min(gimbal_control_output.ident_tx_len, globals->no_dtcm->ident_tx_buffer.size());
      std::memcpy(globals->no_dtcm->ident_tx_buffer.data(), gimbal_control_output.ident_tx_data, tx_len);
      globals->no_dtcm->ident_uart.WriteAsync(globals->no_dtcm->ident_tx_buffer.data(), tx_len, {});
    }
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
  const bool gimbal_rx_valid = globals->gimbal_rx.has_value() && globals->gimbal_rx->frame_count() > 0;
  float fric_speed_target = gimbal_rx_valid ? static_cast<float>(globals->gimbal_rx->fric_speed_target_rpm()) : 0.0f;
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  // Hero：三摩擦轮 + DM 拨盘，ShootController 内置 5 状态机自行下发
  {
    const bool shooter_enter = (gimbal_output.mode == gimbal::Fsm::State::kCombat);
    const bool manual_fire = input.dr16.dial < ns::shoot::kFireDialThreshold || input.tc_remote.left_button;
    const bool fire_flag =
        manual_fire || (gimbal_output.control.active_target_source == wheel_legged::TargetSource::kHost &&
                        (manual_fire || (globals->aimbot->aimbot_state() >> 1) & 1));
    const int32_t heat_delta = globals->referee->data().robot_status.shooter_barrel_heat_limit -
                               globals->referee->data().power_heat_data.shooter_42mm_barrel_heat;
    globals->shoot_controller.Update(shooter_enter, fire_flag, true, heat_delta);
    wl_debug.booster_raw_pos_rad = globals->shoot_controller.booster_pos();
    wl_debug.booster_target_rad = globals->shoot_controller.booster_target();
    wl_debug.shoot_hero_state = static_cast<uint8_t>(globals->shoot_controller.state());
    wl_debug.shoot_hero_fire_trigger = fire_flag ? 1U : 0U;
    wl_debug.shoot_hero_enter = shooter_enter ? 1U : 0U;
    wl_debug.shoot_hero_heat_delta = heat_delta;
    wl_debug.fw_raw_rpm_1 = gimbal_rx_valid ? static_cast<float>(globals->gimbal_rx->fric_left_rpm()) : 0.0f;
    wl_debug.fw_raw_rpm_2 = gimbal_rx_valid ? static_cast<float>(globals->gimbal_rx->fric_right_rpm()) : 0.0f;
    wl_debug.fw_raw_rpm_3 = 0.0f;  // 当前 CAN 协议仅传输 2 个摩擦轮转速
    rm::device::DjiMotorBase::SendCommand(*globals->gimbal_can);

    if (gimbal_rx_valid && globals->gimbal_rx->PopShotDetected()) {
      if (hero_remaining_ammo > 0) {
        --hero_remaining_ammo;
      }
    }
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
    const float fric_left_rpm = gimbal_rx_valid ? static_cast<float>(globals->gimbal_rx->fric_left_rpm()) : 0.0f;
    const float fric_right_rpm = gimbal_rx_valid ? static_cast<float>(globals->gimbal_rx->fric_right_rpm()) : 0.0f;
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
                              in_combat, fric_speed_target, single_shot, ref_barrel_heat);
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

  wl_debug.fric_speed_target_rpm = fric_speed_target;

  // ── 发送 combat_mode 到云台（100Hz，每5个控制周期发一次）──
  if (globals->chassis_tx.has_value()) {
    static uint32_t chassis_tx_counter = 0;
    if (chassis_tx_counter % 5 == 0) {
      const bool combat_mode = (gimbal_output.mode == gimbal::Fsm::State::kCombat);
      const float bullet_speed = globals->referee.has_value()
                                     ? globals->referee->data().shoot_data.initial_speed
                                     : 0.0f;
      globals->chassis_tx->SetCombatMode(combat_mode);
      globals->chassis_tx->SetBulletSpeed(bullet_speed);
      globals->chassis_tx->QueueSend();
    }
    chassis_tx_counter++;
  }

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
  chassis_update_input.motion_target.leg_length_m = chassis_output.control.target_leg_length_m;
  if (stair_sequence_output.controls_motion && chassis_output.mode == chassis::Fsm::State::kStairTask) {
    chassis_update_input.motion_target = stair_sequence_output.target;
  }
  chassis_update_input.keyboard_active = input.tc_remote.valid && !input.tc_remote.tc_from_dr16;
  chassis_update_input.estimator_input = input.estimator_input;
  chassis_update_input.estimator_input.dt_s = kControlLoopDtS;
  chassis_update_input.yaw_centering_complete = gimbal_control_output.yaw_centered;

  // ── 7b. 模式切换处理 ──
  const auto &current_state = chassis_control_output.current_state;
  const bool mode_changed = (chassis_output.mode != ctx.last_chassis_mode);
  const auto is_spin_like = [](chassis::Fsm::State s) {
    return s == chassis::Fsm::State::kSpin || s == chassis::Fsm::State::kSpinExitPending;
  };
  const bool last_was_spin = is_spin_like(ctx.last_chassis_mode);
  const bool now_is_spin = is_spin_like(chassis_output.mode);
  const bool now_is_spin_running = chassis_output.mode == chassis::Fsm::State::kSpin;
  const bool now_is_spin_exit_pending = chassis_output.mode == chassis::Fsm::State::kSpinExitPending;
  const bool now_is_standby = chassis_output.mode == chassis::Fsm::State::kStandby;
  const bool cross_spin = (last_was_spin != now_is_spin);
  if (mode_changed) {
    const auto previous_mode = ctx.last_chassis_mode;
    ctx.ResetOnModeChange(current_state.s, current_state.s_dot);
    if (cross_spin) {
      if (!now_is_spin) {
        if (previous_mode == chassis::Fsm::State::kSpinExitPending) {
          ctx.filtered_yaw_dot = chassis_control_output.current_state.phi_dot;
          ctx.spin_exit_recovery = true;
          ctx.yaw_follow_align_mode = YawFollowAlignMode::kForward;
        }
        // 退出小陀螺：继承当前 phi_dot，后续 yaw follow 使用退出斜坡缓慢收敛
      } else {
        // 进入小陀螺：继承当前车体角速度做平滑起步，强制对齐正前方
        ctx.filtered_yaw_dot = chassis_control_output.current_state.phi_dot;
        ctx.yaw_follow_align_mode = YawFollowAlignMode::kForward;
      }
    } else if (previous_mode == chassis::Fsm::State::kSpin && now_is_spin_exit_pending) {
      ctx.filtered_yaw_dot = chassis_control_output.current_state.phi_dot;
      ctx.spin_exit_recovery = true;
      ctx.yaw_follow_align_mode = YawFollowAlignMode::kForward;
      ctx.yaw_follow_target = SelectNearestYawTarget(input.estimator_input.yaw_motor_rad, 0.0f);
      ctx.yaw_follow_target_initialized = true;
    }
    ctx.last_chassis_mode = chassis_output.mode;
  }

  // ── 7c. 驾驶输入解析 ──
  const bool dr16_online = input.dr16.online;
  const bool tc_remote_active = input.tc_remote.valid;
  const bool has_drive_input = dr16_online || tc_remote_active;

  // AD 屏蔽：暂时永久关闭横向移动（小陀螺模式除外）
  if (chassis_output.mode != chassis::Fsm::State::kSpin &&
      chassis_output.mode != chassis::Fsm::State::kSpinExitPending) {
    input.dr16.keyboard &= ~0x000Cu;
    input.tc_remote.keyboard_value &= ~0x000Cu;
  }

  static DriveInputRampState drive_ramp{};
  const auto drive = ResolveDriveInput(input.dr16, input.tc_remote, drive_ramp);
  const float forward_input_norm = drive.forward;
  const float side_input_norm = drive.side;
  const bool forward_input_active = std::fabs(forward_input_norm) > kVxInputDeadbandNorm;
  const bool side_input_active = std::fabs(side_input_norm) > kVyInputDeadbandNorm;

  // ── 7d. 偏航跟随模式选择（非自旋模式专用，自旋期间侧向指令由 7h 全向投影独立处理）──
  YawFollowAlignMode requested_yaw_follow_align_mode = ctx.yaw_follow_align_mode;
  if (!has_drive_input || chassis_input.request.domain_request == wheel_legged::DomainRequest::kDisabled) {
    requested_yaw_follow_align_mode = YawFollowAlignMode::kForward;
  } else if (now_is_spin_running || now_is_spin_exit_pending) {
    // 自旋期间冻结 yaw follow 模式，不处理方向性输入
    requested_yaw_follow_align_mode = YawFollowAlignMode::kForward;
  } else if (forward_input_active) {
    requested_yaw_follow_align_mode = YawFollowAlignMode::kForward;
  } else if (side_input_active) {
    requested_yaw_follow_align_mode =
        (side_input_norm > 0.0f) ? YawFollowAlignMode::kSidePositive : YawFollowAlignMode::kSideNegative;
  }

  // ── 7e. 偏航目标更新 ──
  const bool yaw_follow_mode_changed = requested_yaw_follow_align_mode != ctx.yaw_follow_align_mode;
  if ((!has_drive_input || chassis_input.request.domain_request == wheel_legged::DomainRequest::kDisabled) &&
      !now_is_spin_exit_pending && !ctx.yaw_target_ramp_active) {
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
  }

  // R 键重置底盘正方向（静止时也生效，目标为固定偏置角）
  wl_debug.reset_yaw_request = input.mode_request.reset_yaw_request ? 1 : 0;
  if (input.mode_request.reset_yaw_request) {
    const auto nearest = SelectNearestYawTarget(input.estimator_input.yaw_motor_rad, 0.0f);
    ctx.yaw_target_ramp_final = nearest.target_rad;
    ctx.defer_yaw_target_rad = nearest.target_rad;
    ctx.yaw_target_ramp_active = true;
    ctx.yaw_follow_target = {input.estimator_input.yaw_motor_rad, nearest.drive_sign};
    ctx.yaw_follow_align_mode = YawFollowAlignMode::kForward;
    ctx.yaw_follow_target_initialized = true;
    ctx.yaw_follow_drive_ready = false;
    ctx.yaw_follow_drive_ready_stable_ticks = 0U;
    ctx.filtered_yaw_dot = 0.0f;
  }

  // yaw 目标斜坡：target 从当前位置以固定步长向最终目标移动
  if (ctx.yaw_target_ramp_active) {
    const float error_to_final =
        rm::modules::Wrap(ctx.yaw_target_ramp_final - ctx.yaw_follow_target.target_rad, -kPi, kPi);
    if (std::fabs(error_to_final) < kYawTargetRampStepRad) {
      ctx.yaw_target_ramp_active = false;
      ctx.yaw_follow_target.target_rad = ctx.yaw_target_ramp_final;
    } else {
      ctx.yaw_follow_target.target_rad =
          rm::modules::Wrap(
              ctx.yaw_follow_target.target_rad + std::copysign(kYawTargetRampStepRad, error_to_final), -kPi, kPi);
    }
  }

  // R 键云台转 180°：翻转底盘驱动方向，抑制偏航跟随直到云台旋转完成
  if (input.mode_request.flip_180_request) {
    ctx.yaw_follow_target.drive_sign = -ctx.yaw_follow_target.drive_sign;
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
  const bool spin_control_enabled = chassis_output.mode == chassis::Fsm::State::kSpin && chassis_output_enable &&
                                    chassis_output.control.run_chassis_update;
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
  const uint8_t sc_err = globals->supercap.has_value() ? globals->supercap->rx_data().error_code : 0xFFU;
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
  float forward_max_speed = forward_speed_base;
  // 大转向时压低速度上限：先减速再转向，避免高速急转翻倒
  const float motor_error =
      rm::modules::Wrap(ctx.yaw_follow_target.target_rad - input.estimator_input.yaw_motor_rad, -kPi, kPi);
  if (std::fabs(motor_error) > kLargeTurnThresholdRad &&
      (std::fabs(current_state.s_dot) > kSafeTurnSpeedMps ||
       std::fabs(current_state.theta_ll) > kLargeTurnThetaThresholdRad ||
       std::fabs(current_state.theta_lr) > kLargeTurnThetaThresholdRad)) {
    forward_max_speed = std::min(forward_max_speed, kSafeTurnSpeedMps);
  }
  // 限速激活时标记恢复状态，解除后用缓加速斜坡逐步恢复速度
  static bool s_large_turn_recovery = false;
  if (forward_max_speed < forward_speed_base) {
    s_large_turn_recovery = true;
  }
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
  } else if (now_is_spin_exit_pending) {
    target_s_dot = 0.0f;
  } else if (!ctx.yaw_follow_drive_ready) {
    target_s_dot = 0.0f;
  } else if (forward_input_active) {
    target_s_dot = yaw_follow_drive_sign * forward_max_speed * forward_input_norm;
  } else if (side_input_active) {
    target_s_dot = yaw_follow_drive_sign * forward_max_speed * side_input_norm;
  }
  target_s_dot += forward_speed_bias;
  const bool stair_seq_active = stair_task_output.mode == wheel_legged::StairTaskMode::kExecuting ||
                                stair_task_output.mode == wheel_legged::StairTaskMode::kBetweenSteps;
  if (!chassis_output_enable || now_is_standby || stair_seq_active) {
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
    // 大转向限速解除后，用缓加速斜坡逐步恢复速度
    float accel_scale = 1.0f;
    if (s_large_turn_recovery && forward_max_speed >= forward_speed_base) {
      accel_scale = kLargeTurnRecoveryAccelScale;
      if (std::fabs(ctx.filtered_s_dot - target_s_dot) < 0.05f) {
        s_large_turn_recovery = false;
      }
    }
    const SdotRampParams effective_ramp{ramp_params.accel_step * accel_scale, ramp_params.brake_step};
    RampValueToTarget(target_s_dot, ctx.filtered_s_dot, effective_ramp);
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
  // ── φ 目标：将偏航电机角度误差映射为 LQR 的底盘朝向误差 ──
  // motor_error = 电机当前角度偏离目标的角度，底盘偏离期望朝向的大小与之相同
  // 非自旋、非翻转、chassis 输出使能时才生效，否则不产生朝向力矩
  // kSpinExitPending 阶段同样生效：主动转向使 yaw 电机靠拢正方向，避免被动等超时
  if (!spin_control_enabled && !ctx.flip_180_in_progress && chassis_output_enable &&
      ctx.yaw_follow_target_initialized) {
    const float motor_error =
        rm::modules::Wrap(ctx.yaw_follow_target.target_rad - input.estimator_input.yaw_motor_rad, -kPi, kPi);
    chassis_update_input.expected.phi = current_state.phi - motor_error;
  } else {
    chassis_update_input.expected.phi = current_state.phi;
  }
  chassis_update_input.expected.phi_dot = 0.0f;

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
  } else {
    const float l_l = chassis_control_output.current_state.l_l;
    const float l_r = chassis_control_output.current_state.l_r;
    chassis_update_input.expected.theta_ll = LookupThetaBiasSingle(l_l, wheel_legged::params::generated::kThetaBiasLl);
    chassis_update_input.expected.theta_lr = LookupThetaBiasSingle(l_r, wheel_legged::params::generated::kThetaBiasLr);
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
    const bool ref_online =
        globals->referee.has_value() && globals->referee->online_status() == rm::device::Device::kOk;
    const uint16_t power_limit = ref_online ? globals->referee->data().robot_status.chassis_power_limit : 0U;
    const float spin_target = ResolveSpinTargetYawDot(power_limit, sc_err);
    RampYawDotToTarget(input.mode_request.spin_dir * spin_target, ctx.filtered_yaw_dot, kSpinYawRampStepRadS);
    chassis_update_input.expected.phi_dot = ctx.filtered_yaw_dot;
  } else if (ctx.flip_180_in_progress) {
    // R 键云台 180° 旋转中：抑制偏航跟随，等待云台旋转完成
    ctx.filtered_yaw_dot = 0.0f;
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
  } else {
    const float target_yaw_dot = 0.0f;
    // 高速时缩小 yaw 斜坡步长，降低转向响应速度以减少翻倒风险
    const float speed_norm = std::fabs(current_state.s_dot) / forward_max_speed;
    constexpr float kMinYawRampScale = 0.4f;
    const float yaw_ramp_scale = std::clamp(1.0f - speed_norm, kMinYawRampScale, 1.0f);
    const float yaw_follow_step = (has_supercap ? kYawFollowRampStepRadS : kYawFollowRampStepRadNoScS) * yaw_ramp_scale;
    const float ramp_step = ctx.spin_exit_recovery ? kSpinExitYawRampStepRadS : yaw_follow_step;
    RampYawDotToTarget(target_yaw_dot, ctx.filtered_yaw_dot, ramp_step);
    chassis_update_input.expected.phi_dot = ctx.filtered_yaw_dot;
    // 退出小陀螺快速恢复：车体角速度降到 1 rad/s 以下切回正常斜坡
    if (ctx.spin_exit_recovery && std::fabs(ctx.filtered_yaw_dot) < 1.0f) {
      ctx.spin_exit_recovery = false;
    }
  }

  // ── 7m. 底盘控制器执行 ──
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  {
    const float shots_fired = static_cast<float>(ns::control_loop::kInitialAmmoCount - hero_remaining_ammo);
    chassis_update_input.displacement_bias =
        ns::control_loop::kExpectedDisplacementBiasMLowLeg + shots_fired * ns::control_loop::kDisplacementBiasPerShot;
    wl_debug.hero_remaining_ammo = hero_remaining_ammo;
    wl_debug.hero_displacement_bias = chassis_update_input.displacement_bias;
  }
#endif
  chassis_update_input.position_hold_active = ctx.integrate_position;
  globals->chassis.Update(chassis_update_input);
  chassis_control_output = globals->chassis.GetOutput();

  // ── 中腿长下压退出：下降沿清除中腿长保持，走斜坡到低腿长 ──
  {
    static bool prev_dip_active = false;
    if (prev_dip_active && !chassis_control_output.mid_leg_dip_active) {
      tc_state.mid_leg_hold = false;
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

  if (chassis_output.mode == chassis::Fsm::State::kDisabled) {
    g_actuators.ResetDmMotorsLatch();
  }
  g_actuators.ApplyChassisOutput(*globals, chassis_control_output, chassis_output_enable);
  wl_debug.dm_enabled_latched = g_actuators.dm_enabled_latched() ? 1U : 0U;

  // 底盘电机在线状态
  {
    const bool lf_ok = globals->dm_lf.has_value() && globals->dm_lf->online_status() == rm::device::Device::kOk;
    const bool lb_ok = globals->dm_lb.has_value() && globals->dm_lb->online_status() == rm::device::Device::kOk;
    const bool rf_ok = globals->dm_rf.has_value() && globals->dm_rf->online_status() == rm::device::Device::kOk;
    const bool rb_ok = globals->dm_rb.has_value() && globals->dm_rb->online_status() == rm::device::Device::kOk;
    wl_debug.dm_lf_online = lf_ok ? 1U : 0U;
    wl_debug.dm_lb_online = lb_ok ? 1U : 0U;
    wl_debug.dm_rf_online = rf_ok ? 1U : 0U;
    wl_debug.dm_rb_online = rb_ok ? 1U : 0U;
  }

  // ═══════════════════════════════════════════════════════════════════════
  // 阶段 8：自瞄通信 — 云台 IMU 欧拉角→CAN 转发
  // ═══════════════════════════════════════════════════════════════════════
  if (globals->aimbot.has_value() && globals->gimbal_rx.has_value() && globals->gimbal_rx->frame_count() > 0) {
    constexpr float kRadToDeg = 180.f / kPi;
    const float yaw_deg = globals->gimbal_rx->euler_yaw_rad() * kRadToDeg;
    const float pitch_deg = globals->gimbal_rx->euler_pitch_rad() * kRadToDeg;
    const float roll_deg = -globals->gimbal_rx->euler_roll_rad() * kRadToDeg;

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
    ui_snapshot.shooter_output = globals->referee->data().robot_status.power_management_shooter_output != 0;
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
    wl_debug.supercap_error_code = globals->supercap->rx_data().error_code;
    wl_debug.supercap_chassis_power = globals->supercap->rx_data().chassis_power;
    wl_debug.supercap_chassis_power_limit = globals->supercap->rx_data().chassis_power_limit;
    wl_debug.supercap_cap_energy = globals->supercap->rx_data().cap_energy;
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
    ui_snapshot.ad_active = chassis_output.mode == chassis::Fsm::State::kSpin ||
                            chassis_output.mode == chassis::Fsm::State::kSpinExitPending;
    ui_snapshot.yaw_display_offset_rad = -ns::control_loop::kYawFollowFixedTargetRad;

    ui_snapshot.supercap_cap_energy =
        globals->supercap.has_value() ? static_cast<float>(globals->supercap->rx_data().cap_energy) : 0.0f;
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
    const bool ui_gimbal_rx_valid = globals->gimbal_rx.has_value() && globals->gimbal_rx->frame_count() > 0;
    ui_snapshot.fric_left_rpm = ui_gimbal_rx_valid ? static_cast<float>(globals->gimbal_rx->fric_left_rpm()) : 0.0f;
    ui_snapshot.fric_right_rpm = ui_gimbal_rx_valid ? static_cast<float>(globals->gimbal_rx->fric_right_rpm()) : 0.0f;
#else
    const bool ui_gimbal_rx_valid = globals->gimbal_rx.has_value() && globals->gimbal_rx->frame_count() > 0;
    ui_snapshot.fric_left_rpm = ui_gimbal_rx_valid ? static_cast<float>(globals->gimbal_rx->fric_left_rpm()) : 0.0f;
    ui_snapshot.fric_right_rpm = ui_gimbal_rx_valid ? static_cast<float>(globals->gimbal_rx->fric_right_rpm()) : 0.0f;
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

      const auto &buff = globals->subReferee->data().enemy_robot_buff;
      ui_snapshot.enemy_hero_1_defense = buff.hero.defense;
      ui_snapshot.enemy_engineer_2_defense = buff.engineer.defense;
      ui_snapshot.enemy_standard_3_defense = buff.infantry3.defense;
      ui_snapshot.enemy_standard_4_defense = buff.infantry4.defense;
      ui_snapshot.enemy_sentry_7_defense = buff.sentry.defense;
    } else {
      ui_snapshot.enemy_gold_remaining = 0;
      ui_snapshot.enemy_gold_total = 0;
      ui_snapshot.enemy_hero_1_allowance = 0;
      ui_snapshot.enemy_standard_3_allowance = 0;
      ui_snapshot.enemy_standard_4_allowance = 0;
      ui_snapshot.enemy_drone_6_allowance = 0;
      ui_snapshot.enemy_sentry_7_allowance = 0;
      ui_snapshot.enemy_hero_1_defense = 0;
      ui_snapshot.enemy_engineer_2_defense = 0;
      ui_snapshot.enemy_standard_3_defense = 0;
      ui_snapshot.enemy_standard_4_defense = 0;
      ui_snapshot.enemy_sentry_7_defense = 0;
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
          case 0:
            target_hp = hp.sentry_7_HP;
            target_allowance = allow.sentry_7_projectile_allowance;
            break;
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
    static_UI_add();
    init_flag = true;
  }
  times++;
  {
    static uint32_t ai_policy_tick = 0U;
    if (++ai_policy_tick >= 10U) {
      ai_policy_tick = 0U;
      if (input.mode_request.leg_request == wheel_legged::LegProfile::kMid) {
        const auto policy_input =
            BuildRealPolicyInput(input, chassis_control_output, ctx.filtered_s_dot, ctx.filtered_yaw_dot);
        wheel_legged::ai::PolicyTestRequest(policy_input);
      }
    }
  }
  // TEMP: Disable UI task execution while measuring the 500 Hz control-loop
  // baseline. UI tasks use blocking HAL_UART_Transmit() and previously ran
  // inside this TIM13 ISR. Keep task registration/snapshots intact so this can
  // be restored after the timing test.
  // if (times % 17 == 0) {
  //   schedule.schedule();
  //   if (globals->ui_refresh_key) {
  //     static_UI_add();
  //   }
  // }

  if (SystemCoreClock != 0U) {
    static uint32_t control_exec_max_us = 0U;
    const uint32_t elapsed_cycles = DWT->CYCCNT - control_exec_start_cycles;
    const uint32_t control_exec_us =
        static_cast<uint32_t>((static_cast<uint64_t>(elapsed_cycles) * 1000000ULL) / SystemCoreClock);
    if (control_exec_us > control_exec_max_us) control_exec_max_us = control_exec_us;
    wl_debug.control_exec_last_us = static_cast<uint16_t>(std::min<uint32_t>(control_exec_us, 0xFFFFU));
    wl_debug.control_exec_max_us = static_cast<uint16_t>(std::min<uint32_t>(control_exec_max_us, 0xFFFFU));
    if (control_exec_us > 2000U) ++wl_debug.control_overrun_count;
  }
}
