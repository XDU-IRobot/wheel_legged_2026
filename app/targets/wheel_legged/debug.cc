/**
 * @file  targets/wheel_legged/debug.cc
 * @brief DebugSnapshot 填充实现
 */

#include "include/debug.hpp"

#include <array>

#include "include/ai/policy_runner.hpp"
#include "include/globals.hpp"

namespace {

constexpr std::size_t kTofMeanWindow = wheel_legged::params::active::tof::kDebugMovingAverageWindow;

struct TofDebugFilterState {
  std::array<float, kTofMeanWindow> window{};
  uint32_t last_sample_count{0U};
  std::size_t next_index{0U};
  std::size_t valid_count{0U};
  float low_pass_mm{0.0F};
  float window_sum_mm{0.0F};
  bool initialized{false};
};

struct TofDebugFilterOutput {
  float low_pass_mm;
  float moving_average_mm;
};

TofDebugFilterOutput UpdateTofDebugFilter(const rm::device::Vl53l4cd &tof, TofDebugFilterState &state) {
  const uint32_t sample_count = tof.sample_count();
  if (sample_count < state.last_sample_count) state = {};
  if (sample_count == state.last_sample_count) {
    return {state.low_pass_mm,
            state.valid_count > 0U ? state.window_sum_mm / static_cast<float>(state.valid_count) : 0.0F};
  }
  state.last_sample_count = sample_count;

  if (sample_count == 0U || !tof.data_valid()) {
    return {state.low_pass_mm,
            state.valid_count > 0U ? state.window_sum_mm / static_cast<float>(state.valid_count) : 0.0F};
  }

  const float raw_mm = static_cast<float>(tof.measurement().distance_mm);
  if (!state.initialized) {
    state.low_pass_mm = raw_mm;
    state.initialized = true;
  } else {
    constexpr float alpha = wheel_legged::params::active::tof::kDebugLowPassAlpha;
    state.low_pass_mm += alpha * (raw_mm - state.low_pass_mm);
  }

  if (state.valid_count == kTofMeanWindow) {
    state.window_sum_mm -= state.window[state.next_index];
  } else {
    ++state.valid_count;
  }
  state.window[state.next_index] = state.low_pass_mm;
  state.window_sum_mm += state.low_pass_mm;
  state.next_index = (state.next_index + 1U) % kTofMeanWindow;

  return {state.low_pass_mm, state.window_sum_mm / static_cast<float>(state.valid_count)};
}

}  // namespace

void UpdateDebugSnapshot(const uint32_t tick_ms, const wheel_legged::control_loop::InputSnapshot &input,
                         const chassis::Fsm::Output &chassis_output, const gimbal::Fsm::Output &gimbal_output,
                         const chassis::Chassis::UpdateOutput &chassis_control_output,
                         const gimbal::Gimbal::UpdateOutput &gimbal_control_output,
                         const chassis::StairTaskCoordinator::Output &stair_task_output,
                         const chassis::StairClimbSequence::Output &stair_sequence_output) {
  // ── 时间戳与状态机 ──
  wl_debug.tick_ms = tick_ms;
  wl_debug.chassis_fsm_state = static_cast<uint8_t>(chassis_output.mode);
  wl_debug.gimbal_fsm_state = static_cast<uint8_t>(gimbal_output.mode);
  wl_debug.chassis_fsm_state_changed = static_cast<uint8_t>(chassis_output.state_changed);
  wl_debug.gimbal_fsm_state_changed = static_cast<uint8_t>(gimbal_output.state_changed);

  wl_debug.tof_runtime_enabled = static_cast<uint8_t>(wheel_legged::params::active::tof::kEnabled);
  wl_debug.tof_requested_mode = static_cast<uint8_t>(globals->requested_tof_mode);
  wl_debug.tof_active_mode = static_cast<uint8_t>(globals->active_tof_mode);
  wl_debug.tof_mode_ready = static_cast<uint8_t>(globals->tof_mode_ready);
  wl_debug.tof_init_error_mask = globals->tof_init_error_mask;
  wl_debug.tof_switch_count = globals->tof_switch_count;
  wl_debug.tof_poll_request_count = globals->tof_poll_request_count;
  wl_debug.tof_poll_process_count = globals->tof_poll_process_count;
  wl_debug.tof_poll_coalesced_count = globals->tof_poll_coalesced_count;
  wl_debug.tof_poll_last_us = globals->tof_poll_last_us;
  wl_debug.tof_poll_max_us = globals->tof_poll_max_us;

#define COPY_TOF_DEBUG(name)                                                \
  if (globals->name.has_value()) {                                          \
    const auto &tof = *globals->name;                                       \
    wl_debug.name##_driver_status = static_cast<uint8_t>(tof.last_error()); \
    wl_debug.name##_range_status = tof.measurement().range_status;          \
    wl_debug.name##_data_valid = static_cast<uint8_t>(tof.data_valid());    \
    wl_debug.name##_ranging = static_cast<uint8_t>(tof.ranging());          \
    wl_debug.name##_model_id = tof.model_id();                              \
    wl_debug.name##_distance_mm = tof.measurement().distance_mm;            \
    wl_debug.name##_sample_count = tof.sample_count();                      \
    wl_debug.name##_last_sample_tick_ms = tof.last_sample_tick_ms();        \
    wl_debug.name##_poll_count = tof.poll_count();                          \
    wl_debug.name##_i2c_error_count = tof.i2c_error_count();                \
  }
  COPY_TOF_DEBUG(left_front_tof)
  COPY_TOF_DEBUG(right_front_tof)
#undef COPY_TOF_DEBUG

  // 下方 TOF 精简字段（不使用 COPY_TOF_DEBUG 宏，因为下方字段集更小）
  if (globals->left_down_tof.has_value()) {
    const auto &tof = *globals->left_down_tof;
    wl_debug.left_down_tof_driver_status = static_cast<uint8_t>(tof.last_error());
    wl_debug.left_down_tof_range_status = tof.measurement().range_status;
    wl_debug.left_down_tof_data_valid = static_cast<uint8_t>(tof.data_valid());
    wl_debug.left_down_tof_ranging = static_cast<uint8_t>(tof.ranging());
    wl_debug.left_down_tof_distance_mm = tof.measurement().distance_mm;
    wl_debug.left_down_tof_sample_count = tof.sample_count();
    wl_debug.left_down_tof_i2c_error_count = tof.i2c_error_count();
  }
  if (globals->right_down_tof.has_value()) {
    const auto &tof = *globals->right_down_tof;
    wl_debug.right_down_tof_driver_status = static_cast<uint8_t>(tof.last_error());
    wl_debug.right_down_tof_range_status = tof.measurement().range_status;
    wl_debug.right_down_tof_data_valid = static_cast<uint8_t>(tof.data_valid());
    wl_debug.right_down_tof_ranging = static_cast<uint8_t>(tof.ranging());
    wl_debug.right_down_tof_distance_mm = tof.measurement().distance_mm;
    wl_debug.right_down_tof_sample_count = tof.sample_count();
    wl_debug.right_down_tof_i2c_error_count = tof.i2c_error_count();
  }

  static TofDebugFilterState left_front_filter{};
  static TofDebugFilterState right_front_filter{};
  if (globals->left_front_tof.has_value()) {
    const auto output = UpdateTofDebugFilter(*globals->left_front_tof, left_front_filter);
    wl_debug.left_front_tof_low_pass_mm = output.low_pass_mm;
    wl_debug.left_front_tof_moving_average_mm = output.moving_average_mm;
  }
  if (globals->right_front_tof.has_value()) {
    const auto output = UpdateTofDebugFilter(*globals->right_front_tof, right_front_filter);
    wl_debug.right_front_tof_low_pass_mm = output.low_pass_mm;
    wl_debug.right_front_tof_moving_average_mm = output.moving_average_mm;
  }

  // 活跃 TOF 对的原始均值（auto_jump 用前向，stair_descend 用下向）
  wl_debug.active_tof_pair_raw_mean_mm = 0.0F;
  if (globals->tof_mode_ready) {
    if (globals->active_tof_mode == wheel_legged::TofMode::kAutoJump && globals->left_front_tof.has_value() &&
        globals->right_front_tof.has_value() && globals->left_front_tof->ranging() &&
        globals->right_front_tof->ranging() && globals->left_front_tof->data_valid() &&
        globals->right_front_tof->data_valid()) {
      wl_debug.active_tof_pair_raw_mean_mm =
          0.5F * (static_cast<float>(globals->left_front_tof->measurement().distance_mm) +
                  static_cast<float>(globals->right_front_tof->measurement().distance_mm));
    } else if (globals->active_tof_mode == wheel_legged::TofMode::kStairDescend && globals->left_down_tof.has_value() &&
               globals->right_down_tof.has_value() && globals->left_down_tof->ranging() &&
               globals->right_down_tof->ranging() && globals->left_down_tof->data_valid() &&
               globals->right_down_tof->data_valid()) {
      wl_debug.active_tof_pair_raw_mean_mm =
          0.5F * (static_cast<float>(globals->left_down_tof->measurement().distance_mm) +
                  static_cast<float>(globals->right_down_tof->measurement().distance_mm));
    }
  }

  // ── DR16 原始输入 ──
  wl_debug.dr16_online = static_cast<uint8_t>(input.dr16.online);

  wl_debug.auto_jump_triggered = static_cast<uint8_t>(input.auto_jump_triggered);
  wl_debug.auto_jump_enabled = static_cast<uint8_t>(input.auto_jump_enabled);
  wl_debug.auto_jump_both_close = static_cast<uint8_t>(input.auto_jump_both_close);
  wl_debug.auto_jump_tof_armed = static_cast<uint8_t>(input.auto_jump_tof_armed_debug);
  wl_debug.auto_jump_both_active = static_cast<uint8_t>(input.auto_jump_both_active);
  wl_debug.auto_jump_trigger_ready = static_cast<uint8_t>(input.auto_jump_trigger_ready);
  wl_debug.stair_descend_request = static_cast<uint8_t>(input.mode_request.stair_descend_request);
  wl_debug.stair_descend_ready = static_cast<uint8_t>(input.mode_request.stair_descend_ready);
  wl_debug.stair_descend_triggered = static_cast<uint8_t>(input.stair_descend_triggered);
  wl_debug.tc_remote_valid = static_cast<uint8_t>(input.tc_remote.valid);
  wl_debug.tc_keyboard_value = input.tc_remote.keyboard_value;
  wl_debug.tc_mouse_x = input.tc_remote.mouse_x;
  wl_debug.tc_mouse_y = input.tc_remote.mouse_y;
  wl_debug.tc_left_button = static_cast<uint8_t>(input.tc_remote.left_button);
  wl_debug.tc_right_button = static_cast<uint8_t>(input.tc_remote.right_button);
  wl_debug.tc_ui_refresh_key = static_cast<uint8_t>(input.ui_refresh_key);
  wl_debug.stair_task_request = static_cast<uint8_t>(input.mode_request.stair_task_request);
  wl_debug.stair_high_leg_request = static_cast<uint8_t>(stair_task_output.request_high_leg);
  wl_debug.stair_task_mode = static_cast<uint8_t>(stair_task_output.mode);
  wl_debug.stair_requested_attempts = stair_task_output.requested_attempts;
  wl_debug.stair_completed_attempts = stair_task_output.completed_attempts;
  wl_debug.stair_phase = static_cast<uint8_t>(stair_sequence_output.phase);
  wl_debug.stair_abort_reason = static_cast<uint8_t>(stair_sequence_output.abort_reason);
  wl_debug.stair_target_leg_length_m = stair_sequence_output.target.leg_length_m;
  wl_debug.stair_target_theta_ll_rad = stair_sequence_output.target.theta_ll_rad;
  wl_debug.stair_target_theta_lr_rad = stair_sequence_output.target.theta_lr_rad;
  wl_debug.stair_theta_ll_error_rad = stair_sequence_output.theta_ll_error_rad;
  wl_debug.stair_theta_lr_error_rad = stair_sequence_output.theta_lr_error_rad;
  wl_debug.stair_leg_length_error_m = stair_sequence_output.leg_length_error_m;
  wl_debug.stair_phase_elapsed_ms = stair_sequence_output.phase_elapsed_ms;
  wl_debug.stair_stable_elapsed_ms = stair_sequence_output.stable_elapsed_ms;
  wl_debug.stair_t_bl_cmd = chassis_control_output.stair_t_bl_cmd;
  wl_debug.stair_t_br_cmd = chassis_control_output.stair_t_br_cmd;


  // ── 底盘状态 ──
  wl_debug.chassis_leg_target_length_m = chassis_control_output.leg_target_length_m;
  wl_debug.chassis_mean_leg_length_m = chassis_control_output.mean_leg_length_m;
  wl_debug.chassis_speed_mps = chassis_control_output.speed_mps;
  wl_debug.chassis_raw_wheel_speed_mps = chassis_control_output.raw_wheel_speed_mps;
  wl_debug.chassis_filtered_wheel_speed_mps = chassis_control_output.filtered_wheel_speed_mps;
  wl_debug.chassis_raw_accel_speed_mps = chassis_control_output.raw_accel_speed_mps;
  wl_debug.chassis_imu_acc_x_integral_mps = chassis_control_output.imu_acc_x_integral_mps;
  wl_debug.chassis_left_force_n = chassis_control_output.left_force_n;
  wl_debug.chassis_right_force_n = chassis_control_output.right_force_n;
  wl_debug.chassis_left_support_force_n = chassis_control_output.left_support_force_n;
  wl_debug.chassis_right_support_force_n = chassis_control_output.right_support_force_n;
  wl_debug.chassis_left_F_bh_n = chassis_control_output.left_F_bh_n;
  wl_debug.chassis_right_F_bh_n = chassis_control_output.right_F_bh_n;
  wl_debug.chassis_left_gravity_support_n = chassis_control_output.left_gravity_support_n;
  wl_debug.chassis_right_gravity_support_n = chassis_control_output.right_gravity_support_n;
  wl_debug.chassis_left_dyn_support_n = chassis_control_output.left_dyn_support_n;
  wl_debug.chassis_right_dyn_support_n = chassis_control_output.right_dyn_support_n;
  wl_debug.chassis_off_ground = static_cast<uint8_t>(chassis_control_output.off_ground_in_mid_high_leg);

  // ── 关节电机原始反馈 ──
  const auto &motor = input.estimator_input;
  wl_debug.motor_raw_lf_pos_rad = motor.left_leg.front.pos_rad;
  wl_debug.motor_raw_lf_vel_rad_s = motor.left_leg.front.vel_rad_s;
  wl_debug.motor_raw_lf_tau_nm = motor.left_leg.front.torque_nm;
  wl_debug.motor_raw_lb_pos_rad = motor.left_leg.back.pos_rad;
  wl_debug.motor_raw_lb_vel_rad_s = motor.left_leg.back.vel_rad_s;
  wl_debug.motor_raw_lb_tau_nm = motor.left_leg.back.torque_nm;
  wl_debug.motor_raw_rf_pos_rad = motor.right_leg.front.pos_rad;
  wl_debug.motor_raw_rf_vel_rad_s = motor.right_leg.front.vel_rad_s;
  wl_debug.motor_raw_rf_tau_nm = motor.right_leg.front.torque_nm;
  wl_debug.motor_raw_rb_pos_rad = motor.right_leg.back.pos_rad;
  wl_debug.motor_raw_rb_vel_rad_s = motor.right_leg.back.vel_rad_s;
  wl_debug.motor_raw_rb_tau_nm = motor.right_leg.back.torque_nm;

  // ── 轮毂电机 ──
  wl_debug.wheel_raw_left_rad_s = motor.wheel.left_rad_s;
  wl_debug.wheel_raw_right_rad_s = motor.wheel.right_rad_s;

  // ── 电机控制输出 ──
  wl_debug.wheel_cmd_left_tau_nm = chassis_control_output.lw_tau;
  wl_debug.wheel_cmd_right_tau_nm = chassis_control_output.rw_tau;
  wl_debug.motor_cmd_lf_tau_nm = chassis_control_output.lf_tau;
  wl_debug.motor_cmd_lb_tau_nm = chassis_control_output.lb_tau;
  wl_debug.motor_cmd_rf_tau_nm = chassis_control_output.rf_tau;
  wl_debug.motor_cmd_rb_tau_nm = chassis_control_output.rb_tau;

  // ── 底盘 IMU 原始反馈 ──
  wl_debug.imu_raw_roll_rad = motor.imu.roll_rad;
  wl_debug.imu_raw_pitch_rad = motor.imu.pitch_rad;
  wl_debug.imu_raw_yaw_rad = motor.imu.yaw_rad;
  wl_debug.imu_raw_gyro_x_rad_s = motor.imu.gyro_x_rad_s;
  wl_debug.imu_raw_gyro_y_rad_s = motor.imu.gyro_y_rad_s;
  wl_debug.imu_raw_gyro_z_rad_s = motor.imu.gyro_z_rad_s;
  wl_debug.imu_raw_acc_x_mps2 = motor.imu.acc_x_mps2;
  wl_debug.imu_raw_acc_y_mps2 = motor.imu.acc_y_mps2;
  wl_debug.imu_raw_acc_z_mps2 = motor.imu.acc_z_mps2;

  // ── 底盘状态向量 ──
  const auto &x = chassis_control_output.current_state;
  wl_debug.state_s_m = x.s;
  wl_debug.state_s_dot_mps = x.s_dot;
  wl_debug.state_phi_rad = x.phi;
  wl_debug.state_phi_dot_rad_s = x.phi_dot;
  wl_debug.state_theta_ll_rad = x.theta_ll;
  wl_debug.state_theta_ll_dot_rad_s = x.theta_ll_dot;
  wl_debug.state_theta_lr_rad = x.theta_lr;
  wl_debug.state_theta_lr_dot_rad_s = x.theta_lr_dot;
  wl_debug.state_theta_b_rad = x.theta_b;
  wl_debug.state_theta_b_dot_rad_s = x.theta_b_dot;
  wl_debug.state_l_l_m = x.l_l;
  wl_debug.state_l_r_m = x.l_r;
  wl_debug.chassis_left_leg_length_m = x.l_l;
  wl_debug.chassis_right_leg_length_m = x.l_r;
  wl_debug.chassis_left_l0_dot_mps = chassis_control_output.left_l0_dot_mps;
  wl_debug.chassis_right_l0_dot_mps = chassis_control_output.right_l0_dot_mps;
  wl_debug.chassis_left_l0_ddot_mps2 = chassis_control_output.left_l0_ddot_mps2;
  wl_debug.chassis_right_l0_ddot_mps2 = chassis_control_output.right_l0_ddot_mps2;
  wl_debug.filtered_theta_ll_dot_rad_s = chassis_control_output.filtered_theta_ll_dot;
  wl_debug.filtered_theta_lr_dot_rad_s = chassis_control_output.filtered_theta_lr_dot;
  wl_debug.chassis_left_l0_pid_out = chassis_control_output.left_l0_pid_out;
  wl_debug.chassis_right_l0_pid_out = chassis_control_output.right_l0_pid_out;

  // ── 云台反馈与电机 ──
  wl_debug.yaw_cmd_target_rad = gimbal_control_output.yaw_target_rad;
  wl_debug.yaw_motor_raw_pos_rad = input.estimator_input.yaw_motor_rad;
  wl_debug.pitch_motor_raw_pos_rad = input.estimator_input.pitch_motor_rad;
  wl_debug.gimbal_yaw_pos_feedback_rad = gimbal_control_output.yaw_pos_rad;
  wl_debug.gimbal_yaw_vel_feedback_rad_s = gimbal_control_output.yaw_vel_rad_s;
  wl_debug.yaw_cmd_torque_nm = gimbal_control_output.yaw_cmd_torque_nm;
  wl_debug.pitch_cmd_target_rad = gimbal_control_output.pitch_target_rad;
  wl_debug.gimbal_pitch_pos_feedback_rad = gimbal_control_output.pitch_pos_rad;
  wl_debug.gimbal_pitch_vel_feedback_rad_s = gimbal_control_output.pitch_vel_rad_s;
  wl_debug.pitch_cmd_torque_nm = gimbal_control_output.pitch_cmd_torque_nm;
  wl_debug.gimbal_yaw_dq_rad_s = gimbal_control_output.yaw_dq;
  wl_debug.gimbal_pitch_dq_rad_s = gimbal_control_output.pitch_dq;
  wl_debug.gimbal_yaw_ddq_rad_s2 = gimbal_control_output.yaw_ddq;
  wl_debug.gimbal_pitch_ddq_rad_s2 = gimbal_control_output.pitch_ddq;
  wl_debug.ff_yaw_inertia = gimbal_control_output.ff_yaw_inertia;
  wl_debug.ff_yaw_gravity = gimbal_control_output.ff_yaw_gravity;
  wl_debug.ff_yaw_friction = gimbal_control_output.ff_yaw_friction;
  wl_debug.ff_pitch_coupling = gimbal_control_output.ff_pitch_coupling;
  wl_debug.ff_pitch_inertia = gimbal_control_output.ff_pitch_inertia;
  wl_debug.ff_pitch_gravity = gimbal_control_output.ff_pitch_gravity;
  wl_debug.ff_pitch_friction = gimbal_control_output.ff_pitch_friction;
  wl_debug.chassis_posture_valid = static_cast<uint8_t>(chassis_control_output.posture_valid);
  wl_debug.chassis_standup_complete = static_cast<uint8_t>(chassis_control_output.standup_complete);
  wl_debug.chassis_standup_phase = chassis_control_output.standup_phase;
  wl_debug.chassis_standup_theta_target_rad = chassis_control_output.standup_theta_target;

  // ── 输入语义（便于调试时定位遥控器/状态机决策根因）──
  wl_debug.input_domain_request = static_cast<uint8_t>(input.mode_request.domain_request);
  wl_debug.input_leg_profile = static_cast<uint8_t>(input.mode_request.leg_request);
  wl_debug.input_combat_profile = static_cast<uint8_t>(input.mode_request.combat_profile);
  wl_debug.input_target_source = static_cast<uint8_t>(input.mode_request.target_source);
  wl_debug.input_host_target_valid = static_cast<uint8_t>(input.mode_request.host_target_valid);
  wl_debug.gimbal_target_yaw_rad = gimbal_control_output.yaw_target_rad;
  wl_debug.gimbal_target_pitch_rad = -gimbal_control_output.pitch_target_rad;

  // ── AI Policy 网络观测输入 ──
  const auto &p = wheel_legged::ai::ai_policy_debug;
  // base_ang_vel * 0.25
  wl_debug.policy_obs_gyro_x = p.observations[0];
  wl_debug.policy_obs_gyro_y = p.observations[1];
  wl_debug.policy_obs_gyro_z = p.observations[2];
  // projected_gravity
  wl_debug.policy_obs_gravity_x = p.observations[3];
  wl_debug.policy_obs_gravity_y = p.observations[4];
  wl_debug.policy_obs_gravity_z = p.observations[5];
  // command
  wl_debug.policy_obs_cmd_vx = p.observations[6];
  wl_debug.policy_obs_cmd_yaw = p.observations[7];
  wl_debug.policy_obs_cmd_height = p.observations[8];
  // leg angle
  wl_debug.policy_obs_theta_ll = p.observations[9];
  wl_debug.policy_obs_theta_lr = p.observations[10];
  // leg angle dot * 0.05
  wl_debug.policy_obs_theta_dot_ll = p.observations[11];
  wl_debug.policy_obs_theta_dot_lr = p.observations[12];
  // leg length * 5.0
  wl_debug.policy_obs_l_l = p.observations[13];
  wl_debug.policy_obs_l_r = p.observations[14];
  // leg length dot * 0.25
  wl_debug.policy_obs_l_dot_l = p.observations[15];
  wl_debug.policy_obs_l_dot_r = p.observations[16];
  // wheel pos
  wl_debug.policy_obs_wheel_pos_l = p.observations[17];
  wl_debug.policy_obs_wheel_pos_r = p.observations[18];
  // wheel vel * 0.05
  wl_debug.policy_obs_wheel_vel_l = p.observations[19];
  wl_debug.policy_obs_wheel_vel_r = p.observations[20];
  // last action
  wl_debug.policy_obs_prev_a_theta_l = p.observations[21];
  wl_debug.policy_obs_prev_a_l0_l = p.observations[22];
  wl_debug.policy_obs_prev_a_wheel_l = p.observations[23];
  wl_debug.policy_obs_prev_a_theta_r = p.observations[24];
  wl_debug.policy_obs_prev_a_l0_r = p.observations[25];
  wl_debug.policy_obs_prev_a_wheel_r = p.observations[26];

  // ── AI Policy 网络动作输出 ──
  wl_debug.policy_act_theta_l = p.actions[0];
  wl_debug.policy_act_l0_l = p.actions[1];
  wl_debug.policy_act_wheel_l = p.actions[2];
  wl_debug.policy_act_theta_r = p.actions[3];
  wl_debug.policy_act_l0_r = p.actions[4];
  wl_debug.policy_act_wheel_r = p.actions[5];

  // ── AI Policy 网络动作输出 (物理单位转换) ──
  constexpr float kL0Min = 0.12192586f;
  constexpr float kL0Max = 0.30063868f;
  auto clamp_l0 = [](float v) { return v < kL0Min ? kL0Min : (v > kL0Max ? kL0Max : v); };
  wl_debug.policy_act_theta_l_rad = p.actions[0] * 0.5f;
  wl_debug.policy_act_theta_r_rad = p.actions[3] * 0.5f;
  wl_debug.policy_act_l0_l_m = clamp_l0(p.actions[1] * 0.1f + 0.17f);
  wl_debug.policy_act_l0_r_m = clamp_l0(p.actions[4] * 0.1f + 0.17f);
  wl_debug.policy_act_wheel_l_rad_s = p.actions[2] * 52.0f;
  wl_debug.policy_act_wheel_r_rad_s = p.actions[5] * 52.0f;

  // ── AI Policy 推理状态 ──
  wl_debug.policy_infer_us = p.last_infer_us;
  wl_debug.policy_step_count = p.step_count;
  wl_debug.policy_ok = static_cast<uint8_t>(p.ok);
}
