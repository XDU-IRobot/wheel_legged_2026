/**
 * @file  targets/wheel_legged/debug.cc
 * @brief DebugSnapshot 填充实现
 */

#include "include/debug.hpp"
#include "include/ai/policy_runner.hpp"
#include "common/device/vl53l4cd.hpp"

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

  wl_debug.vl53l4cd_driver_status = ::device::g_vl53l4cd_state.driver_status;
  wl_debug.vl53l4cd_range_status = ::device::g_vl53l4cd_state.range_status;
  wl_debug.vl53l4cd_model_id = ::device::g_vl53l4cd_state.model_id;
  wl_debug.vl53l4cd_distance_mm = ::device::g_vl53l4cd_state.distance_mm;
  wl_debug.vl53l4cd_signal_kcps = ::device::g_vl53l4cd_state.signal_rate_kcps;
  wl_debug.vl53l4cd_ambient_kcps = ::device::g_vl53l4cd_state.ambient_rate_kcps;
  wl_debug.vl53l4cd_sigma_mm = ::device::g_vl53l4cd_state.sigma_mm;
  wl_debug.vl53l4cd_sample_count = ::device::g_vl53l4cd_state.sample_count;

  // ── DR16 原始输入 ──
  wl_debug.dr16_online = static_cast<uint8_t>(input.dr16.online);
  wl_debug.dr16_switch_l_raw = static_cast<int32_t>(input.dr16.switch_l);
  wl_debug.dr16_switch_r_raw = static_cast<int32_t>(input.dr16.switch_r);
  wl_debug.dr16_dial_raw = input.dr16.dial;
  wl_debug.dr16_mouse_x = input.dr16.mouse_x;
  wl_debug.dr16_mouse_y = input.dr16.mouse_y;
  wl_debug.dr16_mouse_left = static_cast<uint8_t>(input.dr16.mouse_left);
  wl_debug.dr16_mouse_right = static_cast<uint8_t>(input.dr16.mouse_right);
  wl_debug.dr16_keyboard = input.dr16.keyboard;

  // ── DR16 语义请求 ──
  wl_debug.dr16_enable_request = static_cast<uint8_t>(
      input.mode_request.input_valid && input.mode_request.domain_request != wheel_legged::DomainRequest::kDisabled);
  wl_debug.dr16_spin_request = static_cast<uint8_t>(input.mode_request.spin_hold);
  wl_debug.dr16_jump_trigger_edge = static_cast<uint8_t>(input.mode_request.jump_trigger);
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

  wl_debug.dr16_left_x_raw = input.dr16.left_x;
  wl_debug.dr16_left_y_raw = input.dr16.left_y;
  wl_debug.dr16_right_x_raw = input.dr16.right_x;
  wl_debug.dr16_right_y_raw = input.dr16.right_y;

  // ── 底盘状态 ──
  wl_debug.chassis_leg_target_length_m = chassis_control_output.leg_target_length_m;
  wl_debug.chassis_mean_leg_length_m = chassis_control_output.mean_leg_length_m;
  wl_debug.chassis_speed_mps = chassis_control_output.speed_mps;
  wl_debug.chassis_raw_wheel_speed_mps = chassis_control_output.raw_wheel_speed_mps;
  wl_debug.chassis_raw_accel_speed_mps = chassis_control_output.raw_accel_speed_mps;
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
  const auto &mpc = chassis_control_output.roll_leg_mpc_shadow;
  wl_debug.mpc_active = static_cast<uint8_t>(mpc.active);
  wl_debug.mpc_solved = static_cast<uint8_t>(mpc.solved);
  wl_debug.mpc_fallback_reason = static_cast<uint8_t>(mpc.fallback_reason);
  wl_debug.mpc_solver_iterations =
      static_cast<uint8_t>(mpc.solver_iterations > 255 ? 255 : (mpc.solver_iterations < 0 ? 0 : mpc.solver_iterations));
  wl_debug.mpc_left_force_n = mpc.left_force_n;
  wl_debug.mpc_right_force_n = mpc.right_force_n;
  wl_debug.mpc_dF_left_n = mpc.dF_left_n;
  wl_debug.mpc_dF_right_n = mpc.dF_right_n;
  wl_debug.mpc_gravity_left_n = mpc.gravity_left_n;
  wl_debug.mpc_gravity_right_n = mpc.gravity_right_n;
  wl_debug.mpc_e_L = mpc.e_L;
  wl_debug.mpc_D = mpc.D;
  wl_debug.mpc_e_roll = mpc.e_roll;
  wl_debug.mpc_a_y = mpc.a_y;
  wl_debug.mpc_model_h_m = mpc.model_com_height_m;
  wl_debug.mpc_model_I_roll = mpc.model_roll_inertia_kg_m2;

  // ── 云台反馈与电机 ──
  wl_debug.yaw_cmd_target_rad = gimbal_control_output.yaw_target_rad;
  wl_debug.yaw_motor_raw_pos_rad = input.estimator_input.yaw_motor_rad;
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
  wl_debug.chassis_posture_valid = static_cast<uint8_t>(chassis_control_output.posture_valid);

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
