/**
 * @file  targets/wheel_legged/control_loop/debug_export.cc
 * @brief DebugSnapshot 填充实现
 */

#include "debug_export.hpp"

#include "../include/debug_snapshot.hpp"

void UpdateDebugSnapshot(const uint32_t tick_ms, const wheel_legged::control_loop::InputSnapshot &input,
                         const chassis::Fsm::Output &chassis_output, const gimbal::Fsm::Output &gimbal_output,
                         const chassis::Chassis::UpdateOutput &chassis_control_output,
                         const gimbal::Gimbal::UpdateOutput &gimbal_control_output) {
  // ── 时间戳与状态机 ──
  wl_debug.tick_ms = tick_ms;
  wl_debug.chassis_fsm_state = static_cast<uint8_t>(chassis_output.mode);
  wl_debug.gimbal_fsm_state = static_cast<uint8_t>(gimbal_output.mode);
  wl_debug.chassis_fsm_state_changed = static_cast<uint8_t>(chassis_output.state_changed);
  wl_debug.gimbal_fsm_state_changed = static_cast<uint8_t>(gimbal_output.state_changed);

  // ── DR16 原始输入 ──
  wl_debug.dr16_online = static_cast<uint8_t>(input.dr16.online);
  wl_debug.dr16_switch_l_raw = static_cast<int32_t>(input.dr16.switch_l);
  wl_debug.dr16_switch_r_raw = static_cast<int32_t>(input.dr16.switch_r);
  wl_debug.dr16_dial_raw = input.dr16.dial;

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

  wl_debug.dr16_left_x_raw = input.dr16.left_x;
  wl_debug.dr16_left_y_raw = input.dr16.left_y;
  wl_debug.dr16_right_x_raw = input.dr16.right_x;
  wl_debug.dr16_right_y_raw = input.dr16.right_y;

  // ── 底盘状态 ──
  wl_debug.chassis_mean_leg_length_m = chassis_control_output.mean_leg_length_m;
  wl_debug.chassis_speed_mps = chassis_control_output.speed_mps;
  wl_debug.chassis_left_force_n = chassis_control_output.left_force_n;
  wl_debug.chassis_right_force_n = chassis_control_output.right_force_n;
  wl_debug.chassis_left_support_force_n = chassis_control_output.left_support_force_n;
  wl_debug.chassis_right_support_force_n = chassis_control_output.right_support_force_n;
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
  wl_debug.chassis_posture_valid = static_cast<uint8_t>(chassis_control_output.posture_valid);

  // ── 输入语义（便于调试时定位遥控器/状态机决策根因）──
  wl_debug.input_domain_request = static_cast<uint8_t>(input.mode_request.domain_request);
  wl_debug.input_leg_profile = static_cast<uint8_t>(input.mode_request.leg_request);
  wl_debug.input_combat_profile = static_cast<uint8_t>(input.mode_request.combat_profile);
  wl_debug.input_target_source = static_cast<uint8_t>(input.mode_request.target_source);
  wl_debug.input_host_target_valid = static_cast<uint8_t>(input.mode_request.host_target_valid);
  wl_debug.gimbal_target_yaw_rad = gimbal_control_output.yaw_target_rad;
  wl_debug.gimbal_target_pitch_rad = gimbal_control_output.pitch_target_rad;
}
