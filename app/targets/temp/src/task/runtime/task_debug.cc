#include "task/runtime/task_debug.hpp"

#include "cmsis_os.h"

#include <cstring>

namespace {

inline uint8_t B2U8(bool value) { return value ? 1U : 0U; }

} // namespace

extern "C" {

TaskDebugSnapshot g_task_debug_snapshot = {};

void TaskDebugReset(void) {
  std::memset(&g_task_debug_snapshot, 0, sizeof(g_task_debug_snapshot));
}

} // extern "C"

namespace tasking {

void DebugUpdateComm(const CommInputMsg &msg) {
  volatile CommTaskDebugSnapshot &dbg = g_task_debug_snapshot.comm;
  dbg.tick_ms = msg.h.tick_ms;
  dbg.seq = msg.h.seq;
  dbg.update_count = static_cast<uint16_t>(dbg.update_count + 1U);
  dbg.link_ok = B2U8(msg.link_ok);
  dbg.force_enable = B2U8(msg.force_enable);
  dbg.small_gyro_enable = B2U8(msg.small_gyro_enable);
  dbg.leg_length_mode = static_cast<uint8_t>(msg.leg_length_mode);
  dbg.jump_trigger = B2U8(msg.jump_trigger);
  dbg.vx_cmd = msg.vx_cmd;
  dbg.yaw_cmd = msg.yaw_cmd;
}

void DebugUpdateFsm(const Fsm::Input &input, const Fsm::Output &out,
                    const FsmOutputMsg &msg) {
  volatile FsmTaskDebugSnapshot &dbg = g_task_debug_snapshot.fsm;
  dbg.tick_ms = msg.h.tick_ms;
  dbg.seq = msg.h.seq;
  dbg.update_count = static_cast<uint16_t>(dbg.update_count + 1U);
  dbg.input_valid = B2U8(input.input_valid);
  dbg.force_enable = B2U8(input.force_enable);
  dbg.leg_length_mode = static_cast<uint8_t>(input.leg_length_mode);
  dbg.mode = static_cast<uint8_t>(out.mode);
  dbg.state_changed = B2U8(out.state_changed);
  dbg.run_chassis_update = B2U8(out.control.run_chassis_update);
  dbg.enable_dm = B2U8(out.control.enable_dm);
  dbg.jump_phase = out.control.jump_phase;
  dbg.target_leg_length_m = out.control.target_leg_length_m;
}

void DebugUpdateTorque(const TorqueCmd6 &msg, bool has_comm, bool has_fsm,
                       bool has_motor_feedback, bool run_compute,
                       float target_leg_length_m,
                       const wbr::CurrentState *current_state,
                       const wbr::ExpectedState *expected_state,
                       float theta_b_outer_comp, bool imu_valid,
                       float imu_roll_rad, float imu_pitch_rad,
                       float imu_gyro_y_rad_s, float imu_gyro_z_rad_s,
                       float imu_acc_x_mps2, float imu_acc_y_mps2,
                       float imu_acc_z_mps2, float imu_yaw_motor_rad,
                       float left_Fn, float right_Fn,
                       float raw_wheel_speed_mps,
                       float raw_accel_speed_mps,
                       float current_speed_mps) {
  g_task_debug_snapshot.torque.tick_ms = msg.h.tick_ms;
  g_task_debug_snapshot.torque.seq = msg.h.seq;
  g_task_debug_snapshot.torque.update_count =
      static_cast<uint16_t>(g_task_debug_snapshot.torque.update_count + 1U);
  g_task_debug_snapshot.torque.has_comm = B2U8(has_comm);
  g_task_debug_snapshot.torque.has_fsm = B2U8(has_fsm);
  g_task_debug_snapshot.torque.has_motor_feedback = B2U8(has_motor_feedback);
  g_task_debug_snapshot.torque.run_compute = B2U8(run_compute);
  g_task_debug_snapshot.torque.output_valid = B2U8(msg.valid);
  g_task_debug_snapshot.torque.enable_dm = B2U8(msg.enable_dm);
  g_task_debug_snapshot.torque.state_valid = B2U8(current_state != nullptr);
  g_task_debug_snapshot.torque.imu_valid = B2U8(imu_valid);
  g_task_debug_snapshot.torque.target_leg_length_m = target_leg_length_m;
  g_task_debug_snapshot.torque.theta_b_outer_comp = theta_b_outer_comp;
  g_task_debug_snapshot.torque.lf_tau = msg.lf_tau;
  g_task_debug_snapshot.torque.lb_tau = msg.lb_tau;
  g_task_debug_snapshot.torque.rf_tau = msg.rf_tau;
  g_task_debug_snapshot.torque.rb_tau = msg.rb_tau;
  g_task_debug_snapshot.torque.lw_tau = msg.lw_tau;
  g_task_debug_snapshot.torque.rw_tau = msg.rw_tau;
  g_task_debug_snapshot.torque.left_Fn = left_Fn;
  g_task_debug_snapshot.torque.right_Fn = right_Fn;
  g_task_debug_snapshot.torque.raw_wheel_speed_mps = raw_wheel_speed_mps;
  g_task_debug_snapshot.torque.raw_accel_speed_mps = raw_accel_speed_mps;
  g_task_debug_snapshot.torque.current_speed_mps = current_speed_mps;
  if (current_state != nullptr) {
    g_task_debug_snapshot.torque.s = current_state->s;
    g_task_debug_snapshot.torque.s_dot = current_state->s_dot;
    g_task_debug_snapshot.torque.phi = current_state->phi;
    g_task_debug_snapshot.torque.phi_dot = current_state->phi_dot;
    g_task_debug_snapshot.torque.theta_ll = current_state->theta_ll;
    g_task_debug_snapshot.torque.theta_ll_dot = current_state->theta_ll_dot;
    g_task_debug_snapshot.torque.theta_lr = current_state->theta_lr;
    g_task_debug_snapshot.torque.theta_lr_dot = current_state->theta_lr_dot;
    g_task_debug_snapshot.torque.theta_b = current_state->theta_b;
    g_task_debug_snapshot.torque.theta_b_dot = current_state->theta_b_dot;
    g_task_debug_snapshot.torque.l_l = current_state->l_l;
    g_task_debug_snapshot.torque.l_r = current_state->l_r;
  } else {
    g_task_debug_snapshot.torque.s = 0.0f;
    g_task_debug_snapshot.torque.s_dot = 0.0f;
    g_task_debug_snapshot.torque.phi = 0.0f;
    g_task_debug_snapshot.torque.phi_dot = 0.0f;
    g_task_debug_snapshot.torque.theta_ll = 0.0f;
    g_task_debug_snapshot.torque.theta_ll_dot = 0.0f;
    g_task_debug_snapshot.torque.theta_lr = 0.0f;
    g_task_debug_snapshot.torque.theta_lr_dot = 0.0f;
    g_task_debug_snapshot.torque.theta_b = 0.0f;
    g_task_debug_snapshot.torque.theta_b_dot = 0.0f;
    g_task_debug_snapshot.torque.l_l = 0.0f;
    g_task_debug_snapshot.torque.l_r = 0.0f;
    g_task_debug_snapshot.torque.current_speed_mps = 0.0f;
  }

  if (expected_state != nullptr) {
    g_task_debug_snapshot.torque.target_s = expected_state->s;
    g_task_debug_snapshot.torque.target_s_dot = expected_state->s_dot;
    g_task_debug_snapshot.torque.target_phi = expected_state->phi;
    g_task_debug_snapshot.torque.target_phi_dot = expected_state->phi_dot;
    g_task_debug_snapshot.torque.target_theta_ll = expected_state->theta_ll;
    g_task_debug_snapshot.torque.target_theta_ll_dot =
        expected_state->theta_ll_dot;
    g_task_debug_snapshot.torque.target_theta_lr = expected_state->theta_lr;
    g_task_debug_snapshot.torque.target_theta_lr_dot =
        expected_state->theta_lr_dot;
    g_task_debug_snapshot.torque.target_theta_b = expected_state->theta_b;
    g_task_debug_snapshot.torque.target_theta_b_dot =
        expected_state->theta_b_dot;
  } else {
    g_task_debug_snapshot.torque.target_s = 0.0f;
    g_task_debug_snapshot.torque.target_s_dot = 0.0f;
    g_task_debug_snapshot.torque.target_phi = 0.0f;
    g_task_debug_snapshot.torque.target_phi_dot = 0.0f;
    g_task_debug_snapshot.torque.target_theta_ll = 0.0f;
    g_task_debug_snapshot.torque.target_theta_ll_dot = 0.0f;
    g_task_debug_snapshot.torque.target_theta_lr = 0.0f;
    g_task_debug_snapshot.torque.target_theta_lr_dot = 0.0f;
    g_task_debug_snapshot.torque.target_theta_b = 0.0f;
    g_task_debug_snapshot.torque.target_theta_b_dot = 0.0f;
  }

  if (imu_valid) {
    g_task_debug_snapshot.torque.imu_roll_rad = imu_roll_rad;
    g_task_debug_snapshot.torque.imu_pitch_rad = imu_pitch_rad;
    g_task_debug_snapshot.torque.imu_gyro_y_rad_s = imu_gyro_y_rad_s;
    g_task_debug_snapshot.torque.imu_gyro_z_rad_s = imu_gyro_z_rad_s;
    g_task_debug_snapshot.torque.imu_acc_x_mps2 = imu_acc_x_mps2;
    g_task_debug_snapshot.torque.imu_acc_y_mps2 = imu_acc_y_mps2;
    g_task_debug_snapshot.torque.imu_acc_z_mps2 = imu_acc_z_mps2;
    g_task_debug_snapshot.torque.imu_yaw_motor_rad = imu_yaw_motor_rad;
  } else {
    g_task_debug_snapshot.torque.imu_roll_rad = 0.0f;
    g_task_debug_snapshot.torque.imu_pitch_rad = 0.0f;
    g_task_debug_snapshot.torque.imu_gyro_y_rad_s = 0.0f;
    g_task_debug_snapshot.torque.imu_gyro_z_rad_s = 0.0f;
    g_task_debug_snapshot.torque.imu_acc_x_mps2 = 0.0f;
    g_task_debug_snapshot.torque.imu_acc_y_mps2 = 0.0f;
    g_task_debug_snapshot.torque.imu_acc_z_mps2 = 0.0f;
    g_task_debug_snapshot.torque.imu_yaw_motor_rad = 0.0f;
  }

}

void DebugUpdateMotor(bool has_cmd, bool timeout, const TorqueCmd6 &msg,
                      bool dm_enabled_latched) {
  volatile MotorTaskDebugSnapshot &dbg = g_task_debug_snapshot.motor;
  dbg.tick_ms = osKernelGetTickCount();
  dbg.seq = msg.h.seq;
  dbg.update_count = static_cast<uint16_t>(dbg.update_count + 1U);
  dbg.has_cmd = B2U8(has_cmd);
  dbg.timeout = B2U8(timeout);
  dbg.cmd_valid = B2U8(msg.valid);
  dbg.req_enable_dm = B2U8(msg.enable_dm);
  dbg.dm_enabled_latched = B2U8(dm_enabled_latched);
  dbg.lf_tau = msg.lf_tau;
  dbg.lb_tau = msg.lb_tau;
  dbg.rf_tau = msg.rf_tau;
  dbg.rb_tau = msg.rb_tau;
  dbg.lw_tau = msg.lw_tau;
  dbg.rw_tau = msg.rw_tau;
}

void DebugUpdateMotor(const MotorFeedbackMsg &msg) {
  volatile MotorFeedbackDebugSnapshot &dbg = g_task_debug_snapshot.motor_feedback;
  dbg.valid = B2U8(msg.valid);
  dbg.left_leg_front_pos = msg.left_leg.front.pos_rad;
  dbg.left_leg_front_vel = msg.left_leg.front.vel_rad_s;
  dbg.left_leg_front_tau = msg.left_leg.front.torque_nm;
  dbg.left_leg_back_pos = msg.left_leg.back.pos_rad;
  dbg.left_leg_back_vel = msg.left_leg.back.vel_rad_s;
  dbg.left_leg_back_tau = msg.left_leg.back.torque_nm;
  dbg.right_leg_front_pos = msg.right_leg.front.pos_rad;
  dbg.right_leg_front_vel = msg.right_leg.front.vel_rad_s;
  dbg.right_leg_front_tau = msg.right_leg.front.torque_nm;
  dbg.right_leg_back_pos = msg.right_leg.back.pos_rad;
  dbg.right_leg_back_vel = msg.right_leg.back.vel_rad_s;
  dbg.right_leg_back_tau = msg.right_leg.back.torque_nm;
  dbg.wheel_left_rad_s = msg.wheel.left_rad_s;
  dbg.wheel_right_rad_s = msg.wheel.right_rad_s;
}

} // namespace tasking

