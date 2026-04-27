#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file  task/runtime/task_debug.hpp
 * @brief 任务调试快照（全局变量，便于 FreeRTOS 在线观察）
 */

typedef struct {
  uint32_t tick_ms;
  uint16_t seq;
  uint16_t update_count;
  uint8_t link_ok;
  uint8_t force_enable;
  uint8_t small_gyro_enable;
  uint8_t leg_length_mode;
  uint8_t jump_trigger;
  float vx_cmd;
  float yaw_cmd;
} CommTaskDebugSnapshot;

typedef struct {
  uint32_t tick_ms;
  uint16_t seq;
  uint16_t update_count;
  uint8_t input_valid;
  uint8_t force_enable;
  uint8_t leg_length_mode;
  uint8_t mode;
  uint8_t state_changed;
  uint8_t run_chassis_update;
  uint8_t enable_dm;
  uint8_t jump_phase;
  uint8_t reserved0;
  float target_leg_length_m;
} FsmTaskDebugSnapshot;

typedef struct {
  uint32_t tick_ms;
  uint16_t seq;
  uint16_t update_count;
  uint8_t has_comm;
  uint8_t has_fsm;
  uint8_t has_motor_feedback;
  uint8_t run_compute;
  uint8_t output_valid;
  uint8_t enable_dm;
  uint8_t state_valid;
  uint8_t imu_valid;
  float target_leg_length_m;
  float lf_tau;
  float lb_tau;
  float rf_tau;
  float rb_tau;
  float lw_tau;
  float rw_tau;
  // Current state vector snapshot from chassis estimator.
  float s;
  float s_dot;
  float phi;
  float phi_dot;
  float theta_ll;
  float theta_ll_dot;
  float theta_lr;
  float theta_lr_dot;
  float theta_b;
  float theta_b_dot;
  float l_l;
  float l_r;
  // IMU snapshot used by torque estimator input.
  float imu_roll_rad;
  float imu_pitch_rad;
  float imu_gyro_y_rad_s;
  float imu_gyro_z_rad_s;
  float imu_acc_x_mps2;
  float imu_acc_y_mps2;
  float imu_acc_z_mps2;
  float imu_yaw_motor_rad;
  float left_Fn;
  float right_Fn;
  float raw_wheel_speed_mps;
  float raw_accel_speed_mps;
  float current_speed_mps;
  // Desired state vector snapshot used by the controller.
  float target_s;
  float target_s_dot;
  float target_phi;
  float target_phi_dot;
  float target_theta_ll;
  float target_theta_ll_dot;
  float target_theta_lr;
  float target_theta_lr_dot;
  float target_theta_b;
  float target_theta_b_dot;
  float theta_b_outer_comp;
} TorqueTaskDebugSnapshot;

typedef struct {
  uint32_t tick_ms;
  uint16_t seq;
  uint16_t update_count;
  uint8_t has_cmd;
  uint8_t timeout;
  uint8_t cmd_valid;
  uint8_t req_enable_dm;
  uint8_t dm_enabled_latched;
  uint8_t reserved0;
  uint8_t reserved1;
  uint8_t reserved2;
  float lf_tau;
  float lb_tau;
  float rf_tau;
  float rb_tau;
  float lw_tau;
  float rw_tau;
} MotorTaskDebugSnapshot;

typedef struct {
  uint8_t valid;
  uint8_t reserved0;
  uint8_t reserved1;
  uint8_t reserved2;
  float left_leg_front_pos;
  float left_leg_front_vel;
  float left_leg_front_tau;
  float left_leg_back_pos;
  float left_leg_back_vel;
  float left_leg_back_tau;
  float right_leg_front_pos;
  float right_leg_front_vel;
  float right_leg_front_tau;
  float right_leg_back_pos;
  float right_leg_back_vel;
  float right_leg_back_tau;
  float wheel_left_rad_s;
  float wheel_right_rad_s;
} MotorFeedbackDebugSnapshot;

typedef struct {
  CommTaskDebugSnapshot comm;
  FsmTaskDebugSnapshot fsm;
  TorqueTaskDebugSnapshot torque;
  MotorTaskDebugSnapshot motor;
  MotorFeedbackDebugSnapshot motor_feedback;
} TaskDebugSnapshot;

// 全局调试快照，调试器里直接 watch: g_task_debug_snapshot
extern TaskDebugSnapshot g_task_debug_snapshot;

void TaskDebugReset(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

#include "task/messages/control_messages.hpp"

namespace tasking {

void DebugUpdateComm(const CommInputMsg &msg);
void DebugUpdateFsm(const Fsm::Input &input, const Fsm::Output &out, const FsmOutputMsg &msg);
void DebugUpdateTorque(const TorqueCmd6 &msg, bool has_comm, bool has_fsm, bool has_motor_feedback, bool run_compute,
                       float target_leg_length_m, const wbr::CurrentState *current_state,
                       const wbr::ExpectedState *expected_state, float theta_b_outer_comp, bool imu_valid,
                       float imu_roll_rad, float imu_pitch_rad, float imu_gyro_y_rad_s, float imu_gyro_z_rad_s,
                       float imu_acc_x_mps2, float imu_acc_y_mps2, float imu_acc_z_mps2, float imu_yaw_motor_rad,
                       float left_Fn, float right_Fn, float raw_wheel_speed_mps, float raw_accel_speed_mps,
                       float current_speed_mps);
void DebugUpdateMotor(bool has_cmd, bool timeout, const TorqueCmd6 &msg, bool dm_enabled_latched);
void DebugUpdateMotor(const MotorFeedbackMsg &msg);

}  // namespace tasking

#endif
