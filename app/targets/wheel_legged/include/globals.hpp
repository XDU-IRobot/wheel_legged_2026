#pragma once

#include <cstdint>
#include <optional>

#include "fdcan.h"

#include <librm.hpp>

#include "globals_no_dtcm.hpp"

#include "chassis/chassis.hpp"
#include "chassis/fsm.hpp"
#include "gimbal_can_feedback_rx_bridge.hpp"
#include "gimbal/fsm.hpp"
#include "gimbal/gimbal.hpp"
#include "librm/device/remote/dr16.hpp"

/**
 * @file  targets/wheel_legged/include/globals.hpp
 * @brief 轮腿目标共享资源与调试导出变量
 */

/**
 * @brief 任务间共享的设备、控制器与状态机对象
 * @note  与硬件相关的成员在 Init() 中延迟构造，避免静态初始化阶段硬件访问异常
 */
struct SharedResources {
  using DmMitMotor = rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>;

  static constexpr double kJointCanTxLimitHz = 4000.0;
  static constexpr double kWheelCanTxLimitHz = 4000.0;
  static constexpr double kGimbalCanTxLimitHz = 4000.0;
  static constexpr float kPi = 3.14159265358979323846f;

  SharedResourcesNoDtcm *no_dtcm{&globals_no_dtcm};

  rm::device::DR16 dr16{no_dtcm->rc_uart};

  // 保持值语义成员，实际构造延后到 Init()。
  std::optional<rm::hal::ThrottledCan<>> joint_can{};
  std::optional<rm::hal::ThrottledCan<>> wheel_can{};
  std::optional<rm::hal::ThrottledCan<>> gimbal_can{};
  std::optional<DmMitMotor> dm_lf{};
  std::optional<DmMitMotor> dm_lb{};
  std::optional<DmMitMotor> dm_rf{};
  std::optional<DmMitMotor> dm_rb{};
  std::optional<rm::device::M3508> left_wheel{};
  std::optional<rm::device::M3508> right_wheel{};
  std::optional<rm::device::HipnucImu> chassis_imu{};
  std::optional<GimbalCanFeedbackRxBridge> gimbal_imu_feedback_rx{};

  std::optional<DmMitMotor> yaw_motor{};
  std::optional<DmMitMotor> pitch_motor{};

  chassis::Fsm chassis_fsm{};
  chassis::Chassis chassis{};
  gimbal::Fsm gimbal_fsm{};
  gimbal::Gimbal gimbal{};

  static SharedResources &GetInstance() {
    static SharedResources *shared_resources_instance{nullptr};

    if (shared_resources_instance == nullptr) {
      shared_resources_instance = new SharedResources;
      shared_resources_instance->Init();
    }

    return *shared_resources_instance;
  }

  void Init() {
    dr16.Begin();

    if (!joint_can.has_value()) {
      joint_can.emplace(hfdcan1, kJointCanTxLimitHz);
      joint_can->SetFilter(0, 0);
      joint_can->Begin();
    }
    if (!wheel_can.has_value()) {
      wheel_can.emplace(hfdcan2, kWheelCanTxLimitHz);
      wheel_can->SetFilter(0, 0);
      wheel_can->Begin();
    }
    if (!gimbal_can.has_value()) {
      gimbal_can.emplace(hfdcan3, kGimbalCanTxLimitHz);
      gimbal_can->SetFilter(0, 0);
      gimbal_can->Begin();
    }
    if (!gimbal_imu_feedback_rx.has_value()) {
      gimbal_imu_feedback_rx.emplace(*gimbal_can);
    }

    if (!dm_lf.has_value()) {
      dm_lf.emplace(*joint_can, rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{
                                    0x17, 0x07, kPi, 45.0f, 54.0f, {0, 500}, {0, 10}});
      dm_lb.emplace(*joint_can, rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{
                                    0x14, 0x04, kPi, 45.0f, 54.0f, {0, 500}, {0, 10}});
      dm_rf.emplace(*joint_can, rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{
                                    0x16, 0x06, kPi, 45.0f, 54.0f, {0, 500}, {0, 10}});
      dm_rb.emplace(*joint_can, rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>{
                                    0x15, 0x05, kPi, 45.0f, 54.0f, {0, 500}, {0, 10}});
    }

    if (!left_wheel.has_value()) {
      left_wheel.emplace(*wheel_can, 0x06);
      right_wheel.emplace(*wheel_can, 0x05);
    }

    if (!pitch_motor.has_value()) {
      pitch_motor.emplace(*gimbal_can, gimbal::Gimbal::kPitchMotorSettings);
    }

    if (!yaw_motor.has_value()) {
      yaw_motor.emplace(*wheel_can, gimbal::Gimbal::kYawMotorSettings);
    }

    if (!chassis_imu.has_value()) {
      chassis_imu.emplace(no_dtcm->imu_uart);
      chassis_imu->Begin();
    }

    chassis_fsm.Init();
    chassis.Init();
    gimbal_fsm.Init();
    gimbal.Init();
  }
};

extern SharedResources *globals;

extern "C" {
/** @brief 调试导出变量，供上位机与调试器读取 */
extern volatile uint32_t wl_fm_tick_ms;

/** @brief 底盘/云台状态机模式编号 */
extern volatile uint8_t wl_fm_chassis_mode;
extern volatile uint8_t wl_fm_gimbal_mode;

/** @brief 底盘/云台状态机本周期是否发生状态变化 */
extern volatile uint8_t wl_fm_chassis_state_changed;
extern volatile uint8_t wl_fm_gimbal_state_changed;
extern volatile char wl_fm_chassis_mode_text[32];
extern volatile char wl_fm_gimbal_mode_text[32];

/** @brief 遥控器在线与档位信息 */
extern volatile uint8_t wl_fm_dr16_online;
extern volatile int32_t wl_fm_dr16_switch_l;
extern volatile int32_t wl_fm_dr16_switch_r;
extern volatile int16_t wl_fm_dr16_dial;

/** @brief 遥控器语义化请求 */
extern volatile uint8_t wl_fm_dr16_enable_request;
extern volatile uint8_t wl_fm_dr16_spin_request;
extern volatile uint8_t wl_fm_dr16_jump_trigger_edge;

/** @brief 系统时序测量 */
extern volatile float wl_fm_can_loop_freq_hz;
extern volatile float wl_fm_joint_can_fps;
extern volatile float wl_fm_wheel_can_fps;
extern volatile float wl_fm_timer_period_us;

/** @brief 底盘核心观测量 */
extern volatile float wl_fm_chassis_leg_length_m;
extern volatile float wl_fm_left_leg_length_m;
extern volatile float wl_fm_right_leg_length_m;
extern volatile float wl_fm_chassis_speed_mps;

/** @brief 关节与轮毂原始反馈 */
extern volatile float wl_fm_motor_lf_pos_rad;
extern volatile float wl_fm_motor_lf_vel_rad_s;
extern volatile float wl_fm_motor_lf_tau_nm;
extern volatile float wl_fm_motor_lb_pos_rad;
extern volatile float wl_fm_motor_lb_vel_rad_s;
extern volatile float wl_fm_motor_lb_tau_nm;
extern volatile float wl_fm_motor_rf_pos_rad;
extern volatile float wl_fm_motor_rf_vel_rad_s;
extern volatile float wl_fm_motor_rf_tau_nm;
extern volatile float wl_fm_motor_rb_pos_rad;
extern volatile float wl_fm_motor_rb_vel_rad_s;
extern volatile float wl_fm_motor_rb_tau_nm;
extern volatile float wl_fm_wheel_left_rad_s;
extern volatile float wl_fm_wheel_right_rad_s;
extern volatile float wl_fm_wheel_left_tau_nm;
extern volatile float wl_fm_wheel_right_tau_nm;

/** @brief 关节与轮毂控制输出力矩 */
extern volatile float wl_fm_motor_lf_out_tau_nm;
extern volatile float wl_fm_motor_lb_out_tau_nm;
extern volatile float wl_fm_motor_rf_out_tau_nm;
extern volatile float wl_fm_motor_rb_out_tau_nm;

/** @brief 惯导原始反馈 */
extern volatile float wl_fm_imu_roll_rad;
extern volatile float wl_fm_imu_pitch_rad;
extern volatile float wl_fm_imu_yaw_rad;
extern volatile float wl_fm_imu_gyro_x_rad_s;
extern volatile float wl_fm_imu_gyro_y_rad_s;
extern volatile float wl_fm_imu_gyro_z_rad_s;
extern volatile float wl_fm_imu_acc_x_mps2;
extern volatile float wl_fm_imu_acc_y_mps2;
extern volatile float wl_fm_imu_acc_z_mps2;
extern volatile float wl_fm_gimbal_imu_pitch_rad;
extern volatile float wl_fm_gimbal_imu_yaw_rad;

/** @brief 模型状态向量导出 */
extern volatile float wl_fm_model_s_m;
extern volatile float wl_fm_model_s_dot_mps;
extern volatile float wl_fm_model_phi_rad;
extern volatile float wl_fm_model_phi_dot_rad_s;
extern volatile float wl_fm_model_theta_ll_rad;
extern volatile float wl_fm_model_theta_ll_dot_rad_s;
extern volatile float wl_fm_model_theta_lr_rad;
extern volatile float wl_fm_model_theta_lr_dot_rad_s;
extern volatile float wl_fm_model_theta_b_rad;
extern volatile float wl_fm_model_theta_b_dot_rad_s;
extern volatile float wl_fm_model_l_l_m;
extern volatile float wl_fm_model_l_r_m;
extern volatile float wl_fm_yaw_target_rad;
extern volatile float wl_fm_yaw_motor_pos_rad;
extern volatile float wl_fm_yaw_motor_vel_rad_s;
extern volatile float wl_fm_pitch_target_rad;
extern volatile float wl_fm_pitch_motor_pos_rad;
extern volatile float wl_fm_pitch_motor_vel_rad_s;
}

void ControlLoop();
