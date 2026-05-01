#pragma once

#include <cstdint>
#include <optional>

#include "fdcan.h"
#include "stm32h7xx_it.h"
#include "usart.h"

#include <librm.hpp>

#include "wheel_legged_params.hpp"
#include "globals_no_dtcm.hpp"

#include "chassis/chassis.hpp"
#include "chassis/fsm.hpp"
#include "gimbal_to_chassis_rx_bridge.hpp"
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

  SharedResourcesNoDtcm *no_dtcm{&globals_no_dtcm};  ///< DMA 相关外设资源

  rm::device::DR16 dr16{no_dtcm->rc_uart};  ///< 遥控器接收机

  // 保持值语义成员，实际构造延后到 Init()。
  std::optional<rm::hal::ThrottledCan<>> joint_can{};                 ///< 腿部 DM 电机 CAN
  std::optional<rm::hal::ThrottledCan<>> wheel_can{};                 ///< 轮毂/偏航电机 CAN
  std::optional<rm::hal::ThrottledCan<>> gimbal_can{};                ///< 云台俯仰与云台惯导 CAN
  std::optional<DmMitMotor> dm_lf{};                                  ///< 左前腿关节 DM 电机
  std::optional<DmMitMotor> dm_lb{};                                  ///< 左后腿关节 DM 电机
  std::optional<DmMitMotor> dm_rf{};                                  ///< 右前腿关节 DM 电机
  std::optional<DmMitMotor> dm_rb{};                                  ///< 右后腿关节 DM 电机
  std::optional<rm::device::M3508> left_wheel{};                      ///< 左轮 M3508
  std::optional<rm::device::M3508> right_wheel{};                     ///< 右轮 M3508
  std::optional<rm::device::HipnucImu> chassis_imu{};                 ///< 底盘惯导
  std::optional<GimbalToChassisRxBridge> gimbal_rx{};  ///< 云台→底盘 CAN 桥（惯导+键鼠）

  std::optional<DmMitMotor> yaw_motor{};    ///< 云台偏航 DM 电机
  std::optional<DmMitMotor> pitch_motor{};  ///< 云台俯仰 DM 电机

  chassis::Fsm chassis_fsm{};  ///< 底盘状态机
  chassis::Chassis chassis{};  ///< 底盘控制器
  gimbal::Fsm gimbal_fsm{};    ///< 云台状态机
  gimbal::Gimbal gimbal{};     ///< 云台控制器

  /**
   * @brief 懒加载单例入口
   */
  static SharedResources &GetInstance() {
    static SharedResources *shared_resources_instance{nullptr};

    if (shared_resources_instance == nullptr) {
      shared_resources_instance = new SharedResources;
      shared_resources_instance->Init();
    }

    return *shared_resources_instance;
  }

  /**
   * @brief 初始化外设对象、状态机与控制器
   */
  void Init() {
    const auto prepare_uart_rx_to_idle_dma = [](UART_HandleTypeDef &huart, const IRQn_Type uart_irqn,
                                                const IRQn_Type dma_rx_irqn) {
      // Clear stale UART/DMA state before librm starts ReceiveToIdle mode.
      (void)HAL_UART_AbortReceive(&huart);
      (void)HAL_UART_Abort(&huart);
      __HAL_UART_CLEAR_FLAG(&huart, UART_CLEAR_PEF);
      __HAL_UART_CLEAR_FLAG(&huart, UART_CLEAR_FEF);
      __HAL_UART_CLEAR_FLAG(&huart, UART_CLEAR_NEF);
      __HAL_UART_CLEAR_FLAG(&huart, UART_CLEAR_OREF);
      __HAL_UART_SEND_REQ(&huart, UART_RXDATA_FLUSH_REQUEST);
      huart.ErrorCode = HAL_UART_ERROR_NONE;
      huart.RxState = HAL_UART_STATE_READY;
      huart.gState = HAL_UART_STATE_READY;
      HAL_NVIC_ClearPendingIRQ(uart_irqn);
      HAL_NVIC_ClearPendingIRQ(dma_rx_irqn);
    };

    prepare_uart_rx_to_idle_dma(huart5, UART5_IRQn, DMA1_Stream0_IRQn);
    prepare_uart_rx_to_idle_dma(huart10, USART10_IRQn, DMA1_Stream5_IRQn);

    dr16.Begin();

    if (!joint_can.has_value()) {
      joint_can.emplace(wheel_legged::params::active::globals::kJointCanTxLimitHz, hfdcan1);
      joint_can->SetFilter(0, 0);
      joint_can->Begin();
    }
    if (!wheel_can.has_value()) {
      wheel_can.emplace(wheel_legged::params::active::globals::kWheelCanTxLimitHz, hfdcan2);
      wheel_can->SetFilter(0, 0);
      wheel_can->Begin();
    }
    if (!gimbal_can.has_value()) {
      gimbal_can.emplace(wheel_legged::params::active::globals::kGimbalCanTxLimitHz, hfdcan3);
      gimbal_can->SetFilter(0, 0);
      gimbal_can->Begin();
    }
    if (!gimbal_rx.has_value()) {
      gimbal_rx.emplace(*gimbal_can);
    }

    if (!dm_lf.has_value()) {
      dm_lf.emplace(*joint_can, wheel_legged::params::active::globals::kDmLfSettings);
      dm_lb.emplace(*joint_can, wheel_legged::params::active::globals::kDmLbSettings);
      dm_rf.emplace(*joint_can, wheel_legged::params::active::globals::kDmRfSettings);
      dm_rb.emplace(*joint_can, wheel_legged::params::active::globals::kDmRbSettings);
    }

    if (!left_wheel.has_value()) {
      left_wheel.emplace(*wheel_can, wheel_legged::params::active::globals::kLeftWheelId);
      right_wheel.emplace(*wheel_can, wheel_legged::params::active::globals::kRightWheelId);
    }

    if (!pitch_motor.has_value()) {
      pitch_motor.emplace(*gimbal_can, wheel_legged::params::active::gimbal::kPitchMotorSettings);
    }

    if (!yaw_motor.has_value()) {
      yaw_motor.emplace(*wheel_can, wheel_legged::params::active::gimbal::kYawMotorSettings);
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

/** @brief 遥控器在线与档位信息 */
extern volatile uint8_t wl_fm_dr16_online;
extern volatile int32_t wl_fm_dr16_switch_l;
extern volatile int32_t wl_fm_dr16_switch_r;
extern volatile int16_t wl_fm_dr16_dial;

/** @brief 遥控器语义化请求 */
extern volatile uint8_t wl_fm_dr16_enable_request;
extern volatile uint8_t wl_fm_dr16_spin_request;
extern volatile uint8_t wl_fm_dr16_jump_trigger_edge;
extern volatile uint8_t wl_fm_tc_mid_leg_hold;

/** @brief 底盘核心观测量 */
extern volatile float wl_fm_chassis_leg_length_m;
extern volatile float wl_fm_left_leg_length_m;
extern volatile float wl_fm_right_leg_length_m;
extern volatile float wl_fm_chassis_speed_mps;
extern volatile float wl_fm_left_support_force_n;
extern volatile float wl_fm_right_support_force_n;
extern volatile uint8_t wl_fm_off_ground_in_mid_high_leg;

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
extern volatile float wl_fm_target_s_m;
extern volatile float wl_fm_target_s_dot_mps;
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
extern volatile uint8_t wl_fm_yaw_motor_status;
extern volatile uint8_t wl_fm_pitch_motor_status;
extern volatile uint8_t wl_fm_yaw_motor_raw_status_byte;
extern volatile uint8_t wl_fm_pitch_motor_raw_status_byte;
}

void ControlLoop();
