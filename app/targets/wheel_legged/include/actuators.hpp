#pragma once

#include <cstdint>

#include "globals.hpp"

/**
 * @file  targets/wheel_legged/include/actuators.hpp
 * @brief 执行器适配层：反馈采集与电机命令下发
 */

namespace chassis_runtime {

/**
 * @brief 底盘执行器封装
 * @note  本类不调用 CAN::Process()，CAN 发送由主循环统一驱动。
 */
class Actuators {
 public:
  /**
   * @brief 从硬件反馈填充状态估计输入
   */
  void FillEstimatorInput(SharedResources &g, chassis::ChassisStateEstimatorInput &input) {
    if (!IsReady(g)) {
      return;
    }

    // 关节反馈保持硬件原始符号，统一标定在 ChassisStateEstimator 内完成。
    input.left_leg.front = {g.dm_lf->pos(), g.dm_lf->vel(), g.dm_lf->tau()};
    input.left_leg.back = {g.dm_lb->pos(), g.dm_lb->vel(), g.dm_lb->tau()};
    input.right_leg.front = {g.dm_rf->pos(), g.dm_rf->vel(), g.dm_rf->tau()};
    input.right_leg.back = {g.dm_rb->pos(), g.dm_rb->vel(), g.dm_rb->tau()};

    // 左右轮安装方向相反，此处先转换为车体前进方向一致的轮速符号。
    input.wheel.left_rad_s = -static_cast<float>(g.left_wheel->rpm()) * SharedResources::kPi / 30.0f;
    input.wheel.right_rad_s = static_cast<float>(g.right_wheel->rpm()) * SharedResources::kPi / 30.0f;

    f32 euler_rpy_temp[3], quaternion_temp[4] = {g.chassis_imu->quat_w(), g.chassis_imu->quat_x(), g.chassis_imu->quat_y(), g.chassis_imu->quat_z()};
    modules::QuatToEuler(quaternion_temp, euler_rpy_temp);
    input.imu.roll_rad =  euler_rpy_temp[1];
    input.imu.pitch_rad = -euler_rpy_temp[0];
    input.imu.yaw_rad = euler_rpy_temp[2];
    input.imu.gyro_x_rad_s = g.chassis_imu->gyro_y();
    input.imu.gyro_y_rad_s = -g.chassis_imu->gyro_x();
    input.imu.gyro_z_rad_s = g.chassis_imu->gyro_z();
    input.imu.acc_x_mps2 = g.chassis_imu->acc_x();
    input.imu.acc_y_mps2 = g.chassis_imu->acc_y();
    input.imu.acc_z_mps2 = g.chassis_imu->acc_z();
    input.yaw_motor_rad = g.yaw_motor.has_value() ? g.yaw_motor->pos() : 0.0f;
  }

  /**
   * @brief 下发底盘控制输出到关节与轮毂电机
   */
  void ApplyChassisOutput(SharedResources &g, const chassis::Chassis::UpdateOutput &output, bool enable_dm) {
    if (!IsReady(g)) {
      return;
    }

    if (enable_dm) {
      EnableDmIfNeeded(g);
    } else {
      DisableDmIfNeeded(g);
    }

    const float lf_tau = enable_dm ? output.lf_tau : 0.0f;
    const float lb_tau = enable_dm ? output.lb_tau : 0.0f;
    const float rf_tau = enable_dm ? output.rf_tau : 0.0f;
    const float rb_tau = enable_dm ? output.rb_tau : 0.0f;
    SendDmMitCommand(g, lf_tau, lb_tau, rf_tau, rb_tau);

    // M3508 接收电流命令，底盘控制器输出的轮端力矩在此做比例换算。
    const float lw_current = enable_dm ? output.lw_tau * kWheelTorqueToCurrent : 0.0f;
    const float rw_current = enable_dm ? output.rw_tau * kWheelTorqueToCurrent : 0.0f;
    SendWheelCurrent(g, lw_current, rw_current);
  }

 private:
  static constexpr float kWheelTorqueToCurrent = 2300.1f;  ///< 轮毂力矩转电流比例
  bool dm_enabled_latched_{false};                         ///< DM 使能锁存

  static bool IsReady(const SharedResources &g) {
    return g.joint_can.has_value() && g.wheel_can.has_value() && g.dm_lf.has_value() && g.dm_lb.has_value() &&
           g.dm_rf.has_value() && g.dm_rb.has_value() && g.left_wheel.has_value() && g.right_wheel.has_value() &&
           g.chassis_imu.has_value();
  }

  static int16_t ClampToI16(float value) {
    if (value > 16000.0f) {
      return 16000;
    }
    if (value < -16000.0f) {
      return -16000;
    }
    return static_cast<int16_t>(value);
  }

  /**
   * @brief 首次使能全部 DM 电机
   */
  void EnableDmIfNeeded(SharedResources &g) {
    if (dm_enabled_latched_) {
      return;
    }

    g.dm_lf->SendInstruction(rm::device::DmMotorInstructions::kClearError);
    g.dm_lf->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    g.dm_lb->SendInstruction(rm::device::DmMotorInstructions::kClearError);
    g.dm_lb->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    g.dm_rf->SendInstruction(rm::device::DmMotorInstructions::kClearError);
    g.dm_rf->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    g.dm_rb->SendInstruction(rm::device::DmMotorInstructions::kClearError);
    g.dm_rb->SendInstruction(rm::device::DmMotorInstructions::kEnable);

    dm_enabled_latched_ = true;
  }

  /**
   * @brief 失能全部 DM 电机
   */
  void DisableDmIfNeeded(SharedResources &g) {
    if (!dm_enabled_latched_) {
      return;
    }
    g.dm_rf->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    g.dm_rb->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    g.dm_lf->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    g.dm_lb->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    // g.dm_rf->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    // g.dm_rb->SendInstruction(rm::device::DmMotorInstructions::kDisable);

    dm_enabled_latched_ = false;
  }

  static void SendDmMitCommand(SharedResources &g, float lf_tau, float lb_tau, float rf_tau, float rb_tau) {
    // g.dm_lb->SetMitCommand(0.0f, 0.0f, 0, 0.0f, 0.0f);
    // g.dm_lf->SetMitCommand(0.0f, 0.0f, 0, 0.0f, 0.0f);
    // g.dm_rb->SetMitCommand(0.0f, 0.0f, 0, 0.0f, 0.0f);
    // g.dm_rf->SetMitCommand(0.0f, 0.0f, 0, 0.0f, 0.0f);
    //
    g.dm_lb->SetMitCommand(0.0f, 0.0f, lb_tau, 0.0f, 0.0f);
    g.dm_lf->SetMitCommand(0.0f, 0.0f, lf_tau, 0.0f, 0.0f);
    g.dm_rb->SetMitCommand(0.0f, 0.0f, rb_tau, 0.0f, 0.0f);
    g.dm_rf->SetMitCommand(0.0f, 0.0f, rf_tau, 0.0f, 0.0f);
  }

  static void SendWheelCurrent(SharedResources &g, float left_current, float right_current) {
    g.left_wheel->SetCurrent(ClampToI16(left_current));
    g.right_wheel->SetCurrent(ClampToI16(right_current));
    // g.left_wheel->SetCurrent(ClampToI16(0));
    // g.right_wheel->SetCurrent(ClampToI16(0));
    rm::device::DjiMotorBase::SendCommand(*g.wheel_can);
  }
};

}  // namespace chassis_runtime
