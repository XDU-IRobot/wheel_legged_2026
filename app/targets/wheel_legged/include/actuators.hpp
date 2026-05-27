#pragma once

#include <cstdint>

#include "globals.hpp"
#include "params.hpp"
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
    input.wheel.left_rad_s = -static_cast<float>(g.left_wheel->rpm()) * wheel_legged::params::active::kPi / 30.0f;
    input.wheel.right_rad_s = static_cast<float>(g.right_wheel->rpm()) * wheel_legged::params::active::kPi / 30.0f;

    input.imu.roll_rad = g.chassis_imu->roll();
    input.imu.pitch_rad = -g.chassis_imu->pitch();
    input.imu.yaw_rad = g.chassis_imu->yaw();
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
    }
    // DM 电机始终保持使能，不失能

    const float lf_tau = enable_dm ? output.lf_tau : 0.0f;
    const float lb_tau = enable_dm ? output.lb_tau : 0.0f;
    const float rf_tau = enable_dm ? output.rf_tau : 0.0f;
    const float rb_tau = enable_dm ? output.rb_tau : 0.0f;
    SendDmMitCommand(g, lf_tau, lb_tau, rf_tau, rb_tau);

    // M3508 接收电流命令，底盘控制器输出的轮端力矩在此做比例换算。
    const float lw_current =
        enable_dm ? output.lw_tau * wheel_legged::params::active::actuators::kLeftWheelTorqueToCurrent : 0.0f;
    const float rw_current =
        enable_dm ? output.rw_tau * wheel_legged::params::active::actuators::kRightWheelTorqueToCurrent : 0.0f;
    SendWheelCurrent(g, lw_current, rw_current);
  }

  /**
   * @brief 根据云台控制输出下发 yaw/pitch DM MIT 命令与使能/失能
   */
  void ApplyGimbalOutput(SharedResources &g, const gimbal::Gimbal::UpdateOutput &output) {
    if (output.gimbal_enabled) {
      EnableGimbalMotorsIfNeeded(g);
      SendGimbalMitCommand(g, output.yaw_cmd_torque_nm, output.pitch_cmd_torque_nm);
      // SendGimbalMitCommand(g, 0.0f, 0.0f);
    } else {
      SendGimbalMitCommand(g, 0.0f, 0.0f);
      DisableGimbalMotorsIfNeeded(g);
    }
  }

#if WHEEL_LEGGED_ROBOT_VARIANT != 1
  /**
   * @brief 下发双摩擦轮发射机构电流
   */
  void ApplyShootOutput(SharedResources &g, const ShootOutput &output) {
    if (g.fric_left.has_value()) {
      g.fric_left->SetCurrent(static_cast<int16_t>(output.fric_left_current));
      //       g.fric_left->SetCurrent(static_cast<int16_t>(0));
    }
    if (g.fric_right.has_value()) {
      g.fric_right->SetCurrent(static_cast<int16_t>(output.fric_right_current));
      // g.fric_right->SetCurrent(static_cast<int16_t>(0));
    }
    if (g.dial.has_value()) {
      g.dial->SetCurrent(static_cast<int16_t>(output.dial_current));
      // g.dial->SetCurrent(static_cast<int16_t>(0));
    }
    rm::device::DjiMotorBase::SendCommand(*g.gimbal_can);
    rm::device::DjiMotorBase::SendCommand(*g.wheel_can);
  }
#endif

  void ResetDmMotorsLatch() { dm_enabled_latched_ = false; }
  void ResetGimbalMotorsLatch() { gimbal_motors_enabled_latched_ = false; }
  bool dm_enabled_latched() const { return dm_enabled_latched_; }
  bool gimbal_motors_enabled_latched() const { return gimbal_motors_enabled_latched_; }

  // ---------- 发射机构使能/失能 ----------

#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  void EnableBoosterMotor(SharedResources &g) {
    if (booster_enabled_latched_) return;
    if (!g.booster_motor.has_value()) return;
    g.booster_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
    g.booster_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    booster_enabled_latched_ = true;
  }

  void DisableBoosterMotor(SharedResources &g) {
    if (!booster_enabled_latched_) return;
    if (g.booster_motor.has_value()) {
      g.booster_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    }
    booster_enabled_latched_ = false;
  }
#endif

 private:
  bool dm_enabled_latched_{false};             ///< 底盘 DM 使能锁存
  bool gimbal_motors_enabled_latched_{false};  ///< 云台 DM 使能锁存
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  bool booster_enabled_latched_{false};  ///< DM 拨盘使能锁存
#endif

  static bool IsReady(const SharedResources &g) {
    return g.joint_can.has_value() && g.wheel_can.has_value() && g.dm_lf.has_value() && g.dm_lb.has_value() &&
           g.dm_rf.has_value() && g.dm_rb.has_value() && g.left_wheel.has_value() && g.right_wheel.has_value() &&
           g.chassis_imu.has_value();
  }

  static int16_t ClampToI16(float value) {
    if (value > wheel_legged::params::active::actuators::kWheelCurrentClampAbs) {
      return static_cast<int16_t>(wheel_legged::params::active::actuators::kWheelCurrentClampAbs);
    }
    if (value < -wheel_legged::params::active::actuators::kWheelCurrentClampAbs) {
      return static_cast<int16_t>(-wheel_legged::params::active::actuators::kWheelCurrentClampAbs);
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

    g.dm_lf->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    g.dm_lb->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    g.dm_rf->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    g.dm_rb->SendInstruction(rm::device::DmMotorInstructions::kDisable);

    dm_enabled_latched_ = false;
  }

  void EnableGimbalMotorsIfNeeded(SharedResources &g) {
    if (gimbal_motors_enabled_latched_) {
      return;
    }
    if (!g.yaw_motor.has_value() || !g.pitch_motor.has_value()) {
      return;
    }

    g.pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
    g.pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);
    g.yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kClearError);
    g.yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kEnable);

    gimbal_motors_enabled_latched_ = true;
  }

  void DisableGimbalMotorsIfNeeded(SharedResources &g) {
    if (!gimbal_motors_enabled_latched_) {
      return;
    }

    if (g.yaw_motor.has_value()) {
      g.yaw_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    }
    if (g.pitch_motor.has_value()) {
      g.pitch_motor->SendInstruction(rm::device::DmMotorInstructions::kDisable);
    }
    gimbal_motors_enabled_latched_ = false;
  }

  static void SendGimbalMitCommand(SharedResources &g, float yaw_tau, float pitch_tau) {
    if (g.yaw_motor.has_value()) {
      g.yaw_motor->SetMitCommand(0.0f, 0.0f, yaw_tau, 0.0f, 0.0f);
      // g.yaw_motor->SetMitCommand(0.0f, 0.0f, 0, 0.0f, 0.0f);
    }
    if (g.pitch_motor.has_value()) {
      g.pitch_motor->SetMitCommand(0.0f, 0.0f, pitch_tau, 0.0f, 0.0f);
      // g.pitch_motor->SetMitCommand(0.0f, 0.0f, 0, 0.0f, 0.0f);
    }
  }

  static void SendDmMitCommand(SharedResources &g, float lf_tau, float lb_tau, float rf_tau, float rb_tau) {
    // 关关节
    // g.dm_lb->SetMitCommand(0.0f, 0.0f, 0, 0.0f, 0.0f);
    // g.dm_lf->SetMitCommand(0.0f, 0.0f, 0, 0.0f, 0.0f);
    // g.dm_rb->SetMitCommand(0.0f, 0.0f, 0, 0.0f, 0.0f);
    // g.dm_rf->SetMitCommand(0.0f, 0.0f, 0, 0.0f, 0.0f);

    // 开关节
    g.dm_lb->SetMitCommand(0.0f, 0.0f, lb_tau, 0.0f, 0.0f);
    g.dm_lf->SetMitCommand(0.0f, 0.0f, lf_tau, 0.0f, 0.0f);
    g.dm_rb->SetMitCommand(0.0f, 0.0f, rb_tau, 0.0f, 0.0f);
    g.dm_rf->SetMitCommand(0.0f, 0.0f, rf_tau, 0.0f, 0.0f);
  }

  static void SendWheelCurrent(SharedResources &g, float left_current, float right_current) {
    // 关轮子
    // g.left_wheel->SetCurrent(ClampToI16(0));
    // g.right_wheel->SetCurrent(ClampToI16(0));

    // 开轮子
    g.left_wheel->SetCurrent(ClampToI16(left_current));
    g.right_wheel->SetCurrent(ClampToI16(right_current));

    rm::device::DjiMotorBase::SendCommand(*g.wheel_can);
  }
};

}  // namespace chassis_runtime
