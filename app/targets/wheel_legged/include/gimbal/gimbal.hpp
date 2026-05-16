#pragma once

#include <algorithm>
#include <cmath>

#include "librm.hpp"

#include "common/controllers/gimbal_2dof.hpp"
#include "gimbal_ident.hpp"
#include "../fsm_common.hpp"
#include "../params.hpp"

/**
 * @file  targets/wheel_legged/include/gimbal/gimbal.hpp
 * @brief 云台双轴控制器与 DM 电机命令输出
 */

namespace gimbal {

/**
 * @brief 云台控制主类
 */
class Gimbal {
 public:
  using DmMitMotor = rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>;

  /**
   * @brief 单次云台控制更新输入
   */
  struct UpdateInput {
    DmMitMotor *yaw_motor{nullptr};        ///< 偏航 DM 电机对象
    DmMitMotor *pitch_motor{nullptr};      ///< 俯仰 DM 电机对象
    bool gimbal_enable{false};             ///< 是否使能云台输出
    bool align_to_chassis_forward{false};  ///< 是否对齐车体前方
    bool use_yaw_motor_feedback{false};    ///< 是否用偏航电机编码器作为偏航反馈
    bool aimbot_mode{false};               ///< 是否自瞄模式，切换 PID 参数
    wheel_legged::GimbalTarget target{};   ///< 云台角度目标
    float chassis_yaw_rad{0.0f};           ///< 车体偏航角
    float chassis_pitch_rad{0.0f};         ///< 车体俯仰角，用于俯仰重力补偿
    float yaw_motor_rad{0.0f};             ///< 偏航电机编码器角度
    float gimbal_imu_yaw_rad{0.0f};        ///< 云台惯导偏航角
    float gimbal_imu_pitch_rad{0.0f};      ///< 云台惯导俯仰角
    float gimbal_imu_gyro_z_rad_s{0.0f};   ///< 云台惯导偏航角速度（替代偏航电机 vel）
    float gimbal_imu_gyro_x_rad_s{0.0f};   ///< 云台惯导俯仰角速度（替代俯仰电机 vel）
    float dt_s{wheel_legged::params::active::gimbal::kDefaultDtS};  ///< 控制周期

    /// 辨识/验证模式专用
    GimbalIdent *ident{nullptr};                                                             ///< 辨识控制器对象
    wheel_legged::GimbalTestProfile test_profile{wheel_legged::GimbalTestProfile::kNormal};  ///< 测试子模式
  };

  /**
   * @brief 单次云台控制更新输出
   */
  struct UpdateOutput {
    bool gimbal_enabled{false};  ///< 本周期云台是否使能

    float yaw_target_rad{0.0f};     ///< 偏航目标角
    float yaw_pos_rad{0.0f};        ///< 偏航反馈角
    float yaw_vel_rad_s{0.0f};      ///< 偏航反馈角速度
    float yaw_cmd_torque_nm{0.0f};  ///< 偏航输出力矩

    float pitch_target_rad{0.0f};     ///< 俯仰目标角
    float pitch_pos_rad{0.0f};        ///< 俯仰反馈角
    float pitch_vel_rad_s{0.0f};      ///< 俯仰反馈角速度
    float pitch_cmd_torque_nm{0.0f};  ///< 俯仰输出力矩

    /// 辨识模式串口数据
    bool ident_data_pending{false};      ///< 是否有待发送的辨识数据
    const char *ident_tx_data{nullptr};  ///< 待发送数据指针
    size_t ident_tx_len{0};              ///< 待发送数据长度
  };

  /**
   * @brief 初始化 PID
   */
  void Init() {
    last_use_yaw_motor_feedback_ = false;
    ConfigurePid();
    ClearPid();
    output_ = {};
  }

  /**
   * @brief 执行一次云台控制更新，仅计算力矩，不操作电机
   */
  void Update(const UpdateInput &input) {
    output_ = {};

    if (input.yaw_motor == nullptr || input.pitch_motor == nullptr) {
      last_use_yaw_motor_feedback_ = false;
      ClearPid();
      return;
    }

    output_.yaw_pos_rad = input.use_yaw_motor_feedback ? input.yaw_motor_rad : input.gimbal_imu_yaw_rad;
    output_.yaw_vel_rad_s = input.gimbal_imu_gyro_z_rad_s;
    output_.pitch_pos_rad = -input.gimbal_imu_pitch_rad;
    output_.pitch_vel_rad_s = input.gimbal_imu_gyro_x_rad_s;

    const float desired_yaw = input.align_to_chassis_forward ? input.chassis_yaw_rad : input.target.yaw_rad;
    output_.yaw_target_rad = desired_yaw;
    output_.pitch_target_rad = std::clamp(input.target.pitch_rad, wheel_legged::params::active::gimbal::kPitchMinRad,
                                          wheel_legged::params::active::gimbal::kPitchMaxRad);
    output_.gimbal_enabled = input.gimbal_enable;

    if (!input.gimbal_enable) {
      controller_.Enable(false);
      ClearPid();
      last_use_yaw_motor_feedback_ = false;
      return;
    }

    // 辨识/前馈验证模式：使用电机编码器反馈，走专用控制器
    if (input.ident != nullptr && (input.test_profile == wheel_legged::GimbalTestProfile::kIdent ||
                                   input.test_profile == wheel_legged::GimbalTestProfile::kFfVerify)) {
      if (!last_ident_mode_active_) {
        input.ident->Reset();
        ClearPid();
      }
      last_ident_mode_active_ = true;

      GimbalIdent::Input ident_in{};
      ident_in.yaw_motor_pos_rad = input.yaw_motor_rad;
      ident_in.yaw_motor_vel_rad_s = static_cast<float>(input.yaw_motor->vel());
      ident_in.pitch_motor_pos_rad = input.pitch_motor->pos();
      ident_in.pitch_motor_vel_rad_s = static_cast<float>(input.pitch_motor->vel());
      ident_in.dt_s = (input.dt_s > 1e-5f) ? input.dt_s : wheel_legged::params::active::gimbal::kDefaultDtS;

      if (input.test_profile == wheel_legged::GimbalTestProfile::kIdent) {
        const auto ident_out = input.ident->IdentUpdate(ident_in);
        output_.yaw_cmd_torque_nm = ident_out.yaw_cmd_tau;
        output_.pitch_cmd_torque_nm = ident_out.pitch_cmd_tau;
        output_.yaw_target_rad = 0.0f;
        output_.pitch_target_rad = 0.0f;
        output_.yaw_pos_rad = ident_in.yaw_motor_pos_rad;
        output_.pitch_pos_rad = ident_in.pitch_motor_pos_rad;
        output_.ident_data_pending = ident_out.data_pending;
        output_.ident_tx_data = ident_out.tx_data;
        output_.ident_tx_len = ident_out.tx_len;
      } else {
        const auto ident_out = input.ident->FfVerifyUpdate(ident_in);
        output_.yaw_cmd_torque_nm = ident_out.yaw_cmd_tau;
        output_.pitch_cmd_torque_nm = ident_out.pitch_cmd_tau;
        output_.yaw_target_rad = 0.0f;
        output_.pitch_target_rad = 0.0f;
        output_.yaw_pos_rad = ident_in.yaw_motor_pos_rad;
        output_.pitch_pos_rad = ident_in.pitch_motor_pos_rad;
      }
      return;
    }
    // 退出辨识/验证模式时复位
    if (last_ident_mode_active_ && input.ident != nullptr) {
      input.ident->Reset();
      ClearPid();
    }
    last_ident_mode_active_ = false;

    if (input.use_yaw_motor_feedback != last_use_yaw_motor_feedback_) {
      ClearPid();
    }
    last_use_yaw_motor_feedback_ = input.use_yaw_motor_feedback;

    if (input.aimbot_mode != last_aimbot_mode_) {
      ClearPid();
      input.aimbot_mode ? ConfigureAimbotPid() : ConfigurePid();
    }
    last_aimbot_mode_ = input.aimbot_mode;

    const float dt_s = (input.dt_s > 1e-5f) ? input.dt_s : wheel_legged::params::active::gimbal::kDefaultDtS;
    controller_.Enable(true);
    controller_.SetTarget(output_.yaw_target_rad, output_.pitch_target_rad);
    controller_.Update(output_.yaw_pos_rad, output_.yaw_vel_rad_s, output_.pitch_pos_rad, output_.pitch_vel_rad_s,
                       dt_s);

    output_.yaw_cmd_torque_nm =
        std::clamp(controller_.output().yaw, -wheel_legged::params::active::gimbal::kDmTorqueLimitNm,
                   wheel_legged::params::active::gimbal::kDmTorqueLimitNm);
    const float pitch_gravity_ff =
        wheel_legged::params::active::gimbal::kPitchGravityCompensationNm * std::cos(input.gimbal_imu_pitch_rad);
    output_.pitch_cmd_torque_nm = std::clamp(controller_.output().pitch + pitch_gravity_ff,
                                             -wheel_legged::params::active::gimbal::kDmTorqueLimitNm,
                                             wheel_legged::params::active::gimbal::kDmTorqueLimitNm);
  }

  /** @brief 获取最近一次云台控制输出 */
  [[nodiscard]] const UpdateOutput &GetOutput() const { return output_; }

 private:
  /** @brief 配置双环 PID 参数 */
  void ConfigurePid() {
    const auto &yaw_pos = wheel_legged::params::active::gimbal::kYawPositionPid;
    const auto &yaw_spd = wheel_legged::params::active::gimbal::kYawSpeedPid;
    const auto &pitch_pos = wheel_legged::params::active::gimbal::kPitchPositionPid;
    const auto &pitch_spd = wheel_legged::params::active::gimbal::kPitchSpeedPid;
    controller_.pid()
        .yaw_position.SetKp(yaw_pos.kp)
        .SetKi(yaw_pos.ki)
        .SetKd(yaw_pos.kd)
        .SetMaxOut(yaw_pos.max_out)
        .SetMaxIout(yaw_pos.max_iout);
    controller_.pid()
        .yaw_speed.SetKp(yaw_spd.kp)
        .SetKi(yaw_spd.ki)
        .SetKd(yaw_spd.kd)
        .SetMaxOut(yaw_spd.max_out)
        .SetMaxIout(yaw_spd.max_iout);
    controller_.pid()
        .pitch_position.SetKp(pitch_pos.kp)
        .SetKi(pitch_pos.ki)
        .SetKd(pitch_pos.kd)
        .SetMaxOut(pitch_pos.max_out)
        .SetMaxIout(pitch_pos.max_iout);
    controller_.pid()
        .pitch_speed.SetKp(pitch_spd.kp)
        .SetKi(pitch_spd.ki)
        .SetKd(pitch_spd.kd)
        .SetMaxOut(pitch_spd.max_out)
        .SetMaxIout(pitch_spd.max_iout);
  }

  /** @brief 配置自瞄双环 PID 参数 */
  void ConfigureAimbotPid() {
    const auto &yaw_pos = wheel_legged::params::active::aimbot::kYawPositionPid;
    const auto &yaw_spd = wheel_legged::params::active::aimbot::kYawSpeedPid;
    const auto &pitch_pos = wheel_legged::params::active::aimbot::kPitchPositionPid;
    const auto &pitch_spd = wheel_legged::params::active::aimbot::kPitchSpeedPid;
    controller_.pid()
        .yaw_position.SetKp(yaw_pos.kp)
        .SetKi(yaw_pos.ki)
        .SetKd(yaw_pos.kd)
        .SetMaxOut(yaw_pos.max_out)
        .SetMaxIout(yaw_pos.max_iout);
    controller_.pid()
        .yaw_speed.SetKp(yaw_spd.kp)
        .SetKi(yaw_spd.ki)
        .SetKd(yaw_spd.kd)
        .SetMaxOut(yaw_spd.max_out)
        .SetMaxIout(yaw_spd.max_iout);
    controller_.pid()
        .pitch_position.SetKp(pitch_pos.kp)
        .SetKi(pitch_pos.ki)
        .SetKd(pitch_pos.kd)
        .SetMaxOut(pitch_pos.max_out)
        .SetMaxIout(pitch_pos.max_iout);
    controller_.pid()
        .pitch_speed.SetKp(pitch_spd.kp)
        .SetKi(pitch_spd.ki)
        .SetKd(pitch_spd.kd)
        .SetMaxOut(pitch_spd.max_out)
        .SetMaxIout(pitch_spd.max_iout);
  }

  /** @brief 清空双环 PID 积分与历史状态 */
  void ClearPid() {
    controller_.pid().yaw_position.Clear();
    controller_.pid().yaw_speed.Clear();
    controller_.pid().pitch_position.Clear();
    controller_.pid().pitch_speed.Clear();
  }

  bool last_use_yaw_motor_feedback_{false};
  bool last_aimbot_mode_{false};
  bool last_ident_mode_active_{false};

  Gimbal2Dof controller_{};

  UpdateOutput output_{};
};

}  // namespace gimbal