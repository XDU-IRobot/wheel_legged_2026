#pragma once

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "Eigen/Dense"

#include "common/controllers/gimbal_dynamics.hpp"
#include "../params.hpp"

/**
 * @file  targets/wheel_legged/include/gimbal/gimbal_ident.hpp
 * @brief 云台动力学辨识与前馈验证控制器
 *
 * kIdent 模式：五次谐波轨迹 + 单位置环 PID（yaw/pitch 均用电机编码器反馈）
 *             通过串口发送 CSV 数据供上位机离线辨识
 * kFfVerify 模式：纯动力学前馈跟随五次谐波轨迹，用于验证辨识参数
 */

namespace gimbal {

namespace ns = wheel_legged::params;
namespace ns_ident = wheel_legged::params::common::gimbal_ident;

class GimbalIdent {
 public:
  struct Input {
    float yaw_motor_pos_rad{0.0f};
    float yaw_motor_vel_rad_s{0.0f};
    float pitch_motor_pos_rad{0.0f};
    float pitch_motor_vel_rad_s{0.0f};
    float dt_s{ns_ident::kDefaultDtS};
  };

  struct Output {
    float yaw_cmd_tau{0.0f};
    float pitch_cmd_tau{0.0f};
    bool data_pending{false};
    const char *tx_data{nullptr};
    size_t tx_len{0};
  };

 private:

  struct TrajectoryPoint {
    float q;
    float dq;
    float ddq;
  };

  rm::modules::PID ident_pid_yaw_{};
  rm::modules::PID ident_pid_pitch_{};
  Gimbal2DofDynamics dynamics_{};

  bool ident_active_{false};
  float ident_time_s_{0.0f};
  float ident_yaw_center_{0.0f};
  float ident_pitch_center_{0.0f};

  bool ff_verify_active_{false};
  float ff_verify_time_s_{0.0f};

  uint8_t data_tick_counter_{0};

  char tx_buf_[ns_ident::kIdentUartTxBufSize]{};
  size_t tx_len_{0};

  static TrajectoryPoint EvaluateTrajectory(float center, const float (&amplitudes)[ns_ident::kHarmonicCount], float t) {
    TrajectoryPoint point{center, 0.0f, 0.0f};
    const float wf = 2.0f * ns::common::kPi * ns_ident::kBaseFreqHz;
    for (size_t i = 0; i < ns_ident::kHarmonicCount; ++i) {
      const float k = static_cast<float>(i + 1);
      const float kwf = k * wf;
      const float phase = kwf * t;
      point.q += amplitudes[i] * std::sin(phase);
      point.dq += amplitudes[i] * kwf * std::cos(phase);
      point.ddq -= amplitudes[i] * kwf * kwf * std::sin(phase);
    }
    return point;
  }

  void PrepareCsvData(const Input &input, const Output &output) {
    int len = std::snprintf(tx_buf_, sizeof(tx_buf_), "%lu,",
                            static_cast<unsigned long>(ident_time_s_ * 1000.0f));
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, output.yaw_cmd_tau);
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, ",");
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, input.yaw_motor_pos_rad);
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, ",");
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, input.yaw_motor_vel_rad_s);
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, ",");
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, output.pitch_cmd_tau);
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, ",");
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, input.pitch_motor_pos_rad);
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, ",");
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, input.pitch_motor_vel_rad_s);
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, "\r\n");
    if (len <= 0) return;
    if (static_cast<size_t>(len) >= sizeof(tx_buf_)) len = sizeof(tx_buf_) - 1;
    tx_len_ = static_cast<size_t>(len);
  }

  static int AppendFloat(char *buffer, size_t size, float value) {
    if (size == 0) return 0;
    const bool negative = value < 0.0f;
    float abs_value = negative ? -value : value;
    unsigned long integer_part = static_cast<unsigned long>(abs_value);
    unsigned long fractional_part =
        static_cast<unsigned long>((abs_value - static_cast<float>(integer_part)) * 1000000.0f + 0.5f);
    if (fractional_part >= 1000000UL) {
      ++integer_part;
      fractional_part -= 1000000UL;
    }
    return std::snprintf(buffer, size, "%s%lu.%06lu", negative ? "-" : "", integer_part, fractional_part);
  }

 public:

  void Init() {
    ident_pid_yaw_.SetKp(ns_ident::kIdentYawPosPid.kp)
        .SetKi(ns_ident::kIdentYawPosPid.ki)
        .SetKd(ns_ident::kIdentYawPosPid.kd)
        .SetMaxOut(ns_ident::kIdentYawPosPid.max_out)
        .SetMaxIout(ns_ident::kIdentYawPosPid.max_iout);
    ident_pid_pitch_.SetKp(ns_ident::kIdentPitchPosPid.kp)
        .SetKi(ns_ident::kIdentPitchPosPid.ki)
        .SetKd(ns_ident::kIdentPitchPosPid.kd)
        .SetMaxOut(ns_ident::kIdentPitchPosPid.max_out)
        .SetMaxIout(ns_ident::kIdentPitchPosPid.max_iout);
  }

  Output IdentUpdate(const Input &input) {
    if (!ident_active_) {
      ident_time_s_ = 0.0f;
      ident_yaw_center_ = 0.0f;
      ident_pitch_center_ = ns_ident::kIdentPitchCenter;
      ident_pid_yaw_.Clear();
      ident_pid_pitch_.Clear();
      ident_active_ = true;
      data_tick_counter_ = 0;
    }

    const auto yaw_traj = EvaluateTrajectory(ident_yaw_center_, ns_ident::kYawAmp, ident_time_s_);
    const auto pitch_traj = EvaluateTrajectory(ident_pitch_center_, ns_ident::kPitchAmp, ident_time_s_);

    const float yaw_target = yaw_traj.q;
    const float pitch_target =
        std::clamp(pitch_traj.q, ns_ident::kIdentPitchTopLimit, ns_ident::kIdentPitchBottomLimit);

    ident_pid_yaw_.Update(yaw_target, input.yaw_motor_pos_rad, input.dt_s);
    ident_pid_pitch_.Update(pitch_target, input.pitch_motor_pos_rad, input.dt_s);

    ident_time_s_ += input.dt_s;

    Output out{};
    out.yaw_cmd_tau = std::clamp(ident_pid_yaw_.out(), -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);
    out.pitch_cmd_tau = std::clamp(ident_pid_pitch_.out(), -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);

    if (++data_tick_counter_ >= 5) {
      data_tick_counter_ = 0;
      PrepareCsvData(input, out);
      out.data_pending = true;
      out.tx_data = tx_buf_;
      out.tx_len = tx_len_;
    }

    return out;
  }

  Output FfVerifyUpdate(const Input &input) {
    if (!ff_verify_active_) {
      ff_verify_time_s_ = 0.0f;
      ff_verify_active_ = true;
    }

    const auto yaw_traj = EvaluateTrajectory(0.0f, ns_ident::kYawAmp, ff_verify_time_s_);
    const auto pitch_traj = EvaluateTrajectory(0.0f, ns_ident::kPitchAmp, ff_verify_time_s_);

    const Eigen::Vector3f g_stationary(0.0f, 0.0f, -9.81f);
    const auto ff = dynamics_.ComputeFfDecomposed(yaw_traj.q, pitch_traj.q, yaw_traj.dq, pitch_traj.dq,
                                                   yaw_traj.ddq, pitch_traj.ddq, g_stationary);

    ff_verify_time_s_ += input.dt_s;

    Output out{};
    out.yaw_cmd_tau = std::clamp(ff.yaw, -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);
    out.pitch_cmd_tau = std::clamp(ff.pitch, -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);
    return out;
  }

  void Reset() {
    ident_active_ = false;
    ff_verify_active_ = false;
    ident_time_s_ = 0.0f;
    ff_verify_time_s_ = 0.0f;
    ident_pid_yaw_.Clear();
    ident_pid_pitch_.Clear();
  }
};

}  // namespace gimbal
