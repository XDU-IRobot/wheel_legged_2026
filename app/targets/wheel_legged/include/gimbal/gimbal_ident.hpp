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
 * 支持 5 种子模式（编译时通过 kIdentSubMode 选择）：
 *   kGravity      — Step 1: Pitch 多角度静止悬停，辨识重力项 theta[3], theta[4]
 *   kFriction     — Step 2: Yaw/Pitch 分段匀速扫角，辨识摩擦项 theta[5..8]
 *   kPitchInertia — Step 3: Yaw 固定 + Pitch 正弦加减速，辨识惯量 theta[2]
 *   kCoupling     — Step 4: Pitch 固定 + Yaw 正弦加减速，辨识耦合惯量 theta[0], theta[1]
 *   kHarmonic     — Step 5: 双轴五次谐波综合激励 (原有)
 *   kFfVerify     — 纯动力学前馈跟随五次谐波轨迹 (原有, 独立入口)
 */

namespace gimbal {

namespace ns = wheel_legged::params;
namespace ns_ident = wheel_legged::params::active::gimbal_ident;
namespace ns_gimbal = wheel_legged::params::active::gimbal;

class GimbalIdent {
 public:
  using IdentSubMode = ns_ident::IdentSubMode;

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
    float yaw_target_rad{0.0f};
    float pitch_target_rad{0.0f};
    bool data_pending{false};
    const char *tx_data{nullptr};
    size_t tx_len{0};
  };

 private:
  // ──────────────────────────────────────────────────────────────────────────
  // 轨迹计算
  // ──────────────────────────────────────────────────────────────────────────

  struct TrajectoryPoint {
    float q;
    float dq;
    float ddq;
  };

  static TrajectoryPoint EvaluateTrajectory(float center, const float (&amplitudes)[ns_ident::kHarmonicCount],
                                            float t) {
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

  static TrajectoryPoint EvaluateTrajectory(float center, const float (&amplitudes)[ns_ident::kHarmonicCount],
                                            const float (&phase_offsets)[ns_ident::kHarmonicCount], float t) {
    TrajectoryPoint point{center, 0.0f, 0.0f};
    const float wf = 2.0f * ns::common::kPi * ns_ident::kBaseFreqHz;
    for (size_t i = 0; i < ns_ident::kHarmonicCount; ++i) {
      const float k = static_cast<float>(i + 1);
      const float kwf = k * wf;
      const float phase = kwf * t + phase_offsets[i];
      point.q += amplitudes[i] * std::sin(phase);
      point.dq += amplitudes[i] * kwf * std::cos(phase);
      point.ddq -= amplitudes[i] * kwf * kwf * std::sin(phase);
    }
    return point;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // 串口 CSV 输出
  // ──────────────────────────────────────────────────────────────────────────

  void PrepareCsvData(const Input &input, const Output &output, float time_s) {
    int len = std::snprintf(tx_buf_, sizeof(tx_buf_), "%lu,", static_cast<unsigned long>(time_s * 1000.0f));
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, output.yaw_cmd_tau);
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, ",");
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, input.yaw_motor_pos_rad);
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, ",");
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, input.yaw_motor_vel_rad_s);
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, ",");
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, output.pitch_cmd_tau);
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, ",");
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, input.pitch_motor_pos_rad - ns_ident::kIdentPitchCenter);
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

  // ──────────────────────────────────────────────────────────────────────────
  // 通用辅助: PID 目标钳位与过圈处理
  // ──────────────────────────────────────────────────────────────────────────

  float WrapYawTarget(float target, float current) const {
    float err = target - current;
    if (err > ns::common::kPi) err -= 2.0f * ns::common::kPi;
    if (err < -ns::common::kPi) err += 2.0f * ns::common::kPi;
    return current + err;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // 模式状态变量
  // ──────────────────────────────────────────────────────────────────────────

  rm::modules::PID ident_pid_yaw_{};
  rm::modules::PID ident_pid_pitch_{};
  rm::modules::PID ident_vel_pid_yaw_{};    ///< Friction step 专用速度环 PID
  rm::modules::PID ident_vel_pid_pitch_{};  ///< Friction step 专用速度环 PID
  Gimbal2DofDynamics dynamics_{};

  IdentSubMode submode_{IdentSubMode::kHarmonic};

  // Harmonic / PitchInertia / Coupling 共用
  bool ident_active_{false};
  float ident_time_s_{0.0f};
  float ident_yaw_center_{0.0f};
  float ident_pitch_center_{0.0f};

  // Gravity 模式
  int grav_angle_idx_{0};
  int grav_phase_{0};  // 0=settle, 1=measure, 2=done
  float grav_phase_time_{0.0f};

  // Friction 模式
  int fric_axis_{0};   // 0=yaw+, 1=yaw-, 2=pitch, 4=done
  int fric_phase_{0};  // 0=匀速, 1=暂停
  float fric_phase_time_{0.0f};
  float fric_travel_{0.0f};  // 累计角位移 [rad] (绝对值, yaw 用)
  int fric_pitch_cycle_{0};  // pitch 半程计数: 0-5, 共6个半程=3个往返

  // Coupling 模式
  int coup_angle_idx_{0};
  float coup_angle_time_{0.0f};

  // FfVerify
  bool ff_verify_active_{false};
  float ff_verify_time_s_{0.0f};

  // UART TX
  uint8_t data_tick_counter_{0};
  char tx_buf_[ns_ident::kIdentUartTxBufSize]{};
  size_t tx_len_{0};

  // ──────────────────────────────────────────────────────────────────────────
  // 模式入口
  // ──────────────────────────────────────────────────────────────────────────

  void ResetIdentState() {
    ident_active_ = false;
    ident_time_s_ = 0.0f;
    ident_yaw_center_ = 0.0f;
    ident_pitch_center_ = ns_ident::kIdentPitchCenter;
    ident_pid_yaw_.Clear();
    ident_pid_pitch_.Clear();
    ident_vel_pid_yaw_.Clear();
    ident_vel_pid_pitch_.Clear();
  }

  void StartGravity() {
    ResetIdentState();
    grav_angle_idx_ = 0;
    grav_phase_ = 0;
    grav_phase_time_ = 0.0f;
    ident_active_ = true;
    data_tick_counter_ = 0;
  }

  void StartFriction() {
    ResetIdentState();
    fric_axis_ = 0;
    fric_phase_ = 0;
    fric_phase_time_ = 0.0f;
    fric_travel_ = 0.0f;
    fric_pitch_cycle_ = 0;
    ident_active_ = true;
    data_tick_counter_ = 0;
  }

  void StartPitchInertia() {
    ResetIdentState();
    ident_active_ = true;
    data_tick_counter_ = 0;
  }

  void StartCoupling() {
    ResetIdentState();
    coup_angle_idx_ = 0;
    coup_angle_time_ = 0.0f;
    ident_active_ = true;
    data_tick_counter_ = 0;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Step 1: 静态重力 — Pitch 多角度静止悬停
  // ──────────────────────────────────────────────────────────────────────────

  Output GravityUpdate(const Input &input) {
    if (!ident_active_) StartGravity();

    if (grav_phase_ == 2) {
      // 全部角度完成，保持最后位置，不再发送 CSV
      const float q2 = ns_ident::kGravityPitchAngles[ns_ident::kGravityAngleCount - 1];
      Output out{};
      out.yaw_cmd_tau = 0.0f;
      out.pitch_cmd_tau = 0.0f;
      out.yaw_target_rad = 0.0f;
      out.pitch_target_rad = q2;
      return out;
    }

    const float target_q2 = ns_ident::kGravityPitchAngles[grav_angle_idx_];

    const float pitch_target = std::clamp(target_q2, ns_ident::kIdentPitchTopLimit, ns_ident::kIdentPitchBottomLimit);
    ident_pid_pitch_.Update(pitch_target, input.pitch_motor_pos_rad, input.dt_s);

    // Yaw 保持在当前位置（不对 yaw 施加力矩干扰）
    float yaw_target = input.yaw_motor_pos_rad;
    ident_pid_yaw_.Update(yaw_target, input.yaw_motor_pos_rad, input.dt_s);

    grav_phase_time_ += input.dt_s;

    // 状态切换
    if (grav_phase_ == 0 && grav_phase_time_ >= ns_ident::kGravitySettleDuration) {
      grav_phase_ = 1;
      grav_phase_time_ = 0.0f;
    } else if (grav_phase_ == 1 && grav_phase_time_ >= ns_ident::kGravityHoldDuration) {
      grav_angle_idx_++;
      if (grav_angle_idx_ >= static_cast<int>(ns_ident::kGravityAngleCount)) {
        grav_phase_ = 2;
      } else {
        grav_phase_ = 0;
        grav_phase_time_ = 0.0f;
        ident_pid_pitch_.Clear();
      }
    }

    Output out{};
    out.yaw_cmd_tau = std::clamp(ident_pid_yaw_.out(), -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);
    out.pitch_cmd_tau = std::clamp(ident_pid_pitch_.out(), -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);
    out.yaw_target_rad = yaw_target;
    out.pitch_target_rad = pitch_target;

    // 仅在测量阶段发送 CSV
    if (grav_phase_ == 1) {
      if (++data_tick_counter_ >= 5) {
        data_tick_counter_ = 0;
        PrepareCsvData(
            input, out,
            grav_phase_time_ + grav_angle_idx_ * (ns_ident::kGravityHoldDuration + ns_ident::kGravitySettleDuration));
        out.data_pending = true;
        out.tx_data = tx_buf_;
        out.tx_len = tx_len_;
      }
    }

    return out;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Step 2: 低速摩擦 — Yaw 正/反各一圈 + Pitch 正/反扫全程
  // ──────────────────────────────────────────────────────────────────────────

  Output FrictionUpdate(const Input &input) {
    if (!ident_active_) StartFriction();

    if (fric_axis_ == 4) {
      Output out{};
      out.yaw_cmd_tau = 0.0f;
      out.pitch_cmd_tau = 0.0f;
      out.yaw_target_rad = input.yaw_motor_pos_rad;
      out.pitch_target_rad = ns_ident::kIdentPitchCenter;
      return out;
    }

    float cmd_yaw = 0.0f, cmd_pitch = 0.0f;
    float yaw_target, pitch_target;

    if (fric_axis_ <= 1) {
      // ── Yaw: 正/反各一圈, Pitch 位置保持 ──
      const float v_abs = ns_ident::kFrictionVelocityRadS;
      const float v = (fric_axis_ == 1) ? -v_abs : v_abs;
      const float target_dist = 2.0f * ns::common::kPi;
      const float vel_ref = (fric_phase_ == 0) ? v : 0.0f;

      yaw_target = input.yaw_motor_pos_rad;
      pitch_target =
          std::clamp(ns_ident::kIdentPitchCenter, ns_ident::kIdentPitchTopLimit, ns_ident::kIdentPitchBottomLimit);
      ident_vel_pid_yaw_.Update(vel_ref, input.yaw_motor_vel_rad_s, input.dt_s);
      ident_pid_pitch_.Update(pitch_target, input.pitch_motor_pos_rad, input.dt_s);
      cmd_yaw = ident_vel_pid_yaw_.out();
      cmd_pitch = ident_pid_pitch_.out();

      // Yaw 摩擦前馈 + Pitch 重力补偿
      {
        const float pitch_q_model = input.pitch_motor_pos_rad - ns_ident::kIdentPitchCenter;
        const Eigen::Vector3f g_vec(0.0f, 0.0f, -9.81f);
        const auto ff =
            dynamics_.ComputeFfDecomposed(input.yaw_motor_pos_rad, pitch_q_model, 0.f, 0.f, 0.f, 0.f, g_vec);
        cmd_yaw += ff.yaw;
        cmd_pitch += ff.pitch;
      }

      fric_travel_ += std::fabs(input.yaw_motor_vel_rad_s) * input.dt_s;
      fric_phase_time_ += input.dt_s;

      bool advance = false;
      switch (fric_phase_) {
        case 0:
          if (fric_travel_ >= target_dist) advance = true;
          break;
        case 1:
          if (fric_phase_time_ >= ns_ident::kFrictionPauseDuration) advance = true;
          break;
      }

      if (advance) {
        fric_phase_++;
        fric_phase_time_ = 0.0f;
        if (fric_phase_ > 1) {
          fric_phase_ = 0;
          fric_travel_ = 0.0f;
          fric_axis_++;
          if (fric_axis_ == 1) {
            ident_vel_pid_yaw_.Clear();
          } else if (fric_axis_ == 2) {
            ident_vel_pid_yaw_.Clear();
            ident_pid_pitch_.Clear();
            fric_pitch_cycle_ = 0;
          }
        }
      }
    } else {
      // ── Pitch: 在上下限之间匀速往返, 共 3 个来回 (6 个半程) ──
      const float v_abs = ns_ident::kFrictionPitchVelocityRadS;
      const bool toward_bottom = (fric_pitch_cycle_ % 2 == 0);  // 偶数→BottomLimit, 奇数→TopLimit
      const float v = toward_bottom ? v_abs : -v_abs;
      const float vel_ref = (fric_phase_ == 0) ? v : 0.0f;

      yaw_target = WrapYawTarget(input.yaw_motor_pos_rad, input.yaw_motor_pos_rad);
      pitch_target = toward_bottom ? ns_ident::kIdentPitchBottomLimit : ns_ident::kIdentPitchTopLimit;
      ident_pid_yaw_.Update(yaw_target, input.yaw_motor_pos_rad, input.dt_s);
      ident_vel_pid_pitch_.Update(vel_ref, input.pitch_motor_vel_rad_s, input.dt_s);
      cmd_yaw = ident_pid_yaw_.out();
      cmd_pitch = ident_vel_pid_pitch_.out();

      // Pitch 重力+摩擦前馈 + 常值偏置
      {
        const float pitch_q_model = input.pitch_motor_pos_rad - ns_ident::kIdentPitchCenter;
        const Eigen::Vector3f g_vec(0.0f, 0.0f, -9.81f);
        const auto ff = dynamics_.ComputeFfDecomposed(yaw_target, pitch_q_model, 0.f, 0.f, 0.f, 0.f, g_vec);
        cmd_yaw += ff.yaw;
        cmd_pitch += ff.pitch + ns_gimbal::kPitchFeedforwardBiasNm;
      }

      fric_phase_time_ += input.dt_s;

      const bool reached_limit = toward_bottom ? (input.pitch_motor_pos_rad >= ns_ident::kIdentPitchBottomLimit)
                                               : (input.pitch_motor_pos_rad <= ns_ident::kIdentPitchTopLimit);

      bool advance = false;
      switch (fric_phase_) {
        case 0:
          if (reached_limit) advance = true;
          break;
        case 1:
          if (fric_phase_time_ >= ns_ident::kFrictionPauseDuration) advance = true;
          break;
      }

      if (advance) {
        fric_phase_++;
        fric_phase_time_ = 0.0f;
        if (fric_phase_ > 1) {
          fric_phase_ = 0;
          fric_pitch_cycle_++;
          ident_vel_pid_pitch_.Clear();
          if (fric_pitch_cycle_ >= 6) {  // 6 个半程 = 3 个往返
            fric_axis_ = 4;
          }
        }
      }
    }

    Output out{};
    out.yaw_cmd_tau = std::clamp(cmd_yaw, -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);
    out.pitch_cmd_tau = std::clamp(cmd_pitch, -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);
    out.yaw_target_rad = yaw_target;
    out.pitch_target_rad = pitch_target;

    if (++data_tick_counter_ >= 5) {
      data_tick_counter_ = 0;
      PrepareCsvData(input, out, ident_time_s_);
      out.data_pending = true;
      out.tx_data = tx_buf_;
      out.tx_len = tx_len_;
    }

    ident_time_s_ += input.dt_s;
    return out;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Step 3: Pitch 惯量 — Yaw 固定 + Pitch 正弦加减速
  // ──────────────────────────────────────────────────────────────────────────

  Output PitchInertiaUpdate(const Input &input) {
    if (!ident_active_) StartPitchInertia();

    const float w = 2.0f * ns::common::kPi * ns_ident::kPitchInertiaFreqHz;
    const float A = ns_ident::kPitchInertiaAmplitude;
    const float center = ns_ident::kIdentPitchCenter;

    const float phase = w * ident_time_s_;
    const float q2_ref = center + A * std::sin(phase);
    const float pitch_target = std::clamp(q2_ref, ns_ident::kIdentPitchTopLimit, ns_ident::kIdentPitchBottomLimit);
    const float yaw_target = WrapYawTarget(0.0f, input.yaw_motor_pos_rad);

    ident_pid_yaw_.Update(yaw_target, input.yaw_motor_pos_rad, input.dt_s);
    ident_pid_pitch_.Update(pitch_target, input.pitch_motor_pos_rad, input.dt_s);

    ident_time_s_ += input.dt_s;

    Output out{};
    out.yaw_cmd_tau = std::clamp(ident_pid_yaw_.out(), -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);
    out.pitch_cmd_tau = std::clamp(ident_pid_pitch_.out(), -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);
    out.yaw_target_rad = yaw_target;
    out.pitch_target_rad = pitch_target;

    if (++data_tick_counter_ >= 5) {
      data_tick_counter_ = 0;
      PrepareCsvData(input, out, ident_time_s_);
      out.data_pending = true;
      out.tx_data = tx_buf_;
      out.tx_len = tx_len_;
    }

    return out;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Step 4: 耦合惯量 — Pitch 固定 + Yaw 正弦加减速
  // ──────────────────────────────────────────────────────────────────────────

  Output CouplingUpdate(const Input &input) {
    if (!ident_active_) StartCoupling();

    if (coup_angle_idx_ >= static_cast<int>(ns_ident::kCouplingAngleCount)) {
      // 全部角度完成
      Output out{};
      out.yaw_cmd_tau = 0.0f;
      out.pitch_cmd_tau = 0.0f;
      out.yaw_target_rad = input.yaw_motor_pos_rad;
      out.pitch_target_rad = ns_ident::kIdentPitchCenter;
      return out;
    }

    const float w = 2.0f * ns::common::kPi * ns_ident::kCouplingFreqHz;
    const float A = ns_ident::kCouplingAmplitude;
    const float pitch_fixed = ns_ident::kCouplingPitchAngles[coup_angle_idx_];

    // Yaw 正弦轨迹, Pitch 固定在指定角度
    const float yaw_continuous = A * std::sin(w * coup_angle_time_);

    const float yaw_target = WrapYawTarget(yaw_continuous, input.yaw_motor_pos_rad);
    const float pitch_target = std::clamp(pitch_fixed, ns_ident::kIdentPitchTopLimit, ns_ident::kIdentPitchBottomLimit);

    ident_pid_yaw_.Update(yaw_target, input.yaw_motor_pos_rad, input.dt_s);
    ident_pid_pitch_.Update(pitch_target, input.pitch_motor_pos_rad, input.dt_s);

    coup_angle_time_ += input.dt_s;
    ident_time_s_ += input.dt_s;

    // 该角度持续时间到, 切换下一个
    if (coup_angle_time_ >= ns_ident::kCouplingDurationPerAngle) {
      coup_angle_idx_++;
      coup_angle_time_ = 0.0f;
      ident_pid_yaw_.Clear();
      ident_pid_pitch_.Clear();
    }

    Output out{};
    out.yaw_cmd_tau = std::clamp(ident_pid_yaw_.out(), -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);
    out.pitch_cmd_tau = std::clamp(ident_pid_pitch_.out(), -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);
    out.yaw_target_rad = yaw_target;
    out.pitch_target_rad = pitch_target;

    if (++data_tick_counter_ >= 5) {
      data_tick_counter_ = 0;
      PrepareCsvData(input, out, ident_time_s_);
      out.data_pending = true;
      out.tx_data = tx_buf_;
      out.tx_len = tx_len_;
    }

    return out;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Step 5: 五次谐波综合激励 (原有逻辑)
  // ──────────────────────────────────────────────────────────────────────────

  Output HarmonicUpdate(const Input &input) {
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
    const auto pitch_traj =
        EvaluateTrajectory(ident_pitch_center_, ns_ident::kPitchAmp, ns_ident::kPitchPhase, ident_time_s_);

    float yaw_err = yaw_traj.q - input.yaw_motor_pos_rad;
    if (yaw_err > ns::common::kPi) yaw_err -= 2.0f * ns::common::kPi;
    if (yaw_err < -ns::common::kPi) yaw_err += 2.0f * ns::common::kPi;
    const float yaw_target = input.yaw_motor_pos_rad + yaw_err;

    const float pitch_target =
        std::clamp(pitch_traj.q, ns_ident::kIdentPitchTopLimit, ns_ident::kIdentPitchBottomLimit);

    ident_pid_yaw_.Update(yaw_target, input.yaw_motor_pos_rad, input.dt_s);
    ident_pid_pitch_.Update(pitch_target, input.pitch_motor_pos_rad, input.dt_s);

    ident_time_s_ += input.dt_s;

    Output out{};
    out.yaw_cmd_tau = std::clamp(ident_pid_yaw_.out(), -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);
    out.pitch_cmd_tau = std::clamp(ident_pid_pitch_.out(), -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);
    out.yaw_target_rad = yaw_target;
    out.pitch_target_rad = pitch_target;

    if (++data_tick_counter_ >= 5) {
      data_tick_counter_ = 0;
      PrepareCsvData(input, out, ident_time_s_);
      out.data_pending = true;
      out.tx_data = tx_buf_;
      out.tx_len = tx_len_;
    }

    return out;
  }

 public:
  // ──────────────────────────────────────────────────────────────────────────
  // 前馈验证 (原有逻辑, 不变)
  // ──────────────────────────────────────────────────────────────────────────

  Output FfVerifyUpdate(const Input &input) {
    if (!ff_verify_active_) {
      ff_verify_time_s_ = 0.0f;
      ff_verify_active_ = true;
    }

    const auto yaw_traj = EvaluateTrajectory(0.0f, ns_ident::kYawAmp, ff_verify_time_s_);
    const auto pitch_traj = EvaluateTrajectory(0.0f, ns_ident::kPitchAmp, ns_ident::kPitchPhase, ff_verify_time_s_);
    const float pitch_q = pitch_traj.q + ns_ident::kIdentPitchCenter;

    float yaw_err = yaw_traj.q - input.yaw_motor_pos_rad;
    if (yaw_err > ns::common::kPi) yaw_err -= 2.0f * ns::common::kPi;
    if (yaw_err < -ns::common::kPi) yaw_err += 2.0f * ns::common::kPi;
    const float yaw_q = input.yaw_motor_pos_rad + yaw_err;

    const Eigen::Vector3f g_stationary(0.0f, 0.0f, -9.81f);
    const auto ff = dynamics_.ComputeFfDecomposed(yaw_q, pitch_q, yaw_traj.dq, pitch_traj.dq, yaw_traj.ddq,
                                                  pitch_traj.ddq, g_stationary);

    ff_verify_time_s_ += input.dt_s;

    Output out{};
    out.yaw_cmd_tau = std::clamp(ff.yaw, -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);
    out.pitch_cmd_tau = std::clamp(ff.pitch, -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);
    out.yaw_target_rad = yaw_q;
    out.pitch_target_rad = pitch_q;
    return out;
  }

 public:
  // ──────────────────────────────────────────────────────────────────────────
  // 公开接口
  // ──────────────────────────────────────────────────────────────────────────

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
    ident_vel_pid_yaw_.SetKp(ns_ident::kIdentYawVelPid.kp)
        .SetKi(ns_ident::kIdentYawVelPid.ki)
        .SetKd(ns_ident::kIdentYawVelPid.kd)
        .SetMaxOut(ns_ident::kIdentYawVelPid.max_out)
        .SetMaxIout(ns_ident::kIdentYawVelPid.max_iout);
    ident_vel_pid_pitch_.SetKp(ns_ident::kIdentPitchVelPid.kp)
        .SetKi(ns_ident::kIdentPitchVelPid.ki)
        .SetKd(ns_ident::kIdentPitchVelPid.kd)
        .SetMaxOut(ns_ident::kIdentPitchVelPid.max_out)
        .SetMaxIout(ns_ident::kIdentPitchVelPid.max_iout);

    Eigen::Matrix<float, 9, 1> theta;
    for (int i = 0; i < 9; ++i) theta(i) = ns::active::gimbal::kIdentTheta[i];
    dynamics_.SetTheta(theta);

    submode_ = ns_ident::kIdentSubMode;
  }

  /** @brief 主辨识入口，根据编译时选择的子模式分发 */
  Output IdentUpdate(const Input &input) {
    switch (submode_) {
      case IdentSubMode::kGravity:
        return GravityUpdate(input);
      case IdentSubMode::kFriction:
        return FrictionUpdate(input);
      case IdentSubMode::kPitchInertia:
        return PitchInertiaUpdate(input);
      case IdentSubMode::kCoupling:
        return CouplingUpdate(input);
      case IdentSubMode::kHarmonic:
        return HarmonicUpdate(input);
      default:
        return HarmonicUpdate(input);
    }
  }

  void Reset() {
    ident_active_ = false;
    ff_verify_active_ = false;
    ident_time_s_ = 0.0f;
    ff_verify_time_s_ = 0.0f;
    ident_pid_yaw_.Clear();
    ident_pid_pitch_.Clear();
    ident_vel_pid_yaw_.Clear();
    ident_vel_pid_pitch_.Clear();
  }
};

}  // namespace gimbal
