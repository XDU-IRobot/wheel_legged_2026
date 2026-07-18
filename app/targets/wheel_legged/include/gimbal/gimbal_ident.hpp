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
 *   kFriction     — Step 2: Pitch/Yaw 分段匀速扫角，辨识摩擦项 theta[5..8]
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
    bool timing_valid{false};
  };

  struct Output {
    float yaw_cmd_tau{0.0f};
    float pitch_cmd_tau{0.0f};
    float yaw_target_rad{0.0f};
    float pitch_target_rad{0.0f};
    float yaw_target_vel_rad_s{0.0f};
    float pitch_target_vel_rad_s{0.0f};
    float yaw_target_acc_rad_s2{0.0f};
    float pitch_target_acc_rad_s2{0.0f};
    float excitation_frequency_hz{0.0f};
    uint16_t excitation_index{0};
    uint16_t cycle_index{0};
    bool valid_for_fit{false};
    bool data_pending{false};
    const char *tx_data{nullptr};
    size_t tx_len{0};
  };

 private:
  static constexpr uint8_t kCsvDecimation = 10;  // 500 Hz control -> 50 Hz extended CSV
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
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, ",");
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, input.dt_s);
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, ",");
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, output.yaw_target_rad);
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, ",");
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, output.pitch_target_rad - ns_ident::kIdentPitchCenter);
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, ",");
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, output.yaw_target_vel_rad_s);
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, ",");
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, output.pitch_target_vel_rad_s);
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, ",");
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, output.yaw_target_acc_rad_s2);
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, ",");
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, output.pitch_target_acc_rad_s2);
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, ",");
    len += AppendFloat(tx_buf_ + len, sizeof(tx_buf_) - len, output.excitation_frequency_hz);
    len += std::snprintf(tx_buf_ + len, sizeof(tx_buf_) - len, ",%u,%u,%u",
                         static_cast<unsigned>(output.excitation_index), static_cast<unsigned>(output.cycle_index),
                         static_cast<unsigned>(output.valid_for_fit && input.timing_valid));
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
  rm::modules::PID ident_vel_pid_yaw_{};    ///< Friction step yaw 专用速度环 PID
  rm::modules::PID ident_vel_pid_pitch_{};  ///< Friction step pitch 专用速度环 PID
  Gimbal2DofDynamics dynamics_{};

  IdentSubMode submode_{IdentSubMode::kHarmonic};

  // Harmonic / PitchInertia / Coupling 共用
  bool ident_active_{false};
  float ident_time_s_{0.0f};
  float ident_yaw_center_{0.0f};
  float ident_pitch_center_{0.0f};

  size_t pitch_inertia_frequency_idx_{0};
  float pitch_inertia_block_time_s_{0.0f};
  float pitch_inertia_prepare_time_s_{0.0f};
  float pitch_inertia_target_rad_{0.0f};
  bool pitch_inertia_preparing_{true};

  // Gravity 模式
  int grav_angle_idx_{0};
  int grav_sweep_direction_{1};  // +1=正向扫描, -1=反向扫描
  int grav_measurement_idx_{0};  // 往返序列中的测量序号，用于生成单调时间戳
  int grav_phase_{0};            // 0=settle, 1=measure, 2=done
  float grav_phase_time_{0.0f};

  static constexpr int kFricAxisPitch = 0;
  static constexpr int kFricAxisPitchCenter = 1;
  static constexpr int kFricAxisYawPositive = 2;
  static constexpr int kFricAxisYawNegative = 3;
  static constexpr int kFricAxisDone = 4;

  // Friction 模式
  int fric_axis_{kFricAxisPitch};
  // 0=pitch, 1=pitch 回中, 2=yaw+, 3=yaw-, 4=done
  int fric_phase_{0};  // pitch: 0=准备,1=正扫,2=暂停,3=反扫,4=暂停; yaw: 0=匀速,1=暂停
  float fric_phase_time_{0.0f};
  float fric_travel_{0.0f};  // 累计角位移 [rad] (绝对值, yaw 用)
  size_t fric_pitch_velocity_idx_{0};
  size_t fric_yaw_velocity_idx_{0};
  float fric_pitch_target_{0.0f};
  float fric_yaw_hold_target_{0.0f};

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
    grav_sweep_direction_ = 1;
    grav_measurement_idx_ = 0;
    grav_phase_ = 0;
    grav_phase_time_ = 0.0f;
    ident_active_ = true;
    data_tick_counter_ = 0;
  }

  void StartFriction(const Input &input) {
    ResetIdentState();
    fric_axis_ = kFricAxisPitch;
    fric_phase_ = 0;
    fric_phase_time_ = 0.0f;
    fric_travel_ = 0.0f;
    fric_pitch_velocity_idx_ = 0;
    fric_yaw_velocity_idx_ = 0;
    fric_pitch_target_ =
        std::clamp(input.pitch_motor_pos_rad, ns_ident::kIdentPitchTopLimit, ns_ident::kIdentPitchBottomLimit);
    fric_yaw_hold_target_ = input.yaw_motor_pos_rad;
    ident_active_ = true;
    data_tick_counter_ = 0;
  }

  void StartPitchInertia(const Input &input) {
    ResetIdentState();
    pitch_inertia_frequency_idx_ = 0;
    pitch_inertia_block_time_s_ = 0.0f;
    pitch_inertia_prepare_time_s_ = 0.0f;
    pitch_inertia_target_rad_ =
        std::clamp(input.pitch_motor_pos_rad, ns_ident::kIdentPitchTopLimit, ns_ident::kIdentPitchBottomLimit);
    pitch_inertia_preparing_ = true;
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
      // 正向和反向扫描均完成，保持往返序列的终点，不再发送 CSV
      const float q2 = ns_ident::kGravityPitchAngles[0];
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
      ++grav_measurement_idx_;

      const int last_angle_idx = static_cast<int>(ns_ident::kGravityAngleCount) - 1;
      if (grav_sweep_direction_ > 0 && grav_angle_idx_ < last_angle_idx) {
        ++grav_angle_idx_;
      } else if (grav_sweep_direction_ > 0 && last_angle_idx > 0) {
        // 最高角已经测过，不重复停留；直接从相邻角开始反向扫描。
        grav_sweep_direction_ = -1;
        --grav_angle_idx_;
      } else if (grav_sweep_direction_ < 0 && grav_angle_idx_ > 0) {
        --grav_angle_idx_;
      } else {
        grav_phase_ = 2;
      }

      if (grav_phase_ != 2) {
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
      if (++data_tick_counter_ >= kCsvDecimation) {
        data_tick_counter_ = 0;
        PrepareCsvData(input, out,
                       grav_phase_time_ +
                           grav_measurement_idx_ * (ns_ident::kGravityHoldDuration + ns_ident::kGravitySettleDuration));
        out.data_pending = true;
        out.tx_data = tx_buf_;
        out.tx_len = tx_len_;
      }
    }

    return out;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Step 2: Pitch 多速度正反扫角联合辨识重力/摩擦 + Yaw 多速度摩擦
  // ──────────────────────────────────────────────────────────────────────────

  Output FrictionUpdate(const Input &input) {
    if (!ident_active_) StartFriction(input);

    if (fric_axis_ == kFricAxisDone) {
      Output out{};
      out.yaw_cmd_tau = 0.0f;
      out.pitch_cmd_tau = 0.0f;
      out.yaw_target_rad = input.yaw_motor_pos_rad;
      out.pitch_target_rad = ns_ident::kIdentPitchCenter;
      return out;
    }

    float cmd_yaw = 0.0f, cmd_pitch = 0.0f;
    float yaw_target, pitch_target;
    bool emit_data = true;

    if (fric_axis_ == kFricAxisPitch) {
      // ── Pitch 联合辨识: 每个速度档均执行 Top→Bottom 正扫和 Bottom→Top 反扫 ──
      // 位置斜坡 + 位置 PID (带积分) 控制恒定速度。
      // 相比于速度环, 位置环天然补偿 pitch 轴的位置相关重力负载。
      // 准备、端点停顿和转向阶段不发送 CSV，避免加减速数据进入重力/摩擦联合回归。
      const float top = ns_ident::kIdentPitchTopLimit;
      const float bottom = ns_ident::kIdentPitchBottomLimit;
      const float sweep_velocity = ns_ident::kFrictionPitchVelocitiesRadS[fric_pitch_velocity_idx_];
      fric_phase_time_ += input.dt_s;

      if (fric_phase_ == 0) {
        // 先移动到 Top 保护端，稳定后才开始第一个正向匀速段。
        emit_data = false;
        const float delta = top - fric_pitch_target_;
        const float step = ns_ident::kFrictionPitchMoveVelocityRadS * input.dt_s;
        if (std::fabs(delta) <= step) {
          fric_pitch_target_ = top;
        } else {
          fric_pitch_target_ += (delta > 0.0f ? step : -step);
        }
        const bool ready = std::fabs(fric_pitch_target_ - top) < 1e-4f &&
                           std::fabs(input.pitch_motor_pos_rad - top) < 0.03f &&
                           std::fabs(input.pitch_motor_vel_rad_s) < 0.08f;
        if (ready && fric_phase_time_ >= ns_ident::kFrictionPauseDuration) {
          fric_phase_ = 1;
          fric_phase_time_ = 0.0f;
          ident_pid_pitch_.Clear();
          data_tick_counter_ = 0;
        }
      } else if (fric_phase_ == 1) {
        // 正扫: Top → Bottom, 位置斜坡 + PID 跟踪。
        fric_pitch_target_ = std::min(fric_pitch_target_ + sweep_velocity * input.dt_s, bottom);
        if (fric_pitch_target_ >= bottom) {
          fric_phase_ = 2;
          fric_phase_time_ = 0.0f;
          ident_pid_pitch_.Clear();
          emit_data = false;
        }
      } else if (fric_phase_ == 2) {
        // Bottom 端稳定停顿，随后反向。
        emit_data = false;
        fric_pitch_target_ = bottom;
        if (fric_phase_time_ >= ns_ident::kFrictionPauseDuration && std::fabs(input.pitch_motor_vel_rad_s) < 0.08f) {
          fric_phase_ = 3;
          fric_phase_time_ = 0.0f;
          ident_pid_pitch_.Clear();
          data_tick_counter_ = 0;
        }
      } else if (fric_phase_ == 3) {
        // 反扫: Bottom → Top, 位置斜坡 + PID 跟踪。
        fric_pitch_target_ = std::max(fric_pitch_target_ - sweep_velocity * input.dt_s, top);
        if (fric_pitch_target_ <= top) {
          fric_phase_ = 4;
          fric_phase_time_ = 0.0f;
          ident_pid_pitch_.Clear();
          emit_data = false;
        }
      } else {
        // Top 端稳定停顿；切换到下一速度档，全部完成后进入回中阶段。
        emit_data = false;
        fric_pitch_target_ = top;
        if (fric_phase_time_ >= ns_ident::kFrictionPauseDuration && std::fabs(input.pitch_motor_vel_rad_s) < 0.08f) {
          ++fric_pitch_velocity_idx_;
          if (fric_pitch_velocity_idx_ >= ns_ident::kFrictionPitchVelocityCount) {
            fric_axis_ = kFricAxisPitchCenter;
          } else {
            fric_phase_ = 1;
          }
          fric_phase_time_ = 0.0f;
          ident_pid_pitch_.Clear();
          data_tick_counter_ = 0;
        }
      }

      yaw_target = WrapYawTarget(fric_yaw_hold_target_, input.yaw_motor_pos_rad);
      pitch_target = fric_pitch_target_;
      ident_pid_yaw_.Update(yaw_target, input.yaw_motor_pos_rad, input.dt_s);
      ident_pid_pitch_.Update(pitch_target, input.pitch_motor_pos_rad, input.dt_s);
      cmd_yaw = ident_pid_yaw_.out();
      cmd_pitch = ident_pid_pitch_.out();

      // Pitch 重力+摩擦前馈 + 常值偏置
      {
        const float pitch_q_model = input.pitch_motor_pos_rad - ns_ident::kIdentPitchCenter;
        const Eigen::Vector3f g_vec(0.0f, 0.0f, -9.81f);
        const auto ff = dynamics_.ComputeFfDecomposed(yaw_target, pitch_q_model, 0.f, 0.f, 0.f, 0.f, g_vec);
        cmd_yaw += ff.yaw;
        cmd_pitch += ff.pitch + ns_gimbal::kPitchFeedforwardBiasNm;
      }
    } else if (fric_axis_ == kFricAxisPitchCenter) {
      // ── Pitch 回中: 不发送 CSV, 避免过渡段混入摩擦数据 ──
      emit_data = false;
      yaw_target = WrapYawTarget(fric_yaw_hold_target_, input.yaw_motor_pos_rad);
      const float center_target =
          std::clamp(ns_ident::kIdentPitchCenter, ns_ident::kIdentPitchTopLimit, ns_ident::kIdentPitchBottomLimit);
      const float center_delta = center_target - fric_pitch_target_;
      const float center_step = ns_ident::kFrictionPitchMoveVelocityRadS * input.dt_s;
      if (std::fabs(center_delta) <= center_step) {
        fric_pitch_target_ = center_target;
      } else {
        fric_pitch_target_ += (center_delta > 0.0f ? center_step : -center_step);
      }
      pitch_target = fric_pitch_target_;
      ident_pid_yaw_.Update(yaw_target, input.yaw_motor_pos_rad, input.dt_s);
      ident_pid_pitch_.Update(pitch_target, input.pitch_motor_pos_rad, input.dt_s);
      cmd_yaw = ident_pid_yaw_.out();
      cmd_pitch = ident_pid_pitch_.out();

      {
        const float pitch_q_model = input.pitch_motor_pos_rad - ns_ident::kIdentPitchCenter;
        const Eigen::Vector3f g_vec(0.0f, 0.0f, -9.81f);
        const auto ff = dynamics_.ComputeFfDecomposed(yaw_target, pitch_q_model, 0.f, 0.f, 0.f, 0.f, g_vec);
        cmd_yaw += ff.yaw;
        cmd_pitch += ff.pitch + ns_gimbal::kPitchFeedforwardBiasNm;
      }

      fric_phase_time_ += input.dt_s;
      const bool target_centered = std::fabs(fric_pitch_target_ - center_target) < 1e-4f;
      const bool pitch_centered = target_centered && std::fabs(input.pitch_motor_pos_rad - center_target) < 0.03f &&
                                  std::fabs(input.pitch_motor_vel_rad_s) < 0.08f;
      if (pitch_centered && fric_phase_time_ >= ns_ident::kFrictionPauseDuration) {
        fric_axis_ = kFricAxisYawPositive;
        fric_phase_ = 0;
        fric_phase_time_ = 0.0f;
        fric_travel_ = 0.0f;
        ident_pid_pitch_.Clear();
        ident_vel_pid_yaw_.Clear();
        data_tick_counter_ = 0;
      }
    } else {
      // ── Yaw: 正/反各一圈, Pitch 位置保持 ──
      const float v_abs = ns_ident::kFrictionYawVelocitiesRadS[fric_yaw_velocity_idx_];
      const float v = (fric_axis_ == kFricAxisYawNegative) ? -v_abs : v_abs;
      const float target_dist = ns_ident::kFrictionYawTravelRad;
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
          if (fric_axis_ == kFricAxisYawPositive) {
            fric_axis_ = kFricAxisYawNegative;
            ident_vel_pid_yaw_.Clear();
          } else {
            ++fric_yaw_velocity_idx_;
            if (fric_yaw_velocity_idx_ >= ns_ident::kFrictionYawVelocityCount) {
              fric_axis_ = kFricAxisDone;
              ident_pid_pitch_.Clear();
            } else {
              fric_axis_ = kFricAxisYawPositive;
            }
            ident_vel_pid_yaw_.Clear();
          }
        }
      }
    }

    Output out{};
    out.yaw_cmd_tau = std::clamp(cmd_yaw, -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);
    out.pitch_cmd_tau = std::clamp(cmd_pitch, -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);
    out.yaw_target_rad = yaw_target;
    out.pitch_target_rad = pitch_target;

    if (emit_data && ++data_tick_counter_ >= kCsvDecimation) {
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
    if (!ident_active_) StartPitchInertia(input);
    const float center = ns_ident::kIdentPitchCenter;
    const float yaw_target = WrapYawTarget(0.0f, input.yaw_motor_pos_rad);

    Output out{};
    out.yaw_target_rad = yaw_target;

    if (pitch_inertia_preparing_) {
      const float delta = center - pitch_inertia_target_rad_;
      const float step = ns_ident::kPitchInertiaPrepareVelocityRadS * input.dt_s;
      if (std::fabs(delta) <= step) {
        pitch_inertia_target_rad_ = center;
      } else {
        pitch_inertia_target_rad_ += (delta > 0.0f ? step : -step);
      }
      const bool centered =
          std::fabs(input.pitch_motor_pos_rad - center) < 0.02f && std::fabs(input.pitch_motor_vel_rad_s) < 0.08f;
      pitch_inertia_prepare_time_s_ = centered ? pitch_inertia_prepare_time_s_ + input.dt_s : 0.0f;
      if (centered && pitch_inertia_prepare_time_s_ >= ns_ident::kPitchInertiaPrepareDurationS) {
        pitch_inertia_preparing_ = false;
        pitch_inertia_block_time_s_ = 0.0f;
        pitch_inertia_prepare_time_s_ = 0.0f;
        ident_pid_pitch_.Clear();
      }
      out.pitch_target_rad = pitch_inertia_target_rad_;
    } else if (pitch_inertia_frequency_idx_ < ns_ident::kPitchInertiaFrequencyCount) {
      const float frequency = ns_ident::kPitchInertiaFrequenciesHz[pitch_inertia_frequency_idx_];
      const float w = 2.0f * ns::common::kPi * frequency;
      const float phase = w * pitch_inertia_block_time_s_;
      const float amplitude = ns_ident::kPitchInertiaAmplitude;
      const float q2_ref = center + amplitude * std::sin(phase);
      out.pitch_target_rad = std::clamp(q2_ref, ns_ident::kIdentPitchTopLimit, ns_ident::kIdentPitchBottomLimit);
      out.pitch_target_vel_rad_s = amplitude * w * std::cos(phase);
      out.pitch_target_acc_rad_s2 = -amplitude * w * w * std::sin(phase);
      out.excitation_frequency_hz = frequency;
      out.excitation_index = static_cast<uint16_t>(pitch_inertia_frequency_idx_);
      const uint16_t cycle = static_cast<uint16_t>(pitch_inertia_block_time_s_ * frequency);
      out.cycle_index = cycle;
      out.valid_for_fit = cycle >= ns_ident::kPitchInertiaWarmupCycles &&
                          cycle < ns_ident::kPitchInertiaWarmupCycles + ns_ident::kPitchInertiaRecordCycles;

      pitch_inertia_block_time_s_ += input.dt_s;
      const float block_duration =
          static_cast<float>(ns_ident::kPitchInertiaWarmupCycles + ns_ident::kPitchInertiaRecordCycles) / frequency;
      if (pitch_inertia_block_time_s_ >= block_duration) {
        ++pitch_inertia_frequency_idx_;
        pitch_inertia_block_time_s_ = 0.0f;
        ident_pid_pitch_.Clear();
      }
    } else {
      out.pitch_target_rad = center;
      out.valid_for_fit = false;
    }

    ident_pid_yaw_.Update(yaw_target, input.yaw_motor_pos_rad, input.dt_s);
    ident_pid_pitch_.Update(out.pitch_target_rad, input.pitch_motor_pos_rad, input.dt_s);

    ident_time_s_ += input.dt_s;

    out.yaw_cmd_tau = std::clamp(ident_pid_yaw_.out(), -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);
    out.pitch_cmd_tau = std::clamp(ident_pid_pitch_.out(), -ns_ident::kDmTorqueLimitNm, ns_ident::kDmTorqueLimitNm);

    // The extended CSV line is larger than the legacy seven-column line.
    // 50 Hz is sufficient for the 1 Hz excitation and stays below 115200-baud throughput.
    if (++data_tick_counter_ >= kCsvDecimation) {
      data_tick_counter_ = 0;
      PrepareCsvData(input, out, ident_time_s_);
      out.data_pending = true;
      out.tx_data = tx_buf_;
      out.tx_len = tx_len_;
    }

    return out;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Step 4: 耦合惯量 — Pitch 固定 + Yaw 大振幅正弦 (Python 两步法: 每角度 I_eff → 跨角度 I1zz/I2xx)
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

    // Yaw 正弦轨迹 + 解析速度/加速度 (供 CSV ddq1_ref 输出, Python 两步法辨识使用)
    const float yaw_continuous = A * std::sin(w * coup_angle_time_);
    const float yaw_vel_continuous = A * w * std::cos(w * coup_angle_time_);
    const float yaw_acc_continuous = -A * w * w * std::sin(w * coup_angle_time_);

    const float yaw_target = WrapYawTarget(yaw_continuous, input.yaw_motor_pos_rad);
    const float pitch_target = std::clamp(pitch_fixed, ns_ident::kIdentPitchTopLimit, ns_ident::kIdentPitchBottomLimit);

    ident_pid_yaw_.Update(yaw_target, input.yaw_motor_pos_rad, input.dt_s);
    ident_pid_pitch_.Update(pitch_target, input.pitch_motor_pos_rad, input.dt_s);

    coup_angle_time_ += input.dt_s;
    ident_time_s_ += input.dt_s;

    // 周期计数与数据有效性标记
    const uint16_t cycle = static_cast<uint16_t>(coup_angle_time_ * ns_ident::kCouplingFreqHz);
    const bool valid = cycle >= ns_ident::kCouplingWarmupCycles &&
                       cycle < ns_ident::kCouplingWarmupCycles + ns_ident::kCouplingRecordCycles;

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
    out.yaw_target_vel_rad_s = yaw_vel_continuous;
    out.yaw_target_acc_rad_s2 = yaw_acc_continuous;
    out.excitation_frequency_hz = ns_ident::kCouplingFreqHz;
    out.excitation_index = static_cast<uint16_t>(coup_angle_idx_);
    out.cycle_index = cycle;
    out.valid_for_fit = valid && input.timing_valid;

    if (++data_tick_counter_ >= kCsvDecimation) {
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

    if (++data_tick_counter_ >= kCsvDecimation) {
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
