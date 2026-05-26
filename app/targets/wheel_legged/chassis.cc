#include "include/chassis/chassis.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "include/params.hpp"

/**
 * @file  targets/wheel_legged/chassis.cc
 * @brief 搴曠洏鎺у埗瀹炵幇锛氱姸鎬佷及璁°€丩QR銆佽ˉ鍋夸笌鍔涚煩杈撳嚭
 */
uint8_t debug_posture_valid;
f32 left_, right_;
namespace {

constexpr rm::f32 kControlDtS = wheel_legged::params::active::chassis::kControlDtS;  ///< 搴曠洏鎺у埗鍛ㄦ湡锛?00Hz锛?

constexpr rm::f32 kLegL1M = wheel_legged::params::active::chassis::kLegL1M;
constexpr rm::f32 kLegL2M = wheel_legged::params::active::chassis::kLegL2M;

constexpr rm::f32 kBodyMassKg = wheel_legged::params::active::chassis::kBodyMassKg;
constexpr rm::f32 kLegMassKg = wheel_legged::params::active::chassis::kLegMassKg;
constexpr rm::f32 kGravityMps2 = wheel_legged::params::active::chassis::kGravityMps2;
constexpr rm::f32 kWheelRadiusM = wheel_legged::params::active::chassis::kWheelRadiusM;
constexpr rm::f32 kOffGroundSupportForceThresholdN =
    wheel_legged::params::active::chassis::kOffGroundSupportForceThresholdN;
constexpr rm::f32 kOffGroundSupportForceClampN = wheel_legged::params::active::chassis::kOffGroundSupportForceClampN;

constexpr rm::f32 kMidLegDipTriggerLengthM = wheel_legged::params::active::chassis::kMidLegDipTriggerLengthM;
constexpr rm::f32 kMidLegDipTargetLengthM = wheel_legged::params::active::chassis::kMidLegDipTargetLengthM;
constexpr uint16_t kMidLegDipHoldTicks = wheel_legged::params::active::chassis::kMidLegDipHoldTicks;

constexpr const auto &kEtaLookupLegLengthM = wheel_legged::params::active::chassis::kEtaLookupLegLengthM;

constexpr const auto &kEtaLookupLwM = wheel_legged::params::active::chassis::kEtaLookupLwM;

constexpr const auto &kCtrlPLow = wheel_legged::params::active::chassis::kCtrlPLow;
constexpr const auto &kCtrlPMid = wheel_legged::params::active::chassis::kCtrlPMid;
constexpr const auto &kCtrlPHigh = wheel_legged::params::active::chassis::kCtrlPHigh;
constexpr const auto &kCtrlPSpin = wheel_legged::params::active::chassis::kCtrlPSpin;

std::array<std::array<rm::f32, 6>, 40> ToCoeffMatrix(const std::array<float, 240> &flat) {
  std::array<std::array<rm::f32, 6>, 40> result{};
  for (int i = 0; i < 40; ++i) {
    std::copy(&flat[i * 6], &flat[i * 6 + 6], result[i].begin());
  }
  return result;
}

/**
 * @brief 根据腿长插值估算腿部等效质心系数
 */
rm::f32 ComputeEtaFromLegLength(const rm::f32 leg_length_m) {
  constexpr size_t kCount = kEtaLookupLegLengthM.size();
  const rm::f32 min_l = kEtaLookupLegLengthM[0];
  const rm::f32 max_l = kEtaLookupLegLengthM[kCount - 1];
  const rm::f32 l = rm::modules::Clamp(leg_length_m, min_l, max_l);

  for (size_t i = 0; i + 1 < kCount; ++i) {
    const rm::f32 l0 = kEtaLookupLegLengthM[i];
    const rm::f32 l1 = kEtaLookupLegLengthM[i + 1];
    if (l >= l0 && l <= l1) {
      const rm::f32 ratio = (l - l0) / (l1 - l0);
      const rm::f32 lw = kEtaLookupLwM[i] + (kEtaLookupLwM[i + 1] - kEtaLookupLwM[i]) * ratio;
      return lw / l;
    }
  }

  return kEtaLookupLwM[kCount - 1] / max_l;
}

/**
 * @brief 左腿弹簧补偿力矩（三次多项式）
 */
rm::f32 ComputeLeftSpringTorque(const rm::f32 leg_length_m) {
  const rm::f32 l2 = leg_length_m * leg_length_m;
  return wheel_legged::params::active::chassis::kLeftSpringC0 +
         wheel_legged::params::active::chassis::kLeftSpringC1 * leg_length_m +
         wheel_legged::params::active::chassis::kLeftSpringC2 * l2 +
         wheel_legged::params::active::chassis::kLeftSpringC3 * l2 * leg_length_m;
}

/**
 * @brief 右腿弹簧补偿力矩（三次多项式）
 */
rm::f32 ComputeRightSpringTorque(const rm::f32 leg_length_m) {
  const rm::f32 l2 = leg_length_m * leg_length_m;
  return wheel_legged::params::active::chassis::kRightSpringC0 +
         wheel_legged::params::active::chassis::kRightSpringC1 * leg_length_m +
         wheel_legged::params::active::chassis::kRightSpringC2 * l2 +
         wheel_legged::params::active::chassis::kRightSpringC3 * l2 * leg_length_m;
}

/**
 * @brief 是否处于强制安全零输出模式
 */
bool IsSafeStopMode(const chassis::Fsm::State mode) { return mode == chassis::Fsm::State::kDisabled; }

bool IsStandbyMode(const chassis::Fsm::State mode) { return mode == chassis::Fsm::State::kStandby; }

constexpr float kDecelZoneRad = wheel_legged::params::active::chassis::kRecoveryDecelZoneRad;
constexpr float kMinSpeedRadS = wheel_legged::params::active::chassis::kRecoveryMinSpeedRadS;
constexpr float kGravityRampScale = wheel_legged::params::active::chassis::kRecoveryGravityRampScale;

/// 计算恢复减速比例：1.0=全速，接近目标边界时线性衰减到 0
inline float RecoveryProximityScale(const float current_angle, const float target_boundary, const float decel_zone) {
  const float dist = std::fabs(current_angle - target_boundary);
  if (dist >= decel_zone) return 1.0f;
  if (dist < 0.03f) return 0.0f;
  return dist / decel_zone;
}

/// 将原始目标速度按 proximity 缩放，边界处不低于 kMinSpeedRadS
inline float ApplyRecoveryDecel(const float raw_target, const float proximity, const float min_speed) {
  const float abs_target = min_speed + (std::fabs(raw_target) - min_speed) * proximity;
  return std::copysign(abs_target, raw_target);
}

}  // namespace

/**
 * @brief 初始化 PID 参数、LQR 系数与状态估计器
 */
void chassis::Chassis::Init() {
  const auto init_pid = [](rm::modules::PID &pid, const rm::f32 kp, const rm::f32 ki, const rm::f32 kd,
                           const rm::f32 max_out, const rm::f32 max_iout) {
    pid.SetKp(kp).SetKi(ki).SetKd(kd).SetMaxOut(max_out).SetMaxIout(max_iout);
    pid.Clear();
  };

  const auto &left_l0_pid = wheel_legged::params::active::chassis::kLeftL0Pid;
  init_pid(left_l0_pid_, left_l0_pid.kp, left_l0_pid.ki, left_l0_pid.kd, left_l0_pid.max_out, left_l0_pid.max_iout);
  const auto &right_l0_pid = wheel_legged::params::active::chassis::kRightL0Pid;
  init_pid(right_l0_pid_, right_l0_pid.kp, right_l0_pid.ki, right_l0_pid.kd, right_l0_pid.max_out,
           right_l0_pid.max_iout);
  const auto &left_l0_jump2 = wheel_legged::params::active::chassis::kLeftL0PidJumpTwo;
  init_pid(left_l0_pid_jump_two_, left_l0_jump2.kp, left_l0_jump2.ki, left_l0_jump2.kd, left_l0_jump2.max_out,
           left_l0_jump2.max_iout);
  const auto &right_l0_jump2 = wheel_legged::params::active::chassis::kRightL0PidJumpTwo;
  init_pid(right_l0_pid_jump_two_, right_l0_jump2.kp, right_l0_jump2.ki, right_l0_jump2.kd, right_l0_jump2.max_out,
           right_l0_jump2.max_iout);
  const auto &left_l0_jump3 = wheel_legged::params::active::chassis::kLeftL0PidJumpThree;
  init_pid(left_l0_pid_jump_three_, left_l0_jump3.kp, left_l0_jump3.ki, left_l0_jump3.kd, left_l0_jump3.max_out,
           left_l0_jump3.max_iout);
  const auto &right_l0_jump3 = wheel_legged::params::active::chassis::kRightL0PidJumpThree;
  init_pid(right_l0_pid_jump_three_, right_l0_jump3.kp, right_l0_jump3.ki, right_l0_jump3.kd, right_l0_jump3.max_out,
           right_l0_jump3.max_iout);
  const auto &left_l0_dip = wheel_legged::params::active::chassis::kLeftL0PidDip;
  init_pid(left_l0_pid_dip_, left_l0_dip.kp, left_l0_dip.ki, left_l0_dip.kd, left_l0_dip.max_out, left_l0_dip.max_iout);
  const auto &right_l0_dip = wheel_legged::params::active::chassis::kRightL0PidDip;
  init_pid(right_l0_pid_dip_, right_l0_dip.kp, right_l0_dip.ki, right_l0_dip.kd, right_l0_dip.max_out,
           right_l0_dip.max_iout);
  const auto &roll_pid = wheel_legged::params::active::chassis::kRollPid;
  init_pid(roll_pid_, roll_pid.kp, roll_pid.ki, roll_pid.kd, roll_pid.max_out, roll_pid.max_iout);
  const auto &left_leg_turn_pid = wheel_legged::params::active::chassis::kLeftLegTurnPid;
  init_pid(left_leg_turn_pid_, left_leg_turn_pid.kp, left_leg_turn_pid.ki, left_leg_turn_pid.kd,
           left_leg_turn_pid.max_out, left_leg_turn_pid.max_iout);
  const auto &right_leg_turn_pid = wheel_legged::params::active::chassis::kRightLegTurnPid;
  init_pid(right_leg_turn_pid_, right_leg_turn_pid.kp, right_leg_turn_pid.ki, right_leg_turn_pid.kd,
           right_leg_turn_pid.max_out, right_leg_turn_pid.max_iout);
  const auto &left_leg_turn_pid_manual = wheel_legged::params::active::chassis::kLeftLegTurnPidManual;
  init_pid(left_leg_turn_pid_manual_, left_leg_turn_pid_manual.kp, left_leg_turn_pid_manual.ki,
           left_leg_turn_pid_manual.kd, left_leg_turn_pid_manual.max_out, left_leg_turn_pid_manual.max_iout);
  const auto &right_leg_turn_pid_manual = wheel_legged::params::active::chassis::kRightLegTurnPidManual;
  init_pid(right_leg_turn_pid_manual_, right_leg_turn_pid_manual.kp, right_leg_turn_pid_manual.ki,
           right_leg_turn_pid_manual.kd, right_leg_turn_pid_manual.max_out, right_leg_turn_pid_manual.max_iout);
  const auto &stair_theta_pid = wheel_legged::params::active::chassis_fsm::kStairClimb.theta_pid;
  init_pid(left_stair_theta_pid_, stair_theta_pid.kp, stair_theta_pid.ki, stair_theta_pid.kd, stair_theta_pid.max_out,
           stair_theta_pid.max_iout);
  init_pid(right_stair_theta_pid_, stair_theta_pid.kp, stair_theta_pid.ki, stair_theta_pid.kd, stair_theta_pid.max_out,
           stair_theta_pid.max_iout);

  left_stair_theta_pid_.SetCircular(true);
  right_stair_theta_pid_.SetCircular(true);
  left_stair_theta_pid_.SetCircularCycle(2.f * M_PI);
  right_stair_theta_pid_.SetCircularCycle(2.f * M_PI);

  lqr_controller_.SetLqrCoefficients(ToCoeffMatrix(kCtrlPLow));
  current_leg_profile_ = wheel_legged::LegProfile::kLow;

  left_l0_ddot_filter_.set_cutoff_frequency(wheel_legged::params::active::chassis::kL0DdotFilterSampleHz,
                                            wheel_legged::params::active::chassis::kL0DdotFilterCutoffHz);
  right_l0_ddot_filter_.set_cutoff_frequency(wheel_legged::params::active::chassis::kL0DdotFilterSampleHz,
                                             wheel_legged::params::active::chassis::kL0DdotFilterCutoffHz);

  left_theta_dot_filter_.set_cutoff_frequency(500.0f, 50.0f);
  right_theta_dot_filter_.set_cutoff_frequency(500.0f, 50.0f);
  theta_dot_filter_initialized_ = true;

  ChassisStateEstimatorConfig cfg{};
  state_estimator_.Init(cfg);
  SafeStop();
  smoothed_leg_target_length_m_ = params_.leg_target_length_m;
  last_ramp_target_m_ = params_.leg_target_length_m;
  ramp_step_per_tick_m_ = 0.0f;
}

/**
 * @brief 将所有执行器输出清零
 */
void chassis::Chassis::SafeStop() {
  output_.lf_tau = 0.0f;
  output_.lb_tau = 0.0f;
  output_.rf_tau = 0.0f;
  output_.rb_tau = 0.0f;
  output_.lw_tau = 0.0f;
  output_.rw_tau = 0.0f;
}

void chassis::Chassis::UpdateLqrCoefficients(const std::array<std::array<rm::f32, 6>, 40> &coeff_matrix) {
  lqr_controller_.SetLqrCoefficients(coeff_matrix);
}

/**
 * @brief 底盘控制单步更新入口
 * @note  包含状态估计更新、腿运动学刷新、支撑力估计与力矩合成。
 */
void chassis::Chassis::Update(const UpdateInput &input) {
  // 先刷新估计器，保证即使输出关闭也能持续导出传感器与状态观测。
  ChassisStateEstimatorInput estimator_input = input.estimator_input;
  estimator_input.dt_s = (estimator_input.dt_s > 0.0f) ? estimator_input.dt_s : kControlDtS;
  estimator_input.s_ref_m = input.expected.s;
  estimator_input.use_external_s_ref = false;
  // estimator_input.use_wheel_speed_direct = (input.fsm_mode == Fsm::State::kSpin);
  estimator_input.use_wheel_speed_direct = false;  // 小陀螺也用卡尔曼滤波速度

  state_estimator_.Update(estimator_input);
  const ChassisStateEstimatorOutput &state_output = state_estimator_.GetOutput();

  // 检测腿长档位变化，切换 LQR 系数矩阵
  {
    wheel_legged::LegProfile new_profile = current_leg_profile_;
    if (input.fsm_mode == Fsm::State::kLowLeg) {
      new_profile = wheel_legged::LegProfile::kLow;
    } else if (input.fsm_mode == Fsm::State::kMidLeg) {
      new_profile = wheel_legged::LegProfile::kMid;
    } else if (input.fsm_mode == Fsm::State::kHighLeg || input.fsm_mode == Fsm::State::kStairTask) {
      new_profile = wheel_legged::LegProfile::kHigh;
    }
    if (input.fsm_mode == Fsm::State::kSpin) {
      UpdateLqrCoefficients(ToCoeffMatrix(kCtrlPSpin));
    } else if (new_profile != current_leg_profile_) {
      current_leg_profile_ = new_profile;
      switch (current_leg_profile_) {
        case wheel_legged::LegProfile::kLow:
          UpdateLqrCoefficients(ToCoeffMatrix(kCtrlPLow));
          break;
        case wheel_legged::LegProfile::kMid:
          UpdateLqrCoefficients(ToCoeffMatrix(kCtrlPMid));
          break;
        case wheel_legged::LegProfile::kHigh:
          UpdateLqrCoefficients(ToCoeffMatrix(kCtrlPHigh));
          break;
      }
    }
  }

  const CalibratedLegKinematicsInput &leg_input = state_output.calibrated_leg_input;
  // 控制器内部复用估计器已标定的腿部角度，避免二次做零位/符号转换。
  left_leg_.SetPhi1(leg_input.left.phi1_rad);
  left_leg_.SetPhi4(leg_input.left.phi4_rad);
  left_leg_.SetWPhi1(leg_input.left.w_phi1_rad_s);
  left_leg_.SetWPhi4(leg_input.left.w_phi4_rad_s);
  left_leg_.Update(estimator_input.dt_s);

  right_leg_.SetPhi1(leg_input.right.phi1_rad);
  right_leg_.SetPhi4(leg_input.right.phi4_rad);
  right_leg_.SetWPhi1(leg_input.right.w_phi1_rad_s);
  right_leg_.SetWPhi4(leg_input.right.w_phi4_rad_s);
  right_leg_.Update(estimator_input.dt_s);

  imu_roll_ = estimator_input.imu.roll_rad;
  imu_acc_x_mps2_ = estimator_input.imu.acc_x_mps2;
  imu_acc_z_mps2_ = estimator_input.imu.acc_z_mps2;
  lf_real_torque_ = estimator_input.left_leg.front.torque_nm;
  lb_real_torque_ = estimator_input.left_leg.back.torque_nm;
  rf_real_torque_ = estimator_input.right_leg.front.torque_nm;
  rb_real_torque_ = estimator_input.right_leg.back.torque_nm;

  output_.current_state = state_output.current;
  output_.mean_leg_length_m = 0.5f * (state_output.left_leg_length_m + state_output.right_leg_length_m);
  output_.left_l0_dot_mps = left_leg_.l0_dot();
  output_.right_l0_dot_mps = right_leg_.l0_dot();
  output_.wheel_speed_mps = state_output.wheel_speed_mps;
  output_.speed_mps = state_output.fused_speed_mps;
  output_.raw_wheel_speed_mps = state_output.raw_wheel_speed_mps;
  output_.raw_accel_speed_mps = state_output.raw_accel_speed_mps;
  output_.current_speed_mps = state_output.current_speed_mps;

  CalSupportForce();
  output_.left_support_force_n = left_support_force_est_n_;
  output_.right_support_force_n = right_support_force_est_n_;

  // 状态估计与调试量已更新；不允许输出时在这里截断所有执行器命令。
  output_.posture_valid =
      (state_output.current.theta_b >= wheel_legged::params::active::chassis::kPostureThetaBMinRad &&
       state_output.current.theta_b <= wheel_legged::params::active::chassis::kPostureThetaBMaxRad &&
       imu_roll_ >= wheel_legged::params::active::chassis::kPostureRollMinRad &&
       imu_roll_ <= wheel_legged::params::active::chassis::kPostureRollMaxRad &&
       // rm::modules::Wrap(state_output.current.theta_ll, -wheel_legged::params::active::kPi,
       // wheel_legged::params::active::kPi) >= wheel_legged::params::active::chassis::kPostureThetaLegMinRad &&
       // rm::modules::Wrap(state_output.current.theta_ll, -wheel_legged::params::active::kPi,
       // wheel_legged::params::active::kPi) <= wheel_legged::params::active::chassis::kPostureThetaLegMaxRad &&
       // rm::modules::Wrap(state_output.current.theta_lr, -wheel_legged::params::active::kPi,
       // wheel_legged::params::active::kPi) >= wheel_legged::params::active::chassis::kPostureThetaLegMinRad &&
       // rm::modules::Wrap(state_output.current.theta_lr, -wheel_legged::params::active::kPi,
       // wheel_legged::params::active::kPi) <= wheel_legged::params::active::chassis::kPostureThetaLegMaxRad);
       state_output.current.theta_ll >= wheel_legged::params::active::chassis::kPostureThetaLegMinRad &&
       state_output.current.theta_ll <= wheel_legged::params::active::chassis::kPostureThetaLegMaxRad &&
       state_output.current.theta_lr >= wheel_legged::params::active::chassis::kPostureThetaLegMinRad &&
       state_output.current.theta_lr <= wheel_legged::params::active::chassis::kPostureThetaLegMaxRad);

  // 进入 kDisabled 时复位起立状态，重新上电后重走起立
  if (input.fsm_mode == Fsm::State::kDisabled) {
    standup_complete_ = false;
    standup_phase_ = 0;
  }

  if (!output_.posture_valid) {
    if (!input.enable_output || !input.run_chassis_update || IsSafeStopMode(input.fsm_mode)) {
      SafeStop();
      prev_enable_output_ = false;
      return;
    }
    prev_enable_output_ = input.enable_output;
    ComputeActuatorTorque(input, state_output);
    if (output_.off_ground_in_mid_high_leg) {
      off_ground_duration_ticks_++;
    } else {
      off_ground_duration_ticks_ = 0;
    }
    output_.off_ground_gravity_off = off_ground_duration_ticks_ >= 50;  // 0.1s
    return;
  }

  if (!input.enable_output || !input.run_chassis_update || IsSafeStopMode(input.fsm_mode)) {
    SafeStop();
    prev_enable_output_ = false;
    return;
  }

  const bool is_recovery_state =
      (input.fsm_mode == Fsm::State::kRecoveryFallCheck || input.fsm_mode == Fsm::State::kRecoverySelfRight);

  // 恢复→正常过渡：强制走起立，收腿+摆角到位后才开轮子
  if (prev_fsm_was_recovery_ && !is_recovery_state) {
    standup_complete_ = false;
    standup_phase_ = 0;
    force_low_leg_ = true;
  }
  prev_fsm_was_recovery_ = is_recovery_state;

  // 起立三段式：q'q
  // Phase 0: 收腿到低腿长，仅使用腿长PID + 弹簧补偿
  // Phase 1: 摆角收敛到阈值内，引擎全开但轮端关
  // Phase 2: 起立完成，锁存
  if (!standup_complete_) {
    constexpr float kThetaThreshold = wheel_legged::params::active::chassis::kStandupThetaThresholdRad;
    constexpr float kRetractLenThresholdM = wheel_legged::params::active::chassis_fsm::kLowLegLengthM + 0.01f;

    if (standup_phase_ == 0) {
      force_low_leg_ = true;
      //      if (left_leg_.l0() < kRetractLenThresholdM && right_leg_.l0() < kRetractLenThresholdM)
      if (left_leg_.l0() + right_leg_.l0() < 2 * kRetractLenThresholdM) {
        standup_phase_ = 1;
      }
    }

    if (standup_phase_ == 1 && std::fabs(state_output.current.theta_ll) < kThetaThreshold &&
        std::fabs(state_output.current.theta_lr) < kThetaThreshold) {
      standup_complete_ = true;
      standup_phase_ = 2;
      force_low_leg_ = false;
    }
  }
  prev_enable_output_ = input.enable_output;
  output_.standup_complete = standup_complete_;

  {
    // 目标变化时重新计算斜坡步长，保证每次腿长切换都走完 kLegLengthRampTimeS
    if (input.motion_target.leg_length_m != last_ramp_target_m_) {
      const float initial_error = input.motion_target.leg_length_m - smoothed_leg_target_length_m_;
      constexpr float kRampTotalTicks = wheel_legged::params::active::chassis_fsm::kLegLengthRampTimeS / kControlDtS;
      ramp_step_per_tick_m_ = std::fabs(initial_error) / kRampTotalTicks;
      last_ramp_target_m_ = input.motion_target.leg_length_m;
    }
    const float error = input.motion_target.leg_length_m - smoothed_leg_target_length_m_;
    if (std::fabs(error) <= ramp_step_per_tick_m_ || ramp_step_per_tick_m_ == 0.0f) {
      smoothed_leg_target_length_m_ = input.motion_target.leg_length_m;
      ramp_step_per_tick_m_ = 0.0f;
    } else {
      smoothed_leg_target_length_m_ += (error > 0.0f ? ramp_step_per_tick_m_ : -ramp_step_per_tick_m_);
    }
    params_.leg_target_length_m = smoothed_leg_target_length_m_;
  }
  if (force_low_leg_) {
    constexpr uint16_t kLowLegHoldTicks = 100;  // 4s @ 500Hz
    params_.leg_target_length_m = wheel_legged::params::active::chassis_fsm::kLowLegLengthM;
    if (standup_phase_ == 0) {
      force_low_leg_ticks_ = 0;  // 起立收腿阶段不自动过期
    } else {
      force_low_leg_ticks_++;
      if (force_low_leg_ticks_ >= kLowLegHoldTicks) {
        force_low_leg_ = false;
        force_low_leg_ticks_ = 0;
      }
    }
  }
  // 中腿长下压：腿长达到阈值后收腿到目标，维持一段时间再恢复
  // 键盘控制时离地后永久锁定低腿长（不自动过期），直到切换模式
  const bool is_mid_leg = (input.fsm_mode == Fsm::State::kMidLeg);
  if (is_mid_leg) {
    if (!mid_leg_dip_active_) {
      // 上升沿触发：腿必须先在阈值以下（armed），再超过阈值才触发下压
      // 避免从高腿长切换到中腿长时立即误触发
      if (output_.mean_leg_length_m < kMidLegDipTriggerLengthM) {
        mid_leg_dip_armed_ = true;
      }
      if (mid_leg_dip_armed_ && output_.mean_leg_length_m >= kMidLegDipTriggerLengthM) {
        mid_leg_dip_active_ = true;
        mid_leg_dip_ticks_ = 0;
        mid_leg_dip_armed_ = false;
      }
    }
    if (mid_leg_dip_active_) {
      params_.leg_target_length_m = kMidLegDipTargetLengthM;
      mid_leg_dip_ticks_++;
      if (mid_leg_dip_ticks_ >= kMidLegDipHoldTicks) {
        mid_leg_dip_active_ = false;
        mid_leg_dip_ticks_ = 0;
        last_ramp_target_m_ = input.motion_target.leg_length_m;
      }
    }
  } else {
    mid_leg_dip_active_ = false;
    mid_leg_dip_armed_ = false;
    mid_leg_dip_ticks_ = 0;
  }
  output_.mid_leg_dip_active = mid_leg_dip_active_;
  output_.leg_target_length_m = params_.leg_target_length_m;
  ComputeActuatorTorque(input, state_output);

  // 离地持续时间计数
  if (output_.off_ground_in_mid_high_leg) {
    off_ground_duration_ticks_++;
  } else {
    off_ground_duration_ticks_ = 0;
  }
  output_.off_ground_gravity_off = off_ground_duration_ticks_ >= 50;  // 0.1s
}

/**
 * @brief 组合 LQR 与补偿项，计算六电机最终力矩
 */
void chassis::Chassis::ComputeActuatorTorque(const UpdateInput &input,
                                             const ChassisStateEstimatorOutput &state_output) {
  output_.off_ground_in_mid_high_leg = false;

  const auto set_all_zero = [this]() {
    output_.lb_tau = 0.0f;
    output_.lf_tau = 0.0f;
    output_.rb_tau = 0.0f;
    output_.rf_tau = 0.0f;
    output_.lw_tau = 0.0f;
    output_.rw_tau = 0.0f;
  };

  if (IsSafeStopMode(input.fsm_mode)) {
    set_all_zero();
    output_.left_force_n = 0.0f;
    output_.right_force_n = 0.0f;
    return;
  }

  // 腿摆角速度滤波后传入 LQR
  auto filtered_state = state_output.current;
  filtered_state.theta_ll_dot = left_theta_dot_filter_.apply(filtered_state.theta_ll_dot);
  filtered_state.theta_lr_dot = right_theta_dot_filter_.apply(filtered_state.theta_lr_dot);
  filtered_theta_ll_dot_ = filtered_state.theta_ll_dot;
  filtered_theta_lr_dot_ = filtered_state.theta_lr_dot;
  output_.filtered_theta_ll_dot = filtered_theta_ll_dot_;
  output_.filtered_theta_lr_dot = filtered_theta_lr_dot_;
  base_torque_ = lqr_controller_.ComputeControl(filtered_state, input.expected);

  const rm::f32 eta_left = ComputeEtaFromLegLength(left_leg_.l0());
  const rm::f32 eta_right = ComputeEtaFromLegLength(right_leg_.l0());
  const rm::f32 effective_mass_left_kg = 0.5f * kBodyMassKg + eta_left * kLegMassKg;
  const rm::f32 effective_mass_right_kg = 0.5f * kBodyMassKg + eta_right * kLegMassKg;
  const rm::f32 gravity_ff_left = effective_mass_left_kg * kGravityMps2;
  const rm::f32 gravity_ff_right = effective_mass_right_kg * kGravityMps2;
  const rm::f32 wheel_radius_m = (kWheelRadiusM > 1e-5f) ? kWheelRadiusM : 1e-5f;

  const rm::f32 avg_leg_length_m = 0.5f * (left_leg_.l0() + right_leg_.l0());
  // 着地后腿长 PID D 项输入放大
  if (input.fsm_mode == Fsm::State::kSpin) {
    constexpr float kSpinLegLengthBiasM = wheel_legged::params::active::control_loop::kSpinLegLengthBiasM;
    left_l0_pid_.UpdateExtDiff(params_.leg_target_length_m + kSpinLegLengthBiasM, left_leg_.l0(), -left_leg_.l0_dot(),
                               2);
    right_l0_pid_.UpdateExtDiff(params_.leg_target_length_m - kSpinLegLengthBiasM, right_leg_.l0(),
                                -right_leg_.l0_dot(), 2);
  } else {
    left_l0_pid_.UpdateExtDiff(params_.leg_target_length_m, avg_leg_length_m, -left_leg_.l0_dot(), 2);
    right_l0_pid_.UpdateExtDiff(params_.leg_target_length_m, avg_leg_length_m, -right_leg_.l0_dot(), 2);
  }
  // 下压目标腿长时使用独立 PID（持续 1s）
  if (mid_leg_dip_active_) {
    left_l0_pid_dip_.UpdateExtDiff(params_.leg_target_length_m, avg_leg_length_m, -left_leg_.l0_dot(), 2);
    right_l0_pid_dip_.UpdateExtDiff(params_.leg_target_length_m, avg_leg_length_m, -right_leg_.l0_dot(), 2);
  }
  output_.left_l0_pid_out = mid_leg_dip_active_ ? left_l0_pid_dip_.out() : left_l0_pid_.out();
  output_.right_l0_pid_out = mid_leg_dip_active_ ? right_l0_pid_dip_.out() : right_l0_pid_.out();
  const rm::f32 length_force_base = 0.5f * (output_.left_l0_pid_out + output_.right_l0_pid_out);

  l_spring_torque_ = ComputeLeftSpringTorque(left_leg_.l0());
  r_spring_torque_ = ComputeRightSpringTorque(right_leg_.l0());

  const bool use_jump_retract1 = (input.fsm_mode == Fsm::State::kJumpPrep);
  const bool use_jump_extend = (input.fsm_mode == Fsm::State::kJumpPush);
  const bool use_jump_retract2 = (input.fsm_mode == Fsm::State::kJumpRecover);
  const bool use_stair_target = input.motion_target.use_stair_theta_controller;
  const bool is_jump_state = use_jump_retract1 || use_jump_extend || use_jump_retract2;

  // 离地 > 0.1s 后去掉重力补偿
  constexpr uint16_t kGravityOffDelayTicks = 50;  // 0.1s @ 500Hz
  const bool off_ground_for_force = off_ground_duration_ticks_ >= kGravityOffDelayTicks;
  const rm::f32 grav_left = off_ground_for_force ? 0.0f : gravity_ff_left;
  const rm::f32 grav_right = off_ground_for_force ? 0.0f : gravity_ff_right;
  debug_posture_valid = output_.posture_valid;
  if (output_.posture_valid) {
    roll_pid_.Update(wheel_legged::params::active::chassis::kRollBalanceTargetRad, imu_roll_);
    rm::f32 leg_length_force = length_force_base;

    // 跳跃阶段分别使用收腿/蹬伸/回收三套腿长控制策略。
    if (use_jump_extend) {
      left_force_ = 250.0f + roll_pid_.out();
      right_force_ = 250.0f - roll_pid_.out();
    } else if (use_jump_retract2) {
      left_l0_pid_jump_three_.UpdateExtDiff(params_.leg_target_length_m, avg_leg_length_m, -left_leg_.l0_dot(), 2);
      right_l0_pid_jump_three_.UpdateExtDiff(params_.leg_target_length_m, avg_leg_length_m, -right_leg_.l0_dot(), 2);
      leg_length_force = 0.5f * (left_l0_pid_jump_three_.out() + right_l0_pid_jump_three_.out());
      left_force_ = leg_length_force + roll_pid_.out() + l_spring_torque_;
      right_force_ = leg_length_force - roll_pid_.out() + r_spring_torque_;
    } else if (use_jump_retract1) {
      left_force_ = leg_length_force + roll_pid_.out() + l_spring_torque_;
      right_force_ = leg_length_force - roll_pid_.out() + r_spring_torque_;
    } else if (!standup_complete_ && standup_phase_ == 0) {
      // 起立收腿阶段：仅腿长PID + 弹簧补偿，不用LQR/重力/roll
      left_force_ = output_.left_l0_pid_out + l_spring_torque_;
      right_force_ = output_.right_l0_pid_out + r_spring_torque_;
    } else {
      // 常规支撑时叠加腿长 PID、重力前馈、横滚补偿、惯性补偿和弹簧补偿。
      const rm::f32 inertial_ff_left = effective_mass_left_kg * (left_leg_.l0() / (2.0f * wheel_radius_m)) *
                                       state_output.current.phi_dot * state_output.current.s_dot;
      const rm::f32 inertial_ff_right = effective_mass_right_kg * (right_leg_.l0() / (2.0f * wheel_radius_m)) *
                                        state_output.current.phi_dot * state_output.current.s_dot;

      left_force_ = output_.left_l0_pid_out + grav_left + roll_pid_.out() - inertial_ff_left + l_spring_torque_;
      right_force_ = output_.right_l0_pid_out + grav_right - roll_pid_.out() + inertial_ff_right + r_spring_torque_;
    }

    const bool off_ground_in_mid_high_leg = !is_jump_state && !mid_leg_dip_active_ &&
                                            input.fsm_mode == Fsm::State::kMidLeg &&
                                            (left_support_force_est_n_ < kOffGroundSupportForceThresholdN ||
                                             right_support_force_est_n_ < kOffGroundSupportForceThresholdN);
    output_.off_ground_in_mid_high_leg = off_ground_in_mid_high_leg;

    // 离地时支持力限幅
    if (off_ground_in_mid_high_leg) {
      left_force_ = std::clamp(left_force_, -kOffGroundSupportForceClampN, kOffGroundSupportForceClampN);
      right_force_ = std::clamp(right_force_, -kOffGroundSupportForceClampN, kOffGroundSupportForceClampN);
    }

    // 动作序列、跳跃回收、离地或起立未完成时关闭轮端力矩。
    if (IsStandbyMode(input.fsm_mode) || use_jump_retract2 || off_ground_in_mid_high_leg ||
        input.motion_target.disable_wheel_torque || !standup_complete_) {
      output_.lw_tau = 0.0f;
      output_.rw_tau = 0.0f;
    } else {
      output_.lw_tau = -base_torque_.t_wl;
      output_.rw_tau = base_torque_.t_wr;
    }

    rm::f32 t_bl_cmd;
    rm::f32 t_br_cmd;
    if (use_stair_target) {
      left_stair_theta_pid_.UpdateExtDiff(input.motion_target.theta_ll_rad, state_output.current.theta_ll,
                                          state_output.current.theta_ll_dot);
      right_stair_theta_pid_.UpdateExtDiff(input.motion_target.theta_lr_rad, state_output.current.theta_lr,
                                           state_output.current.theta_lr_dot);
      t_bl_cmd = -left_stair_theta_pid_.out();
      t_br_cmd = -right_stair_theta_pid_.out();
    } else {
      t_bl_cmd = -base_torque_.t_bl;
      t_br_cmd = -base_torque_.t_br;
    }

    if (standup_phase_ == 0) {
      t_bl_cmd = 0;
      t_br_cmd = 0;
    }

    output_.lb_tau = left_leg_.jacobi_00() * left_force_ + left_leg_.jacobi_01() * t_bl_cmd;
    output_.lf_tau = left_leg_.jacobi_10() * left_force_ + left_leg_.jacobi_11() * t_bl_cmd;
    output_.rb_tau = right_leg_.jacobi_00() * right_force_ + right_leg_.jacobi_01() * t_br_cmd;
    output_.rf_tau = right_leg_.jacobi_10() * right_force_ + right_leg_.jacobi_11() * t_br_cmd;

    output_.lf_tau = -output_.lf_tau;
    output_.lb_tau = -output_.lb_tau;
  } else if (input.recovery_manual_mode) {
    // 手动倒地自启：键盘 A/D/Ctrl+A/D 直接控制腿摆速度，使用独立 PID
    left_leg_turn_pid_manual_.Update(input.manual_left_leg_speed, state_output.current.theta_ll_dot);
    right_leg_turn_pid_manual_.Update(input.manual_right_leg_speed, state_output.current.theta_lr_dot);

    left_force_ = 0.0f;
    right_force_ = 0.0f;

    output_.lb_tau = left_leg_.jacobi_00() * left_force_ + left_leg_.jacobi_01() * (-left_leg_turn_pid_manual_.out());
    output_.lf_tau = left_leg_.jacobi_10() * left_force_ + left_leg_.jacobi_11() * (-left_leg_turn_pid_manual_.out());
    output_.rb_tau =
        right_leg_.jacobi_00() * right_force_ + right_leg_.jacobi_01() * (-right_leg_turn_pid_manual_.out());
    output_.rf_tau =
        right_leg_.jacobi_10() * right_force_ + right_leg_.jacobi_11() * (-right_leg_turn_pid_manual_.out());

    output_.lf_tau = -output_.lf_tau;
    output_.lb_tau = -output_.lb_tau;

    output_.lw_tau = 0.0f;
    output_.rw_tau = 0.0f;
  } else {
    if (state_output.current.theta_b > wheel_legged::params::active::chassis::kPostureThetaBMinRad &&
        state_output.current.theta_b < wheel_legged::params::active::chassis::kPostureThetaBMaxRad &&
        imu_roll_ > wheel_legged::params::active::chassis::kPostureRollMinRad &&
        imu_roll_ < wheel_legged::params::active::chassis::kPostureRollMaxRad) {
      // 腿在安全区间内 → PID 输出置零（不发力，随重力自然下落）
      // 腿在安全区间外 → PID 跟踪恢复速度目标
      constexpr float kThetaLegMin = wheel_legged::params::active::chassis::kPostureThetaLegMinRad;
      constexpr float kThetaLegMax = wheel_legged::params::active::chassis::kPostureThetaLegMaxRad;
      constexpr float kRecoverVel = wheel_legged::params::active::chassis::kLegRecoverThetaDotTarget;

      const bool ll_in_range =
          (state_output.current.theta_ll >= kThetaLegMin && state_output.current.theta_ll <= kThetaLegMax);
      const bool lr_in_range =
          (state_output.current.theta_lr >= kThetaLegMin && state_output.current.theta_lr <= kThetaLegMax);

      float ll_pid_out = 0.0f;
      float lr_pid_out = 0.0f;
      float prox_left = 1.0f;
      float prox_right = 1.0f;
      if (!ll_in_range) {
        const float boundary = (state_output.current.theta_ll < kThetaLegMin) ? kThetaLegMin : kThetaLegMax;
        prox_left = RecoveryProximityScale(state_output.current.theta_ll, boundary, kDecelZoneRad);
        const float scaled_vel = ApplyRecoveryDecel(kRecoverVel, prox_left, kMinSpeedRadS);
        left_leg_turn_pid_.Update(scaled_vel, state_output.current.theta_ll_dot);
        ll_pid_out = -left_leg_turn_pid_.out();
      } else {
        left_leg_turn_pid_.Clear();
      }
      if (!lr_in_range) {
        const float boundary = (state_output.current.theta_lr < kThetaLegMin) ? kThetaLegMin : kThetaLegMax;
        prox_right = RecoveryProximityScale(state_output.current.theta_lr, boundary, kDecelZoneRad);
        const float scaled_vel = ApplyRecoveryDecel(kRecoverVel, prox_right, kMinSpeedRadS);
        right_leg_turn_pid_.Update(scaled_vel, state_output.current.theta_lr_dot);
        lr_pid_out = -right_leg_turn_pid_.out();
      } else {
        right_leg_turn_pid_.Clear();
      }

      const float grav_scale = kGravityRampScale * (1.0f - std::min(prox_left, prox_right));
      left_force_ = effective_mass_left_kg * kGravityMps2 * grav_scale;
      right_force_ = effective_mass_right_kg * kGravityMps2 * grav_scale;

      output_.lb_tau = left_leg_.jacobi_00() * left_force_ + left_leg_.jacobi_01() * ll_pid_out;
      output_.lf_tau = left_leg_.jacobi_10() * left_force_ + left_leg_.jacobi_11() * ll_pid_out;
      output_.rb_tau = right_leg_.jacobi_00() * right_force_ + right_leg_.jacobi_01() * lr_pid_out;
      output_.rf_tau = right_leg_.jacobi_10() * right_force_ + right_leg_.jacobi_11() * lr_pid_out;

      output_.lf_tau = -output_.lf_tau;
      output_.lb_tau = -output_.lb_tau;

      output_.lw_tau = 0.0f;
      output_.rw_tau = 0.0f;
    } else if (state_output.current.theta_b < wheel_legged::params::active::chassis::kPostureThetaBMinRad ||
               state_output.current.theta_b > wheel_legged::params::active::chassis::kPostureThetaBMaxRad) {
      constexpr rm::f32 kVel = wheel_legged::params::active::chassis::kLegRecoverThetaDotTarget;
      constexpr rm::f32 kRangeLowMin = -4.f;
      constexpr rm::f32 kRangeLowMax = -3.5f;
      constexpr rm::f32 kRangeHighMin = -2.2f;
      constexpr rm::f32 kRangeHighMax = -1.8f;
      const rm::f32 lw = state_output.current.theta_ll;
      const rm::f32 rw = state_output.current.theta_lr;

      const bool is_front =
          (state_output.current.theta_b < wheel_legged::params::active::chassis::kPostureThetaBMinRad);
      const rm::f32 tgt_min = is_front ? kRangeLowMin : kRangeHighMin;
      const rm::f32 tgt_max = is_front ? kRangeLowMax : kRangeHighMax;
      const rm::f32 dir = is_front ? kVel : -kVel;

      const bool l_in = (lw >= tgt_min && lw <= tgt_max);
      const bool r_in = (rw >= tgt_min && rw <= tgt_max);

      if (l_in && r_in) {
        left_leg_turn_pid_.Update(dir, state_output.current.theta_ll_dot);
        right_leg_turn_pid_.Update(dir, state_output.current.theta_lr_dot);
      } else {
        const rm::f32 lv = l_in ? 0.0f : dir;
        const rm::f32 rv = r_in ? 0.0f : dir;
        left_leg_turn_pid_.Update(lv, state_output.current.theta_ll_dot);
        right_leg_turn_pid_.Update(rv, state_output.current.theta_lr_dot);
      }
      left_force_ = 0.0f;
      right_force_ = 0.0f;

      output_.lb_tau = left_leg_.jacobi_00() * left_force_ + left_leg_.jacobi_01() * (-left_leg_turn_pid_.out());
      output_.lf_tau = left_leg_.jacobi_10() * left_force_ + left_leg_.jacobi_11() * (-left_leg_turn_pid_.out());
      output_.rb_tau = right_leg_.jacobi_00() * right_force_ + right_leg_.jacobi_01() * (-right_leg_turn_pid_.out());
      output_.rf_tau = right_leg_.jacobi_10() * right_force_ + right_leg_.jacobi_11() * (-right_leg_turn_pid_.out());

      left_ = left_leg_turn_pid_.out();
      right_ = right_leg_turn_pid_.out();

      output_.lf_tau = -output_.lf_tau;
      output_.lb_tau = -output_.lb_tau;

      output_.lw_tau = 0.0f;
      output_.rw_tau = 0.0f;
    } else if (state_output.current.theta_b > wheel_legged::params::active::chassis::kPostureThetaBMinRad &&
               state_output.current.theta_b < wheel_legged::params::active::chassis::kPostureThetaBMaxRad &&
               (imu_roll_ < wheel_legged::params::active::chassis::kPostureRollMinRad ||
                imu_roll_ > wheel_legged::params::active::chassis::kPostureRollMaxRad)) {
      f32 left_leg_turn_pid_out = 0.0f;
      f32 right_leg_turn_pid_out = 0.0f;

      if (imu_roll_ > wheel_legged::params::active::chassis::kPostureRollMaxRad) {
        right_leg_turn_pid_.Update(1.5f * wheel_legged::params::active::chassis::kLegRecoverThetaDotTarget,
                                   state_output.current.theta_lr_dot);
        left_leg_turn_pid_out = 0;
        right_leg_turn_pid_out = -right_leg_turn_pid_.out();
      } else if (imu_roll_ < wheel_legged::params::active::chassis::kPostureRollMinRad) {
        left_leg_turn_pid_.Update(1.5f * wheel_legged::params::active::chassis::kLegRecoverThetaDotTarget,
                                  state_output.current.theta_ll_dot);
        left_leg_turn_pid_out = -left_leg_turn_pid_.out();
        right_leg_turn_pid_out = 0;
      }

      left_force_ = 0.0f;
      right_force_ = 0.0f;

      output_.lb_tau = left_leg_.jacobi_00() * left_force_ + left_leg_.jacobi_01() * left_leg_turn_pid_out;
      output_.lf_tau = left_leg_.jacobi_10() * left_force_ + left_leg_.jacobi_11() * left_leg_turn_pid_out;
      output_.rb_tau = right_leg_.jacobi_00() * right_force_ + right_leg_.jacobi_01() * right_leg_turn_pid_out;
      output_.rf_tau = right_leg_.jacobi_10() * right_force_ + right_leg_.jacobi_11() * right_leg_turn_pid_out;

      output_.lf_tau = -output_.lf_tau;
      output_.lb_tau = -output_.lb_tau;

      output_.lw_tau = 0.0f;
      output_.rw_tau = 0.0f;

    } else {
      set_all_zero();
    }
  }
  output_.left_force_n = left_force_;
  output_.right_force_n = right_force_;
}

/**
 * @brief 根据实测关节力矩估计左右支撑力
 */
void chassis::Chassis::CalSupportForce() {
  static constexpr rm::f32 kPi = wheel_legged::params::active::kPi;

  const rm::f32 det_l = left_leg_.jacobi_00() * left_leg_.jacobi_11() - left_leg_.jacobi_01() * left_leg_.jacobi_10();
  const rm::f32 det_r =
      right_leg_.jacobi_00() * right_leg_.jacobi_11() - right_leg_.jacobi_01() * right_leg_.jacobi_10();

  if (std::fabs(det_l) < 1e-5f || std::fabs(det_r) < 1e-5f || std::fabs(left_leg_.l0()) < 1e-5f ||
      std::fabs(right_leg_.l0()) < 1e-5f) {
    left_support_force_est_n_ = 0.0f;
    right_support_force_est_n_ = 0.0f;
    return;
  }

  const rm::f32 l_f = -(left_leg_.jacobi_11() * lb_real_torque_ - left_leg_.jacobi_01() * lf_real_torque_) / det_l;

  const rm::f32 rf_tau_unified = -rf_real_torque_;
  const rm::f32 rb_tau_unified = -rb_real_torque_;
  const rm::f32 r_f = -(right_leg_.jacobi_11() * rb_tau_unified - right_leg_.jacobi_01() * rf_tau_unified) / det_r;

  const rm::f32 theta_ll = rm::modules::Wrap(output_.current_state.theta_ll, -kPi, kPi);
  const rm::f32 theta_lr = rm::modules::Wrap(output_.current_state.theta_lr, -kPi, kPi);
  const rm::f32 theta_b = rm::modules::Wrap(output_.current_state.theta_b, -kPi, kPi);

  const rm::f32 left_F_bh = l_f * std::cos(theta_ll);
  const rm::f32 right_F_bh = r_f * std::cos(theta_lr);

  const rm::f32 left_l0_ddot_raw = (left_leg_.l0_dot() - left_l0_dot_prev_) / kControlDtS;
  const rm::f32 right_l0_ddot_raw = (right_leg_.l0_dot() - right_l0_dot_prev_) / kControlDtS;
  left_l0_dot_prev_ = left_leg_.l0_dot();
  right_l0_dot_prev_ = right_leg_.l0_dot();

  const rm::f32 left_l0_ddot = left_l0_ddot_filter_.apply(left_l0_ddot_raw);
  const rm::f32 right_l0_ddot = right_l0_ddot_filter_.apply(right_l0_ddot_raw);

  const rm::f32 eta_left = ComputeEtaFromLegLength(left_leg_.l0());
  const rm::f32 eta_right = ComputeEtaFromLegLength(right_leg_.l0());
  const rm::f32 effective_mass_left_kg = 0.5f * kBodyMassKg + eta_left * kLegMassKg;
  const rm::f32 effective_mass_right_kg = 0.5f * kBodyMassKg + eta_right * kLegMassKg;

  const rm::f32 left_leg_dyn_comp = -(1.0f - eta_left) * left_l0_ddot * std::cos(theta_ll);
  const rm::f32 right_leg_dyn_comp = -(1.0f - eta_right) * right_l0_ddot * std::cos(theta_lr);

  // a_c: body vertical acceleration in world frame (z-up), estimated from IMU acceleration and pitch.
  const rm::f32 body_vertical_acc_mps2 =
      imu_acc_x_mps2_ * std::sin(theta_b) + imu_acc_z_mps2_ * std::cos(theta_b) - kGravityMps2;
  const rm::f32 body_vertical_acc_limited_mps2 =
      std::clamp(body_vertical_acc_mps2, -2.0f * kGravityMps2, 2.0f * kGravityMps2);

  const rm::f32 gravity_support_left = effective_mass_left_kg * kGravityMps2;
  const rm::f32 gravity_support_right = effective_mass_right_kg * kGravityMps2;
  const rm::f32 dyn_support_left = effective_mass_left_kg * (body_vertical_acc_limited_mps2 + left_leg_dyn_comp);
  const rm::f32 dyn_support_right = effective_mass_right_kg * (body_vertical_acc_limited_mps2 + right_leg_dyn_comp);

  // left_support_force_est_n_ = left_F_bh + gravity_support_left + dyn_support_left;
  // right_support_force_est_n_ = right_F_bh + gravity_support_right + dyn_support_right;
  left_support_force_est_n_ = left_F_bh + gravity_support_left;
  right_support_force_est_n_ = right_F_bh + gravity_support_right;

  output_.left_F_bh_n = left_F_bh;
  output_.right_F_bh_n = right_F_bh;
  output_.left_gravity_support_n = gravity_support_left;
  output_.right_gravity_support_n = gravity_support_right;
  output_.left_dyn_support_n = dyn_support_left;
  output_.right_dyn_support_n = dyn_support_right;
  output_.left_l0_ddot_mps2 = left_l0_ddot;
  output_.right_l0_ddot_mps2 = right_l0_ddot;
}
