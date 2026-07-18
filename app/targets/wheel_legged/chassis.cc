#include "include/chassis/chassis.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "include/params.hpp"
#include "include/state_ctx.hpp"

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
constexpr rm::f32 kWheelPhysicalRadiusM = wheel_legged::params::active::state_estimator::kWheelRadiusM;
constexpr rm::f32 kOffGroundSupportForceThresholdN =
    wheel_legged::params::active::chassis::kOffGroundSupportForceThresholdN;
constexpr rm::f32 kOffGroundSupportForceClampN = wheel_legged::params::active::chassis::kOffGroundSupportForceClampN;

constexpr rm::f32 kMidLegDipTriggerLengthM = wheel_legged::params::active::chassis::kMidLegDipTriggerLengthM;
constexpr rm::f32 kMidLegDipTargetLengthM = wheel_legged::params::active::chassis::kMidLegDipTargetLengthM;
constexpr uint16_t kMidLegDipHoldTicks = wheel_legged::params::active::chassis::kMidLegDipHoldTicks;

constexpr const auto &kEtaLookupLegLengthM = wheel_legged::params::active::chassis::kEtaLookupLegLengthM;

constexpr const auto &kEtaLookupLwM = wheel_legged::params::active::chassis::kEtaLookupLwM;

constexpr const auto &kCtrlPLow = wheel_legged::params::active::chassis::kCtrlPLow;

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
constexpr float kJumpPushForceN = wheel_legged::params::active::chassis::kJumpPushForceN;

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

/// 检查 theta 是否在环形区间 [min, max] 内（跨 2π 等价角也正确处理）
inline bool ThetaInRange(const float theta, const float min, const float max) {
  constexpr float kTwoPi = 2.0f * static_cast<float>(M_PI);
  float a = std::fmod(theta - min, kTwoPi);
  if (a < 0) a += kTwoPi;
  return a <= (max - min);
}

/// 返回 theta 到环形区间 [min, max] 较近边界值的距离（带符号：正=往max方向）
inline float ThetaDistToNearestBoundary(const float theta, const float min, const float max) {
  if (ThetaInRange(theta, min, max)) return 0.0f;
  constexpr float kTwoPi = 2.0f * static_cast<float>(M_PI);
  const float dist_to_min = std::fmod(theta - min + kTwoPi, kTwoPi);
  const float dist_to_max = std::fmod(max - theta + kTwoPi, kTwoPi);
  return dist_to_min < dist_to_max ? -dist_to_min : dist_to_max;
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
  const auto &left_l0_jump1 = wheel_legged::params::active::chassis::kLeftL0PidJumpOne;
  init_pid(left_l0_pid_jump_one_, left_l0_jump1.kp, left_l0_jump1.ki, left_l0_jump1.kd, left_l0_jump1.max_out,
           left_l0_jump1.max_iout);
  const auto &right_l0_jump1 = wheel_legged::params::active::chassis::kRightL0PidJumpOne;
  init_pid(right_l0_pid_jump_one_, right_l0_jump1.kp, right_l0_jump1.ki, right_l0_jump1.kd, right_l0_jump1.max_out,
           right_l0_jump1.max_iout);
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
  const auto &left_leg_angle_pid_standup = wheel_legged::params::active::chassis::kLeftLegAnglePidStandup;
  init_pid(left_leg_angle_pid_standup_, left_leg_angle_pid_standup.kp, left_leg_angle_pid_standup.ki,
           left_leg_angle_pid_standup.kd, left_leg_angle_pid_standup.max_out, left_leg_angle_pid_standup.max_iout);
  const auto &right_leg_angle_pid_standup = wheel_legged::params::active::chassis::kRightLegAnglePidStandup;
  init_pid(right_leg_angle_pid_standup_, right_leg_angle_pid_standup.kp, right_leg_angle_pid_standup.ki,
           right_leg_angle_pid_standup.kd, right_leg_angle_pid_standup.max_out, right_leg_angle_pid_standup.max_iout);
  const auto &right_leg_turn_pid = wheel_legged::params::active::chassis::kRightLegTurnPid;
  init_pid(right_leg_turn_pid_, right_leg_turn_pid.kp, right_leg_turn_pid.ki, right_leg_turn_pid.kd,
           right_leg_turn_pid.max_out, right_leg_turn_pid.max_iout);
  const auto &left_leg_turn_pid_manual = wheel_legged::params::active::chassis::kLeftLegTurnPidManual;
  init_pid(left_leg_turn_pid_manual_, left_leg_turn_pid_manual.kp, left_leg_turn_pid_manual.ki,
           left_leg_turn_pid_manual.kd, left_leg_turn_pid_manual.max_out, left_leg_turn_pid_manual.max_iout);
  const auto &right_leg_turn_pid_manual = wheel_legged::params::active::chassis::kRightLegTurnPidManual;
  init_pid(right_leg_turn_pid_manual_, right_leg_turn_pid_manual.kp, right_leg_turn_pid_manual.ki,
           right_leg_turn_pid_manual.kd, right_leg_turn_pid_manual.max_out, right_leg_turn_pid_manual.max_iout);

  const auto &left_leg_angle_pid_stair = wheel_legged::params::active::chassis::kLeftLegAnglePidStandup;
  init_pid(left_stair_theta_pid_, left_leg_angle_pid_stair.kp, left_leg_angle_pid_stair.ki, left_leg_angle_pid_stair.kd,
           left_leg_angle_pid_stair.max_out, left_leg_angle_pid_stair.max_iout);
  const auto &right_leg_angle_pid_stair = wheel_legged::params::active::chassis::kRightLegAnglePidStandup;
  init_pid(right_stair_theta_pid_, right_leg_angle_pid_stair.kp, right_leg_angle_pid_stair.ki,
           right_leg_angle_pid_stair.kd, right_leg_angle_pid_stair.max_out, right_leg_angle_pid_stair.max_iout);
  left_stair_theta_pid_.SetCircular(true);
  right_stair_theta_pid_.SetCircular(true);
  left_stair_theta_pid_.SetCircularCycle(2.f * M_PI);
  right_stair_theta_pid_.SetCircularCycle(2.f * M_PI);

  const auto &left_leg_angle_pid_jump_retract2 = wheel_legged::params::active::chassis::kLeftLegAnglePidJumpRetract2;
  init_pid(left_leg_angle_pid_jump_retract2_, left_leg_angle_pid_jump_retract2.kp, left_leg_angle_pid_jump_retract2.ki,
           left_leg_angle_pid_jump_retract2.kd, left_leg_angle_pid_jump_retract2.max_out,
           left_leg_angle_pid_jump_retract2.max_iout);
  const auto &right_leg_angle_pid_jump_retract2 = wheel_legged::params::active::chassis::kRightLegAnglePidJumpRetract2;
  init_pid(right_leg_angle_pid_jump_retract2_, right_leg_angle_pid_jump_retract2.kp,
           right_leg_angle_pid_jump_retract2.ki, right_leg_angle_pid_jump_retract2.kd,
           right_leg_angle_pid_jump_retract2.max_out, right_leg_angle_pid_jump_retract2.max_iout);

  lqr_controller_.SetLqrCoefficients(ToCoeffMatrix(kCtrlPLow));

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
  imu_acc_x_integral_mps_ = 0.0f;
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
  imu_acc_x_integral_mps_ += imu_acc_x_mps2_ * estimator_input.dt_s;
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
  output_.filtered_wheel_speed_mps = state_output.filtered_wheel_speed_mps;
  output_.speed_mps = state_output.fused_speed_mps;
  output_.raw_wheel_speed_mps = state_output.raw_wheel_speed_mps;
  output_.raw_accel_speed_mps = state_output.raw_accel_speed_mps;
  output_.imu_acc_x_integral_mps = imu_acc_x_integral_mps_;
  output_.current_speed_mps = state_output.current_speed_mps;

  CalSupportForce();
  output_.left_support_force_n = left_support_force_est_n_;
  output_.right_support_force_n = right_support_force_est_n_;

  // 状态估计与调试量已更新；不允许输出时在这里截断所有执行器命令。
  const bool pitch_roll_valid =
      (state_output.current.theta_b >= wheel_legged::params::active::chassis::kPostureThetaBMinRad &&
       state_output.current.theta_b <= wheel_legged::params::active::chassis::kPostureThetaBMaxRad &&
       imu_roll_ >= wheel_legged::params::active::chassis::kPostureRollMinRad &&
       imu_roll_ <= wheel_legged::params::active::chassis::kPostureRollMaxRad);
  const bool theta_valid =
      (ThetaInRange(state_output.current.theta_ll, wheel_legged::params::active::chassis::kPostureThetaLegMinRad,
                    wheel_legged::params::active::chassis::kPostureThetaLegMaxRad) &&
       ThetaInRange(state_output.current.theta_lr, wheel_legged::params::active::chassis::kPostureThetaLegMinRad,
                    wheel_legged::params::active::chassis::kPostureThetaLegMaxRad));
  output_.posture_valid = pitch_roll_valid && theta_valid;
  output_.pitch_roll_valid_theta_invalid = pitch_roll_valid && !theta_valid;

  // 进入 kDisabled 时复位起立状态，重新上电后重走起立
  if (input.fsm_mode == Fsm::State::kDisabled) {
    standup_complete_ = false;
    standup_phase_ = 0;
    standup_theta_target_ = 0.0f;
    trigger_standup_latched_ = false;
    standup_from_recovery_latch_ = false;
  }

  // 台阶 step2 hook 完成后触发起立
  if (input.motion_target.trigger_standup && !trigger_standup_latched_ && standup_complete_) {
    standup_complete_ = false;
    standup_phase_ = 0;
    force_low_leg_ = true;
    trigger_standup_latched_ = true;
  }
  if (standup_complete_) {
    trigger_standup_latched_ = false;
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

  // 姿态恢复时重置 theta 恢复阶段
  theta_recovery_phase_ = 0;

  if (!input.enable_output || !input.run_chassis_update || IsSafeStopMode(input.fsm_mode)) {
    SafeStop();
    prev_enable_output_ = false;
    return;
  }

  const bool is_recovery_state =
      (input.fsm_mode == Fsm::State::kRecoveryFallCheck || input.fsm_mode == Fsm::State::kRecoverySelfRight);

  // 恢复→正常过渡
  if (prev_fsm_was_recovery_ && !is_recovery_state) {
    if (standup_from_recovery_latch_) {
      // theta恢复已完成并设置起立Phase 1，跳过standup复位
      standup_from_recovery_latch_ = false;
    } else {
      standup_complete_ = false;
      standup_phase_ = 0;
      force_low_leg_ = true;
    }
    theta_recovery_active_ = false;
  }
  prev_fsm_was_recovery_ = is_recovery_state;

  // 起立三段式
  // Phase 0: 收腿 + 摆角 PID 到目标值，腿长PID + 弹簧补偿
  // Phase 1: 腿长 0.1m + 摆角斜坡回 0，完成后轮端开放
  // Phase 2: 起立完成，锁存
  if (!standup_complete_) {
    constexpr float kThetaInit = wheel_legged::params::active::chassis::kStandupPhase0ThetaTargetRad;
    constexpr float kThetaTol = wheel_legged::params::active::chassis::kStandupPhase1ThetaTolRad;
    constexpr float kRampStep = wheel_legged::params::active::chassis::kStandupThetaRampStepRad;
    constexpr float kRetractLenThresholdM = wheel_legged::params::active::chassis_fsm::kLowLegLengthM + 0.02f;

    if (standup_phase_ == 0) {
      force_low_leg_ = true;
      standup_theta_target_ = kThetaInit;
      if (left_leg_.l0() + right_leg_.l0() < 2 * kRetractLenThresholdM) {
        standup_phase_ = 1;
      }
    }

    if (standup_phase_ == 1) {
      force_low_leg_ = false;
      // Phase 1 腿长目标覆盖
      params_.leg_target_length_m = wheel_legged::params::active::chassis::kStandupPhase1TargetLengthM;
      // 摆角目标斜坡回 0
      if (standup_theta_target_ > 0.0f) {
        standup_theta_target_ -= kRampStep;
        if (standup_theta_target_ < 0.0f) standup_theta_target_ = 0.0f;
      }
      const float theta_tol =
          trigger_standup_latched_ ? wheel_legged::params::active::chassis::kStandupPhase1ThetaTolStairRad : kThetaTol;
      constexpr float kTwoPi = 2.0f * static_cast<float>(M_PI);
      const float theta_ll_0_2pi = std::fmod(state_output.current.theta_ll, kTwoPi);
      const float theta_lr_0_2pi = std::fmod(state_output.current.theta_lr, kTwoPi);
      const float theta_ll_pos = theta_ll_0_2pi < 0 ? theta_ll_0_2pi + kTwoPi : theta_ll_0_2pi;
      const float theta_lr_pos = theta_lr_0_2pi < 0 ? theta_lr_0_2pi + kTwoPi : theta_lr_0_2pi;
      if (std::min(theta_ll_pos, kTwoPi - theta_ll_pos) < theta_tol &&
          std::min(theta_lr_pos, kTwoPi - theta_lr_pos) < theta_tol) {
        standup_complete_ = true;
        standup_phase_ = 2;
        force_low_leg_ = false;
        standup_theta_target_ = 0.0f;
      }
    }
  }
  prev_enable_output_ = input.enable_output;
  output_.standup_complete = standup_complete_;
  output_.standup_phase = standup_phase_;

  const bool is_jump_state = (input.fsm_mode == Fsm::State::kJumpPrep || input.fsm_mode == Fsm::State::kJumpPush ||
                              input.fsm_mode == Fsm::State::kJumpRecover);
  if (is_jump_state) {
    // 跳跃阶段直接使用 FSM 设定的各阶段目标腿长，不走斜坡
    params_.leg_target_length_m = input.motion_target.leg_length_m;
    smoothed_leg_target_length_m_ = params_.leg_target_length_m;
    last_ramp_target_m_ = params_.leg_target_length_m;
    ramp_step_per_tick_m_ = 0.0f;
  } else if (!standup_complete_) {
    // 起立阶段 bypass 正常腿长斜坡，由起立逻辑自行设置
    smoothed_leg_target_length_m_ = params_.leg_target_length_m;
    last_ramp_target_m_ = params_.leg_target_length_m;
    ramp_step_per_tick_m_ = 0.0f;
  } else {
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
    if (!standup_complete_) {
      force_low_leg_ticks_ = 0;  // 起立全程不自动过期
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
  const auto pv_scales = wheel_legged::control_loop::ResolvePositionVelocityScales(input.fsm_mode);
  const float pos_scale = input.position_hold_active ? pv_scales.position_scale : 1.0f;
  const float vel_scale = input.position_hold_active ? pv_scales.velocity_scale : 1.0f;
  base_torque_ =
      lqr_controller_.ComputeControl(filtered_state, input.expected, input.displacement_bias, pos_scale, vel_scale);

  const rm::f32 eta_left = ComputeEtaFromLegLength(left_leg_.l0());
  const rm::f32 eta_right = ComputeEtaFromLegLength(right_leg_.l0());
  const rm::f32 effective_mass_left_kg = 0.5f * kBodyMassKg + eta_left * kLegMassKg;
  const rm::f32 effective_mass_right_kg = 0.5f * kBodyMassKg + eta_right * kLegMassKg;
  const rm::f32 gravity_ff_left = effective_mass_left_kg * kGravityMps2;
  const rm::f32 gravity_ff_right = effective_mass_right_kg * kGravityMps2;
  const rm::f32 wheel_radius_m = (kWheelRadiusM > 1e-5f) ? kWheelRadiusM : 1e-5f;

  const rm::f32 avg_leg_length_m = 0.5f * (left_leg_.l0() + right_leg_.l0());

  const bool use_stair_target = input.motion_target.use_stair_theta_controller;

  // 着地后腿长 PID D 项输入放大
  if (input.fsm_mode == Fsm::State::kSpin) {
    constexpr float kSpinLegLengthBiasM = wheel_legged::params::active::control_loop::kSpinLegLengthBiasM;
    left_l0_pid_.UpdateExtDiff(params_.leg_target_length_m + kSpinLegLengthBiasM, left_leg_.l0(), -left_leg_.l0_dot(),
                               2);
    right_l0_pid_.UpdateExtDiff(params_.leg_target_length_m - kSpinLegLengthBiasM, right_leg_.l0(),
                                -right_leg_.l0_dot(), 2);
  } else if (use_stair_target) {
    left_l0_pid_.UpdateExtDiff(params_.leg_target_length_m, left_leg_.l0(), -left_leg_.l0_dot(), 2);
    right_l0_pid_.UpdateExtDiff(params_.leg_target_length_m, right_leg_.l0(), -right_leg_.l0_dot(), 2);
  } else {
    left_l0_pid_.UpdateExtDiff(params_.leg_target_length_m, avg_leg_length_m, -left_leg_.l0_dot(), 2);
    right_l0_pid_.UpdateExtDiff(params_.leg_target_length_m, avg_leg_length_m, -right_leg_.l0_dot(), 2);
  }
  // 下压目标腿长时使用独立 PID（持续 1s）
  if (mid_leg_dip_active_) {
    left_l0_pid_dip_.UpdateExtDiff(params_.leg_target_length_m, avg_leg_length_m, -left_leg_.l0_dot(), 2);
    right_l0_pid_dip_.UpdateExtDiff(params_.leg_target_length_m, avg_leg_length_m, -right_leg_.l0_dot(), 2);
  }
  output_.left_l0_pid_out = mid_leg_dip_active_
                                ? left_l0_pid_dip_.out()
                                : left_l0_pid_.out();
  output_.right_l0_pid_out = mid_leg_dip_active_
                                 ? right_l0_pid_dip_.out()
                                 : right_l0_pid_.out();
  const rm::f32 length_force_base = 0.5f * (output_.left_l0_pid_out + output_.right_l0_pid_out);

  l_spring_torque_ = ComputeLeftSpringTorque(left_leg_.l0());
  r_spring_torque_ = ComputeRightSpringTorque(right_leg_.l0());

  // 起立摆角 PID：将测量值 wrap 到目标角附近，避免 2π 跳变导致 PID 绕远路
  if (!standup_complete_) {
    auto wrap_near_target = [](float theta, float target) {
      constexpr float kTwoPi = 2.0f * static_cast<float>(M_PI);
      constexpr float kPi = static_cast<float>(M_PI);
      float a = std::fmod(theta, kTwoPi);
      if (a < 0) a += kTwoPi;
      float diff = a - target;
      if (diff > kPi)
        a -= kTwoPi;
      else if (diff < -kPi)
        a += kTwoPi;
      return a;
    };
    const float theta_ll_wrapped = wrap_near_target(state_output.current.theta_ll, standup_theta_target_);
    const float theta_lr_wrapped = wrap_near_target(state_output.current.theta_lr, standup_theta_target_);
    left_leg_angle_pid_standup_.UpdateExtDiff(standup_theta_target_, theta_ll_wrapped,
                                              -state_output.current.theta_ll_dot, 2);
    right_leg_angle_pid_standup_.UpdateExtDiff(standup_theta_target_, theta_lr_wrapped,
                                               -state_output.current.theta_lr_dot, 2);
  }

  const bool use_jump_retract1 = (input.fsm_mode == Fsm::State::kJumpPrep);
  const bool use_jump_extend = (input.fsm_mode == Fsm::State::kJumpPush);
  const bool use_jump_retract2 = (input.fsm_mode == Fsm::State::kJumpRecover);
  const bool is_jump_state = use_jump_retract1 || use_jump_extend || use_jump_retract2;

  // 台阶序列摆角 PID
  if (use_stair_target) {
    left_stair_theta_pid_.UpdateExtDiff(input.motion_target.theta_ll_rad, state_output.current.theta_ll,
                                        -state_output.current.theta_ll_dot, 2);
    right_stair_theta_pid_.UpdateExtDiff(input.motion_target.theta_lr_rad, state_output.current.theta_lr,
                                         -state_output.current.theta_lr_dot, 2);
  }

  // 跳跃收腿第二阶段摆角 PID
  if (use_jump_retract2) {
    left_leg_angle_pid_jump_retract2_.UpdateExtDiff(0.0f, state_output.current.theta_ll,
                                                    -state_output.current.theta_ll_dot, 2);
    right_leg_angle_pid_jump_retract2_.UpdateExtDiff(0.0f, state_output.current.theta_lr,
                                                     -state_output.current.theta_lr_dot, 2);
  }

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
      left_l0_pid_jump_two_.UpdateExtDiff(params_.leg_target_length_m, left_leg_.l0(), -left_leg_.l0_dot(), 2);
      right_l0_pid_jump_two_.UpdateExtDiff(params_.leg_target_length_m, right_leg_.l0(), -right_leg_.l0_dot(), 2);
      const float left_leg_length_force = left_l0_pid_jump_two_.out();
      const float right_leg_length_force = right_l0_pid_jump_two_.out();
      // left_force_ = left_leg_length_force + roll_pid_.out();
      // right_force_ = right_leg_length_force - roll_pid_.out();
      // left_force_ = left_leg_length_force + roll_pid_.out() + l_spring_torque_;
      // right_force_ = right_leg_length_force - roll_pid_.out() + r_spring_torque_;

      left_force_ = kJumpPushForceN + roll_pid_.out();
      right_force_ = kJumpPushForceN + roll_pid_.out();

      // left_force_ = 200.f;
      // right_force_ = 200.f;
    } else if (use_jump_retract2) {
      left_l0_pid_jump_three_.UpdateExtDiff(params_.leg_target_length_m, left_leg_.l0(), -left_leg_.l0_dot(), 2);
      right_l0_pid_jump_three_.UpdateExtDiff(params_.leg_target_length_m, right_leg_.l0(), -right_leg_.l0_dot(), 2);
      const float left_leg_length_force = left_l0_pid_jump_three_.out();
      const float right_leg_length_force = right_l0_pid_jump_three_.out();
      // left_force_ = left_leg_length_force + roll_pid_.out() + l_spring_torque_;
      // right_force_ = right_leg_length_force - roll_pid_.out() + r_spring_torque_;

      left_force_ = left_leg_length_force + l_spring_torque_;
      right_force_ = right_leg_length_force + r_spring_torque_;
    } else if (use_jump_retract1) {
      left_l0_pid_jump_one_.UpdateExtDiff(params_.leg_target_length_m, left_leg_.l0(), -left_leg_.l0_dot(), 2);
      right_l0_pid_jump_one_.UpdateExtDiff(params_.leg_target_length_m, right_leg_.l0(), -right_leg_.l0_dot(), 2);
      const float left_leg_length_force = left_l0_pid_jump_one_.out();
      const float right_leg_length_force = right_l0_pid_jump_one_.out();
      left_force_ = left_leg_length_force + roll_pid_.out() + l_spring_torque_;
      right_force_ = right_leg_length_force - roll_pid_.out() + r_spring_torque_;
    } else if (!standup_complete_ || use_stair_target) {
      // 起立 / 台阶序列：腿长PID + 弹簧补偿，不用重力/roll/惯性
      left_force_ = output_.left_l0_pid_out + l_spring_torque_;
      right_force_ = output_.right_l0_pid_out + r_spring_torque_;
      // left_force_ =  l_spring_torque_;
      // right_force_ =  r_spring_torque_;
    } else {
      // 常规支撑时叠加腿长 PID、重力前馈、横滚补偿、惯性补偿和弹簧补偿。
      const rm::f32 inertial_ff_left = effective_mass_left_kg *
                                       ((left_leg_.l0() + kWheelPhysicalRadiusM) / (2.0f * wheel_radius_m)) *
                                       state_output.current.phi_dot * state_output.current.s_dot;
      const rm::f32 inertial_ff_right = effective_mass_right_kg *
                                        ((right_leg_.l0() + kWheelPhysicalRadiusM) / (2.0f * wheel_radius_m)) *
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

    if (!standup_complete_) {
      t_bl_cmd = -left_leg_angle_pid_standup_.out();
      t_br_cmd = -right_leg_angle_pid_standup_.out();
    } else if (use_stair_target) {
      t_bl_cmd = -left_stair_theta_pid_.out();
      t_br_cmd = -right_stair_theta_pid_.out();
    } else if (use_jump_retract2) {
      t_bl_cmd = -left_leg_angle_pid_jump_retract2_.out();
      t_br_cmd = -right_leg_angle_pid_jump_retract2_.out();
    } else {
      t_bl_cmd = -base_torque_.t_bl;
      t_br_cmd = -base_torque_.t_br;
    }

    output_.lb_tau = left_leg_.jacobi_00() * left_force_ + left_leg_.jacobi_01() * t_bl_cmd;
    output_.lf_tau = left_leg_.jacobi_10() * left_force_ + left_leg_.jacobi_11() * t_bl_cmd;
    output_.rb_tau = right_leg_.jacobi_00() * right_force_ + right_leg_.jacobi_01() * t_br_cmd;
    output_.rf_tau = right_leg_.jacobi_10() * right_force_ + right_leg_.jacobi_11() * t_br_cmd;

    output_.lf_tau = -output_.lf_tau;
    output_.lb_tau = -output_.lb_tau;
  } else {
    if (state_output.current.theta_b > wheel_legged::params::active::chassis::kPostureThetaBMinRad &&
        state_output.current.theta_b < wheel_legged::params::active::chassis::kPostureThetaBMaxRad &&
        imu_roll_ > wheel_legged::params::active::chassis::kPostureRollMinRad &&
        imu_roll_ < wheel_legged::params::active::chassis::kPostureRollMaxRad) {
      // pitch/roll正常但theta异常 → 先等云台归中，再摆腿恢复
      // 首次进入时清零 PID，避免历史积分残留导致每次恢复行为不一致
      if (!theta_recovery_active_) {
        left_leg_turn_pid_.Clear();
        right_leg_turn_pid_.Clear();
        theta_recovery_active_ = true;
        theta_recovery_phase_ = 0;
      }

      if (theta_recovery_phase_ == 0) {
        // Phase 0: 等待云台 yaw 归中完成，不输出腿力矩
        left_force_ = 0.0f;
        right_force_ = 0.0f;
        output_.lb_tau = 0.0f;
        output_.lf_tau = 0.0f;
        output_.rb_tau = 0.0f;
        output_.rf_tau = 0.0f;
        output_.lw_tau = 0.0f;
        output_.rw_tau = 0.0f;
        if (input.yaw_centering_complete) {
          theta_recovery_phase_ = 1;
        }
      }

      if (theta_recovery_phase_ == 1) {
        // Phase 1: 云台归中完成，执行摆腿恢复
        constexpr float kThetaLegMin = wheel_legged::params::active::chassis::kPostureThetaLegMinRad;
        constexpr float kThetaLegMax = wheel_legged::params::active::chassis::kPostureThetaLegMaxRad;
        constexpr float kRecoverVel = wheel_legged::params::active::chassis::kLegRecoverThetaDotTarget;

        const bool ll_in_range = ThetaInRange(state_output.current.theta_ll, kThetaLegMin, kThetaLegMax);
        const bool lr_in_range = ThetaInRange(state_output.current.theta_lr, kThetaLegMin, kThetaLegMax);

        auto wrap_near = [](float theta, float target) {
          constexpr float kTwoPi = 2.0f * static_cast<float>(M_PI);
          constexpr float kPi = static_cast<float>(M_PI);
          float a = std::fmod(theta, kTwoPi);
          if (a < 0) a += kTwoPi;
          float diff = a - target;
          if (diff > kPi)
            a -= kTwoPi;
          else if (diff < -kPi)
            a += kTwoPi;
          return a;
        };

        float ll_pid_out = 0.0f;
        float lr_pid_out = 0.0f;
        if (!ll_in_range) {
          constexpr float kTwoPi = 2.0f * static_cast<float>(M_PI);
          float a = std::fmod(state_output.current.theta_ll - kThetaLegMin, kTwoPi);
          if (a < 0) a += kTwoPi;
          const float boundary = (a - (kThetaLegMax - kThetaLegMin) < kTwoPi - a) ? kThetaLegMax : kThetaLegMin;
          const float theta_near = wrap_near(state_output.current.theta_ll, boundary);
          const float prox = RecoveryProximityScale(theta_near, boundary, kDecelZoneRad);
          const float scaled_vel = ApplyRecoveryDecel(kRecoverVel, prox, kMinSpeedRadS);
          left_leg_turn_pid_.Update(scaled_vel, state_output.current.theta_ll_dot);
          ll_pid_out = -left_leg_turn_pid_.out();
        } else {
          left_leg_turn_pid_.Clear();
        }
        if (!lr_in_range) {
          constexpr float kTwoPi = 2.0f * static_cast<float>(M_PI);
          float a = std::fmod(state_output.current.theta_lr - kThetaLegMin, kTwoPi);
          if (a < 0) a += kTwoPi;
          const float boundary = (a - (kThetaLegMax - kThetaLegMin) < kTwoPi - a) ? kThetaLegMax : kThetaLegMin;
          const float theta_near = wrap_near(state_output.current.theta_lr, boundary);
          const float prox = RecoveryProximityScale(theta_near, boundary, kDecelZoneRad);
          const float scaled_vel = ApplyRecoveryDecel(kRecoverVel, prox, kMinSpeedRadS);
          right_leg_turn_pid_.Update(scaled_vel, state_output.current.theta_lr_dot);
          lr_pid_out = -right_leg_turn_pid_.out();
        } else {
          right_leg_turn_pid_.Clear();
        }

        // 双腿回到安全范围 → 直接进起立Phase 1，跳过LQR
        if (ll_in_range && lr_in_range) {
          standup_complete_ = false;
          standup_phase_ = 1;
          standup_theta_target_ = 0.0f;
          theta_recovery_active_ = false;
          standup_from_recovery_latch_ = true;
        }

        left_force_ = 0.0f;
        right_force_ = 0.0f;

        output_.lb_tau = left_leg_.jacobi_00() * left_force_ + left_leg_.jacobi_01() * ll_pid_out;
        output_.lf_tau = left_leg_.jacobi_10() * left_force_ + left_leg_.jacobi_11() * ll_pid_out;
        output_.rb_tau = right_leg_.jacobi_00() * right_force_ + right_leg_.jacobi_01() * lr_pid_out;
        output_.rf_tau = right_leg_.jacobi_10() * right_force_ + right_leg_.jacobi_11() * lr_pid_out;

        output_.lf_tau = -output_.lf_tau;
        output_.lb_tau = -output_.lb_tau;

        output_.lw_tau = 0.0f;
        output_.rw_tau = 0.0f;
      }
    } else if (state_output.current.theta_b < wheel_legged::params::active::chassis::kPostureThetaBMinRad ||
               state_output.current.theta_b > wheel_legged::params::active::chassis::kPostureThetaBMaxRad) {
      theta_recovery_phase_ = 0;
      theta_recovery_active_ = false;
      constexpr rm::f32 kVel = wheel_legged::params::active::chassis::kLegRecoverThetaDotTarget;
      float ll_pid_out = 0.0f;
      float lr_pid_out = 0.0f;

      constexpr rm::f32 kRangeLowMin = wheel_legged::params::active::chassis::kRecoveryThetaRangeLowMin;
      constexpr rm::f32 kRangeLowMax = wheel_legged::params::active::chassis::kRecoveryThetaRangeLowMax;
      constexpr rm::f32 kRangeHighMin = wheel_legged::params::active::chassis::kRecoveryThetaRangeHighMin;
      constexpr rm::f32 kRangeHighMax = wheel_legged::params::active::chassis::kRecoveryThetaRangeHighMax;

      const rm::f32 lw = state_output.current.theta_ll;
      const rm::f32 rw = state_output.current.theta_lr;

      const bool is_front =
          (state_output.current.theta_b < wheel_legged::params::active::chassis::kPostureThetaBMinRad);
      const rm::f32 tgt_min = is_front ? kRangeLowMin : kRangeHighMin;
      const rm::f32 tgt_max = is_front ? kRangeLowMax : kRangeHighMax;
      const rm::f32 dir = is_front ? kVel : -kVel;

      const bool l_in = ThetaInRange(lw, tgt_min, tgt_max);
      const bool r_in = ThetaInRange(rw, tgt_min, tgt_max);

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
      if (l_in && r_in) {
        const float err_pitch = std::copysign(state_output.current.theta_b, left_leg_turn_pid_.out());
        ll_pid_out = -(left_leg_turn_pid_.out() +
                       wheel_legged::params::active::chassis::kRecoveryPitchFeedforwardKp * err_pitch);
        lr_pid_out = -(right_leg_turn_pid_.out() +
                       wheel_legged::params::active::chassis::kRecoveryPitchFeedforwardKp * err_pitch);
      } else {
        ll_pid_out = -1.8 * left_leg_turn_pid_.out();
        lr_pid_out = -1.8 * right_leg_turn_pid_.out();
      }

      output_.lb_tau = left_leg_.jacobi_00() * left_force_ + left_leg_.jacobi_01() * ll_pid_out;
      output_.lf_tau = left_leg_.jacobi_10() * left_force_ + left_leg_.jacobi_11() * ll_pid_out;
      output_.rb_tau = right_leg_.jacobi_00() * right_force_ + right_leg_.jacobi_01() * lr_pid_out;
      output_.rf_tau = right_leg_.jacobi_10() * right_force_ + right_leg_.jacobi_11() * lr_pid_out;

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
      theta_recovery_phase_ = 0;
      theta_recovery_active_ = false;
      float ll_pid_out = 0.0f;
      float lr_pid_out = 0.0f;

      if (imu_roll_ > wheel_legged::params::active::chassis::kPostureRollMaxRad) {
        right_leg_turn_pid_.Update(-1.5f * wheel_legged::params::active::chassis::kLegRecoverThetaDotTarget,
                                   state_output.current.theta_lr_dot);
        const float err_roll = std::copysign(imu_roll_, right_leg_turn_pid_.out());
        ll_pid_out = 0;
        lr_pid_out =
            -(right_leg_turn_pid_.out() + wheel_legged::params::active::chassis::kRecoveryRollFeedforwardKp * err_roll);
      } else if (imu_roll_ < wheel_legged::params::active::chassis::kPostureRollMinRad) {
        left_leg_turn_pid_.Update(-1.5f * wheel_legged::params::active::chassis::kLegRecoverThetaDotTarget,
                                  state_output.current.theta_ll_dot);
        const float err_roll = std::copysign(imu_roll_, left_leg_turn_pid_.out());
        ll_pid_out =
            -(left_leg_turn_pid_.out() + wheel_legged::params::active::chassis::kRecoveryRollFeedforwardKp * err_roll);
        lr_pid_out = 0;
      }

      left_force_ = 0.0f;
      right_force_ = 0.0f;

      output_.lb_tau = left_leg_.jacobi_00() * left_force_ + left_leg_.jacobi_01() * ll_pid_out;
      output_.lf_tau = left_leg_.jacobi_10() * left_force_ + left_leg_.jacobi_11() * ll_pid_out;
      output_.rb_tau = right_leg_.jacobi_00() * right_force_ + right_leg_.jacobi_01() * lr_pid_out;
      output_.rf_tau = right_leg_.jacobi_10() * right_force_ + right_leg_.jacobi_11() * lr_pid_out;

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
