#include "include/chassis/chassis.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include "include/params.hpp"

/**
 * @file  targets/wheel_legged/chassis.cc
 * @brief 搴曠洏鎺у埗瀹炵幇锛氱姸鎬佷及璁°€丩QR銆佽ˉ鍋夸笌鍔涚煩杈撳嚭
 */

namespace {

constexpr rm::f32 kControlDtS = wheel_legged::params::active::chassis::kControlDtS;  ///< 搴曠洏鎺у埗鍛ㄦ湡锛?00Hz锛?

constexpr rm::f32 kLegL1M = wheel_legged::params::active::chassis::kLegL1M;
constexpr rm::f32 kLegL2M = wheel_legged::params::active::chassis::kLegL2M;
constexpr rm::f32 kSpringTorqueScale = wheel_legged::params::active::chassis::kSpringTorqueScale;

constexpr rm::f32 kBodyMassKg = wheel_legged::params::active::chassis::kBodyMassKg;
constexpr rm::f32 kLegMassKg = wheel_legged::params::active::chassis::kLegMassKg;
constexpr rm::f32 kGravityMps2 = wheel_legged::params::active::chassis::kGravityMps2;
constexpr rm::f32 kWheelRadiusM = wheel_legged::params::active::chassis::kWheelRadiusM;
constexpr rm::f32 kOffGroundSupportForceThresholdN =
    wheel_legged::params::active::chassis::kOffGroundSupportForceThresholdN;

constexpr const auto &kEtaLookupLegLengthM = wheel_legged::params::active::chassis::kEtaLookupLegLengthM;

constexpr const auto &kEtaLookupLwM = wheel_legged::params::active::chassis::kEtaLookupLwM;

constexpr const auto &kCtrlP = wheel_legged::params::active::chassis::kCtrlP;

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
 * @brief 根据腿长估算机械弹簧对关节的等效补偿力矩
 */
rm::f32 ComputeSpringTorqueFromLegLength(const rm::f32 leg_length_m) {
  static constexpr rm::f32 kPi = wheel_legged::params::active::kPi;
  const rm::f32 x =
      std::acos((kLegL2M * kLegL2M + kLegL1M * kLegL1M - leg_length_m * leg_length_m) / (2.0f * kLegL2M * kLegL1M));
  return -(std::sqrt(wheel_legged::params::active::chassis::kSpringModelA -
                     wheel_legged::params::active::chassis::kSpringModelB * std::cos(x)) *
           std::sin(x - kPi / wheel_legged::params::active::chassis::kSpringPhaseDivisor) /
           (std::sqrt(wheel_legged::params::active::chassis::kSpringModelC -
                      wheel_legged::params::active::chassis::kSpringModelD *
                          std::cos(x - kPi / wheel_legged::params::active::chassis::kSpringPhaseDivisor)) *
            std::sin(x))) *
         kSpringTorqueScale;
}

/**
 * @brief 是否处于强制安全零输出模式
 */
bool IsSafeStopMode(const chassis::Fsm::State mode) { return mode == chassis::Fsm::State::kDisabled; }

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
  const auto &roll_pid = wheel_legged::params::active::chassis::kRollPid;
  init_pid(roll_pid_, roll_pid.kp, roll_pid.ki, roll_pid.kd, roll_pid.max_out, roll_pid.max_iout);
  const auto &left_leg_turn_pid = wheel_legged::params::active::chassis::kLeftLegTurnPid;
  init_pid(left_leg_turn_pid_, left_leg_turn_pid.kp, left_leg_turn_pid.ki, left_leg_turn_pid.kd,
           left_leg_turn_pid.max_out, left_leg_turn_pid.max_iout);
  const auto &right_leg_turn_pid = wheel_legged::params::active::chassis::kRightLegTurnPid;
  init_pid(right_leg_turn_pid_, right_leg_turn_pid.kp, right_leg_turn_pid.ki, right_leg_turn_pid.kd,
           right_leg_turn_pid.max_out, right_leg_turn_pid.max_iout);

  std::array<std::array<rm::f32, 6>, 40> coeff_vec{};
  for (int i = 0; i < 40; ++i) {
    std::copy(&kCtrlP[i * 6], &kCtrlP[i * 6 + 6], coeff_vec[i].begin());
  }
  lqr_controller_.SetLqrCoefficients(coeff_vec);

  ChassisStateEstimatorConfig cfg{};
  state_estimator_.Init(cfg);
  SafeStop();
  smoothed_leg_target_length_m_ = params_.leg_target_length_m;
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
  estimator_input.use_wheel_speed_direct = (input.fsm_mode == Fsm::State::kSpin);

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
  lf_real_torque_ = estimator_input.left_leg.front.torque_nm;
  lb_real_torque_ = estimator_input.left_leg.back.torque_nm;
  rf_real_torque_ = estimator_input.right_leg.front.torque_nm;
  rb_real_torque_ = estimator_input.right_leg.back.torque_nm;

  output_.current_state = state_output.current;
  output_.mean_leg_length_m = 0.5f * (state_output.left_leg_length_m + state_output.right_leg_length_m);
  output_.wheel_speed_mps = state_output.wheel_speed_mps;
  output_.speed_mps = state_output.fused_speed_mps;
  output_.raw_wheel_speed_mps = state_output.raw_wheel_speed_mps;
  output_.raw_accel_speed_mps = state_output.raw_accel_speed_mps;
  output_.current_speed_mps = state_output.current_speed_mps;
  output_.off_ground_in_mid_high_leg = false;

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

  if (!output_.posture_valid) {
  } else if (!input.enable_output || !input.run_chassis_update || IsSafeStopMode(input.fsm_mode)) {
    SafeStop();
    prev_enable_output_ = false;
    standup_complete_ = false;
    return;
  }

  // 检测 enable_output 上升沿：关节先出力，轮端等待起立完成再输出
  if (input.enable_output && !prev_enable_output_) {
    standup_complete_ = false;
  }
  prev_enable_output_ = input.enable_output;

  // 起立完成判定：双腿 theta 均小于阈值后锁存
  constexpr float kThetaThreshold = wheel_legged::params::active::chassis::kStandupThetaThresholdRad;
  if (!standup_complete_ && std::fabs(state_output.current.theta_ll) < kThetaThreshold &&
      std::fabs(state_output.current.theta_lr) < kThetaThreshold) {
    standup_complete_ = true;
  }

  output_.standup_complete = standup_complete_;

  const bool ramp_enabled = (input.fsm_mode == Fsm::State::kLowLeg || input.fsm_mode == Fsm::State::kMidLeg ||
                             input.fsm_mode == Fsm::State::kHighLeg || input.fsm_mode == Fsm::State::kSpin);
  if (ramp_enabled) {
    const float ramp_rate = (wheel_legged::params::active::chassis_fsm::kHighLegLengthM -
                             wheel_legged::params::active::chassis_fsm::kLowLegLengthM) /
                            wheel_legged::params::active::chassis_fsm::kLegLengthRampTimeS;
    const float max_step = ramp_rate * kControlDtS;
    const float error = input.target_leg_length_m - smoothed_leg_target_length_m_;
    if (std::fabs(error) <= max_step) {
      smoothed_leg_target_length_m_ = input.target_leg_length_m;
    } else {
      smoothed_leg_target_length_m_ += (error > 0.0f ? max_step : -max_step);
    }
    params_.leg_target_length_m = smoothed_leg_target_length_m_;
  } else {
    smoothed_leg_target_length_m_ = input.target_leg_length_m;
    params_.leg_target_length_m = input.target_leg_length_m;
  }
  ComputeActuatorTorque(input, state_output);
}

/**
 * @brief 组合 LQR 与补偿项，计算六电机最终力矩
 */
void chassis::Chassis::ComputeActuatorTorque(const UpdateInput &input,
                                             const ChassisStateEstimatorOutput &state_output) {
  static constexpr rm::f32 kPi = wheel_legged::params::active::kPi;
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

  base_torque_ = lqr_controller_.ComputeControl(state_output.current, input.expected);

  const rm::f32 eta_left = ComputeEtaFromLegLength(left_leg_.l0());
  const rm::f32 eta_right = ComputeEtaFromLegLength(right_leg_.l0());
  const rm::f32 effective_mass_left_kg = 0.5f * kBodyMassKg + eta_left * kLegMassKg;
  const rm::f32 effective_mass_right_kg = 0.5f * kBodyMassKg + eta_right * kLegMassKg;
  const rm::f32 gravity_ff_left = effective_mass_left_kg * kGravityMps2;
  const rm::f32 gravity_ff_right = effective_mass_right_kg * kGravityMps2;
  const rm::f32 wheel_radius_m = (kWheelRadiusM > 1e-5f) ? kWheelRadiusM : 1e-5f;

  const rm::f32 avg_leg_length_m = 0.5f * (left_leg_.l0() + right_leg_.l0());
  left_l0_pid_.Update(params_.leg_target_length_m, avg_leg_length_m);
  right_l0_pid_.Update(params_.leg_target_length_m, avg_leg_length_m);
  const rm::f32 length_force_base = 0.5f * (left_l0_pid_.out() + right_l0_pid_.out());

  l_spring_torque_ = ComputeSpringTorqueFromLegLength(left_leg_.l0());
  r_spring_torque_ = ComputeSpringTorqueFromLegLength(right_leg_.l0());

  const bool use_jump_retract1 = (input.fsm_mode == Fsm::State::kJumpPrep);
  const bool use_jump_extend = (input.fsm_mode == Fsm::State::kJumpPush);
  const bool use_jump_retract2 = (input.fsm_mode == Fsm::State::kJumpRecover);
  const bool use_stair_climb = (input.fsm_mode == Fsm::State::kStairClimb);

  if (output_.posture_valid) {
    roll_pid_.Update(wheel_legged::params::active::chassis::kRollBalanceTargetRad, imu_roll_);
    rm::f32 leg_length_force = length_force_base;

    // 跳跃阶段分别使用收腿/蹬伸/回收三套腿长控制策略。
    if (use_jump_extend) {
      left_l0_pid_jump_two_.Update(params_.leg_target_length_m, avg_leg_length_m);
      right_l0_pid_jump_two_.Update(params_.leg_target_length_m, avg_leg_length_m);
      leg_length_force = 0.5f * (left_l0_pid_jump_two_.out() + right_l0_pid_jump_two_.out());
      left_force_ = leg_length_force + roll_pid_.out();
      right_force_ = leg_length_force - roll_pid_.out();
    } else if (use_jump_retract2) {
      left_l0_pid_jump_three_.Update(params_.leg_target_length_m, avg_leg_length_m);
      right_l0_pid_jump_three_.Update(params_.leg_target_length_m, avg_leg_length_m);
      leg_length_force = 0.5f * (left_l0_pid_jump_three_.out() + right_l0_pid_jump_three_.out());
      left_force_ = leg_length_force + roll_pid_.out() + l_spring_torque_;
      right_force_ = leg_length_force - roll_pid_.out() + r_spring_torque_;
    } else if (use_jump_retract1) {
      left_force_ = leg_length_force + roll_pid_.out() + l_spring_torque_;
      right_force_ = leg_length_force - roll_pid_.out() + r_spring_torque_;
    } else {
      // 常规支撑时叠加腿长 PID、重力前馈、横滚补偿、惯性补偿和弹簧补偿。
      const rm::f32 inertial_ff_left = effective_mass_left_kg * (left_leg_.l0() / (2.0f * wheel_radius_m)) *
                                       state_output.current.phi_dot * state_output.current.s_dot;
      const rm::f32 inertial_ff_right = effective_mass_right_kg * (right_leg_.l0() / (2.0f * wheel_radius_m)) *
                                        state_output.current.phi_dot * state_output.current.s_dot;

      left_force_ = leg_length_force + gravity_ff_left + roll_pid_.out() - inertial_ff_left + l_spring_torque_;
      right_force_ = leg_length_force + gravity_ff_right - roll_pid_.out() + inertial_ff_right + r_spring_torque_;

      // left_force_ = l_spring_torque_;
      // right_force_ = r_spring_torque_;
    }

    const bool off_ground_in_mid_high_leg =
        (input.fsm_mode == Fsm::State::kMidLeg || input.fsm_mode == Fsm::State::kHighLeg) &&
        (left_support_force_est_n_ < kOffGroundSupportForceThresholdN ||
         right_support_force_est_n_ < kOffGroundSupportForceThresholdN);
    output_.off_ground_in_mid_high_leg = off_ground_in_mid_high_leg;

    // 离地时限制竖直力幅值，防止腾空瞬间力尖峰导致腿异常蹬伸/收束
    if (off_ground_in_mid_high_leg) {
      left_force_ = std::clamp(left_force_, -30.0f, 30.0f);
      right_force_ = std::clamp(right_force_, -30.0f, 30.0f);
    }

    // 离地、跳跃回收、上台阶或起立未完成时关闭轮端力矩。
    if (use_jump_retract2 || off_ground_in_mid_high_leg || use_stair_climb || !standup_complete_) {
      output_.lw_tau = 0.0f;
      output_.rw_tau = 0.0f;
    } else {
      output_.lw_tau = -base_torque_.t_wl;
      output_.rw_tau = base_torque_.t_wr;
    }

    const rm::f32 t_bl_cmd = -base_torque_.t_bl;
    const rm::f32 t_br_cmd = -base_torque_.t_br;

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
      left_leg_turn_pid_.Update(wheel_legged::params::active::chassis::kLegRecoverThetaDotTarget,
                                state_output.current.theta_ll_dot);
      right_leg_turn_pid_.Update(wheel_legged::params::active::chassis::kLegRecoverThetaDotTarget,
                                 state_output.current.theta_lr_dot);

      left_force_ = 0.0f;
      right_force_ = 0.0f;

      output_.lb_tau = left_leg_.jacobi_00() * left_force_ + left_leg_.jacobi_01() * (-left_leg_turn_pid_.out());
      output_.lf_tau = left_leg_.jacobi_10() * left_force_ + left_leg_.jacobi_11() * (-left_leg_turn_pid_.out());
      output_.rb_tau = right_leg_.jacobi_00() * right_force_ + right_leg_.jacobi_01() * (-right_leg_turn_pid_.out());
      output_.rf_tau = right_leg_.jacobi_10() * right_force_ + right_leg_.jacobi_11() * (-right_leg_turn_pid_.out());

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
      // 以下代码可能有问题
      // const rm::f32 lw = rm::modules::Wrap(state_output.current.theta_ll, -kPi, kPi);
      // const rm::f32 rw = rm::modules::Wrap(state_output.current.theta_lr, -kPi, kPi);

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
        // 以下代码可能有问题
        // if (l_in) {
        //   left_leg_turn_pid_.Clear();
        // } else {
        //   left_leg_turn_pid_.Update(dir, state_output.current.theta_ll_dot);
        // }
        // if (r_in) {
        //   right_leg_turn_pid_.Clear();
        // } else {
        //   right_leg_turn_pid_.Update(dir, state_output.current.theta_lr_dot);
        // }
      }
      left_force_ = 0.0f;
      right_force_ = 0.0f;

      output_.lb_tau = left_leg_.jacobi_00() * left_force_ + left_leg_.jacobi_01() * (-left_leg_turn_pid_.out());
      output_.lf_tau = left_leg_.jacobi_10() * left_force_ + left_leg_.jacobi_11() * (-left_leg_turn_pid_.out());
      output_.rb_tau = right_leg_.jacobi_00() * right_force_ + right_leg_.jacobi_01() * (-right_leg_turn_pid_.out());
      output_.rf_tau = right_leg_.jacobi_10() * right_force_ + right_leg_.jacobi_11() * (-right_leg_turn_pid_.out());

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
        right_leg_turn_pid_.Update(1.5 * wheel_legged::params::active::chassis::kLegRecoverThetaDotTarget,
                                   state_output.current.theta_lr_dot);
        left_leg_turn_pid_out = 0;
        right_leg_turn_pid_out = -right_leg_turn_pid_.out();
      } else if (imu_roll_ < wheel_legged::params::active::chassis::kPostureRollMinRad) {
        left_leg_turn_pid_.Update(1.5 * wheel_legged::params::active::chassis::kLegRecoverThetaDotTarget,
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

  const rm::f32 left_F_bh = l_f * std::cos(theta_ll);
  const rm::f32 right_F_bh = r_f * std::cos(theta_lr);

  const rm::f32 left_l0_ddot = (left_leg_.l0_dot() - left_l0_dot_prev_) / kControlDtS;
  const rm::f32 right_l0_ddot = (right_leg_.l0_dot() - right_l0_dot_prev_) / kControlDtS;
  left_l0_dot_prev_ = left_leg_.l0_dot();
  right_l0_dot_prev_ = right_leg_.l0_dot();

  const rm::f32 eta_left = ComputeEtaFromLegLength(left_leg_.l0());
  const rm::f32 eta_right = ComputeEtaFromLegLength(right_leg_.l0());

  const rm::f32 left_leg_dyn_comp = -(1.0f - eta_left) * left_l0_ddot * std::cos(theta_ll);
  const rm::f32 right_leg_dyn_comp = -(1.0f - eta_right) * right_l0_ddot * std::cos(theta_lr);

  const rm::f32 gravity_support_left = (0.5f * kBodyMassKg + eta_left * kLegMassKg) * kGravityMps2;
  const rm::f32 gravity_support_right = (0.5f * kBodyMassKg + eta_right * kLegMassKg) * kGravityMps2;

  left_support_force_est_n_ = left_F_bh + gravity_support_left + kLegMassKg * left_leg_dyn_comp;
  right_support_force_est_n_ = right_F_bh + gravity_support_right + kLegMassKg * right_leg_dyn_comp;
}
