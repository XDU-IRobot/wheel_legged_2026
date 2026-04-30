#include "include/chassis/chassis.hpp"

#include <algorithm>
#include <array>
#include <cmath>

/**
 * @file  targets/wheel_legged/chassis.cc
 * @brief 底盘控制实现：状态估计、LQR、补偿与力矩输出
 */

namespace {

constexpr rm::f32 kControlDtS = 0.002f;  ///< 底盘控制周期（500Hz）

constexpr rm::f32 kLegL1M = 0.215f;
constexpr rm::f32 kLegL2M = 0.254f;
constexpr rm::f32 kSpringTorqueScale = 90.0f;

constexpr rm::f32 kBodyMassKg = 16.0f;
constexpr rm::f32 kLegMassKg = 2.3f;
constexpr rm::f32 kGravityMps2 = 9.81f;
constexpr rm::f32 kWheelRadiusM = 0.2025f;
constexpr rm::f32 kOffGroundSupportForceThresholdN = 10.0f;

constexpr rm::f32 kEtaLookupLegLengthM[] = {
    0.11f, 0.12f, 0.13f, 0.14f, 0.15f, 0.16f, 0.17f, 0.18f, 0.19f, 0.20f, 0.21f, 0.22f,
    0.23f, 0.24f, 0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.30f, 0.31f, 0.32f, 0.33f, 0.34f,
};

constexpr rm::f32 kEtaLookupLwM[] = {
    0.061990f, 0.067466f, 0.072986f, 0.078550f, 0.084158f, 0.089810f, 0.095506f, 0.101246f,
    0.107030f, 0.112858f, 0.118730f, 0.124646f, 0.130606f, 0.136610f, 0.142658f, 0.148750f,
    0.154886f, 0.161066f, 0.167290f, 0.173558f, 0.179870f, 0.186226f, 0.192626f, 0.199070f,
};

constexpr rm::f32 kCtrlP[240] = {
-4.2563,  -23.862,  19.078,  34.833,  -10.629,  -18.421,
     -7.3548,  -30.706,  31.115,  48.643,  -25.922,  -27.978,
     -5.2196,  18.8,  -7.0942,  -25.704,  5.2372,  9.2574,
     -4.8527,  18.333,  -7.5462,  -24.022,  4.4359,  10.117,
     -12.268,  -83.071,  19.912,  72.873,  -0.30452,  -26.353,
     -0.77324,  -7.1542,  3.0787,  -5.087,  2.5542,  -4.2719,
     -5.2981,  10.582,  -15.657,  -5.7461,  10.764,  4.1248,
     -0.42859,  -1.2288,  -1.3887,  4.7828,  -8.7533,  0.46825,
     -19.595,  28.635,  30.021,  -1.2348,  -37.096,  -26.07,
     -2.7479,  2.3222,  6.7265,  2.441,  -7.2635,  -6.5305,
     -4.2563,  19.078,  -23.862,  -18.421,  -10.629,  34.833,
     -7.3548,  31.115,  -30.706,  -27.978,  -25.922,  48.643,
     5.2196,  7.0942,  -18.8,  -9.2574,  -5.2372,  25.704,
     4.8527,  7.5462,  -18.333,  -10.117,  -4.4359,  24.022,
     -5.2981,  -15.657,  10.582,  4.1248,  10.764,  -5.7461,
     -0.42859,  -1.3887,  -1.2288,  0.46825,  -8.7533,  4.7828,
     -12.268,  19.912,  -83.071,  -26.353,  -0.30452,  72.873,
     -0.77324,  3.0787,  -7.1542,  -4.2719,  2.5542,  -5.087,
     -19.595,  30.021,  28.635,  -26.07,  -37.096,  -1.2348,
     -2.7479,  6.7265,  2.3222,  -6.5305,  -7.2635,  2.441,
     6.1276,  7.0757,  -20.978,  -33.409,  32.961,  10.174,
     9.4182,  8.9176,  -33.95,  -47.612,  56.18,  15.461,
     -3.241,  -17.653,  -11.754,  35.915,  -12.715,  16.657,
     -3.0304,  -17.768,  -10.728,  34.919,  -13.032,  14.77,
     38.115,  -46.657,  13.673,  55.395,  63.834,  -31.962,
     1.9945,  0.35314,  -0.768,  5.1513,  4.1234,  -1.1649,
     -2.9803,  -28.281,  -16.585,  49.151,  -51.23,  -13.197,
     -0.27861,  -0.60736,  2.9129,  -2.198,  2.0823,  -10.894,
     -27.136,  -121.1,  46.437,  139,  11.685,  -46.418,
     -1.576,  -12.31,  2.8197,  10.444,  6.1554,  -3.3792,
     6.1276,  -20.978,  7.0757,  10.174,  32.961,  -33.409,
     9.4182,  -33.95,  8.9176,  15.461,  56.18,  -47.612,
     3.241,  11.754,  17.653,  -16.657,  12.715,  -35.915,
     3.0304,  10.728,  17.768,  -14.77,  13.032,  -34.919,
     -2.9803,  -16.585,  -28.281,  -13.197,  -51.23,  49.151,
     -0.27861,  2.9129,  -0.60736,  -10.894,  2.0823,  -2.198,
     38.115,  13.673,  -46.657,  -31.962,  63.834,  55.395,
     1.9945,  -0.768,  0.35314,  -1.1649,  4.1234,  5.1513,
     -27.136,  46.437,  -121.1,  -46.418,  11.685,  139,
     -1.576,  2.8197,  -12.31,  -3.3792,  6.1554,  10.444

};

/**
 * @brief 根据腿长插值估算腿部等效质心系数
 */
rm::f32 ComputeEtaFromLegLength(const rm::f32 leg_length_m) {
  constexpr size_t kCount = sizeof(kEtaLookupLegLengthM) / sizeof(rm::f32);
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
  static constexpr rm::f32 kPi = 3.14159265358979323846f;
  const rm::f32 x =
      std::acos((kLegL2M * kLegL2M + kLegL1M * kLegL1M - leg_length_m * leg_length_m) / (2.0f * kLegL2M * kLegL1M));
  return -(std::sqrt(1082.0f - 1070.0f * std::cos(x)) * std::sin(x - kPi / 18.0f) /
           (std::sqrt(404.0f - 177.0f * std::cos(x - kPi / 18.0f)) * std::sin(x))) *
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

  init_pid(left_l0_pid_, 5900.0f, 0.04f, 95500.0f, 170.0f, 10.0f);
  init_pid(right_l0_pid_, 5900.0f, 0.04f, 95500.0f, 170.0f, 10.0f);
  init_pid(left_l0_pid_jump_two_, 6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f);
  init_pid(right_l0_pid_jump_two_, 6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f);
  init_pid(left_l0_pid_jump_three_, 6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f);
  init_pid(right_l0_pid_jump_three_, 6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f);
  init_pid(roll_pid_, 600.0f, 0.0f, 200.0f, 180.0f, 0.0f);
  init_pid(left_leg_turn_pid_, 32.0f, 0.0f, 10.0f, 20.0f, 0.0f);
  init_pid(right_leg_turn_pid_, 32.0f, 0.0f, 10.0f, 20.0f, 0.0f);

  std::array<std::array<rm::f32, 6>, 40> coeff_vec{};
  for (int i = 0; i < 40; ++i) {
    std::copy(&kCtrlP[i * 6], &kCtrlP[i * 6 + 6], coeff_vec[i].begin());
  }
  lqr_controller_.SetLqrCoefficients(coeff_vec);

  ChassisStateEstimatorConfig cfg{};
  state_estimator_.Init(cfg);
  SafeStop();
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
  if (!input.enable_output || !input.run_chassis_update || IsSafeStopMode(input.fsm_mode)) {
    SafeStop();
    return;
  }

  params_.leg_target_length_m = input.target_leg_length_m;
  ComputeActuatorTorque(input, state_output);
}

/**
 * @brief 组合 LQR 与补偿项，计算六电机最终力矩
 */
void chassis::Chassis::ComputeActuatorTorque(const UpdateInput &input,
                                             const ChassisStateEstimatorOutput &state_output) {
  static constexpr rm::f32 kPi = 3.14159265358979323846f;
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

  const rm::f32 left_theta = rm::modules::Wrap(state_output.current.theta_ll, -kPi, kPi);
  const rm::f32 right_theta = rm::modules::Wrap(state_output.current.theta_lr, -kPi, kPi);
  const rm::f32 theta_b = state_output.current.theta_b;
  const bool posture_valid = (theta_b >= -0.7f && theta_b <= 0.7f && left_theta >= -0.8f && left_theta <= 1.4f &&
                              right_theta >= -0.8f && right_theta <= 1.4f);

  const bool use_jump_retract1 = (input.fsm_mode == Fsm::State::kJumpPrep);
  const bool use_jump_extend = (input.fsm_mode == Fsm::State::kJumpPush);
  const bool use_jump_retract2 = (input.fsm_mode == Fsm::State::kJumpRecover);

  if (posture_valid) {
    roll_pid_.Update(0.003f, imu_roll_);
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

    // 离地或跳跃回收时关闭轮端力矩，避免轮系在失去支撑时积分/空转。
    if (use_jump_retract2 || off_ground_in_mid_high_leg) {
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
    // 姿态越界时不再执行 LQR 支撑，转为腿部摆角收回，轮端保持零力矩。
    left_leg_turn_pid_.Update(-2.0f, state_output.current.theta_ll_dot);
    right_leg_turn_pid_.Update(-2.0f, state_output.current.theta_lr_dot);

    left_force_ = 0.0f;
    right_force_ = 0.0f;

    output_.lb_tau = left_leg_.jacobi_00() * left_force_ + left_leg_.jacobi_01() * (-left_leg_turn_pid_.out());
    output_.lf_tau = left_leg_.jacobi_10() * left_force_ + left_leg_.jacobi_11() * (-left_leg_turn_pid_.out());
    output_.rb_tau = right_leg_.jacobi_00() * right_force_ + right_leg_.jacobi_01() * (-right_leg_turn_pid_.out());
    output_.rf_tau = right_leg_.jacobi_10() * right_force_ + right_leg_.jacobi_11() * (-right_leg_turn_pid_.out());

    output_.lf_tau = -output_.lf_tau;
    output_.lb_tau = -output_.lb_tau;

    const rm::f32 left_theta_recover = rm::modules::Wrap(state_output.current.theta_ll, -kPi, kPi);
    const rm::f32 right_theta_recover = rm::modules::Wrap(state_output.current.theta_lr, -kPi, kPi);

    if (left_theta_recover >= 0.0f && left_theta_recover <= 1.4f) {
      output_.lb_tau = 0.0f;
      output_.lf_tau = 0.0f;
    }
    if (right_theta_recover >= 0.0f && right_theta_recover <= 1.4f) {
      output_.rb_tau = 0.0f;
      output_.rf_tau = 0.0f;
    }
    output_.lw_tau = 0.0f;
    output_.rw_tau = 0.0f;
  }
}

/**
 * @brief 根据实测关节力矩估计左右支撑力
 */
void chassis::Chassis::CalSupportForce() {
  static constexpr rm::f32 kPi = 3.14159265358979323846f;

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
