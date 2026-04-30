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
   -6.4023,  -32.43,  26.987,  48.31,  -17.626,  -23.58,
     -9.1593,  -32.115,  36.484,  53.708,  -35.494,  -29.288,
     -1.8644,  6.9629,  -2.3819,  -10.093,  2.4113,  3.2883,
     -5.9224,  22.484,  -7.9455,  -32.209,  7.5338,  11.073,
     -15.33,  -85.936,  22.293,  81.512,  -12.674,  -27.899,
     -1.0307,  -7.5754,  3.3724,  -4.8252,  1.7317,  -4.2342,
     -6.8171,  19.359,  -17.348,  -24.925,  19.084,  14.102,
     -0.5427,  -0.60498,  -1.8188,  3.8993,  -9.2146,  2.0734,
     -24.863,  49.919,  32.407,  -24.218,  -44.417,  -28.556,
     -3.1443,  3.7348,  7.2199,  1.4122,  -8.6015,  -6.9574,
     -6.4023,  26.987,  -32.43,  -23.58,  -17.626,  48.31,
     -9.1593,  36.484,  -32.115,  -29.288,  -35.494,  53.708,
     1.8644,  2.3819,  -6.9629,  -3.2883,  -2.4113,  10.093,
     5.9224,  7.9455,  -22.484,  -11.073,  -7.5338,  32.209,
     -6.8171,  -17.348,  19.359,  14.102,  19.084,  -24.925,
     -0.5427,  -1.8188,  -0.60498,  2.0734,  -9.2146,  3.8993,
     -15.33,  22.293,  -85.936,  -27.899,  -12.674,  81.512,
     -1.0307,  3.3724,  -7.5754,  -4.2342,  1.7317,  -4.8252,
     -24.863,  32.407,  49.919,  -28.556,  -44.417,  -24.218,
     -3.1443,  7.2199,  3.7348,  -6.9574,  -8.6015,  1.4122,
     6.0136,  3.9864,  -20.631,  -26.942,  35.73,  7.0398,
     7.601,  3.7194,  -28.277,  -30.876,  49.773,  9.6895,
     -1.0462,  -4.6797,  -3.7563,  9.4845,  -3.18,  5.2909,
     -3.3283,  -15.259,  -11.922,  30.577,  -10.412,  16.623,
     31.864,  -46.531,  18.803,  69.74,  55.156,  -39.648,
     1.9362,  -2.071,  -0.043944,  8.4472,  3.5604,  -2.1211,
     -4.1763,  -28.192,  -18.292,  52.655,  -37.332,  -19.427,
     -0.33365,  -1.4512,  2.337,  0.19983,  2.4441,  -11.262,
     -39.092,  -129.33,  64.175,  162.07,  1.1558,  -68.44,
     -2.3916,  -12.611,  4.7066,  12.445,  4.5242,  -5.5482,
     6.0136,  -20.631,  3.9864,  7.0398,  35.73,  -26.942,
     7.601,  -28.277,  3.7194,  9.6895,  49.773,  -30.876,
     1.0462,  3.7563,  4.6797,  -5.2909,  3.18,  -9.4845,
     3.3283,  11.922,  15.259,  -16.623,  10.412,  -30.577,
     -4.1763,  -18.292,  -28.192,  -19.427,  -37.332,  52.655,
     -0.33365,  2.337,  -1.4512,  -11.262,  2.4441,  0.19983,
     31.864,  18.803,  -46.531,  -39.648,  55.156,  69.74,
     1.9362,  -0.043944,  -2.071,  -2.1211,  3.5604,  8.4472,
     -39.092,  64.175,  -129.33,  -68.44,  1.1558,  162.07,
     -2.3916,  4.7066,  -12.611,  -5.5482,  4.5242,  12.445
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

  init_pid(left_l0_pid_, 6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f);
  init_pid(right_l0_pid_, 6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f);
  init_pid(left_l0_pid_jump_two_, 6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f);
  init_pid(right_l0_pid_jump_two_, 6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f);
  init_pid(left_l0_pid_jump_three_, 6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f);
  init_pid(right_l0_pid_jump_three_, 6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f);
  init_pid(roll_pid_, 500.0f, 0.0f, 80.0f, 80.0f, 0.0f);
  init_pid(left_leg_turn_pid_, 20.0f, 0.0f, 0.0f, 15.0f, 0.0f);
  init_pid(right_leg_turn_pid_, 20.0f, 0.0f, 0.0f, 15.0f, 0.0f);

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
        ((input.fsm_mode == Fsm::State::kMidLeg || input.fsm_mode == Fsm::State::kHighLeg) &&
         input.target_leg_length_m > 0.21f) &&
        (left_support_force_est_n_ < kOffGroundSupportForceThresholdN ||
         right_support_force_est_n_ < kOffGroundSupportForceThresholdN);

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
    left_leg_turn_pid_.Update(-5.5f, state_output.current.theta_ll_dot);
    right_leg_turn_pid_.Update(-5.5f, state_output.current.theta_lr_dot);

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
