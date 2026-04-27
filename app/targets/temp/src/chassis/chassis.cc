#include "chassis.hpp"
#include <stdio.h>

using namespace rm;
using namespace rm::modules;
using namespace rm::device;
using namespace wbr;
using namespace hal;

f32 left_force, right_force = 0.f;

namespace {
// 主控制周期（500 Hz）。
constexpr f32 kControlDtS = 0.002f;

constexpr f32 kLegL1M = 0.215f;
constexpr f32 kLegL2M = 0.254f;
constexpr f32 kSpringTorqueScale = 90.0f;

// 前馈参数：F_gravity = (1/2 m_b + eta m_l) g
//            F_inertial = (1/2 m_b + eta m_l) * l / (2 R_l) * phi_dot * s_dot
constexpr f32 kBodyMassKg = 16.0f;
constexpr f32 kLegMassKg = 2.3f;
constexpr f32 kGravityMps2 = 9.81f;
constexpr f32 kWheelRadiusM = 0.2025f;
constexpr f32 kOffGroundSupportForceThresholdN = 10.0f;

// CAD 数据表（来源：HerKules_VOCAL_SJ_LQR_v4_with_data.m 的 Leg_data_l）。
// eta 定义为 eta = l_w / l，其中 l_w 为轮心到腿部质心距离，l 为虚拟腿长。
constexpr f32 kEtaLookupLegLengthM[] = {
    0.11f, 0.12f, 0.13f, 0.14f, 0.15f, 0.16f, 0.17f, 0.18f, 0.19f, 0.20f, 0.21f, 0.22f,
    0.23f, 0.24f, 0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.30f, 0.31f, 0.32f, 0.33f, 0.34f,
};

constexpr f32 kEtaLookupLwM[] = {
    0.061990, 0.067466, 0.072986, 0.078550, 0.084158, 0.089810, 0.095506, 0.101246,
    0.107030, 0.112858, 0.118730, 0.124646, 0.130606, 0.136610, 0.142658, 0.148750,
    0.154886, 0.161066, 0.167290, 0.173558, 0.179870, 0.186226, 0.192626, 0.199070,
};

f32 ComputeEtaFromLegLength(f32 leg_length_m) {
  constexpr size_t kCount = sizeof(kEtaLookupLegLengthM) / sizeof(f32);
  const f32 min_l = kEtaLookupLegLengthM[0];
  const f32 max_l = kEtaLookupLegLengthM[kCount - 1];
  const f32 l = Clamp(leg_length_m, min_l, max_l);

  for (size_t i = 0; i + 1 < kCount; ++i) {
    const f32 l0 = kEtaLookupLegLengthM[i];
    const f32 l1 = kEtaLookupLegLengthM[i + 1];
    if (l >= l0 && l <= l1) {
      const f32 ratio = (l - l0) / (l1 - l0);
      const f32 lw = kEtaLookupLwM[i] + (kEtaLookupLwM[i + 1] - kEtaLookupLwM[i]) * ratio;
      return lw / l;
    }
  }

  return kEtaLookupLwM[kCount - 1] / max_l;
}

f32 ComputeSpringTorqueFromLegLength(f32 leg_length_m) {
  const f32 x =
      acosf((kLegL2M * kLegL2M + kLegL1M * kLegL1M - leg_length_m * leg_length_m) / (2.0f * kLegL2M * kLegL1M));
  return -(sqrtf(1082.0f - 1070.0f * cosf(x)) * sinf(x - PI / 18.0f) /
           (sqrtf(404.0f - 177.0f * cosf(x - PI / 18.0f)) * sinf(x))) *
         kSpringTorqueScale;
}

constexpr f32 kCtrlP[240] = {
    -2.494,  -11.314,  8.6684,    16.033,    -5.0423,  -7.1215, -4.4352,  -14.034,  15.629,   22.941,    -13.921,
    -12.103, -0.78242, 2.5615,    -0.83756,  -4.9463,  2.2927,  1.154,    -5.5465,  18.287,   -6.0248,   -35.178,
    16.331,  8.3203,   -13.53,    -50.072,   11.345,   53.457,  -30.045,  -12.072,  -0.42523, -4.4126,   1.2734,
    -1.4889, 0.056401, -1.4827,   -2.2208,   15.652,   -24.82,  -40.927,  57.572,   29.386,   -0.052456, -0.27998,
    -2.3225, 0.51579,  -0.75639,  2.9708,    -20.707,  57.551,  18.525,   -43.052,  -32.983,  -17.281,   -2.3227,
    4.5849,  3.645,    -1.1418,   -5.6754,   -3.5824,  -2.494,  8.6684,   -11.314,  -7.1215,  -5.0423,   16.033,
    -4.4352, 15.629,   -14.034,   -12.103,   -13.921,  22.941,  0.78242,  0.83756,  -2.5615,  -1.154,    -2.2927,
    4.9463,  5.5465,   6.0248,    -18.287,   -8.3203,  -16.331, 35.178,   -2.2208,  -24.82,   15.652,    29.386,
    57.572,  -40.927,  -0.052456, -2.3225,   -0.27998, 2.9708,  -0.75639, 0.51579,  -13.53,   11.345,    -50.072,
    -12.072, -30.045,  53.457,    -0.42523,  1.2734,   -4.4126, -1.4827,  0.056401, -1.4889,  -20.707,   18.525,
    57.551,  -17.281,  -32.983,   -43.052,   -2.3227,  3.645,   4.5849,   -3.5824,  -5.6754,  -1.1418,   2.5402,
    0.2878,  -9.1394,  -7.6683,   18.772,    -0.85692, 4.0535,  -0.42671, -15.537,  -10.926,  31.146,    0.44319,
    -1.1035, 0.50541,  -2.4381,   1.2344,    -2.1999,  3.3908,  -7.8139,  3.4065,   -17.308,  8.977,     -15.725,
    23.985,  19.218,   -12.538,   15.969,    16.072,   76.598,  -39.481,  0.64413,  0.55597,  -0.044438, 2.342,
    4.2572,  -2.047,   1.2649,    -14.808,   -54.699,  42.054,  -40.441,  23.515,   0.15168,  -0.77208,  -0.7123,
    1.0587,  0.50401,  -5.9725,   -30.283,   -124.1,   67.032,  168.12,   -23.886,  -57.758,  -2.3512,   -11.915,
    6.0507,  13.63,    0.48573,   -5.4422,   2.5402,   -9.1394, 0.2878,   -0.85692, 18.772,   -7.6683,   4.0535,
    -15.537, -0.42671, 0.44319,   31.146,    -10.926,  1.1035,  2.4381,   -0.50541, -3.3908,  2.1999,    -1.2344,
    7.8139,  17.308,   -3.4065,   -23.985,   15.725,   -8.977,  1.2649,   -54.699,  -14.808,  23.515,    -40.441,
    42.054,  0.15168,  -0.7123,   -0.77208,  -5.9725,  0.50401, 1.0587,   19.218,   15.969,   -12.538,   -39.481,
    76.598,  16.072,   0.64413,   -0.044438, 0.55597,  -2.047,  4.2572,   2.342,    -30.283,  67.032,    -124.1,
    -57.758, -23.886,  168.12,    -2.3512,   6.0507,   -11.915, -5.4422,  0.48573,  13.63,
};
}  // namespace

Chassis::Chassis(f32 l1, f32 l2) : base_torque{}, left_leg_(l1, l2), right_leg_(l1, l2) {}

/**
 * @brief 初始化底盘控制器与状态估计器参数。
 * @note  本函数只完成算法对象构建与参数配置。
 */
void Chassis::Init() {
  // 初始化 pid
  left_l0_pid = std::make_unique<PID>(PID{6500, 0.15, 50000, 170, 30});
  right_l0_pid = std::make_unique<PID>(PID{6500, 0.15, 50000, 170, 30});
  left_l0_pid_jump_two = std::make_unique<PID>(PID{6000, 0., 40000, 250, 0});
  right_l0_pid_jump_two = std::make_unique<PID>(PID{6000, 0., 40000, 250, 0});
  left_l0_pid_jump_three = std::make_unique<PID>(PID{6500, 0.15, 50000, 170, 30});
  right_l0_pid_jump_three = std::make_unique<PID>(PID{6500, 0.15, 50000, 170, 30});
  roll_pid = std::make_unique<PID>(PID{500, 0, 80, 80, 0});
  left_leg_turn_pid = std::make_unique<PID>(PID{20, 0, 0, 15, 0});
  right_leg_turn_pid = std::make_unique<PID>(PID{20, 0, 0, 15, 0});

  std::vector<std::array<f32, 6>> coeff_vec(40);

  for (int i = 0; i < 40; ++i) {
    std::copy(&kCtrlP[i * 6], &kCtrlP[i * 6 + 6], coeff_vec[i].begin());
  }
  wbr_controller_.SetLqrCoefficients(coeff_vec);

  ChassisStateEstimatorConfig estimator_cfg;
  state_estimator_.Init(estimator_cfg);
}

/**
 * @brief 底盘控制单步更新
 * @note  对外统一入口：状态估计 + WBR + 最终力矩合成。
 */
void Chassis::Update(const UpdateInput &input) {
  ChassisStateEstimatorInput estimator_input = input.estimator_input;
  estimator_input.dt_s = estimator_input.dt_s > 0.0f ? estimator_input.dt_s : kControlDtS;
  estimator_input.s_ref_m = input.expected.s;
  estimator_input.use_external_s_ref = false;
  estimator_input.use_wheel_speed_direct = (input.fsm_mode == Fsm::State::kSmallGyro);

  // 状态估计模块统一负责状态派生与关节标定。
  state_estimator_.Update(estimator_input);
  const ChassisStateEstimatorOutput &state_output = state_estimator_.GetOutput();
  const CalibratedLegKinematicsInput &calibrated_leg_input = state_output.calibrated_leg_input;

  imu_roll_ = estimator_input.imu.roll_rad;

  left_leg_.SetPhi1(calibrated_leg_input.left.phi1_rad);
  left_leg_.SetPhi4(calibrated_leg_input.left.phi4_rad);
  left_leg_.SetWPhi1(calibrated_leg_input.left.w_phi1_rad_s);
  left_leg_.SetWPhi4(calibrated_leg_input.left.w_phi4_rad_s);
  left_leg_.Update();

  right_leg_.SetPhi1(calibrated_leg_input.right.phi1_rad);
  right_leg_.SetPhi4(calibrated_leg_input.right.phi4_rad);
  right_leg_.SetWPhi1(calibrated_leg_input.right.w_phi1_rad_s);
  right_leg_.SetWPhi4(calibrated_leg_input.right.w_phi4_rad_s);
  right_leg_.Update();

  lf_real_torque_ = estimator_input.left_leg.front.torque_nm;
  lb_real_torque_ = estimator_input.left_leg.back.torque_nm;
  rf_real_torque_ = estimator_input.right_leg.front.torque_nm;
  rb_real_torque_ = estimator_input.right_leg.back.torque_nm;
  current_state_ = state_output.current;
  first_wheel_speed_ = state_output.wheel_speed_mps;
  Cal_Fn();

  params_.leg_target_length_m = input.target_leg_length_m;

  // 2. 调用LQR/VMC解算器计算所需力矩
  base_torque = wbr_controller_.ComputeControl(current_state_, input.expected);
  ComputeUltiTau(input.fsm_mode);

  // if (left_Fn_ < 30.0f) {
  //   left_wheel_tau_ = 0.0f;
  // }
  // if (right_Fn_ < 30.0f) {
  //   right_wheel_tau_ = 0.0f;
  // }

  output_.lf_tau = lf_tau_;
  output_.lb_tau = lb_tau_;
  output_.rf_tau = rf_tau_;
  output_.rb_tau = rb_tau_;
  output_.lw_tau = left_wheel_tau_;
  output_.rw_tau = right_wheel_tau_;
  output_.wheel_speed_mps = first_wheel_speed_;
  output_.raw_wheel_speed_mps = state_output.raw_wheel_speed_mps;
  output_.raw_accel_speed_mps = state_output.raw_accel_speed_mps;
  output_.current_speed_mps = state_output.current_speed_mps;
  output_.left_Fn = left_Fn_;
  output_.right_Fn = right_Fn_;
  output_.current_state = current_state_;
}
/**
 * @brief 结合补偿项计算六电机最终力矩。
 * @note  在模型力矩基础上叠加弹簧补偿、滚转耦合与雅可比映射。
 */
void Chassis::ComputeUltiTau(Fsm::State mode) {
  const auto set_all_zero = [this]() {
    lb_tau_ = 0.0f;
    lf_tau_ = 0.0f;
    rb_tau_ = 0.0f;
    rf_tau_ = 0.0f;
    left_wheel_tau_ = 0.0f;
    right_wheel_tau_ = 0.0f;
  };

  if (mode == Fsm::State::kNoForce) {
    set_all_zero();
    return;
  }

  const f32 eta_left = ComputeEtaFromLegLength(left_leg_.l0());
  const f32 eta_right = ComputeEtaFromLegLength(right_leg_.l0());
  const f32 effective_mass_left_kg = 0.5f * kBodyMassKg + eta_left * kLegMassKg;
  const f32 effective_mass_right_kg = 0.5f * kBodyMassKg + eta_right * kLegMassKg;
  const f32 gravity_ff_left = effective_mass_left_kg * kGravityMps2;
  const f32 gravity_ff_right = effective_mass_right_kg * kGravityMps2;
  const f32 wheel_radius_m = (kWheelRadiusM > 1e-5f) ? kWheelRadiusM : 1e-5f;

  const f32 avg_leg_length_m = 0.5f * (left_leg_.l0() + right_leg_.l0());
  left_l0_pid->Update(params_.leg_target_length_m, avg_leg_length_m);
  right_l0_pid->Update(params_.leg_target_length_m, avg_leg_length_m);
  const f32 length_force_base = 0.5f * (left_l0_pid->out() + right_l0_pid->out());
  l_spring_torque_ = ComputeSpringTorqueFromLegLength(left_leg_.l0());
  r_spring_torque_ = ComputeSpringTorqueFromLegLength(right_leg_.l0());

  const f32 left_theta = Wrap(current_state_.theta_ll, -M_PI, M_PI);
  const f32 right_theta = Wrap(current_state_.theta_lr, -M_PI, M_PI);
  const f32 theta_b = current_state_.theta_b;
  const bool posture_valid = (theta_b >= -0.7f && theta_b <= 0.7f && left_theta >= -0.8f && left_theta <= 1.4f &&
                              right_theta >= -0.8f && right_theta <= 1.4f);
  const bool use_jump_retract1 = (mode == Fsm::State::kJumpRetract1);
  const bool use_jump_extend = (mode == Fsm::State::kJumpExtend);
  const bool use_jump_retract2 = (mode == Fsm::State::kJumpRetract2);

  if (posture_valid) {
    roll_pid->Update(0.003f, imu_roll_);
    f32 leg_length_force = length_force_base;

    if (use_jump_extend) {
      left_l0_pid_jump_two->Update(params_.leg_target_length_m, avg_leg_length_m);
      right_l0_pid_jump_two->Update(params_.leg_target_length_m, avg_leg_length_m);
      leg_length_force = 0.5f * (left_l0_pid_jump_two->out() + right_l0_pid_jump_two->out());
      // kJumpExtend keeps normal roll coupling but disables spring compensation.
      left_force_ = leg_length_force + roll_pid->out();
      right_force_ = leg_length_force - roll_pid->out();
    } else if (use_jump_retract2) {
      left_l0_pid_jump_three->Update(params_.leg_target_length_m, avg_leg_length_m);
      right_l0_pid_jump_three->Update(params_.leg_target_length_m, avg_leg_length_m);
      leg_length_force = 0.5f * (left_l0_pid_jump_three->out() + right_l0_pid_jump_three->out());
      left_force_ = leg_length_force + roll_pid->out() + l_spring_torque_;
      right_force_ = leg_length_force - roll_pid->out() + r_spring_torque_;
    } else if (use_jump_retract1) {
      left_force_ = leg_length_force + roll_pid->out() + l_spring_torque_;
      right_force_ = leg_length_force - roll_pid->out() + r_spring_torque_;
    } else {
      const f32 inertial_ff_left = effective_mass_left_kg * (left_leg_.l0() / (2.0f * wheel_radius_m)) *
                                   current_state_.phi_dot * current_state_.s_dot;
      const f32 inertial_ff_right = effective_mass_right_kg * (right_leg_.l0() / (2.0f * wheel_radius_m)) *
                                    current_state_.phi_dot * current_state_.s_dot;

      left_force_ = leg_length_force + gravity_ff_left + roll_pid->out() - inertial_ff_left + l_spring_torque_;
      right_force_ = leg_length_force + gravity_ff_right - roll_pid->out() + inertial_ff_right + r_spring_torque_;
    }

    left_force = leg_length_force;
    right_force = leg_length_force;

    const bool off_ground_in_mid_leg =
        (mode == Fsm::State::kNormalMidLeg) &&
        (left_Fn_ < kOffGroundSupportForceThresholdN || right_Fn_ < kOffGroundSupportForceThresholdN);

    if (use_jump_retract2 || off_ground_in_mid_leg) {
      left_wheel_tau_ = 0.f;
      right_wheel_tau_ = 0.f;
    } else {
      left_wheel_tau_ = -base_torque.t_wl;
      right_wheel_tau_ = base_torque.t_wr;
    }

    const f32 t_bl_cmd = -base_torque.t_bl;
    const f32 t_br_cmd = -base_torque.t_br;

    lb_tau_ = left_leg_.jacobi_00() * left_force_ + left_leg_.jacobi_01() * t_bl_cmd;
    lf_tau_ = left_leg_.jacobi_10() * left_force_ + left_leg_.jacobi_11() * t_bl_cmd;
    rb_tau_ = right_leg_.jacobi_00() * right_force_ + right_leg_.jacobi_01() * t_br_cmd;
    rf_tau_ = right_leg_.jacobi_10() * right_force_ + right_leg_.jacobi_11() * t_br_cmd;

    lf_tau_ = -lf_tau_;
    lb_tau_ = -lb_tau_;
  } else {
    left_leg_turn_pid->Update(-5.5, current_state_.theta_ll_dot);
    right_leg_turn_pid->Update(-5.5, current_state_.theta_lr_dot);

    left_force_ = 0.f;
    right_force_ = 0.f;

    lb_tau_ = left_leg_.jacobi_00() * left_force_ + left_leg_.jacobi_01() * (-left_leg_turn_pid->out());
    lf_tau_ = left_leg_.jacobi_10() * left_force_ + left_leg_.jacobi_11() * (-left_leg_turn_pid->out());
    rb_tau_ = right_leg_.jacobi_00() * right_force_ + right_leg_.jacobi_01() * (-right_leg_turn_pid->out());
    rf_tau_ = right_leg_.jacobi_10() * right_force_ + right_leg_.jacobi_11() * (-right_leg_turn_pid->out());

    lf_tau_ = -lf_tau_;
    lb_tau_ = -lb_tau_;

    const f32 left_theta_recover = Wrap(current_state_.theta_ll, -M_PI, M_PI);
    const f32 right_theta_recover = Wrap(current_state_.theta_lr, -M_PI, M_PI);

    if (left_theta_recover >= 0 && left_theta_recover <= 1.4f) {
      lb_tau_ = 0;
      lf_tau_ = 0;
    }
    if (right_theta_recover >= 0 && right_theta_recover <= 1.4f) {
      rb_tau_ = 0;
      rf_tau_ = 0;
    }
    left_wheel_tau_ = 0;
    right_wheel_tau_ = 0;
  }
}

/**
 * @brief 系统辨识专用：只激励右腿，使用 LQR 利用框架跟踪 -0.6~0.6 的角度
 *        并在 0.2~0.35 的范围内变化腿长
 */
void Chassis::ComputeRightLegExcitation(float dt_s) {
  sysid_time_s_ += dt_s;

  // 设定较慢的速度规律
  const float T_angle = 8.0f;    // 摆角周期8秒
  const float T_length = 11.0f;  // 腿长周期11秒（不同步，能解耦出更多数据）

  float phase_angle = 2.0f * M_PI * (sysid_time_s_ / T_angle);
  float phase_length = 2.0f * M_PI * (sysid_time_s_ / T_length);

  // 1. 目标摆角 -0.6 到 0.6
  float target_theta = 0.6f * arm_sin_f32(phase_angle);

  // 2. 目标腿长 0.2 到 0.35 (中心 0.275，振幅 0.075)
  float target_length = 0.275f + 0.075f * arm_sin_f32(phase_length);

  // 3. 更新腿长环 PID
  right_l0_pid->Update(target_length, right_leg_.l0());
  float length_force = right_l0_pid->out();

  // 4. 为了充分利用 LQR：
  // LQR 默认目标角度0，当我们把传入 LQR 的状态误差加上目标摆角时，
  // LQR 就会主动去消除这个新误差，从而完美复用 LQR 中的动力学 K 矩阵！
  wbr::CurrentState fake_state = current_state_;
  fake_state.theta_lr = current_state_.theta_lr - target_theta;

  wbr::ExpectedState expected_zero{};
  // 我们同时也把 LQR 计算所需的腿长替换成目标的
  params_.leg_target_length_m = target_length;

  wbr::MotorTorque fake_torque = wbr_controller_.ComputeControl(fake_state, expected_zero);

  // 提取 LQR 给出的右侧髋关节力矩
  float t_br_cmd = -fake_torque.t_br;

  // 5. 雅可比映射到实际电机 (腿长推力 + 髋部角力矩 混合)
  float rp_tau = right_leg_.jacobi_00() * length_force + right_leg_.jacobi_01() * t_br_cmd;
  float ra_tau = right_leg_.jacobi_10() * length_force + right_leg_.jacobi_11() * t_br_cmd;

  // 6. 切断其他电机，防止翻车
  left_wheel_tau_ = 0.0f;
  right_wheel_tau_ = 0.0f;
  lf_tau_ = 0.0f;
  lb_tau_ = 0.0f;

  // 给右腿电机赋值 (反号规则和正常模式一致)
  rb_tau_ = rp_tau;
  rf_tau_ = ra_tau;

  // 7. 提取平滑角加速度
  float current_theta_dot = current_state_.theta_lr_dot;
  float raw_ddtheta = (current_theta_dot - last_theta_lr_dot_) / dt_s;
  last_theta_lr_dot_ = current_theta_dot;
  filtered_ddtheta_lr_ = 0.1f * raw_ddtheta + 0.9f * filtered_ddtheta_lr_;

  // 通过串口输出数据用于拟合，降频到 100Hz 以免撑爆串口缓冲
  static uint32_t print_cnt = 0;
  if (++print_cnt % 5 == 0) {
    printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n", sysid_time_s_, t_br_cmd, current_state_.theta_lr,
           current_state_.theta_lr_dot, filtered_ddtheta_lr_, right_leg_.l0(), length_force);
  }
}

/**
 * @brief 基于实测关节力矩估计双腿支撑力。
 * @note  通过雅可比逆关系把关节空间力矩映射回虚拟腿空间力。
 */
void Chassis::Cal_Fn() {
  float l_f = 0.0f;
  float r_f = 0.0f;

  // 对照文档：F_wh,i ≈ F_bh,i + m_li (g + a_e - l_bi_ddot * cos(theta_li))。
  // 目前先忽略 a_e（机体平动加速度项），保留腿长加速度补偿项。
  // 站立基线采用与控制前馈一致的等效重力质量：(1/2 m_b + eta m_l)。
  constexpr float kLegMassPerSideKg = kLegMassKg;

  // J^{-1} 的标准行列式应为 det = J00 * J11 - J01 * J10。
  const float det_l = left_leg_.jacobi_00() * left_leg_.jacobi_11() - left_leg_.jacobi_01() * left_leg_.jacobi_10();
  const float det_r = right_leg_.jacobi_00() * right_leg_.jacobi_11() - right_leg_.jacobi_01() * right_leg_.jacobi_10();
  if (fabsf(det_l) < 1e-5f || fabsf(det_r) < 1e-5f || fabsf(left_leg_.l0()) < 1e-5f || fabsf(right_leg_.l0()) < 1e-5f) {
    left_Fn_ = 0.0f;
    right_Fn_ = 0.0f;
    return;
  }

  // 腿轴向虚拟力（左腿沿现有关节正方向）。
  l_f = -(left_leg_.jacobi_11() * lb_real_torque_ - left_leg_.jacobi_01() * lf_real_torque_) / det_l;

  // 右腿力矩先统一到与左腿一致的关节正方向，再走同一套解算公式。
  const float rf_tau_unified = -rf_real_torque_;
  const float rb_tau_unified = -rb_real_torque_;
  r_f = -(right_leg_.jacobi_11() * rb_tau_unified - right_leg_.jacobi_01() * rf_tau_unified) / det_r;

  const float theta_ll = Wrap(current_state_.theta_ll, -M_PI, M_PI);
  const float theta_lr = Wrap(current_state_.theta_lr, -M_PI, M_PI);

  // F_bh,i：腿轴向力在机体竖向支撑方向上的分量。
  const float left_F_bh = l_f * arm_cos_f32(theta_ll);
  const float right_F_bh = r_f * arm_cos_f32(theta_lr);

  // 文档近似 l_bi ≈ (1 - eta_l) * l_i，故 l_bi_ddot ≈ (1 - eta_l) * l_i_ddot。
  const float left_l0_ddot = (left_leg_.l0_dot() - left_l0_dot_prev_) / kControlDtS;
  const float right_l0_ddot = (right_leg_.l0_dot() - right_l0_dot_prev_) / kControlDtS;
  left_l0_dot_prev_ = left_leg_.l0_dot();
  right_l0_dot_prev_ = right_leg_.l0_dot();

  const float eta_left = ComputeEtaFromLegLength(left_leg_.l0());
  const float eta_right = ComputeEtaFromLegLength(right_leg_.l0());
  const float left_leg_dyn_comp = -(1.0f - eta_left) * left_l0_ddot * arm_cos_f32(theta_ll);
  const float right_leg_dyn_comp = -(1.0f - eta_right) * right_l0_ddot * arm_cos_f32(theta_lr);

  const float gravity_support_left = (0.5f * kBodyMassKg + eta_left * kLegMassPerSideKg) * kGravityMps2;
  const float gravity_support_right = (0.5f * kBodyMassKg + eta_right * kLegMassPerSideKg) * kGravityMps2;

  // 动态项仍按腿质量补偿，避免把机体质量误并入腿长加速度项。
  left_Fn_ = left_F_bh + gravity_support_left + kLegMassPerSideKg * left_leg_dyn_comp;
  right_Fn_ = right_F_bh + gravity_support_right + kLegMassPerSideKg * right_leg_dyn_comp;
}
