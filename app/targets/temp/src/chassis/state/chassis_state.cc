#include "chassis_state.hpp"

#include <cmath>

/**
 * @brief 初始化状态估计器
 * @param config 配置参数
 */
void ChassisStateEstimator::Init(const ChassisStateEstimatorConfig &config) {
  config_ = config;
  left_leg_ = wbr::LegKinematics(config_.leg_l1_m, config_.leg_l2_m);
  right_leg_ = wbr::LegKinematics(config_.leg_l1_m, config_.leg_l2_m);
  theta_ll_dot_filter_.set_cutoff_frequency(500.0f,
                                            config_.theta_dot_filter_cutoff_hz);
  theta_lr_dot_filter_.set_cutoff_frequency(500.0f,
                                            config_.theta_dot_filter_cutoff_hz);
  Reset();
  speed_estimator_.Init();
}

/**
 * @brief 清空估计输出
 */
void ChassisStateEstimator::Reset() { output_ = {}; }

/**
 * @brief 执行一次状态估计更新
 * @param input 输入反馈
 * @param dt_s  控制周期，单位 s
 */
void ChassisStateEstimator::Update(const ChassisStateEstimatorInput &input) {
  rm::f32 dt_s = input.dt_s;
  if (dt_s <= 0.0f) {
    dt_s = 0.002f;
  }

  UpdateBodyState(input);
  UpdateLegState(input, dt_s);
  UpdateSpeedState(input, dt_s);

  if (input.use_external_s_ref) {
    output_.current.s = input.s_ref_m;
  }
}

/**
 * @brief 获取状态估计输出
 */
const ChassisStateEstimatorOutput &ChassisStateEstimator::GetOutput() const {
  return output_;
}

/**
 * @brief 获取可供控制器使用的 current 状态
 */
const wbr::CurrentState &ChassisStateEstimator::GetCurrentState() const {
  return output_.current;
}

/**
 * @brief 统一执行关节零位/符号标定
 * @param input 原始关节反馈
 * @return 标定后的双腿关节输入
 */
CalibratedLegKinematicsInput ChassisStateEstimator::BuildCalibratedLegInput(
    const ChassisStateEstimatorInput &input) const {
  CalibratedLegKinematicsInput leg_input{};

  leg_input.left.phi1_rad = ConvertLeftPhi1(input.left_leg.front);
  leg_input.left.phi4_rad = ConvertLeftPhi4(input.left_leg.back);
  leg_input.left.w_phi1_rad_s = input.left_leg.front.vel_rad_s;
  leg_input.left.w_phi4_rad_s = input.left_leg.back.vel_rad_s;

  leg_input.right.phi1_rad = ConvertRightPhi1(input.right_leg.front);
  leg_input.right.phi4_rad = ConvertRightPhi4(input.right_leg.back);
  leg_input.right.w_phi1_rad_s = -input.right_leg.front.vel_rad_s;
  leg_input.right.w_phi4_rad_s = -input.right_leg.back.vel_rad_s;

  return leg_input;
}

/**
 * @brief 更新腿部相关状态（腿长、摆角、摆角速度）
 * @param input 输入反馈
 * @param dt_s  控制周期，单位 s
 */
void ChassisStateEstimator::UpdateLegState(
    const ChassisStateEstimatorInput &input, rm::f32 dt_s) {
  if (dt_s <= 0.0f) {
    dt_s = 0.002f;
  }

  const rm::f32 prev_theta_ll = output_.current.theta_ll;
  const rm::f32 prev_theta_lr = output_.current.theta_lr;

  const CalibratedLegKinematicsInput leg_input = BuildCalibratedLegInput(input);
  output_.calibrated_leg_input = leg_input;

  left_leg_.SetPhi1(leg_input.left.phi1_rad);
  left_leg_.SetPhi4(leg_input.left.phi4_rad);
  left_leg_.SetWPhi1(leg_input.left.w_phi1_rad_s);
  left_leg_.SetWPhi4(leg_input.left.w_phi4_rad_s);
  left_leg_.Update(dt_s);

  right_leg_.SetPhi1(leg_input.right.phi1_rad);
  right_leg_.SetPhi4(leg_input.right.phi4_rad);
  right_leg_.SetWPhi1(leg_input.right.w_phi1_rad_s);
  right_leg_.SetWPhi4(leg_input.right.w_phi4_rad_s);
  right_leg_.Update(dt_s);

  output_.left_leg_length_m = left_leg_.l0();
  output_.right_leg_length_m = right_leg_.l0();
  output_.current.l_l = output_.left_leg_length_m;
  output_.current.l_r = output_.right_leg_length_m;

  // 摆角定义沿用原控制链路约定。
  const rm::f32 theta_ll =
      -1.5708f - (-output_.current.theta_b - left_leg_.phi0());
  const rm::f32 theta_lr =
      -1.5708f - (-output_.current.theta_b - right_leg_.phi0());

  output_.left_leg_angle_rad = theta_ll;
  output_.right_leg_angle_rad = theta_lr;
  output_.current.theta_ll = theta_ll;
  output_.current.theta_lr = theta_lr;

  // 摆角速度采用差分并低通。
  const rm::f32 raw_theta_ll_dot = (theta_ll - prev_theta_ll) / dt_s;
  const rm::f32 raw_theta_lr_dot = (theta_lr - prev_theta_lr) / dt_s;
  output_.current.theta_ll_dot = theta_ll_dot_filter_.apply(raw_theta_ll_dot);
  output_.current.theta_lr_dot = theta_lr_dot_filter_.apply(raw_theta_lr_dot);
}

/**
 * @brief 更新机体姿态相关状态
 * @param input 输入反馈
 */
void ChassisStateEstimator::UpdateBodyState(
    const ChassisStateEstimatorInput &input) {
  output_.current.theta_b = input.imu.pitch_rad;
  output_.current.theta_b_dot = input.imu.gyro_y_rad_s;

  // output_.current.phi = input.yaw_motor_rad;
  output_.current.phi = 0.f;
  output_.current.phi_dot = input.imu.gyro_z_rad_s;
}

/**
 * @brief 更新前向速度与位移状态
 * @param input 输入反馈
 * @param dt_s  控制周期，单位 s
 */
void ChassisStateEstimator::UpdateSpeedState(
    const ChassisStateEstimatorInput &input, rm::f32 dt_s) {
  (void)dt_s;

  // 计算轮速
  const f32 left_wheel_vel = input.wheel.left_rad_s *
                             config_.wheel_reduction_ratio *
                             config_.wheel_radius_m;
  const f32 right_wheel_vel = input.wheel.right_rad_s *
                              config_.wheel_reduction_ratio *
                              config_.wheel_radius_m;

  // 转换到机身速度
  const f32 left_speed =
      left_wheel_vel +
      output_.current.l_l * output_.current.theta_ll_dot *
          arm_cos_f32(output_.current.theta_ll) +
      left_leg_.l0_dot() * arm_sin_f32(output_.current.theta_ll);

  const f32 right_speed =
      right_wheel_vel +
      output_.current.l_r * output_.current.theta_lr_dot *
          arm_cos_f32(output_.current.theta_lr) +
      right_leg_.l0_dot() * arm_sin_f32(output_.current.theta_lr);

  output_.wheel_speed_mps = 0.5f * (left_speed + right_speed);

  if (input.use_wheel_speed_direct) {
    // 小陀螺模式直接使用轮速，绕开卡尔曼融合链路。
    output_.raw_wheel_speed_mps = output_.wheel_speed_mps;
    output_.raw_accel_speed_mps = 0.0f;
    output_.current_speed_mps = output_.wheel_speed_mps;
    output_.fused_speed_mps = output_.wheel_speed_mps;
    output_.current.s_dot = output_.wheel_speed_mps;
    output_.current.s += output_.current.s_dot * dt_s;
    return;
  }

  const SpeedEstimatorInput speed_input{
      output_.wheel_speed_mps,
      input.imu.acc_x_mps2,
      input.imu.acc_y_mps2,
      input.imu.pitch_rad,
  };
  speed_estimator_.Update(speed_input);
  output_.raw_wheel_speed_mps = speed_estimator_.GetRawWheelSpeed();
  output_.raw_accel_speed_mps = speed_estimator_.GetRawAccelSpeed();
  output_.current_speed_mps = speed_estimator_.GetCurrentSpeed();
  output_.fused_speed_mps = output_.current_speed_mps;

  // 融合速度越界时回退到轮速，避免异常观测扰动控制。
  if (std::fabs(output_.fused_speed_mps) > config_.max_valid_speed_mps) {
    output_.fused_speed_mps = output_.wheel_speed_mps;
  }

  output_.current.s_dot = output_.current_speed_mps;
  output_.current.s += output_.current.s_dot * dt_s;
}

/**
 * @brief 左腿前关节角度标定
 */
rm::f32
ChassisStateEstimator::ConvertLeftPhi1(const JointFeedback &joint) const {
  return joint.pos_rad + config_.left_phi1_offset_rad;
}

/**
 * @brief 左腿后关节角度标定
 */
rm::f32
ChassisStateEstimator::ConvertLeftPhi4(const JointFeedback &joint) const {
  return joint.pos_rad + config_.left_phi4_offset_rad;
}

/**
 * @brief 右腿前关节角度标定
 */
rm::f32
ChassisStateEstimator::ConvertRightPhi1(const JointFeedback &joint) const {
  return -joint.pos_rad + config_.right_phi1_offset_rad;
}

/**
 * @brief 右腿后关节角度标定
 */
rm::f32
ChassisStateEstimator::ConvertRightPhi4(const JointFeedback &joint) const {
  return -joint.pos_rad + config_.right_phi4_offset_rad;
}
