#include "speed_estimator.hpp"
#include "arm_math.h"

#include <cmath>
#include <cstring>

/**
 * @brief 构造函数，初始化内部状态
 */
SpeedEstimator::SpeedEstimator()
    : kf_(2, 0, 2),
      wheel_speed_(0.f),
      accel_bias_(0.f),
      bias_count_(0),
      bias_sum_(0.f),
      accel_forward_(0.f),
      accel_speed_raw_(0.f),
      current_speed_(0.f) {}

/**
 * @brief 初始化滤波器与卡尔曼矩阵
 * @note  当前默认控制周期按 2ms 配置
 */
void SpeedEstimator::Init() {
  accel_x_filter_.set_cutoff_frequency(500.f, 10.f);
  accel_y_filter_.set_cutoff_frequency(500.f, 10.f);

  kf_.UseAutoAdjustment = 0;

  std::memset(kf_.xhat_data, 0, sizeof(float) * 2);
  std::memset(kf_.xhatminus_data, 0, sizeof(float) * 2);
  std::memset(kf_.MeasuredVector, 0, sizeof(float) * 2);
  std::memset(kf_.FilteredValue, 0, sizeof(float) * 2);

  wheel_speed_ = 0.f;
  accel_bias_ = 0.f;
  bias_count_ = 0;
  bias_sum_ = 0.f;
  accel_forward_ = 0.f;
  accel_speed_raw_ = 0.f;
  current_speed_ = 0.f;

  float dt = 0.002f;

  float F[4] = {1, dt, 0, 1};

  float Q[4] = {0.0005f, 0, 0, 0.04f};

  float R[4] = {0.5f, 0, 0, 2.0f};

  float P[4] = {10, 0, 0, 10};

  float H[4] = {1, 0, 0, 1};

  std::memcpy(kf_.F_data, F, sizeof(F));
  std::memcpy(kf_.Q_data, Q, sizeof(Q));
  std::memcpy(kf_.R_data, R, sizeof(R));
  std::memcpy(kf_.P_data, P, sizeof(P));
  std::memcpy(kf_.H_data, H, sizeof(H));
}

/**
 * @brief 由 IMU 平面加速度估计前向加速度
 * @param input 当前输入
 * @return 前向加速度，单位 m/s^2
 */
float SpeedEstimator::CalcAccelForward(const SpeedEstimatorInput &input) {
  const float accel_x = accel_x_filter_.apply(input.imu_acc_x_mps2);
  const float accel_y = accel_y_filter_.apply(input.imu_acc_y_mps2);
  return accel_x - accel_y * arm_sin_f32(input.imu_pitch_rad);
}

/**
 * @brief 更新速度估计
 * @param input 当前轮速和 IMU 输入
 * @note  含启动零偏标定与低速锁零逻辑
 */
void SpeedEstimator::Update(const SpeedEstimatorInput &input) {
  constexpr float kDtS = 0.002f;

  wheel_speed_ = input.wheel_speed_mps;
  accel_forward_ = CalcAccelForward(input);

  // 启动阶段自动估计加速度零偏。
  if (bias_count_ < 1500) {
    bias_sum_ += accel_forward_;
    bias_count_++;

    accel_bias_ = bias_sum_ / static_cast<float>(bias_count_);
  }

  accel_forward_ -= accel_bias_;

  // 静止判据触发后将加速度置零，抑制漂移。
  if (std::fabs(wheel_speed_) < 0.02f && std::fabs(accel_forward_) < 0.5f) {
    accel_forward_ = 0;
  }

  // 原始加速计速度观测：仅用于调试，采用固定周期积分。
  accel_speed_raw_ += accel_forward_ * kDtS;
  if (std::fabs(wheel_speed_) < 0.02f && std::fabs(accel_forward_) < 0.2f) {
    accel_speed_raw_ = 0.0f;
  }

  // Kalman 观测向量: [轮速, 前向加速度]。
  kf_.MeasuredVector[0] = wheel_speed_;
  kf_.MeasuredVector[1] = accel_forward_;

  kf_.Update();
  current_speed_ = kf_.FilteredValue[0];
}

/**
 * @brief 获取当前融合速度
 * @return 前向速度，单位 m/s
 */
float SpeedEstimator::GetSpeed() const { return kf_.FilteredValue[0]; }

float SpeedEstimator::GetRawWheelSpeed() const { return wheel_speed_; }

float SpeedEstimator::GetRawAccelSpeed() const { return accel_speed_raw_; }

float SpeedEstimator::GetCurrentSpeed() const { return current_speed_; }
