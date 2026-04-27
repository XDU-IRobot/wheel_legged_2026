#pragma once

#include "librm.hpp"
#include "librm/modules/ahrs/kalman_filter.hpp"

/**
 * @file  chassis/state/speed_estimator.hpp
 * @brief 前向速度估计器接口
 */

/**
 * @brief 速度估计输入
 * @note  全部使用 SI 单位
 */
struct SpeedEstimatorInput {
  float wheel_speed_mps{0.0f};
  float imu_acc_x_mps2{0.0f};
  float imu_acc_y_mps2{0.0f};
  float imu_pitch_rad{0.0f};
};

/**
 * @brief 基于轮速+IMU 前向加速度的卡尔曼速度估计器
 */
class SpeedEstimator {
 public:
  SpeedEstimator();

  /**
   * @brief 初始化滤波器与卡尔曼参数
   */
  void Init();

  /**
   * @brief 速度估计更新
   * @param input 轮速与 IMU 输入
   */
  void Update(const SpeedEstimatorInput &input);

  /**
   * @brief 获取融合后的前向速度
   * @return 前向速度，单位 m/s
   */
  [[nodiscard]] float GetSpeed() const;

  /** @brief 获取原始轮速观测，单位 m/s */
  [[nodiscard]] float GetRawWheelSpeed() const;

  /** @brief 获取原始加速计速度观测（由前向加速度积分），单位 m/s */
  [[nodiscard]] float GetRawAccelSpeed() const;

  /** @brief 获取当前融合速度观测，单位 m/s */
  [[nodiscard]] float GetCurrentSpeed() const;

 private:
  /**
   * @brief 计算机体前向加速度
   * @note  使用 pitch 对 IMU 平面加速度做简化补偿
   */
  float CalcAccelForward(const SpeedEstimatorInput &input);

  rm::modules::KalmanFilter kf_;

  rm::modules::LowPassFilterConstDt<float> accel_x_filter_;
  rm::modules::LowPassFilterConstDt<float> accel_y_filter_;

  float wheel_speed_;

  float accel_bias_;  // 加速度零偏
  int bias_count_;    // 标定计数
  float bias_sum_;

  float accel_forward_;
  float accel_speed_raw_;
  float current_speed_;
};