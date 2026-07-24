#include "include/posture_observer.hpp"

#include <algorithm>
#include <cmath>

namespace wheel_legged {

PostureObservation PostureObserver::Update(const float qw, const float qx, const float qy, const float qz,
                                           const float gx, const float gy, const float gz, const float ax,
                                           const float ay, const float az, const uint32_t timestamp_ms) {
  PostureObservation obs{};
  obs.last_update_ms = timestamp_ms;

  // ── 1. 有限性检查 ──
  bool quat_finite = true;
  if (std::isnan(qw) || std::isnan(qx) || std::isnan(qy) || std::isnan(qz)) {
    obs.fault_flags |= static_cast<uint8_t>(PostureFault::kQuatNaN);
    quat_finite = false;
  }
  if (std::isinf(qw) || std::isinf(qx) || std::isinf(qy) || std::isinf(qz)) {
    obs.fault_flags |= static_cast<uint8_t>(PostureFault::kQuatInf);
    quat_finite = false;
  }
  if (std::isnan(ax) || std::isnan(ay) || std::isnan(az)) {
    obs.fault_flags |= static_cast<uint8_t>(PostureFault::kAccelNaN);
  }
  if (std::isnan(gx) || std::isnan(gy) || std::isnan(gz)) {
    obs.fault_flags |= static_cast<uint8_t>(PostureFault::kGyroNaN);
  }

  // ── 2. 数据新鲜度检查 ──
  if (prev_valid_ && (timestamp_ms - prev_timestamp_ms_) > config_.params.imu_stale_ms) {
    obs.fault_flags |= static_cast<uint8_t>(PostureFault::kImuStale);
  }

  if (!quat_finite) {
    obs.sensor_valid = false;
    return obs;
  }

  // ── 3. 四元数范数检查 ──
  const float norm2 = qw * qw + qx * qx + qy * qy + qz * qz;
  const float norm = std::sqrt(norm2);
  if (norm < 1e-6f) {
    obs.fault_flags |= static_cast<uint8_t>(PostureFault::kQuatNormBad);
    obs.sensor_valid = false;
    return obs;
  }
  if (std::abs(norm - 1.0f) > config_.params.quat_norm_tolerance) {
    obs.fault_flags |= static_cast<uint8_t>(PostureFault::kQuatNormBad);
  }

  // ── 4. 归一化 ──
  const float inv_norm = 1.0f / norm;
  const float nqw = qw * inv_norm;
  const float nqx = qx * inv_norm;
  const float nqy = qy * inv_norm;
  const float nqz = qz * inv_norm;

  // ── 5. 跳变检测（归一化四元数点积）──
  if (prev_valid_) {
    const float dot = nqw * prev_qw_ + nqx * prev_qx_ + nqy * prev_qy_ + nqz * prev_qz_;
    if (std::abs(dot) < config_.params.quat_discontinuity_threshold) {
      obs.fault_flags |= static_cast<uint8_t>(PostureFault::kQuatDiscontinuity);
    }
  }

  // ── 6. 计算 up_body（IMU 传感器系）──
  // 标准 Hamilton R_WB 约定：up_sensor = R_WB^T · [0,0,1]^T
  float ux_imu = 2.0f * (nqx * nqz - nqw * nqy);
  float uy_imu = 2.0f * (nqy * nqz + nqw * nqx);
  float uz_imu = 1.0f - 2.0f * (nqx * nqx + nqy * nqy);
  // 夹紧到 [-1, 1] 以修正浮点舍入
  uz_imu = std::clamp(uz_imu, -1.0f, 1.0f);

  // ── 7. 传感器系 → 机体系旋转（R_BS^T · up_sensor）──
  const auto& R = config_.params;
  obs.up_body_x = R.R_bs_00 * ux_imu + R.R_bs_10 * uy_imu + R.R_bs_20 * uz_imu;
  obs.up_body_y = R.R_bs_01 * ux_imu + R.R_bs_11 * uy_imu + R.R_bs_21 * uz_imu;
  obs.up_body_z = R.R_bs_02 * ux_imu + R.R_bs_12 * uy_imu + R.R_bs_22 * uz_imu;
  obs.up_body_z = std::clamp(obs.up_body_z, -1.0f, 1.0f);

  // ── 8. 陀螺仪旋转到机体系 ──
  obs.gyro_body_x = R.R_bs_00 * gx + R.R_bs_10 * gy + R.R_bs_20 * gz;
  obs.gyro_body_y = R.R_bs_01 * gx + R.R_bs_11 * gy + R.R_bs_21 * gz;
  obs.gyro_body_z = R.R_bs_02 * gx + R.R_bs_12 * gy + R.R_bs_22 * gz;

  // ── 9. 加速度计旋转到机体系 ──
  obs.accel_body_x = R.R_bs_00 * ax + R.R_bs_10 * ay + R.R_bs_20 * az;
  obs.accel_body_y = R.R_bs_01 * ax + R.R_bs_11 * ay + R.R_bs_21 * az;
  obs.accel_body_z = R.R_bs_02 * ax + R.R_bs_12 * ay + R.R_bs_22 * az;

  // ── 10. 衍生量 ──
  obs.gyro_norm_rad_s = std::sqrt(obs.gyro_body_x * obs.gyro_body_x + obs.gyro_body_y * obs.gyro_body_y +
                                  obs.gyro_body_z * obs.gyro_body_z);
  obs.accel_norm_mps2 = std::sqrt(obs.accel_body_x * obs.accel_body_x + obs.accel_body_y * obs.accel_body_y +
                                  obs.accel_body_z * obs.accel_body_z);

  // 低动态 = 加速度模长接近重力 (9.81 m/s²)
  obs.accel_low_dynamic = std::abs(obs.accel_norm_mps2 - 9.81f) < config_.params.accel_low_dynamic_threshold_mps2;

  // ── 11. 传感器有效标志 ──
  obs.sensor_valid = (obs.fault_flags == 0);

  // ── 12. 保存历史值用于下一帧跳变检测 ──
  prev_qw_ = nqw;
  prev_qx_ = nqx;
  prev_qy_ = nqy;
  prev_qz_ = nqz;
  prev_timestamp_ms_ = timestamp_ms;
  prev_valid_ = obs.sensor_valid;

  return obs;
}

}  // namespace wheel_legged
