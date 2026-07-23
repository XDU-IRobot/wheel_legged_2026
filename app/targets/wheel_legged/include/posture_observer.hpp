#pragma once

#include <cstdint>

#include "params.hpp"

namespace wheel_legged {

/// @brief 姿态观测器故障码（位掩码，可组合）
enum class PostureFault : uint8_t {
  kNone = 0x00,
  kQuatNaN = 0x01,            ///< 四元数包含 NaN
  kQuatInf = 0x02,            ///< 四元数包含 Inf
  kQuatNormBad = 0x04,        ///< 四元数范数偏离 1 超过容差
  kQuatDiscontinuity = 0x08,  ///< 四元数相邻帧跳变过大
  kImuStale = 0x10,           ///< IMU 数据超时未更新
  kAccelNaN = 0x20,           ///< 加速度计包含 NaN
  kGyroNaN = 0x40,            ///< 陀螺仪包含 NaN
};

/// @brief 姿态观测器输出：质量检查后的机体系 IMU 数据
struct PostureObservation {
  // 世界竖直方向在机体系下的表示
  float up_body_x{0.0f};
  float up_body_y{0.0f};
  float up_body_z{1.0f};  ///< = cos(tilt), 1.0 = 直立

  // 机体系角速度 [rad/s]（已从传感器系旋转到机体系）
  float gyro_body_x{0.0f};
  float gyro_body_y{0.0f};
  float gyro_body_z{0.0f};

  // 机体系加速度 [m/s²]（已从传感器系旋转到机体系）
  float accel_body_x{0.0f};
  float accel_body_y{0.0f};
  float accel_body_z{0.0f};

  // 衍生量
  float gyro_norm_rad_s{0.0f};    ///< 角速度模长 [rad/s]
  float accel_norm_mps2{0.0f};    ///< 加速度模长 [m/s²]
  bool accel_low_dynamic{false};  ///< 加速度模长接近 1g → 低动态

  // 质量标志
  uint8_t fault_flags{0};    ///< PostureFault 位掩码
  bool sensor_valid{false};  ///< 传感器数据有效（fault_flags == 0 且数据新鲜）

  uint32_t last_update_ms{0};

  bool HasFault(PostureFault f) const { return (fault_flags & static_cast<uint8_t>(f)) != 0; }
};

/// @brief 基于四元数的姿态观测器
///
/// 职责：IMU 数据质量检查、四元数归一化、传感器系→机体系旋转变换、
///       计算 up_body / tilt_cos / 低动态标志。
/// 不负责：倒地阈值、FSM、电机命令。
class PostureObserver {
 public:
  struct Config {
    ::wheel_legged::params::PostureObserverParams params;
  };

  void Init(const Config& config) { config_ = config; }

  /// @brief 处理一帧 IMU 原始数据（传感器系），输出质量检查后的机体系观测
  PostureObservation Update(float qw, float qx, float qy, float qz, float gx, float gy, float gz, float ax, float ay,
                            float az, uint32_t timestamp_ms);

 private:
  Config config_{};

  // 跳变检测用历史值（存储上一帧归一化后的四元数）
  float prev_qw_{0.0f};
  float prev_qx_{0.0f};
  float prev_qy_{0.0f};
  float prev_qz_{0.0f};
  uint32_t prev_timestamp_ms_{0};
  bool prev_valid_{false};
};

}  // namespace wheel_legged
