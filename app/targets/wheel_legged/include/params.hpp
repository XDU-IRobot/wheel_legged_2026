#pragma once

#include <array>
#include <cstdint>
#include <librm.hpp>
#include "librm.hpp"
#include "targets/wheel_legged/include/utils/yaw_speed_feedforward.hpp"
namespace wheel_legged::params {

#ifndef WHEEL_LEGGED_ROBOT_VARIANT
#define WHEEL_LEGGED_ROBOT_VARIANT 2
#endif

/// @brief PID 增益结构体
struct PidGains {
  float kp;        ///< 比例增益
  float ki;        ///< 积分增益
  float kd;        ///< 微分增益
  float max_out;   ///< 输出限幅
  float max_iout;  ///< 积分输出限幅
};

struct StairClimbParams {
  float high_leg_length_m;
  float hook_leg_length_m;
  float retract_leg_length_m;
  float settle_leg_length_m;
  float contact_theta_threshold_rad;
  float hook_theta_target_rad;
  float retract_theta_target_rad;
  float hook_theta_tolerance_rad;
  float leg_length_tolerance_m;
  float theta_dot_tolerance_rad_s;
  float settle_theta_tolerance_rad;
  float settle_theta_target_rad;
  float settle_pitch_tolerance_rad;
  float settle_pitch_dot_tolerance_rad_s;
  float settle_roll_tolerance_rad;
  std::uint32_t hook_stable_ms;
  std::uint32_t retract_stable_ms;
  std::uint32_t settle_stable_ms;
  std::uint32_t hook_timeout_ms;
  std::uint32_t retract_timeout_ms;
  std::uint32_t settle_timeout_ms;
  PidGains theta_pid;
};

// ══════════════════════════════════════════════════════════════════════════════
// 三变体完全相同的公共参数（仅保留真正不随变体变化的常量与配置）
// ══════════════════════════════════════════════════════════════════════════════
namespace common {

constexpr float kPi = 3.14159265358979323846f;

// ── 系统级参数 ──
namespace main {
inline constexpr float kControlLoopFrequencyHz = 500.0f;  ///< 主控制循环频率 [Hz]
}  // namespace main

// ── CAN/UART 通信配置 ──
namespace globals {
constexpr double kJointCanTxLimitHz = 4000.0;   ///< 关节电机 CAN 发送频率上限 [Hz]
constexpr double kWheelCanTxLimitHz = 4000.0;   ///< 轮毂电机 CAN 发送频率上限 [Hz]
constexpr double kGimbalCanTxLimitHz = 4000.0;  ///< 云台电机 CAN 发送频率上限 [Hz]

constexpr std::size_t kDr16UartRxBufferSize = 18;      ///< DR16 遥控器串口接收缓冲区大小 [byte]
constexpr std::size_t kImuUartRxBufferSize = 518;      ///< IMU 串口接收缓冲区大小 [byte]
constexpr std::size_t kRefereeUartRxBufferSize = 256;  ///< 裁判系统串口接收缓冲区大小 [byte]
constexpr std::size_t kDypUartRxBufferSize = 8;        ///< DYP 超声波串口接收缓冲区大小 [byte]

using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

constexpr std::uint16_t kDmLfMasterId = 0x13;  ///< 左前关节主电机 CAN ID
constexpr std::uint16_t kDmLfSlaveId = 0x03;   ///< 左前关节从电机 CAN ID
constexpr std::uint16_t kDmLbMasterId = 0x14;  ///< 左后关节主电机 CAN ID
constexpr std::uint16_t kDmLbSlaveId = 0x04;   ///< 左后关节从电机 CAN ID
constexpr std::uint16_t kDmRfMasterId = 0x15;  ///< 右前关节主电机 CAN ID
constexpr std::uint16_t kDmRfSlaveId = 0x05;   ///< 右前关节从电机 CAN ID
constexpr std::uint16_t kDmRbMasterId = 0x16;  ///< 右后关节主电机 CAN ID
constexpr std::uint16_t kDmRbSlaveId = 0x06;   ///< 右后关节从电机 CAN ID

const DmMitSettings kDmLfSettings{kDmLfMasterId, kDmLfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmLbSettings{kDmLbMasterId, kDmLbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmRfSettings{kDmRfMasterId, kDmRfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmRbSettings{kDmRbMasterId, kDmRbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};

constexpr std::uint16_t kSupercapRxStdId = 0x210;  ///< 超级电容反馈 CAN 标准 ID (wheel_can)
}  // namespace globals

// ── 云台公共（控制周期、力矩上限、重力补偿、电机配置）──
namespace gimbal {
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

constexpr float kDefaultDtS = 0.002f;      ///< 云台控制默认周期 [s]
constexpr float kDmTorqueLimitNm = 10.0f;  ///< DM 电机力矩上限 [Nm]

const DmMitSettings kPitchMotorSettings{0x11, 0x01, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
const DmMitSettings kYawMotorSettings{0x12, 0x02, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
}  // namespace gimbal

// ── 云台辨识公共常量 ──
namespace gimbal_ident_common {
constexpr size_t kHarmonicCount = 5;                   ///< 五次谐波
constexpr float kRpmToRadPerSec = kPi * 2.0f / 60.0f;  ///< rpm → rad/s
constexpr size_t kIdentUartTxBufSize = 128;            ///< 辨识串口发送缓冲区大小 [byte]
}  // namespace gimbal_ident_common

// ── 执行器公共 ──
namespace actuators {
constexpr float kWheelCurrentClampAbs = 16000.0f;  ///< 轮电机电流限幅绝对值
}  // namespace actuators

// ── 图传 CAN 桥通信 ──
namespace remote_control_can_bridge {
constexpr std::uint16_t kRxStdIdA = 0x110;  ///< CAN 帧 A 标准 ID（键鼠/云台 IMU 数据）
constexpr std::uint16_t kRxStdIdB = 0x111;  ///< CAN 帧 B 标准 ID
constexpr std::uint16_t kRxStdIdC = 0x112;  ///< CAN 帧 C 标准 ID（欧拉角）
constexpr std::size_t kPayloadSizeA = 8U;   ///< 帧 A 数据长度 [byte]
constexpr std::size_t kPayloadSizeB = 8U;   ///< 帧 B 数据长度 [byte]
constexpr std::size_t kPayloadSizeC = 8U;   ///< 帧 C 数据长度 [byte]
}  // namespace remote_control_can_bridge

}  // namespace common

// ══════════════════════════════════════════════════════════════════════════════
// Hero（变体 1）专用参数
// ══════════════════════════════════════════════════════════════════════════════
namespace hero {
using namespace common;

inline YawSpeedFeedforward yaw_ff{0.002f, 1.f};

// ── CAN 总线 ID 与电机配置 ──
namespace globals {
using namespace common::globals;

constexpr std::uint16_t kLeftWheelId = 0x05;   ///< 左轮毂电机 CAN ID
constexpr std::uint16_t kRightWheelId = 0x02;  ///< 右轮毂电机 CAN ID

constexpr std::uint16_t kDmLfMasterId = 0x14;  ///< 左前关节主电机 CAN ID
constexpr std::uint16_t kDmLfSlaveId = 0x04;   ///< 左前关节从电机 CAN ID
constexpr std::uint16_t kDmLbMasterId = 0x13;  ///< 左后关节主电机 CAN ID
constexpr std::uint16_t kDmLbSlaveId = 0x03;   ///< 左后关节从电机 CAN ID
constexpr std::uint16_t kDmRfMasterId = 0x15;  ///< 右前关节主电机 CAN ID
constexpr std::uint16_t kDmRfSlaveId = 0x05;   ///< 右前关节从电机 CAN ID
constexpr std::uint16_t kDmRbMasterId = 0x16;  ///< 右后关节主电机 CAN ID
constexpr std::uint16_t kDmRbSlaveId = 0x06;   ///< 右后关节从电机 CAN ID

const DmMitSettings kDmLfSettings{kDmLfMasterId, kDmLfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmLbSettings{kDmLbMasterId, kDmLbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmRfSettings{kDmRfMasterId, kDmRfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmRbSettings{kDmRbMasterId, kDmRbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
}  // namespace globals

// ── 云台 ──
namespace gimbal {
using namespace common::gimbal;

constexpr float kPitchMinRad = -0.35f;  ///< 俯仰角下限 [rad]
constexpr float kPitchMaxRad = 0.7f;    ///< 俯仰角上限 [rad]

constexpr PidGains kYawPositionPid{27.0f, 0.0f, 0.0f, 1000.0f, 1.0f};    ///< 偏航位置 PID
constexpr PidGains kYawSpeedPid{1.1f, 0.0f, 0.0f, 10.0f, 0.4f};          ///< 偏航速度 PID
constexpr PidGains kPitchPositionPid{25.0f, 0.0f, 0.0f, 1000.0f, 0.4f};  ///< 俯仰位置 PID
constexpr PidGains kPitchSpeedPid{2.f, 0.0f, 0.0f, 10.0f, 0.0f};         ///< 俯仰速度 PID

/// @brief 辨识得到的 9 个动力学参数（theta_0 ~ theta_8），用于前馈验证
constexpr float kIdentTheta[9] = {
    // 0.f, 0.f,  0.03675504, -0.26935183*2.5  ,  0.26007541*2.5,
    //         0.69615303,  0.51871814,  0.69915412,  0.0656076
    0, 0, 0, 0, 0, 0, 0, 0, 0};
}  // namespace gimbal

// ── 云台辨识 ──
namespace gimbal_ident {
using namespace common::gimbal_ident_common;
constexpr float kBaseFreqHz = 0.1f;                                                 ///< 辨识轨迹基频 [Hz]
constexpr float kDmTorqueLimitNm = 10.0f;                                           ///< DM 电机力矩上限 [Nm]
constexpr float kDefaultDtS = 0.002f;                                               ///< 辨识控制周期 [s]
constexpr float kYawAmp[kHarmonicCount] = {1.0f, -0.6f, 0.4f, -0.35f, 0.1f};        ///< yaw 轴五次谐波幅值 [rad]
constexpr float kPitchAmp[kHarmonicCount] = {0.27f, -0.14f, 0.09f, -0.05f, 0.03f};  ///< pitch 轴五次谐波幅值 [rad]
constexpr float kPitchPhase[kHarmonicCount] = {};                    ///< pitch 轴五次谐波相位 [rad]
constexpr PidGains kIdentYawPosPid{20.0f, 0.0f, 0.1f, 10.0f, 0.0f};  ///< 辨识模式 yaw 位置 PID（单位置环，高增益）
constexpr PidGains kIdentPitchPosPid{60.0f, 0.0f, 0.5f, 10.0f, 0.0f};  ///< 辨识模式 pitch 位置 PID（单位置环）
constexpr float kIdentPitchCenter = 1.f;  ///< 辨识轨迹 pitch 中心角 [rad]（机械中位，实际需根据云台标定）
constexpr float kIdentPitchTopLimit = 0.6f;     ///< 辨识轨迹 pitch 下限 [rad]
constexpr float kIdentPitchBottomLimit = 1.6f;  ///< 辨识轨迹 pitch 上限 [rad]
}  // namespace gimbal_ident

// ── 发射机构（Hero：三摩擦轮 + DM 拨盘）──
namespace shoot {
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

inline constexpr int kFrictionWheelCount = 3;            ///< 摩擦轮数量
inline constexpr float kBoosterZeroPointRad = 0.35f;     ///< 拨盘零位角度 [rad]
inline constexpr float kSegmentAngleRad = kPi / 3.f;     ///< 拨盘分段角度 [rad]
inline constexpr uint16_t kInitDelayTicks = 600;         ///< 初始化延迟周期数
inline constexpr uint16_t kShootDelayTicks = 360;        ///< 发射延迟周期数
inline constexpr float kStallThresholdRad = kPi / 18.f;  ///< 堵转判定角度阈值 [rad]
inline constexpr float kStallFallbackRad = kPi / 90.f;   ///< 堵转回退角度 [rad]
inline constexpr float kFwTargetSpeedRpm = 3650.0f;      ///< 摩擦轮目标转速 [rpm]  12m/s
// inline constexpr float kFwTargetSpeedRpm = 4690.0f;                       ///< 摩擦轮目标转速 [rpm]   16m/s
inline constexpr float kFricSpeedTargetRpm = kFwTargetSpeedRpm;  ///< alias for cross-variant compatibility
inline constexpr float kFwReadySpeedThresholdRpm = kFwTargetSpeedRpm - 600.f;  ///< 摩擦轮就绪判定转速 [rpm]
inline constexpr int16_t kFireDialThreshold = -100;                            ///< 发射触发拨轮阈值
inline constexpr PidGains kBoosterPositionPid{60.f, 0.f, 560.f, 24.f, 0.f};    ///< 拨盘位置 PID
inline constexpr PidGains kBoosterSpeedPid{0.3f, 0.f, 0.02f, 6.4f, 0.f};       ///< 拨盘速度 PID
inline constexpr PidGains kFwSpeedPid{30.f, 0.01f, 0.f, 10000.f, 0.f};         ///< 摩擦轮速度 PID
inline constexpr uint16_t kFwMotor1Id = 0x01;                                  ///< 摩擦轮电机 1 CAN ID
inline constexpr uint16_t kFwMotor2Id = 0x02;                                  ///< 摩擦轮电机 2 CAN ID
inline constexpr uint16_t kFwMotor3Id = 0x03;                                  ///< 摩擦轮电机 3 CAN ID
inline constexpr uint16_t kBoosterMasterId = 0x10;                             ///< 拨盘主电机 CAN ID
inline constexpr uint16_t kBoosterSlaveId = 0x09;                              ///< 拨盘从电机 CAN ID
inline const DmMitSettings kBoosterDmSettings{0x10, 0x09, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
inline constexpr float kFricSpeedStepRpm = 20.0f;  ///< Z/X 键每次调整摩擦轮转速步长 [rpm]
}  // namespace shoot

// ── 底盘状态机 ──
namespace chassis_fsm {

constexpr StairClimbParams kStairClimb{
    .high_leg_length_m = 0.30f,
    .hook_leg_length_m = 0.13f,
    .retract_leg_length_m = 0.13f,
    .settle_leg_length_m = 0.13f,
    .contact_theta_threshold_rad = 0.40f,
    .hook_theta_target_rad = 1.35f,
    .retract_theta_target_rad = 1.3f,
    .hook_theta_tolerance_rad = 0.20f,
    .leg_length_tolerance_m = 0.01f,
    .theta_dot_tolerance_rad_s = 0.50f,
    .settle_theta_tolerance_rad = 0.3f,
    .settle_theta_target_rad = 0.f,
    .settle_pitch_tolerance_rad = 0.20f,
    .settle_pitch_dot_tolerance_rad_s = 0.50f,
    .settle_roll_tolerance_rad = 0.20f,
    .hook_stable_ms = 180U,
    .retract_stable_ms = 180U,
    .settle_stable_ms = 1000U,
    .hook_timeout_ms = 1200U,
    .retract_timeout_ms = 1200U,
    .settle_timeout_ms = 2000U,
    .theta_pid = {15.0f, 0.0f, 5.0f, 15.0f, 0.0f},
};

// ==== 倒地自起 ====
constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;  ///< 倒地确认时间（持续倒地超过此值进入自启） [ms]
constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;  ///< 自启超时（超时后强制停机） [ms]

// ==== 跳跃（低腿长）====
constexpr std::uint32_t kJumpLowPrepMs = 100U;     ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpLowPushMaxMs = 500U;  ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpLowRecoverMs = 150U;  ///< 跳跃回收阶段持续时间（保底超时）[ms]
constexpr std::uint32_t kJumpLowRecoverMinMs = 60U;  ///< 跳跃回收阶段最低维持时间（此后开始判断离地）[ms]
constexpr float kJumpLowPrepLegLengthM = 0.13f;         ///< 跳跃预备阶段目标腿长 [m]（蓄力收腿）
constexpr float kJumpLowPushLegLengthM = 0.25f;         ///< 跳跃蹬伸阶段目标腿长 [m]（爆发推地）
constexpr float kJumpLowRecoverLegLengthM = 0.16f;      ///< 跳跃回收阶段目标腿长 [m]（落地缓冲）
constexpr float kJumpLowPushReachedLegLengthM = 0.26f;  ///< 蹬伸到位判定腿长 [m]（到达此值提前结束 push）

// ==== 自动跳跃（低腿长触发）====
constexpr std::uint32_t kJumpAutoPrepMs = 60U;       ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpAutoPushMaxMs = 1000U;  ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpAutoRecoverMs = 250U;   ///< 跳跃回收阶段持续时间（保底超时）[ms]
constexpr std::uint32_t kJumpAutoRecoverMinMs = 100U;  ///< 跳跃回收阶段最低维持时间（此后开始判断离地）[ms]
constexpr float kJumpAutoPrepLegLengthM = 0.18f;          ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpAutoPushLegLengthM = 0.29f;          ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpAutoRecoverLegLengthM = 0.18f;       ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpAutoPushReachedLegLengthM = 0.275f;  ///< 蹬伸到位判定腿长 [m]

// ==== 基本运动（腿长档位）====
constexpr float kLowLegLengthM = 0.13f;      ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.18f;      ///< 中腿长档位目标腿长 [m]
constexpr float kHighLegLengthM = 0.3f;      ///< 高腿长档位目标腿长 [m]
constexpr float kLegLengthRampTimeS = 0.3f;  ///< 腿长切换斜坡时间 [s]（从低到高腿长的过渡时间）
constexpr std::uint32_t kSpinExitTimeoutMs = 3000U;  ///< 小陀螺预测退出超时兜底 [ms]
}  // namespace chassis_fsm

// ── 底盘控制 ──
namespace chassis {

// ==== 物理/机械参数 ====
constexpr float kControlDtS = 0.002f;  ///< 底盘控制周期 [s]（500Hz）
constexpr float kLegL1M = 0.215f;      ///< 五连杆主动杆长度 [m]
constexpr float kLegL2M = 0.254f;      ///< 五连杆从动杆长度 [m]

// -- 左腿弹簧补偿三次多项式系数：tau = c0 + c1*l + c2*l^2 + c3*l^3 --
constexpr float kLeftSpringC0 = -120.005613f;
constexpr float kLeftSpringC1 = -129.143860f;
constexpr float kLeftSpringC2 = -915.501683f;
constexpr float kLeftSpringC3 = 2986.584000f;
// -- 右腿弹簧补偿三次多项式系数 --
constexpr float kRightSpringC0 = -120.005613f;
constexpr float kRightSpringC1 = -129.143860f;
constexpr float kRightSpringC2 = -915.501683f;
constexpr float kRightSpringC3 = 2986.584000f;

// -- 质量/惯量/重力 --
constexpr float kLegMassKg = 2.3f;     ///< 单条腿质量 [kg]
constexpr float kGravityMps2 = 9.81f;  ///< 重力加速度 [m/s²]

// -- 轮参数 --
constexpr float kWheelRadiusM = 0.2025f;  ///< 驱动轮轮距的一半（R_l） [m]

// -- 腿长→等效质心系数查找表（用于支撑力/重力前馈的 eta 插值）--
constexpr std::array<float, 24> kEtaLookupLegLengthM{
    0.11f, 0.12f, 0.13f, 0.14f, 0.15f, 0.16f, 0.17f, 0.18f, 0.19f, 0.20f, 0.21f, 0.22f,
    0.23f, 0.24f, 0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.30f, 0.31f, 0.32f, 0.33f, 0.34f,
};
constexpr std::array<float, 24> kEtaLookupLwM{
    0.061990f, 0.067466f, 0.072986f, 0.078550f, 0.084158f, 0.089810f, 0.095506f, 0.101246f,
    0.107030f, 0.112858f, 0.118730f, 0.124646f, 0.130606f, 0.136610f, 0.142658f, 0.148750f,
    0.154886f, 0.161066f, 0.167290f, 0.173558f, 0.179870f, 0.186226f, 0.192626f, 0.199070f,
};

// ==== 姿态安全/倒地恢复 ====
constexpr float kStandupThetaThresholdRad = 1.f;  ///< 起立完成判定：双腿摆角绝对值低于此值后允许轮端输出 [rad]
constexpr float kPostureRollMinRad = -0.3f;      ///< 横滚角安全下限 [rad]
constexpr float kPostureRollMaxRad = 0.3f;       ///< 横滚角安全上限 [rad]
constexpr float kPostureThetaBMinRad = -0.58f;   ///< 机体俯仰角安全下限 [rad]
constexpr float kPostureThetaBMaxRad = 0.5f;     ///< 机体俯仰角安全上限 [rad]
constexpr float kPostureThetaLegMinRad = -0.9f;  ///< 腿摆角安全下限 [rad]
constexpr float kPostureThetaLegMaxRad = 1.55f;  ///< 腿摆角安全上限 [rad]
constexpr float kLegRecoverThetaDotTarget = -1.2f;  ///< 倒地恢复时腿摆角速度目标 [rad/s]（负号表示前摆方向）
constexpr float kLegRecoverThetaDotRampStep = 0.06f;  ///< 倒地恢复腿摆角速度斜坡步长 [(rad/s)/周期]
constexpr float kManualRecoveryLegSpeedRadS = 1.2f;   ///< 手动倒地恢复腿摆角速度 [rad/s]
constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;   ///< 倒地恢复零力矩区间下限 [rad]
constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;   ///< 倒地恢复零力矩区间上限 [rad]

// ==== 倒地恢复软着陆 ====
constexpr float kRecoveryDecelZoneRad = 0.6f;   ///< 恢复减速区宽度 [rad]（接近目标边界时开始减速）
constexpr float kRecoveryMinSpeedRadS = 0.08f;  ///< 恢复减速区边界最低速度 [rad/s]
constexpr float kRecoveryGravityRampScale = 0.35f;  ///< 恢复时重力补偿斜坡比例（越大身体越不砸）

// -- 离地检测 --
constexpr float kOffGroundSupportForceThresholdN = 10.0f;  ///< 支撑力低于此值判定为离地 [N]
constexpr float kOffGroundSupportForceClampN = 100.0f;     ///< 离地时支持力限幅值 [N]

// -- 中腿长下压 --
constexpr float kMidLegDipTriggerLengthM = 0.2f;  ///< 中腿长模式下触发下压的腿长阈值 [m]
constexpr float kMidLegDipTargetLengthM = 0.22f;  ///< 下压目标腿长 [m]
constexpr uint16_t kMidLegDipHoldTicks = 500;     ///< 下压维持时间 [ticks @ 500Hz = 1s]

// ==== 物理参数（变体专属）====
constexpr float kBodyMassKg = 30.0f;  ///< 机体质量 [kg]

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.030f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵 — 40 组多项式系数）====
/// 由 MATLAB 离线拟合得到，p(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l² + p11*l_l*l_r + p02*l_r²
/// 共 40 行，对应 4×10 增益矩阵 K 的 40 个元素（按行主序展平）
/// 每行 6 个系数：[p00, p10, p01, p20, p11, p02]
constexpr std::array<float, 240> kCtrlPLow{
    -2.077,   -14.131, 11.443,   21.297,  -10.383,  -7.8401, -5.0628,  -23.25,   24.903,   40.387,   -29.641,  -15.484,
    -0.4412,  1.8788,  -0.67014, -1.9874, 0.16415,  1.0331,  -1.7261,  7.6074,   -2.9365,  -7.7444,  0.18224,  4.6515,
    -15.193,  -85.868, 11.417,   97.535,  0.12216,  -14.634, -0.81019, -5.2648,  2.106,    -2.9927,  1.7263,   -2.3199,
    -4.2728,  9.3124,  -6.2248,  -6.5059, 0.64759,  6.4436,  -0.3745,  -0.64145, -0.38372, 3.6045,   -7.9641,  1.9519,
    -25.487,  43.538,  29.98,    -11.603, -33.395,  -28.72,  -2.6853,  2.9307,   5.8224,   1.3014,   -6.3677,  -5.5361,
    -2.077,   11.443,  -14.131,  -7.8401, -10.383,  21.297,  -5.0628,  24.903,   -23.25,   -15.484,  -29.641,  40.387,
    0.4412,   0.67014, -1.8788,  -1.0331, -0.16415, 1.9874,  1.7261,   2.9365,   -7.6074,  -4.6515,  -0.18224, 7.7444,
    -4.2728,  -6.2248, 9.3124,   6.4436,  0.64759,  -6.5059, -0.3745,  -0.38372, -0.64145, 1.9519,   -7.9641,  3.6045,
    -15.193,  11.417,  -85.868,  -14.634, 0.12216,  97.535,  -0.81019, 2.106,    -5.2648,  -2.3199,  1.7263,   -2.9927,
    -25.487,  29.98,   43.538,   -28.72,  -33.395,  -11.603, -2.6853,  5.8224,   2.9307,   -5.5361,  -6.3677,  1.3014,
    4.2216,   1.4569,  -10.729,  -24.255, 24.483,   5.4547,  9.3534,   1.3102,   -28.514,  -47.756,  60.824,   14.451,
    -0.45338, -2.4258, -0.97686, 4.1071,  -0.94351, 1.4984,  -1.7756,  -9.94,    -3.583,   16.636,   -3.9304,  5.4498,
    63.261,   -127.77, 10.145,   119.84,  27.723,   -22.857, 2.626,    -1.9541,  -1.0778,  2.7713,   4.8044,   -0.54027,
    -3.7678,  -28.674, -6.5171,  37.623,  -9.9959,  -12.814, -0.20489, -0.91561, 2.3884,   -2.5422,  2.0083,   -6.4135,
    -44.333,  -205.55, 69.775,   256,     26.626,   -95.267, -2.2192,  -16.372,  4.418,    16.148,   7.7677,   -7.2413,
    4.2216,   -10.729, 1.4569,   5.4547,  24.483,   -24.255, 9.3534,   -28.514,  1.3102,   14.451,   60.824,   -47.756,
    0.45338,  0.97686, 2.4258,   -1.4984, 0.94351,  -4.1071, 1.7756,   3.583,    9.94,     -5.4498,  3.9304,   -16.636,
    -3.7678,  -6.5171, -28.674,  -12.814, -9.9959,  37.623,  -0.20489, 2.3884,   -0.91561, -6.4135,  2.0083,   -2.5422,
    63.261,   10.145,  -127.77,  -22.857, 27.723,   119.84,  2.626,    -1.0778,  -1.9541,  -0.54027, 4.8044,   2.7713,
    -44.333,  69.775,  -205.55,  -95.267, 26.626,   256,     -2.2192,  4.418,    -16.372,  -7.2413,  7.7677,   16.148,
};
constexpr std::array<float, 240> kCtrlPMid{
    -2.077,   -14.131, 11.443,   21.297,  -10.383,  -7.8401, -5.0628,  -23.25,   24.903,   40.387,   -29.641,  -15.484,
    -0.4412,  1.8788,  -0.67014, -1.9874, 0.16415,  1.0331,  -1.7261,  7.6074,   -2.9365,  -7.7444,  0.18224,  4.6515,
    -15.193,  -85.868, 11.417,   97.535,  0.12216,  -14.634, -0.81019, -5.2648,  2.106,    -2.9927,  1.7263,   -2.3199,
    -4.2728,  9.3124,  -6.2248,  -6.5059, 0.64759,  6.4436,  -0.3745,  -0.64145, -0.38372, 3.6045,   -7.9641,  1.9519,
    -25.487,  43.538,  29.98,    -11.603, -33.395,  -28.72,  -2.6853,  2.9307,   5.8224,   1.3014,   -6.3677,  -5.5361,
    -2.077,   11.443,  -14.131,  -7.8401, -10.383,  21.297,  -5.0628,  24.903,   -23.25,   -15.484,  -29.641,  40.387,
    0.4412,   0.67014, -1.8788,  -1.0331, -0.16415, 1.9874,  1.7261,   2.9365,   -7.6074,  -4.6515,  -0.18224, 7.7444,
    -4.2728,  -6.2248, 9.3124,   6.4436,  0.64759,  -6.5059, -0.3745,  -0.38372, -0.64145, 1.9519,   -7.9641,  3.6045,
    -15.193,  11.417,  -85.868,  -14.634, 0.12216,  97.535,  -0.81019, 2.106,    -5.2648,  -2.3199,  1.7263,   -2.9927,
    -25.487,  29.98,   43.538,   -28.72,  -33.395,  -11.603, -2.6853,  5.8224,   2.9307,   -5.5361,  -6.3677,  1.3014,
    4.2216,   1.4569,  -10.729,  -24.255, 24.483,   5.4547,  9.3534,   1.3102,   -28.514,  -47.756,  60.824,   14.451,
    -0.45338, -2.4258, -0.97686, 4.1071,  -0.94351, 1.4984,  -1.7756,  -9.94,    -3.583,   16.636,   -3.9304,  5.4498,
    63.261,   -127.77, 10.145,   119.84,  27.723,   -22.857, 2.626,    -1.9541,  -1.0778,  2.7713,   4.8044,   -0.54027,
    -3.7678,  -28.674, -6.5171,  37.623,  -9.9959,  -12.814, -0.20489, -0.91561, 2.3884,   -2.5422,  2.0083,   -6.4135,
    -44.333,  -205.55, 69.775,   256,     26.626,   -95.267, -2.2192,  -16.372,  4.418,    16.148,   7.7677,   -7.2413,
    4.2216,   -10.729, 1.4569,   5.4547,  24.483,   -24.255, 9.3534,   -28.514,  1.3102,   14.451,   60.824,   -47.756,
    0.45338,  0.97686, 2.4258,   -1.4984, 0.94351,  -4.1071, 1.7756,   3.583,    9.94,     -5.4498,  3.9304,   -16.636,
    -3.7678,  -6.5171, -28.674,  -12.814, -9.9959,  37.623,  -0.20489, 2.3884,   -0.91561, -6.4135,  2.0083,   -2.5422,
    63.261,   10.145,  -127.77,  -22.857, 27.723,   119.84,  2.626,    -1.0778,  -1.9541,  -0.54027, 4.8044,   2.7713,
    -44.333,  69.775,  -205.55,  -95.267, 26.626,   256,     -2.2192,  4.418,    -16.372,  -7.2413,  7.7677,   16.148,
};
constexpr std::array<float, 240> kCtrlPHigh{
    -2.077,   -14.131, 11.443,   21.297,  -10.383,  -7.8401, -5.0628,  -23.25,   24.903,   40.387,   -29.641,  -15.484,
    -0.4412,  1.8788,  -0.67014, -1.9874, 0.16415,  1.0331,  -1.7261,  7.6074,   -2.9365,  -7.7444,  0.18224,  4.6515,
    -15.193,  -85.868, 11.417,   97.535,  0.12216,  -14.634, -0.81019, -5.2648,  2.106,    -2.9927,  1.7263,   -2.3199,
    -4.2728,  9.3124,  -6.2248,  -6.5059, 0.64759,  6.4436,  -0.3745,  -0.64145, -0.38372, 3.6045,   -7.9641,  1.9519,
    -25.487,  43.538,  29.98,    -11.603, -33.395,  -28.72,  -2.6853,  2.9307,   5.8224,   1.3014,   -6.3677,  -5.5361,
    -2.077,   11.443,  -14.131,  -7.8401, -10.383,  21.297,  -5.0628,  24.903,   -23.25,   -15.484,  -29.641,  40.387,
    0.4412,   0.67014, -1.8788,  -1.0331, -0.16415, 1.9874,  1.7261,   2.9365,   -7.6074,  -4.6515,  -0.18224, 7.7444,
    -4.2728,  -6.2248, 9.3124,   6.4436,  0.64759,  -6.5059, -0.3745,  -0.38372, -0.64145, 1.9519,   -7.9641,  3.6045,
    -15.193,  11.417,  -85.868,  -14.634, 0.12216,  97.535,  -0.81019, 2.106,    -5.2648,  -2.3199,  1.7263,   -2.9927,
    -25.487,  29.98,   43.538,   -28.72,  -33.395,  -11.603, -2.6853,  5.8224,   2.9307,   -5.5361,  -6.3677,  1.3014,
    4.2216,   1.4569,  -10.729,  -24.255, 24.483,   5.4547,  9.3534,   1.3102,   -28.514,  -47.756,  60.824,   14.451,
    -0.45338, -2.4258, -0.97686, 4.1071,  -0.94351, 1.4984,  -1.7756,  -9.94,    -3.583,   16.636,   -3.9304,  5.4498,
    63.261,   -127.77, 10.145,   119.84,  27.723,   -22.857, 2.626,    -1.9541,  -1.0778,  2.7713,   4.8044,   -0.54027,
    -3.7678,  -28.674, -6.5171,  37.623,  -9.9959,  -12.814, -0.20489, -0.91561, 2.3884,   -2.5422,  2.0083,   -6.4135,
    -44.333,  -205.55, 69.775,   256,     26.626,   -95.267, -2.2192,  -16.372,  4.418,    16.148,   7.7677,   -7.2413,
    4.2216,   -10.729, 1.4569,   5.4547,  24.483,   -24.255, 9.3534,   -28.514,  1.3102,   14.451,   60.824,   -47.756,
    0.45338,  0.97686, 2.4258,   -1.4984, 0.94351,  -4.1071, 1.7756,   3.583,    9.94,     -5.4498,  3.9304,   -16.636,
    -3.7678,  -6.5171, -28.674,  -12.814, -9.9959,  37.623,  -0.20489, 2.3884,   -0.91561, -6.4135,  2.0083,   -2.5422,
    63.261,   10.145,  -127.77,  -22.857, 27.723,   119.84,  2.626,    -1.0778,  -1.9541,  -0.54027, 4.8044,   2.7713,
    -44.333,  69.775,  -205.55,  -95.267, 26.626,   256,     -2.2192,  4.418,    -16.372,  -7.2413,  7.7677,   16.148,
};
constexpr std::array<float, 240> kCtrlPSpin{
    -2.077,   -14.131, 11.443,   21.297,  -10.383,  -7.8401, -5.0628,  -23.25,   24.903,   40.387,   -29.641,  -15.484,
    -0.4412,  1.8788,  -0.67014, -1.9874, 0.16415,  1.0331,  -1.7261,  7.6074,   -2.9365,  -7.7444,  0.18224,  4.6515,
    -15.193,  -85.868, 11.417,   97.535,  0.12216,  -14.634, -0.81019, -5.2648,  2.106,    -2.9927,  1.7263,   -2.3199,
    -4.2728,  9.3124,  -6.2248,  -6.5059, 0.64759,  6.4436,  -0.3745,  -0.64145, -0.38372, 3.6045,   -7.9641,  1.9519,
    -25.487,  43.538,  29.98,    -11.603, -33.395,  -28.72,  -2.6853,  2.9307,   5.8224,   1.3014,   -6.3677,  -5.5361,
    -2.077,   11.443,  -14.131,  -7.8401, -10.383,  21.297,  -5.0628,  24.903,   -23.25,   -15.484,  -29.641,  40.387,
    0.4412,   0.67014, -1.8788,  -1.0331, -0.16415, 1.9874,  1.7261,   2.9365,   -7.6074,  -4.6515,  -0.18224, 7.7444,
    -4.2728,  -6.2248, 9.3124,   6.4436,  0.64759,  -6.5059, -0.3745,  -0.38372, -0.64145, 1.9519,   -7.9641,  3.6045,
    -15.193,  11.417,  -85.868,  -14.634, 0.12216,  97.535,  -0.81019, 2.106,    -5.2648,  -2.3199,  1.7263,   -2.9927,
    -25.487,  29.98,   43.538,   -28.72,  -33.395,  -11.603, -2.6853,  5.8224,   2.9307,   -5.5361,  -6.3677,  1.3014,
    4.2216,   1.4569,  -10.729,  -24.255, 24.483,   5.4547,  9.3534,   1.3102,   -28.514,  -47.756,  60.824,   14.451,
    -0.45338, -2.4258, -0.97686, 4.1071,  -0.94351, 1.4984,  -1.7756,  -9.94,    -3.583,   16.636,   -3.9304,  5.4498,
    63.261,   -127.77, 10.145,   119.84,  27.723,   -22.857, 2.626,    -1.9541,  -1.0778,  2.7713,   4.8044,   -0.54027,
    -3.7678,  -28.674, -6.5171,  37.623,  -9.9959,  -12.814, -0.20489, -0.91561, 2.3884,   -2.5422,  2.0083,   -6.4135,
    -44.333,  -205.55, 69.775,   256,     26.626,   -95.267, -2.2192,  -16.372,  4.418,    16.148,   7.7677,   -7.2413,
    4.2216,   -10.729, 1.4569,   5.4547,  24.483,   -24.255, 9.3534,   -28.514,  1.3102,   14.451,   60.824,   -47.756,
    0.45338,  0.97686, 2.4258,   -1.4984, 0.94351,  -4.1071, 1.7756,   3.583,    9.94,     -5.4498,  3.9304,   -16.636,
    -3.7678,  -6.5171, -28.674,  -12.814, -9.9959,  37.623,  -0.20489, 2.3884,   -0.91561, -6.4135,  2.0083,   -2.5422,
    63.261,   10.145,  -127.77,  -22.857, 27.723,   119.84,  2.626,    -1.0778,  -1.9541,  -0.54027, 4.8044,   2.7713,
    -44.333,  69.775,  -205.55,  -95.267, 26.626,   256,     -2.2192,  4.418,    -16.372,  -7.2413,  7.7677,   16.148,
};

// ==== 基本运动（PID 增益）====
constexpr PidGains kLeftL0Pid{2500.0f, 0.f, 200.0f, 130.0f, 0.0f};   ///< 左腿腿长 PID（常规）
constexpr PidGains kRightL0Pid{2500.0f, 0.f, 200.0f, 130.0f, 0.0f};  ///< 右腿腿长 PID（常规）
constexpr PidGains kRollPid{1000.0f, 0.1f, 20.0f, 100.0f, 40.0f};    ///< 横滚平衡 PID

// ==== 跳跃（PID 增益）====
constexpr float kJumpPushForceN = 250.0f;                                    ///< 蹬伸阶段单腿基础支撑力 [N]
constexpr PidGains kLeftL0PidJumpTwo{4000.0f, 0.0f, 60.0f, 120.0f, 0.0f};    ///< 左腿蹬伸 PID（JumpPush）
constexpr PidGains kRightL0PidJumpTwo{4000.0f, 0.0f, 60.0f, 120.0f, 0.0f};   ///< 右腿蹬伸 PID（JumpPush）
constexpr PidGains kLeftL0PidJumpThree{4000.0f, 0.f, 60.0f, 120.0f, 0.0f};   ///< 左腿回收 PID（JumpRecover）
constexpr PidGains kRightL0PidJumpThree{4000.0f, 0.f, 60.0f, 120.0f, 0.0f};  ///< 右腿回收 PID（JumpRecover）

// ==== 中腿长下压（PID 增益）====
constexpr PidGains kLeftL0PidDip{2500.0f, 0.f, 500.0f, 180.0f, 0.0f};   ///< 左腿下压腿长 PID
constexpr PidGains kRightL0PidDip{2500.0f, 0.f, 500.0f, 180.0f, 0.0f};  ///< 右腿下压腿长 PID

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{30.0f, 0.0f, 60.0f, 22.0f, 0.0f};   ///< 左腿摆角速度 PID（倒地恢复用）
constexpr PidGains kRightLegTurnPid{30.0f, 0.0f, 60.0f, 22.0f, 0.0f};  ///< 右腿摆角速度 PID（倒地恢复用）
constexpr PidGains kLeftLegTurnPidManual{30.0f, 0.0f, 60.0f, 22.0f, 0.0f};  ///< 左腿摆角速度 PID（手动倒地恢复）
constexpr PidGains kRightLegTurnPidManual{30.0f, 0.0f, 60.0f, 22.0f, 0.0f};  ///< 右腿摆角速度 PID（手动倒地恢复）

// ==== l0_ddot 低通滤波 ====
constexpr float kL0DdotFilterCutoffHz = 5.0f;    ///< l0_ddot 低通滤波截止频率 [Hz]
constexpr float kL0DdotFilterSampleHz = 500.0f;  ///< l0_ddot 低通滤波采样频率 [Hz]
}  // namespace chassis

// ── 控制环 ──
namespace control_loop {

/// @brief 纵向速度斜坡参数（加速/制动步长分离）
struct SdotRampParams {
  float accel_step;  ///< 加速步长 [(m/s)/周期]
  float brake_step;  ///< 制动步长 [(m/s)/周期]
};

// -- 小陀螺/跳跃触发阈值（DR16 拨轮值）--
constexpr std::int16_t kWheelSpinThreshold = 220;    ///< 拨轮超过此值触发小陀螺保持
constexpr std::int16_t kWheelActionThreshold = 320;  ///< 拨轮回中后快速负推超过此值触发跳跃
constexpr std::int16_t kWheelCenterThreshold = 80;  ///< 拨轮归中阈值（绝对值小于此值视为归中，重新就绪）
constexpr std::uint16_t kAutoJumpDistanceThresholdMm = 500U;  ///< 自动跳跃 DYP 测距阈值 [mm]
constexpr float kAutoJumpHoldTimeS = 1.0f;                    ///< 自动跳跃开关拨轮保持时间 [s]
constexpr float kAutoJumpDistanceHoldTimeS = 0.05f;           ///< 自动跳跃测距持续低于阈值判定时间 [s]

constexpr float kControlLoopDtS = 0.002f;  ///< 控制环周期 [s]

// -- 摇杆/键鼠输入归一化 --
constexpr std::int16_t kDr16AxisMaxAbs = 660;       ///< DR16 摇杆轴最大绝对值（用于归一化到 [-1,1]）
constexpr float kRcStickMax = 660.0f;               ///< RC 摇杆最大值（用于积分目标速率计算）
constexpr float kTcMouseMax = 200.0f;               ///< 图传鼠标增量最大值（用于积分目标速率计算）
constexpr float kRcYawRateMaxRadS = -2.5f;          ///< RC 摇杆满偏时偏航积分速率 [rad/s]
constexpr float kRcPitchRateMaxRadS = 1.5f;         ///< RC 摇杆满偏时俯仰积分速率 [rad/s]
constexpr float kTcMouseYawRateMaxRadS = -2.0f;     ///< 图传鼠标满偏时偏航积分速率 [rad/s]
constexpr float kTcMousePitchRateMaxRadS = 1.0f;    ///< 图传鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kDr16MouseMax = 1600.0f;            ///< DR16 鼠标增量最大值（用于积分目标速率计算）
constexpr float kDr16MouseYawRateMaxRadS = -2.0f;   ///< DR16 鼠标满偏时偏航积分速率 [rad/s]
constexpr float kDr16MousePitchRateMaxRadS = 1.0f;  ///< DR16 鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kPitchTargetMinRad = -0.35f;        ///< RC 积分俯仰目标下限 [rad]
constexpr float kPitchTargetMaxRad = 0.7f;          ///< RC 积分俯仰目标上限 [rad]
constexpr float kKeyboardAccelRampStep = 0.004f;    ///< 键盘 WASD 加速斜坡步进（每周期，0→1 约 0.5s）
constexpr float kKeyboardBrakeRampStep = 0.008f;    ///< 键盘 WASD 减速斜坡步进（每周期，1→0 约 0.25s）

// -- 云台启动归中判稳 --
constexpr float kGimbalStartupYawAlignErrorRad = 0.04f;           ///< 归中完成位置误差阈值 [rad]
constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;            ///< 归中完成速度阈值 [rad/s]
constexpr std::uint32_t kGimbalStartupYawAlignStableTicks = 50U;  ///< 归中判稳所需连续满足周期数

// -- 偏航就绪判稳（允许底盘纵向驱动前确认偏航跟踪到位）--
constexpr float kYawFollowDriveReadyErrorRad = 0.04f;           ///< 偏航就绪位置误差阈值 [rad]
constexpr float kYawFollowDriveReadyVelRadS = 0.25f;            ///< 偏航就绪速度阈值 [rad/s]
constexpr std::uint32_t kYawFollowDriveReadyStableTicks = 50U;  ///< 偏航就绪判稳所需连续周期数

// ==== 基本运动 ====
constexpr float kMaxSafeYawRateRadS = 3.5f;               ///< 摩擦圆最大安全偏航速率 [rad/s]
constexpr float kTargetForwardSpeedMaxMps = 2.15f;        ///< 最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.3f;  ///< 高腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxMidLegMps = 1.0f;   ///< F键中腿长模式最大前进速度 [m/s]
constexpr float kTargetSpeedBiasLowLegMps = 0.0f;         ///< 低腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegMps = 0.0f;         ///< C键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegFMps = 0.0f;        ///< F键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasHighLegMps = 0.0f;        ///< 高腿长目标速度偏置 [m/s]
constexpr float kVxInputDeadbandNorm = 0.1f;              ///< 前进输入死区（归一化值，低于此忽略）
constexpr float kVyInputDeadbandNorm = 0.1f;              ///< 平移输入死区（归一化值）
constexpr float kYawFollowRampStepRadS = 0.1f;            ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.15f;  ///< 位置锚定冻结速度阈值 [m/s]（车速低于此值时锁定位置）

// ==== 落地减速（离地→落地时通过腿摆角辅助减速）====
constexpr float kLandingDecelThetaGain = 0.f;                ///< 落地减速腿摆角增益 [rad/(m/s)]
constexpr float kLandingDecelThetaMaxRad = 0.3f;             ///< 落地减速腿摆角最大偏置 [rad]
constexpr float kLandingDecelThetaRampStepRad = 0.01f;       ///< 落地减速腿摆角每周期斜坡步长 [rad/tick]
constexpr std::uint32_t kLandingDecelOffGroundMinMs = 100U;  ///< 离地最短持续时间（防单帧误判）[ms]
constexpr std::uint32_t kLandingDecelStableDurationMs = 400U;  ///< 落地减速稳定保持时间 [ms]

constexpr float kYawFollowFixedTargetRad = -1.9f;      ///< 偏航跟随固定目标偏置角 [rad]（前进方向）
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;  ///< 偏航跟随侧向目标偏置角 [rad]（±π/2）
constexpr PidGains kYawFollowPid{10.0f, 0.0f, 2.f, 4.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置（腿摆角/机体俯仰）====
constexpr float kExpectedThetaLlBiasRadLowLeg = -0.13f;   ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = -0.13;    ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLeg = -0.11f;   ///< 中腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLeg = -0.11f;   ///< 中腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadHighLeg = -0.08f;  ///< 高腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadHighLeg = -0.08f;  ///< 高腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = -0.168f;         ///< 期望机体俯仰偏置 [rad]

// ==== 速度斜坡参数（按腿长档位分级）====
constexpr SdotRampParams kSdotRampLowLeg{0.007f, 0.007f};   ///< 低腿长速度斜坡（加速/制动步长）
constexpr SdotRampParams kSdotRampMidLeg{0.005f, 0.005f};   ///< 中腿长速度斜坡(C 键触发)
constexpr SdotRampParams kSdotRampMidLegF{0.006f, 0.006f};  ///< 中腿长速度斜坡（F 键触发）
constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};  ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.1f;            ///< 小陀螺进入偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinExitYawRampStepRadS = 0.15f;       ///< 小陀螺退出偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS = 7.5f;           ///< 小陀螺目标自旋角速度 [rad/s]
constexpr float kSpinExitYawAlignThresholdRad = 0.15f;  ///< 小陀螺预测退出：yaw 对齐阈值 [rad]
constexpr float kSpinTranslationGain = 1.0f;  ///< 小陀螺平移增益（将云台系前进指令投影到车体系的比例）
constexpr float kSpinThetaLlBiasRad = -0.13f;  ///< 小陀螺时左腿摆角偏置 [rad]
constexpr float kSpinThetaLrBiasRad = -0.14f;  ///< 小陀螺时右腿摆角偏置 [rad]
constexpr float kSpinLegLengthBiasM = -0.0f;   ///< 小陀螺时腿长偏差（左+右-）[m]
constexpr float kSpinThetaBBiasRad = -0.168f;  ///< 小陀螺时俯仰目标偏置 [rad]

// ==== 跳跃腿摆角偏置 ====
constexpr float kJumpThetaLlBiasRad = 0.0f;  ///< 跳跃时左腿摆角偏置 [rad]
constexpr float kJumpThetaLrBiasRad = 0.0f;  ///< 跳跃时右腿摆角偏置 [rad]
}  // namespace control_loop

// ── 执行器 ──
namespace actuators {
using namespace common::actuators;

constexpr float kLeftWheelTorqueToCurrent = 2436.0f;   ///< 左轮力矩→电流转换系数
constexpr float kRightWheelTorqueToCurrent = 2436.0f;  ///< 右轮力矩→电流转换系数
}  // namespace actuators

// ── 状态估计（关节零位标定 + 滤波器/卡尔曼参数）──
namespace state_estimator {
constexpr float kDefaultDtS = 0.002f;                   ///< 估计器默认周期 [s]
constexpr float kDefaultExpectedSdotMps = 0.05f;        ///< 默认期望纵向速度 [m/s]
constexpr float kLegL1M = 0.215f;                       ///< 五连杆主动杆长度 [m]
constexpr float kLegL2M = 0.254f;                       ///< 五连杆从动杆长度 [m]
constexpr float kWheelRadiusM = 0.0575f;                ///< 驱动轮半径 [m]
constexpr float kWheelReductionRatio = 17.0f / 268.0f;  ///< 轮电机到车轮的速度换算比
constexpr float kMaxValidSpeedMps = 8.0f;               ///< 速度融合可信上限 [m/s]
constexpr float kThetaDotFilterCutoffHz = 8.0f;         ///< 腿摆角速度低通滤波截止频率 [Hz]

// -- IMU 加速度融合 --
constexpr float kImuAccelFilterSampleHz = 500.0f;          ///< 加速度低通滤波器采样频率 [Hz]
constexpr float kImuAccelFilterCutoffHz = 10.0f;           ///< 加速度低通滤波器截止频率 [Hz]
constexpr std::uint32_t kAccelBiasInitSamples = 1500U;     ///< 加速度零偏估计所需初始样本数
constexpr float kAccelZeroWheelSpeedThresholdMps = 0.02f;  ///< 轮速零速判定阈值 [m/s]
constexpr float kAccelZeroHighThresholdMps2 = 0.5f;        ///< 加速度零偏校准上阈值 [m/s²]
constexpr float kAccelZeroLowThresholdMps2 = 0.2f;         ///< 加速度零偏校准下阈值 [m/s²]

// -- 卡尔曼滤波器矩阵（2 状态：速度、加速度）--
constexpr float kKalmanMinVariance = 1e-5f;
constexpr float kThetaPiHalf = 1.57079632679489661923f;
constexpr std::array<float, 4> kKalmanF{1.0f, kDefaultDtS, 0.0f, 1.0f};  ///< 状态转移矩阵 F
constexpr std::array<float, 4> kKalmanQ{0.0005f, 0.0f, 0.0f, 0.04f};     ///< 过程噪声协方差 Q
constexpr std::array<float, 4> kKalmanR{0.5f, 0.0f, 0.0f, 2.0f};         ///< 观测噪声协方差 R
constexpr std::array<float, 4> kKalmanP{10.0f, 0.0f, 0.0f, 10.0f};       ///< 初始估计协方差 P
constexpr std::array<float, 4> kKalmanH{1.0f, 0.0f, 0.0f, 1.0f};         ///< 观测矩阵 H

// -- 关节零位偏移 --
constexpr float kLeftPhi1OffsetRad = 2.8f + M_PI;                       ///< 左腿前关节零位偏移 [rad]
constexpr float kLeftPhi4OffsetRad = 0.4f;                              ///< 左腿后关节零位偏移 [rad]
constexpr float kRightPhi1OffsetRad = 2.1244f + M_PI + 0.13f - 0.068f;  ///< 右腿前关节零位偏移 [rad]
constexpr float kRightPhi4OffsetRad = 0.46f + 0.123f + 0.136;           ///< 右腿后关节零位偏移 [rad]
}  // namespace state_estimator

// ── 腿部运动学 ──
namespace leg_kinematics {
constexpr float kDefaultDtS = 0.002f;           ///< 运动学默认解算周期 [s]
constexpr float kMinSin = 1e-5f;                ///< 正弦值下限（避免除零奇异）
constexpr float kMinLen = 1e-5f;                ///< 长度下限（避免除零奇异）
constexpr float kL0DotFilterCutoffHz = 10.0f;   ///< l0_dot 低通滤波截止频率 [Hz]
constexpr float kL0DotFilterSampleHz = 500.0f;  ///< l0_dot 低通滤波采样频率 [Hz]
}  // namespace leg_kinematics

namespace remote_control_can_bridge {
using namespace common::remote_control_can_bridge;
}
namespace main {
using namespace common::main;
}

// ── 自瞄通信 ──
namespace aimbot {
constexpr uint8_t kRobotId = 1U;                                     ///< 机器人 ID（裁判系统回退值）
constexpr float kBulletSpeedMps = 11.5f;                             ///< 弹速 [m/s]（裁判系统回退值）
constexpr float kBulletDefaultSpeedMps = 11.5f;                      ///< 默认弹速
constexpr float kBulletBoundarySpeedMps = 10.5f;                     ///< 区分裁判系统返回值是否正确
constexpr PidGains kYawPositionPid{80.0f, 0.2, 3.f, 1000.0f, 4.0f};  ///< 自瞄偏航位置 PID
constexpr PidGains kYawSpeedPid{0.7f, 0.0f, 0.0f, 10.0f, 0.3f};      ///< 自瞄偏航速度 PID
constexpr PidGains kPitchPositionPid{80, 0.2f, 3.0f, 1000.0f, 4.f};  ///< 自瞄俯仰位置 PID3
constexpr PidGains kPitchSpeedPid{0.7f, 0.0f, 0.0f, 9.0f, 0.4f};     ///< 自瞄俯仰速度 PID
}  // namespace aimbot

}  // namespace hero

// ══════════════════════════════════════════════════════════════════════════════
// Infantry3（变体 2，默认）专用参数
// ══════════════════════════════════════════════════════════════════════════════
namespace infantry3 {
using namespace common;

// ── CAN 总线 ID 与电机配置 ──
namespace globals {
using namespace common::globals;

constexpr std::uint16_t kLeftWheelId = 0x02;   ///< 左轮毂电机 CAN ID
constexpr std::uint16_t kRightWheelId = 0x01;  ///< 右轮毂电机 CAN ID

constexpr std::uint16_t kFricLeftId = 0x01;   ///< 左摩擦轮电机 CAN ID
constexpr std::uint16_t kFricRightId = 0x04;  ///< 右摩擦轮电机 CAN ID
constexpr std::uint16_t kDialId = 0x03;       ///< 拨盘电机 CAN ID
}  // namespace globals

// ── 云台 ──
namespace gimbal {
using namespace common::gimbal;

constexpr float kPitchMinRad = -0.32f;  ///< 俯仰角下限 [rad]
constexpr float kPitchMaxRad = 0.6f;    ///< 俯仰角上限 [rad]

inline constexpr PidGains kYawPositionPid{32.0f, 1.f, 1.f, 10.0f, 1.0f};    ///< 偏航位置 PID
inline constexpr PidGains kYawSpeedPid{0.6f, 0.0f, 0.0f, 6.0f, 0.4f};       ///< 偏航速度 PID
inline constexpr PidGains kPitchPositionPid{26.0f, 0.f, 1.f, 10.0f, 0.6f};  ///< 俯仰位置 PID
inline constexpr PidGains kPitchSpeedPid{0.6f, 0.0f, 0.0008f, 8.0f, 0.0f};  ///< 俯仰速度 PID

/// @brief 辨识得到的 9 个动力学参数（theta_0 ~ theta_8），用于前馈验证
constexpr float kIdentTheta[9] = {
    0.012257f,  // theta_0: I1zz_com
    0.032210f,  // theta_1: I2xx_com
    0.043163f,  // theta_2: I2yy_com
    0.097069f,  // theta_3: m2*l2x 水平前向偏心
    0.114014f,  // theta_4: m2*l2z 垂直上向偏心
    0.259783f,  // theta_5: fv1  yaw 粘滞摩擦
    0.226098f,  // theta_6: fc1  yaw 库仑摩擦
    0.788615f,  // theta_7: fv2  pitch 粘滞摩擦
    0.118902f,  // theta_8: fc2  pitch 库仑摩擦
};
}  // namespace gimbal

// ── 云台辨识 ──
namespace gimbal_ident {
using namespace common::gimbal_ident_common;
constexpr float kBaseFreqHz = 0.1f;                                                 ///< 辨识轨迹基频 [Hz]
constexpr float kDmTorqueLimitNm = 10.0f;                                           ///< DM 电机力矩上限 [Nm]
constexpr float kDefaultDtS = 0.002f;                                               ///< 辨识控制周期 [s]
constexpr float kYawAmp[kHarmonicCount] = {1.0f, -0.6f, 0.4f, -0.35f, 0.1f};        ///< yaw 轴五次谐波幅值 [rad]
constexpr float kPitchAmp[kHarmonicCount] = {0.27f, -0.14f, 0.09f, -0.05f, 0.03f};  ///< pitch 轴五次谐波幅值 [rad]
constexpr float kPitchPhase[kHarmonicCount] = {1.2177f, 0.4006f, -0.8970f, 0.1462f, -2.6391f};
constexpr PidGains kIdentYawPosPid{20.0f, 0.0f, 0.1f, 10.0f, 0.0f};  ///< 辨识模式 yaw 位置 PID（单位置环，高增益）
constexpr PidGains kIdentPitchPosPid{60.0f, 0.0f, 0.5f, 10.0f, 0.0f};  ///< 辨识模式 pitch 位置 PID（单位置环）
constexpr float kIdentPitchCenter = -2.286921;  ///< 辨识轨迹 pitch 中心角 [rad]（机械中位，实际需根据云台标定）
constexpr float kIdentPitchTopLimit = -2.6f;     ///< 辨识轨迹 pitch 下限 [rad]
constexpr float kIdentPitchBottomLimit = -1.6f;  ///< 辨识轨迹 pitch 上限 [rad
}  // namespace gimbal_ident

// ── 发射机构（双摩擦轮 + M3508 拨盘）──
namespace shoot {
inline constexpr int kFrictionWheelCount = 2;                            ///< 摩擦轮数量
constexpr float kFricSpeedTargetRpm = 6000.0f;                           ///< 摩擦轮目标转速 [rpm]
constexpr PidGains kFricSpeedPid{10.0f, 0.0f, 0.0f, 16000.0f, 2000.0f};  ///< 摩擦轮速度 PID
constexpr PidGains kDialSpeedPid{10.0f, 0.f, 0.f, 16000.0f, 1000.0f};    ///< 拨盘速度 PID
constexpr PidGains kDialPositionPid{0.5f, 0.f, 0.0f, 16000.0f, 0.0f};    ///< 拨盘位置 PID
constexpr int16_t kDialFireThreshold = -600;                             ///< 发射触发拨轮阈值
constexpr float kFricSpeedStepRpm = 20.0f;  ///< Z/X 键每次调整摩擦轮转速步长 [rpm]

// ── 本地热量闭环 ──
constexpr float kHeatPerShot = 10.0f;  ///< 每发子弹热量增量 [热量单位]
constexpr float kHeatSafetyMargin = 45.0f;  ///< 高热量模式停火余量：heat + kHeatPerShot > limit - margin 时抑制发射
constexpr float kLowHeatSafetyMargin = 20.0f;  ///< 低热量模式停火余量
constexpr float kHeatResumeMargin = 20.0f;  ///< 恢复余量：heat < limit - margin 时恢复，与停火线构成迟滞
constexpr uint16_t kDefaultHeatLimit = 240;       ///< 裁判系统离线时默认热量上限
constexpr uint16_t kDefaultCoolingRate = 40;      ///< 裁判系统离线时默认冷却速率 [热量单位/秒]
constexpr uint16_t kLowHeatLimitThreshold = 100;  ///< 低热量上限阈值：低于此值时降频并使用本地热量
constexpr float kLowHeatShootFrequencyHz = 6.0f;  ///< 低热量上限时发射频率 [Hz]
constexpr float kNormalShootFrequencyHz = 10.0f;  ///< 正常发射频率 [Hz]
}  // namespace shoot

// ── 底盘状态机 ──
namespace chassis_fsm {

constexpr StairClimbParams kStairClimb{
    .high_leg_length_m = 0.33f,
    .hook_leg_length_m = 0.3f,
    .retract_leg_length_m = 0.1f,
    .settle_leg_length_m = 0.15f,
    .contact_theta_threshold_rad = 0.50f,
    .hook_theta_target_rad = 0.7f,
    .retract_theta_target_rad = -0.2f,
    .hook_theta_tolerance_rad = 0.1f,
    .leg_length_tolerance_m = 0.05f,
    .theta_dot_tolerance_rad_s = 0.50f,
    .settle_theta_tolerance_rad = 0.3f,
    .settle_theta_target_rad = 0.f,
    .settle_pitch_tolerance_rad = 0.18f,
    .settle_pitch_dot_tolerance_rad_s = 0.50f,
    .settle_roll_tolerance_rad = 0.25f,
    .hook_stable_ms = 80U,
    .retract_stable_ms = 50U,
    .settle_stable_ms = 500U,
    .hook_timeout_ms = 1000U,
    .retract_timeout_ms = 500U,
    .settle_timeout_ms = 3000U,
    .theta_pid = {40.0f, 0.0f, 8.0f, 60.0f, 30.0f},
};

// ==== 倒地自启 ====
constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;        ///< 倒地确认时间 [ms]
constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;  ///< 自启超时 [ms]

// ==== 跳跃 ====
constexpr std::uint32_t kJumpLowPrepMs = 150U;     ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpLowPushMaxMs = 350U;  ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpLowRecoverMs = 800U;  ///< 跳跃回收阶段持续时间（保底超时）[ms]
constexpr std::uint32_t kJumpLowRecoverMinMs = 450U;  ///< 跳跃回收阶段最低维持时间（此后开始判断离地）[ms]
constexpr float kJumpLowPrepLegLengthM = 0.135f;         ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpLowPushLegLengthM = 0.35f;          ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpLowRecoverLegLengthM = 0.13f;       ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpLowPushReachedLegLengthM = 0.335f;  ///< 蹬伸到位判定腿长 [m]

// ==== 自动跳跃（低腿长触发）====
constexpr std::uint32_t kJumpAutoPrepMs = 0U;        ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpAutoPushMaxMs = 1000U;  ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpAutoRecoverMs = 1200U;  ///< 跳跃回收阶段持续时间（保底超时）[ms]
constexpr std::uint32_t kJumpAutoRecoverMinMs = 900U;  ///< 跳跃回收阶段最低维持时间（此后开始判断离地）[ms]
constexpr float kJumpAutoPrepLegLengthM = 0.16f;         ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpAutoPushLegLengthM = 0.25f;         ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpAutoRecoverLegLengthM = 0.18f;      ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpAutoPushReachedLegLengthM = 0.26f;  ///< 蹬伸到位判定腿长 [m]

// ==== 基本运动（腿长档位）====
constexpr float kLowLegLengthM = 0.16f;              ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.23f;              ///< 中腿长档位目标腿长 [m]
constexpr float kHighLegLengthM = 0.33f;             ///< 高腿长档位目标腿长 [m]
constexpr float kLegLengthRampTimeS = 0.4f;          ///< 腿长切换斜坡时间 [s]
constexpr std::uint32_t kSpinExitTimeoutMs = 3000U;  ///< 小陀螺预测退出超时兜底 [ms]
}  // namespace chassis_fsm

// ── 底盘控制 ──
namespace chassis {

// ==== 物理/机械参数 ====
constexpr float kControlDtS = 0.002f;  ///< 底盘控制周期 [s]（500Hz）
constexpr float kLegL1M = 0.215f;      ///< 五连杆主动杆长度 [m]
constexpr float kLegL2M = 0.254f;      ///< 五连杆从动杆长度 [m]

// -- 左腿弹簧补偿三次多项式系数：tau = c0 + c1*l + c2*l^2 + c3*l^3 --
constexpr float kLeftSpringC0 = 75.269401f;
constexpr float kLeftSpringC1 = -1872.833463f;
constexpr float kLeftSpringC2 = 4779.020146f;
constexpr float kLeftSpringC3 = -3726.603083f;
// -- 右腿弹簧补偿三次多项式系数 --
constexpr float kRightSpringC0 = -128.58f;
constexpr float kRightSpringC1 = 941.47f;
constexpr float kRightSpringC2 = -7421.14f;
constexpr float kRightSpringC3 = 13247.60f;

// -- 质量/惯量/重力 --
constexpr float kLegMassKg = 2.3f;     ///< 单条腿质量 [kg]
constexpr float kGravityMps2 = 9.81f;  ///< 重力加速度 [m/s²]

// -- 轮参数 --
constexpr float kWheelRadiusM = 0.2025f;  ///< 驱动轮轮距的一半（R_l） [m]

// -- 腿长→等效质心系数查找表 --
constexpr std::array<float, 24> kEtaLookupLegLengthM{
    0.11f, 0.12f, 0.13f, 0.14f, 0.15f, 0.16f, 0.17f, 0.18f, 0.19f, 0.20f, 0.21f, 0.22f,
    0.23f, 0.24f, 0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.30f, 0.31f, 0.32f, 0.33f, 0.34f,
};
constexpr std::array<float, 24> kEtaLookupLwM{
    0.061990f, 0.067466f, 0.072986f, 0.078550f, 0.084158f, 0.089810f, 0.095506f, 0.101246f,
    0.107030f, 0.112858f, 0.118730f, 0.124646f, 0.130606f, 0.136610f, 0.142658f, 0.148750f,
    0.154886f, 0.161066f, 0.167290f, 0.173558f, 0.179870f, 0.186226f, 0.192626f, 0.199070f,
};

// ==== 姿态安全/倒地恢复 ====
constexpr float kStandupThetaThresholdRad = 0.95f;  ///< 起立完成判定：双腿摆角绝对值低于此值后允许轮端输出 [rad]
constexpr float kPostureRollMinRad = -1.f;             ///< 横滚角安全下限 [rad]
constexpr float kPostureRollMaxRad = 1.f;              ///< 横滚角安全上限 [rad]
constexpr float kPostureThetaBMinRad = -0.7f;          ///< 机体俯仰角安全下限 [rad]
constexpr float kPostureThetaBMaxRad = 0.7f;           ///< 机体俯仰角安全上限 [rad]
constexpr float kPostureThetaLegMinRad = -0.8f;        ///< 腿摆角安全下限 [rad]
constexpr float kPostureThetaLegMaxRad = 2.f;          ///< 腿摆角安全上限 [rad]
constexpr float kLegRecoverThetaDotTarget = -2.0f;     ///< 倒地恢复时腿摆角速度目标 [rad/s]
constexpr float kLegRecoverThetaDotRampStep = 0.008f;  ///< 倒地恢复腿摆角速度斜坡步长 [(rad/s)/周期]
constexpr float kManualRecoveryLegSpeedRadS = 0.5f;    ///< 手动倒地恢复腿摆角速度 [rad/s]
constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;    ///< 倒地恢复零力矩区间下限 [rad]
constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;    ///< 倒地恢复零力矩区间上限 [rad]
                                                       // ==== 倒地恢复软着陆 ====
constexpr float kRecoveryDecelZoneRad = 0.6f;   ///< 恢复减速区宽度 [rad]（接近目标边界时开始减速）
constexpr float kRecoveryMinSpeedRadS = 0.08f;  ///< 恢复减速区边界最低速度 [rad/s]
constexpr float kRecoveryGravityRampScale = 0.35f;  ///< 恢复时重力补偿斜坡比例（越大身体越不砸）

// -- 离地检测 --
constexpr float kOffGroundSupportForceThresholdN = 20.0f;  ///< 支撑力低于此值判定为离地 [N]
constexpr float kOffGroundSupportForceClampN = 100.0f;     ///< 离地时支持力限幅值 [N]

// -- 中腿长下压 --
constexpr float kMidLegDipTriggerLengthM = 0.27f;  ///< 中腿长模式下触发下压的腿长阈值 [m]
constexpr float kMidLegDipTargetLengthM = 0.22f;   ///< 下压目标腿长 [m]
constexpr uint16_t kMidLegDipHoldTicks = 1000;     ///< 下压维持时间 [ticks @ 500Hz = 1s]

// ==== 物理参数（变体专属）====

constexpr float kBodyMassKg = 22.0f;  ///< 机体质量 [kg]

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.0126f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵）====
constexpr std::array<float, 240> kCtrlPLow{
    -3.9952,   -31.099,  25.547,   51.296,    -27.317,  -17.356,  -6.6029, -32.803, 35.452,   60.233,   -45.273,
    -22.325,   -0.47235, 2.4464,   -0.86416,  -2.2606,  -0.35469, 1.5934,  -1.0881, 6.0101,   -2.4597,  -5.1594,
    -1.4438,   4.5897,   -15.015,  -81.829,   12.951,   87.621,   3.075,   -16.739, -0.90686, -6.2509,  2.7337,
    -3.0308,   2.0331,   -2.9589,  -3.8652,   5.5339,   -2.8657,  3.385,   -16.37,  9.9954,   -0.37861, -1.3563,
    -0.079043, 5.9454,   -11.978,  3.0551,    -24.574,  45.992,   27.82,   -19.759, -29.667,  -28.07,   -2.8223,
    2.9411,    6.4948,   1.6592,   -7.0005,   -6.3468,  -3.9952,  25.547,  -31.099, -17.356,  -27.317,  51.296,
    -6.6029,   35.452,   -32.803,  -22.325,   -45.273,  60.233,   0.47235, 0.86416, -2.4464,  -1.5934,  0.35469,
    2.2606,    1.0881,   2.4597,   -6.0101,   -4.5897,  1.4438,   5.1594,  -3.8652, -2.8657,  5.5339,   9.9954,
    -16.37,    3.385,    -0.37861, -0.079043, -1.3563,  3.0551,   -11.978, 5.9454,  -15.015,  12.951,   -81.829,
    -16.739,   3.075,    87.621,   -0.90686,  2.7337,   -6.2509,  -2.9589, 2.0331,  -3.0308,  -24.574,  27.82,
    45.992,    -28.07,   -29.667,  -19.759,   -2.8223,  6.4948,   2.9411,  -6.3468, -7.0005,  1.6592,   7.4357,
    -8.669,    -8.9088,  -33.739,  43.56,     2.8614,   10.796,   -8.7452, -24.317, -47.28,   70.339,   12.596,
    -0.52696,  -2.7732,  -0.70884, 4.7871,    -0.92834, 1.356,    -1.2067, -7.0428, -1.404,   12.035,   -2.7615,
    2.8427,    50.985,   -104.21,  4.7856,    88.179,   27.509,   -15.358, 2.4642,  -1.6297,  -1.4535,  -1.6732,
    7.5313,    -0.20855, -2.9125,  -22.511,   -1.3031,  24.447,   -5.1564, -4.0222, -0.10099, -0.79204, 2.5626,
    -3.2352,   0.27145,  -2.8521,  -41.282,   -173.12,  56.484,   222.08,  25.636,  -84.274,  -2.0236,  -15.818,
    3.8083,    16.004,   8.2727,   -7.2298,   7.4357,   -8.9088,  -8.669,  2.8614,  43.56,    -33.739,  10.796,
    -24.317,   -8.7452,  12.596,   70.339,    -47.28,   0.52696,  0.70884, 2.7732,  -1.356,   0.92834,  -4.7871,
    1.2067,    1.404,    7.0428,   -2.8427,   2.7615,   -12.035,  -2.9125, -1.3031, -22.511,  -4.0222,  -5.1564,
    24.447,    -0.10099, 2.5626,   -0.79204,  -2.8521,  0.27145,  -3.2352, 50.985,  4.7856,   -104.21,  -15.358,
    27.509,    88.179,   2.4642,   -1.4535,   -1.6297,  -0.20855, 7.5313,  -1.6732, -41.282,  56.484,   -173.12,
    -84.274,   25.636,   222.08,   -2.0236,   3.8083,   -15.818,  -7.2298, 8.2727,  16.004,
};
constexpr std::array<float, 240> kCtrlPMid{
    -2.9257,  -25.003, 19.175,   37.658,  -18.038,  -13.151, -4.6374,   -24.601,  24.972,   41.489,  -29.615,  -15.874,
    -0.36294, 1.5961,  -0.74022, -1.3823, -0.14071, 1.215,   -1.1703,   5.3446,   -2.6442,  -4.4075, -0.81422, 4.4144,
    -9.1809,  -73.172, 14.051,   72.752,  3.9622,   -18.311, -0.68399,  -5.2774,  2.1759,   -2.2017, 1.9567,   -2.4763,
    -3.4174,  4.0412,  -7.7198,  3.8001,  -8.1853,  13.402,  -0.33506,  -0.90715, -0.20099, 4.0848,  -8.353,   2.0955,
    -19.747,  22.252,  29.355,   12.938,  -31.023,  -27.397, -2.3731,   1.5913,   5.8258,   2.9079,  -6.0988,  -5.5366,
    -2.9257,  19.175,  -25.003,  -13.151, -18.038,  37.658,  -4.6374,   24.972,   -24.601,  -15.874, -29.615,  41.489,
    0.36294,  0.74022, -1.5961,  -1.215,  0.14071,  1.3823,  1.1703,    2.6442,   -5.3446,  -4.4144, 0.81422,  4.4075,
    -3.4174,  -7.7198, 4.0412,   13.402,  -8.1853,  3.8001,  -0.33506,  -0.20099, -0.90715, 2.0955,  -8.353,   4.0848,
    -9.1809,  14.051,  -73.172,  -18.311, 3.9622,   72.752,  -0.68399,  2.1759,   -5.2774,  -2.4763, 1.9567,   -2.2017,
    -19.747,  29.355,  22.252,   -27.397, -31.023,  12.938,  -2.3731,   5.8258,   1.5913,   -5.5366, -6.0988,  2.9079,
    9.7578,   3.9625,  -23.274,  -60.856, 56.76,    12.467,  13.415,    2.9426,   -40.294,  -73.145, 87.507,   21.568,
    -0.35744, -3.0372, -0.69665, 5.0425,  -1.3582,  1.187,   -1.1514,   -10.324,  -1.9539,  16.965,  -4.6965,  3.3454,
    54.977,   -72.403, 3.2821,   24.085,  44.439,   -17.679, 3.1589,    -0.32123, -2.2097,  -2.6026, 7.0841,   0.26991,
    -0.76531, -32.727, -9.6703,  34.254,  -11.987,  -3.1237, -0.005155, -0.68482, 2.67,     -4.8235, 2.6131,   -5.8471,
    -31.715,  -199.12, 55.092,   225.22,  44.456,   -79.224, -1.1981,   -17.394,  2.3281,   14.645,  11.502,   -5.4909,
    9.7578,   -23.274, 3.9625,   12.467,  56.76,    -60.856, 13.415,    -40.294,  2.9426,   21.568,  87.507,   -73.145,
    0.35744,  0.69665, 3.0372,   -1.187,  1.3582,   -5.0425, 1.1514,    1.9539,   10.324,   -3.3454, 4.6965,   -16.965,
    -0.76531, -9.6703, -32.727,  -3.1237, -11.987,  34.254,  -0.005155, 2.67,     -0.68482, -5.8471, 2.6131,   -4.8235,
    54.977,   3.2821,  -72.403,  -17.679, 44.439,   24.085,  3.1589,    -2.2097,  -0.32123, 0.26991, 7.0841,   -2.6026,
    -31.715,  55.092,  -199.12,  -79.224, 44.456,   225.22,  -1.1981,   2.3281,   -17.394,  -5.4909, 11.502,   14.645,
};
constexpr std::array<float, 240> kCtrlPHigh{
    -2.9088,   -26.536,   20.618,   41.637,    -21.641,  -13.449,  -4.6247,   -25.876,  26.184,   44.744,   -32.681,
    -16.023,   -0.36288,  1.7221,   -0.81614,  -1.3476,  -0.37319, 1.4385,    -0.84321, 4.2575,   -2.2356,  -3.0528,
    -1.3592,   4.0076,    -9.2344,  -74.128,   13.801,   74.917,   3.9448,    -17.884,  -0.6893,  -5.365,   2.2159,
    -2.0344,   1.8632,    -2.448,   -3.3539,   3.2159,   -5.7336,  6.1906,    -13.427,  13.69,    -0.32916, -1.0318,
    -0.029402, 4.5226,    -9.1603,  2.3561,    -19.74,   22.208,   29.342,    13.123,   -30.288,  -28.212,  -2.3713,
    1.5221,    5.8832,    3.0652,   -6.1125,   -5.6625,  -2.9088,  20.618,    -26.536,  -13.449,  -21.641,  41.637,
    -4.6247,   26.184,    -25.876,  -16.023,   -32.681,  44.744,   0.36288,   0.81614,  -1.7221,  -1.4385,  0.37319,
    1.3476,    0.84321,   2.2356,   -4.2575,   -4.0076,  1.3592,   3.0528,    -3.3539,  -5.7336,  3.2159,   13.69,
    -13.427,   6.1906,    -0.32916, -0.029402, -1.0318,  2.3561,   -9.1603,   4.5226,   -9.2344,  13.801,   -74.128,
    -17.884,   3.9448,    74.917,   -0.6893,   2.2159,   -5.365,   -2.448,    1.8632,   -2.0344,  -19.74,   29.342,
    22.208,    -28.212,   -30.288,  13.123,    -2.3713,  5.8832,   1.5221,    -5.6625,  -6.1125,  3.0652,   9.7382,
    -0.038923, -19.157,   -59.858,  58.528,    9.5701,   13.399,   -0.059169, -37.199,  -73.328,  88.912,   20.24,
    -0.36681,  -3.2941,   -0.57194, 5.4756,    -1.4385,  1.1441,   -0.8483,   -8.3606,  -0.95759, 13.726,   -3.8571,
    2.1023,    55.042,    -75.912,  3.0821,    20.271,   42.114,   -16.973,   3.145,    -0.36421, -2.1901,  -4.4225,
    8.1009,    0.22766,   -0.8461,  -31.536,   -7.0615,  30.839,   -7.6355,   1.2811,   0.006438, -0.62219, 2.6426,
    -5.0993,   1.9895,    -4.1107,  -31.716,   -200.17,  56.164,   229.6,     44.556,   -83.731,  -1.1994,  -17.725,
    2.6674,    15.382,    11.523,   -6.263,    9.7382,   -19.157,  -0.038923, 9.5701,   58.528,   -59.858,  13.399,
    -37.199,   -0.059169, 20.24,    88.912,    -73.328,  0.36681,  0.57194,   3.2941,   -1.1441,  1.4385,   -5.4756,
    0.8483,    0.95759,   8.3606,   -2.1023,   3.8571,   -13.726,  -0.8461,   -7.0615,  -31.536,  1.2811,   -7.6355,
    30.839,    0.006438,  2.6426,   -0.62219,  -4.1107,  1.9895,   -5.0993,   55.042,   3.0821,   -75.912,  -16.973,
    42.114,    20.271,    3.145,    -2.1901,   -0.36421, 0.22766,  8.1009,    -4.4225,  -31.716,  56.164,   -200.17,
    -83.731,   44.556,    229.6,    -1.1994,   2.6674,   -17.725,  -6.263,    11.523,   15.382,
};
constexpr std::array<float, 240> kCtrlPSpin{
    -5.1906,  -36.378, 31.017,   61.269,  -35.38,   -19.374, -9.4285,  -41.275,  47.815,   79.662,   -65.688, -27.106,
    -0.53132, 2.9081,  -0.97554, -2.6194, -0.83052, 1.924,   -1.2161,  7.2641,   -2.9523,  -5.9349,  -3.0331, 5.8494,
    -25.844,  -97.21,  12.496,   111.39,  4.4898,   -15.838, -1.256,   -7.5963,  3.3795,   -4.0625,  2.4827,  -3.2803,
    -4.6304,  8.7047,  -1.0585,  -1.3044, -21.967,  10.389,  -0.46218, -1.6762,  -0.27312, 7.7432,   -16.302, 4.8017,
    -36.074,  87.448,  29.631,   -68.389, -32.434,  -30.787, -3.5547,  4.7246,   7.7279,   0.21458,  -8.7587, -7.3558,
    -5.1906,  31.017,  -36.378,  -19.374, -35.38,   61.269,  -9.4285,  47.815,   -41.275,  -27.106,  -65.688, 79.662,
    0.53132,  0.97554, -2.9081,  -1.924,  0.83052,  2.6194,  1.2161,   2.9523,   -7.2641,  -5.8494,  3.0331,  5.9349,
    -4.6304,  -1.0585, 8.7047,   10.389,  -21.967,  -1.3044, -0.46218, -0.27312, -1.6762,  4.8017,   -16.302, 7.7432,
    -25.844,  12.496,  -97.21,   -15.838, 4.4898,   111.39,  -1.256,   3.3795,   -7.5963,  -3.2803,  2.4827,  -4.0625,
    -36.074,  29.631,  87.448,   -30.787, -32.434,  -68.389, -3.5547,  7.7279,   4.7246,   -7.3558,  -8.7587, 0.21458,
    6.453,    -12.838, -4.0542,  -23.368, 39.54,    -1.708,  10.428,   -12.812,  -21.83,   -40.078,  70.628,  9.5317,
    -0.64424, -2.3065, -0.5581,  4.0099,  -0.80288, 1.1877,  -1.4733,  -5.9733,  -1.014,   10.253,   -2.5436, 2.411,
    60.476,   -155.71, 4.3724,   165.17,  19.948,   -13.325, 2.4625,   -2.9155,  -1.4028,  0.047878, 7.6689,  -0.31153,
    -4.9629,  -16.996, 4.1695,   19.106,  -0.76016, -14.109, -0.15576, -0.82053, 2.5954,   -2.9099,  0.2714,  -2.9618,
    -63.566,  -195.29, 68.259,   267.97,  18.596,   -103,    -2.9136,  -15.804,  4.3916,   17.07,    7.6639,  -8.1898,
    6.453,    -4.0542, -12.838,  -1.708,  39.54,    -23.368, 10.428,   -21.83,   -12.812,  9.5317,   70.628,  -40.078,
    0.64424,  0.5581,  2.3065,   -1.1877, 0.80288,  -4.0099, 1.4733,   1.014,    5.9733,   -2.411,   2.5436,  -10.253,
    -4.9629,  4.1695,  -16.996,  -14.109, -0.76016, 19.106,  -0.15576, 2.5954,   -0.82053, -2.9618,  0.2714,  -2.9099,
    60.476,   4.3724,  -155.71,  -13.325, 19.948,   165.17,  2.4625,   -1.4028,  -2.9155,  -0.31153, 7.6689,  0.047878,
    -63.566,  68.259,  -195.29,  -103,    18.596,   267.97,  -2.9136,  4.3916,   -15.804,  -8.1898,  7.6639,  17.07,
};

// ==== 基本运动（PID 增益）====
constexpr PidGains kLeftL0Pid{3000.0f, 0.f, 180.0f, 170.0f, 10.0f};   ///< 左腿腿长 PID（常规）
constexpr PidGains kRightL0Pid{3000.0f, 0.f, 180.0f, 170.0f, 10.0f};  ///< 右腿腿长 PID（常规）
constexpr PidGains kRollPid{800.0f, 0.0f, 200.0f, 180.0f, 0.0f};      ///< 横滚平衡 PID

// ==== 跳跃（PID 增益）====
constexpr float kJumpPushForceN = 120.0f;                                     ///< 蹬伸阶段单腿基础支撑力 [N]
constexpr PidGains kLeftL0PidJumpTwo{3800.0f, 0.0f, 200.0f, 250.0f, 0.0f};    ///< 左腿蹬伸 PID
constexpr PidGains kRightL0PidJumpTwo{3800.0f, 0.0f, 200.0f, 250.0f, 0.0f};   ///< 右腿蹬伸 PID
constexpr PidGains kLeftL0PidJumpThree{1800.0f, 0.f, 100.0f, 200.0f, 30.0f};  ///< 左腿回收 PID
constexpr PidGains kRightL0PidJumpThree{1800.0f, 0.f, 100.0f, 200.0f, 30.0f};  ///< 右腿回收 PID

// ==== 中腿长下压（PID 增益）====
constexpr PidGains kLeftL0PidDip{450.0f, 0.f, 200.0f, 170.0f, 10.0f};   ///< 左腿下压腿长 PID
constexpr PidGains kRightL0PidDip{450.0f, 0.f, 200.0f, 170.0f, 10.0f};  ///< 右腿下压腿长 PID

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{18.0f, 1.0f, 10.0f, 25.0f, 5.0f};   ///< 左腿摆角速度 PID
constexpr PidGains kRightLegTurnPid{18.0f, 1.0f, 10.0f, 25.0f, 5.0f};  ///< 右腿摆角速度 PID
constexpr PidGains kLeftLegTurnPidManual{10.0f, 0.0f, 2.0f, 20.0f, 0.0f};  ///< 左腿摆角速度 PID（手动倒地恢复）
constexpr PidGains kRightLegTurnPidManual{10.0f, 0.0f, 2.0f, 20.0f, 0.0f};  ///< 右腿摆角速度 PID（手动倒地恢复）

// ==== l0_ddot 低通滤波 ====
constexpr float kL0DdotFilterCutoffHz = 5.0f;    ///< l0_ddot 低通滤波截止频率 [Hz]
constexpr float kL0DdotFilterSampleHz = 500.0f;  ///< l0_ddot 低通滤波采样频率 [Hz]
}  // namespace chassis

// ── 控制环 ──
namespace control_loop {

/// @brief 纵向速度斜坡参数（加速/制动步长分离）
struct SdotRampParams {
  float accel_step;  ///< 加速步长 [(m/s)/周期]
  float brake_step;  ///< 制动步长 [(m/s)/周期]
};

// -- 小陀螺/跳跃触发阈值 --
constexpr std::int16_t kWheelSpinThreshold = 220;             ///< 拨轮超过此值触发小陀螺保持
constexpr std::int16_t kWheelActionThreshold = 320;           ///< 拨轮回中后快速负推超过此值触发跳跃
constexpr std::int16_t kWheelCenterThreshold = 80;            ///< 拨轮归中阈值
constexpr std::uint16_t kAutoJumpDistanceThresholdMm = 600U;  ///< 自动跳跃 DYP 测距阈值 [mm]
constexpr float kAutoJumpHoldTimeS = 1.0f;                    ///< 自动跳跃开关拨轮保持时间 [s]
constexpr float kAutoJumpDistanceHoldTimeS = 0.2f;            ///< 自动跳跃测距持续低于阈值判定时间 [s]

constexpr float kControlLoopDtS = 0.002f;  ///< 控制环周期 [s]

// -- 摇杆/键鼠输入归一化 --
constexpr std::int16_t kDr16AxisMaxAbs = 660;       ///< DR16 摇杆轴最大绝对值
constexpr float kRcStickMax = 660.0f;               ///< RC 摇杆最大值
constexpr float kTcMouseMax = 200.0f;               ///< 图传鼠标增量最大值
constexpr float kRcYawRateMaxRadS = -5.5f;          ///< RC 摇杆满偏时偏航积分速率 [rad/s]
constexpr float kRcPitchRateMaxRadS = 5.5f;         ///< RC 摇杆满偏时俯仰积分速率 [rad/s]
constexpr float kTcMouseYawRateMaxRadS = -2.0f;     ///< 图传鼠标满偏时偏航积分速率 [rad/s]
constexpr float kTcMousePitchRateMaxRadS = 1.0f;    ///< 图传鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kDr16MouseMax = 1600.0f;            ///< DR16 鼠标增量最大值（用于积分目标速率计算）
constexpr float kDr16MouseYawRateMaxRadS = -2.0f;   ///< DR16 鼠标满偏时偏航积分速率 [rad/s]
constexpr float kDr16MousePitchRateMaxRadS = 1.0f;  ///< DR16 鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kPitchTargetMinRad = -0.3f;         ///< RC 积分俯仰目标下限 [rad]
constexpr float kPitchTargetMaxRad = 0.65f;         ///< RC 积分俯仰目标上限 [rad]
constexpr float kKeyboardAccelRampStep = 0.004f;    ///< 键盘 WASD 加速斜坡步进（每周期，0→1 约 0.5s）
constexpr float kKeyboardBrakeRampStep = 0.1f;      ///< 键盘 WASD 减速斜坡步进（每周期，1→0 约 0.25s）

// -- 云台启动归中判稳 --
constexpr float kGimbalStartupYawAlignErrorRad = 0.04f;           ///< 归中完成位置误差阈值 [rad]
constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;            ///< 归中完成速度阈值 [rad/s]
constexpr std::uint32_t kGimbalStartupYawAlignStableTicks = 10U;  ///< 归中判稳所需连续满足周期数

// -- 偏航就绪判稳 --
constexpr float kYawFollowDriveReadyErrorRad = 0.04f;           ///< 偏航就绪位置误差阈值 [rad]
constexpr float kYawFollowDriveReadyVelRadS = 0.25f;            ///< 偏航就绪速度阈值 [rad/s]
constexpr std::uint32_t kYawFollowDriveReadyStableTicks = 50U;  ///< 偏航就绪判稳所需连续周期数

// ==== 基本运动 ====
constexpr float kTargetForwardSpeedMaxMps = 2.22f;        ///< 最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.1f;  ///< 高腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxMidLegMps = 2.0f;   ///< F键中腿长模式最大前进速度 [m/s]
constexpr float kTargetSpeedBiasLowLegMps = 0.0f;         ///< 低腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegMps = 0.0f;         ///< C键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegFMps = 0.0f;        ///< F键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasHighLegMps = 0.0f;        ///< 高腿长目标速度偏置 [m/s]
constexpr float kMaxSafeYawRateRadS = 4.5f;               ///< 摩擦圆最大安全偏航速率 [rad/s]
constexpr float kVxInputDeadbandNorm = 0.05f;             ///< 前进输入死区
constexpr float kVyInputDeadbandNorm = 0.05f;             ///< 平移输入死区
constexpr float kYawFollowRampStepRadS = 0.11f;           ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.3f;  ///< 位置锚定冻结速度阈值 [m/s]

// ==== 落地减速（离地→落地时通过腿摆角辅助减速）====
constexpr float kLandingDecelThetaGain = -0.2f;               ///< 落地减速腿摆角增益 [rad/(m/s)]
constexpr float kLandingDecelThetaMaxRad = 0.3f;              ///< 落地减速腿摆角最大偏置 [rad]
constexpr float kLandingDecelThetaRampStepRad = 0.008f;       ///< 落地减速腿摆角每周期斜坡步长 [rad/tick]
constexpr std::uint32_t kLandingDecelOffGroundMinMs = 40U;    ///< 离地最短持续时间（防单帧误判）[ms]
constexpr std::uint32_t kLandingDecelStableDurationMs = 50U;  ///< 落地减速稳定保持时间 [ms]

constexpr float kYawFollowFixedTargetRad = 0.38f;                ///< 偏航跟随固定目标偏置角 [rad]
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;            ///< 偏航跟随侧向目标偏置角 [rad]
constexpr PidGains kYawFollowPid{26.0f, 0.0f, 3.f, 8.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置 ====
constexpr float kExpectedThetaLlBiasRadLowLeg = 0.085f;  ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = 0.085f;  ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLeg = 0.05f;   ///< 中腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLeg = 0.05f;   ///< 中腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadHighLeg = 0.06f;  ///< 高腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadHighLeg = 0.06f;  ///< 高腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = -0.005f;        ///< 期望机体俯仰偏置 [rad]

// ==== 速度斜坡参数 ====
constexpr SdotRampParams kSdotRampLowLeg{0.0065f, 0.0065f};  ///< 低腿长速度斜坡
constexpr SdotRampParams kSdotRampMidLeg{0.0035f, 0.007f};   ///< 中腿长速度斜坡(C 键触发)
constexpr SdotRampParams kSdotRampMidLegF{0.0045f, 0.006f};  ///< 中腿长速度斜坡（F 键触发）
constexpr SdotRampParams kSdotRampHighLeg{0.005f, 0.005f};   ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.05f;           ///< 小陀螺进入偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinExitYawRampStepRadS = 0.05f;       ///< 小陀螺退出偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS = 8.5f;           ///< 小陀螺目标自旋角速度 [rad/s]
constexpr float kSpinExitYawAlignThresholdRad = 0.15f;  ///< 小陀螺预测退出：yaw 对齐阈值 [rad]
constexpr float kSpinTranslationGain = 0.4f;  ///< 小陀螺平移增益（系数2补偿 cos² 平均衰减，使平均车速=摇杆指令值）
constexpr float kSpinThetaLlBiasRad = 0.08f;  ///< 小陀螺时左腿摆角偏置 [rad]
constexpr float kSpinThetaLrBiasRad = 0.1f;   ///< 小陀螺时右腿摆角偏置 [rad]
constexpr float kSpinLegLengthBiasM = 0.0f;   ///< 小陀螺时腿长偏差（左+右-）[m]
constexpr float kSpinThetaBBiasRad = 0.05f;   ///< 小陀螺时俯仰目标偏置 [rad]

// ==== 跳跃腿摆角偏置 ====
constexpr float kJumpThetaLlBiasRad = 0.04f;  ///< 跳跃时左腿摆角偏置 [rad]
constexpr float kJumpThetaLrBiasRad = 0.04f;  ///< 跳跃时右腿摆角偏置 [rad]
}  // namespace control_loop

// ── 执行器 ──
namespace actuators {
using namespace common::actuators;

constexpr float kLeftWheelTorqueToCurrent = 2430.0f;   ///< 左轮力矩→电流转换系数
constexpr float kRightWheelTorqueToCurrent = 2430.0f;  ///< 右轮力矩→电流转换系数
}  // namespace actuators

// ── 状态估计 ──
namespace state_estimator {
constexpr float kDefaultDtS = 0.002f;                   ///< 估计器默认周期 [s]
constexpr float kDefaultExpectedSdotMps = 0.05f;        ///< 默认期望纵向速度 [m/s]
constexpr float kLegL1M = 0.215f;                       ///< 五连杆主动杆长度 [m]
constexpr float kLegL2M = 0.254f;                       ///< 五连杆从动杆长度 [m]
constexpr float kWheelRadiusM = 0.0575f;                ///< 驱动轮半径 [m]
constexpr float kWheelReductionRatio = 17.0f / 268.0f;  ///< 轮电机到车轮的速度换算比
constexpr float kMaxValidSpeedMps = 8.0f;               ///< 速度融合可信上限 [m/s]
constexpr float kThetaDotFilterCutoffHz = 8.0f;         ///< 腿摆角速度低通滤波截止频率 [Hz]

// -- IMU 加速度融合 --
constexpr float kImuAccelFilterSampleHz = 500.0f;          ///< 加速度低通滤波器采样频率 [Hz]
constexpr float kImuAccelFilterCutoffHz = 10.0f;           ///< 加速度低通滤波器截止频率 [Hz]
constexpr std::uint32_t kAccelBiasInitSamples = 1500U;     ///< 加速度零偏估计所需初始样本数
constexpr float kAccelZeroWheelSpeedThresholdMps = 0.02f;  ///< 轮速零速判定阈值 [m/s]
constexpr float kAccelZeroHighThresholdMps2 = 0.5f;        ///< 加速度零偏校准上阈值 [m/s²]
constexpr float kAccelZeroLowThresholdMps2 = 0.2f;         ///< 加速度零偏校准下阈值 [m/s²]

// -- 卡尔曼滤波器矩阵 --
constexpr float kKalmanMinVariance = 1e-5f;
constexpr float kThetaPiHalf = 1.57079632679489661923f;
constexpr std::array<float, 4> kKalmanF{1.0f, kDefaultDtS, 0.0f, 1.0f};  ///< 状态转移矩阵 F
constexpr std::array<float, 4> kKalmanQ{0.0005f, 0.0f, 0.0f, 0.04f};     ///< 过程噪声协方差 Q
constexpr std::array<float, 4> kKalmanR{0.5f, 0.0f, 0.0f, 2.0f};         ///< 观测噪声协方差 R
constexpr std::array<float, 4> kKalmanP{10.0f, 0.0f, 0.0f, 10.0f};       ///< 初始估计协方差 P
constexpr std::array<float, 4> kKalmanH{1.0f, 0.0f, 0.0f, 1.0f};         ///< 观测矩阵 H

// -- 关节零位偏移 --
constexpr float kLeftPhi1OffsetRad = 1.38f - 1.8383f + M_PI;            ///< 左腿前关节零位偏移 [rad]
constexpr float kLeftPhi4OffsetRad = 0.86f + 0.7531f;                   ///< 左腿后关节零位偏移 [rad]
constexpr float kRightPhi1OffsetRad = 1.26f + 0.06f - 2.16641f + M_PI;  ///< 右腿前关节零位偏移 [rad]
constexpr float kRightPhi4OffsetRad = 0.65f;                            ///< 右腿后关节零位偏移 [rad]
}  // namespace state_estimator

// ── 腿部运动学 ──
namespace leg_kinematics {
constexpr float kDefaultDtS = 0.002f;           ///< 运动学默认解算周期 [s]
constexpr float kMinSin = 1e-5f;                ///< 正弦值下限（避免除零奇异）
constexpr float kMinLen = 1e-5f;                ///< 长度下限（避免除零奇异）
constexpr float kL0DotFilterCutoffHz = 10.0f;   ///< l0_dot 低通滤波截止频率 [Hz]
constexpr float kL0DotFilterSampleHz = 500.0f;  ///< l0_dot 低通滤波采样频率 [Hz]
}  // namespace leg_kinematics

namespace remote_control_can_bridge {
using namespace common::remote_control_can_bridge;
}
namespace main {
using namespace common::main;
}

// ── 自瞄通信 ──
namespace aimbot {
constexpr uint8_t kRobotId = 3U;                                     ///< 机器人 ID
constexpr float kBulletSpeedMps = 23.0f;                             ///< 弹速 [m/s]
constexpr float kBulletDefaultSpeedMps = 23.f;                       ///< 默认弹速
constexpr float kBulletBoundarySpeedMps = 20.f;                      ///< 区分裁判系统返回值是否正确
constexpr PidGains kYawPositionPid{50.0f, 0.f, 0.5f, 10.0f, 2.2f};   ///< 自瞄偏航位置 PID
constexpr PidGains kYawSpeedPid{0.65f, 0.0f, 0.0f, 10.0f, 0.f};      ///< 自瞄偏航速度 PID
constexpr PidGains kPitchPositionPid{40.0f, 0.f, 0.6f, 10.0f, 2.f};  ///< 自瞄俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.45f, 0.0f, 0.0f, 10.0f, 0.f};    ///< 自瞄俯仰速度 PID
}  // namespace aimbot

}  // namespace infantry3

// ══════════════════════════════════════════════════════════════════════════════
// Infantry4（变体 3）专用参数
// ══════════════════════════════════════════════════════════════════════════════
namespace infantry4 {
using namespace common;

// ── CAN 总线 ID 与电机配置 ──
namespace globals {
using namespace common::globals;

constexpr std::uint16_t kLeftWheelId = 0x02;   ///< 左轮毂电机 CAN ID
constexpr std::uint16_t kRightWheelId = 0x01;  ///< 右轮毂电机 CAN ID

constexpr std::uint16_t kFricLeftId = 0x01;   ///< 左摩擦轮电机 CAN ID
constexpr std::uint16_t kFricRightId = 0x04;  ///< 右摩擦轮电机 CAN ID
constexpr std::uint16_t kDialId = 0x03;       ///< 拨盘电机 CAN ID
}  // namespace globals

// ── 云台 ──
namespace gimbal {
using namespace common::gimbal;

constexpr float kPitchMinRad = -0.35f;  ///< 俯仰角下限 [rad]
constexpr float kPitchMaxRad = 0.65f;   ///< 俯仰角上限 [rad]

constexpr PidGains kYawPositionPid{30.0f, 0.0f, 0.5f, 10.0f, 1.0f};    ///< 偏航位置 PID
constexpr PidGains kYawSpeedPid{0.6f, 0.0f, 0.0f, 6.0f, 0.4f};         ///< 偏航速度 PID
constexpr PidGains kPitchPositionPid{23.0f, 0.0f, 0.8f, 10.0f, 0.4f};  ///< 俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 8.0f, 0.0f};      ///< 俯仰速度 PID

/// @brief 辨识得到的 9 个动力学参数（theta_0 ~ theta_8），用于前馈验证
constexpr float kIdentTheta[9] = {
    0.01840039f,   // theta_0: I1zz_com
    0.01493814f,   // theta_1: I2xx_com
    0.05832193f,   // theta_2: I2yy_com
    -0.03308606f,  // theta_3: m2*l2x 水平前向偏心
    -0.17656592,   // theta_4: m2*l2z 垂直上向偏心
    0.23623077,    // theta_5: fv1  yaw 粘滞摩擦
    0.33683372,    // theta_6: fc1  yaw 库仑摩擦
    0.61965618,    // theta_7: fv2  pitch 粘滞摩擦
    0.13903768,    // theta_8: fc2  pitch 库仑摩擦
};
}  // namespace gimbal

// ── 云台辨识 ──
namespace gimbal_ident {
using namespace common::gimbal_ident_common;
constexpr float kBaseFreqHz = 0.1f;                                           ///< 辨识轨迹基频 [Hz]
constexpr float kDmTorqueLimitNm = 10.0f;                                     ///< DM 电机力矩上限 [Nm]
constexpr float kDefaultDtS = 0.002f;                                         ///< 辨识控制周期 [s]
constexpr float kYawAmp[kHarmonicCount] = {1.0f, -0.6f, 0.4f, -0.35f, 0.1f};  ///< yaw 轴五次谐波幅值 [rad]
constexpr float kPitchAmp[kHarmonicCount] = {0.2106f, 0.2463f, 0.0804f, 0.0554f, 0.0391f};
constexpr float kPitchPhase[kHarmonicCount] = {1.2177f, 0.4006f, -0.8970f, 0.1462f, -2.6391f};
constexpr PidGains kIdentYawPosPid{20.0f, 0.0f, 0.1f, 10.0f, 0.0f};  ///< 辨识模式 yaw 位置 PID（单位置环，高增益）
constexpr PidGains kIdentPitchPosPid{70.0f, 0.0f, 0.5f, 10.0f, 0.0f};
constexpr float kIdentPitchCenter = 0.944f;  ///< 辨识轨迹 pitch 中心角 [rad]（机械中位，实际需根据云台标定）
constexpr float kIdentPitchTopLimit = 0.6f;     ///< 辨识轨迹 pitch 下限 [rad]
constexpr float kIdentPitchBottomLimit = 1.6f;  ///< 辨识轨迹 pitch 上限 [rad]
}  // namespace gimbal_ident

// ── 发射机构（双摩擦轮 + M3508 拨盘）──
namespace shoot {
inline constexpr int kFrictionWheelCount = 2;                             ///< 摩擦轮数量
constexpr float kFricSpeedTargetRpm = 6000.0f;                            ///< 摩擦轮目标转速 [rpm]
constexpr PidGains kFricSpeedPid{10.0f, 0.0f, 0.0f, 16000.0f, 2000.0f};   ///< 摩擦轮速度 PID
constexpr PidGains kDialSpeedPid{10.0f, 0.f, 0.f, 16000.0f, 1000.0f};     ///< 拨盘速度 PID
constexpr PidGains kDialPositionPid{0.5f, 0.f, 0.01f, 30000.0f, 500.0f};  ///< 拨盘位置 PID
constexpr int16_t kDialFireThreshold = -600;                              ///< 发射触发拨轮阈值
constexpr float kFricSpeedStepRpm = 20.0f;  ///< Z/X 键每次调整摩擦轮转速步长 [rpm]

// ── 本地热量闭环 ──
constexpr float kHeatPerShot = 10.0f;  ///< 每发子弹热量增量 [热量单位]
constexpr float kHeatSafetyMargin = 45.0f;  ///< 高热量模式停火余量：heat + kHeatPerShot > limit - margin 时抑制发射
constexpr float kLowHeatSafetyMargin = 20.0f;  ///< 低热量模式停火余量
constexpr float kHeatResumeMargin = 20.0f;  ///< 恢复余量：heat < limit - margin 时恢复，与停火线构成迟滞
constexpr uint16_t kDefaultHeatLimit = 240;       ///< 裁判系统离线时默认热量上限
constexpr uint16_t kDefaultCoolingRate = 40;      ///< 裁判系统离线时默认冷却速率 [热量单位/秒]
constexpr uint16_t kLowHeatLimitThreshold = 100;  ///< 低热量上限阈值：低于此值时降频并使用本地热量
constexpr float kLowHeatShootFrequencyHz = 6.0f;  ///< 低热量上限时发射频率 [Hz]
constexpr float kNormalShootFrequencyHz = 10.0f;  ///< 正常发射频率 [Hz]
}  // namespace shoot

// ── 底盘状态机 ──
namespace chassis_fsm {

constexpr StairClimbParams kStairClimb{
    .high_leg_length_m = 0.35f,
    .hook_leg_length_m = 0.35f,
    .retract_leg_length_m = 0.15f,
    .settle_leg_length_m = 0.16f,
    .contact_theta_threshold_rad = 0.50f,
    .hook_theta_target_rad = 0.80f,
    .retract_theta_target_rad = 1.f,
    .hook_theta_tolerance_rad = 0.10f,
    .leg_length_tolerance_m = 0.02f,
    .theta_dot_tolerance_rad_s = 0.50f,
    .settle_theta_tolerance_rad = 0.3f,
    .settle_theta_target_rad = 0.f,
    .settle_pitch_tolerance_rad = 0.18f,
    .settle_pitch_dot_tolerance_rad_s = 0.50f,
    .settle_roll_tolerance_rad = 0.25f,
    .hook_stable_ms = 20U,
    .retract_stable_ms = 50U,
    .settle_stable_ms = 100U,
    .hook_timeout_ms = 1200U,
    .retract_timeout_ms = 1200U,
    .settle_timeout_ms = 1500U,
    .theta_pid = {6.0f, 0.0f, 1.5f, 15.0f, 0.0f},
};

// ==== 倒地自启 ====
constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;        ///< 倒地确认时间 [ms]
constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;  ///< 自启超时 [ms]

// ==== 跳跃 ====
constexpr std::uint32_t kJumpLowPrepMs = 250U;      ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpLowPushMaxMs = 1000U;  ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpLowRecoverMs = 250U;   ///< 跳跃回收阶段持续时间（保底超时）[ms]
constexpr std::uint32_t kJumpLowRecoverMinMs = 80U;  ///< 跳跃回收阶段最低维持时间（此后开始判断离地）[ms]
constexpr float kJumpLowPrepLegLengthM = 0.13f;         ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpLowPushLegLengthM = 0.28f;         ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpLowRecoverLegLengthM = 0.15f;      ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpLowPushReachedLegLengthM = 0.26f;  ///< 蹬伸到位判定腿长 [m]

// ==== 自动跳跃（低腿长触发）====
constexpr std::uint32_t kJumpAutoPrepMs = 250U;      ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpAutoPushMaxMs = 1000U;  ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpAutoRecoverMs = 500U;   ///< 跳跃回收阶段持续时间（保底超时）[ms]
constexpr std::uint32_t kJumpAutoRecoverMinMs = 400U;  ///< 跳跃回收阶段最低维持时间（此后开始判断离地）[ms]
constexpr float kJumpAutoPrepLegLengthM = 0.16f;         ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpAutoPushLegLengthM = 0.25f;         ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpAutoRecoverLegLengthM = 0.18f;      ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpAutoPushReachedLegLengthM = 0.24f;  ///< 蹬伸到位判定腿长 [m]

// ==== 基本运动（腿长档位）====
constexpr float kLowLegLengthM = 0.16f;              ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.235f;             ///< 中腿长档位目标腿长 [m]
constexpr float kHighLegLengthM = 0.35f;             ///< 高腿长档位目标腿长 [m]
constexpr float kLegLengthRampTimeS = 0.5f;          ///< 腿长切换斜坡时间 [s]
constexpr std::uint32_t kSpinExitTimeoutMs = 3000U;  ///< 小陀螺预测退出超时兜底 [ms]
}  // namespace chassis_fsm

// ── 底盘控制 ──
namespace chassis {

// ==== 物理/机械参数 ====
constexpr float kControlDtS = 0.002f;  ///< 底盘控制周期 [s]（500Hz）
constexpr float kLegL1M = 0.215f;      ///< 五连杆主动杆长度 [m]
constexpr float kLegL2M = 0.254f;      ///< 五连杆从动杆长度 [m]

// -- 左腿弹簧补偿三次多项式系数：tau = c0 + c1*l + c2*l^2 + c3*l^3 --
constexpr float kLeftSpringC0 = 83.269401f;
constexpr float kLeftSpringC1 = -1832.833463f;
constexpr float kLeftSpringC2 = 4779.020146f;
constexpr float kLeftSpringC3 = -3726.603083f;
// -- 右腿弹簧补偿三次多项式系数 --
constexpr float kRightSpringC0 = -123.58f;
constexpr float kRightSpringC1 = 961.47f;
constexpr float kRightSpringC2 = -7421.14f;
constexpr float kRightSpringC3 = 13247.60f;

// -- 质量/惯量/重力 --
constexpr float kLegMassKg = 2.3f;     ///< 单条腿质量 [kg]
constexpr float kGravityMps2 = 9.81f;  ///< 重力加速度 [m/s²]

// -- 轮参数 --
constexpr float kWheelRadiusM = 0.2025f;  ///< 驱动轮轮距的一半（R_l） [m]

// -- 腿长→等效质心系数查找表 --
constexpr std::array<float, 24> kEtaLookupLegLengthM{
    0.11f, 0.12f, 0.13f, 0.14f, 0.15f, 0.16f, 0.17f, 0.18f, 0.19f, 0.20f, 0.21f, 0.22f,
    0.23f, 0.24f, 0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.30f, 0.31f, 0.32f, 0.33f, 0.34f,
};
constexpr std::array<float, 24> kEtaLookupLwM{
    0.061990f, 0.067466f, 0.072986f, 0.078550f, 0.084158f, 0.089810f, 0.095506f, 0.101246f,
    0.107030f, 0.112858f, 0.118730f, 0.124646f, 0.130606f, 0.136610f, 0.142658f, 0.148750f,
    0.154886f, 0.161066f, 0.167290f, 0.173558f, 0.179870f, 0.186226f, 0.192626f, 0.199070f,
};

// ==== 姿态安全/倒地恢复 ====
constexpr float kStandupThetaThresholdRad = 0.8f;     ///< 起立完成判定 [rad]
constexpr float kPostureRollMinRad = -0.5f;           ///< 横滚角安全下限 [rad]
constexpr float kPostureRollMaxRad = 0.5f;            ///< 横滚角安全上限 [rad]
constexpr float kPostureThetaBMinRad = -0.7f;         ///< 机体俯仰角安全下限 [rad]
constexpr float kPostureThetaBMaxRad = 0.7f;          ///< 机体俯仰角安全上限 [rad]
constexpr float kPostureThetaLegMinRad = -0.8f;       ///< 腿摆角安全下限 [rad]
constexpr float kPostureThetaLegMaxRad = 2.f;         ///< 摆角安全上限 [rad]
constexpr float kLegRecoverThetaDotTarget = -2.0f;    ///< 倒地恢复时腿摆角速度目标 [rad/s]
constexpr float kLegRecoverThetaDotRampStep = 0.06f;  ///< 倒地恢复腿摆角速度斜坡步长 [(rad/s)/周期]
constexpr float kManualRecoveryLegSpeedRadS = 0.5f;   ///< 手动倒地恢复腿摆角速度 [rad/s]
constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;   ///< 倒地恢复零力矩区间下限 [rad]
constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;   ///< 倒地恢复零力矩区间上限 [rad]

// ==== 倒地恢复软着陆 ====
constexpr float kRecoveryDecelZoneRad = 0.6f;   ///< 恢复减速区宽度 [rad]（接近目标边界时开始减速）
constexpr float kRecoveryMinSpeedRadS = 0.08f;  ///< 恢复减速区边界最低速度 [rad/s]
constexpr float kRecoveryGravityRampScale = 0.35f;  ///< 恢复时重力补偿斜坡比例（越大身体越不砸）

// -- 离地检测 --
constexpr float kOffGroundSupportForceThresholdN = 20.0f;  ///< 支撑力低于此值判定为离地 [N]
constexpr float kOffGroundSupportForceClampN = 100.0f;     ///< 离地时支持力限幅值 [N]

// -- 中腿长下压 --
constexpr float kMidLegDipTriggerLengthM = 0.27f;  ///< 中腿长模式下触发下压的腿长阈值 [m]
constexpr float kMidLegDipTargetLengthM = 0.22f;   ///< 下压目标腿长 [m]
constexpr uint16_t kMidLegDipHoldTicks = 1000;     ///< 下压维持时间 [ticks @ 500Hz = 1s]

// ==== 物理参数（变体专属）====
constexpr float kBodyMassKg = 22.0f;  ///< 机体质量 [kg]

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.002f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵）====
constexpr std::array<float, 240> kCtrlPLow{
    -4.2002,  -30.506, 23.552,  45.072,  -19.612,  -17.089, -7.184,   -34.896,  35.254,   58.276,  -39.847, -23.324,
    -0.74946, 3.0279,  -1.3247, -3.0189, -0.21905, 2.0457,  -2.3907,  9.9908,   -4.6238,  -9.5696, -1.3151, 7.2824,
    -14.807,  -92.661, 18.737,  91.855,  7.4523,   -24.563, -0.98121, -7.4867,  2.9782,   -3.4798, 3.117,   -3.4797,
    -5.1682,  6.188,   -8.5495, 6.4569,  -13.528,  9.1089,  -0.46257, -1.3609,  -0.55063, 5.8901,  -12.352, 2.3801,
    -34.369,  52.234,  44.371,  -4.7128, -50.002,  -39.316, -3.3224,  2.8432,   7.8258,   2.7356,  -8.3379, -7.2571,
    -4.2002,  23.552,  -30.506, -17.089, -19.612,  45.072,  -7.184,   35.254,   -34.896,  -23.324, -39.847, 58.276,
    0.74946,  1.3247,  -3.0279, -2.0457, 0.21905,  3.0189,  2.3907,   4.6238,   -9.9908,  -7.2824, 1.3151,  9.5696,
    -5.1682,  -8.5495, 6.188,   9.1089,  -13.528,  6.4569,  -0.46257, -0.55063, -1.3609,  2.3801,  -12.352, 5.8901,
    -14.807,  18.737,  -92.661, -24.563, 7.4523,   91.855,  -0.98121, 2.9782,   -7.4867,  -3.4797, 3.117,   -3.4798,
    -34.369,  44.371,  52.234,  -39.316, -50.002,  -4.7128, -3.3224,  7.8258,   2.8432,   -7.2571, -8.3379, 2.7356,
    10.587,   6.4431,  -30.97,  -60.017, 59.419,   17.886,  16.234,   6.2743,   -54.442,  -81.903, 102.49,  30.232,
    -0.73658, -4.4047, -1.485,  7.3166,  -1.9731,  2.3067,  -2.3564,  -14.677,  -4.3388,  24.027,  -6.4526, 6.6163,
    67.53,    -114.05, 7.0866,  95.782,  50.752,   -24.393, 3.7451,   -1.1537,  -2.6787,  2.0095,  5.9814,  0.59065,
    -3.6052,  -35.537, -7.1719, 36.018,  -18.819,  -14.289, -0.14134, -0.37346, 3.4323,   -6.2892, 5.1385,  -9.6003,
    -64.556,  -295.86, 99.525,  354.72,  47.074,   -133.77, -2.3135,  -20.381,  3.2486,   17.873,  12.713,  -6.6133,
    10.587,   -30.97,  6.4431,  17.886,  59.419,   -60.017, 16.234,   -54.442,  6.2743,   30.232,  102.49,  -81.903,
    0.73658,  1.485,   4.4047,  -2.3067, 1.9731,   -7.3166, 2.3564,   4.3388,   14.677,   -6.6163, 6.4526,  -24.027,
    -3.6052,  -7.1719, -35.537, -14.289, -18.819,  36.018,  -0.14134, 3.4323,   -0.37346, -9.6003, 5.1385,  -6.2892,
    67.53,    7.0866,  -114.05, -24.393, 50.752,   95.782,  3.7451,   -2.6787,  -1.1537,  0.59065, 5.9814,  2.0095,
    -64.556,  99.525,  -295.86, -133.77, 47.074,   354.72,  -2.3135,  3.2486,   -20.381,  -6.6133, 12.713,  17.873,
};
constexpr std::array<float, 240> kCtrlPMid{
    -4.7828,  -30.819,  26.254,   49.299,   -26.06,   -17.764,  -8.5171, -35.316, 41.185,   65.63,    -52.491,
    -25.089,  -0.55182, 2.6835,   -0.79655, -2.9092,  0.023611, 1.3715,  -1.9308, 9.7635,   -3.2278,  -10.176,
    -0.51743, 5.6605,   -21.71,   -91.175,  13.082,   101.64,   0.81835, -16.402, -1.1473,  -7.0184,  3.0579,
    -3.9499,  2.3004,   -3.1712,  -4.7856,  11.166,   -4.9881,  -8.2335, -8.5185, 10.714,   -0.45538, -1.1835,
    -0.58146, 5.9747,   -13.049,  3.9045,   -28.614,  63.925,   28.04,   -42.691, -33.167,  -27.862,  -3.189,
    3.9729,   7.1511,   0.80752,  -8.2142,  -6.8093,  -4.7828,  26.254,  -30.819, -17.764,  -26.06,   49.299,
    -8.5171,  41.185,   -35.316,  -25.089,  -52.491,  65.63,    0.55182, 0.79655, -2.6835,  -1.3715,  -0.023611,
    2.9092,   1.9308,   3.2278,   -9.7635,  -5.6605,  0.51743,  10.176,  -4.7856, -4.9881,  11.166,   10.714,
    -8.5185,  -8.2335,  -0.45538, -0.58146, -1.1835,  3.9045,   -13.049, 5.9747,  -21.71,   13.082,   -91.175,
    -16.402,  0.81835,  101.64,   -1.1473,  3.0579,   -7.0184,  -3.1712, 2.3004,  -3.9499,  -28.614,  28.04,
    63.925,   -27.862,  -33.167,  -42.691,  -3.189,   7.1511,   3.9729,  -6.8093, -8.2142,  0.80752,  5.8233,
    -5.0714,  -9.5145,  -25.509,  34.192,   3.0685,   9.2045,   -5.82,   -23.531, -38.736,  61.189,   10.465,
    -0.59133, -2.1136,  -0.92769, 3.6731,   -0.58584, 1.4401,   -2.069,  -7.8347, -3.0977,  13.466,   -2.3958,
    4.8023,   53.643,   -126.15,  7.0838,   141.38,   20.528,   -17.277, 2.3965,  -2.8356,  -1.0481,  3.0321,
    5.547,    -0.55239, -4.4676,  -18.91,   -3.4741,  24.682,   -2.9578, -15.919, -0.21029, -0.86499, 2.3666,
    -2.2778,  1.3475,   -5.2515,  -47.783,  -164.14,  61.822,   218.45,  15.272,  -86.351,  -2.5237,  -14.592,
    4.3634,   15.252,   6.7954,   -7.2945,  5.8233,   -9.5145,  -5.0714, 3.0685,  34.192,   -25.509,  9.2045,
    -23.531,  -5.82,    10.465,   61.189,   -38.736,  0.59133,  0.92769, 2.1136,  -1.4401,  0.58584,  -3.6731,
    2.069,    3.0977,   7.8347,   -4.8023,  2.3958,   -13.466,  -4.4676, -3.4741, -18.91,   -15.919,  -2.9578,
    24.682,   -0.21029, 2.3666,   -0.86499, -5.2515,  1.3475,   -2.2778, 53.643,  7.0838,   -126.15,  -17.277,
    20.528,   141.38,   2.3965,   -1.0481,  -2.8356,  -0.55239, 5.547,   3.0321,  -47.783,  61.822,   -164.14,
    -86.351,  15.272,   218.45,   -2.5237,  4.3634,   -14.592,  -7.2945, 6.7954,  15.252,
};
constexpr std::array<float, 240> kCtrlPHigh{
    -4.1135,  -26.56,   22.505,   42.158,   -22.061, -15.25,   -7.7594, -32.49,  37.55,   59.934,   -47.426,
    -22.936,  -0.53884, 2.5706,   -0.76192, -2.8049, 0.095475, 1.2874,  -1.886,  9.3544,  -3.0902,  -9.8119,
    -0.23946, 5.3322,   -20.641,  -89.524,  12.46,   100.85,   0.22103, -15.637, -1.0921, -6.6055,  2.8286,
    -3.6983,  2.1315,   -2.954,   -4.6739,  11.155,  -5.1196,  -8.9279, -5.9781, 9.9242,  -0.43672, -1.0334,
    -0.54561, 5.3781,   -11.837,  3.5094,   -27.985, 61.293,   27.697,  -39.197, -32.511, -27.502,  -3.0818,
    3.8762,   6.7872,   0.70551,  -7.7605,  -6.4771, -4.1135,  22.505,  -26.56,  -15.25,  -22.061,  42.158,
    -7.7594,  37.55,    -32.49,   -22.936,  -47.426, 59.934,   0.53884, 0.76192, -2.5706, -1.2874,  -0.095475,
    2.8049,   1.886,    3.0902,   -9.3544,  -5.3322, 0.23946,  9.8119,  -4.6739, -5.1196, 11.155,   9.9242,
    -5.9781,  -8.9279,  -0.43672, -0.54561, -1.0334, 3.5094,   -11.837, 5.3781,  -20.641, 12.46,    -89.524,
    -15.637,  0.22103,  100.85,   -1.0921,  2.8286,  -6.6055,  -2.954,  2.1315,  -3.6983, -27.985,  27.697,
    61.293,   -27.502,  -32.511,  -39.197,  -3.0818, 6.7872,   3.8762,  -6.4771, -7.7605, 0.70551,  5.3137,
    -3.6706,  -9.4828,  -24.317,  31.162,   3.5255,  8.9446,   -4.7766, -23.538, -38.585, 59.245,   10.813,
    -0.57579, -2.152,   -0.94911, 3.7242,   -0.6008, 1.4604,   -2.0151, -7.9747, -3.1661, 13.649,   -2.4422,
    4.859,    54.168,   -125.58,  7.482,    138.9,   20.393,   -17.745, 2.4085,  -2.8468, -1.0042,  3.2536,
    5.2569,   -0.54842, -4.3787,  -19.851,  -3.9712, 26.218,   -3.2372, -15.598, -0.2148, -0.88153, 2.3137,
    -2.1699,  1.4049,   -5.3602,  -47.168,  -167.34, 62.412,   221.61,  15.839,  -86.946, -2.5109,  -14.682,
    4.4296,   15.36,    6.6937,   -7.3188,  5.3137,  -9.4828,  -3.6706, 3.5255,  31.162,  -24.317,  8.9446,
    -23.538,  -4.7766,  10.813,   59.245,   -38.585, 0.57579,  0.94911, 2.152,   -1.4604, 0.6008,   -3.7242,
    2.0151,   3.1661,   7.9747,   -4.859,   2.4422,  -13.649,  -4.3787, -3.9712, -19.851, -15.598,  -3.2372,
    26.218,   -0.2148,  2.3137,   -0.88153, -5.3602, 1.4049,   -2.1699, 54.168,  7.482,   -125.58,  -17.745,
    20.393,   138.9,    2.4085,   -1.0042,  -2.8468, -0.54842, 5.2569,  3.2536,  -47.168, 62.412,   -167.34,
    -86.946,  15.839,   221.61,   -2.5109,  4.4296,  -14.682,  -7.3188, 6.6937,  15.36,
};
constexpr std::array<float, 240> kCtrlPSpin{
    -2.5313,  -18.606,  14.889,   28.222,   -13.797,  -10.267,  -5.0287, -23.997,  25.574,   41.608,   -30.631,
    -16.082,  -0.41487, 1.8144,   -0.70769, -1.7942,  0.045487, 1.1262,  -1.4572,  6.6125,   -2.7863,  -6.2634,
    -0.26751, 4.5377,   -12.487,  -80.394,  11.998,   87.091,   1.7282,  -15.552,  -0.73969, -5.2244,  2.1644,
    -2.8311,  1.8017,   -2.4078,  -3.8,     7.0948,   -6.6524,  -2.5484, -2.702,   8.9348,   -0.35713, -0.76625,
    -0.30118, 3.8783,   -8.2524,  2.0651,   -21.688,  32.315,   28.251,  -0.95623, -30.809,  -26.937,  -2.5158,
    2.3223,   5.7882,   1.9831,   -6.2379,  -5.5146,  -2.5313,  14.889,  -18.606,  -10.267,  -13.797,  28.222,
    -5.0287,  25.574,   -23.997,  -16.082,  -30.631,  41.608,   0.41487, 0.70769,  -1.8144,  -1.1262,  -0.045487,
    1.7942,   1.4572,   2.7863,   -6.6125,  -4.5377,  0.26751,  6.2634,  -3.8,     -6.6524,  7.0948,   8.9348,
    -2.702,   -2.5484,  -0.35713, -0.30118, -0.76625, 2.0651,   -8.2524, 3.8783,   -12.487,  11.998,   -80.394,
    -15.552,  1.7282,   87.091,   -0.73969, 2.1644,   -5.2244,  -2.4078, 1.8017,   -2.8311,  -21.688,  28.251,
    32.315,   -26.937,  -30.809,  -0.95623, -2.5158,  5.7882,   2.3223,  -5.5146,  -6.2379,  1.9831,   6.0093,
    1.7314,   -14.219,  -35.457,  34.815,   7.1886,   10.632,   1.2894,  -31.457,  -55.656,  69.414,   15.964,
    -0.42295, -2.6712,  -0.88955, 4.5063,   -1.0731,  1.4089,   -1.4858, -9.8706,  -2.8694,  16.466,   -4.0614,
    4.5247,   59.222,   -106.36,  7.6109,   84.332,   31.927,   -20.393, 2.6476,   -1.0191,  -1.4403,  0.39592,
    5.6948,   -0.28932, -2.523,   -29.241,  -7.5923,  35.948,   -10.274, -8.451,   -0.12058, -0.82727, 2.4513,
    -3.2903,  2.0896,   -5.8965,  -35.715,  -188.01,  58.576,   226.43,  31.331,   -81.613,  -1.7245,  -16.25,
    3.4698,   15.173,   8.8959,   -6.3425,  6.0093,   -14.219,  1.7314,  7.1886,   34.815,   -35.457,  10.632,
    -31.457,  1.2894,   15.964,   69.414,   -55.656,  0.42295,  0.88955, 2.6712,   -1.4089,  1.0731,   -4.5063,
    1.4858,   2.8694,   9.8706,   -4.5247,  4.0614,   -16.466,  -2.523,  -7.5923,  -29.241,  -8.451,   -10.274,
    35.948,   -0.12058, 2.4513,   -0.82727, -5.8965,  2.0896,   -3.2903, 59.222,   7.6109,   -106.36,  -20.393,
    31.927,   84.332,   2.6476,   -1.4403,  -1.0191,  -0.28932, 5.6948,  0.39592,  -35.715,  58.576,   -188.01,
    -81.613,  31.331,   226.43,   -1.7245,  3.4698,   -16.25,   -6.3425, 8.8959,   15.173,
};

// ==== 基本运动（PID 增益）====
constexpr PidGains kLeftL0Pid{2500.0f, 0.f, 200.0f, 170.0f, 30.0f};   ///< 左腿腿长 PID（常规）
constexpr PidGains kRightL0Pid{2500.0f, 0.f, 200.0f, 170.0f, 30.0f};  ///< 右腿腿长 PID（常规）
constexpr PidGains kRollPid{800.0f, 0.0f, 200.0f, 80.0f, 0.0f};       ///< 横滚平衡 PID

// ==== 跳跃（PID 增益）====
constexpr float kJumpPushForceN = 200.0f;                                    ///< 蹬伸阶段单腿基础支撑力 [N]
constexpr PidGains kLeftL0PidJumpTwo{3500.0f, 0.0f, 300.0f, 180.0f, 0.0f};   ///< 左腿蹬伸 PID
constexpr PidGains kRightL0PidJumpTwo{3000.0f, 0.0f, 300.0f, 180.0f, 0.0f};  ///< 右腿蹬伸 PID
constexpr PidGains kLeftL0PidJumpThree{4000.0f, 0.15f, 400.0f, 170.0f, 30.0f};   ///< 左腿回收 PID
constexpr PidGains kRightL0PidJumpThree{3500.0f, 0.15f, 400.0f, 170.0f, 30.0f};  ///< 右腿回收 PID

// ==== 中腿长下压（PID 增益）====
constexpr PidGains kLeftL0PidDip{400.0f, 0.f, 200.0f, 170.0f, 30.0f};   ///< 左腿下压腿长 PID
constexpr PidGains kRightL0PidDip{400.0f, 0.f, 200.0f, 170.0f, 30.0f};  ///< 右腿下压腿长 PID

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{20.0f, 0.0f, 10.0f, 25.0f, 0.0f};   ///< 左腿摆角速度 PID
constexpr PidGains kRightLegTurnPid{20.0f, 0.0f, 10.0f, 25.0f, 0.0f};  ///< 右腿摆角速度 PID
constexpr PidGains kLeftLegTurnPidManual{10.0f, 0.0f, 2.0f, 20.0f, 0.0f};  ///< 左腿摆角速度 PID（手动倒地恢复）
constexpr PidGains kRightLegTurnPidManual{10.0f, 0.0f, 2.0f, 20.0f, 0.0f};  ///< 右腿摆角速度 PID（手动倒地恢复）

// ==== l0_ddot 低通滤波 ====
constexpr float kL0DdotFilterCutoffHz = 5.0f;    ///< l0_ddot 低通滤波截止频率 [Hz]
constexpr float kL0DdotFilterSampleHz = 500.0f;  ///< l0_ddot 低通滤波采样频率 [Hz]
}  // namespace chassis

// ── 控制环 ──
namespace control_loop {

/// @brief 纵向速度斜坡参数（加速/制动步长分离）
struct SdotRampParams {
  float accel_step;  ///< 加速步长 [(m/s)/周期]
  float brake_step;  ///< 制动步长 [(m/s)/周期]
};

// -- 小陀螺/跳跃触发阈值 --
constexpr std::int16_t kWheelSpinThreshold = 220;             ///< 拨轮超过此值触发小陀螺保持
constexpr std::int16_t kWheelActionThreshold = 320;           ///< 拨轮回中后快速负推超过此值触发跳跃
constexpr std::int16_t kWheelCenterThreshold = 80;            ///< 拨轮归中阈值
constexpr std::uint16_t kAutoJumpDistanceThresholdMm = 500U;  ///< 自动跳跃 DYP 测距阈值 [mm]
constexpr float kAutoJumpHoldTimeS = 1.0f;                    ///< 自动跳跃开关拨轮保持时间 [s]
constexpr float kAutoJumpDistanceHoldTimeS = 0.05f;           ///< 自动跳跃测距持续低于阈值判定时间 [s]

constexpr float kControlLoopDtS = 0.002f;  ///< 控制环周期 [s]

// -- 摇杆/键鼠输入归一化 --
constexpr std::int16_t kDr16AxisMaxAbs = 660;       ///< DR16 摇杆轴最大绝对值
constexpr float kRcStickMax = 660.0f;               ///< RC 摇杆最大值
constexpr float kTcMouseMax = 200.0f;               ///< 图传鼠标增量最大值
constexpr float kRcYawRateMaxRadS = -4.f;           ///< RC 摇杆满偏时偏航积分速率 [rad/s]
constexpr float kRcPitchRateMaxRadS = 2.5f;         ///< RC 摇杆满偏时俯仰积分速率 [rad/s]
constexpr float kTcMouseYawRateMaxRadS = -2.0f;     ///< 图传鼠标满偏时偏航积分速率 [rad/s]
constexpr float kTcMousePitchRateMaxRadS = 1.5f;    ///< 图传鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kDr16MouseMax = 1600.0f;            ///< DR16 鼠标增量最大值（用于积分目标速率计算）
constexpr float kDr16MouseYawRateMaxRadS = -2.0f;   ///< DR16 鼠标满偏时偏航积分速率 [rad/s]
constexpr float kDr16MousePitchRateMaxRadS = 1.5f;  ///< DR16 鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kPitchTargetMinRad = -0.35f;        ///< RC 积分俯仰目标下限 [rad]
constexpr float kPitchTargetMaxRad = 0.6f;          ///< RC 积分俯仰目标上限 [rad]
constexpr float kKeyboardAccelRampStep = 0.006f;    ///< 键盘 WASD 加速斜坡步进（每周期，0→1 约 0.5s）
constexpr float kKeyboardBrakeRampStep = 0.2f;      ///< 键盘 WASD 减速斜坡步进（每周期，1→0 约 0.25s）

// -- 云台启动归中判稳 --
constexpr float kGimbalStartupYawAlignErrorRad = 0.04f;           ///< 归中完成位置误差阈值 [rad]
constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;            ///< 归中完成速度阈值 [rad/s]
constexpr std::uint32_t kGimbalStartupYawAlignStableTicks = 50U;  ///< 归中判稳所需连续满足周期数

// -- 偏航就绪判稳 --
constexpr float kYawFollowDriveReadyErrorRad = 0.04f;           ///< 偏航就绪位置误差阈值 [rad]
constexpr float kYawFollowDriveReadyVelRadS = 0.25f;            ///< 偏航就绪速度阈值 [rad/s]
constexpr std::uint32_t kYawFollowDriveReadyStableTicks = 50U;  ///< 偏航就绪判稳所需连续周期数

// ==== 基本运动 ====
constexpr float kTargetForwardSpeedMaxMps = 2.1f;         ///< 最大前进速度 [m/s+]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.1f;  ///< 高腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxMidLegMps = 1.0f;   ///< F键中腿长模式最大前进速度 [m/s]
constexpr float kTargetSpeedBiasLowLegMps = 0.0f;         ///< 低腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegMps = 0.f;          ///< C键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegFMps = 0.f;         ///< F键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasHighLegMps = 0.0f;        ///< 高腿长目标速度偏置 [m/s]
constexpr float kMaxSafeYawRateRadS = 4.0f;               ///< 摩擦圆最大安全偏航速率 [rad/s]
constexpr float kVxInputDeadbandNorm = 0.1f;              ///< 前进输入死区
constexpr float kVyInputDeadbandNorm = 0.1f;              ///< 平移输入死区
constexpr float kYawFollowRampStepRadS = 0.065f;          ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.3f;  ///< 位置锚定冻结速度阈值 [m/s]

// ==== 落地减速（离地→落地时通过腿摆角辅助减速）====
constexpr float kLandingDecelThetaGain = 0.f;               ///< 落地减速腿摆角增益 [rad/(m/s)]
constexpr float kLandingDecelThetaMaxRad = 0.3f;            ///< 落地减速腿摆角最大偏置 [rad]
constexpr float kLandingDecelThetaRampStepRad = 0.01f;      ///< 落地减速腿摆角每周期斜坡步长 [rad/tick]
constexpr std::uint32_t kLandingDecelOffGroundMinMs = 40U;  ///< 离地最短持续时间（防单帧误判）[ms]
constexpr std::uint32_t kLandingDecelStableDurationMs = 400U;  ///< 落地减速稳定保持时间 [ms]

constexpr float kYawFollowFixedTargetRad = 0.f;                  ///< 偏航跟随固定目标偏置角 [rad]
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;            ///< 偏航跟随侧向目标偏置角 [rad]
constexpr PidGains kYawFollowPid{8.2f, 0.0f, 1.2f, 6.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置 ====
constexpr float kExpectedThetaLlBiasRadLowLeg = 0.01f;     ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = 0.01f;     ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLeg = 0.001f;    ///< 中腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLeg = 0.001f;    ///< 中腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadHighLeg = -0.019f;  ///< 高腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadHighLeg = -0.019f;  ///< 高腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = 0.027f;           ///< 期望机体俯仰偏置 [rad]

// ==== 速度斜坡参数 ====
constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.008f};    ///< 低腿长速度斜坡
constexpr SdotRampParams kSdotRampMidLeg{0.007f, 0.007f};   ///< 中腿长速度斜坡(C 键触发)
constexpr SdotRampParams kSdotRampMidLegF{0.01f, 0.008f};   ///< 中腿长速度斜坡（F 键触发）
constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};  ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.05f;           ///< 小陀螺进入偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinExitYawRampStepRadS = 0.15f;       ///< 小陀螺退出偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS = 8.f;            ///< 小陀螺目标自旋角速度 [rad/s]
constexpr float kSpinExitYawAlignThresholdRad = 0.15f;  ///< 小陀螺预测退出：yaw 对齐阈值 [rad]
constexpr float kSpinTranslationGain = 0.5f;            ///< 小陀螺平移增益
constexpr float kSpinThetaLlBiasRad = 0.02f;            ///< 小陀螺时左腿摆角偏置 [rad]
constexpr float kSpinThetaLrBiasRad = 0.02f;            ///< 小陀螺时右腿摆角偏置 [rad]
constexpr float kSpinLegLengthBiasM = 0.0f;             ///< 小陀螺时腿长偏差（左+右-）[m]
constexpr float kSpinThetaBBiasRad = 0.018f;            ///< 小陀螺时俯仰目标偏置 [rad]

// ==== 跳跃腿摆角偏置 ====
constexpr float kJumpThetaLlBiasRad = -0.1f;  ///< 跳跃时左腿摆角偏置 [rad]
constexpr float kJumpThetaLrBiasRad = -0.1f;  ///< 跳跃时右腿摆角偏置 [rad]
}  // namespace control_loop

// ── 执行器 ──
namespace actuators {
using namespace common::actuators;

constexpr float kLeftWheelTorqueToCurrent = 2436.5f;   ///< 左轮力矩→电流转换系数
constexpr float kRightWheelTorqueToCurrent = 2436.5f;  ///< 右轮力矩→电流转换系数
}  // namespace actuators

// ── 状态估计 ──
namespace state_estimator {
constexpr float kDefaultDtS = 0.002f;                   ///< 估计器默认周期 [s]
constexpr float kDefaultExpectedSdotMps = 0.05f;        ///< 默认期望纵向速度 [m/s]
constexpr float kLegL1M = 0.215f;                       ///< 五连杆主动杆长度 [m]
constexpr float kLegL2M = 0.254f;                       ///< 五连杆从动杆长度 [m]
constexpr float kWheelRadiusM = 0.0575f;                ///< 驱动轮半径 [m]
constexpr float kWheelReductionRatio = 17.0f / 268.0f;  ///< 轮电机到车轮的速度换算比
constexpr float kMaxValidSpeedMps = 8.0f;               ///< 速度融合可信上限 [m/s]
constexpr float kThetaDotFilterCutoffHz = 8.0f;         ///< 腿摆角速度低通滤波截止频率 [Hz]

// -- IMU 加速度融合 --
constexpr float kImuAccelFilterSampleHz = 500.0f;          ///< 加速度低通滤波器采样频率 [Hz]
constexpr float kImuAccelFilterCutoffHz = 5.0f;            ///< 加速度低通滤波器截止频率 [Hz]
constexpr std::uint32_t kAccelBiasInitSamples = 1500U;     ///< 加速度零偏估计所需初始样本数
constexpr float kAccelZeroWheelSpeedThresholdMps = 0.02f;  ///< 轮速零速判定阈值 [m/s]
constexpr float kAccelZeroHighThresholdMps2 = 0.5f;        ///< 加速度零偏校准上阈值 [m/s²]
constexpr float kAccelZeroLowThresholdMps2 = 0.2f;         ///< 加速度零偏校准下阈值 [m/s²]

// -- 卡尔曼滤波器矩阵 --
constexpr float kKalmanMinVariance = 1e-5f;
constexpr float kThetaPiHalf = 1.57079632679489661923f;
constexpr std::array<float, 4> kKalmanF{1.0f, kDefaultDtS, 0.0f, 1.0f};  ///< 状态转移矩阵 F
constexpr std::array<float, 4> kKalmanQ{0.0005f, 0.0f, 0.0f, 0.04f};     ///< 过程噪声协方差 Q
constexpr std::array<float, 4> kKalmanR{0.5f, 0.0f, 0.0f, 2.0f};         ///< 观测噪声协方差 R
constexpr std::array<float, 4> kKalmanP{10.0f, 0.0f, 0.0f, 10.0f};       ///< 初始估计协方差 P
constexpr std::array<float, 4> kKalmanH{1.0f, 0.0f, 0.0f, 1.0f};         ///< 观测矩阵 H

// -- 关节零位偏移 --
constexpr float kLeftPhi1OffsetRad = 1.38f + M_PI;   ///< 左腿前关节零位偏移 [rad]
constexpr float kLeftPhi4OffsetRad = 0.86f;          ///< 左腿后关节零位偏移 [rad]
constexpr float kRightPhi1OffsetRad = 1.26f + M_PI;  ///< 右腿前关节零位偏移 [rad]
constexpr float kRightPhi4OffsetRad = 1.02f;         ///< 右腿后关节零位偏移 [rad]
}  // namespace state_estimator

// ── 腿部运动学 ──
namespace leg_kinematics {
constexpr float kDefaultDtS = 0.002f;           ///< 运动学默认解算周期 [s]
constexpr float kMinSin = 1e-5f;                ///< 正弦值下限（避免除零奇异）
constexpr float kMinLen = 1e-5f;                ///< 长度下限（避免除零奇异）
constexpr float kL0DotFilterCutoffHz = 10.0f;   ///< l0_dot 低通滤波截止频率 [Hz]
constexpr float kL0DotFilterSampleHz = 500.0f;  ///< l0_dot 低通滤波采样频率 [Hz]
}  // namespace leg_kinematics

namespace remote_control_can_bridge {
using namespace common::remote_control_can_bridge;
}
namespace main {
using namespace common::main;
}

// ── 自瞄通信 ──
namespace aimbot {
constexpr uint8_t kRobotId = 4U;                 ///< 机器人 ID
constexpr float kBulletSpeedMps = 23.0f;         ///< 弹速 [m/s]
constexpr float kBulletDefaultSpeedMps = 23.f;   ///< 默认弹速
constexpr float kBulletBoundarySpeedMps = 20.f;  ///< 区分裁判系统返回值是否正确

constexpr PidGains kYawPositionPid{30.0f, 0.f, 0.5f, 10.0f, 1.5f};     ///< 自瞄偏航位置 PID
constexpr PidGains kYawSpeedPid{0.55f, 0.0f, 0.0f, 8.0f, 0.4f};        ///< 自瞄偏航速度 PID
constexpr PidGains kPitchPositionPid{30.0f, 1.8f, 0.7f, 10.0f, 2.5f};  ///< 自瞄俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.56f, 0.0f, 0.0f, 8.0f, 0.0f};      ///< 自瞄俯仰速度 PID
}  // namespace aimbot

}  // namespace infantry4

// ══════════════════════════════════════════════════════════════════════════════
// 编译期变体选择：通过 WHEEL_LEGGED_ROBOT_VARIANT 宏切换到目标变体命名空间
// ══════════════════════════════════════════════════════════════════════════════
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
namespace active = hero;
#elif WHEEL_LEGGED_ROBOT_VARIANT == 2
namespace active = infantry3;
#elif WHEEL_LEGGED_ROBOT_VARIANT == 3
namespace active = infantry4;
#else
#error "WHEEL_LEGGED_ROBOT_VARIANT must be 1, 2, or 3"
#endif

}  // namespace wheel_legged::params
