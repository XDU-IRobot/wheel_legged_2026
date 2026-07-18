#pragma once

#include <array>
#include <cstdint>
#include <librm.hpp>
#include "librm.hpp"
#include "targets/wheel_legged/include/utils/yaw_speed_feedforward.hpp"

#ifndef WHEEL_LEGGED_ROBOT_VARIANT
#define WHEEL_LEGGED_ROBOT_VARIANT 2
#endif

#include "../tools/kctrlp_generated.hpp"

namespace wheel_legged::params {

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
  float retract_theta_tolerance_rad;
  float hook_theta_tolerance_rad;
  float leg_length_tolerance_m;
  float settle_theta_tolerance_rad;
  float settle_theta_target_rad;
  float settle_theta_ramp_step_rad;
  std::uint32_t hook_stable_ms;
  std::uint32_t retract_stable_ms;
  std::uint32_t settle_stable_ms;
  std::uint32_t hook_timeout_ms;
  std::uint32_t retract_timeout_ms;
  std::uint32_t settle_timeout_ms;
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

}  // namespace globals

// ── 云台公共（控制周期、力矩上限、重力补偿、电机配置）──
namespace gimbal {
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

constexpr float kDefaultDtS = 0.002f;      ///< 云台控制默认周期 [s]
constexpr float kDmTorqueLimitNm = 10.0f;  ///< DM 电机力矩上限 [Nm]

const DmMitSettings kYawMotorSettings{0x12, 0x02, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
}  // namespace gimbal

// ── 云台辨识公共常量 ──
namespace gimbal_ident_common {
constexpr size_t kHarmonicCount = 5;                   ///< 五次谐波
constexpr float kRpmToRadPerSec = kPi * 2.0f / 60.0f;  ///< rpm → rad/s
constexpr size_t kIdentUartTxBufSize = 256;            ///< 扩展辨识 CSV 串口发送缓冲区大小 [byte]

/// @brief 辨识子模式 (编译时选择，通过遥控器左 DOWN + 右 UP 激活 kIdent 状态后生效)
enum class IdentSubMode : uint8_t {
  kGravity = 0,   ///< Step 1: 静态重力 — Pitch 多角度静止悬停
  kFriction,      ///< Step 2: Pitch 重力/摩擦联合辨识 + Yaw 摩擦
  kPitchInertia,  ///< Step 3: Pitch 惯量 — Yaw 固定, Pitch 正弦加减速
  kCoupling,      ///< Step 4: 耦合惯量 — Pitch 固定, Yaw 正弦加减速
  kHarmonic,      ///< Step 5: 五次谐波 — 综合激励 (原有)
};

/// @brief 重力模式 Pitch 角度序列元素个数上限
constexpr size_t kGravityAngleMaxCount = 10;
/// @brief 摩擦模式速度档位个数上限
constexpr size_t kFrictionVelocityMaxCount = 8;
/// @brief 耦合模式 Pitch 角度序列元素个数上限
constexpr size_t kCouplingAngleMaxCount = 12;
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
constexpr std::uint16_t kRxStdIdD = 0x113;  ///< CAN 帧 D 标准 ID（机器人血量）
constexpr std::uint16_t kRxStdIdE = 0x114;  ///< CAN 帧 E 标准 ID（哨兵血量）
constexpr std::size_t kPayloadSizeA = 8U;   ///< 帧 A 数据长度 [byte]
constexpr std::size_t kPayloadSizeB = 8U;   ///< 帧 B 数据长度 [byte]
constexpr std::size_t kPayloadSizeC = 8U;   ///< 帧 C 数据长度 [byte]
constexpr std::size_t kPayloadSizeD = 8U;   ///< 帧 D 数据长度 [byte]
constexpr std::size_t kPayloadSizeE = 8U;   ///< 帧 E 数据长度 [byte]
}  // namespace remote_control_can_bridge

}  // namespace common

// ══════════════════════════════════════════════════════════════════════════════
// Hero（变体 1）专用参数
// ══════════════════════════════════════════════════════════════════════════════
namespace hero {
using namespace common;

inline YawSpeedFeedforward yaw_ff{0.002f, 0.5f};

// ── CAN 总线 ID 与电机配置 ──
namespace globals {
using namespace common::globals;

constexpr std::uint16_t kLeftWheelId = 0x05;   ///< 左轮毂电机 CAN ID
constexpr std::uint16_t kRightWheelId = 0x02;  ///< 右轮毂电机 CAN ID

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
}  // namespace globals

// ── 云台 ──
namespace gimbal {
using namespace common::gimbal;

const DmMitSettings kPitchMotorSettings{0x17, 0x07, 12.5, 10.f, 28.f, {0.f, 500.f}, {0.f, 5.f}};

constexpr float kPitchMinRad = -0.2f;  ///< 俯仰角下限 [rad]
constexpr float kPitchMaxRad = 0.7f;   ///< 俯仰角上限 [rad]
constexpr float kPitchGravityCompensationNm = 2.35f;

constexpr PidGains kYawPositionPid{25.0f, 0.0f, 0.3f, 1000.0f, 1.0f};    ///< 偏航位置 PID
constexpr PidGains kYawSpeedPid{1.5f, 0.0f, 0.0f, 10.0f, 0.4f};          ///< 偏航速度 PID
constexpr PidGains kPitchPositionPid{25.0f, 0.0f, 0.5f, 1000.0f, 0.4f};  ///< 俯仰位置 PID
constexpr PidGains kPitchSpeedPid{2.f, 0.0f, 0.0f, 28.0f, 0.0f};         ///< 俯仰速度 PID

/// @brief 辨识得到的 9 个动力学参数（theta_0 ~ theta_8），用于前馈验证
constexpr float kIdentTheta[9] = {
    // 0.f, 0.f,  0.03675504, -0.26935183*2.5  ,  0.26007541*2.5,
    //         0.69615303,  0.51871814,  0.69915412,  0.0656076
    // 0, 0, 0, -0.2915259f, 0.1695807f, 0, 0, 0, 0
    0, 0, 0, -0.164315f, 0.085155, 0, 0, 0.f, 1.014933f};

/// @brief Pitch 力矩前馈常值偏置  [Nm]
constexpr float kPitchFeedforwardBiasNm = 0.3811f;
}  // namespace gimbal

// ── 云台辨识 ──
namespace gimbal_ident {
using namespace common::gimbal_ident_common;
/// @brief 当前激活的辨识子模式（编译时选择）
static constexpr IdentSubMode kIdentSubMode = IdentSubMode::kPitchInertia;
constexpr float kBaseFreqHz = 0.3f;                                                 ///< 辨识轨迹基频 [Hz]
constexpr float kDmTorqueLimitNm = 10.0f;                                           ///< DM 电机力矩上限 [Nm]
constexpr float kDefaultDtS = 0.002f;                                               ///< 辨识控制周期 [s]
constexpr float kYawAmp[kHarmonicCount] = {0.35f, -0.2f, 0.13f, -0.12f, 0.04f};     ///< yaw 轴五次谐波幅值 [rad]
constexpr float kPitchAmp[kHarmonicCount] = {0.27f, -0.14f, 0.09f, -0.05f, 0.03f};  ///< pitch 轴五次谐波幅值 [rad]
constexpr float kPitchPhase[kHarmonicCount] = {};                      ///< pitch 轴五次谐波相位 [rad]
constexpr PidGains kIdentYawPosPid{20.0f, 0.0f, 0.1f, 10.0f, 0.0f};    ///< 辨识模式 yaw 位置 PID
constexpr PidGains kIdentPitchPosPid{40.0f, 2.0f, 1.5f, 10.0f, 5.0f};  ///< 辨识模式 pitch 位置 PID
constexpr PidGains kIdentYawVelPid{1.5f, 10.0f, 0.f, 10.0f, 2.0f};     ///< Friction step yaw 速度 PID
constexpr PidGains kIdentPitchVelPid{2.f, 10.0f, 0.f, 10.0f, 10.0f};   ///< 兼容保留，pitch 摩擦现用位置 PID
constexpr float kIdentPitchCenter = -0.505264f;  ///< 辨识轨迹 pitch 中心角 [rad]（机械中位，实际需根据云台标定）
constexpr float kIdentPitchTopLimit = -0.80f;     ///< 辨识轨迹 pitch 下限 [rad]
constexpr float kIdentPitchBottomLimit = 0.215f;  ///< 辨识轨迹 pitch 上限 [rad]

// ── 分步辨识参数 ──
/// @brief 重力模式: Pitch 目标角度序列 [rad]
constexpr float kGravityPitchAngles[] = {-0.8f, -0.7f, -0.6f, -0.5f, -0.4f, -0.3f, -0.2f, -0.1f, -0.0f, 0.10f, 0.2f};
constexpr size_t kGravityAngleCount = 11;
constexpr float kGravityHoldDuration = 3.0f;    ///< 每角度测量时间 [s]
constexpr float kGravitySettleDuration = 1.5f;  ///< 到位稳定时间 [s]

/// @brief 摩擦模式: Pitch 位置目标匀速积分，Yaw 速度环匀速转一圈 (2π)
constexpr float kFrictionYawVelocitiesRadS[] = {0.05f, 0.10f, 0.20f, 0.30f};
constexpr size_t kFrictionYawVelocityCount = sizeof(kFrictionYawVelocitiesRadS) / sizeof(kFrictionYawVelocitiesRadS[0]);
constexpr float kFrictionYawTravelRad = 6.28f;  ///< 每个正/反匀速段的累计转角 [rad] (≈2π, 接近 360°)
constexpr float kFrictionPitchVelocitiesRadS[] = {0.05f, 0.10f, 0.20f, 0.30f};
constexpr size_t kFrictionPitchVelocityCount =
    sizeof(kFrictionPitchVelocitiesRadS) / sizeof(kFrictionPitchVelocitiesRadS[0]);
constexpr float kFrictionPitchMoveVelocityRadS = 0.30f;  ///< 非采样过渡段速度 [rad/s]
constexpr float kFrictionPauseDuration = 0.5f;           ///< 段间停顿 [s]

/// @brief Pitch 惯量模式: 正弦参数
constexpr float kPitchInertiaFrequenciesHz[] = {2.0f, 3.0f, 4.0f};
constexpr size_t kPitchInertiaFrequencyCount =
    sizeof(kPitchInertiaFrequenciesHz) / sizeof(kPitchInertiaFrequenciesHz[0]);
constexpr float kPitchInertiaAmplitude = 0.2f;
constexpr uint16_t kPitchInertiaWarmupCycles = 3;
constexpr uint16_t kPitchInertiaRecordCycles = 8;
constexpr float kPitchInertiaPrepareVelocityRadS = 0.30f;
constexpr float kPitchInertiaPrepareDurationS = 0.5f;

/// @brief 耦合惯量模式: Pitch 固定角度序列 [rad]
constexpr float kCouplingPitchAngles[] = {0.75f, 1.00f, 1.25f, 1.50f};
constexpr size_t kCouplingAngleCount = 4;
constexpr float kCouplingFreqHz = 1.0f;            ///< Yaw 正弦频率 [Hz]
constexpr float kCouplingAmplitude = 2.5f;         ///< Yaw 正弦幅值 [rad]
constexpr float kCouplingDurationPerAngle = 8.0f;  ///< 每角度持续时间 [s] (2 warmup + 3 record)
constexpr uint16_t kCouplingWarmupCycles = 2;
constexpr uint16_t kCouplingRecordCycles = 3;
}  // namespace gimbal_ident

// ── 发射机构（Hero：DM 拨盘）──
namespace shoot {
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

inline constexpr float kBoosterZeroPointRad = -0.75f;                        ///< 拨盘零位角度 [rad]
inline constexpr float kSegmentAngleRad = kPi / 3.f;                         ///< 拨盘分段角度 [rad]
inline constexpr uint16_t kInitDelayTicks = 600;                             ///< 初始化延迟周期数
inline constexpr uint16_t kShootDelayTicks = 360;                            ///< 发射延迟周期数
inline constexpr float kStallThresholdRad = kPi / 18.f;                      ///< 堵转判定角度阈值 [rad]
inline constexpr float kStallFallbackRad = kPi / 90.f;                       ///< 堵转回退角度 [rad]
inline constexpr int16_t kFireDialThreshold = -100;                          ///< 发射触发拨轮阈值
inline constexpr PidGains kBoosterPositionPid{60.f, 0.f, 560.f, 24.f, 0.f};  ///< 拨盘位置 PID
inline constexpr PidGains kBoosterSpeedPid{0.3f, 0.f, 0.02f, 6.4f, 0.f};     ///< 拨盘速度 PID
inline constexpr uint16_t kBoosterMasterId = 0x10;                           ///< 拨盘主电机 CAN ID
inline constexpr uint16_t kBoosterSlaveId = 0x09;                            ///< 拨盘从电机 CAN ID
inline const DmMitSettings kBoosterDmSettings{0x10, 0x09, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
}  // namespace shoot

// ── 底盘状态机 ──
namespace chassis_fsm {

constexpr float kStairDescendLegLengthM = 0.16f;
constexpr float kStairDescendThetaBTriggerRad = 0.18f;

constexpr StairClimbParams kStairClimb{
    .high_leg_length_m = 0.37f,
    .hook_leg_length_m = 0.37f,
    .retract_leg_length_m = 0.1f,
    .settle_leg_length_m = 0.1f,
    .contact_theta_threshold_rad = 0.50f,
    .hook_theta_target_rad = 1.f,
    .retract_theta_target_rad = 1.2f,
    .retract_theta_tolerance_rad = 0.6f,
    .hook_theta_tolerance_rad = 0.3f,
    .leg_length_tolerance_m = 0.08f,
    .settle_theta_tolerance_rad = 0.5f,
    .settle_theta_target_rad = -0.f,
    .settle_theta_ramp_step_rad = 0.02f,
    .hook_stable_ms = 10U,
    .retract_stable_ms = 20U,
    .settle_stable_ms = 100U,
    .hook_timeout_ms = 1000U,
    .retract_timeout_ms = 1000U,
    .settle_timeout_ms = 1000U,
};
constexpr StairClimbParams kStairClimbStep2{
    .high_leg_length_m = 0.34f,
    .hook_leg_length_m = 0.34f,
    .retract_leg_length_m = 0.1f,
    .settle_leg_length_m = 0.1f,
    .contact_theta_threshold_rad = 0.50f,
    .hook_theta_target_rad = 1.2f,
    .retract_theta_target_rad = 1.25f,
    .retract_theta_tolerance_rad = 0.6f,
    .hook_theta_tolerance_rad = 0.3f,
    .leg_length_tolerance_m = 0.08f,
    .settle_theta_tolerance_rad = 0.4f,
    .settle_theta_target_rad = -0.2f,
    .settle_theta_ramp_step_rad = 0.02f,
    .hook_stable_ms = 10U,
    .retract_stable_ms = 20U,
    .settle_stable_ms = 50U,
    .hook_timeout_ms = 2000U,
    .retract_timeout_ms = 500U,
    .settle_timeout_ms = 500U,
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
constexpr std::uint32_t kJumpPushReachedHoldMs = 5U;  ///< 蹬伸腿长到位后持续满足此时间才切换 [ms]

// 自动跳跃已移除

// ==== 基本运动（腿长档位）====
constexpr float kLowLegLengthM = 0.14f;      ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.21f;      ///< 中腿长档位目标腿长 [m]
constexpr float kHighLegLengthM = 0.37f;     ///< 高腿长档位目标腿长 [m]
constexpr float kLegLengthRampTimeS = 0.3f;  ///< 腿长切换斜坡时间 [s]（从低到高腿长的过渡时间）
constexpr std::uint32_t kSpinExitTimeoutMs = 3000U;  ///< 小陀螺预测退出超时兜底 [ms]
constexpr float kSpinEntrySpeedThresholdMps = 0.3f;  ///< 进入小陀螺的速度阈值 [m/s]
}  // namespace chassis_fsm

// ── 底盘控制 ──
namespace chassis {

// ==== 物理/机械参数 ====
constexpr float kControlDtS = 0.002f;  ///< 底盘控制周期 [s]（500Hz）
constexpr float kLegL1M = 0.215f;      ///< 五连杆主动杆长度 [m]
constexpr float kLegL2M = 0.254f;      ///< 五连杆从动杆长度 [m]

// -- 左腿弹簧补偿三次多项式系数：tau = c0 + c1*l + c2*l^2 + c3*l^3 --
constexpr float kLeftSpringC0 = -10.7181f;
constexpr float kLeftSpringC1 = -663.9702f;
constexpr float kLeftSpringC2 = 1070.0496f;
constexpr float kLeftSpringC3 = -400.8656f;
// -- 右腿弹簧补偿三次多项式系数 --
constexpr float kRightSpringC0 = -10.7181f;
constexpr float kRightSpringC1 = -663.9702f;
constexpr float kRightSpringC2 = 1070.0496f;
constexpr float kRightSpringC3 = -400.8656f;

// -- 质量/惯量/重力 --
constexpr float kLegMassKg = 2.3f;     ///< 单条腿质量 [kg]
constexpr float kGravityMps2 = 9.81f;  ///< 重力加速度 [m/s²]

// -- 轮参数 --
constexpr float kWheelRadiusM = 0.24f;  ///< 驱动轮轮距的一半（R_l） [m]

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
constexpr float kStandupThetaThresholdRad = 1.1f;  ///< 起立完成判定：双腿摆角绝对值低于此值后允许轮端输出 [rad]
constexpr float kStandupPhase0ThetaTargetRad = 1.2f;  ///< 起立 Phase 0/1 腿摆角目标 [rad]
constexpr float kStandupPhase1TargetLengthM = 0.1f;   ///< 起立 Phase 1 目标腿长 [m]
constexpr float kStandupPhase1ThetaTolRad = 0.6f;  ///< 起立 Phase 1 完成判定：摆角与目标差值容许 [rad]
constexpr float kStandupPhase1ThetaTolStairRad = 1.1f;  ///< 台阶 step2 触发起立 Phase 1 容许 [rad]
constexpr float kStandupThetaRampStepRad = 0.02f;       ///< 起立摆角斜坡步长 [rad/周期]
constexpr float kPostureRollMinRad = -0.3f;             ///< 横滚角安全下限 [rad]
constexpr float kPostureRollMaxRad = 0.3f;              ///< 横滚角安全上限 [rad]
constexpr float kPostureThetaBMinRad = -0.58f;          ///< 机体俯仰角安全下限 [rad]
constexpr float kPostureThetaBMaxRad = 0.5f;            ///< 机体俯仰角安全上限 [rad]
constexpr float kPostureThetaLegMinRad = -0.9f;         ///< 腿摆角安全下限 [rad]
constexpr float kPostureThetaLegMaxRad = 2.f;           ///< 腿摆角安全上限 [rad]
constexpr float kLegRecoverThetaDotTarget = -1.2f;  ///< 倒地恢复时腿摆角速度目标 [rad/s]（负号表示前摆方向）
constexpr float kLegRecoverThetaDotRampStep = 0.06f;  ///< 倒地恢复腿摆角速度斜坡步长 [(rad/s)/周期]
constexpr float kManualRecoveryLegSpeedRadS = 1.2f;   ///< 手动倒地恢复腿摆角速度 [rad/s]
constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;   ///< 倒地恢复零力矩区间下限 [rad]
constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;   ///< 倒地恢复零力矩区间上限 [rad]

// ==== 倒地恢复软着陆 ====
constexpr float kRecoveryDecelZoneRad = 0.6f;   ///< 恢复减速区宽度 [rad]（接近目标边界时开始减速）
constexpr float kRecoveryMinSpeedRadS = 0.08f;  ///< 恢复减速区边界最低速度 [rad/s]
constexpr float kRecoveryGravityRampScale = 0.35f;    ///< 恢复时重力补偿斜坡比例（越大身体越不砸）
constexpr float kRecoveryPitchFeedforwardKp = 23.0f;  ///< 倒地恢复俯仰角前馈系数
constexpr float kRecoveryRollFeedforwardKp = 6.0f;    ///< 倒地恢复横滚角前馈系数

// ==== pitch 恢复刹车 ====
constexpr float kPitchBrakeZoneRad = 0.55f;           ///< pitch 恢复刹车区间 [rad]
constexpr float kPitchBrakeRateStartRadS = 0.6f;      ///< pitch 刹车起始角速度 [rad/s]
constexpr float kPitchBrakeRateFullRadS = 2.0f;       ///< pitch 刹车满力度角速度 [rad/s]
constexpr float kPitchBrakeMinScale = 0.25f;          ///< pitch 刹车最小缩放系数
constexpr float kPitchBrakeReverseRateRadS = 2.6f;    ///< pitch 反转角速度阈值 [rad/s]
constexpr float kPitchBrakeReverseSpeedRadS = 0.35f;  ///< pitch 反转目标速度 [rad/s]

// ==== 倒地恢复腿摆角目标范围 ====
constexpr float kRecoveryThetaRangeLowMin = -4.0f;   ///< 前倒恢复腿摆角下限 [rad]
constexpr float kRecoveryThetaRangeLowMax = -3.5f;   ///< 前倒恢复腿摆角上限 [rad]
constexpr float kRecoveryThetaRangeHighMin = -2.2f;  ///< 后倒恢复腿摆角下限 [rad]
constexpr float kRecoveryThetaRangeHighMax = -1.8f;  ///< 后倒恢复腿摆角上限 [rad]

// -- 离地检测 --
constexpr float kOffGroundSupportForceThresholdN = 10.0f;  ///< 支撑力低于此值判定为离地 [N]
constexpr float kOffGroundSupportForceClampN = 20.0f;      ///< 离地时支持力限幅值 [N]

// -- 中腿长下压 --
constexpr float kMidLegDipTriggerLengthM = 0.26f;  ///< 中腿长模式下触发下压的腿长阈值 [m]
constexpr float kMidLegDipTargetLengthM = 0.21f;   ///< 下压目标腿长 [m]
constexpr uint16_t kMidLegDipHoldTicks = 500;      ///< 下压维持时间 [ticks @ 500Hz = 1s]

// -- 上台阶退勾起立 --
constexpr uint16_t kStairExitStandupHookDelayTicks = 0;  ///< 上台阶退出后延迟触发起立三段式 [ticks @ 500Hz]，0=立即触发

// ==== 物理参数（变体专属）====
constexpr float kBodyMassKg = 27.0f;  ///< 机体质量 [kg]

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.03f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵 — 40 组多项式系数）====
/// 由 MATLAB 离线拟合得到，p(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l² + p11*l_l*l_r + p02*l_r²
/// 共 40 行，对应 4×10 增益矩阵 K 的 40 个元素（按行主序展平）
/// 每行 6 个系数：[p00, p10, p01, p20, p11, p02]
static constexpr const auto &kCtrlPLow = ::wheel_legged::params::generated::kCtrlP;

// ==== 基本运动（PID 增益）====
constexpr PidGains kLeftL0Pid{3000.0f, 0.1f, 130.0f, 170.0f, 10.0f};   ///< 左腿腿长 PID（常规）
constexpr PidGains kRightL0Pid{3000.0f, 0.1f, 130.0f, 170.0f, 10.0f};  ///< 右腿腿长 PID（常规）

constexpr PidGains kRollPid{1000.0f, 0.1f, 20.0f, 140.0f, 40.0f};  ///< 横滚平衡 PID

// ==== 跳跃（PID 增益）====
constexpr float kJumpPushForceN = 0.0f;                                      ///< 蹬伸阶段单腿基础支撑力 [N]
constexpr PidGains kLeftL0PidJumpOne{3500.0f, 0.f, 100.0f, 120.0f, 0.0f};    ///< 左腿预备 PID（JumpPrep）
constexpr PidGains kRightL0PidJumpOne{3500.0f, 0.f, 100.0f, 120.0f, 0.0f};   ///< 右腿预备 PID（JumpPrep）
constexpr PidGains kLeftL0PidJumpTwo{4000.0f, 0.0f, 60.0f, 120.0f, 0.0f};    ///< 左腿蹬伸 PID（JumpPush）
constexpr PidGains kRightL0PidJumpTwo{4000.0f, 0.0f, 60.0f, 120.0f, 0.0f};   ///< 右腿蹬伸 PID（JumpPush）
constexpr PidGains kLeftL0PidJumpThree{4000.0f, 0.f, 60.0f, 120.0f, 0.0f};   ///< 左腿回收 PID（JumpRecover）
constexpr PidGains kRightL0PidJumpThree{4000.0f, 0.f, 60.0f, 120.0f, 0.0f};  ///< 右腿回收 PID（JumpRecover）
constexpr PidGains kLeftLegAnglePidJumpRetract2{15.0f, 0.0f, 0.0f, 20.0f, 0.0f};  ///< 左腿摆角 PID（跳跃收腿第二阶段）
constexpr PidGains kRightLegAnglePidJumpRetract2{15.0f, 0.0f, 0.0f, 20.0f, 0.0f};  ///< 右腿摆角 PID（跳跃收腿第二阶段）

// ==== 中腿长下压（PID 增益）====
constexpr PidGains kLeftL0PidDip{500.0f, 0.f, 130.0f, 180.0f, 0.0f};   ///< 左腿下压腿长 PID
constexpr PidGains kRightL0PidDip{500.0f, 0.f, 130.0f, 180.0f, 0.0f};  ///< 右腿下压腿长 PID
// constexpr PidGains kLeftL0PidDip{1000.0f, 0.1f, 130.0f, 120.0f, 10.0f};   ///< 左腿腿长 PID（常规）
// constexpr PidGains kRightL0PidDip{1000.0f, 0.1f, 130.0f, 120.0f, 10.0f};  ///< 右腿腿长 PID（常规）

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{30.0f, 0.f, 60.0f, 15.0f, 9.0f};   ///< 左腿摆角速度 PID（倒地恢复用）
constexpr PidGains kRightLegTurnPid{30.0f, 0.f, 60.0f, 15.0f, 9.0f};  ///< 右腿摆角速度 PID（倒地恢复用）
constexpr PidGains kLeftLegAnglePidStandup{15.0f, 0.0f, 0.0f, 20.0f, 0.0f};   ///< 左腿摆角 PID（起立用）
constexpr PidGains kRightLegAnglePidStandup{15.0f, 0.0f, 0.0f, 20.0f, 0.0f};  ///< 右腿摆角 PID（起立用）
constexpr PidGains kLeftLegTurnPidManual{30.0f, 0.f, 60.0f, 25.0f, 9.0f};  ///< 左腿摆角速度 PID（手动倒地恢复）
constexpr PidGains kRightLegTurnPidManual{30.0f, 0.f, 60.0f, 25.0f, 9.0f};  ///< 右腿摆角速度 PID（手动倒地恢复）

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
// 自动跳跃触发阈值已移除

constexpr float kControlLoopDtS = 0.002f;  ///< 控制环周期 [s]

// -- 摇杆/键鼠输入归一化 --
constexpr std::int16_t kDr16AxisMaxAbs = 660;       ///< DR16 摇杆轴最大绝对值（用于归一化到 [-1,1]）
constexpr float kRcStickMax = 660.0f;               ///< RC 摇杆最大值（用于积分目标速率计算）
constexpr float kTcMouseMax = 200.0f;               ///< 图传鼠标增量最大值（用于积分目标速率计算）
constexpr float kRcYawRateMaxRadS = -4.5f;          ///< RC 摇杆满偏时偏航积分速率 [rad/s]
constexpr float kRcPitchRateMaxRadS = 1.5f;         ///< RC 摇杆满偏时俯仰积分速率 [rad/s]
constexpr float kTcMouseYawRateMaxRadS = -6.0f;     ///< 图传鼠标满偏时偏航积分速率 [rad/s]
constexpr float kTcMousePitchRateMaxRadS = 3.0f;    ///< 图传鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kDr16MouseMax = 1600.0f;            ///< DR16 鼠标增量最大值（用于积分目标速率计算）
constexpr float kDr16MouseYawRateMaxRadS = -6.0f;   ///< DR16 鼠标满偏时偏航积分速率 [rad/s]
constexpr float kDr16MousePitchRateMaxRadS = 3.0f;  ///< DR16 鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kPitchTargetMinRad = -0.2f;         ///< RC 积分俯仰目标下限 [rad]
constexpr float kPitchTargetMaxRad = 0.7f;          ///< RC 积分俯仰目标上限 [rad]
constexpr float kKeyboardAccelRampStep = 0.008f;    ///< 键盘 WASD 加速斜坡步进（每周期，0→1 约 0.5s）
constexpr float kKeyboardBrakeRampStep = 0.008f;    ///< 键盘 WASD 减速斜坡步进（每周期，1→0 约 0.25s）

// -- 云台启动归中判稳 --
constexpr float kGimbalStartupYawAlignErrorRad = 0.04f;           ///< 归中完成位置误差阈值 [rad]
constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;            ///< 归中完成速度阈值 [rad/s]
constexpr std::uint32_t kGimbalStartupYawAlignStableTicks = 50U;  ///< 归中判稳所需连续满足周期数

// -- 偏航就绪判稳（允许底盘纵向驱动前确认偏航跟踪到位）--
constexpr float kYawFollowDriveReadyErrorRad = 0.08f;           ///< 偏航就绪位置误差阈值 [rad]
constexpr float kYawFollowDriveReadyVelRadS = 0.45f;            ///< 偏航就绪速度阈值 [rad/s]
constexpr std::uint32_t kYawFollowDriveReadyStableTicks = 20U;  ///< 偏航就绪判稳所需连续周期数

// ==== 基本运动 ====
constexpr float kTargetForwardSpeedMaxMps = 2.1f;         ///< 最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.2f;  ///< 高腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxMidLegMps = 1.7f;   ///< F键中腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxNoScMps = 1.1f;     ///< 无超电最大前进速度 [m/s]
constexpr float kTargetSpeedBiasLowLegMps = 0.0f;         ///< 低腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegMps = 0.f;          ///< C键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegFMps = 0.0f;        ///< F键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasHighLegMps = 0.0f;        ///< 高腿长目标速度偏置 [m/s]
constexpr float kMaxSafeYawRateRadS = 3.5f;               ///< 摩擦圆最大安全偏航速率 [rad/s]
constexpr float kLargeTurnThresholdRad = 0.5f;  ///< 大转向检测阈值：motor_error 超此值触发先减速再转向 [rad]
constexpr float kSafeTurnSpeedMps = 0.5f;  ///< 大转向安全速度上限：超此速度时强制限速 [m/s]
constexpr float kLargeTurnThetaThresholdRad = 0.1f;  ///< 大转向腿摆角阈值：摆角超此值需先回正再转向 [rad]
constexpr float kLargeTurnRecoveryAccelScale = 0.6f;  ///< 大转向恢复加速缩放：限速解除后加速斜坡乘以该系数，越小越缓
constexpr float kVxInputDeadbandNorm = 0.1f;         ///< 前进输入死区（归一化值，低于此忽略）
constexpr float kVyInputDeadbandNorm = 0.1f;         ///< 平移输入死区（归一化值）
constexpr float kYawFollowRampStepRadS = 0.3f;       ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kYawFollowRampStepRadNoScS = 0.15f;  ///< 偏航跟随角速度斜坡步长（无超电）[(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.15f;  ///< 位置锚定冻结速度阈值 [m/s]（车速低于此值时锁定位置）

constexpr uint32_t kPositionHoldTimeoutTicks =
    1000U;  ///< 位置锚定超时 [ticks]（斜坡归零后最多等待此周期数，超时强制冻结）
constexpr float kPositionErrorScaleLowLeg = 1.0f;      ///< 低腿长位置误差缩放
constexpr float kVelocityErrorScaleLowLeg = 1.0f;      ///< 低腿长速度误差缩放
constexpr float kPositionErrorScaleMidLeg = 1.0f;      ///< 中腿长位置误差缩放
constexpr float kVelocityErrorScaleMidLeg = 1.0f;      ///< 中腿长速度误差缩放
constexpr float kPositionErrorScaleHighLeg = 1.0f;     ///< 高腿长位置误差缩放
constexpr float kVelocityErrorScaleHighLeg = 1.0f;     ///< 高腿长速度误差缩放
constexpr float kYawFollowFixedTargetRad = 0.5739f;    ///< 偏航跟随固定目标偏置角 [rad]（前进方向）
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;  ///< 偏航跟随侧向目标偏置角 [rad]（±π/2）

// ==== 期望状态偏置（腿摆角/机体俯仰）====
constexpr float kExpectedThetaLlBiasRadLowLeg = 0.f;  ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = 0.f;  ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = -0.108f;     ///< 期望机体俯仰偏置 [rad]

constexpr float kExpectedDisplacementBiasM = 0.0f;         ///< 期望位移偏置 [m]
constexpr float kExpectedDisplacementBiasMLowLeg = -0.3f;  ///< 低腿长期望位移偏置 [m]（满弹量时）
constexpr int kInitialAmmoCount = 60;                      ///< 初始弹量 [发]
constexpr float kDisplacementBiasPerShot = 1.f / 120.f;    ///< 每发弹的位移偏置变化量 [m/发]（待标定）

// ==== 速度斜坡参数（按腿长档位分级）====
constexpr SdotRampParams kSdotRampLowLeg{0.007f, 0.007f};     ///< 低腿长速度斜坡（加速/制动步长）
constexpr SdotRampParams kSdotRampMidLeg{0.0040f, 0.006f};    ///< 中腿长速度斜坡(C 键触发)
constexpr SdotRampParams kSdotRampMidLegF{0.0055f, 0.0055f};  ///< 中腿长速度斜坡（G 键触发）
constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};    ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.15f;           ///< 小陀螺进入偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinExitYawRampStepRadS = 0.15f;       ///< 小陀螺退出偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS1 = 7.5f;          ///< 小陀螺目标自旋角速度 [rad/s] ≤55W
constexpr float kSpinTargetYawDotRadS2 = 7.5f;          ///< 小陀螺目标自旋角速度 [rad/s] 55-65W
constexpr float kSpinTargetYawDotRadS3 = 7.5f;          ///< 小陀螺目标自旋角速度 [rad/s] 65-75W
constexpr float kSpinTargetYawDotRadS4 = 7.5f;          ///< 小陀螺目标自旋角速度 [rad/s] >75W
constexpr float kSpinTargetYawDotRadNoScS1 = 1.0f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] ≤55W
constexpr float kSpinTargetYawDotRadNoScS2 = 1.0f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] 55-65W
constexpr float kSpinTargetYawDotRadNoScS3 = 1.0f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] 65-75W
constexpr float kSpinTargetYawDotRadNoScS4 = 2.0f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] >75W
constexpr float kSpinExitYawAlignThresholdRad = 0.15f;  ///< 小陀螺预测退出：yaw 对齐阈值 [rad]
constexpr float kSpinTranslationGain = 0.3f;  ///< 小陀螺平移增益（将云台系前进指令投影到车体系的比例）
constexpr float kSpinThetaLlBiasRad = 0.02f;   ///< 小陀螺时左腿摆角偏置 [rad]
constexpr float kSpinThetaLrBiasRad = -0.05f;  ///< 小陀螺时右腿摆角偏置 [rad]
constexpr float kSpinLegLengthBiasM = -0.0f;   ///< 小陀螺时腿长偏差（左+右-）[m]
constexpr float kSpinThetaBBiasRad = -0.1f;    ///< 小陀螺时俯仰目标偏置 [rad]

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
constexpr float kWheelSpeedFilterCutoffHz = 5.0f;       ///< 轮速低通滤波截止频率 [Hz]
constexpr float kWheelSpeedFilterSampleHz = 500.0f;     ///< 轮速低通滤波采样频率 [Hz]

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
constexpr float kLeftPhi1OffsetRad = 0.f;              ///< 左腿前关节零位偏移 [rad]
constexpr float kLeftPhi4OffsetRad = 2.223f;           ///< 左腿后关节零位偏移 [rad]
constexpr float kRightPhi1OffsetRad = -0.257f + M_PI;  ///< 右腿前关节零位偏移 [rad]
constexpr float kRightPhi4OffsetRad = 2.199f;          ///< 右腿后关节零位偏移 [rad]
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
constexpr uint8_t kRobotId = 1U;                                           ///< 机器人 ID（裁判系统回退值）
constexpr float kBulletSpeedMps = 11.5f;                                   ///< 弹速 [m/s]（裁判系统回退值）
constexpr float kBulletDefaultSpeedMps = 11.5f;                            ///< 默认弹速
constexpr float kBulletBoundarySpeedMps = 10.5f;                           ///< 区分裁判系统返回值是否正确
constexpr PidGains kYawPositionPidRune{25.0f, 0.0f, 0.3f, 1000.0f, 1.0f};  ///< 自瞄偏航位置 PID（打符）
constexpr PidGains kYawSpeedPidRune{0.8f, 0.f, 0.0f, 6.4f, 0.6f};          ///< 自瞄偏航速度 PID（打符）
constexpr PidGains kPitchPositionPidRune{55, 0.f, 1.5f, 8.0f, 4.f};        ///< 自瞄俯仰位置 PID（打符）
constexpr PidGains kPitchSpeedPidRune{0.8f, 0.0f, 0.0f, 6.4f, 0.4f};       ///< 自瞄俯仰速度 PID（打符）
constexpr PidGains kYawPositionPid{25.0f, 0.0f, 0.3f, 1000.0f, 1.0f};      ///< 自瞄偏航位置 PID
constexpr PidGains kYawSpeedPid{0.8f, 0.f, 0.0f, 6.4f, 0.6f};              ///< 自瞄偏航速度 PID
constexpr PidGains kPitchPositionPid{55, 0.f, 1.5f, 8.0f, 4.f};            ///< 自瞄俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.8f, 0.0f, 0.0f, 6.4f, 0.4f};           ///< 自瞄俯仰速度 PID
}  // namespace aimbot

// ── 自瞄 + 小陀螺模式 PID ──
namespace aimbot_spin {
constexpr PidGains kYawPositionPid{80.0f, 0.2, 3.f, 1000.0f, 4.0f};  ///< 自瞄+小陀螺偏航位置 PID
constexpr PidGains kYawSpeedPid{0.7f, 0.0f, 0.0f, 10.0f, 0.3f};      ///< 自瞄+小陀螺偏航速度 PID
constexpr PidGains kPitchPositionPid{80, 0.2f, 3.0f, 1000.0f, 4.f};  ///< 自瞄+小陀螺俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.7f, 0.0f, 0.0f, 9.0f, 0.4f};     ///< 自瞄+小陀螺俯仰速度 PID

// ── 自旋偏航目标偏置（补偿小陀螺自旋时的角度滞后）──
constexpr float kYawTargetBiasSpeedThresholds[3] = {8.0f, 9.5f, 10.5f};  ///< 四档偏置的自旋速度分界 [rad/s]
constexpr float kYawTargetBiasRad[4] = {0.0f, 0.0f, 0.0f, 0.0f};         ///< 各档位偏航目标偏置 [rad]
constexpr float kYawSpeedFeedforwardRadS[4] = {0.0f, 0.0f, 0.0f, 0.0f};  ///< 自瞄小陀螺各挡 yaw 速度前馈 [rad/s]
}  // namespace aimbot_spin

}  // namespace hero

// ══════════════════════════════════════════════════════════════════════════════
// Infantry3（变体 2，默认）专用参数
// ══════════════════════════════════════════════════════════════════════════════
namespace infantry3 {
using namespace common;

// ── CAN 总线 ID 与电机配置 ──
namespace globals {
using namespace common::globals;

constexpr std::uint16_t kLeftWheelId = 0x01;   ///< 左轮毂电机 CAN ID
constexpr std::uint16_t kRightWheelId = 0x02;  ///< 右轮毂电机 CAN ID

constexpr std::uint16_t kDialId = 0x03;  ///< 拨盘电机 CAN ID
}  // namespace globals

// ── 云台 ──
namespace gimbal {
using namespace common::gimbal;

const DmMitSettings kPitchMotorSettings{0x17, 0x07, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};

constexpr float kPitchMinRad = -0.5f;  ///< 俯仰角下限 [rad]
constexpr float kPitchMaxRad = 0.6f;   ///< 俯仰角上限 [rad]

inline constexpr PidGains kYawPositionPid{30.0f, 0.f, 0.7f, 10.0f, 0.0f};    ///< 偏航位置 PID
inline constexpr PidGains kYawSpeedPid{0.5f, 0.0f, 0.0f, 6.0f, 0.4f};        ///< 偏航速度 PID
inline constexpr PidGains kPitchPositionPid{20.0f, 0.f, 0.3f, 10.0f, 0.6f};  ///< 俯仰位置 PID
inline constexpr PidGains kPitchSpeedPid{0.5f, 0.0f, 0.f, 8.0f, 0.0f};       ///< 俯仰速度 PID

/// @brief 辨识得到的 9 个动力学参数（theta_0 ~ theta_8），用于前馈验证
// constexpr float kIdentTheta[9] = {
//     0.028104f,  // theta_0: I1zz_com
//     0.020778f,  // theta_1: I2xx_com
//     0.030588f,  // theta_2: I2yy_com
//     0.040213f,  // theta_3: m2*l2x 水平前向偏心
//     0.074719f,  // theta_4: m2*l2z 垂直上向偏心
//     0.032147f,  // theta_5: fv1  yaw 粘滞摩擦
//     0.207946f,  // theta_6: fc1  yaw 库仑摩擦
//     0.f,  // theta_7: fv2  pitch 粘滞摩擦
//     0.260744f,  // theta_8: fc2  pitch 库仑摩擦
// };

constexpr float kIdentTheta[9] = {
    0.028104f,  // theta_0: I1zz_com
    0.020778f,  // theta_1: I2xx_com
    0.030588f,  // theta_2: I2yy_com
    0.040213f,  // theta_3: m2*l2x 水平前向偏心
    0.074719f,  // theta_4: m2*l2z 垂直上向偏心
    0.f,        // theta_5: fv1  yaw 粘滞摩擦
    0.f,        // theta_6: fc1  yaw 库仑摩擦
    0.f,        // theta_7: fv2  pitch 粘滞摩擦
    0.f,        // theta_8: fc2  pitch 库仑摩擦
};

constexpr float kPitchFeedforwardBiasNm = 0.2374f;
// constexpr float kPitchFeedforwardBiasNm = 0.f;
}  // namespace gimbal

// ── 云台辨识 ──
namespace gimbal_ident {
using namespace common::gimbal_ident_common;
/// @brief 当前激活的辨识子模式（编译时选择）
static constexpr IdentSubMode kIdentSubMode = IdentSubMode::kHarmonic;
constexpr float kBaseFreqHz = 0.3f;                                                 ///< 辨识轨迹基频 [Hz]
constexpr float kDmTorqueLimitNm = 10.0f;                                           ///< DM 电机力矩上限 [Nm]
constexpr float kDefaultDtS = 0.002f;                                               ///< 辨识控制周期 [s]
constexpr float kYawAmp[kHarmonicCount] = {0.35f, -0.2f, 0.13f, -0.12f, 0.04f};     ///< yaw 轴五次谐波幅值 [rad]
constexpr float kPitchAmp[kHarmonicCount] = {0.27f, -0.14f, 0.09f, -0.05f, 0.03f};  ///< pitch 轴五次谐波幅值 [rad]
constexpr float kPitchPhase[kHarmonicCount] = {1.2177f, 0.4006f, -0.8970f, 0.1462f, -2.6391f};
constexpr PidGains kIdentYawPosPid{20.0f, 0.0f, 0.1f, 10.0f, 0.0f};  ///< 辨识模式 yaw 位置 PID
constexpr PidGains kIdentPitchPosPid{30.0f, 5.0f, 0.2f, 10.0f, 2.0f};  ///< 辨识模式 pitch 位置 PID (Ki 克服静摩擦)
constexpr PidGains kIdentYawVelPid{1.5f, 10.0f, 0.0f, 10.0f, 2.0f};  ///< Friction step yaw 速度 PID
constexpr PidGains kIdentPitchVelPid{1.1f, 4.0f, 0.0f, 5.0f, 2.0f};  ///< Friction step pitch 速度环 PID
constexpr float kIdentPitchCenter = 0.51;  ///< 辨识轨迹 pitch 中心角 [rad]（机械中位，实际需根据云台标定）
constexpr float kIdentPitchTopLimit = 0.1f;     ///< 辨识轨迹 pitch 上限 [rad]
constexpr float kIdentPitchBottomLimit = 1.1f;  ///< 辨识轨迹 pitch 下限 [rad]

// ── 分步辨识参数 ──
constexpr float kGravityPitchAngles[] = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.f, 1.1f};
constexpr size_t kGravityAngleCount = 11;
constexpr float kGravityHoldDuration = 3.0f;
constexpr float kGravitySettleDuration = 1.5f;

/// @brief 摩擦模式: Pitch 位置目标匀速积分，Yaw 速度环匀速转一圈 (2π)
constexpr float kFrictionYawVelocitiesRadS[] = {0.05f, 0.10f, 0.20f, 0.30f};
constexpr size_t kFrictionYawVelocityCount = sizeof(kFrictionYawVelocitiesRadS) / sizeof(kFrictionYawVelocitiesRadS[0]);
constexpr float kFrictionYawTravelRad = 6.28f;  ///< 每个正/反匀速段的累计转角 [rad] (≈2π, 接近 360°)
constexpr float kFrictionPitchVelocitiesRadS[] = {0.05f, 0.10f, 0.20f, 0.30f};
constexpr size_t kFrictionPitchVelocityCount =
    sizeof(kFrictionPitchVelocitiesRadS) / sizeof(kFrictionPitchVelocitiesRadS[0]);
constexpr float kFrictionPitchMoveVelocityRadS = 0.30f;  ///< 非采样过渡段速度 [rad/s]
constexpr float kFrictionPauseDuration = 0.5f;           ///< 段间停顿 [s]

constexpr float kPitchInertiaFrequenciesHz[] = {2.0f, 3.0f, 4.0f};
constexpr size_t kPitchInertiaFrequencyCount =
    sizeof(kPitchInertiaFrequenciesHz) / sizeof(kPitchInertiaFrequenciesHz[0]);
constexpr float kPitchInertiaAmplitude = 0.2f;
constexpr uint16_t kPitchInertiaWarmupCycles = 3;
constexpr uint16_t kPitchInertiaRecordCycles = 8;
constexpr float kPitchInertiaPrepareVelocityRadS = 0.30f;
constexpr float kPitchInertiaPrepareDurationS = 0.5f;

constexpr float kCouplingPitchAngles[] = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.f, 1.1f};
constexpr size_t kCouplingAngleCount = 11;
constexpr float kCouplingFreqHz = 1.0f;
constexpr float kCouplingAmplitude = 2.5f;
constexpr float kCouplingDurationPerAngle = 8.0f;  // (2 warmup + 3 record) / 0.2 Hz
constexpr uint16_t kCouplingWarmupCycles = 2;
constexpr uint16_t kCouplingRecordCycles = 3;
}  // namespace gimbal_ident

// ── 发射机构（M3508 拨盘）──
namespace shoot {
constexpr PidGains kDialSpeedPid{10.0f, 0.f, 0.f, 16000.0f, 1000.0f};  ///< 拨盘速度 PID
constexpr PidGains kDialPositionPid{0.5f, 0.f, 0.0f, 16000.0f, 0.0f};  ///< 拨盘位置 PID
constexpr int16_t kDialFireThreshold = -600;                           ///< 发射触发拨轮阈值
constexpr float kFricSpeedTargetRpm = 6200.0f;  ///< 摩擦轮目标转速 [rpm]（打弹检测用）
constexpr PidGains kFricSpeedPid{10.0f, 0.0f, 0.0f, 16000.0f, 2000.0f};  ///< 摩擦轮速度 PID（Shoot控制器用）

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

constexpr float kStairDescendLegLengthM = 0.16f;
constexpr float kStairDescendThetaBTriggerRad = 0.18f;

constexpr StairClimbParams kStairClimb{
    .high_leg_length_m = 0.33f,
    .hook_leg_length_m = 0.33f,
    .retract_leg_length_m = 0.1f,
    .settle_leg_length_m = 0.1f,
    .contact_theta_threshold_rad = 0.50f,
    .hook_theta_target_rad = 1.f,
    .retract_theta_target_rad = 1.2f,
    .retract_theta_tolerance_rad = 0.5f,
    .hook_theta_tolerance_rad = 0.3f,
    .leg_length_tolerance_m = 0.05f,
    .settle_theta_tolerance_rad = 0.5f,
    .settle_theta_target_rad = 0.f,
    .settle_theta_ramp_step_rad = 0.05f,
    .hook_stable_ms = 10U,
    .retract_stable_ms = 20U,
    .settle_stable_ms = 50U,
    .hook_timeout_ms = 1000U,
    .retract_timeout_ms = 1000U,
    .settle_timeout_ms = 1000U,
};
constexpr StairClimbParams kStairClimbStep2{
    .high_leg_length_m = 0.29f,
    .hook_leg_length_m = 0.29f,
    .retract_leg_length_m = 0.1f,
    .settle_leg_length_m = 0.1f,
    .contact_theta_threshold_rad = 0.50f,
    .hook_theta_target_rad = 1.2f,
    .retract_theta_target_rad = 0.f,
    .retract_theta_tolerance_rad = 0.4f,
    .hook_theta_tolerance_rad = 0.8f,
    .leg_length_tolerance_m = 0.05f,
    .settle_theta_tolerance_rad = 0.4f,
    .settle_theta_target_rad = 0.f,
    .settle_theta_ramp_step_rad = 0.02f,
    .hook_stable_ms = 100U,
    .retract_stable_ms = 100U,
    .settle_stable_ms = 100U,
    .hook_timeout_ms = 1000U,
    .retract_timeout_ms = 1000U,
    .settle_timeout_ms = 1000U,
};

// ==== 倒地自启 ====
constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;        ///< 倒地确认时间 [ms]
constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;  ///< 自启超时 [ms]

// ==== 跳跃 ====
constexpr std::uint32_t kJumpLowPrepMs = 50U;       ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpLowPushMaxMs = 2500U;  ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpLowRecoverMs = 2000U;  ///< 跳跃回收阶段持续时间（保底超时）[ms]
constexpr std::uint32_t kJumpLowRecoverMinMs = 1500U;  ///< 跳跃回收阶段最低维持时间（此后开始判断离地）[ms]
constexpr float kJumpLowPrepLegLengthM = 0.14f;         ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpLowPushLegLengthM = 0.34f;         ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpLowRecoverLegLengthM = 0.128f;     ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpLowPushReachedLegLengthM = 0.34f;  ///< 蹬伸到位判定腿长 [m]
constexpr std::uint32_t kJumpPushReachedHoldMs = 3U;    ///< 蹬伸腿长到位后持续满足此时间才切换 [ms]

// ==== 基本运动（腿长档位）====
constexpr float kLowLegLengthM = 0.17f;              ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.21f;              ///< 中腿长档位目标腿长 [m]
constexpr float kHighLegLengthM = 0.3f;              ///< 高腿长档位目标腿长 [m]
constexpr float kLegLengthRampTimeS = 0.3f;          ///< 腿长切换斜坡时间 [s]
constexpr std::uint32_t kSpinExitTimeoutMs = 3000U;  ///< 小陀螺预测退出超时兜底 [ms]
constexpr float kSpinEntrySpeedThresholdMps = 0.3f;  ///< 进入小陀螺的速度阈值 [m/s]
}  // namespace chassis_fsm

// ── 底盘控制 ──
namespace chassis {

// ==== 物理/机械参数 ====
constexpr float kControlDtS = 0.002f;  ///< 底盘控制周期 [s]（500Hz）
constexpr float kLegL1M = 0.215f;      ///< 五连杆主动杆长度 [m]
constexpr float kLegL2M = 0.254f;      ///< 五连杆从动杆长度 [m]

// -- 左腿弹簧补偿三次多项式系数：tau = c0 + c1*l + c2*l^2 + c3*l^3 --
constexpr float kLeftSpringC0 = -124.177211f;
constexpr float kLeftSpringC1 = -80.971522f;
constexpr float kLeftSpringC2 = -943.224356f;
constexpr float kLeftSpringC3 = 2683.828696f;
// -- 右腿弹簧补偿三次多项式系数 --
constexpr float kRightSpringC0 = -124.177211f;
constexpr float kRightSpringC1 = -80.971522f;
constexpr float kRightSpringC2 = -943.224356f;
constexpr float kRightSpringC3 = 2683.828696f;

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
constexpr float kStandupThetaThresholdRad = 1.1f;  ///< 起立完成判定：双腿摆角绝对值低于此值后允许轮端输出 [rad]
constexpr float kStandupPhase0ThetaTargetRad = 1.5f;  ///< 起立 Phase 0/1 腿摆角目标 [rad]
constexpr float kStandupPhase1TargetLengthM = 0.1f;   ///< 起立 Phase 1 目标腿长 [m]
constexpr float kStandupPhase1ThetaTolRad = 0.6f;  ///< 起立 Phase 1 完成判定：摆角与目标差值容许 [rad]
constexpr float kStandupPhase1ThetaTolStairRad = 1.1f;  ///< 台阶 step2 触发起立 Phase 1 容许 [rad]
constexpr float kStandupThetaRampStepRad = 0.02f;       ///< 起立摆角斜坡步长 [rad/周期]
constexpr float kPostureRollMinRad = -1.f;              ///< 横滚角安全下限 [rad]
constexpr float kPostureRollMaxRad = 1.f;               ///< 横滚角安全上限 [rad]
constexpr float kPostureThetaBMinRad = -0.7f;           ///< 机体俯仰角安全下限 [rad]
constexpr float kPostureThetaBMaxRad = 0.7f;            ///< 机体俯仰角安全上限 [rad]
constexpr float kPostureThetaLegMinRad = -0.8f;         ///< 腿摆角安全下限 [rad]
constexpr float kPostureThetaLegMaxRad = 2.f;           ///< 腿摆角安全上限 [rad]
constexpr float kLegRecoverThetaDotTarget = -2.0f;      ///< 倒地恢复时腿摆角速度目标 [rad/s]
constexpr float kLegRecoverThetaDotRampStep = 0.008f;   ///< 倒地恢复腿摆角速度斜坡步长 [(rad/s)/周期]
constexpr float kManualRecoveryLegSpeedRadS = 0.5f;     ///< 手动倒地恢复腿摆角速度 [rad/s]
constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;     ///< 倒地恢复零力矩区间下限 [rad]
constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;     ///< 倒地恢复零力矩区间上限 [rad]
                                                        // ==== 倒地恢复软着陆 ====
constexpr float kRecoveryDecelZoneRad = 0.7f;   ///< 恢复减速区宽度 [rad]（接近目标边界时开始减速）
constexpr float kRecoveryMinSpeedRadS = 0.07f;  ///< 恢复减速区边界最低速度 [rad/s]
constexpr float kRecoveryGravityRampScale = 0.35f;   ///< 恢复时重力补偿斜坡比例（越大身体越不砸）
constexpr float kRecoveryPitchFeedforwardKp = 30.f;  ///< 倒地恢复俯仰角前馈系数
constexpr float kRecoveryRollFeedforwardKp = 5.0f;   ///< 倒地恢复横滚角前馈系数

// ==== pitch 恢复刹车 ====
constexpr float kPitchBrakeZoneRad = 0.55f;       ///< pitch 恢复刹车区间 [rad]
constexpr float kPitchBrakeRateStartRadS = 0.6f;  ///< pitch 刹车起始角速度 [rad/s]
constexpr float kPitchBrakeRateFullRadS = 2.0f;   ///< pitch 刹车满力度角速度 [rad/s]
constexpr float kPitchBrakeMinScale = 0.25f;      ///< pitch 刹车最小缩放系数
constexpr float kPitchBrakeReverseRateRadS = 2.6f;
///< pitch 反转角速度阈值 [rad/s]
constexpr float kPitchBrakeReverseSpeedRadS = 0.35f;  ///< pitch 反转目标速度 [rad/s]

// ==== 倒地恢复腿摆角目标范围 ====]
constexpr float kRecoveryThetaRangeLowMin = -4.7f;   ///< 前倒恢复腿摆角下限 [rad]
constexpr float kRecoveryThetaRangeLowMax = -4.3f;   ///< 前倒恢复腿摆角上限 [rad]
constexpr float kRecoveryThetaRangeHighMin = -2.1f;  ///< 后倒恢复腿摆角下限 [rad]
constexpr float kRecoveryThetaRangeHighMax = -1.7f;  ///< 后倒恢复腿摆角上限 [rad]

// -- 离地检测 --
constexpr float kOffGroundSupportForceThresholdN = 20.0f;  ///< 支撑力低于此值判定为离地 [N]
constexpr float kOffGroundSupportForceClampN = 100.0f;     ///< 离地时支持力限幅值 [N]

// -- 中腿长下压 --
constexpr float kMidLegDipTriggerLengthM = 0.27f;  ///< 中腿长模式下触发下压的腿长阈值 [m]
constexpr float kMidLegDipTargetLengthM = 0.22f;   ///< 下压目标腿长 [m]
constexpr uint16_t kMidLegDipHoldTicks = 1000;     ///< 下压维持时间 [ticks @ 500Hz = 1s]

// -- 上台阶退勾起立 --
constexpr uint16_t kStairExitStandupHookDelayTicks = 0;  ///< 上台阶退出后延迟触发起立三段式 [ticks @ 500Hz]，0=禁用

// ==== 物理参数（变体专属）====

constexpr float kBodyMassKg = 22.0f;  ///< 机体质量 [kg]

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.0035f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵）====
static constexpr const auto &kCtrlPLow = ::wheel_legged::params::generated::kCtrlP;

// ==== 基本运动（PID 增益）====
constexpr PidGains kLeftL0Pid{2000.0f, 0.f, 120.0f, 170.0f, 10.0f};   ///< 左腿腿长 PID（常规）
constexpr PidGains kRightL0Pid{2000.0f, 0.f, 120.0f, 170.0f, 10.0f};  ///< 右腿腿长 PID（常规）
constexpr PidGains kRollPid{1000.0f, 0.1f, 20.0f, 140.0f, 40.0f};     ///< 横滚平衡 PID

// ==== 跳跃（PID 增益）====
constexpr float kJumpPushForceN = 250.0f;                                     ///< 蹬伸阶段单腿基础支撑力 [N]
constexpr PidGains kLeftL0PidJumpOne{700.0f, 0.f, 100.0f, 120.0f, 20.0f};     ///< 左腿预备 PID（JumpPrep）
constexpr PidGains kRightL0PidJumpOne{700.0f, 0.f, 100.0f, 120.0f, 20.0f};    ///< 右腿预备 PID（JumpPrep）
constexpr PidGains kLeftL0PidJumpTwo{3800.0f, 0.0f, 200.0f, 200.0f, 0.0f};    ///< 左腿蹬伸 PID
constexpr PidGains kRightL0PidJumpTwo{3800.0f, 0.0f, 200.0f, 200.0f, 0.0f};   ///< 右腿蹬伸 PID
constexpr PidGains kLeftL0PidJumpThree{3500.0f, 0.f, 150.0f, 150.0f, 30.0f};  ///< 左腿回收 PID
constexpr PidGains kRightL0PidJumpThree{3500.0f, 0.f, 150.0f, 150.0f, 30.0f};  ///< 右腿回收 PID
constexpr PidGains kLeftLegAnglePidJumpRetract2{15.0f, 0.0f, 0.0f, 20.0f, 0.0f};  ///< 左腿摆角 PID（跳跃收腿第二阶段）
constexpr PidGains kRightLegAnglePidJumpRetract2{15.0f, 0.0f, 0.0f, 20.0f, 0.0f};  ///< 右腿摆角 PID（跳跃收腿第二阶段）

// ==== 中腿长下压（PID 增益）====
constexpr PidGains kLeftL0PidDip{450.0f, 0.f, 200.0f, 170.0f, 10.0f};   ///< 左腿下压腿长 PID
constexpr PidGains kRightL0PidDip{450.0f, 0.f, 200.0f, 170.0f, 10.0f};  ///< 右腿下压腿长 PID

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{18.0f, 0.f, 0.0f, 11.5f, 10.0f};           ///< 左腿摆角速度 PID
constexpr PidGains kRightLegTurnPid{18.0f, 0.f, 0.0f, 11.5f, 10.0f};          ///< 右腿摆角速度 PID
constexpr PidGains kLeftLegAnglePidStandup{15.0f, 0.0f, 0.0f, 20.0f, 0.0f};   ///< 左腿摆角 PID（起立用）
constexpr PidGains kRightLegAnglePidStandup{15.0f, 0.0f, 0.0f, 20.0f, 0.0f};  ///< 右腿摆角 PID（起立用）
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
constexpr std::int16_t kWheelSpinThreshold = 220;    ///< 拨轮超过此值触发小陀螺保持
constexpr std::int16_t kWheelActionThreshold = 320;  ///< 拨轮回中后快速负推超过此值触发跳跃
constexpr std::int16_t kWheelCenterThreshold = 80;   ///< 拨轮归中阈值
// 自动跳跃已移除

constexpr float kControlLoopDtS = 0.002f;  ///< 控制环周期 [s]

// -- 摇杆/键鼠输入归一化 --
constexpr std::int16_t kDr16AxisMaxAbs = 660;       ///< DR16 摇杆轴最大绝对值
constexpr float kRcStickMax = 660.0f;               ///< RC 摇杆最大值
constexpr float kTcMouseMax = 200.0f;               ///< 图传鼠标增量最大值
constexpr float kRcYawRateMaxRadS = -5.5f;          ///< RC 摇杆满偏时偏航积分速率 [rad/s]
constexpr float kRcPitchRateMaxRadS = 5.5f;         ///< RC 摇杆满偏时俯仰积分速率 [rad/s]
constexpr float kTcMouseYawRateMaxRadS = -3.0f;     ///< 图传鼠标满偏时偏航积分速率 [rad/s]
constexpr float kTcMousePitchRateMaxRadS = 1.5f;    ///< 图传鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kDr16MouseMax = 1600.0f;            ///< DR16 鼠标增量最大值（用于积分目标速率计算）
constexpr float kDr16MouseYawRateMaxRadS = -2.0f;   ///< DR16 鼠标满偏时偏航积分速率 [rad/s]
constexpr float kDr16MousePitchRateMaxRadS = 1.0f;  ///< DR16 鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kPitchTargetMinRad = -0.5f;         ///< RC 积分俯仰目标下限 [rad]
constexpr float kPitchTargetMaxRad = 0.65f;         ///< RC 积分俯仰目标上限 [rad]
constexpr float kKeyboardAccelRampStep = 0.004f;    ///< 键盘 WASD 加速斜坡步进（每周期，0→1 约 0.5s）
constexpr float kKeyboardBrakeRampStep = 0.1f;      ///< 键盘 WASD 减速斜坡步进（每周期，1→0 约 0.25s）

// -- 云台启动归中判稳 --
constexpr float kGimbalStartupYawAlignErrorRad = 0.1f;            ///< 归中完成位置误差阈值 [rad]
constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;            ///< 归中完成速度阈值 [rad/s]
constexpr std::uint32_t kGimbalStartupYawAlignStableTicks = 10U;  ///< 归中判稳所需连续满足周期数

// -- 偏航就绪判稳 --
constexpr float kYawFollowDriveReadyErrorRad = 0.1f;            ///< 偏航就绪位置误差阈值 [rad]
constexpr float kYawFollowDriveReadyVelRadS = 0.25f;            ///< 偏航就绪速度阈值 [rad/s]
constexpr std::uint32_t kYawFollowDriveReadyStableTicks = 50U;  ///< 偏航就绪判稳所需连续周期数

// ==== 基本运动 ====
constexpr float kTargetForwardSpeedMaxMps = 2.1f;        ///< 最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxNoScMps = 2.1f;    ///< 无超电最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.f;  ///< 高腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxMidLegMps = 1.0f;  ///< F键中腿长模式最大前进速度 [m/s]
constexpr float kTargetSpeedBiasLowLegMps = 0.f;         ///< 低腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegMps = 0.0f;        ///< C键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegFMps = 0.0f;       ///< F键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasHighLegMps = 0.0f;       ///< 高腿长目标速度偏置 [m/s]
constexpr float kMaxSafeYawRateRadS = 4.5f;              ///< 摩擦圆最大安全偏航速率 [rad/s]
constexpr float kLargeTurnThresholdRad = 0.5f;  ///< 大转向检测阈值：motor_error 超此值触发先减速再转向 [rad]
constexpr float kSafeTurnSpeedMps = 0.6f;  ///< 大转向安全速度上限：超此速度时强制限速 [m/s]
constexpr float kLargeTurnThetaThresholdRad = 0.15f;  ///< 大转向腿摆角阈值：摆角超此值需先回正再转向 [rad]
constexpr float kLargeTurnRecoveryAccelScale = 0.4f;  ///< 大转向恢复加速缩放：限速解除后加速斜坡乘以该系数，越小越缓
constexpr float kVxInputDeadbandNorm = 0.05f;        ///< 前进输入死区
constexpr float kVyInputDeadbandNorm = 0.05f;        ///< 平移输入死区
constexpr float kYawFollowRampStepRadS = 0.11f;      ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kYawFollowRampStepRadNoScS = 0.06f;  ///< 偏航跟随角速度斜坡步长（无超电）[(rad/s)/周期]

constexpr float kPositionFreezeSpeedThresholdMps = 0.05f;  ///< 位置锚定冻结速度阈值 [m/s]
constexpr uint32_t kPositionHoldTimeoutTicks =
    500U;  ///< 位置锚定超时 [ticks]（斜坡归零后最多等待此周期数，超时强制冻结）
constexpr float kPositionErrorScaleLowLeg = 3.f;     ///< 低腿长位置误差缩放
constexpr float kVelocityErrorScaleLowLeg = 1.3f;    ///< 低腿长速度误差缩放
constexpr float kPositionErrorScaleMidLeg = 1.0f;    ///< 中腿长位置误差缩放
constexpr float kVelocityErrorScaleMidLeg = 1.0f;    ///< 中腿长速度误差缩放
constexpr float kPositionErrorScaleHighLeg = 1.0f;   ///< 高腿长位置误差缩放
constexpr float kVelocityErrorScaleHighLeg = 1.0f;   ///< 高腿长速度误差缩放
constexpr float kYawFollowFixedTargetRad = -3.087f;  ///< 偏航跟随固定目标偏置角 [rad]
constexpr float kYawFollowSideOffsetRad = 0.5f * wheel_legged::params::common::kPi;  ///< 偏航跟随侧向目标偏置角 [rad]
// kYawFollowPid 已移除：转向由 LQR φ 通道 + 偏航电机反作用力驱动

// ==== 期望状态偏置 ====
constexpr float kExpectedThetaLlBiasRadLowLeg = 0.0f;       ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = 0.0f;       ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = 0.0f;              ///< 期望机体俯仰偏置 [rad]
constexpr float kExpectedDisplacementBiasMLowLeg = -0.55f;  ///< 低腿长期望位移偏置 [m]

// ==== 速度斜坡参数 ====
constexpr SdotRampParams kSdotRampLowLeg{0.006f, 0.007f};    ///< 低腿长速度斜坡
constexpr SdotRampParams kSdotRampMidLeg{0.0035f, 0.007f};   ///< 中腿长速度斜坡(C 键触发)
constexpr SdotRampParams kSdotRampMidLegF{0.0045f, 0.006f};  ///< 中腿长速度斜坡（F 键触发）
constexpr SdotRampParams kSdotRampHighLeg{0.006f, 0.005f};   ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.04f;           ///< 小陀螺进入偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinExitYawRampStepRadS = 0.04f;       ///< 小陀螺退出偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS1 = 8.5f;          ///< 小陀螺目标自旋角速度 [rad/s] ≤55W
constexpr float kSpinTargetYawDotRadS2 = 9.5f;          ///< 小陀螺目标自旋角速度 [rad/s] 55-65W
constexpr float kSpinTargetYawDotRadS3 = 10.5f;         ///< 小陀螺目标自旋角速度 [rad/s] 65-75W
constexpr float kSpinTargetYawDotRadS4 = 11.5f;         ///< 小陀螺目标自旋角速度 [rad/s] >75W
constexpr float kSpinTargetYawDotRadNoScS1 = 7.0f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] ≤55W
constexpr float kSpinTargetYawDotRadNoScS2 = 8.0f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] 55-65W
constexpr float kSpinTargetYawDotRadNoScS3 = 9.0f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] 65-75W
constexpr float kSpinTargetYawDotRadNoScS4 = 10.f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] >75W
constexpr float kSpinExitYawAlignThresholdRad = 0.15f;  ///< 小陀螺预测退出：yaw 对齐阈值 [rad]
constexpr float kSpinTranslationGain = 0.2f;  ///< 小陀螺平移增益（系数2补偿 cos² 平均衰减，使平均车速=摇杆指令值）
constexpr float kSpinThetaLlBiasRad = 0.f;   ///< 小陀螺时左腿摆角偏置 [rad]
constexpr float kSpinThetaLrBiasRad = 0.f;   ///< 小陀螺时右腿摆角偏置 [rad]
constexpr float kSpinLegLengthBiasM = 0.0f;  ///< 小陀螺时腿长偏差（左+右-）[m]
constexpr float kSpinThetaBBiasRad = 0.0f;   ///< 小陀螺时俯仰目标偏置 [rad]

// ==== 跳跃腿摆角偏置 ====
constexpr float kJumpThetaLlBiasRad = 0.0f;  ///< 跳跃时左腿摆角偏置 [rad]
constexpr float kJumpThetaLrBiasRad = 0.0f;  ///< 跳跃时右腿摆角偏置 [rad]
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
constexpr float kWheelSpeedFilterCutoffHz = 5.0f;       ///< 轮速低通滤波截止频率 [Hz]
constexpr float kWheelSpeedFilterSampleHz = 500.0f;     ///< 轮速低通滤波采样频率 [Hz]

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
constexpr float kLeftPhi1OffsetRad = 1.784f + M_PI + 0.07f;  ///< 左腿前关节零位偏移 [rad]-1.784
constexpr float kLeftPhi4OffsetRad = -2.2637f;               ///< 左腿后关节零位偏移 [rad]2.2637
constexpr float kRightPhi1OffsetRad = 0.8493f + M_PI;        ///< 右腿前关节零位偏移 [rad]0.8493
constexpr float kRightPhi4OffsetRad = -1.653f;               ///< 右腿后关节零位偏移 [rad]-1.653

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
constexpr PidGains kYawPositionPid{70.0f, 0.f, 1.5f, 10.0f, 2.2f};   ///< 自瞄偏航位置 PID（打装甲板）
constexpr PidGains kYawSpeedPid{0.65f, 0.0f, 0.0f, 10.0f, 0.f};      ///< 自瞄偏航速度 PID（打装甲板）
constexpr PidGains kPitchPositionPid{40.0f, 0.f, 1.5f, 10.0f, 2.f};  ///< 自瞄俯仰位置 PID（打装甲板）
constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 10.0f, 0.f};    ///< 自瞄俯仰速度 PID（打装甲板）

constexpr PidGains kYawPositionPidRune{60.0f, 0.f, 1.5f, 10.0f, 2.2f};   ///< 自瞄偏航位置 PID（打符）
constexpr PidGains kYawSpeedPidRune{0.65f, 0.0f, 0.0f, 10.0f, 0.f};      ///< 自瞄偏航速度 PID（打符）
constexpr PidGains kPitchPositionPidRune{40.0f, 0.f, 1.5f, 10.0f, 2.f};  ///< 自瞄俯仰位置 PID（打符）
constexpr PidGains kPitchSpeedPidRune{0.55f, 0.0f, 0.0f, 10.0f, 0.f};    ///< 自瞄俯仰速度 PID（打符）
}  // namespace aimbot

// ── 自瞄 + 小陀螺模式 PID ──
namespace aimbot_spin {
constexpr PidGains kYawPositionPid{60.0f, 0.f, 1.5f, 10.0f, 2.2f};   ///< 自瞄+小陀螺偏航位置 PID
constexpr PidGains kYawSpeedPid{0.65f, 0.0f, 0.0f, 10.0f, 0.f};      ///< 自瞄+小陀螺偏航速度 PID
constexpr PidGains kPitchPositionPid{40.0f, 0.f, 1.5f, 10.0f, 2.f};  ///< 自瞄+小陀螺俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 10.0f, 0.f};    ///< 自瞄+小陀螺俯仰速度 PID

// ── 自旋偏航目标偏置（补偿小陀螺自旋时的角度滞后）──
constexpr float kYawTargetBiasSpeedThresholds[3] = {8.0f, 9.5f, 10.5f};  ///< 四档偏置的自旋速度分界 [rad/s]
constexpr float kYawTargetBiasRad[4] = {-0.0f, -0.0f, -0.0f, -0.0f};     ///< 各档位偏航目标偏置 [rad]
constexpr float kYawSpeedFeedforwardRadS[4] = {0.0f, 0.0f, 0.0f, 0.0f};  ///< 自瞄小陀螺各挡 yaw 速度前馈 [rad/s]
}  // namespace aimbot_spin

}  // namespace infantry3

// ══════════════════════════════════════════════════════════════════════════════
// Infantry4（变体 3）专用参数
// ══════════════════════════════════════════════════════════════════════════════
namespace infantry4 {
using namespace common;

// ── CAN 总线 ID 与电机配置 ──
namespace globals {
using namespace common::globals;

constexpr std::uint16_t kLeftWheelId = 0x01;   ///< 左轮毂电机 CAN ID
constexpr std::uint16_t kRightWheelId = 0x02;  ///< 右轮毂电机 CAN ID

constexpr std::uint16_t kDialId = 0x03;  ///< 拨盘电机 CAN ID
}  // namespace globals

// ── 云台 ──
namespace gimbal {
using namespace common::gimbal;

const DmMitSettings kPitchMotorSettings{0x17, 0x07, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};

constexpr float kPitchMinRad = -0.35f;  ///< 俯仰角下限 [rad]
constexpr float kPitchMaxRad = 0.65f;   ///< 俯仰角上限 [rad]

constexpr PidGains kYawPositionPid{30.0f, 0.0f, 0.5f, 10.0f, 1.0f};    ///< 偏航位置 PID
constexpr PidGains kYawSpeedPid{0.6f, 0.0f, 0.0f, 6.0f, 0.4f};         ///< 偏航速度 PID
constexpr PidGains kPitchPositionPid{20.0f, 0.0f, 0.4f, 10.0f, 0.4f};  ///< 俯仰位置 PID
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

constexpr float kPitchFeedforwardBiasNm = 0.f;
}  // namespace gimbal

// ── 云台辨识 ──
namespace gimbal_ident {
using namespace common::gimbal_ident_common;
/// @brief 当前激活的辨识子模式（编译时选择）
static constexpr IdentSubMode kIdentSubMode = IdentSubMode::kHarmonic;
constexpr float kBaseFreqHz = 0.3f;                                              ///< 辨识轨迹基频 [Hz]
constexpr float kDmTorqueLimitNm = 10.0f;                                        ///< DM 电机力矩上限 [Nm]
constexpr float kDefaultDtS = 0.002f;                                            ///< 辨识控制周期 [s]
constexpr float kYawAmp[kHarmonicCount] = {0.35f, -0.2f, 0.13f, -0.12f, 0.04f};  ///< yaw 轴五次谐波幅值 [rad]
constexpr float kPitchAmp[kHarmonicCount] = {0.2106f, 0.2463f, 0.0804f, 0.0554f, 0.0391f};
constexpr float kPitchPhase[kHarmonicCount] = {1.2177f, 0.4006f, -0.8970f, 0.1462f, -2.6391f};
constexpr PidGains kIdentYawPosPid{20.0f, 0.0f, 0.1f, 10.0f, 0.0f};    ///< 辨识模式 yaw 位置 PID
constexpr PidGains kIdentPitchPosPid{70.0f, 0.0f, 0.5f, 10.0f, 0.0f};  ///< 辨识模式 pitch 位置 PID
constexpr PidGains kIdentYawVelPid{3.0f, 15.0f, 0.0f, 10.0f, 2.0f};    ///< Friction step yaw 速度 PID
constexpr PidGains kIdentPitchVelPid{5.0f, 20.0f, 0.0f, 10.0f, 2.0f};  ///< 兼容保留，pitch 摩擦现用位置 PID
constexpr float kIdentPitchCenter = 0.944f;  ///< 辨识轨迹 pitch 中心角 [rad]（机械中位，实际需根据云台标定）
constexpr float kIdentPitchTopLimit = 0.6f;     ///< 辨识轨迹 pitch 下限 [rad]
constexpr float kIdentPitchBottomLimit = 1.6f;  ///< 辨识轨迹 pitch 上限 [rad]

// ── 分步辨识参数 ──
constexpr float kGravityPitchAngles[] = {0.65f, 0.80f, 0.94f, 1.10f, 1.25f, 1.40f, 1.55f};
constexpr size_t kGravityAngleCount = 7;
constexpr float kGravityHoldDuration = 2.0f;
constexpr float kGravitySettleDuration = 0.5f;

constexpr float kFrictionYawVelocitiesRadS[] = {0.05f, 0.10f, 0.20f};
constexpr size_t kFrictionYawVelocityCount = sizeof(kFrictionYawVelocitiesRadS) / sizeof(kFrictionYawVelocitiesRadS[0]);
constexpr float kFrictionYawTravelRad = 6.28f;  // ≈2π, 接近 360°
constexpr float kFrictionPitchVelocitiesRadS[] = {0.02f, 0.05f, 0.10f, 0.20f};
constexpr size_t kFrictionPitchVelocityCount =
    sizeof(kFrictionPitchVelocitiesRadS) / sizeof(kFrictionPitchVelocitiesRadS[0]);
constexpr float kFrictionPitchMoveVelocityRadS = 0.20f;
constexpr float kFrictionPauseDuration = 0.5f;

constexpr float kPitchInertiaFrequenciesHz[] = {2.0f, 3.0f, 4.0f};
constexpr size_t kPitchInertiaFrequencyCount =
    sizeof(kPitchInertiaFrequenciesHz) / sizeof(kPitchInertiaFrequenciesHz[0]);
constexpr float kPitchInertiaAmplitude = 0.2f;
constexpr uint16_t kPitchInertiaWarmupCycles = 3;
constexpr uint16_t kPitchInertiaRecordCycles = 8;
constexpr float kPitchInertiaPrepareVelocityRadS = 0.30f;
constexpr float kPitchInertiaPrepareDurationS = 0.5f;

constexpr float kCouplingPitchAngles[] = {0.70f, 0.94f, 1.20f, 1.50f};
constexpr size_t kCouplingAngleCount = 4;
constexpr float kCouplingFreqHz = 1.0f;
constexpr float kCouplingAmplitude = 2.5f;
constexpr float kCouplingDurationPerAngle = 8.0f;  // (2 warmup + 3 record) / 0.2 Hz
constexpr uint16_t kCouplingWarmupCycles = 2;
constexpr uint16_t kCouplingRecordCycles = 3;
}  // namespace gimbal_ident

// ── 发射机构（M3508 拨盘）──
namespace shoot {
constexpr PidGains kDialSpeedPid{10.0f, 0.f, 0.f, 16000.0f, 1000.0f};     ///< 拨盘速度 PID
constexpr PidGains kDialPositionPid{0.5f, 0.f, 0.01f, 30000.0f, 500.0f};  ///< 拨盘位置 PID
constexpr int16_t kDialFireThreshold = -600;                              ///< 发射触发拨轮阈值
constexpr float kFricSpeedTargetRpm = 6000.0f;  ///< 摩擦轮目标转速 [rpm]（打弹检测用）
constexpr PidGains kFricSpeedPid{10.0f, 0.0f, 0.0f, 16000.0f, 2000.0f};  ///< 摩擦轮速度 PID（Shoot控制器用）

// ── 本地热量闭环 ──
constexpr float kHeatPerShot = 10.0f;  ///< 每发子弹热量增量 [热量单位]
constexpr float kHeatSafetyMargin = 45.0f;  ///< 高热量模式停火余量：heat + kHeatPerShot > limit - margin 时抑制发射
constexpr float kLowHeatSafetyMargin = 20.0f;  ///< 低热量模式停火余量
constexpr float kHeatResumeMargin = 20.0f;  ///< 恢复余量：heat < limit - margin 时恢复，与停火线构成迟滞
constexpr uint16_t kDefaultHeatLimit = 240;       ///< 裁判系统离线时默认热量上限
constexpr uint16_t kDefaultCoolingRate = 40;      ///< 裁判系统离线时默认冷却速率 [热量单位/秒]
constexpr uint16_t kLowHeatLimitThreshold = 100;  ///< 低热量上限阈值：低于此值时降频并使用本地热量
constexpr float kLowHeatShootFrequencyHz = 6.0f;  ///< 低热量上限时发射频率 [Hz]
constexpr float kNormalShootFrequencyHz = 12.0f;  ///< 正常发射频率 [Hz]
}  // namespace shoot

// ── 底盘状态机 ──
namespace chassis_fsm {

constexpr float kStairDescendLegLengthM = 0.16f;
constexpr float kStairDescendThetaBTriggerRad = 0.18f;

constexpr StairClimbParams kStairClimb{
    .high_leg_length_m = 0.35f,
    .hook_leg_length_m = 0.35f,
    .retract_leg_length_m = 0.1f,
    .settle_leg_length_m = 0.1f,
    .contact_theta_threshold_rad = 0.40f,
    .hook_theta_target_rad = 1.f,
    .retract_theta_target_rad = -0.2f,
    .retract_theta_tolerance_rad = 0.7f,
    .hook_theta_tolerance_rad = 0.3f,
    .leg_length_tolerance_m = 0.05f,
    .settle_theta_tolerance_rad = 0.33f,
    .settle_theta_target_rad = 0.f,
    .settle_theta_ramp_step_rad = 0.02f,
    .hook_stable_ms = 100U,
    .retract_stable_ms = 200U,
    .settle_stable_ms = 200U,
    .hook_timeout_ms = 1000U,
    .retract_timeout_ms = 1000U,
    .settle_timeout_ms = 1000U,
};

constexpr StairClimbParams kStairClimbStep2{
    .high_leg_length_m = 0.31f,
    .hook_leg_length_m = 0.31f,
    .retract_leg_length_m = 0.1f,
    .settle_leg_length_m = 0.1f,
    .contact_theta_threshold_rad = 0.40f,
    .hook_theta_target_rad = 0.8f,
    .retract_theta_target_rad = -0.2f,
    .retract_theta_tolerance_rad = 0.7f,
    .hook_theta_tolerance_rad = 1.f,
    .leg_length_tolerance_m = 0.05f,
    .settle_theta_tolerance_rad = 0.33f,
    .settle_theta_target_rad = 0.f,
    .settle_theta_ramp_step_rad = 0.02f,
    .hook_stable_ms = 100U,
    .retract_stable_ms = 200U,
    .settle_stable_ms = 200U,
    .hook_timeout_ms = 1000U,
    .retract_timeout_ms = 1000U,
    .settle_timeout_ms = 1000U,
};

// ==== 倒地自启 ====
constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;        ///< 倒地确认时间 [ms]
constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;  ///< 自启超时 [ms]

// ==== 跳跃 ====
constexpr std::uint32_t kJumpLowPrepMs = 400U;      ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpLowPushMaxMs = 2500U;  ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpLowRecoverMs = 1000U;  ///< 跳跃回收阶段持续时间（保底超时）[ms]
constexpr std::uint32_t kJumpLowRecoverMinMs = 500U;  ///< 跳跃回收阶段最低维持时间（此后开始判断离地）[ms]
constexpr float kJumpLowPrepLegLengthM = 0.15f;         ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpLowPushLegLengthM = 0.35f;         ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpLowRecoverLegLengthM = 0.13f;      ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpLowPushReachedLegLengthM = 0.34f;  ///< 蹬伸到位判定腿长 [m]
constexpr std::uint32_t kJumpPushReachedHoldMs = 3U;    ///< 蹬伸腿长到位后持续满足此时间才切换 [ms]

// ==== 基本运动（腿长档位）====
constexpr float kLowLegLengthM = 0.18f;              ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.245f;             ///< 中腿长档位目标腿长 [m]
constexpr float kHighLegLengthM = 0.35f;             ///< 高腿长档位目标腿长 [m]
constexpr float kLegLengthRampTimeS = 0.3f;          ///< 腿长切换斜坡时间 [s]
constexpr std::uint32_t kSpinExitTimeoutMs = 3000U;  ///< 小陀螺预测退出超时兜底 [ms]
constexpr float kSpinEntrySpeedThresholdMps = 0.3f;  ///< 进入小陀螺的速度阈值 [m/s]
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
constexpr float kStandupPhase0ThetaTargetRad = 1.5f;  ///< 起立 Phase 0/1 腿摆角目标 [rad]
constexpr float kStandupPhase1TargetLengthM = 0.1f;   ///< 起立 Phase 1 目标腿长 [m]
constexpr float kStandupPhase1ThetaTolRad = 0.6f;  ///< 起立 Phase 1 完成判定：摆角与目标差值容许 [rad]
constexpr float kStandupPhase1ThetaTolStairRad = 1.1f;  ///< 台阶 step2 触发起立 Phase 1 容许 [rad]
constexpr float kStandupThetaRampStepRad = 0.02f;       ///< 起立摆角斜坡步长 [rad/周期]
constexpr float kPostureRollMinRad = -0.5f;             ///< 横滚角安全下限 [rad]
constexpr float kPostureRollMaxRad = 0.5f;              ///< 横滚角安全上限 [rad]
constexpr float kPostureThetaBMinRad = -0.7f;           ///< 机体俯仰角安全下限 [rad]
constexpr float kPostureThetaBMaxRad = 0.7f;            ///< 机体俯仰角安全上限 [rad]
constexpr float kPostureThetaLegMinRad = -0.8f;         ///< 腿摆角安全下限 [rad]
constexpr float kPostureThetaLegMaxRad = 2.f;           ///< 摆角安全上限 [rad]
constexpr float kLegRecoverThetaDotTarget = -2.0f;      ///< 倒地恢复时腿摆角速度目标 [rad/s]
constexpr float kLegRecoverThetaDotRampStep = 0.06f;    ///< 倒地恢复腿摆角速度斜坡步长 [(rad/s)/周期]
constexpr float kManualRecoveryLegSpeedRadS = 0.5f;     ///< 手动倒地恢复腿摆角速度 [rad/s]
constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;     ///< 倒地恢复零力矩区间下限 [rad]
constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;     ///< 倒地恢复零力矩区间上限 [rad]

// ==== 倒地恢复软着陆 ====
constexpr float kRecoveryDecelZoneRad = 0.6f;   ///< 恢复减速区宽度 [rad]（接近目标边界时开始减速）
constexpr float kRecoveryMinSpeedRadS = 0.08f;  ///< 恢复减速区边界最低速度 [rad/s]
constexpr float kRecoveryGravityRampScale = 0.35f;    ///< 恢复时重力补偿斜坡比例（越大身体越不砸）
constexpr float kRecoveryPitchFeedforwardKp = 30.0f;  ///< 倒地恢复俯仰角前馈系数
constexpr float kRecoveryRollFeedforwardKp = 5.0f;    ///< 倒地恢复横滚角前馈系数

// ==== pitch 恢复刹车 ====
constexpr float kPitchBrakeZoneRad = 0.55f;           ///< pitch 恢复刹车区间 [rad]
constexpr float kPitchBrakeRateStartRadS = 0.6f;      ///< pitch 刹车起始角速度 [rad/s]
constexpr float kPitchBrakeRateFullRadS = 2.0f;       ///< pitch 刹车满力度角速度 [rad/s]
constexpr float kPitchBrakeMinScale = 0.25f;          ///< pitch 刹车最小缩放系数
constexpr float kPitchBrakeReverseRateRadS = 2.6f;    ///< pitch 反转角速度阈值 [rad/s]
constexpr float kPitchBrakeReverseSpeedRadS = 0.35f;  ///< pitch 反转目标速度 [rad/s]

// ==== 倒地恢复腿摆角目标范围 ====
constexpr float kRecoveryThetaRangeLowMin = -4.8f;   ///< 前倒恢复腿摆角下限 [rad]
constexpr float kRecoveryThetaRangeLowMax = -4.2f;   ///< 前倒恢复腿摆角上限 [rad]
constexpr float kRecoveryThetaRangeHighMin = -2.3f;  ///< 后倒恢复腿摆角下限 [rad]
constexpr float kRecoveryThetaRangeHighMax = -1.7f;  ///< 后倒恢复腿摆角上限 [rad]

// -- 离地检测 --
constexpr float kOffGroundSupportForceThresholdN = 20.0f;  ///< 支撑力低于此值判定为离地 [N]
constexpr float kOffGroundSupportForceClampN = 100.0f;     ///< 离地时支持力限幅值 [N]

// -- 中腿长下压 --
constexpr float kMidLegDipTriggerLengthM = 0.27f;  ///< 中腿长模式下触发下压的腿长阈值 [m]
constexpr float kMidLegDipTargetLengthM = 0.22f;   ///< 下压目标腿长 [m]
constexpr uint16_t kMidLegDipHoldTicks = 1000;     ///< 下压维持时间 [ticks @ 500Hz = 1s]

// -- 上台阶退勾起立 --
constexpr uint16_t kStairExitStandupHookDelayTicks = 0;  ///< 上台阶退出后延迟触发起立三段式 [ticks @ 500Hz]，0=禁用

// ==== 物理参数（变体专属）====
constexpr float kBodyMassKg = 22.0f;  ///< 机体质量 [kg]

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.002f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵）====
static constexpr const auto &kCtrlPLow = ::wheel_legged::params::generated::kCtrlP;

// ==== 基本运动（PID 增益）====
constexpr PidGains kLeftL0Pid{2500.0f, 0.f, 200.0f, 170.0f, 30.0f};   ///< 左腿腿长 PID（常规）
constexpr PidGains kRightL0Pid{2500.0f, 0.f, 200.0f, 170.0f, 30.0f};  ///< 右腿腿长 PID（常规）
constexpr PidGains kRollPid{800.0f, 0.0f, 200.0f, 80.0f, 0.0f};       ///< 横滚平衡 PID

// ==== 跳跃（PID 增益）====
constexpr float kJumpPushForceN = 0.0f;                                       ///< 蹬伸阶段单腿基础支撑力 [N]
constexpr PidGains kLeftL0PidJumpOne{1300.0f, 0.f, 200.0f, 100.0f, 20.0f};    ///< 左腿预备 PID（JumpPrep）
constexpr PidGains kRightL0PidJumpOne{1300.0f, 0.f, 200.0f, 100.0f, 20.0f};   ///< 右腿预备 PID（JumpPrep）
constexpr PidGains kLeftL0PidJumpTwo{3500.0f, 0.0f, 300.0f, 180.0f, 0.0f};    ///< 左腿蹬伸 PID
constexpr PidGains kRightL0PidJumpTwo{3000.0f, 0.0f, 300.0f, 180.0f, 0.0f};   ///< 右腿蹬伸 PID
constexpr PidGains kLeftL0PidJumpThree{1400.0f, 0.f, 200.0f, 100.0f, 30.0f};  ///< 左腿回收 PID
constexpr PidGains kRightL0PidJumpThree{1400.0f, 0.f, 200.0f, 100.0f, 30.0f};  ///< 右腿回收 PID
constexpr PidGains kLeftLegAnglePidJumpRetract2{15.0f, 0.0f, 0.0f, 20.0f, 0.0f};  ///< 左腿摆角 PID（跳跃收腿第二阶段）
constexpr PidGains kRightLegAnglePidJumpRetract2{15.0f, 0.0f, 0.0f, 20.0f, 0.0f};  ///< 右腿摆角 PID（跳跃收腿第二阶段）

// ==== 中腿长下压（PID 增益）====
constexpr PidGains kLeftL0PidDip{400.0f, 0.f, 200.0f, 170.0f, 30.0f};   ///< 左腿下压腿长 PID
constexpr PidGains kRightL0PidDip{400.0f, 0.f, 200.0f, 170.0f, 30.0f};  ///< 右腿下压腿长 PID

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{18.0f, 0.f, 30.0f, 13.0f, 10.0f};          ///< 左腿摆角速度 PID
constexpr PidGains kRightLegTurnPid{18.0f, 0.f, 30.0f, 13.0f, 10.0f};         ///< 右腿摆角速度 PID
constexpr PidGains kLeftLegAnglePidStandup{10.0f, 0.0f, 0.0f, 10.0f, 0.0f};   ///< 左腿摆角 PID（起立用）
constexpr PidGains kRightLegAnglePidStandup{10.0f, 0.0f, 0.0f, 10.0f, 0.0f};  ///< 右腿摆角 PID（起立用）
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
constexpr std::int16_t kWheelSpinThreshold = 220;    ///< 拨轮超过此值触发小陀螺保持
constexpr std::int16_t kWheelActionThreshold = 320;  ///< 拨轮回中后快速负推超过此值触发跳跃
constexpr std::int16_t kWheelCenterThreshold = 80;   ///< 拨轮归中阈值
// 自动跳跃触发阈值已移除

constexpr float kControlLoopDtS = 0.002f;  ///< 控制环周期 [s]

// -- 摇杆/键鼠输入归一化 --
constexpr std::int16_t kDr16AxisMaxAbs = 660;       ///< DR16 摇杆轴最大绝对值
constexpr float kRcStickMax = 660.0f;               ///< RC 摇杆最大值
constexpr float kTcMouseMax = 200.0f;               ///< 图传鼠标增量最大值
constexpr float kRcYawRateMaxRadS = -4.f;           ///< RC 摇杆满偏时偏航积分速率 [rad/s]
constexpr float kRcPitchRateMaxRadS = 2.5f;         ///< RC 摇杆满偏时俯仰积分速率 [rad/s]
constexpr float kTcMouseYawRateMaxRadS = -3.0f;     ///< 图传鼠标满偏时偏航积分速率 [rad/s]
constexpr float kTcMousePitchRateMaxRadS = 2.5f;    ///< 图传鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kDr16MouseMax = 1600.0f;            ///< DR16 鼠标增量最大值（用于积分目标速率计算）
constexpr float kDr16MouseYawRateMaxRadS = -2.0f;   ///< DR16 鼠标满偏时偏航积分速率 [rad/s]
constexpr float kDr16MousePitchRateMaxRadS = 1.5f;  ///< DR16 鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kPitchTargetMinRad = -0.35f;        ///< RC 积分俯仰目标下限 [rad]
constexpr float kPitchTargetMaxRad = 0.6f;          ///< RC 积分俯仰目标上限 [rad]
constexpr float kKeyboardAccelRampStep = 0.006f;    ///< 键盘 WASD 加速斜坡步进（每周期，0→1 约 0.5s）
constexpr float kKeyboardBrakeRampStep = 0.1f;      ///< 键盘 WASD 减速斜坡步进（每周期，1→0 约 0.25s）

// -- 云台启动归中判稳 --
constexpr float kGimbalStartupYawAlignErrorRad = 0.1f;            ///< 归中完成位置误差阈值 [rad]
constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;            ///< 归中完成速度阈值 [rad/s]
constexpr std::uint32_t kGimbalStartupYawAlignStableTicks = 10U;  ///< 归中判稳所需连续满足周期数

// -- 偏航就绪判稳 --
constexpr float kYawFollowDriveReadyErrorRad = 0.04f;           ///< 偏航就绪位置误差阈值 [rad]
constexpr float kYawFollowDriveReadyVelRadS = 0.25f;            ///< 偏航就绪速度阈值 [rad/s]
constexpr std::uint32_t kYawFollowDriveReadyStableTicks = 50U;  ///< 偏航就绪判稳所需连续周期数

// ==== 基本运动 ====
constexpr float kTargetForwardSpeedMaxMps = 2.1f;         ///< 最大前进速度 [m/s+]
constexpr float kTargetForwardSpeedMaxNoScMps = 2.1f;     ///< 无超电最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.2f;  ///< 高腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxMidLegMps = 1.0f;   ///< F键中腿长模式最大前进速度 [m/s]
constexpr float kTargetSpeedBiasLowLegMps = 0.f;          ///< 低腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegMps = 0.f;          ///< C键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegFMps = 0.f;         ///< F键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasHighLegMps = -0.f;        ///< 高腿长目标速度偏置 [m/s]
constexpr float kMaxSafeYawRateRadS = 4.0f;               ///< 摩擦圆最大安全偏航速率 [rad/s]
constexpr float kLargeTurnThresholdRad = 0.5f;  ///< 大转向检测阈值：motor_error 超此值触发先减速再转向 [rad]
constexpr float kSafeTurnSpeedMps = 0.5f;  ///< 大转向安全速度上限：超此速度时强制限速 [m/s]
constexpr float kLargeTurnThetaThresholdRad = 0.1f;  ///< 大转向腿摆角阈值：摆角超此值需先回正再转向 [rad]
constexpr float kLargeTurnRecoveryAccelScale = 0.6f;  ///< 大转向恢复加速缩放：限速解除后加速斜坡乘以该系数，越小越缓
constexpr float kVxInputDeadbandNorm = 0.05f;        ///< 前进输入死区
constexpr float kVyInputDeadbandNorm = 0.05f;        ///< 平移输入死区
constexpr float kYawFollowRampStepRadS = 0.1f;       ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kYawFollowRampStepRadNoScS = 0.06f;  ///< 偏航跟随角速度斜坡步长（无超电）[(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.15f;  ///< 位置锚定冻结速度阈值 [m/s]
constexpr uint32_t kPositionHoldTimeoutTicks =
    1000U;  ///< 位置锚定超时 [ticks]（斜坡归零后最多等待此周期数，超时强制冻结）
constexpr float kPositionErrorScaleLowLeg = 1.0f;      ///< 低腿长位置误差缩放
constexpr float kVelocityErrorScaleLowLeg = 1.0f;      ///< 低腿长速度误差缩放
constexpr float kPositionErrorScaleMidLeg = 1.0f;      ///< 中腿长位置误差缩放
constexpr float kVelocityErrorScaleMidLeg = 1.0f;      ///< 中腿长速度误差缩放
constexpr float kPositionErrorScaleHighLeg = 1.0f;     ///< 高腿长位置误差缩放
constexpr float kVelocityErrorScaleHighLeg = 1.0f;     ///< 高腿长速度误差缩放
constexpr float kYawFollowFixedTargetRad = -2.662f;    ///< 偏航跟随固定目标偏置角 [rad]
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;  ///< 偏航跟随侧向目标偏置角 [rad]

// ==== 期望状态偏置 ====
constexpr float kExpectedThetaLlBiasRadLowLeg = 0.0f;     ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = 0.0f;     ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = -0.033f;         ///< 期望机体俯仰偏置 [rad]
constexpr float kExpectedDisplacementBiasMLowLeg = 0.0f;  ///< 低腿长期望位移偏置 [m]

// ==== 速度斜坡参数 ====
constexpr SdotRampParams kSdotRampLowLeg{0.0065f, 0.007f};   ///< 低腿长速度斜坡
constexpr SdotRampParams kSdotRampMidLeg{0.0035f, 0.007f};   ///< 中腿长速度斜坡(C 键触发)
constexpr SdotRampParams kSdotRampMidLegF{0.0045f, 0.006f};  ///< 中腿长速度斜坡（F 键触发）
constexpr SdotRampParams kSdotRampHighLeg{0.006f, 0.005f};   ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.05f;           ///< 小陀螺进入偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinExitYawRampStepRadS = 0.05f;       ///< 小陀螺退出偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS1 = 8.5f;          ///< 小陀螺目标自旋角速度 [rad/s] ≤55W
constexpr float kSpinTargetYawDotRadS2 = 9.5f;          ///< 小陀螺目标自旋角速度 [rad/s] 55-65W
constexpr float kSpinTargetYawDotRadS3 = 10.5f;         ///< 小陀螺目标自旋角速度 [rad/s] 65-75W
constexpr float kSpinTargetYawDotRadS4 = 11.5f;         ///< 小陀螺目标自旋角速度 [rad/s] >75W
constexpr float kSpinTargetYawDotRadNoScS1 = 7.0f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] ≤55W
constexpr float kSpinTargetYawDotRadNoScS2 = 8.0f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] 55-65W
constexpr float kSpinTargetYawDotRadNoScS3 = 9.0f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] 65-75W
constexpr float kSpinTargetYawDotRadNoScS4 = 10.f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] >75W
constexpr float kSpinExitYawAlignThresholdRad = 0.15f;  ///< 小陀螺预测退出：yaw 对齐阈值 [rad]
constexpr float kSpinTranslationGain = 0.25f;           ///< 小陀螺平移增益
constexpr float kSpinThetaLlBiasRad = 0.0f;             ///< 小陀螺时左腿摆角偏置 [rad]
constexpr float kSpinThetaLrBiasRad = 0.f;              ///< 小陀螺时右腿摆角偏置 [rad]
constexpr float kSpinLegLengthBiasM = 0.0f;             ///< 小陀螺时腿长偏差（左+右-）[m]
constexpr float kSpinThetaBBiasRad = 0.f;               ///< 小陀螺时俯仰目标偏置 [rad]

// ==== 跳跃腿摆角偏置 ====
constexpr float kJumpThetaLlBiasRad = 0.0f;  ///< 跳跃时左腿摆角偏置 [rad]
constexpr float kJumpThetaLrBiasRad = 0.0f;  ///< 跳跃时右腿摆角偏置 [rad]
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
constexpr float kWheelSpeedFilterCutoffHz = 5.0f;       ///< 轮速低通滤波截止频率 [Hz]
constexpr float kWheelSpeedFilterSampleHz = 500.0f;     ///< 轮速低通滤波采样频率 [Hz]

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
constexpr float kLeftPhi1OffsetRad = 1.38f + M_PI;              ///< 左腿前关节零位偏移 [rad]
constexpr float kLeftPhi4OffsetRad = 0.86f;                     ///< 左腿后关节零位偏移 [rad]
constexpr float kRightPhi1OffsetRad = 1.26f + 0.53926f + M_PI;  ///< 右腿前关节零位偏移 [rad]
constexpr float kRightPhi4OffsetRad = 1.02f + 3.554876f;        ///< 右腿后关节零位偏移 [rad]
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

constexpr PidGains kYawPositionPid{50.0f, 0.f, 1.5f, 10.0f, 2.2f};  ///< 自瞄偏航位置 PID（打装甲板）
constexpr PidGains kYawSpeedPid{0.6f, 0.0f, 0.0f, 10.0f, 0.f};      ///< 自瞄偏航速度 PID（打装甲板）
constexpr PidGains kPitchPositionPid{45.0f, 0.f, 2.f, 10.0f, 2.f};  ///< 自瞄俯仰位置 PID（打装甲板）
constexpr PidGains kPitchSpeedPid{0.5f, 0.0f, 0.0f, 10.0f, 0.f};    ///< 自瞄俯仰速度 PID（打装甲板）

constexpr PidGains kYawPositionPidRune{50.0f, 0.f, 1.5f, 10.0f, 2.2f};  ///< 自瞄偏航位置 PID（打符）
constexpr PidGains kYawSpeedPidRune{0.6f, 0.0f, 0.0f, 10.0f, 0.f};      ///< 自瞄偏航速度 PID（打符）
constexpr PidGains kPitchPositionPidRune{45.0f, 0.f, 2.f, 10.0f, 2.f};  ///< 自瞄俯仰位置 PID（打符）
constexpr PidGains kPitchSpeedPidRune{0.5f, 0.0f, 0.0f, 10.0f, 0.f};    ///< 自瞄俯仰速度 PID（打符）
}  // namespace aimbot

// ── 自瞄 + 小陀螺模式 PID ──
namespace aimbot_spin {
constexpr PidGains kYawPositionPid{60.0f, 0.f, 1.5f, 10.0f, 2.2f};   ///< 自瞄+小陀螺偏航位置 PID
constexpr PidGains kYawSpeedPid{0.6f, 0.0f, 0.0f, 10.0f, 0.f};       ///< 自瞄+小陀螺偏航速度 PID
constexpr PidGains kPitchPositionPid{50.0f, 0.f, 1.6f, 10.0f, 4.f};  ///< 自瞄+小陀螺俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.6f, 0.0f, 0.0f, 10.0f, 0.f};     ///< 自瞄+小陀螺俯仰速度 PID

// ── 自旋偏航目标偏置（补偿小陀螺自旋时的角度滞后）──
constexpr float kYawTargetBiasSpeedThresholds[3] = {8.0f, 9.5f, 10.5f};  ///< 四档偏置的自旋速度分界 [rad/s]
constexpr float kYawTargetBiasRad[4] = {0.0f, 0.0f, 0.0f, 0.0f};         ///< 各档位偏航目标偏置 [rad]
constexpr float kYawSpeedFeedforwardRadS[4] = {0.f, 0.f, 0.f, 0.f};  ///< 自瞄小陀螺各挡 yaw 速度前馈 [rad/s]
}  // namespace aimbot_spin

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
