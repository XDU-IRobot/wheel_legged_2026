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

constexpr float kDefaultDtS = 0.002f;                ///< 云台控制默认周期 [s]
constexpr float kDmTorqueLimitNm = 10.0f;            ///< DM 电机力矩上限 [Nm]
constexpr float kPitchGravityCompensationNm = 1.3f;  ///< 俯仰重力补偿力矩 [Nm]

const DmMitSettings kPitchMotorSettings{0x11, 0x01, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
const DmMitSettings kYawMotorSettings{0x12, 0x02, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
}  // namespace gimbal

// ── 云台辨识 ──
namespace gimbal_ident {
constexpr float kBaseFreqHz = 0.1f;                    ///< 辨识轨迹基频 [Hz]
constexpr size_t kHarmonicCount = 5;                   ///< 五次谐波
constexpr float kRpmToRadPerSec = kPi * 2.0f / 60.0f;  ///< rpm → rad/s
constexpr float kDmTorqueLimitNm = 10.0f;              ///< DM 电机力矩上限 [Nm]
constexpr float kDefaultDtS = 0.002f;                  ///< 辨识控制周期 [s]

/// @brief yaw 轴五次谐波幅值 [rad]（sum_abs=6.0，~70%峰值因子下覆盖 ±241°）
constexpr float kYawAmp[kHarmonicCount] = {1.0f, -0.6f, 0.4f, -0.35f, 0.1f};
/// @brief pitch 轴五次谐波幅值 [rad]（sum_abs=0.58，峰值~±0.44，留余量不触及机械限位 [0.6, 1.6]）
constexpr float kPitchAmp[kHarmonicCount] = {0.27f, -0.14f, 0.09f, -0.05f, 0.03f};

/// @brief 辨识模式 yaw 位置 PID（单位置环，高增益）
constexpr PidGains kIdentYawPosPid{20.0f, 0.0f, 0.1f, 10.0f, 0.0f};
/// @brief 辨识模式 pitch 位置 PID（单位置环）
constexpr PidGains kIdentPitchPosPid{60.0f, 0.0f, 0.5f, 10.0f, 0.0f};

/// @brief 辨识轨迹 pitch 中心角 [rad]（机械中位，实际需根据云台标定）
constexpr float kIdentPitchCenter = 0.48f;
/// @brief 辨识轨迹 pitch 下限 [rad]
constexpr float kIdentPitchTopLimit = 0.37f;
/// @brief 辨识轨迹 pitch 上限 [rad]
constexpr float kIdentPitchBottomLimit = 0.66f;

constexpr size_t kIdentUartTxBufSize = 128;  ///< 辨识串口发送缓冲区大小 [byte]

}  // namespace gimbal_ident

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
0,0,0,0,0,0,0,0,0
};
}  // namespace gimbal

// ── 发射机构（Hero：三摩擦轮 + DM 拨盘）──
namespace shoot {
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

inline constexpr int kFrictionWheelCount = 3;                                ///< 摩擦轮数量
inline constexpr float kBoosterZeroPointRad = 0.345f;                        ///< 拨盘零位角度 [rad]
inline constexpr float kSegmentAngleRad = kPi / 3.f;                         ///< 拨盘分段角度 [rad]
inline constexpr uint16_t kInitDelayTicks = 600;                             ///< 初始化延迟周期数
inline constexpr uint16_t kShootDelayTicks = 360;                            ///< 发射延迟周期数
inline constexpr float kStallThresholdRad = kPi / 18.f;                      ///< 堵转判定角度阈值 [rad]
inline constexpr float kStallFallbackRad = kPi / 90.f;                       ///< 堵转回退角度 [rad]
inline constexpr float kFwTargetSpeedRpm = 3650.0f;                          ///< 摩擦轮目标转速 [rpm]  12m/s
// inline constexpr float kFwTargetSpeedRpm = 4690.0f;                       ///< 摩擦轮目标转速 [rpm]   16m/s
inline constexpr float kFricSpeedTargetRpm = kFwTargetSpeedRpm;              ///< alias for cross-variant compatibility
inline constexpr float kFwReadySpeedThresholdRpm = kFwTargetSpeedRpm - 600.f;                  ///< 摩擦轮就绪判定转速 [rpm]
inline constexpr int16_t kFireDialThreshold = -100;                          ///< 发射触发拨轮阈值
inline constexpr PidGains kBoosterPositionPid{60.f, 0.f, 560.f, 24.f, 0.f};  ///< 拨盘位置 PID
inline constexpr PidGains kBoosterSpeedPid{0.3f, 0.f, 0.02f, 6.4f, 0.f};     ///< 拨盘速度 PID
inline constexpr PidGains kFwSpeedPid{30.f, 0.01f, 0.f, 10000.f, 0.f};       ///< 摩擦轮速度 PID
inline constexpr uint16_t kFwMotor1Id = 0x01;                                ///< 摩擦轮电机 1 CAN ID
inline constexpr uint16_t kFwMotor2Id = 0x02;                                ///< 摩擦轮电机 2 CAN ID
inline constexpr uint16_t kFwMotor3Id = 0x03;                                ///< 摩擦轮电机 3 CAN ID
inline constexpr uint16_t kBoosterMasterId = 0x10;                           ///< 拨盘主电机 CAN ID
inline constexpr uint16_t kBoosterSlaveId = 0x09;                            ///< 拨盘从电机 CAN ID
inline const DmMitSettings kBoosterDmSettings{0x10, 0x09, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
}  // namespace shoot

// ── 底盘状态机 ──
namespace chassis_fsm {

// ==== 上台阶 ====
constexpr float kStairClimbThetaThresholdRad = 0.5f;               ///< 双腿摆角均超过此值触发上台阶 [rad]
constexpr float kStairClimbLegLengthM = 0.16f;                     ///< 上台阶收腿目标腿长 [m]
constexpr float kStairClimbPhase0LegLengthM = 0.16f;               ///< 上台阶 Phase 0 转腿目标腿长 [m]
constexpr float kStairClimbThetaTargetRad = 1.5f;                 ///< 上台阶目标腿摆角（统一偏置） [rad]
constexpr std::uint32_t kStairClimbDurationMs =450U;              ///< 上台阶最长持续时间 [ms]
constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.2f;  ///< 腿长到位容差 [m]
constexpr float kStairClimbThetaNearZeroThresholdRad = 0.2f;  ///< 摆角归零判定阈值（上台阶完成） [rad]
constexpr std::uint32_t kStairClimbPitchStableMs = 300U;      ///< 上台阶完成后俯仰稳定等待时间 [ms]

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

// ==== 跳跃（中腿长）====
constexpr std::uint32_t kJumpMidPrepMs = 100U;      ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpMidPushMaxMs = 500U;  ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpMidRecoverMs = 250U;   ///< 跳跃回收阶段持续时间（保底超时）[ms]
constexpr std::uint32_t kJumpMidRecoverMinMs = 100U;  ///< 跳跃回收阶段最低维持时间（此后开始判断离地）[ms]
constexpr float kJumpMidPrepLegLengthM = 0.13f;         ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpMidPushLegLengthM = 0.27f;         ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpMidRecoverLegLengthM = 0.15f;      ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpMidPushReachedLegLengthM = 0.24f;  ///< 蹬伸到位判定腿长 [m]

// ==== 基本运动（腿长档位）====
constexpr float kLowLegLengthM = 0.13f;      ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.18f;      ///< 中腿长档位目标腿长 [m]
constexpr float kHighLegLengthM = 0.3f;     ///< 高腿长档位目标腿长 [m]
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
constexpr float kLeftSpringC0 = -120.005613f ;
constexpr float kLeftSpringC1 = -129.143860f ;
constexpr float kLeftSpringC2 = -915.501683f;
constexpr float kLeftSpringC3 = 2986.584000f ;
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
constexpr float kStandupThetaThresholdRad = 1.35f;  ///< 起立完成判定：双腿摆角绝对值低于此值后允许轮端输出 [rad]
constexpr float kPostureRollMinRad = -0.5f;      ///< 横滚角安全下限 [rad]
constexpr float kPostureRollMaxRad = 0.5f;       ///< 横滚角安全上限 [rad]
constexpr float kPostureThetaBMinRad = -0.8f;    ///< 机体俯仰角安全下限 [rad]
constexpr float kPostureThetaBMaxRad = 0.8f;     ///< 机体俯仰角安全上限 [rad]
constexpr float kPostureThetaLegMinRad = -0.8f;  ///< 腿摆角安全下限 [rad]
constexpr float kPostureThetaLegMaxRad = 1.55f;  ///< 腿摆角安全上限 [rad]
constexpr float kLegRecoverThetaDotTarget = -1.2f;  ///< 倒地恢复时腿摆角速度目标 [rad/s]（负号表示前摆方向）
constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;  ///< 倒地恢复零力矩区间下限 [rad]
constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;  ///< 倒地恢复零力矩区间上限 [rad]

// -- 离地检测 --
constexpr float kOffGroundSupportForceThresholdN = 10.0f;  ///< 支撑力低于此值判定为离地 [N]
constexpr float kOffGroundSupportForceClampN = 120.0f;     ///< 离地时支持力限幅值 [N]

// ==== 物理参数（变体专属）====
constexpr float kBodyMassKg = 30.0f;  ///< 机体质量 [kg]

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.030f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵 — 40 组多项式系数）====
/// 由 MATLAB 离线拟合得到，p(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l² + p11*l_l*l_r + p02*l_r²
/// 共 40 行，对应 4×10 增益矩阵 K 的 40 个元素（按行主序展平）
/// 每行 6 个系数：[p00, p10, p01, p20, p11, p02]
constexpr std::array<float, 240> kCtrlP{
-3.3866,  -26.129,  21.122,  42.49,  -19.006,  -17.279,
     -5.7288,  -31.678,  31.601,  55.149,  -33.98,  -24.626,
     -0.64256,  3.1726,  -0.91746,  -3.4083,  0.32618,  1.5208,
     -1.4732,  7.5742,  -2.4694,  -7.8075,  0.42677,  4.1651,
     -11.894,  -72.744,  13.997,  69.093,  1.2412,  -18.704,
     -0.81286,  -6.0337,  2.6598,  -3.4943,  2.0596,  -3.3897,
     -3.6456,  3.3754,  -3.6583,  8.0481,  -10.633,  5.3927,
     -0.33489,  -1.5348,  -0.11371,  5.7861,  -10.322,  1.6358,
     -22.079,  39.419,  26.482,  -15.06,  -28.283,  -26.446,
     -2.6573,  2.6011,  6.0756,  1.79,  -6.2068,  -6.2083,
     -3.3866,  21.122,  -26.129,  -17.279,  -19.006,  42.49,
     -5.7288,  31.601,  -31.678,  -24.626,  -33.98,  55.149,
     0.64256,  0.91746,  -3.1726,  -1.5208,  -0.32618,  3.4083,
     1.4732,  2.4694,  -7.5742,  -4.1651,  -0.42677,  7.8075,
     -3.6456,  -3.6583,  3.3754,  5.3927,  -10.633,  8.0481,
     -0.33489,  -0.11371,  -1.5348,  1.6358,  -10.322,  5.7861,
     -11.894,  13.997,  -72.744,  -18.704,  1.2412,  69.093,
     -0.81286,  2.6598,  -6.0337,  -3.3897,  2.0596,  -3.4943,
     -22.079,  26.482,  39.419,  -26.446,  -28.283,  -15.06,
     -2.6573,  6.0756,  2.6011,  -6.2083,  -6.2068,  1.79,
     6.2141,  -5.6444,  -9.4097,  -26.398,  31.952,  6.0862,
     9.3863,  -7.4233,  -20.114,  -37.842,  53.647,  13.158,
     -0.65112,  -3.3948,  -1.214,  5.9573,  -0.90818,  1.9974,
     -1.4808,  -8.3624,  -2.6885,  14.549,  -2.6095,  4.4751,
     37.194,  -59.544,  5.8094,  45.979,  30.141,  -16.303,
     2.1167,  -0.8874,  -1.0494,  -0.83448,  6.2398,  -0.20081,
     -2.2057,  -22.331,  -4.6445,  25.157,  -15.496,  3.0432,
     -0.14912,  -0.71691,  2.6072,  -2.4947,  -1.0183,  -2.6838,
     -36.741,  -146.07,  49.532,  183.66,  22.489,  -71.232,
     -1.9561,  -14.519,  3.8866,  14.635,  6.8275,  -6.5254,
     6.2141,  -9.4097,  -5.6444,  6.0862,  31.952,  -26.398,
     9.3863,  -20.114,  -7.4233,  13.158,  53.647,  -37.842,
     0.65112,  1.214,  3.3948,  -1.9974,  0.90818,  -5.9573,
     1.4808,  2.6885,  8.3624,  -4.4751,  2.6095,  -14.549,
     -2.2057,  -4.6445,  -22.331,  3.0432,  -15.496,  25.157,
     -0.14912,  2.6072,  -0.71691,  -2.6838,  -1.0183,  -2.4947,
     37.194,  5.8094,  -59.544,  -16.303,  30.141,  45.979,
     2.1167,  -1.0494,  -0.8874,  -0.20081,  6.2398,  -0.83448,
     -36.741,  49.532,  -146.07,  -71.232,  22.489,  183.66,
     -1.9561,  3.8866,  -14.519,  -6.5254,  6.8275,  14.635,
};

// ==== 基本运动（PID 增益）====
constexpr PidGains kLeftL0Pid{2500.0f, 0.f, 200.0f, 130.0f, 0.0f};   ///< 左腿腿长 PID（常规）
constexpr PidGains kRightL0Pid{2500.0f, 0.f, 200.0f, 130.0f, 0.0f};  ///< 右腿腿长 PID（常规）
constexpr PidGains kRollPid{800.0f, 0.f, 20.0f, 100.0f, 0.0f};      ///< 横滚平衡 PID

// ==== 跳跃（PID 增益）====
constexpr PidGains kLeftL0PidJumpTwo{6000.0f, 0.0f, 70000.0f, 150.0f, 0.0f};       ///< 左腿蹬伸 PID（JumpPush）
constexpr PidGains kRightL0PidJumpTwo{6000.0f, 0.0f, 70000.0f, 150.0f, 0.0f};      ///< 右腿蹬伸 PID（JumpPush）
constexpr PidGains kLeftL0PidJumpThree{8000.0f, 0.f, 80000.0f, 150.0f, 0.0f};  ///< 左腿回收 PID（JumpRecover）
constexpr PidGains kRightL0PidJumpThree{8000.0f, 0.f, 80000.0f, 150.0f, 0.0f};  ///< 右腿回收 PID（JumpRecover）

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{30.0f, 0.0f, 30.0f, 12.0f, 0.0f};   ///< 左腿摆角速度 PID（倒地恢复用）
constexpr PidGains kRightLegTurnPid{30.0f, 0.0f, 30.0f, 12.0f, 0.0f};  ///< 右腿摆角速度 PID（倒地恢复用）

// ==== 上台阶（腿摆角 PID）====
constexpr PidGains kStairClimbThetaPid{12.0f, 0.0f, 1.0f, 15.0f,
                                       0.0f};  ///< 上台阶腿摆角 PID（位置环，跟踪 kStairClimbThetaTargetRad）

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
constexpr float kTargetForwardSpeedMaxMps = 2.5f;         ///< 最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.3f;  ///< 高腿长模式最大前进速度 [m/s]
constexpr float kVxInputDeadbandNorm = 0.1f;              ///< 前进输入死区（归一化值，低于此忽略）
constexpr float kVyInputDeadbandNorm = 0.1f;              ///< 平移输入死区（归一化值）
constexpr float kYawFollowRampStepRadS = 0.1f;            ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.15f;  ///< 位置锚定冻结速度阈值 [m/s]（车速低于此值时锁定位置）

// ==== 落地减速（离地→落地时通过腿摆角辅助减速）====
constexpr float kLandingDecelThetaGain = 0.f;               ///< 落地减速腿摆角增益 [rad/(m/s)]
constexpr float kLandingDecelThetaMaxRad = 0.3f;            ///< 落地减速腿摆角最大偏置 [rad]
constexpr float kLandingDecelThetaRampStepRad = 0.01f;      ///< 落地减速腿摆角每周期斜坡步长 [rad/tick]
constexpr std::uint32_t kLandingDecelOffGroundMinMs = 100U;  ///< 离地最短持续时间（防单帧误判）[ms]
constexpr std::uint32_t kLandingDecelStableDurationMs = 400U;  ///< 落地减速稳定保持时间 [ms]

constexpr float kYawFollowFixedTargetRad = -1.9f;      ///< 偏航跟随固定目标偏置角 [rad]（前进方向）
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;  ///< 偏航跟随侧向目标偏置角 [rad]（±π/2）
constexpr PidGains kYawFollowPid{8.0f, 0.0f, 2.f, 4.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置（腿摆角/机体俯仰）====
constexpr float kExpectedThetaLlBiasRadLowLeg = -0.03f;  ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = -0.03f;  ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLeg = -0.2f;   ///< 中腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLeg = -0.2f;   ///< 中腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadHighLeg = -0.0f;  ///< 高腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadHighLeg = -0.0f;  ///< 高腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = -0.087f;         ///< 期望机体俯仰偏置 [rad]

// ==== 速度斜坡参数（按腿长档位分级）====
constexpr SdotRampParams kSdotRampLowLeg{0.008f, 0.008f};   ///< 低腿长速度斜坡（加速/制动步长）
constexpr SdotRampParams kSdotRampMidLeg{0.004f, 0.004f};   ///< 中腿长速度斜坡
constexpr SdotRampParams kSdotRampHighLeg{0.008f, 0.005f};  ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.1f;            ///< 小陀螺进入偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinExitYawRampStepRadS = 0.15f;       ///< 小陀螺退出偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS = 7.5f;           ///< 小陀螺目标自旋角速度 [rad/s]
constexpr float kSpinExitYawAlignThresholdRad = 0.15f;  ///< 小陀螺预测退出：yaw 对齐阈值 [rad]
constexpr float kSpinTranslationGain = 1.0f;  ///< 小陀螺平移增益（将云台系前进指令投影到车体系的比例）
constexpr float kSpinThetaLlBiasRad = 0.05f;  ///< 小陀螺时左腿摆角偏置 [rad]
constexpr float kSpinThetaLrBiasRad = -0.05f;  ///< 小陀螺时右腿摆角偏置 [rad]
constexpr float kSpinLegLengthBiasM = -0.0f;  ///< 小陀螺时腿长偏差（左+右-）[m]
constexpr float kSpinThetaBBiasRad = -0.1f;   ///< 小陀螺时俯仰目标偏置 [rad]

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
constexpr float kLeftPhi1OffsetRad = 2.8f+ M_PI ;          ///< 左腿前关节零位偏移 [rad]
constexpr float kLeftPhi4OffsetRad = 0.4f ;                 ///< 左腿后关节零位偏移 [rad]
constexpr float kRightPhi1OffsetRad = 2.1244f + M_PI + 0.13f-0.068f  ;  ///< 右腿前关节零位偏移 [rad]
constexpr float kRightPhi4OffsetRad = 0.46f + 0.123f+ 0.136 ;         ///< 右腿后关节零位偏移 [rad]
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
constexpr uint8_t kRobotId = 1U;                                          ///< 机器人 ID（裁判系统回退值）
constexpr float kBulletSpeedMps = 11.8f;                                  ///< 弹速 [m/s]（裁判系统回退值）
constexpr PidGains kYawPositionPid{36.0f, 0.2, 3.f, 1000.0f, 4.0f};    ///< 自瞄偏航位置 PID
constexpr PidGains kYawSpeedPid{0.7f, 0.0f, 0.0f, 10.0f, 0.3f};           ///< 自瞄偏航速度 PID
constexpr PidGains kPitchPositionPid{20.0f, 0.f, 3.0f, 1000.0f, 1.5f};  ///< 自瞄俯仰位置 PID
constexpr PidGains kPitchSpeedPid{1.f, 0.0f, 0.0f, 10.0f, 0.3f};         ///< 自瞄俯仰速度 PID
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

constexpr float kPitchMinRad = -0.3f;  ///< 俯仰角下限 [rad]
constexpr float kPitchMaxRad = 0.6f;   ///< 俯仰角上限 [rad]

inline constexpr PidGains kYawPositionPid{25.0f, 0.0f, 0.05f, 10.0f, 1.0f};    ///< 偏航位置 PID
inline constexpr PidGains kYawSpeedPid{0.6f, 0.0f, 0.0f, 6.0f, 0.4f};          ///< 偏航速度 PID
inline constexpr PidGains kPitchPositionPid{23.0f, 0.0f, 0.15f, 10.0f, 0.4f};  ///< 俯仰位置 PID
inline constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 8.0f, 0.0f};       ///< 俯仰速度 PID

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

// ── 发射机构（双摩擦轮 + M3508 拨盘）──
namespace shoot {
inline constexpr int kFrictionWheelCount = 2;                            ///< 摩擦轮数量
constexpr float kFricSpeedTargetRpm = 6900.0f;                           ///< 摩擦轮目标转速 [rpm]
constexpr PidGains kFricSpeedPid{20.0f, 1.0f, 0.0f, 16000.0f, 2000.0f};  ///< 摩擦轮速度 PID
constexpr PidGains kDialSpeedPid{15.0f, 0.f, 0.2f, 8000.0f, 0.0f};       ///< 拨盘速度 PID
constexpr PidGains kDialPositionPid{8.0f, 0.f, 0.0f, 1500.0f, 0.0f};     ///< 拨盘位置 PID
constexpr int16_t kDialFireThreshold = -600;                             ///< 发射触发拨轮阈值
constexpr float kShootFrequencyHz = 10.0f;                               ///< 发射频率 [Hz]
}  // namespace shoot

// ── 底盘状态机 ──
namespace chassis_fsm {

// ==== 上台阶 ====
constexpr float kStairClimbThetaThresholdRad = 0.5f;               ///< 双腿摆角均超过此值触发上台阶 [rad]
constexpr float kStairClimbLegLengthM = 0.18f;                     ///< 上台阶收腿目标腿长 [m]
constexpr float kStairClimbPhase0LegLengthM = 0.18f;               ///< 上台阶 Phase 0 转腿目标腿长 [m]
constexpr float kStairClimbThetaTargetRad = 0.85f;                 ///< 上台阶目标腿摆角（统一偏置） [rad]
constexpr std::uint32_t kStairClimbDurationMs = 220U;              ///< 上台阶最长持续时间 [ms]
constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.03f;  ///< 腿长到位容差 [m]
constexpr float kStairClimbThetaNearZeroThresholdRad = 0.12f;  ///< 摆角归零判定阈值（上台阶完成） [rad]
constexpr std::uint32_t kStairClimbPitchStableMs = 150U;       ///< 上台阶完成后俯仰稳定等待时间 [ms]

// ==== 倒地自启 ====
constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;        ///< 倒地确认时间 [ms]
constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;  ///< 自启超时 [ms]

// ==== 跳跃 ====
constexpr std::uint32_t kJumpLowPrepMs = 200U;      ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpLowPushMaxMs = 1000U;  ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpLowRecoverMs = 800U;   ///< 跳跃回收阶段持续时间（保底超时）[ms]
constexpr std::uint32_t kJumpLowRecoverMinMs = 450U;  ///< 跳跃回收阶段最低维持时间（此后开始判断离地）[ms]
constexpr float kJumpLowPrepLegLengthM = 0.13f;         ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpLowPushLegLengthM = 0.35f;         ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpLowRecoverLegLengthM = 0.15f;      ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpLowPushReachedLegLengthM = 0.33f;  ///< 蹬伸到位判定腿长 [m]

// ==== 跳跃（中腿长）====
constexpr std::uint32_t kJumpMidPrepMs = 200U;      ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpMidPushMaxMs = 1000U;  ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpMidRecoverMs = 800U;   ///< 跳跃回收阶段持续时间（保底超时）[ms]
constexpr std::uint32_t kJumpMidRecoverMinMs = 250U;  ///< 跳跃回收阶段最低维持时间（此后开始判断离地）[ms]
constexpr float kJumpMidPrepLegLengthM = 0.18f;         ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpMidPushLegLengthM = 0.27f;         ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpMidRecoverLegLengthM = 0.20f;      ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpMidPushReachedLegLengthM = 0.25f;  ///< 蹬伸到位判定腿长 [m]

// ==== 基本运动（腿长档位）====
constexpr float kLowLegLengthM = 0.15f;              ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.21f;              ///< 中腿长档位目标腿长 [m]
constexpr float kHighLegLengthM = 0.32f;             ///< 高腿长档位目标腿长 [m]
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
constexpr float kStandupThetaThresholdRad = 1.05f;  ///< 起立完成判定：双腿摆角绝对值低于此值后允许轮端输出 [rad]
constexpr float kPostureRollMinRad = -0.5f;          ///< 横滚角安全下限 [rad]
constexpr float kPostureRollMaxRad = 0.5f;           ///< 横滚角安全上限 [rad]
constexpr float kPostureThetaBMinRad = -0.7f;        ///< 机体俯仰角安全下限 [rad]
constexpr float kPostureThetaBMaxRad = 0.7f;         ///< 机体俯仰角安全上限 [rad]
constexpr float kPostureThetaLegMinRad = -0.8f;      ///< 腿摆角安全下限 [rad]
constexpr float kPostureThetaLegMaxRad = 1.4f;       ///< 腿摆角安全上限 [rad]
constexpr float kLegRecoverThetaDotTarget = -2.0f;   ///< 倒地恢复时腿摆角速度目标 [rad/s]
constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;  ///< 倒地恢复零力矩区间下限 [rad]
constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;  ///< 倒地恢复零力矩区间上限 [rad]

// -- 离地检测 --
constexpr float kOffGroundSupportForceThresholdN = 10.0f;  ///< 支撑力低于此值判定为离地 [N]
constexpr float kOffGroundSupportForceClampN = 110.0f;     ///< 离地时支持力限幅值 [N]

// ==== 物理参数（变体专属）====
constexpr float kBodyMassKg = 22.0f;  ///< 机体质量 [kg]

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.0126f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵）====
constexpr std::array<float, 240> kCtrlP{
    -4.767,  -31.725, 23.182,  43.951,  -14.193, -19.559, -7.1484,   -31.349,  31.414,   50.081,  -31.84,   -23.479,
    -4.8792, 16.549,  -7.2259, -19.136, 1.4887,  9.6495,  -3.1978,   11.903,   -6.0468,  -12.317, -0.69019, 8.5993,
    -12.473, -87.131, 18.972,  81.503,  10.397,  -25.449, -0.78461,  -6.827,   2.9136,   -4.3911, 3.1628,   -3.6628,
    -5.2484, 7.2365,  -12.577, 3.395,   -5.7454, 5.0888,  -0.46319,  -1.1722,  -0.86147, 5.078,   -10.418,  1.1182,
    -31.533, 41.16,   44.398,  7.5549,  -52.94,  -34.511, -3.1827,   2.5043,   7.601,    2.8829,  -8.1832,  -6.8124,
    -4.767,  23.182,  -31.725, -19.559, -14.193, 43.951,  -7.1484,   31.414,   -31.349,  -23.479, -31.84,   50.081,
    4.8792,  7.2259,  -16.549, -9.6495, -1.4887, 19.136,  3.1978,    6.0468,   -11.903,  -8.5993, 0.69019,  12.317,
    -5.2484, -12.577, 7.2365,  5.0888,  -5.7454, 3.395,   -0.46319,  -0.86147, -1.1722,  1.1182,  -10.418,  5.078,
    -12.473, 18.972,  -87.131, -25.449, 10.397,  81.503,  -0.78461,  2.9136,   -6.827,   -3.6628, 3.1628,   -4.3911,
    -31.533, 44.398,  41.16,   -34.511, -52.94,  7.5549,  -3.1827,   7.601,    2.5043,   -6.8124, -8.1832,  2.8829,
    13.949,  19.168,  -50.101, -84.023, 75.043,  28.61,   18.453,    16.958,   -70.156,  -96.768, 113.87,   37.475,
    -4.1722, -27.846, -11.995, 49.206,  -15.783, 18.131,  -2.7685,   -20.627,  -6.4871,  34.425,  -11.022,  8.9824,
    67.35,   -95.684, 6.9208,  82.14,   64.412,  -26.508, 3.2541,    2.2424,   -3.3646,  2.1775,  4.2895,   1.1179,
    -3.443,  -40.169, -9.3036, 43.835,  -34.304, -25.834, -0.084859, -0.12068, 3.8016,   -7.4019, 7.6654,   -14.172,
    -58.803, -288.49, 88.124,  320.23,  57.557,  -108.7,  -1.6898,   -19.571,  1.0562,   14.595,  14.539,   -3.4373,
    13.949,  -50.101, 19.168,  28.61,   75.043,  -84.023, 18.453,    -70.156,  16.958,   37.475,  113.87,   -96.768,
    4.1722,  11.995,  27.846,  -18.131, 15.783,  -49.206, 2.7685,    6.4871,   20.627,   -8.9824, 11.022,   -34.425,
    -3.443,  -9.3036, -40.169, -25.834, -34.304, 43.835,  -0.084859, 3.8016,   -0.12068, -14.172, 7.6654,   -7.4019,
    67.35,   6.9208,  -95.684, -26.508, 64.412,  82.14,   3.2541,    -3.3646,  2.2424,   1.1179,  4.2895,   2.1775,
    -58.803, 88.124,  -288.49, -108.7,  57.557,  320.23,  -1.6898,   1.0562,   -19.571,  -3.4373, 14.539,   14.595,
};

// ==== 基本运动（PID 增益）====
constexpr PidGains kLeftL0Pid{3000.0f, 0.f, 180.0f, 170.0f, 10.0f};   ///< 左腿腿长 PID（常规）
constexpr PidGains kRightL0Pid{3000.0f, 0.f, 180.0f, 170.0f, 10.0f};  ///< 右腿腿长 PID（常规）
constexpr PidGains kRollPid{800.0f, 0.0f, 200.0f, 180.0f, 0.0f};      ///< 横滚平衡 PID

// ==== 跳跃（PID 增益）====
constexpr PidGains kLeftL0PidJumpTwo{2000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};     ///< 左腿蹬伸 PID
constexpr PidGains kRightL0PidJumpTwo{2000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};    ///< 右腿蹬伸 PID
constexpr PidGains kLeftL0PidJumpThree{2000.0f, 0.f, 50000.0f, 170.0f, 30.0f};   ///< 左腿回收 PID
constexpr PidGains kRightL0PidJumpThree{2000.0f, 0.f, 50000.0f, 170.0f, 30.0f};  ///< 右腿回收 PID

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};   ///< 左腿摆角速度 PID
constexpr PidGains kRightLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};  ///< 右腿摆角速度 PID

// ==== 上台阶（腿摆角 PID）====
constexpr PidGains kStairClimbThetaPid{40.0f, 0.0f, 8.0f, 60.0f,
                                       30.0f};  ///< 上台阶腿摆角 PID（位置环，跟踪 kStairClimbThetaTargetRad）

// ==== 跳跃蹬伸 Theta PID ====
constexpr PidGains kJumpThetaPid{150.0f, 0.0f, 50.0f, 250.0f, 40.0f};  ///< 跳跃蹬伸腿摆角 PID（位置环，目标 0 rad）

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
constexpr std::uint16_t kAutoJumpDistanceThresholdMm = 850U;  ///< 自动跳跃 DYP 测距阈值 [mm]
constexpr float kAutoJumpHoldTimeS = 1.0f;                    ///< 自动跳跃开关拨轮保持时间 [s]
constexpr float kAutoJumpDistanceHoldTimeS = 0.05f;           ///< 自动跳跃测距持续低于阈值判定时间 [s]

constexpr float kControlLoopDtS = 0.002f;  ///< 控制环周期 [s]

// -- 摇杆/键鼠输入归一化 --
constexpr std::int16_t kDr16AxisMaxAbs = 660;       ///< DR16 摇杆轴最大绝对值
constexpr float kRcStickMax = 660.0f;               ///< RC 摇杆最大值
constexpr float kTcMouseMax = 200.0f;               ///< 图传鼠标增量最大值
constexpr float kRcYawRateMaxRadS = -3.5f;          ///< RC 摇杆满偏时偏航积分速率 [rad/s]
constexpr float kRcPitchRateMaxRadS = 2.5f;         ///< RC 摇杆满偏时俯仰积分速率 [rad/s]
constexpr float kTcMouseYawRateMaxRadS = -2.0f;     ///< 图传鼠标满偏时偏航积分速率 [rad/s]
constexpr float kTcMousePitchRateMaxRadS = 1.0f;    ///< 图传鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kDr16MouseMax = 1600.0f;            ///< DR16 鼠标增量最大值（用于积分目标速率计算）
constexpr float kDr16MouseYawRateMaxRadS = -2.0f;   ///< DR16 鼠标满偏时偏航积分速率 [rad/s]
constexpr float kDr16MousePitchRateMaxRadS = 1.0f;  ///< DR16 鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kPitchTargetMinRad = -0.3f;         ///< RC 积分俯仰目标下限 [rad]
constexpr float kPitchTargetMaxRad = 0.65f;         ///< RC 积分俯仰目标上限 [rad]

// -- 云台启动归中判稳 --
constexpr float kGimbalStartupYawAlignErrorRad = 0.04f;           ///< 归中完成位置误差阈值 [rad]
constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;            ///< 归中完成速度阈值 [rad/s]
constexpr std::uint32_t kGimbalStartupYawAlignStableTicks = 10U;  ///< 归中判稳所需连续满足周期数

// -- 偏航就绪判稳 --
constexpr float kYawFollowDriveReadyErrorRad = 0.04f;           ///< 偏航就绪位置误差阈值 [rad]
constexpr float kYawFollowDriveReadyVelRadS = 0.25f;            ///< 偏航就绪速度阈值 [rad/s]
constexpr std::uint32_t kYawFollowDriveReadyStableTicks = 50U;  ///< 偏航就绪判稳所需连续周期数

// ==== 基本运动 ====
constexpr float kTargetForwardSpeedMaxMps = 1.9f;         ///< 最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.1f;  ///< 高腿长 3模式最大前进速度 [m/s]
constexpr float kMaxSafeYawRateRadS = 4.5f;               ///< 摩擦圆最大安全偏航速率 [rad/s]
constexpr float kVxInputDeadbandNorm = 0.05f;             ///< 前进输入死区
constexpr float kVyInputDeadbandNorm = 0.05f;             ///< 平移输入死区
constexpr float kYawFollowRampStepRadS = 0.05f;           ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.3f;  ///< 位置锚定冻结速度阈值 [m/s]

// ==== 落地减速（离地→落地时通过腿摆角辅助减速）====
constexpr float kLandingDecelThetaGain = -0.2f;               ///< 落地减速腿摆角增益 [rad/(m/s)]
constexpr float kLandingDecelThetaMaxRad = 0.3f;              ///< 落地减速腿摆角最大偏置 [rad]
constexpr float kLandingDecelThetaRampStepRad = 0.008f;       ///< 落地减速腿摆角每周期斜坡步长 [rad/tick]
constexpr std::uint32_t kLandingDecelOffGroundMinMs = 40U;    ///< 离地最短持续时间（防单帧误判）[ms]
constexpr std::uint32_t kLandingDecelStableDurationMs = 50U;  ///< 落地减速稳定保持时间 [ms]

constexpr float kYawFollowFixedTargetRad = 0.38f;                 ///< 偏航跟随固定目标偏置角 [rad]
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;             ///< 偏航跟随侧向目标偏置角 [rad]
constexpr PidGains kYawFollowPid{20.0f, 0.0f, 10.f, 6.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置 ====
constexpr float kExpectedThetaLlBiasRadLowLeg = 0.09f;   ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = 0.09f;   ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLeg = 0.02f;   ///< 中腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLeg = 0.02f;   ///< 中腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadHighLeg = 0.06f;  ///< 高腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadHighLeg = 0.06f;  ///< 高腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = 0.f;            ///< 期望机体俯仰偏置 [rad]

// ==== 速度斜坡参数 ====
constexpr SdotRampParams kSdotRampLowLeg{0.0065f, 0.0065f};  ///< 低腿长速度斜坡
constexpr SdotRampParams kSdotRampMidLeg{0.007f, 0.007f};    ///< 中腿长速度斜坡
constexpr SdotRampParams kSdotRampHighLeg{0.005f, 0.005f};   ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.05f;           ///< 小陀螺进入偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinExitYawRampStepRadS = 0.05f;       ///< 小陀螺退出偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS = 6.5f;           ///< 小陀螺目标自旋角速度 [rad/s]
constexpr float kSpinExitYawAlignThresholdRad = 0.15f;  ///< 小陀螺预测退出：yaw 对齐阈值 [rad]
constexpr float kSpinTranslationGain = 1.0f;  ///< 小陀螺平移增益（系数2补偿 cos² 平均衰减，使平均车速=摇杆指令值）
constexpr float kSpinThetaLlBiasRad = 0.09f;  ///< 小陀螺时左腿摆角偏置 [rad]
constexpr float kSpinThetaLrBiasRad = 0.13f;  ///< 小陀螺时右腿摆角偏置 [rad]
constexpr float kSpinLegLengthBiasM = 0.0f;   ///< 小陀螺时腿长偏差（左+右-）[m]
constexpr float kSpinThetaBBiasRad = 0.0f;    ///< 小陀螺时俯仰目标偏置 [rad]

// ==== 跳跃腿摆角偏置 ====
constexpr float kJumpThetaLlBiasRad = 0.15f;  ///< 跳跃时左腿摆角偏置 [rad]
constexpr float kJumpThetaLrBiasRad = 0.15f;  ///< 跳跃时右腿摆角偏置 [rad]
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
constexpr uint8_t kRobotId = 3U;                                        ///< 机器人 ID
constexpr float kBulletSpeedMps = 23.0f;                                ///< 弹速 [m/s]
constexpr PidGains kYawPositionPid{20.0f, 0.5f, 0.05f, 10.0f, 2.0f};    ///< 自瞄偏航位置 PID
constexpr PidGains kYawSpeedPid{0.5f, 0.0f, 0.0f, 6.0f, 0.3f};          ///< 自瞄偏航速度 PID
constexpr PidGains kPitchPositionPid{22.0f, 0.5f, 0.05f, 10.0f, 1.5f};  ///< 自瞄俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.45f, 0.0f, 0.0f, 8.0f, 0.3f};       ///< 自瞄俯仰速度 PID
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

constexpr std::uint16_t kFricLeftId = 0x04;   ///< 左摩擦轮电机 CAN ID
constexpr std::uint16_t kFricRightId = 0x01;  ///< 右摩擦轮电机 CAN ID
constexpr std::uint16_t kDialId = 0x03;       ///< 拨盘电机 CAN ID
}  // namespace globals

// ── 云台 ──
namespace gimbal {
using namespace common::gimbal;

constexpr float kPitchMinRad = -0.35f;  ///< 俯仰角下限 [rad]
constexpr float kPitchMaxRad = 0.65f;   ///< 俯仰角上限 [rad]

constexpr PidGains kYawPositionPid{25.0f, 0.0f, 0.05f, 10.0f, 1.0f};   ///< 偏航位置 PID
constexpr PidGains kYawSpeedPid{0.6f, 0.0f, 0.0f, 6.0f, 0.4f};         ///< 偏航速度 PID
constexpr PidGains kPitchPositionPid{30.0f, 0.0f, 0.8f, 10.0f, 0.4f};  ///< 俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 8.0f, 0.0f};      ///< 俯仰速度 PID

/// @brief 辨识得到的 9 个动力学参数（theta_0 ~ theta_8），用于前馈验证
constexpr float kIdentTheta[9] = {
    0.01840039f,   // theta_0: I1zz_com
    0.01493814f,   // theta_1: I2xx_com
    0.03804828f,   // theta_2: I2yy_com
    -0.10281495f,  // theta_3: m2*l2x 水平前向偏心
    -0.13614424f,  // theta_4: m2*l2z 垂直上向偏心
    0.13102762f,   // theta_5: fv1  yaw 粘滞摩擦
    0.52290111f,   // theta_6: fc1  yaw 库仑摩擦
    0.89830570f,   // theta_7: fv2  pitch 粘滞摩擦
    0.04230919f,   // theta_8: fc2  pitch 库仑摩擦
};
}  // namespace gimbal

// ── 发射机构（双摩擦轮 + M3508 拨盘）──
namespace shoot {
inline constexpr int kFrictionWheelCount = 2;                            ///< 摩擦轮数量
constexpr float kFricSpeedTargetRpm = 6000.0f;                           ///< 摩擦轮目标转速 [rpm]
constexpr PidGains kFricSpeedPid{20.0f, 1.0f, 0.0f, 16000.0f, 2000.0f};  ///< 摩擦轮速度 PID
constexpr PidGains kDialSpeedPid{10.0f, 0.5f, 0.0f, 16000.0f, 1000.0f};  ///< 拨盘速度 PID
constexpr PidGains kDialPositionPid{5.0f, 0.1f, 0.0f, 1500.0f, 500.0f};  ///< 拨盘位置 PID
constexpr int16_t kDialFireThreshold = -600;                             ///< 发射触发拨轮阈值
constexpr float kShootFrequencyHz = 24.0f;                               ///< 发射频率 [Hz]
}  // namespace shoot

// ── 底盘状态机 ──
namespace chassis_fsm {

// ==== 上台阶 ====
constexpr float kStairClimbThetaThresholdRad = 0.5f;               ///< 双腿摆角均超过此值触发上台阶 [rad]
constexpr float kStairClimbLegLengthM = 0.15f;                     ///< 上台阶收腿目标腿长 [m]
constexpr float kStairClimbPhase0LegLengthM = 0.35f;               ///< 上台阶 Phase 0 转腿目标腿长 [m]
constexpr float kStairClimbThetaTargetRad = 0.8f;                  ///< 上台阶目标腿摆角 [rad]
constexpr std::uint32_t kStairClimbDurationMs = 200U;              ///< 上台阶最长持续时间 [ms]
constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.02f;  ///< 腿长到位容差 [m]
constexpr float kStairClimbThetaNearZeroThresholdRad = 0.1f;       ///< 摆角归零判定阈值 [rad]
constexpr std::uint32_t kStairClimbPitchStableMs = 450U;           ///< 上台阶完成后俯仰稳定等待时间 [ms]

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

// ==== 跳跃（中腿长）====
constexpr std::uint32_t kJumpMidPrepMs = 250U;      ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpMidPushMaxMs = 1000U;  ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpMidRecoverMs = 300U;   ///< 跳跃回收阶段持续时间（保底超时）[ms]
constexpr std::uint32_t kJumpMidRecoverMinMs = 50U;  ///< 跳跃回收阶段最低维持时间（此后开始判断离地）[ms]
constexpr float kJumpMidPrepLegLengthM = 0.18f;         ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpMidPushLegLengthM = 0.23f;         ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpMidRecoverLegLengthM = 0.20f;      ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpMidPushReachedLegLengthM = 0.23f;  ///< 蹬伸到位判定腿长 [m]

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
constexpr float kStandupThetaThresholdRad = 0.8f;    ///< 起立完成判定 [rad]
constexpr float kPostureRollMinRad = -0.5f;          ///< 横滚角安全下限 [rad]
constexpr float kPostureRollMaxRad = 0.5f;           ///< 横滚角安全上限 [rad]
constexpr float kPostureThetaBMinRad = -0.7f;        ///< 机体俯仰角安全下限 [rad]
constexpr float kPostureThetaBMaxRad = 0.7f;         ///< 机体俯仰角安全上限 [rad]
constexpr float kPostureThetaLegMinRad = -0.8f;      ///< 腿摆角安全下限 [rad]
constexpr float kPostureThetaLegMaxRad = 1.4f;       ///< 摆角安全上限 [rad]
constexpr float kLegRecoverThetaDotTarget = -2.0f;   ///< 倒地恢复时腿摆角速度目标 [rad/s]
constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;  ///< 倒地恢复零力矩区间下限 [rad]
constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;  ///< 倒地恢复零力矩区间上限 [rad]

// -- 离地检测 --
constexpr float kOffGroundSupportForceThresholdN = 20.0f;  ///< 支撑力低于此值判定为离地 [N]
constexpr float kOffGroundSupportForceClampN = 30.0f;      ///< 离地时支持力限幅值 [N]

// ==== 物理参数（变体专属）====
constexpr float kBodyMassKg = 22.0f;  ///< 机体质量 [kg]

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.002f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵）====
constexpr std::array<float, 240> kCtrlP{
    -1.9104,  -15.059, 11.777,   22.4,     -10.444, -8.4635,  -5.2372,  -30.334,  28.645,   49.172,  -31.155, -19.401,
    -0.50616, 2.0751,  -1.0044,  -1.7406,  -0.4023, 1.5987,   -1.621,   6.9517,   -3.5894,  -5.4714, -1.9668, 5.8526,
    -13.545,  -96.254, 14.843,   98.165,   7.9028,  -20.044,  -0.74956, -6.4014,  2.5235,   -4.1887, 2.9712,  -3.1051,
    -4.3442,  2.0527,  -3.6762,  15.784,   -14.7,   -0.62633, -0.37558, -1.4608,  -0.14065, 5.664,   -11.1,   1.5366,
    -28.704,  31.244,  40.762,   17.927,   -42.888, -37.269,  -2.8887,  1.7945,   6.9762,   3.3742,  -6.9911, -6.6081,
    -1.9104,  11.777,  -15.059,  -8.4635,  -10.444, 22.4,     -5.2372,  28.645,   -30.334,  -19.401, -31.155, 49.172,
    0.50616,  1.0044,  -2.0751,  -1.5987,  0.4023,  1.7406,   1.621,    3.5894,   -6.9517,  -5.8526, 1.9668,  5.4714,
    -4.3442,  -3.6762, 2.0527,   -0.62633, -14.7,   15.784,   -0.37558, -0.14065, -1.4608,  1.5366,  -11.1,   5.664,
    -13.545,  14.843,  -96.254,  -20.044,  7.9028,  98.165,   -0.74956, 2.5235,   -6.4014,  -3.1051, 2.9712,  -4.1887,
    -28.704,  40.762,  31.244,   -37.269,  -42.888, 17.927,   -2.8887,  6.9762,   1.7945,   -6.6081, -6.9911, 3.3742,
    5.9408,   5.6583,  -16.512,  -40.193,  33.14,   10.997,   15.03,    10.845,   -47.194,  -92.07,  92.469,  29.518,
    -0.51586, -3.7901, -0.95708, 6.0945,   -1.8254, 1.6974,   -1.6606,  -12.851,  -2.5254,  20.319,  -6.0233, 4.5513,
    84.651,   -140.17, 9.4216,   87.295,   49.369,  -25.573,  3.4046,   1.7896,   -2.5341,  -3.3677, 6.7898,  0.78891,
    -3.5749,  -35.826, -2.7377,  30.206,   -28.458, 6.5997,   -0.19616, 0.74305,  3.6914,   -8.6711, 3.3687,  -6.1628,
    -50.011,  -291,    88.009,   331.3,    56.761,  -123.07,  -1.692,   -20.091,  3.0431,   16.546,  12.726,  -6.4838,
    5.9408,   -16.512, 5.6583,   10.997,   33.14,   -40.193,  15.03,    -47.194,  10.845,   29.518,  92.469,  -92.07,
    0.51586,  0.95708, 3.7901,   -1.6974,  1.8254,  -6.0945,  1.6606,   2.5254,   12.851,   -4.5513, 6.0233,  -20.319,
    -3.5749,  -2.7377, -35.826,  6.5997,   -28.458, 30.206,   -0.19616, 3.6914,   0.74305,  -6.1628, 3.3687,  -8.6711,
    84.651,   9.4216,  -140.17,  -25.573,  49.369,  87.295,   3.4046,   -2.5341,  1.7896,   0.78891, 6.7898,  -3.3677,
    -50.011,  88.009,  -291,     -123.07,  56.761,  331.3,    -1.692,   3.0431,   -20.091,  -6.4838, 12.726,  16.546,
};

// ==== 基本运动（PID 增益）====
constexpr PidGains kLeftL0Pid{2000.0f, 0.f, 200.0f, 180.0f, 30.0f};   ///< 左腿腿长 PID（常规）
constexpr PidGains kRightL0Pid{2000.0f, 0.f, 200.0f, 180.0f, 30.0f};  ///< 右腿腿长 PID（常规）
constexpr PidGains kRollPid{800.0f, 0.0f, 200.0f, 80.0f, 0.0f};       ///< 横滚平衡 PID

// ==== 跳跃（PID 增益）====
constexpr PidGains kLeftL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 180.0f, 0.0f};       ///< 左腿蹬伸 PID
constexpr PidGains kRightL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 180.0f, 0.0f};      ///< 右腿蹬伸 PID
constexpr PidGains kLeftL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};   ///< 左腿回收 PID
constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};  ///< 右腿回收 PID

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{20.0f, 0.0f, 0.0f, 15.0f, 0.0f};   ///< 左腿摆角速度 PID
constexpr PidGains kRightLegTurnPid{20.0f, 0.0f, 0.0f, 15.0f, 0.0f};  ///< 右腿摆角速度 PID

// ==== 上台阶（腿摆角 PID）====
constexpr PidGains kStairClimbThetaPid{6.0f, 0.0f, 1.0f, 15.0f,
                                       0.0f};  ///< 上台阶腿摆角 PID（位置环，跟踪 kStairClimbThetaTargetRad）

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
constexpr float kRcYawRateMaxRadS = -2.5f;          ///< RC 摇杆满偏时偏航积分速率 [rad/s]
constexpr float kRcPitchRateMaxRadS = 1.5f;         ///< RC 摇杆满偏时俯仰积分速率 [rad/s]
constexpr float kTcMouseYawRateMaxRadS = -2.0f;     ///< 图传鼠标满偏时偏航积分速率 [rad/s]
constexpr float kTcMousePitchRateMaxRadS = 1.5f;    ///< 图传鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kDr16MouseMax = 1600.0f;            ///< DR16 鼠标增量最大值（用于积分目标速率计算）
constexpr float kDr16MouseYawRateMaxRadS = -2.0f;   ///< DR16 鼠标满偏时偏航积分速率 [rad/s]
constexpr float kDr16MousePitchRateMaxRadS = 1.5f;  ///< DR16 鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kPitchTargetMinRad = -0.35f;        ///< RC 积分俯仰目标下限 [rad]
constexpr float kPitchTargetMaxRad = 0.6f;          ///< RC 积分俯仰目标上限 [rad]

// -- 云台启动归中判稳 --
constexpr float kGimbalStartupYawAlignErrorRad = 0.04f;           ///< 归中完成位置误差阈值 [rad]
constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;            ///< 归中完成速度阈值 [rad/s]
constexpr std::uint32_t kGimbalStartupYawAlignStableTicks = 50U;  ///< 归中判稳所需连续满足周期数

// -- 偏航就绪判稳 --
constexpr float kYawFollowDriveReadyErrorRad = 0.04f;           ///< 偏航就绪位置误差阈值 [rad]
constexpr float kYawFollowDriveReadyVelRadS = 0.25f;            ///< 偏航就绪速度阈值 [rad/s]
constexpr std::uint32_t kYawFollowDriveReadyStableTicks = 50U;  ///< 偏航就绪判稳所需连续周期数

// ==== 基本运动 ====
constexpr float kTargetForwardSpeedMaxMps = 2.2f;         ///< 最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.3f;  ///< 高腿长模式最大前进速度 [m/s]
constexpr float kMaxSafeYawRateRadS = 4.0f;               ///< 摩擦圆最大安全偏航速率 [rad/s]
constexpr float kVxInputDeadbandNorm = 0.1f;              ///< 前进输入死区
constexpr float kVyInputDeadbandNorm = 0.1f;              ///< 平移输入死区
constexpr float kYawFollowRampStepRadS = 0.06f;           ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.3f;  ///< 位置锚定冻结速度阈值 [m/s]

// ==== 落地减速（离地→落地时通过腿摆角辅助减速）====
constexpr float kLandingDecelThetaGain = 0.f;               ///< 落地减速腿摆角增益 [rad/(m/s)]
constexpr float kLandingDecelThetaMaxRad = 0.3f;            ///< 落地减速腿摆角最大偏置 [rad]
constexpr float kLandingDecelThetaRampStepRad = 0.01f;      ///< 落地减速腿摆角每周期斜坡步长 [rad/tick]
constexpr std::uint32_t kLandingDecelOffGroundMinMs = 60U;  ///< 离地最短持续时间（防单帧误判）[ms]
constexpr std::uint32_t kLandingDecelStableDurationMs = 400U;  ///< 落地减速稳定保持时间 [ms]

constexpr float kYawFollowFixedTargetRad = 0.f;                  ///< 偏航跟随固定目标偏置角 [rad]
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;            ///< 偏航跟随侧向目标偏置角 [rad]
constexpr PidGains kYawFollowPid{8.2f, 0.0f, 1.2f, 6.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置 ====
constexpr float kExpectedThetaLlBiasRadLowLeg = 0.02f;   ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = 0.02f;   ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLeg = 0.01f;   ///< 中腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLeg = 0.01f;   ///< 中腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadHighLeg = 0.02f;  ///< 高腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadHighLeg = 0.02f;  ///< 高腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = 0.024f;         ///< 期望机体俯仰偏置 [rad]

// ==== 速度斜坡参数 ====
constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.008f};    ///< 低腿长速度斜坡
constexpr SdotRampParams kSdotRampMidLeg{0.008f, 0.008f};   ///< 中腿长速度斜坡
constexpr SdotRampParams kSdotRampHighLeg{0.005f, 0.005f};  ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.05f;           ///< 小陀螺进入偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinExitYawRampStepRadS = 0.15f;       ///< 小陀螺退出偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS = 8.5f;           ///< 小陀螺目标自旋角速度 [rad/s]
constexpr float kSpinExitYawAlignThresholdRad = 0.15f;  ///< 小陀螺预测退出：yaw 对齐阈值 [rad]
constexpr float kSpinTranslationGain = 1.2f;            ///< 小陀螺平移增益
constexpr float kSpinThetaLlBiasRad = 0.03f;            ///< 小陀螺时左腿摆角偏置 [rad]
constexpr float kSpinThetaLrBiasRad = 0.03f;            ///< 小陀螺时右腿摆角偏置 [rad]
constexpr float kSpinLegLengthBiasM = 0.0f;             ///< 小陀螺时腿长偏差（左+右-）[m]
constexpr float kSpinThetaBBiasRad = 0.02f;             ///< 小陀螺时俯仰目标偏置 [rad]

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
constexpr uint8_t kRobotId = 4U;                                        ///< 机器人 ID
constexpr float kBulletSpeedMps = 23.0f;                                ///< 弹速 [m/s]
constexpr PidGains kYawPositionPid{20.0f, 0.5f, 0.05f, 10.0f, 2.0f};    ///< 自瞄偏航位置 PID
constexpr PidGains kYawSpeedPid{0.5f, 0.0f, 0.0f, 6.0f, 0.3f};          ///< 自瞄偏航速度 PID
constexpr PidGains kPitchPositionPid{22.0f, 0.5f, 0.05f, 10.0f, 1.5f};  ///< 自瞄俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.45f, 0.0f, 0.0f, 8.0f, 0.3f};       ///< 自瞄俯仰速度 PID
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
