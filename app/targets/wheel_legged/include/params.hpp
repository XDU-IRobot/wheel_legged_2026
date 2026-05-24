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

// ==== 上台阶 ====
constexpr float kStairClimbThetaThresholdRad = 0.4f;               ///< 双腿摆角均超过此值触发上台阶 [rad]
constexpr float kStairClimbLegLengthM = 0.13f;                     ///< 上台阶收腿目标腿长 [m]
constexpr float kStairClimbPhase0LegLengthM = 0.13f;               ///< 上台阶 Phase 0 转腿目标腿长 [m]
constexpr float kStairClimbThetaTargetRad = 1.35f;                 ///< 上台阶目标腿摆角（统一偏置） [rad]
constexpr std::uint32_t kStairClimbDurationMs = 550U;              ///< 上台阶最长持续时间 [ms]
constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.01f;  ///< 腿长到位容差 [m]
constexpr float kStairClimbThetaNearZeroThresholdRad = 0.2f;  ///< 摆角归零判定阈值（上台阶完成） [rad]
constexpr std::uint32_t kStairClimbPitchStableMs = 1000U;     ///< 上台阶完成后俯仰稳定等待时间 [ms]

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
constexpr std::uint32_t kJumpMidPrepMs = 60U;       ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpMidPushMaxMs = 1000U;  ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpMidRecoverMs = 250U;   ///< 跳跃回收阶段持续时间（保底超时）[ms]
constexpr std::uint32_t kJumpMidRecoverMinMs = 100U;  ///< 跳跃回收阶段最低维持时间（此后开始判断离地）[ms]
constexpr float kJumpMidPrepLegLengthM = 0.18f;          ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpMidPushLegLengthM = 0.29f;          ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpMidRecoverLegLengthM = 0.18f;       ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpMidPushReachedLegLengthM = 0.275f;  ///< 蹬伸到位判定腿长 [m]

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
constexpr float kManualRecoveryLegSpeedRadS = 1.2f;  ///< 手动倒地恢复腿摆角速度 [rad/s]
constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;  ///< 倒地恢复零力矩区间下限 [rad]
constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;  ///< 倒地恢复零力矩区间上限 [rad]

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

// ==== 上台阶（腿摆角 PID）====
constexpr PidGains kStairClimbThetaPid{15.0f, 0.0f, 5.0f, 15.0f,
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
constexpr float kTargetForwardSpeedMaxMps = 2.15f;        ///< 最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.3f;  ///< 高腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxMidLegMps = 1.0f;   ///< G键中腿长模式最大前进速度 [m/s]
constexpr float kTargetSpeedBiasLowLegMps = 0.0f;         ///< 低腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegMps = 0.0f;         ///< C键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegGMps = 0.0f;        ///< G键中腿长目标速度偏置 [m/s]
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
constexpr SdotRampParams kSdotRampMidLegG{0.006f, 0.006f};  ///< 中腿长速度斜坡（G 键触发）
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
constexpr float kFricSpeedTargetRpm = 6800.0f;                           ///< 摩擦轮目标转速 [rpm]
constexpr PidGains kFricSpeedPid{20.0f, 1.0f, 0.0f, 16000.0f, 2000.0f};  ///< 摩擦轮速度 PID
constexpr PidGains kDialSpeedPid{10.0f, 0.5f, 0.f, 16000.0f, 0.0f};      ///< 拨盘速度 PID
constexpr PidGains kDialPositionPid{0.5f, 0.f, 0.0f, 16000.0f, 0.0f};    ///< 拨盘位置 PID
constexpr int16_t kDialFireThreshold = -600;                             ///< 发射触发拨轮阈值
constexpr float kShootFrequencyHz = 5.0f;                                ///< 发射频率 [Hz] ///< 发射频率 [Hz]
constexpr float kFricSpeedStepRpm = 20.0f;  ///< Z/X 键每次调整摩擦轮转速步长 [rpm]

// ── 本地热量闭环 ──
constexpr float kHeatPerShot = 10.0f;       ///< 每发子弹热量增量 [热量单位]
constexpr float kHeatSafetyMargin = 20.0f;  ///< 停火余量：heat + kHeatPerShot > limit - margin 时抑制发射
constexpr float kHeatResumeMargin = 20.0f;  ///< 恢复余量：heat < limit - margin 时恢复，与停火线构成迟滞
constexpr uint16_t kDefaultHeatLimit = 240;   ///< 裁判系统离线时默认热量上限
constexpr uint16_t kDefaultCoolingRate = 40;  ///< 裁判系统离线时默认冷却速率 [热量单位/秒]
}  // namespace shoot

// ── 底盘状态机 ──
namespace chassis_fsm {

// ==== 上台阶 ====
constexpr float kStairClimbThetaThresholdRad = 0.5f;               ///< 双腿摆角均超过此值触发上台阶 [rad]
constexpr float kStairClimbLegLengthM = 0.18f;                     ///< 上台阶收腿目标腿长 [m]
constexpr float kStairClimbPhase0LegLengthM = 0.33f;               ///< 上台阶 Phase 0 转腿目标腿长 [m]
constexpr float kStairClimbThetaTargetRad = 0.85f;                 ///< 上台阶目标腿摆角（统一偏置） [rad]
constexpr std::uint32_t kStairClimbDurationMs = 220U;              ///< 上台阶最长持续时间 [ms]
constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.03f;  ///< 腿长到位容差 [m]
constexpr float kStairClimbThetaNearZeroThresholdRad = 0.12f;  ///< 摆角归零判定阈值（上台阶完成） [rad]
constexpr std::uint32_t kStairClimbPitchStableMs = 450U;       ///< 上台阶完成后俯仰稳定等待时间 [ms]

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
constexpr float kLowLegLengthM = 0.14f;              ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.20f;              ///< 中腿长档位目标腿长 [m]
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
constexpr float kPostureRollMinRad = -0.5f;          ///< 横滚角安全下限 [rad]
constexpr float kPostureRollMaxRad = 0.5f;           ///< 横滚角安全上限 [rad]
constexpr float kPostureThetaBMinRad = -0.7f;        ///< 机体俯仰角安全下限 [rad]
constexpr float kPostureThetaBMaxRad = 0.7f;         ///< 机体俯仰角安全上限 [rad]
constexpr float kPostureThetaLegMinRad = -0.8f;      ///< 腿摆角安全下限 [rad]
constexpr float kPostureThetaLegMaxRad = 1.45f;      ///< 腿摆角安全上限 [rad]
constexpr float kLegRecoverThetaDotTarget = -2.0f;   ///< 倒地恢复时腿摆角速度目标 [rad/s]
constexpr float kManualRecoveryLegSpeedRadS = 0.5f;  ///< 手动倒地恢复腿摆角速度 [rad/s]
constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;  ///< 倒地恢复零力矩区间下限 [rad]
constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;  ///< 倒地恢复零力矩区间上限 [rad]

// -- 离地检测 --
constexpr float kOffGroundSupportForceThresholdN = 20.0f;  ///< 支撑力低于此值判定为离地 [N]
constexpr float kOffGroundSupportForceClampN = 100.0f;     ///< 离地时支持力限幅值 [N]

// -- 中腿长下压 --
constexpr float kMidLegDipTriggerLengthM = 0.27f;  ///< 中腿长模式下触发下压的腿长阈值 [m]
constexpr float kMidLegDipTargetLengthM = 0.18f;   ///< 下压目标腿长 [m]
constexpr uint16_t kMidLegDipHoldTicks = 1000;     ///< 下压维持时间 [ticks @ 500Hz = 1s]

// ==== 物理参数（变体专属）====

constexpr float kBodyMassKg = 22.0f;  ///< 机体质量 [kg]

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.0126f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵）====
constexpr std::array<float, 240> kCtrlPLow{
    -2.8738,  -25.531, 19.435,   38.814,  -17.922,  -14.007, -4.5018,   -25.911,  25.237,   43.021,  -28.637,  -17.155,
    -0.364,   1.6136,  -0.74551, -1.3902, -0.13605, 1.2222,  -1.174,    5.3865,   -2.6429,  -4.4343, -0.75261, 4.3969,
    -8.5608,  -70.706, 14.811,   66.758,  4.6985,   -19.552, -0.67071,  -5.3412,  2.2422,   -2.4914, 2.067,    -2.6791,
    -3.339,   2.4674,  -7.1034,  8.0791,  -10.92,   12.178,  -0.32482,  -1.086,   -0.10843, 4.4641,  -8.7343,  1.8098,
    -19.686,  21.331,  29.743,   13.917,  -31.289,  -27.69,  -2.37,     1.4437,   5.874,    3.0987,  -6.0086,  -5.6544,
    -2.8738,  19.435,  -25.531,  -14.007, -17.922,  38.814,  -4.5018,   25.237,   -25.911,  -17.155, -28.637,  43.021,
    0.364,    0.74551, -1.6136,  -1.2222, 0.13605,  1.3902,  1.174,     2.6429,   -5.3865,  -4.3969, 0.75261,  4.4343,
    -3.339,   -7.1034, 2.4674,   12.178,  -10.92,   8.0791,  -0.32482,  -0.10843, -1.086,   1.8098,  -8.7343,  4.4641,
    -8.5608,  14.811,  -70.706,  -19.552, 4.6985,   66.758,  -0.67071,  2.2422,   -5.3412,  -2.6791, 2.067,    -2.4914,
    -19.686,  29.743,  21.331,   -27.69,  -31.289,  13.917,  -2.37,     5.874,    1.4437,   -5.6544, -6.0086,  3.0987,
    9.9177,   3.526,   -23.358,  -61.078, 56.015,   14.347,  13.522,    2.7255,   -39.489,  -73.739, 85.089,   23.584,
    -0.35371, -3.1021, -0.688,   5.1741,  -1.4145,  1.214,   -1.1383,   -10.516,  -1.9609,  17.386,  -4.9099,  3.5054,
    50.302,   -57.752, 2.2937,   7.9872,  47.998,   -16.433, 3.092,     0.17832,  -2.2806,  -3.4084, 7.3969,   0.53664,
    -0.55742, -31.421, -9.2266,  29.855,  -16.201,  2.85,    -0.018638, -0.41975, 2.7871,   -5.3465, 1.9087,   -4.8759,
    -31.905,  -196.55, 54.291,   220.39,  45.917,   -78.709, -1.1708,   -17.34,   2.2834,   14.416,  11.478,   -5.3529,
    9.9177,   -23.358, 3.526,    14.347,  56.015,   -61.078, 13.522,    -39.489,  2.7255,   23.584,  85.089,   -73.739,
    0.35371,  0.688,   3.1021,   -1.214,  1.4145,   -5.1741, 1.1383,    1.9609,   10.516,   -3.5054, 4.9099,   -17.386,
    -0.55742, -9.2266, -31.421,  2.85,    -16.201,  29.855,  -0.018638, 2.7871,   -0.41975, -4.8759, 1.9087,   -5.3465,
    50.302,   2.2937,  -57.752,  -16.433, 47.998,   7.9872,  3.092,     -2.2806,  0.17832,  0.53664, 7.3969,   -3.4084,
    -31.905,  54.291,  -196.55,  -78.709, 45.917,   220.39,  -1.1708,   2.2834,   -17.34,   -5.3529, 11.478,   14.416,
};
constexpr std::array<float, 240> kCtrlPMid{
    -2.8738,  -25.531, 19.435,   38.814,  -17.922,  -14.007, -4.5018,   -25.911,  25.237,   43.021,  -28.637,  -17.155,
    -0.364,   1.6136,  -0.74551, -1.3902, -0.13605, 1.2222,  -1.174,    5.3865,   -2.6429,  -4.4343, -0.75261, 4.3969,
    -8.5608,  -70.706, 14.811,   66.758,  4.6985,   -19.552, -0.67071,  -5.3412,  2.2422,   -2.4914, 2.067,    -2.6791,
    -3.339,   2.4674,  -7.1034,  8.0791,  -10.92,   12.178,  -0.32482,  -1.086,   -0.10843, 4.4641,  -8.7343,  1.8098,
    -19.686,  21.331,  29.743,   13.917,  -31.289,  -27.69,  -2.37,     1.4437,   5.874,    3.0987,  -6.0086,  -5.6544,
    -2.8738,  19.435,  -25.531,  -14.007, -17.922,  38.814,  -4.5018,   25.237,   -25.911,  -17.155, -28.637,  43.021,
    0.364,    0.74551, -1.6136,  -1.2222, 0.13605,  1.3902,  1.174,     2.6429,   -5.3865,  -4.3969, 0.75261,  4.4343,
    -3.339,   -7.1034, 2.4674,   12.178,  -10.92,   8.0791,  -0.32482,  -0.10843, -1.086,   1.8098,  -8.7343,  4.4641,
    -8.5608,  14.811,  -70.706,  -19.552, 4.6985,   66.758,  -0.67071,  2.2422,   -5.3412,  -2.6791, 2.067,    -2.4914,
    -19.686,  29.743,  21.331,   -27.69,  -31.289,  13.917,  -2.37,     5.874,    1.4437,   -5.6544, -6.0086,  3.0987,
    9.9177,   3.526,   -23.358,  -61.078, 56.015,   14.347,  13.522,    2.7255,   -39.489,  -73.739, 85.089,   23.584,
    -0.35371, -3.1021, -0.688,   5.1741,  -1.4145,  1.214,   -1.1383,   -10.516,  -1.9609,  17.386,  -4.9099,  3.5054,
    50.302,   -57.752, 2.2937,   7.9872,  47.998,   -16.433, 3.092,     0.17832,  -2.2806,  -3.4084, 7.3969,   0.53664,
    -0.55742, -31.421, -9.2266,  29.855,  -16.201,  2.85,    -0.018638, -0.41975, 2.7871,   -5.3465, 1.9087,   -4.8759,
    -31.905,  -196.55, 54.291,   220.39,  45.917,   -78.709, -1.1708,   -17.34,   2.2834,   14.416,  11.478,   -5.3529,
    9.9177,   -23.358, 3.526,    14.347,  56.015,   -61.078, 13.522,    -39.489,  2.7255,   23.584,  85.089,   -73.739,
    0.35371,  0.688,   3.1021,   -1.214,  1.4145,   -5.1741, 1.1383,    1.9609,   10.516,   -3.5054, 4.9099,   -17.386,
    -0.55742, -9.2266, -31.421,  2.85,    -16.201,  29.855,  -0.018638, 2.7871,   -0.41975, -4.8759, 1.9087,   -5.3465,
    50.302,   2.2937,  -57.752,  -16.433, 47.998,   7.9872,  3.092,     -2.2806,  0.17832,  0.53664, 7.3969,   -3.4084,
    -31.905,  54.291,  -196.55,  -78.709, 45.917,   220.39,  -1.1708,   2.2834,   -17.34,   -5.3529, 11.478,   14.416,
};
constexpr std::array<float, 240> kCtrlPHigh{
    -2.8738,  -25.531, 19.435,   38.814,  -17.922,  -14.007, -4.5018,   -25.911,  25.237,   43.021,  -28.637,  -17.155,
    -0.364,   1.6136,  -0.74551, -1.3902, -0.13605, 1.2222,  -1.174,    5.3865,   -2.6429,  -4.4343, -0.75261, 4.3969,
    -8.5608,  -70.706, 14.811,   66.758,  4.6985,   -19.552, -0.67071,  -5.3412,  2.2422,   -2.4914, 2.067,    -2.6791,
    -3.339,   2.4674,  -7.1034,  8.0791,  -10.92,   12.178,  -0.32482,  -1.086,   -0.10843, 4.4641,  -8.7343,  1.8098,
    -19.686,  21.331,  29.743,   13.917,  -31.289,  -27.69,  -2.37,     1.4437,   5.874,    3.0987,  -6.0086,  -5.6544,
    -2.8738,  19.435,  -25.531,  -14.007, -17.922,  38.814,  -4.5018,   25.237,   -25.911,  -17.155, -28.637,  43.021,
    0.364,    0.74551, -1.6136,  -1.2222, 0.13605,  1.3902,  1.174,     2.6429,   -5.3865,  -4.3969, 0.75261,  4.4343,
    -3.339,   -7.1034, 2.4674,   12.178,  -10.92,   8.0791,  -0.32482,  -0.10843, -1.086,   1.8098,  -8.7343,  4.4641,
    -8.5608,  14.811,  -70.706,  -19.552, 4.6985,   66.758,  -0.67071,  2.2422,   -5.3412,  -2.6791, 2.067,    -2.4914,
    -19.686,  29.743,  21.331,   -27.69,  -31.289,  13.917,  -2.37,     5.874,    1.4437,   -5.6544, -6.0086,  3.0987,
    9.9177,   3.526,   -23.358,  -61.078, 56.015,   14.347,  13.522,    2.7255,   -39.489,  -73.739, 85.089,   23.584,
    -0.35371, -3.1021, -0.688,   5.1741,  -1.4145,  1.214,   -1.1383,   -10.516,  -1.9609,  17.386,  -4.9099,  3.5054,
    50.302,   -57.752, 2.2937,   7.9872,  47.998,   -16.433, 3.092,     0.17832,  -2.2806,  -3.4084, 7.3969,   0.53664,
    -0.55742, -31.421, -9.2266,  29.855,  -16.201,  2.85,    -0.018638, -0.41975, 2.7871,   -5.3465, 1.9087,   -4.8759,
    -31.905,  -196.55, 54.291,   220.39,  45.917,   -78.709, -1.1708,   -17.34,   2.2834,   14.416,  11.478,   -5.3529,
    9.9177,   -23.358, 3.526,    14.347,  56.015,   -61.078, 13.522,    -39.489,  2.7255,   23.584,  85.089,   -73.739,
    0.35371,  0.688,   3.1021,   -1.214,  1.4145,   -5.1741, 1.1383,    1.9609,   10.516,   -3.5054, 4.9099,   -17.386,
    -0.55742, -9.2266, -31.421,  2.85,    -16.201,  29.855,  -0.018638, 2.7871,   -0.41975, -4.8759, 1.9087,   -5.3465,
    50.302,   2.2937,  -57.752,  -16.433, 47.998,   7.9872,  3.092,     -2.2806,  0.17832,  0.53664, 7.3969,   -3.4084,
    -31.905,  54.291,  -196.55,  -78.709, 45.917,   220.39,  -1.1708,   2.2834,   -17.34,   -5.3529, 11.478,   14.416,
};
constexpr std::array<float, 240> kCtrlPSpin{
    -2.8738,  -25.531, 19.435,   38.814,  -17.922,  -14.007, -4.5018,   -25.911,  25.237,   43.021,  -28.637,  -17.155,
    -0.364,   1.6136,  -0.74551, -1.3902, -0.13605, 1.2222,  -1.174,    5.3865,   -2.6429,  -4.4343, -0.75261, 4.3969,
    -8.5608,  -70.706, 14.811,   66.758,  4.6985,   -19.552, -0.67071,  -5.3412,  2.2422,   -2.4914, 2.067,    -2.6791,
    -3.339,   2.4674,  -7.1034,  8.0791,  -10.92,   12.178,  -0.32482,  -1.086,   -0.10843, 4.4641,  -8.7343,  1.8098,
    -19.686,  21.331,  29.743,   13.917,  -31.289,  -27.69,  -2.37,     1.4437,   5.874,    3.0987,  -6.0086,  -5.6544,
    -2.8738,  19.435,  -25.531,  -14.007, -17.922,  38.814,  -4.5018,   25.237,   -25.911,  -17.155, -28.637,  43.021,
    0.364,    0.74551, -1.6136,  -1.2222, 0.13605,  1.3902,  1.174,     2.6429,   -5.3865,  -4.3969, 0.75261,  4.4343,
    -3.339,   -7.1034, 2.4674,   12.178,  -10.92,   8.0791,  -0.32482,  -0.10843, -1.086,   1.8098,  -8.7343,  4.4641,
    -8.5608,  14.811,  -70.706,  -19.552, 4.6985,   66.758,  -0.67071,  2.2422,   -5.3412,  -2.6791, 2.067,    -2.4914,
    -19.686,  29.743,  21.331,   -27.69,  -31.289,  13.917,  -2.37,     5.874,    1.4437,   -5.6544, -6.0086,  3.0987,
    9.9177,   3.526,   -23.358,  -61.078, 56.015,   14.347,  13.522,    2.7255,   -39.489,  -73.739, 85.089,   23.584,
    -0.35371, -3.1021, -0.688,   5.1741,  -1.4145,  1.214,   -1.1383,   -10.516,  -1.9609,  17.386,  -4.9099,  3.5054,
    50.302,   -57.752, 2.2937,   7.9872,  47.998,   -16.433, 3.092,     0.17832,  -2.2806,  -3.4084, 7.3969,   0.53664,
    -0.55742, -31.421, -9.2266,  29.855,  -16.201,  2.85,    -0.018638, -0.41975, 2.7871,   -5.3465, 1.9087,   -4.8759,
    -31.905,  -196.55, 54.291,   220.39,  45.917,   -78.709, -1.1708,   -17.34,   2.2834,   14.416,  11.478,   -5.3529,
    9.9177,   -23.358, 3.526,    14.347,  56.015,   -61.078, 13.522,    -39.489,  2.7255,   23.584,  85.089,   -73.739,
    0.35371,  0.688,   3.1021,   -1.214,  1.4145,   -5.1741, 1.1383,    1.9609,   10.516,   -3.5054, 4.9099,   -17.386,
    -0.55742, -9.2266, -31.421,  2.85,    -16.201,  29.855,  -0.018638, 2.7871,   -0.41975, -4.8759, 1.9087,   -5.3465,
    50.302,   2.2937,  -57.752,  -16.433, 47.998,   7.9872,  3.092,     -2.2806,  0.17832,  0.53664, 7.3969,   -3.4084,
    -31.905,  54.291,  -196.55,  -78.709, 45.917,   220.39,  -1.1708,   2.2834,   -17.34,   -5.3529, 11.478,   14.416,
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

// ==== 中腿长下压（PID 增益）====
constexpr PidGains kLeftL0PidDip{3000.0f, 0.f, 180.0f, 170.0f, 10.0f};   ///< 左腿下压腿长 PID
constexpr PidGains kRightL0PidDip{3000.0f, 0.f, 180.0f, 170.0f, 10.0f};  ///< 右腿下压腿长 PID

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{20.0f, 0.0f, 10.0f, 25.0f, 0.0f};   ///< 左腿摆角速度 PID
constexpr PidGains kRightLegTurnPid{20.0f, 0.0f, 10.0f, 25.0f, 0.0f};  ///< 右腿摆角速度 PID
constexpr PidGains kLeftLegTurnPidManual{10.0f, 0.0f, 2.0f, 20.0f, 0.0f};  ///< 左腿摆角速度 PID（手动倒地恢复）
constexpr PidGains kRightLegTurnPidManual{10.0f, 0.0f, 2.0f, 20.0f, 0.0f};  ///< 右腿摆角速度 PID（手动倒地恢复）

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
constexpr float kRcYawRateMaxRadS = -5.5f;          ///< RC 摇杆满偏时偏航积分速率 [rad/s]
constexpr float kRcPitchRateMaxRadS = 5.5f;         ///< RC 摇杆满偏时俯仰积分速率 [rad/s]
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
constexpr float kTargetForwardSpeedMaxMps = 2.6f;         ///< 最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.1f;  ///< 高腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxMidLegMps = 1.0f;   ///< G键中腿长模式最大前进速度 [m/s]
constexpr float kTargetSpeedBiasLowLegMps = 0.0f;         ///< 低腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegMps = 0.0f;         ///< C键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegGMps = 0.0f;        ///< G键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasHighLegMps = 0.0f;        ///< 高腿长目标速度偏置 [m/s]
constexpr float kMaxSafeYawRateRadS = 4.5f;               ///< 摩擦圆最大安全偏航速率 [rad/s]
constexpr float kVxInputDeadbandNorm = 0.05f;             ///< 前进输入死区
constexpr float kVyInputDeadbandNorm = 0.05f;             ///< 平移输入死区
constexpr float kYawFollowRampStepRadS = 0.065f;          ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.3f;  ///< 位置锚定冻结速度阈值 [m/s]

// ==== 落地减速（离地→落地时通过腿摆角辅助减速）====
constexpr float kLandingDecelThetaGain = -0.2f;               ///< 落地减速腿摆角增益 [rad/(m/s)]
constexpr float kLandingDecelThetaMaxRad = 0.3f;              ///< 落地减速腿摆角最大偏置 [rad]
constexpr float kLandingDecelThetaRampStepRad = 0.008f;       ///< 落地减速腿摆角每周期斜坡步长 [rad/tick]
constexpr std::uint32_t kLandingDecelOffGroundMinMs = 40U;    ///< 离地最短持续时间（防单帧误判）[ms]
constexpr std::uint32_t kLandingDecelStableDurationMs = 50U;  ///< 落地减速稳定保持时间 [ms]

constexpr float kYawFollowFixedTargetRad = 0.38f;                ///< 偏航跟随固定目标偏置角 [rad]
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;            ///< 偏航跟随侧向目标偏置角 [rad]
constexpr PidGains kYawFollowPid{16.0f, 0.0f, 2.f, 6.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置 ====
constexpr float kExpectedThetaLlBiasRadLowLeg = 0.09f;   ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = 0.09f;   ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLeg = 0.03f;   ///< 中腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLeg = 0.03f;   ///< 中腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadHighLeg = 0.06f;  ///< 高腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadHighLeg = 0.06f;  ///< 高腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = 0.f;            ///< 期望机体俯仰偏置 [rad]

// ==== 速度斜坡参数 ====
constexpr SdotRampParams kSdotRampLowLeg{0.0065f, 0.0065f};  ///< 低腿长速度斜坡
constexpr SdotRampParams kSdotRampMidLeg{0.008f, 0.008f};    ///< 中腿长速度斜坡(C 键触发)
constexpr SdotRampParams kSdotRampMidLegG{0.006f, 0.006f};   ///< 中腿长速度斜坡（G 键触发）
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
constexpr float kJumpThetaLlBiasRad = 0.f;  ///< 跳跃时左腿摆角偏置 [rad]
constexpr float kJumpThetaLrBiasRad = 0.f;  ///< 跳跃时右腿摆角偏置 [rad]
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
constexpr PidGains kFricSpeedPid{20.0f, 1.0f, 0.0f, 16000.0f, 2000.0f};   ///< 摩擦轮速度 PID
constexpr PidGains kDialSpeedPid{10.0f, 0.5f, 0.0f, 16000.0f, 1000.0f};   ///< 拨盘速度 PID
constexpr PidGains kDialPositionPid{0.5f, 0.f, 0.01f, 30000.0f, 500.0f};  ///< 拨盘位置 PID
constexpr int16_t kDialFireThreshold = -600;                              ///< 发射触发拨轮阈值
constexpr float kShootFrequencyHz = 7.0f;                                 ///< 发射频率 [Hz]
constexpr float kFricSpeedStepRpm = 20.0f;  ///< Z/X 键每次调整摩擦轮转速步长 [rpm]

// ── 本地热量闭环 ──
constexpr float kHeatPerShot = 10.0f;       ///< 每发子弹热量增量 [热量单位]
constexpr float kHeatSafetyMargin = 20.0f;  ///< 停火余量：heat + kHeatPerShot > limit - margin 时抑制发射
constexpr float kHeatResumeMargin = 20.0f;  ///< 恢复余量：heat < limit - margin 时恢复，与停火线构成迟滞
constexpr uint16_t kDefaultHeatLimit = 240;   ///< 裁判系统离线时默认热量上限
constexpr uint16_t kDefaultCoolingRate = 40;  ///< 裁判系统离线时默认冷却速率 [热量单位/秒]
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
constexpr float kManualRecoveryLegSpeedRadS = 0.5f;  ///< 手动倒地恢复腿摆角速度 [rad/s]
constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;  ///< 倒地恢复零力矩区间下限 [rad]
constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;  ///< 倒地恢复零力矩区间上限 [rad]

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
    -5.1003,  -30.687, 26.728,   49.712,  -25.513,  -19.271, -9.0793,  -36.423,  43.106,   68.094,   -52.854, -28.257,
    -0.65508, 3.2742,  -0.72844, -3.9295, 0.48505,  1.2416,  -2.2904,  11.838,   -2.9905,  -13.797,  1.2249,  5.194,
    -23.818,  -85.341, 13.315,   92.289,  -3.3689,  -16.196, -1.3317,  -7.1848,  3.1738,   -4.2585,  2.1099,  -3.4166,
    -4.9182,  12.489,  -4.8435,  -11.086, -4.6259,  10.888,  -0.45791, -1.193,   -0.75242, 5.9989,   -12.936, 3.9414,
    -29.23,   75.32,   24.233,   -62.611, -31.24,   -24.41,  -3.2844,  4.7608,   6.9714,   -0.20591, -8.2421, -6.782,
    -5.1003,  26.728,  -30.687,  -19.271, -25.513,  49.712,  -9.0793,  43.106,   -36.423,  -28.257,  -52.854, 68.094,
    0.65508,  0.72844, -3.2742,  -1.2416, -0.48505, 3.9295,  2.2904,   2.9905,   -11.838,  -5.194,   -1.2249, 13.797,
    -4.9182,  -4.8435, 12.489,   10.888,  -4.6259,  -11.086, -0.45791, -0.75242, -1.193,   3.9414,   -12.936, 5.9989,
    -23.818,  13.315,  -85.341,  -16.196, -3.3689,  92.289,  -1.3317,  3.1738,   -7.1848,  -3.4166,  2.1099,  -4.2585,
    -29.23,   24.233,  75.32,    -24.41,  -31.24,   -62.611, -3.2844,  6.9714,   4.7608,   -6.782,   -8.2421, -0.20591,
    4.3189,   -5.9518, -5.9048,  -15.377, 24.788,   1.5585,  6.8597,   -7.3116,  -15.689,  -24.314,  44.456,  6.892,
    -0.666,   -1.793,  -1.0297,  3.2082,  -0.27804, 1.5192,  -2.326,   -6.6456,  -3.5559,  11.771,   -1.3076, 5.2372,
    40.955,   -98.328, 7.2102,   121.9,   16.566,   -15.919, 2.079,    -3.5227,  -0.53068, 4.6633,   4.5528,  -0.74424,
    -4.5134,  -14.589, -4.7856,  20.62,   -3.9411,  -13.562, -0.27174, -0.90946, 2.0589,   -1.0619,  0.34223, -4.632,
    -48.549,  -131.85, 56.049,   181.21,  7.4325,   -77.041, -2.8599,  -12.731,  4.8409,   14.089,   4.7622,  -7.3524,
    4.3189,   -5.9048, -5.9518,  1.5585,  24.788,   -15.377, 6.8597,   -15.689,  -7.3116,  6.892,    44.456,  -24.314,
    0.666,    1.0297,  1.793,    -1.5192, 0.27804,  -3.2082, 2.326,    3.5559,   6.6456,   -5.2372,  1.3076,  -11.771,
    -4.5134,  -4.7856, -14.589,  -13.562, -3.9411,  20.62,   -0.27174, 2.0589,   -0.90946, -4.632,   0.34223, -1.0619,
    40.955,   7.2102,  -98.328,  -15.919, 16.566,   121.9,   2.079,    -0.53068, -3.5227,  -0.74424, 4.5528,  4.6633,
    -48.549,  56.049,  -131.85,  -77.041, 7.4325,   181.21,  -2.8599,  4.8409,   -12.731,  -7.3524,  4.7622,  14.089,
};
constexpr std::array<float, 240> kCtrlPMid{
  -4.0535,  -22.779,  20.378,  36.761,  -21.041,  -12.926,
     -8.8721,  -31.794,  40.576,  62.169,  -54.539,  -22.794,
     -0.5744,  2.8791,  -0.60755,  -3.385,  0.28828,  1.0784,
     -2.0085,  10.549,  -2.6745,  -11.88,  0.27334,  4.8586,
     -30.888,  -91.063,  9.7333,  110.42,  -2.8147,  -11.403,
     -1.4028,  -6.4431,  2.8347,  -4.1747,  1.7979,  -2.657,
     -4.832,  16.739,  -5.3818,  -24.228,  -1.0044,  14.838,
     -0.45911,  -0.70265,  -0.80753,  4.9507,  -12.053,  4.5849,
     -30.31,  84.096,  20.849,  -75.873,  -28.749,  -21.068,
     -3.2615,  5.3572,  6.5227,  -1.2699,  -8.154,  -6.1068,
     -4.0535,  20.378,  -22.779,  -12.926,  -21.041,  36.761,
     -8.8721,  40.576,  -31.794,  -22.794,  -54.539,  62.169,
     0.5744,  0.60755,  -2.8791,  -1.0784,  -0.28828,  3.385,
     2.0085,  2.6745,  -10.549,  -4.8586,  -0.27334,  11.88,
     -4.832,  -5.3818,  16.739,  14.838,  -1.0044,  -24.228,
     -0.45911,  -0.80753,  -0.70265,  4.5849,  -12.053,  4.9507,
     -30.888,  9.7333,  -91.063,  -11.403,  -2.8147,  110.42,
     -1.4028,  2.8347,  -6.4431,  -2.657,  1.7979,  -4.1747,
     -30.31,  20.849,  84.096,  -21.068,  -28.749,  -75.873,
     -3.2615,  6.5227,  5.3572,  -6.1068,  -8.154,  -1.2699,
     2.8699,  -4.0193,  -3.462,  -11.684,  18.047,  -0.058069,
     5.6345,  -5.4228,  -13.308,  -22.977,  39.856,  4.6216,
     -0.63526,  -1.4421,  -0.84977,  2.5483,  -0.18236,  1.2662,
     -2.221,  -5.4287,  -2.8992,  9.4787,  -1.0092,  4.3227,
     50.352,  -138.04,  6.2802,  173.62,  8.9799,  -13.186,
     2.0491,  -4.1376,  -0.41689,  5.6282,  3.9642,  -0.7866,
     -5.3724,  -10.28,  -2.6966,  16.049,  3.9189,  -20.893,
     -0.30269,  -0.79082,  1.7936,  -0.96958,  0.7387,  -4.6465,
     -50.599,  -131.43,  55.051,  184.8,  5.3091,  -76.977,
     -3.0364,  -12.155,  4.9235,  13.842,  4.2229,  -7.5171,
     2.8699,  -3.462,  -4.0193,  -0.058069,  18.047,  -11.684,
     5.6345,  -13.308,  -5.4228,  4.6216,  39.856,  -22.977,
     0.63526,  0.84977,  1.4421,  -1.2662,  0.18236,  -2.5483,
     2.221,  2.8992,  5.4287,  -4.3227,  1.0092,  -9.4787,
     -5.3724,  -2.6966,  -10.28,  -20.893,  3.9189,  16.049,
     -0.30269,  1.7936,  -0.79082,  -4.6465,  0.7387,  -0.96958,
     50.352,  6.2802,  -138.04,  -13.186,  8.9799,  173.62,
     2.0491,  -0.41689,  -4.1376,  -0.7866,  3.9642,  5.6282,
     -50.599,  55.051,  -131.43,  -76.977,  5.3091,  184.8,
     -3.0364,  4.9235,  -12.155,  -7.5171,  4.2229,  13.842,
};
constexpr std::array<float, 240> kCtrlPHigh{
    -5.1003,  -30.687, 26.728,   49.712,  -25.513,  -19.271, -9.0793,  -36.423,  43.106,   68.094,   -52.854, -28.257,
    -0.65508, 3.2742,  -0.72844, -3.9295, 0.48505,  1.2416,  -2.2904,  11.838,   -2.9905,  -13.797,  1.2249,  5.194,
    -23.818,  -85.341, 13.315,   92.289,  -3.3689,  -16.196, -1.3317,  -7.1848,  3.1738,   -4.2585,  2.1099,  -3.4166,
    -4.9182,  12.489,  -4.8435,  -11.086, -4.6259,  10.888,  -0.45791, -1.193,   -0.75242, 5.9989,   -12.936, 3.9414,
    -29.23,   75.32,   24.233,   -62.611, -31.24,   -24.41,  -3.2844,  4.7608,   6.9714,   -0.20591, -8.2421, -6.782,
    -5.1003,  26.728,  -30.687,  -19.271, -25.513,  49.712,  -9.0793,  43.106,   -36.423,  -28.257,  -52.854, 68.094,
    0.65508,  0.72844, -3.2742,  -1.2416, -0.48505, 3.9295,  2.2904,   2.9905,   -11.838,  -5.194,   -1.2249, 13.797,
    -4.9182,  -4.8435, 12.489,   10.888,  -4.6259,  -11.086, -0.45791, -0.75242, -1.193,   3.9414,   -12.936, 5.9989,
    -23.818,  13.315,  -85.341,  -16.196, -3.3689,  92.289,  -1.3317,  3.1738,   -7.1848,  -3.4166,  2.1099,  -4.2585,
    -29.23,   24.233,  75.32,    -24.41,  -31.24,   -62.611, -3.2844,  6.9714,   4.7608,   -6.782,   -8.2421, -0.20591,
    4.3189,   -5.9518, -5.9048,  -15.377, 24.788,   1.5585,  6.8597,   -7.3116,  -15.689,  -24.314,  44.456,  6.892,
    -0.666,   -1.793,  -1.0297,  3.2082,  -0.27804, 1.5192,  -2.326,   -6.6456,  -3.5559,  11.771,   -1.3076, 5.2372,
    40.955,   -98.328, 7.2102,   121.9,   16.566,   -15.919, 2.079,    -3.5227,  -0.53068, 4.6633,   4.5528,  -0.74424,
    -4.5134,  -14.589, -4.7856,  20.62,   -3.9411,  -13.562, -0.27174, -0.90946, 2.0589,   -1.0619,  0.34223, -4.632,
    -48.549,  -131.85, 56.049,   181.21,  7.4325,   -77.041, -2.8599,  -12.731,  4.8409,   14.089,   4.7622,  -7.3524,
    4.3189,   -5.9048, -5.9518,  1.5585,  24.788,   -15.377, 6.8597,   -15.689,  -7.3116,  6.892,    44.456,  -24.314,
    0.666,    1.0297,  1.793,    -1.5192, 0.27804,  -3.2082, 2.326,    3.5559,   6.6456,   -5.2372,  1.3076,  -11.771,
    -4.5134,  -4.7856, -14.589,  -13.562, -3.9411,  20.62,   -0.27174, 2.0589,   -0.90946, -4.632,   0.34223, -1.0619,
    40.955,   7.2102,  -98.328,  -15.919, 16.566,   121.9,   2.079,    -0.53068, -3.5227,  -0.74424, 4.5528,  4.6633,
    -48.549,  56.049,  -131.85,  -77.041, 7.4325,   181.21,  -2.8599,  4.8409,   -12.731,  -7.3524,  4.7622,  14.089,
};
constexpr std::array<float, 240> kCtrlPSpin{
  -3.3453,  -22.321,  18.627,  34.893,  -17.917,  -12.688,
     -6.5586,  -28.483,  32.146,  51.506,  -39.891,  -19.805,
     -0.49348,  2.2785,  -0.73626,  -2.4326,  0.11325,  1.2067,
     -1.7291,  8.2938,  -2.9549,  -8.5059,  -0.11629,  4.9575,
     -17.292,  -86.563,  12.044,  97.171,  0.41234,  -15.272,
     -0.94715,  -6.0057,  2.5164,  -3.2571,  1.9463,  -2.6836,
     -4.3058,  9.8049,  -5.7638,  -7.2144,  -3.6879,  9.3255,
     -0.40242,  -0.87677,  -0.45191,  4.6214,  -10.147,  2.875,
     -24.632,  47.754,  27.257,  -22.475,  -31.141,  -26.74,
     -2.8143,  3.2317,  6.2842,  1.1615,  -7.0477,  -5.9999,
     -3.3453,  18.627,  -22.321,  -12.688,  -17.917,  34.893,
     -6.5586,  32.146,  -28.483,  -19.805,  -39.891,  51.506,
     0.49348,  0.73626,  -2.2785,  -1.2067,  -0.11325,  2.4326,
     1.7291,  2.9549,  -8.2938,  -4.9575,  0.11629,  8.5059,
     -4.3058,  -5.7638,  9.8049,  9.3255,  -3.6879,  -7.2144,
     -0.40242,  -0.45191,  -0.87677,  2.875,  -10.147,  4.6214,
     -17.292,  12.044,  -86.563,  -15.272,  0.41234,  97.171,
     -0.94715,  2.5164,  -6.0057,  -2.6836,  1.9463,  -3.2571,
     -24.632,  27.257,  47.754,  -26.74,  -31.141,  -22.475,
     -2.8143,  6.2842,  3.2317,  -5.9999,  -7.0477,  1.1615,
     5.2906,  -1.5358,  -10.634,  -27.098,  30.954,  4.4943,
     9.2627,  -2.4783,  -25.453,  -43.556,  61.184,  11.939,
     -0.52151,  -2.3321,  -0.96,  4.0104,  -0.75339,  1.4808,
     -1.8271,  -8.6291,  -3.1764,  14.674,  -2.9648,  4.882,
     56.241,  -121,  7.7858,  123.38,  23.675,  -18.831,
     2.4938,  -2.3795,  -1.0781,  2.591,  5.2883,  -0.53011,
     -3.6752,  -22.887,  -5.6764,  29.882,  -5.1994,  -12.95,
     -0.18796,  -0.86191,  2.3277,  -2.4464,  1.6378,  -5.5457,
     -40.801,  -169.16,  59.339,  217.27,  19.858,  -82.264,
     -2.1908,  -14.989,  4.1106,  15.165,  7.2074,  -6.9259,
     5.2906,  -10.634,  -1.5358,  4.4943,  30.954,  -27.098,
     9.2627,  -25.453,  -2.4783,  11.939,  61.184,  -43.556,
     0.52151,  0.96,  2.3321,  -1.4808,  0.75339,  -4.0104,
     1.8271,  3.1764,  8.6291,  -4.882,  2.9648,  -14.674,
     -3.6752,  -5.6764,  -22.887,  -12.95,  -5.1994,  29.882,
     -0.18796,  2.3277,  -0.86191,  -5.5457,  1.6378,  -2.4464,
     56.241,  7.7858,  -121,  -18.831,  23.675,  123.38,
     2.4938,  -1.0781,  -2.3795,  -0.53011,  5.2883,  2.591,
     -40.801,  59.339,  -169.16,  -82.264,  19.858,  217.27,
     -2.1908,  4.1106,  -14.989,  -6.9259,  7.2074,  15.165,
};

// ==== 基本运动（PID 增益）====
constexpr PidGains kLeftL0Pid{2500.0f, 0.f, 200.0f, 170.0f, 30.0f};   ///< 左腿腿长 PID（常规）
constexpr PidGains kRightL0Pid{2500.0f, 0.f, 200.0f, 170.0f, 30.0f};  ///< 右腿腿长 PID（常规）
constexpr PidGains kRollPid{800.0f, 0.0f, 200.0f, 80.0f, 0.0f};       ///< 横滚平衡 PID

// ==== 跳跃（PID 增益）====
constexpr PidGains kLeftL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 180.0f, 0.0f};       ///< 左腿蹬伸 PID
constexpr PidGains kRightL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 180.0f, 0.0f};      ///< 右腿蹬伸 PID
constexpr PidGains kLeftL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};   ///< 左腿回收 PID
constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};  ///< 右腿回收 PID

// ==== 中腿长下压（PID 增益）====
constexpr PidGains kLeftL0PidDip{2500.0f, 0.f, 200.0f, 170.0f, 30.0f};   ///< 左腿下压腿长 PID
constexpr PidGains kRightL0PidDip{2500.0f, 0.f, 200.0f, 170.0f, 30.0f};  ///< 右腿下压腿长 PID

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{20.0f, 0.0f, 10.0f, 25.0f, 0.0f};   ///< 左腿摆角速度 PID
constexpr PidGains kRightLegTurnPid{20.0f, 0.0f, 10.0f, 25.0f, 0.0f};  ///< 右腿摆角速度 PID
constexpr PidGains kLeftLegTurnPidManual{10.0f, 0.0f, 2.0f, 20.0f, 0.0f};  ///< 左腿摆角速度 PID（手动倒地恢复）
constexpr PidGains kRightLegTurnPidManual{10.0f, 0.0f, 2.0f, 20.0f, 0.0f};  ///< 右腿摆角速度 PID（手动倒地恢复）

// ==== 上台阶（腿摆角 PID）====
constexpr PidGains kStairClimbThetaPid{6.0f, 0.0f, 1.5f, 15.0f,
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
constexpr float kRcYawRateMaxRadS = -4.f;           ///< RC 摇杆满偏时偏航积分速率 [rad/s]
constexpr float kRcPitchRateMaxRadS = 2.5f;         ///< RC 摇杆满偏时俯仰积分速率 [rad/s]
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
constexpr float kTargetForwardSpeedMaxMps = 2.1f;         ///< 最大前进速度 [m/s+]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.0f;  ///< 高腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxMidLegMps = 1.0f;   ///< G键中腿长模式最大前进速度 [m/s]
constexpr float kTargetSpeedBiasLowLegMps = 0.0f;         ///< 低腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegMps = -0.15f;         ///< C键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegGMps = -0.4f;       ///< G键中腿长目标速度偏置 [m/s]
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
constexpr float kExpectedThetaLlBiasRadLowLeg = 0.02f;   ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = 0.02f;   ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLeg = 0.05f;   ///< 中腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLeg = 0.05f;   ///< 中腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadHighLeg = 0.02f;  ///< 高腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadHighLeg = 0.02f;  ///< 高腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = 0.018f;         ///< 期望机体俯仰偏置 [rad]

// ==== 速度斜坡参数 ====
constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.008f};    ///< 低腿长速度斜坡
constexpr SdotRampParams kSdotRampMidLeg{0.007f, 0.007f};   ///< 中腿长速度斜坡(C 键触发)
constexpr SdotRampParams kSdotRampMidLegG{0.01f, 0.008f};   ///< 中腿长速度斜坡（G 键触发）
constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};  ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.05f;           ///< 小陀螺进入偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinExitYawRampStepRadS = 0.15f;       ///< 小陀螺退出偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS = 8.f;           ///< 小陀螺目标自旋角速度 [rad/s]
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

constexpr PidGains kYawPositionPid{42.0f, 0.f, 1.5f, 10.0f, 1.5f};   ///< 自瞄偏航位置 PID
constexpr PidGains kYawSpeedPid{0.65f, 0.0f, 0.0f, 8.0f, 0.4f};      ///< 自瞄偏航速度 PID
constexpr PidGains kPitchPositionPid{30.0f, 0.f, 0.7f, 10.0f, 2.f};  ///< 自瞄俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 8.0f, 0.0f};    ///< 自瞄俯仰速度 PID
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
