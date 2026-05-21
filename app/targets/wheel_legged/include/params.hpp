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
constexpr float kIdentTheta[9] = {0.f,        0.f,        0.03675504, -0.26935183 * 1.9, 0.26007541 * 1.9,
                                  0.69615303, 0.51871814, 0.69915412, 0.0656076};
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
inline constexpr float kBoosterZeroPointRad = 0.345f;    ///< 拨盘零位角度 [rad]
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
constexpr float kStairClimbThetaThresholdRad = 0.5f;              ///< 双腿摆角均超过此值触发上台阶 [rad]
constexpr float kStairClimbLegLengthM = 0.16f;                    ///< 上台阶收腿目标腿长 [m]
constexpr float kStairClimbPhase0LegLengthM = 0.16f;              ///< 上台阶 Phase 0 转腿目标腿长 [m]
constexpr float kStairClimbThetaTargetRad = 1.5f;                 ///< 上台阶目标腿摆角（统一偏置） [rad]
constexpr std::uint32_t kStairClimbDurationMs = 450U;             ///< 上台阶最长持续时间 [ms]
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
constexpr std::uint32_t kJumpMidPrepMs = 100U;     ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpMidPushMaxMs = 500U;  ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpMidRecoverMs = 250U;  ///< 跳跃回收阶段持续时间（保底超时）[ms]
constexpr std::uint32_t kJumpMidRecoverMinMs = 100U;  ///< 跳跃回收阶段最低维持时间（此后开始判断离地）[ms]
constexpr float kJumpMidPrepLegLengthM = 0.18f;         ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpMidPushLegLengthM = 0.28f;         ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpMidRecoverLegLengthM = 0.18f;      ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpMidPushReachedLegLengthM = 0.26f;  ///< 蹬伸到位判定腿长 [m]

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
constexpr float kOffGroundSupportForceClampN = 50.0f;      ///< 离地时支持力限幅值 [N]

// -- 中腿长下压 --
constexpr float kMidLegDipTriggerLengthM = 0.26f;  ///< 中腿长模式下触发下压的腿长阈值 [m]
constexpr float kMidLegDipTargetLengthM = 0.17f;   ///< 下压目标腿长 [m]
constexpr uint16_t kMidLegDipHoldTicks = 500;      ///< 下压维持时间 [ticks @ 500Hz = 1s]

// ==== 物理参数（变体专属）====
constexpr float kBodyMassKg = 30.0f;  ///< 机体质量 [kg]

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.030f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵 — 40 组多项式系数）====
/// 由 MATLAB 离线拟合得到，p(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l² + p11*l_l*l_r + p02*l_r²
/// 共 40 行，对应 4×10 增益矩阵 K 的 40 个元素（按行主序展平）
/// 每行 6 个系数：[p00, p10, p01, p20, p11, p02]
constexpr std::array<float, 240> kCtrlP{
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
constexpr PidGains kRollPid{800.0f, 0.f, 20.0f, 100.0f, 0.0f};       ///< 横滚平衡 PID

// ==== 跳跃（PID 增益）====
constexpr PidGains kLeftL0PidJumpTwo{4000.0f, 0.0f, 60.0f, 120.0f, 0.0f};    ///< 左腿蹬伸 PID（JumpPush）
constexpr PidGains kRightL0PidJumpTwo{4000.0f, 0.0f, 60.0f, 120.0f, 0.0f};   ///< 右腿蹬伸 PID（JumpPush）
constexpr PidGains kLeftL0PidJumpThree{4000.0f, 0.f, 60.0f, 120.0f, 0.0f};   ///< 左腿回收 PID（JumpRecover）
constexpr PidGains kRightL0PidJumpThree{4000.0f, 0.f, 60.0f, 120.0f, 0.0f};  ///< 右腿回收 PID（JumpRecover）

// ==== 中腿长下压（PID 增益）====
constexpr PidGains kLeftL0PidDip{2500.0f, 0.f, 200.0f, 130.0f, 0.0f};   ///< 左腿下压腿长 PID
constexpr PidGains kRightL0PidDip{2500.0f, 0.f, 200.0f, 130.0f, 0.0f};  ///< 右腿下压腿长 PID

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{30.0f, 0.0f, 30.0f, 12.0f, 0.0f};   ///< 左腿摆角速度 PID（倒地恢复用）
constexpr PidGains kRightLegTurnPid{30.0f, 0.0f, 30.0f, 12.0f, 0.0f};  ///< 右腿摆角速度 PID（倒地恢复用）
constexpr PidGains kLeftLegTurnPidManual{24.0f, 0.0f, 2.0f, 20.0f, 0.0f};  ///< 左腿摆角速度 PID（手动倒地恢复）
constexpr PidGains kRightLegTurnPidManual{24.0f, 0.0f, 2.0f, 20.0f, 0.0f};  ///< 右腿摆角速度 PID（手动倒地恢复）

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
constexpr float kTargetForwardSpeedMaxMps = 2.15f;        ///< 最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.3f;  ///< 高腿长模式最大前进速度 [m/s]
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
constexpr PidGains kYawFollowPid{8.0f, 0.0f, 2.f, 4.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置（腿摆角/机体俯仰）====
constexpr float kExpectedThetaLlBiasRadLowLeg = -0.05f;   ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = -0.05f;   ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLeg = -0.03f;   ///< 中腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLeg = -0.03f;   ///< 中腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadHighLeg = -0.01f;  ///< 高腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadHighLeg = -0.01f;  ///< 高腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = -0.087f;         ///< 期望机体俯仰偏置 [rad]

// ==== 速度斜坡参数（按腿长档位分级）====
constexpr SdotRampParams kSdotRampLowLeg{0.008f, 0.008f};   ///< 低腿长速度斜坡（加速/制动步长）
constexpr SdotRampParams kSdotRampMidLeg{0.008f, 0.008f};   ///< 中腿长速度斜坡(C 键触发)
constexpr SdotRampParams kSdotRampMidLegG{0.006f, 0.006f};  ///< 中腿长速度斜坡（G 键触发）
constexpr SdotRampParams kSdotRampHighLeg{0.008f, 0.005f};  ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.1f;            ///< 小陀螺进入偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinExitYawRampStepRadS = 0.15f;       ///< 小陀螺退出偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS = 7.5f;           ///< 小陀螺目标自旋角速度 [rad/s]
constexpr float kSpinExitYawAlignThresholdRad = 0.15f;  ///< 小陀螺预测退出：yaw 对齐阈值 [rad]
constexpr float kSpinTranslationGain = 1.0f;  ///< 小陀螺平移增益（将云台系前进指令投影到车体系的比例）
constexpr float kSpinThetaLlBiasRad = 0.0f;    ///< 小陀螺时左腿摆角偏置 [rad]
constexpr float kSpinThetaLrBiasRad = -0.06f;  ///< 小陀螺时右腿摆角偏置 [rad]
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
constexpr uint8_t kRobotId = 1U;                                          ///< 机器人 ID（裁判系统回退值）
constexpr float kBulletSpeedMps = 11.8f;                                  ///< 弹速 [m/s]（裁判系统回退值）
constexpr PidGains kYawPositionPid{900.0f, 0.2, 3.f, 1000.0f, 4.0f};      ///< 自瞄偏航位置 PID
constexpr PidGains kYawSpeedPid{0.7f, 0.0f, 0.0f, 10.0f, 0.3f};           ///< 自瞄偏航速度 PID
constexpr PidGains kPitchPositionPid{90.0f, 0.05f, 3.0f, 1000.0f, 1.5f};  ///< 自瞄俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.7f, 0.02f, 0.0f, 10.0f, 0.4f};        ///< 自瞄俯仰速度 PID
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
constexpr float kHeatResumeMargin = 80.0f;  ///< 恢复余量：heat < limit - margin 时恢复，与停火线构成迟滞
constexpr uint16_t kDefaultHeatLimit = 240;   ///< 裁判系统离线时默认热量上限
constexpr uint16_t kDefaultCoolingRate = 40;  ///< 裁判系统离线时默认冷却速率 [热量单位/秒]
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
constexpr float kMidLegLengthM = 0.215f;             ///< 中腿长档位目标腿长 [m]
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
constexpr float kStandupThetaThresholdRad = 1.15f;  ///< 起立完成判定：双腿摆角绝对值低于此值后允许轮端输出 [rad]
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
constexpr float kMidLegDipTriggerLengthM = 0.26f;  ///< 中腿长模式下触发下压的腿长阈值 [m]
constexpr float kMidLegDipTargetLengthM = 0.17f;   ///< 下压目标腿长 [m]
constexpr uint16_t kMidLegDipHoldTicks = 500;      ///< 下压维持时间 [ticks @ 500Hz = 1s]

// ==== 物理参数（变体专属）====

constexpr float kBodyMassKg = 22.0f;  ///< 机体质量 [kg]

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.0126f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵）====
constexpr std::array<float, 240> kCtrlP{
    -2.3463,  -15.614,   12.159,   22.696,    -8.3925, -10.288,   -5.6897, -28.722,  27.328,   45.597,    -25.528,
    -21.532,  -0.50829,  1.9091,   -0.79848,  -2.1832, 0.17895,   1.1184,  -3.2255,  12.272,   -5.2543,   -13.841,
    0.89833,  7.4352,    -12.906,  -85.001,   16.431,  82.219,    5.3401,  -22.014,  -0.77297, -6.4289,   2.5898,
    -4.285,   2.7951,    -3.4258,  -4.7007,   6.2018,  -10.057,   4.579,   -0.42209, 0.86909,  -0.38927,  -1.2647,
    -0.69357, 4.8922,    -9.2024,  0.75259,   -25.632, 35.37,     35.264,  1.1083,   -40.676,  -29.896,   -2.8345,
    2.2728,   6.6283,    2.4626,   -6.8811,   -6.2451, -2.3463,   12.159,  -15.614,  -10.288,  -8.3925,   22.696,
    -5.6897,  27.328,    -28.722,  -21.532,   -25.528, 45.597,    0.50829, 0.79848,  -1.9091,  -1.1184,   -0.17895,
    2.1832,   3.2255,    5.2543,   -12.272,   -7.4352, -0.89833,  13.841,  -4.7007,  -10.057,  6.2018,    0.86909,
    -0.42209, 4.579,     -0.38927, -0.69357,  -1.2647, 0.75259,   -9.2024, 4.8922,   -12.906,  16.431,    -85.001,
    -22.014,  5.3401,    82.219,   -0.77297,  2.5898,  -6.4289,   -3.4258, 2.7951,   -4.285,   -25.632,   35.264,
    35.37,    -29.896,   -40.676,  1.1083,    -2.8345, 6.6283,    2.2728,  -6.2451,  -6.8811,  2.4626,    5.2875,
    5.7807,   -16.984,   -31.131,  27.811,    10.078,  11.82,     9.8107,  -40.54,   -63.72,   68.94,     23.023,
    -0.43436, -2.6095,   -1.1236,  4.6588,    -1.4267, 1.6494,    -2.7602, -16.846,  -6.9685,  29.861,    -9.147,
    10.146,   57.841,    -88.263,  10.496,    81.111,  54.726,    -26.814, 2.7714,   0.32177,  -1.6102,   3.2784,
    4.4453,   -0.052521, -3.4099,  -31.86,    -10.772, 40.074,    -41.531, -7.6645,  -0.26086, -0.086809, 3.3042,
    -4.8065,  2.6436,    -9.3608,  -42.94,    -206.59, 68.627,    237.43,  31.85,    -82.288,  -1.8853,   -16.275,
    3.011,    13.659,    9.2054,   -4.7325,   5.2875,  -16.984,   5.7807,  10.078,   27.811,   -31.131,   11.82,
    -40.54,   9.8107,    23.023,   68.94,     -63.72,  0.43436,   1.1236,  2.6095,   -1.6494,  1.4267,    -4.6588,
    2.7602,   6.9685,    16.846,   -10.146,   9.147,   -29.861,   -3.4099, -10.772,  -31.86,   -7.6645,   -41.531,
    40.074,   -0.26086,  3.3042,   -0.086809, -9.3608, 2.6436,    -4.8065, 57.841,   10.496,   -88.263,   -26.814,
    54.726,   81.111,    2.7714,   -1.6102,   0.32177, -0.052521, 4.4453,  3.2784,   -42.94,   68.627,    -206.59,
    -82.288,  31.85,     237.43,   -1.8853,   3.011,   -16.275,   -4.7325, 9.2054,   13.659,
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
constexpr float kTargetForwardSpeedMaxMps = 2.25f;        ///< 最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.1f;  ///< 高腿长 3模式最大前进速度 [m/s]
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
constexpr PidGains kYawFollowPid{15.0f, 0.0f, 2.f, 6.0f, 0.0f};  ///< 偏航跟随 PID

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
constexpr uint8_t kRobotId = 3U;                                      ///< 机器人 ID
constexpr float kBulletSpeedMps = 23.0f;                              ///< 弹速 [m/s]
constexpr PidGains kYawPositionPid{35.0f, 0.5f, 0.05f, 10.0f, 2.2f};  ///< 自瞄偏航位置 PID
constexpr PidGains kYawSpeedPid{0.6f, 0.0f, 0.001f, 10.0f, 0.f};      ///< 自瞄偏航速度 PID
constexpr PidGains kPitchPositionPid{35.0f, 1.f, 0.6f, 10.0f, 2.f};   ///< 自瞄俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.45f, 0.0f, 0.001f, 10.0f, 0.f};   ///< 自瞄俯仰速度 PID
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
constexpr float kFricSpeedTargetRpm = 6700.0f;                            ///< 摩擦轮目标转速 [rpm]
constexpr PidGains kFricSpeedPid{20.0f, 1.0f, 0.0f, 16000.0f, 2000.0f};   ///< 摩擦轮速度 PID
constexpr PidGains kDialSpeedPid{10.0f, 0.5f, 0.0f, 16000.0f, 1000.0f};   ///< 拨盘速度 PID
constexpr PidGains kDialPositionPid{0.5f, 0.f, 0.01f, 30000.0f, 500.0f};  ///< 拨盘位置 PID
constexpr int16_t kDialFireThreshold = -600;                              ///< 发射触发拨轮阈值
constexpr float kShootFrequencyHz = 14.0f;                                ///< 发射频率 [Hz]
constexpr float kFricSpeedStepRpm = 20.0f;  ///< Z/X 键每次调整摩擦轮转速步长 [rpm]

// ── 本地热量闭环 ──
constexpr float kHeatPerShot = 10.0f;       ///< 每发子弹热量增量 [热量单位]
constexpr float kHeatSafetyMargin = 20.0f;  ///< 停火余量：heat + kHeatPerShot > limit - margin 时抑制发射
constexpr float kHeatResumeMargin = 80.0f;  ///< 恢复余量：heat < limit - margin 时恢复，与停火线构成迟滞
constexpr uint16_t kDefaultHeatLimit = 240;   ///< 裁判系统离线时默认热量上限
constexpr uint16_t kDefaultCoolingRate = 40;  ///< 裁判系统离线时默认冷却速率 [热量单位/秒]
}  // namespace shoot

// ── 底盘状态机 ──
namespace chassis_fsm {

// ==== 上台阶 ====
constexpr float kStairClimbThetaThresholdRad = 0.5f;               ///< 双腿摆角均超过此值触发上台阶 [rad]
constexpr float kStairClimbLegLengthM = 0.15f;                     ///< 上台阶收腿目标腿长 [m]
constexpr float kStairClimbPhase0LegLengthM = 0.15f;               ///< 上台阶 Phase 0 转腿目标腿长 [m]
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
constexpr uint16_t kMidLegDipHoldTicks = 1000;      ///< 下压维持时间 [ticks @ 500Hz = 1s]

// ==== 物理参数（变体专属）====
constexpr float kBodyMassKg = 22.0f;  ///< 机体质量 [kg]

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.002f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵）====
constexpr std::array<float, 240> kCtrlP{
-1.7984,  -16.114,  12.247,  24.544,  -10.303,  -9.908,
     -3.8127,  -25.533,  22.852,  40.954,  -22.737,  -17.816,
     -0.36621,  1.5816,  -0.70489,  -1.4123,  -0.028304,  1.1039,
     -1.2899,  5.7247,  -2.6872,  -4.9215,  -0.35335,  4.2687,
     -7.9577,  -68.204,  13.85,  60.467,  5.8125,  -19.073,
     -0.57148,  -5.0653,  2.1825,  -3.4601,  2.2047,  -2.9004,
     -3.1117,  0.34834,  -5.7153,  13.643,  -10.188,  5.0147,
     -0.28832,  -1.3472,  -0.0073236,  4.8053,  -8.3824,  0.84382,
     -18.989,  18.407,  29.308,  15.929,  -30.294,  -27.067,
     -2.3055,  1.1979,  5.6793,  3.2292,  -5.508,  -5.6026,
     -1.7984,  12.247,  -16.114,  -9.908,  -10.303,  24.544,
     -3.8127,  22.852,  -25.533,  -17.816,  -22.737,  40.954,
     0.36621,  0.70489,  -1.5816,  -1.1039,  0.028304,  1.4123,
     1.2899,  2.6872,  -5.7247,  -4.2687,  0.35335,  4.9215,
     -3.1117,  -5.7153,  0.34834,  5.0147,  -10.188,  13.643,
     -0.28832,  -0.0073236,  -1.3472,  0.84382,  -8.3824,  4.8053,
     -7.9577,  13.85,  -68.204,  -19.073,  5.8125,  60.467,
     -0.57148,  2.1825,  -5.0653,  -2.9004,  2.2047,  -3.4601,
     -18.989,  29.308,  18.407,  -27.067,  -30.294,  15.929,
     -2.3055,  5.6793,  1.1979,  -5.6026,  -5.508,  3.2292,
     6.3015,  3.4258,  -15.446,  -39.538,  32.707,  11.938,
     12.07,  4.7978,  -33.863,  -69.47,  68.71,  24.653,
     -0.34545,  -3.0767,  -0.74642,  5.1766,  -1.4933,  1.3448,
     -1.2145,  -11.29,  -2.4107,  18.852,  -5.6124,  4.4147,
     47.316,  -47.972,  4.0148,  1.8982,  47.847,  -17.07,
     2.5151,  2.193,  -2.01,  -4.612,  6.7911,  0.79304,
     -1.1264,  -28.66,  -6.2788,  24.867,  -29.898,  11.815,
     -0.11664,  0.26335,  3.0873,  -6.0973,  0.091276,  -3.3987,
     -30.783,  -185.59,  50.834,  202.82,  45.842,  -73.375,
     -1.1628,  -16.686,  2.4024,  13.562,  10.416,  -4.9705,
     6.3015,  -15.446,  3.4258,  11.938,  32.707,  -39.538,
     12.07,  -33.863,  4.7978,  24.653,  68.71,  -69.47,
     0.34545,  0.74642,  3.0767,  -1.3448,  1.4933,  -5.1766,
     1.2145,  2.4107,  11.29,  -4.4147,  5.6124,  -18.852,
     -1.1264,  -6.2788,  -28.66,  11.815,  -29.898,  24.867,
     -0.11664,  3.0873,  0.26335,  -3.3987,  0.091276,  -6.0973,
     47.316,  4.0148,  -47.972,  -17.07,  47.847,  1.8982,
     2.5151,  -2.01,  2.193,  0.79304,  6.7911,  -4.612,
     -30.783,  50.834,  -185.59,  -73.375,  45.842,  202.82,
     -1.1628,  2.4024,  -16.686,  -4.9705,  10.416,  13.562,
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
constexpr PidGains kLeftLegTurnPid{20.0f, 0.0f, 0.0f, 15.0f, 0.0f};   ///< 左腿摆角速度 PID
constexpr PidGains kRightLegTurnPid{20.0f, 0.0f, 0.0f, 15.0f, 0.0f};  ///< 右腿摆角速度 PID
constexpr PidGains kLeftLegTurnPidManual{16.0f, 0.0f, 0.0f, 12.0f, 0.0f};  ///< 左腿摆角速度 PID（手动倒地恢复）
constexpr PidGains kRightLegTurnPidManual{16.0f, 0.0f, 0.0f, 12.0f, 0.0f};  ///< 右腿摆角速度 PID（手动倒地恢复）

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
constexpr float kTargetForwardSpeedMaxMps = 2.4f;         ///< 最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.f;  ///< 高腿长模式最大前进速度 [m/s]
constexpr float kMaxSafeYawRateRadS = 4.0f;               ///< 摩擦圆最大安全偏航速率 [rad/s]
constexpr float kVxInputDeadbandNorm = 0.1f;              ///< 前进输入死区
constexpr float kVyInputDeadbandNorm = 0.1f;              ///< 平移输入死区
constexpr float kYawFollowRampStepRadS = 0.065f;           ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
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
constexpr float kExpectedThetaLlBiasRadMidLeg = 0.03f;  ///< 中腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLeg = 0.03f;  ///< 中腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadHighLeg = 0.02f;  ///< 高腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadHighLeg = 0.02f;  ///< 高腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = 0.024f;         ///< 期望机体俯仰偏置 [rad]

// ==== 速度斜坡参数 ====
constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.008f};    ///< 低腿长速度斜坡
constexpr SdotRampParams kSdotRampMidLeg{0.007f, 0.009f};   ///< 中腿长速度斜坡(C 键触发)
constexpr SdotRampParams kSdotRampMidLegG{0.0018f, 0.01f};  ///< 中腿长速度斜坡（G 键触发）
constexpr SdotRampParams kSdotRampHighLeg{0.002f, 0.005f};  ///< 高腿长速度斜坡

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
constexpr float kLeftPhi1OffsetRad = 1.38f + M_PI;  ///< 左腿前关节零位偏移 [rad]
constexpr float kLeftPhi4OffsetRad = 0.86f;         ///< 左腿后关节零位偏移 [rad]
// constexpr float kRightPhi1OffsetRad = 1.38f + M_PI -2.08f;  ///< 右腿前关节零位偏移 [rad]
// constexpr float kRightPhi4OffsetRad = 0.86f - 2.5f;         ///< 右腿后关节零位偏移 [rad]
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
constexpr uint8_t kRobotId = 4U;          ///< 机器人 ID
constexpr float kBulletSpeedMps = 23.0f;  ///< 弹速 [m/s]
// constexpr PidGains kYawPositionPid{30.0f, 0.f, 1.5f, 10.0f, 1.5f};  ///< 自瞄偏航位置 PID
// constexpr PidGains kYawSpeedPid{0.7f, 0.0f, 0.0f, 8.0f, 0.4f};    ///< 自瞄偏航速度 PID
// constexpr PidGains kPitchPositionPid{30.0f, 0.0f, 0.5f, 10.0f, 0.4f};  ///< 自瞄俯仰位置 PID
// constexpr PidGains kPitchSpeedPid{0.7f, 0.0f, 0.0f, 8.0f, 0.0f};     ///< 自瞄俯仰速度 PID

// constexpr PidGains kYawPositionPid{30.0f, 0.0f, 0.5f, 10.0f, 1.0f};    ///< 偏航位置 PID
// constexpr PidGains kYawSpeedPid{0.6f, 0.0f, 0.0f, 6.0f, 0.4f};         ///< 偏航速度 PID
// constexpr PidGains kPitchPositionPid{20.0f, 0.0f, 0.8f, 10.0f, 0.4f};  ///< 俯仰位置 PID
// constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 8.0f, 0.0f};      ///< 俯仰速度 PID

constexpr PidGains kYawPositionPid{42.0f, 0.f, 1.5f, 10.0f, 1.5f};  ///< 自瞄偏航位置 PID
constexpr PidGains kYawSpeedPid{0.65f, 0.0f, 0.0f, 8.0f, 0.4f};     ///< 自瞄偏航速度 PID
// constexpr PidGains kPitchPositionPid{22.0f, 0.5f, 0.8f, 10.0f, 1.5f};  ///< 自瞄俯仰位置 PID
// constexpr PidGains kPitchSpeedPid{0.6f, 0.0f, 0.0f, 8.0f, 0.0f};       ///< 自瞄俯仰速度 PID
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
