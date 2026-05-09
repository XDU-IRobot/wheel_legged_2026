#pragma once

#include <array>
#include <cstdint>
#include <librm.hpp>

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
}  // namespace globals

// ── 云台公共（控制周期、力矩上限、重力补偿）──
namespace gimbal {
constexpr float kDefaultDtS = 0.002f;                ///< 云台控制默认周期 [s]
constexpr float kDmTorqueLimitNm = 10.0f;            ///< DM 电机力矩上限 [Nm]
constexpr float kPitchGravityCompensationNm = 1.3f;  ///< 俯仰重力补偿力矩 [Nm]
}  // namespace gimbal

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

// ── CAN 总线 ID 与电机配置 ──
namespace globals {
using namespace common::globals;
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

constexpr std::uint16_t kLeftWheelId = 0x06;   ///< 左轮毂电机 CAN ID
constexpr std::uint16_t kRightWheelId = 0x05;  ///< 右轮毂电机 CAN ID

constexpr std::uint16_t kDmLfMasterId = 0x04;  ///< 左前关节主电机 CAN ID
constexpr std::uint16_t kDmLfSlaveId = 0x03;   ///< 左前关节从电机 CAN ID
constexpr std::uint16_t kDmLbMasterId = 0x06;  ///< 左后关节主电机 CAN ID
constexpr std::uint16_t kDmLbSlaveId = 0x05;   ///< 左后关节从电机 CAN ID
constexpr std::uint16_t kDmRfMasterId = 0x02;  ///< 右前关节主电机 CAN ID
constexpr std::uint16_t kDmRfSlaveId = 0x01;   ///< 右前关节从电机 CAN ID
constexpr std::uint16_t kDmRbMasterId = 0x08;  ///< 右后关节主电机 CAN ID
constexpr std::uint16_t kDmRbSlaveId = 0x07;   ///< 右后关节从电机 CAN ID

const DmMitSettings kDmLfSettings{kDmLfMasterId, kDmLfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmLbSettings{kDmLbMasterId, kDmLbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmRfSettings{kDmRfMasterId, kDmRfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmRbSettings{kDmRbMasterId, kDmRbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
}  // namespace globals

// ── 云台 ──
namespace gimbal {
using namespace common::gimbal;
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

const DmMitSettings kPitchMotorSettings{0x13, 0x12, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
const DmMitSettings kYawMotorSettings{0x21, 0x11, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};

constexpr float kPitchMinRad = -0.35f;  ///< 俯仰角下限 [rad]
constexpr float kPitchMaxRad = 0.7f;    ///< 俯仰角上限 [rad]

constexpr PidGains kYawPositionPid{24.0f, 0.0f, 0.0f, 1000.0f, 1.0f};    ///< 偏航位置 PID
constexpr PidGains kYawSpeedPid{1.f, 0.0f, 0.0f, 10.0f, 0.4f};           ///< 偏航速度 PID
constexpr PidGains kPitchPositionPid{25.0f, 0.0f, 0.0f, 1000.0f, 0.4f};  ///< 俯仰位置 PID
constexpr PidGains kPitchSpeedPid{2.f, 0.0f, 0.0f, 0.0f, 0.0f};          ///< 俯仰速度 PID
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
inline constexpr float kFwReadySpeedThresholdRpm = 4000.0f;                  ///< 摩擦轮就绪判定转速 [rpm]
inline constexpr float kFwTargetSpeedRpm = 3800.0f;                          ///< 摩擦轮目标转速 [rpm]
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
constexpr float kStairClimbThetaThresholdRad = 0.3f;               ///< 双腿摆角均超过此值触发上台阶 [rad]
constexpr float kStairClimbLegLengthM = 0.14f;                     ///< 上台阶收腿目标腿长 [m]
constexpr float kStairClimbPhase0LegLengthM = 0.25f;              ///< 上台阶 Phase 0 转腿目标腿长 [m]
constexpr float kStairClimbThetaTargetRad = 1.35f;                  ///< 上台阶目标腿摆角（统一偏置） [rad]
constexpr std::uint32_t kStairClimbDurationMs = 500U;              ///< 上台阶最长持续时间 [ms]
constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.01f;  ///< 腿长到位容差 [m]
constexpr float kStairClimbThetaNearZeroThresholdRad = 0.1f;  ///< 摆角归零判定阈值（上台阶完成） [rad]
constexpr std::uint32_t kStairClimbPitchStableMs = 600U;       ///< 上台阶完成后俯仰稳定等待时间 [ms]

// ==== 倒地自起 ====
constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;  ///< 倒地确认时间（持续倒地超过此值进入自启） [ms]
constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;  ///< 自启超时（超时后强制停机） [ms]

// ==== 跳跃（低腿长）====
constexpr std::uint32_t kJumpLowPrepMs = 100U;          ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpLowPushMaxMs = 500U;       ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpLowRecoverMs = 150U;       ///< 跳跃回收阶段持续时间 [ms]
constexpr float kJumpLowPrepLegLengthM = 0.13f;         ///< 跳跃预备阶段目标腿长 [m]（蓄力收腿）
constexpr float kJumpLowPushLegLengthM = 0.25f;         ///< 跳跃蹬伸阶段目标腿长 [m]（爆发推地）
constexpr float kJumpLowRecoverLegLengthM = 0.16f;      ///< 跳跃回收阶段目标腿长 [m]（落地缓冲）
constexpr float kJumpLowPushReachedLegLengthM = 0.26f;  ///< 蹬伸到位判定腿长 [m]（到达此值提前结束 push）

// ==== 跳跃（中腿长）====
constexpr std::uint32_t kJumpMidPrepMs = 100U;          ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpMidPushMaxMs = 1000U;      ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpMidRecoverMs = 450U;       ///< 跳跃回收阶段持续时间 [ms]
constexpr float kJumpMidPrepLegLengthM = 0.13f;         ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpMidPushLegLengthM = 0.25f;         ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpMidRecoverLegLengthM = 0.15f;      ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpMidPushReachedLegLengthM = 0.23f;  ///< 蹬伸到位判定腿长 [m]

// ==== 基本运动（腿长档位）====
constexpr float kLowLegLengthM = 0.11f;      ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.18f;      ///< 中腿长档位目标腿长 [m]
constexpr float kHighLegLengthM = 0.32f;     ///< 高腿长档位目标腿长 [m]
constexpr float kLegLengthRampTimeS = 0.5f;  ///< 腿长切换斜坡时间 [s]（从低到高腿长的过渡时间）
}  // namespace chassis_fsm

// ── 底盘控制 ──
namespace chassis {

// ==== 物理/机械参数 ====
constexpr float kControlDtS = 0.002f;  ///< 底盘控制周期 [s]（500Hz）
constexpr float kLegL1M = 0.215f;      ///< 五连杆主动杆长度 [m]
constexpr float kLegL2M = 0.254f;      ///< 五连杆从动杆长度 [m]

// -- 弹簧模型参数（用于计算关节等效弹簧补偿力矩）--
constexpr float kSpringModelA = 1082.f;      ///< 弹簧模型系数 A
constexpr float kSpringModelB = 1070.f;      ///< 弹簧模型系数 B
constexpr float kSpringModelC = 411.f;       ///< 弹簧模型系数 C
constexpr float kSpringModelD = 203.8f;      ///< 弹簧模型系数 D
constexpr float kSpringPhaseDivisor = 18.f;  ///< 弹簧相位除数

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
constexpr float kStandupThetaThresholdRad = 1.1f;  ///< 起立完成判定：双腿摆角绝对值低于此值后允许轮端输出 [rad]
constexpr float kPostureRollMinRad = -0.5f;      ///< 横滚角安全下限 [rad]
constexpr float kPostureRollMaxRad = 0.5f;       ///< 横滚角安全上限 [rad]
constexpr float kPostureThetaBMinRad = -0.8f;    ///< 机体俯仰角安全下限 [rad]
constexpr float kPostureThetaBMaxRad = 0.8f;     ///< 机体俯仰角安全上限 [rad]
constexpr float kPostureThetaLegMinRad = -0.8f;  ///< 腿摆角安全下限 [rad]
constexpr float kPostureThetaLegMaxRad = 1.55f;  ///< 腿摆角安全上限 [rad]
constexpr float kLegRecoverThetaDotTarget = -2.0f;  ///< 倒地恢复时腿摆角速度目标 [rad/s]（负号表示前摆方向）
constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;  ///< 倒地恢复零力矩区间下限 [rad]
constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;  ///< 倒地恢复零力矩区间上限 [rad]

// -- 离地检测 --
constexpr float kOffGroundSupportForceThresholdN = 10.0f;  ///< 支撑力低于此值判定为离地 [N]

// ==== 物理参数（变体专属）====
constexpr float kBodyMassKg = 24.0f;          ///< 机体质量 [kg]
constexpr float kSpringTorqueScale = 90.0f;  ///< 弹簧力矩缩放系数（调参用）

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = -0.073f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵 — 40 组多项式系数）====
/// 由 MATLAB 离线拟合得到，p(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l² + p11*l_l*l_r + p02*l_r²
/// 共 40 行，对应 4×10 增益矩阵 K 的 40 个元素（按行主序展平）
/// 每行 6 个系数：[p00, p10, p01, p20, p11, p02]
constexpr std::array<float, 240> kCtrlP{
-1.7283,  -15.237,  10.752,  20.178,  -6.9625,  -8.8702,
     -3.6871,  -22.77,  19.847,  32.964,  -18.069,  -15.297,
     -0.80734,  2.7559,  -1.6917,  -2.0622,  -0.5922,  2.4419,
     -1.6494,  5.921,  -3.8205,  -4.0814,  -1.8615,  5.6707,
     -7.3779,  -73.036,  15.153,  60.045,  13.042,  -20.697,
     -0.51095,  -4.8298,  2.087,  -4.1979,  2.8421,  -2.8044,
     -3.4457,  -0.62202,  -8.749,  17.39,  -10.733,  2.9099,
     -0.32617,  -1.1142,  -0.14729,  4.1496,  -7.8853,  0.26558,
     -23.185,  8.5522,  39.477,  45.257,  -43.756,  -30.201,
     -2.4594,  0.73505,  6.1741,  4.091,  -6.103,  -5.529,
     -1.7283,  10.752,  -15.237,  -8.8702,  -6.9625,  20.178,
     -3.6871,  19.847,  -22.77,  -15.297,  -18.069,  32.964,
     0.80734,  1.6917,  -2.7559,  -2.4419,  0.5922,  2.0622,
     1.6494,  3.8205,  -5.921,  -5.6707,  1.8615,  4.0814,
     -3.4457,  -8.749,  -0.62202,  2.9099,  -10.733,  17.39,
     -0.32617,  -0.14729,  -1.1142,  0.26558,  -7.8853,  4.1496,
     -7.3779,  15.153,  -73.036,  -20.697,  13.042,  60.045,
     -0.51095,  2.087,  -4.8298,  -2.8044,  2.8421,  -4.1979,
     -23.185,  39.477,  8.5522,  -30.201,  -43.756,  45.257,
     -2.4594,  6.1741,  0.73505,  -5.529,  -6.103,  4.091,
     10.405,  21.088,  -36.326,  -82.449,  55.531,  25.693,
     19.904,  30.336,  -73.3,  -138.89,  119.72,  48.056,
     -0.72868,  -8.4759,  -1.4346,  13.39,  -5.1791,  2.6428,
     -1.5067,  -18.446,  -1.9304,  28.402,  -10.525,  3.6408,
     81.901,  -64.977,  -0.25445,  -36.059,  80.487,  -17.089,
     3.5937,  7.4546,  -4.5191,  -8.9899,  6.7108,  2.9208,
     -1.1642,  -39.643,  -7.7752,  17.631,  -49.199,  11.894,
     -0.066191,  2.2113,  4.1898,  -14.086,  6.1782,  -8.9275,
     -41.519,  -316.79,  69.783,  283.42,  106.66,  -95.973,
     -0.45391,  -20.977,  -1.1447,  10.672,  18.998,  -1.5859,
     10.405,  -36.326,  21.088,  25.693,  55.531,  -82.449,
     19.904,  -73.3,  30.336,  48.056,  119.72,  -138.89,
     0.72868,  1.4346,  8.4759,  -2.6428,  5.1791,  -13.39,
     1.5067,  1.9304,  18.446,  -3.6408,  10.525,  -28.402,
     -1.1642,  -7.7752,  -39.643,  11.894,  -49.199,  17.631,
     -0.066191,  4.1898,  2.2113,  -8.9275,  6.1782,  -14.086,
     81.901,  -0.25445,  -64.977,  -17.089,  80.487,  -36.059,
     3.5937,  -4.5191,  7.4546,  2.9208,  6.7108,  -8.9899,
     -41.519,  69.783,  -316.79,  -95.973,  106.66,  283.42,
     -0.45391,  -1.1447,  -20.977,  -1.5859,  18.998,  10.672,
};

// ==== 基本运动（PID 增益）====
constexpr PidGains kLeftL0Pid{7000.0f, 0.1f, 70000.0f, 190.0f, 10.0f};   ///< 左腿腿长 PID（常规）
constexpr PidGains kRightL0Pid{7000.0f, 0.1f, 70000.0f, 190.0f, 10.0f};  ///< 右腿腿长 PID（常规）
constexpr PidGains kRollPid{800.0f, 0.1f, 4000.0f, 60.0f, 10.0f};          ///< 横滚平衡 PID

// ==== 跳跃（PID 增益）====
constexpr PidGains kLeftL0PidJumpTwo{8000.0f, 0.0f, 70000.0f, 250.0f, 0.0f};       ///< 左腿蹬伸 PID（JumpPush）
constexpr PidGains kRightL0PidJumpTwo{8000.0f, 0.0f, 70000.0f, 250.0f, 0.0f};      ///< 右腿蹬伸 PID（JumpPush）
constexpr PidGains kLeftL0PidJumpThree{12000.0f, 0.15f, 80000.0f, 190.0f, 30.0f};  ///< 左腿回收 PID（JumpRecover）
constexpr PidGains kRightL0PidJumpThree{12000.0f, 0.15f, 80000.0f, 190.0f, 30.0f};  ///< 右腿回收 PID（JumpRecover）

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};   ///< 左腿摆角速度 PID（倒地恢复用）
constexpr PidGains kRightLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};  ///< 右腿摆角速度 PID（倒地恢复用）

// ==== 上台阶（腿摆角 PID）====
constexpr PidGains kStairClimbThetaPid{6.0f, 0.0f, 1.0f, 15.0f, 0.0f};  ///< 上台阶腿摆角 PID（位置环，跟踪 kStairClimbThetaTargetRad）
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

constexpr float kControlLoopDtS = 0.002f;  ///< 控制环周期 [s]

// -- 摇杆/键鼠输入归一化 --
constexpr std::int16_t kDr16AxisMaxAbs = 660;     ///< DR16 摇杆轴最大绝对值（用于归一化到 [-1,1]）
constexpr float kRcStickMax = 660.0f;             ///< RC 摇杆最大值（用于积分目标速率计算）
constexpr float kTcMouseMax = 200.0f;             ///< 图传鼠标增量最大值（用于积分目标速率计算）
constexpr float kRcYawRateMaxRadS = -2.5f;        ///< RC 摇杆满偏时偏航积分速率 [rad/s]
constexpr float kRcPitchRateMaxRadS = 1.5f;       ///< RC 摇杆满偏时俯仰积分速率 [rad/s]
constexpr float kTcMouseYawRateMaxRadS = -2.0f;   ///< 图传鼠标满偏时偏航积分速率 [rad/s]
constexpr float kTcMousePitchRateMaxRadS = 1.0f;  ///< 图传鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kPitchTargetMinRad = -0.35f;      ///< RC 积分俯仰目标下限 [rad]
constexpr float kPitchTargetMaxRad = 0.7f;        ///< RC 积分俯仰目标上限 [rad]

// -- 云台启动归中判稳 --
constexpr float kGimbalStartupYawAlignErrorRad = 0.04f;           ///< 归中完成位置误差阈值 [rad]
constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;            ///< 归中完成速度阈值 [rad/s]
constexpr std::uint32_t kGimbalStartupYawAlignStableTicks = 50U;  ///< 归中判稳所需连续满足周期数

// -- 偏航就绪判稳（允许底盘纵向驱动前确认偏航跟踪到位）--
constexpr float kYawFollowDriveReadyErrorRad = 0.04f;           ///< 偏航就绪位置误差阈值 [rad]
constexpr float kYawFollowDriveReadyVelRadS = 0.25f;            ///< 偏航就绪速度阈值 [rad/s]
constexpr std::uint32_t kYawFollowDriveReadyStableTicks = 50U;  ///< 偏航就绪判稳所需连续周期数

// ==== 基本运动 ====
constexpr float kTargetForwardSpeedMaxMps = 2.1f;         ///< 最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 0.5f;  ///< 高腿长模式最大前进速度 [m/s]
constexpr float kVxInputDeadbandNorm = 0.1f;              ///< 前进输入死区（归一化值，低于此忽略）
constexpr float kVyInputDeadbandNorm = 0.1f;              ///< 平移输入死区（归一化值）
constexpr float kYawFollowRampStepRadS = 0.05f;           ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.15f;  ///< 位置锚定冻结速度阈值 [m/s]（车速低于此值时锁定位置）
constexpr float kYawFollowFixedTargetRad = -1.9f;      ///< 偏航跟随固定目标偏置角 [rad]（前进方向）
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;  ///< 偏航跟随侧向目标偏置角 [rad]（±π/2）
constexpr PidGains kYawFollowPid{8.0f, 0.0f, 1.f, 4.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置（腿摆角/机体俯仰）====
constexpr float kExpectedThetaLlBiasRad = -0.067f;  ///< 期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRad = -0.067f;  ///< 期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = -0.123f;  ///< 期望机体俯仰偏置 [rad]

// ==== 速度斜坡参数（按腿长档位分级）====
constexpr SdotRampParams kSdotRampLowLeg{0.006f, 0.006f};    ///< 低腿长速度斜坡（加速/制动步长）
constexpr SdotRampParams kSdotRampMidLeg{0.003f, 0.003f};   ///< 中腿长速度斜坡
constexpr SdotRampParams kSdotRampHighLeg{0.002f, 0.002f};  ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.005f;  ///< 小陀螺偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS = 7.5f;   ///< 小陀螺目标自旋角速度 [rad/s]
constexpr float kSpinTranslationGain = 1.0f;  ///< 小陀螺平移增益（将云台系前进指令投影到车体系的比例）
constexpr float kSpinThetaLlBiasRad = -0.02f;  ///< 小陀螺时左腿摆角偏置 [rad]
constexpr float kSpinLegLengthBiasM = 0.f;  ///< 小陀螺时腿长偏差（左+右-）[m]
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
constexpr float kLeftPhi1OffsetRad = -0.05f + M_PI;          ///< 左腿前关节零位偏移 [rad]
constexpr float kLeftPhi4OffsetRad = -0.59 + 0.07f;          ///< 左腿后关节零位偏移 [rad]
constexpr float kRightPhi1OffsetRad = 3.04 + M_PI - 0.318f;  ///< 右腿前关节零位偏移 [rad]
constexpr float kRightPhi4OffsetRad = -2.17 + 0.05f;         ///< 右腿后关节零位偏移 [rad]
}  // namespace state_estimator

// ── 腿部运动学 ──
namespace leg_kinematics {
constexpr float kDefaultDtS = 0.002f;  ///< 运动学默认解算周期 [s]
constexpr float kMinSin = 1e-5f;       ///< 正弦值下限（避免除零奇异）
constexpr float kMinLen = 1e-5f;       ///< 长度下限（避免除零奇异）
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
constexpr PidGains kYawPositionPid{18.0f, 0.5f, 0.02f, 1000.0f, 2.0f};    ///< 自瞄偏航位置 PID
constexpr PidGains kYawSpeedPid{0.8f, 0.0f, 0.0f, 10.0f, 0.3f};           ///< 自瞄偏航速度 PID
constexpr PidGains kPitchPositionPid{20.0f, 0.5f, 0.02f, 1000.0f, 1.5f};  ///< 自瞄俯仰位置 PID
constexpr PidGains kPitchSpeedPid{1.5f, 0.0f, 0.0f, 10.0f, 0.3f};         ///< 自瞄俯仰速度 PID
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
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

constexpr std::uint16_t kLeftWheelId = 0x02;   ///< 左轮毂电机 CAN ID
constexpr std::uint16_t kRightWheelId = 0x03;  ///< 右轮毂电机 CAN ID

constexpr std::uint16_t kDmLfMasterId = 0x17;  ///< 左前关节主电机 CAN ID
constexpr std::uint16_t kDmLfSlaveId = 0x07;   ///< 左前关节从电机 CAN ID
constexpr std::uint16_t kDmLbMasterId = 0x14;  ///< 左后关节主电机 CAN ID
constexpr std::uint16_t kDmLbSlaveId = 0x04;   ///< 左后关节从电机 CAN ID
constexpr std::uint16_t kDmRfMasterId = 0x16;  ///< 右前关节主电机 CAN ID
constexpr std::uint16_t kDmRfSlaveId = 0x06;   ///< 右前关节从电机 CAN ID
constexpr std::uint16_t kDmRbMasterId = 0x15;  ///< 右后关节主电机 CAN ID
constexpr std::uint16_t kDmRbSlaveId = 0x05;   ///< 右后关节从电机 CAN ID

constexpr std::uint16_t kFricLeftId = 0x02;   ///< 左摩擦轮电机 CAN ID
constexpr std::uint16_t kFricRightId = 0x04;  ///< 右摩擦轮电机 CAN ID
constexpr std::uint16_t kDialId = 0x01;       ///< 拨盘电机 CAN ID

const DmMitSettings kDmLfSettings{kDmLfMasterId, kDmLfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmLbSettings{kDmLbMasterId, kDmLbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmRfSettings{kDmRfMasterId, kDmRfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmRbSettings{kDmRbMasterId, kDmRbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
}  // namespace globals

// ── 云台 ──
namespace gimbal {
using namespace common::gimbal;
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

const DmMitSettings kPitchMotorSettings{0x05, 0x04, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
const DmMitSettings kYawMotorSettings{0x10, 0x09, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};

constexpr float kPitchMinRad = -0.3f;  ///< 俯仰角下限 [rad]
constexpr float kPitchMaxRad = 0.25f;  ///< 俯仰角上限 [rad]

inline constexpr PidGains kYawPositionPid{25.0f, 0.0f, 0.05f, 10.0f, 1.0f};    ///< 偏航位置 PID
inline constexpr PidGains kYawSpeedPid{0.6f, 0.0f, 0.0f, 6.0f, 0.4f};          ///< 偏航速度 PID
inline constexpr PidGains kPitchPositionPid{23.0f, 0.0f, 0.15f, 10.0f, 0.4f};  ///< 俯仰位置 PID
inline constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 8.0f, 0.0f};       ///< 俯仰速度 PID
}  // namespace gimbal

// ── 发射机构（双摩擦轮 + M3508 拨盘）──
namespace shoot {
inline constexpr int kFrictionWheelCount = 2;                            ///< 摩擦轮数量
constexpr float kFricSpeedTargetRpm = 6900.0f;                           ///< 摩擦轮目标转速 [rpm]
constexpr PidGains kFricSpeedPid{20.0f, 1.0f, 0.0f, 16000.0f, 2000.0f};  ///< 摩擦轮速度 PID
constexpr PidGains kDialSpeedPid{15.0f, 0.f, 0.2f, 2000.0f, 0.0f};       ///< 拨盘速度 PID
constexpr PidGains kDialPositionPid{8.0f, 0.f, 0.0f, 12000.0f, 0.0f};    ///< 拨盘位置 PID
constexpr int16_t kDialFireThreshold = -600;                             ///< 发射触发拨轮阈值
constexpr float kShootFrequencyHz = 10.0f;                               ///< 发射频率 [Hz]
}  // namespace shoot

// ── 底盘状态机 ──
namespace chassis_fsm {

// ==== 上台阶 ====
constexpr float kStairClimbThetaThresholdRad = 0.5f;               ///< 双腿摆角均超过此值触发上台阶 [rad]
constexpr float kStairClimbLegLengthM = 0.14f;                     ///< 上台阶收腿目标腿长 [m]
constexpr float kStairClimbPhase0LegLengthM = 0.20f;               ///< 上台阶 Phase 0 转腿目标腿长 [m]
constexpr float kStairClimbThetaTargetRad = 0.2f;                  ///< 上台阶目标腿摆角（统一偏置） [rad]
constexpr std::uint32_t kStairClimbDurationMs = 3000U;             ///< 上台阶最长持续时间 [ms]
constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.03f;  ///< 腿长到位容差 [m]
constexpr float kStairClimbThetaNearZeroThresholdRad = 0.1f;  ///< 摆角归零判定阈值（上台阶完成） [rad]
constexpr std::uint32_t kStairClimbPitchStableMs = 1000U;     ///< 上台阶完成后俯仰稳定等待时间 [ms]

// ==== 倒地自启 ====
constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;        ///< 倒地确认时间 [ms]
constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;  ///< 自启超时 [ms]

// ==== 跳跃 ====
constexpr std::uint32_t kJumpLowPrepMs = 200U;          ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpLowPushMaxMs = 1000U;      ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpLowRecoverMs = 250U;       ///< 跳跃回收阶段持续时间 [ms]
constexpr float kJumpLowPrepLegLengthM = 0.13f;         ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpLowPushLegLengthM = 0.25f;         ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpLowRecoverLegLengthM = 0.20f;      ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpLowPushReachedLegLengthM = 0.21f;  ///< 蹬伸到位判定腿长 [m]

// ==== 跳跃（中腿长）====
constexpr std::uint32_t kJumpMidPrepMs = 200U;          ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpMidPushMaxMs = 1000U;      ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpMidRecoverMs = 250U;       ///< 跳跃回收阶段持续时间 [ms]
constexpr float kJumpMidPrepLegLengthM = 0.13f;         ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpMidPushLegLengthM = 0.22f;         ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpMidRecoverLegLengthM = 0.20f;      ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpMidPushReachedLegLengthM = 0.19f;  ///< 蹬伸到位判定腿长 [m]

// ==== 基本运动（腿长档位）====
constexpr float kLowLegLengthM = 0.127f;     ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.18f;      ///< 中腿长档位目标腿长 [m]
constexpr float kHighLegLengthM = 0.3f;      ///< 高腿长档位目标腿长 [m]
constexpr float kLegLengthRampTimeS = 0.8f;  ///< 腿长切换斜坡时间 [s]
}  // namespace chassis_fsm

// ── 底盘控制 ──
namespace chassis {

// ==== 物理/机械参数 ====
constexpr float kControlDtS = 0.002f;  ///< 底盘控制周期 [s]（500Hz）
constexpr float kLegL1M = 0.215f;      ///< 五连杆主动杆长度 [m]
constexpr float kLegL2M = 0.254f;      ///< 五连杆从动杆长度 [m]

// -- 弹簧模型参数 --
constexpr float kSpringModelA = 1082.0f;      ///< 弹簧模型系数 A
constexpr float kSpringModelB = 1070.0f;      ///< 弹簧模型系数 B
constexpr float kSpringModelC = 404.0f;       ///< 弹簧模型系数 C
constexpr float kSpringModelD = 177.0f;       ///< 弹簧模型系数 D
constexpr float kSpringPhaseDivisor = 18.0f;  ///< 弹簧相位除数

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
constexpr float kStandupThetaThresholdRad = 0.85f;  ///< 起立完成判定：双腿摆角绝对值低于此值后允许轮端输出 [rad]
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

// ==== 物理参数（变体专属）====
constexpr float kBodyMassKg = 22.0f;         ///< 机体质量 [kg]
constexpr float kSpringTorqueScale = 90.0f;  ///< 弹簧力矩缩放系数

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.003f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵）====
constexpr std::array<float, 240> kCtrlP{
    -3.5611,  -30.773,  24.204,   49.27,   -22.027,  -19.477, -5.9313,  -36.889,  34.858,   62.344,  -37.433,  -26.905,
    -0.38518, 1.8237,   -0.73941, -1.668,  -0.18375, 1.2356,  -1.5105,  7.3371,   -3.1349,  -6.4854, -1.0271,  5.2894,
    -10.956,  -78.834,  16.086,   69.139,  7.6072,   -22.308, -0.72054, -6.7038,  3.0018,   -4.3907, 2.8223,   -3.9011,
    -3.6446,  -0.68148, -3.3396,  20.145,  -20.964,  2.93,    -0.33916, -2.0985,  0.045752, 7.2851,  -12.66,   1.5759,
    -23.053,  29.876,   32.954,   4.3946,  -33.814,  -31.656, -2.7417,  1.6265,   6.9444,   3.506,   -6.8174,  -6.9601,
    -3.5611,  24.204,   -30.773,  -19.477, -22.027,  49.27,   -5.9313,  34.858,   -36.889,  -26.905, -37.433,  62.344,
    0.38518,  0.73941,  -1.8237,  -1.2356, 0.18375,  1.668,   1.5105,   3.1349,   -7.3371,  -5.2894, 1.0271,   6.4854,
    -3.6446,  -3.3396,  -0.68148, 2.93,    -20.964,  20.145,  -0.33916, 0.045752, -2.0985,  1.5759,  -12.66,   7.2851,
    -10.956,  16.086,   -78.834,  -22.308, 7.6072,   69.139,  -0.72054, 3.0018,   -6.7038,  -3.9011, 2.8223,   -4.3907,
    -23.053,  32.954,   29.876,   -31.656, -33.814,  4.3946,  -2.7417,  6.9444,   1.6265,   -6.9601, -6.8174,  3.506,
    9.1291,   -1.9062,  -17.612,  -48.654, 47.114,   13.514,  13.502,   -2.1689,  -33.382,  -67.639, 77.333,   24.071,
    -0.3831,  -2.6376,  -0.64534, 4.511,   -1.1575,  1.2208,  -1.5003,  -10.756,  -2.3566,  18.277,  -4.8971,  4.5585,
    47.384,   -64.757,  3.9902,   32.704,  44.328,   -16.67,  2.4081,   2.0579,   -2.1393,  -5.4513, 8.0842,   0.77762,
    -2.1265,  -25.244,  -1.4489,  20.509,  -27.359,  9.4924,  -0.11488, 0.24722,  3.457,    -6.3006, -0.48196, -2.5744,
    -37.196,  -184.77,  55.678,   216.22,  38.939,   -81.1,   -1.4163,  -16.78,   2.7189,   14.63,   10.281,   -5.6197,
    9.1291,   -17.612,  -1.9062,  13.514,  47.114,   -48.654, 13.502,   -33.382,  -2.1689,  24.071,  77.333,   -67.639,
    0.3831,   0.64534,  2.6376,   -1.2208, 1.1575,   -4.511,  1.5003,   2.3566,   10.756,   -4.5585, 4.8971,   -18.277,
    -2.1265,  -1.4489,  -25.244,  9.4924,  -27.359,  20.509,  -0.11488, 3.457,    0.24722,  -2.5744, -0.48196, -6.3006,
    47.384,   3.9902,   -64.757,  -16.67,  44.328,   32.704,  2.4081,   -2.1393,  2.0579,   0.77762, 8.0842,   -5.4513,
    -37.196,  55.678,   -184.77,  -81.1,   38.939,   216.22,  -1.4163,  2.7189,   -16.78,   -5.6197, 10.281,   14.63,
};

// ==== 基本运动（PID 增益）====
constexpr PidGains kLeftL0Pid{7500.0f, 0.04f, 90000.0f, 170.0f, 10.0f};   ///< 左腿腿长 PID（常规）
constexpr PidGains kRightL0Pid{8500.0f, 0.04f, 90000.0f, 170.0f, 10.0f};  ///< 右腿腿长 PID（常规）
constexpr PidGains kRollPid{800.0f, 0.0f, 200.0f, 180.0f, 0.0f};          ///< 横滚平衡 PID

// ==== 跳跃（PID 增益）====
constexpr PidGains kLeftL0PidJumpTwo{7000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};       ///< 左腿蹬伸 PID
constexpr PidGains kRightL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};      ///< 右腿蹬伸 PID
constexpr PidGains kLeftL0PidJumpThree{7500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};   ///< 左腿回收 PID
constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};  ///< 右腿回收 PID

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};   ///< 左腿摆角速度 PID
constexpr PidGains kRightLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};  ///< 右腿摆角速度 PID

// ==== 上台阶（腿摆角 PID）====
constexpr PidGains kStairClimbThetaPid{80.0f, 0.0f, 5.0f, 60.0f, 30.0f};  ///< 上台阶腿摆角 PID（位置环，跟踪 kStairClimbThetaTargetRad）
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

constexpr float kControlLoopDtS = 0.002f;  ///< 控制环周期 [s]

// -- 摇杆/键鼠输入归一化 --
constexpr std::int16_t kDr16AxisMaxAbs = 660;     ///< DR16 摇杆轴最大绝对值
constexpr float kRcStickMax = 660.0f;             ///< RC 摇杆最大值
constexpr float kTcMouseMax = 200.0f;             ///< 图传鼠标增量最大值
constexpr float kRcYawRateMaxRadS = -2.5f;        ///< RC 摇杆满偏时偏航积分速率 [rad/s]
constexpr float kRcPitchRateMaxRadS = 1.5f;       ///< RC 摇杆满偏时俯仰积分速率 [rad/s]
constexpr float kTcMouseYawRateMaxRadS = -2.0f;   ///< 图传鼠标满偏时偏航积分速率 [rad/s]
constexpr float kTcMousePitchRateMaxRadS = 1.0f;  ///< 图传鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kPitchTargetMinRad = -0.35f;      ///< RC 积分俯仰目标下限 [rad]
constexpr float kPitchTargetMaxRad = 0.25f;       ///< RC 积分俯仰目标上限 [rad]

// -- 云台启动归中判稳 --
constexpr float kGimbalStartupYawAlignErrorRad = 0.04f;           ///< 归中完成位置误差阈值 [rad]
constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;            ///< 归中完成速度阈值 [rad/s]
constexpr std::uint32_t kGimbalStartupYawAlignStableTicks = 50U;  ///< 归中判稳所需连续满足周期数

// -- 偏航就绪判稳 --
constexpr float kYawFollowDriveReadyErrorRad = 0.04f;           ///< 偏航就绪位置误差阈值 [rad]
constexpr float kYawFollowDriveReadyVelRadS = 0.25f;            ///< 偏航就绪速度阈值 [rad/s]
constexpr std::uint32_t kYawFollowDriveReadyStableTicks = 50U;  ///< 偏航就绪判稳所需连续周期数

// ==== 基本运动 ====
constexpr float kTargetForwardSpeedMaxMps = 2.1f;          ///< 最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 0.9f;   ///< 高腿长模式最大前进速度 [m/s]
constexpr float kVxInputDeadbandNorm = 0.05f;              ///< 前进输入死区
constexpr float kVyInputDeadbandNorm = 0.05f;              ///< 平移输入死区
constexpr float kYawFollowRampStepRadS = 0.05f;            ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.35f;  ///< 位置锚定冻结速度阈值 [m/s]
constexpr float kYawFollowFixedTargetRad = 0.f;            ///< 偏航跟随固定目标偏置角 [rad]
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;      ///< 偏航跟随侧向目标偏置角 [rad]
constexpr PidGains kYawFollowPid{10.0f, 0.0f, 2.2f, 10.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置 ====
constexpr float kExpectedThetaLlBiasRad = 0.13f;  ///< 期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRad = 0.13f;  ///< 期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = 0.0f;    ///< 期望机体俯仰偏置 [rad]

// ==== 速度斜坡参数 ====
constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.01f};     ///< 低腿长速度斜坡
constexpr SdotRampParams kSdotRampMidLeg{0.007f, 0.007f};   ///< 中腿长速度斜坡
constexpr SdotRampParams kSdotRampHighLeg{0.005f, 0.005f};  ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.02f;  ///< 小陀螺偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS = 6.0f;  ///< 小陀螺目标自旋角速度 [rad/s]
constexpr float kSpinTranslationGain = 1.0f;   ///< 小陀螺平移增益
constexpr float kSpinThetaLlBiasRad = 0.f;     ///< 小陀螺时左腿摆角偏置 [rad]
constexpr float kSpinLegLengthBiasM = 0.0f;  ///< 小陀螺时腿长偏差（左+右-）[m]
}  // namespace control_loop

// ── 执行器 ──
namespace actuators {
using namespace common::actuators;

constexpr float kLeftWheelTorqueToCurrent = 2500.0f;   ///< 左轮力矩→电流转换系数
constexpr float kRightWheelTorqueToCurrent = 2300.0f;  ///< 右轮力矩→电流转换系数
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
constexpr float kLeftPhi1OffsetRad = kPi - 2.94f;  ///< 左腿前关节零位偏移 [rad]
constexpr float kLeftPhi4OffsetRad = 0.59f;        ///< 左腿后关节零位偏移 [rad]
constexpr float kRightPhi1OffsetRad = kPi + 2.4f;  ///< 右腿前关节零位偏移 [rad]
constexpr float kRightPhi4OffsetRad = -1.87f;      ///< 右腿后关节零位偏移 [rad]
}  // namespace state_estimator

// ── 腿部运动学 ──
namespace leg_kinematics {
constexpr float kDefaultDtS = 0.002f;  ///< 运动学默认解算周期 [s]
constexpr float kMinSin = 1e-5f;       ///< 正弦值下限（避免除零奇异）
constexpr float kMinLen = 1e-5f;       ///< 长度下限（避免除零奇异）
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
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

constexpr std::uint16_t kLeftWheelId = 0x06;   ///< 左轮毂电机 CAN ID
constexpr std::uint16_t kRightWheelId = 0x05;  ///< 右轮毂电机 CAN ID

constexpr std::uint16_t kDmLfMasterId = 0x02;  ///< 左前关节主电机 CAN ID
constexpr std::uint16_t kDmLfSlaveId = 0x01;   ///< 左前关节从电机 CAN ID
constexpr std::uint16_t kDmLbMasterId = 0x04;  ///< 左后关节主电机 CAN ID
constexpr std::uint16_t kDmLbSlaveId = 0x03;   ///< 左后关节从电机 CAN ID
constexpr std::uint16_t kDmRfMasterId = 0x06;  ///< 右前关节主电机 CAN ID
constexpr std::uint16_t kDmRfSlaveId = 0x05;   ///< 右前关节从电机 CAN ID
constexpr std::uint16_t kDmRbMasterId = 0x08;  ///< 右后关节主电机 CAN ID
constexpr std::uint16_t kDmRbSlaveId = 0x07;   ///< 右后关节从电机 CAN ID

constexpr std::uint16_t kFricLeftId = 0x07;   ///< 左摩擦轮电机 CAN ID
constexpr std::uint16_t kFricRightId = 0x08;  ///< 右摩擦轮电机 CAN ID
constexpr std::uint16_t kDialId = 0x07;       ///< 拨盘电机 CAN ID

const DmMitSettings kDmLfSettings{kDmLfMasterId, kDmLfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmLbSettings{kDmLbMasterId, kDmLbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmRfSettings{kDmRfMasterId, kDmRfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmRbSettings{kDmRbMasterId, kDmRbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
}  // namespace globals

// ── 云台 ──
namespace gimbal {
using namespace common::gimbal;
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

const DmMitSettings kPitchMotorSettings{0x12, 0x11, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
const DmMitSettings kYawMotorSettings{0x13, 0x03, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};

constexpr float kPitchMinRad = -0.3f;  ///< 俯仰角下限 [rad]
constexpr float kPitchMaxRad = 0.3f;   ///< 俯仰角上限 [rad]

constexpr PidGains kYawPositionPid{25.0f, 0.0f, 0.05f, 10.0f, 1.0f};   ///< 偏航位置 PID
constexpr PidGains kYawSpeedPid{0.6f, 0.0f, 0.0f, 6.0f, 0.4f};         ///< 偏航速度 PID
constexpr PidGains kPitchPositionPid{26.0f, 0.0f, 0.1f, 10.0f, 0.4f};  ///< 俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 8.0f, 0.0f};      ///< 俯仰速度 PID
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
constexpr float kStairClimbPhase0LegLengthM = 0.15f;               ///< 上台阶 Phase 0 转腿目标腿长 [m]
constexpr float kStairClimbThetaTargetRad = 0.6f;                  ///< 上台阶目标腿摆角 [rad]
constexpr std::uint32_t kStairClimbDurationMs = 200U;              ///< 上台阶最长持续时间 [ms]
constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.02f;  ///< 腿长到位容差 [m]
constexpr float kStairClimbThetaNearZeroThresholdRad = 0.1f;       ///< 摆角归零判定阈值 [rad]
constexpr std::uint32_t kStairClimbPitchStableMs = 450U;           ///< 上台阶完成后俯仰稳定等待时间 [ms]

// ==== 倒地自启 ====
constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;        ///< 倒地确认时间 [ms]
constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;  ///< 自启超时 [ms]

// ==== 跳跃 ====
constexpr std::uint32_t kJumpLowPrepMs = 250U;          ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpLowPushMaxMs = 1000U;      ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpLowRecoverMs = 250U;       ///< 跳跃回收阶段持续时间 [ms]
constexpr float kJumpLowPrepLegLengthM = 0.13f;         ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpLowPushLegLengthM = 0.25f;         ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpLowRecoverLegLengthM = 0.20f;      ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpLowPushReachedLegLengthM = 0.25f;  ///< 蹬伸到位判定腿长 [m]

// ==== 跳跃（中腿长）====
constexpr std::uint32_t kJumpMidPrepMs = 250U;          ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpMidPushMaxMs = 1000U;      ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpMidRecoverMs = 250U;       ///< 跳跃回收阶段持续时间 [ms]
constexpr float kJumpMidPrepLegLengthM = 0.13f;         ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpMidPushLegLengthM = 0.22f;         ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpMidRecoverLegLengthM = 0.20f;      ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpMidPushReachedLegLengthM = 0.22f;  ///< 蹬伸到位判定腿长 [m]

// ==== 基本运动（腿长档位）====
constexpr float kLowLegLengthM = 0.15f;      ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.22f;      ///< 中腿长档位目标腿长 [m]
constexpr float kHighLegLengthM = 0.35f;     ///< 高腿长档位目标腿长 [m]
constexpr float kLegLengthRampTimeS = 0.5f;  ///< 腿长切换斜坡时间 [s]
}  // namespace chassis_fsm

// ── 底盘控制 ──
namespace chassis {

// ==== 物理/机械参数 ====
constexpr float kControlDtS = 0.002f;  ///< 底盘控制周期 [s]（500Hz）
constexpr float kLegL1M = 0.215f;      ///< 五连杆主动杆长度 [m]
constexpr float kLegL2M = 0.254f;      ///< 五连杆从动杆长度 [m]

// -- 弹簧模型参数 --
constexpr float kSpringModelA = 1082.0f;      ///< 弹簧模型系数 A
constexpr float kSpringModelB = 1070.0f;      ///< 弹簧模型系数 B
constexpr float kSpringModelC = 404.0f;       ///< 弹簧模型系数 C
constexpr float kSpringModelD = 177.0f;       ///< 弹簧模型系数 D
constexpr float kSpringPhaseDivisor = 18.0f;  ///< 弹簧相位除数

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
constexpr float kStandupThetaThresholdRad = 0.85f;   ///< 起立完成判定 [rad]
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

// ==== 物理参数（变体专属）====
constexpr float kBodyMassKg = 18.0f;         ///< 机体质量 [kg]
constexpr float kSpringTorqueScale = 70.0f;  ///< 弹簧力矩缩放系数

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵）====
constexpr std::array<float, 240> kCtrlP{
    -5.1972,  -41.847,  34.355,   71.415,  -33.813,  -28.187, -7.5685,  -43.368,  43.483,   78.17,    -49.413, -34.205,
    -0.44283, 2.4167,   -0.72297, -2.4328, -0.14004, 1.3175,  -1.4258,  8.0329,   -2.6284,  -7.8117,  -0.7592, 4.8083,
    -14.048,  -80.106,  16.36,    71.99,   4.4695,   -22.286, -0.91937, -7.5616,  3.507,    -4.241,   2.5456,  -4.4668,
    -3.7683,  -0.52638, -0.52139, 20.459,  -25.997,  2.7492,  -0.35989, -2.5141,  0.13179,  8.7738,   -14.877, 2.0674,
    -24.859,  46.433,   29.28,    -22.531, -29.984,  -30.021, -2.9976,  2.6628,   7.2471,   2.4205,   -7.2679, -7.5031,
    -5.1972,  34.355,   -41.847,  -28.187, -33.813,  71.415,  -7.5685,  43.483,   -43.368,  -34.205,  -49.413, 78.17,
    0.44283,  0.72297,  -2.4167,  -1.3175, 0.14004,  2.4328,  1.4258,   2.6284,   -8.0329,  -4.8083,  0.7592,  7.8117,
    -3.7683,  -0.52139, -0.52638, 2.7492,  -25.997,  20.459,  -0.35989, 0.13179,  -2.5141,  2.0674,   -14.877, 8.7738,
    -14.048,  16.36,    -80.106,  -22.286, 4.4695,   71.99,   -0.91937, 3.507,    -7.5616,  -4.4668,  2.5456,  -4.241,
    -24.859,  29.28,    46.433,   -30.021, -29.984,  -22.531, -2.9976,  7.2471,   2.6628,   -7.5031,  -7.2679, 2.4205,
    8.313,    -14.919,  -5.7796,  -27.963, 42.335,   2.5885,  10.668,   -14.925,  -16.633,  -37.039,  60.489,  10.556,
    -0.46902, -2.2014,  -0.62344, 3.9533,  -0.67703, 1.2261,  -1.5013,  -7.5091,  -1.9429,  13.411,   -2.6176, 3.9271,
    37.049,   -64.899,  4.7213,   49.267,  33.512,   -15.402, 2.0352,   -0.43327, -1.1426,  -3.5327,  8.4175,  -0.3284,
    -2.5248,  -19.633,  0.88644,  19.212,  -19.583,  6.8038,  -0.12853, -0.62553, 3.1208,   -3.0356,  -2.6624, -0.77665,
    -40.184,  -145.01,  49.398,   185.7,   22.646,   -73.536, -1.9905,  -14.811,  3.8744,   15.046,   7.3029,  -6.8706,
    8.313,    -5.7796,  -14.919,  2.5885,  42.335,   -27.963, 10.668,   -16.633,  -14.925,  10.556,   60.489,  -37.039,
    0.46902,  0.62344,  2.2014,   -1.2261, 0.67703,  -3.9533, 1.5013,   1.9429,   7.5091,   -3.9271,  2.6176,  -13.411,
    -2.5248,  0.88644,  -19.633,  6.8038,  -19.583,  19.212,  -0.12853, 3.1208,   -0.62553, -0.77665, -2.6624, -3.0356,
    37.049,   4.7213,   -64.899,  -15.402, 33.512,   49.267,  2.0352,   -1.1426,  -0.43327, -0.3284,  8.4175,  -3.5327,
    -40.184,  49.398,   -145.01,  -73.536, 22.646,   185.7,   -1.9905,  3.8744,   -14.811,  -6.8706,  7.3029,  15.046,
};

// ==== 基本运动（PID 增益）====
constexpr PidGains kLeftL0Pid{8000.0f, 0.15f, 50000.0f, 170.0f, 30.0f};   ///< 左腿腿长 PID（常规）
constexpr PidGains kRightL0Pid{8000.0f, 0.15f, 50000.0f, 170.0f, 30.0f};  ///< 右腿腿长 PID（常规）
constexpr PidGains kRollPid{800.0f, 0.0f, 80.0f, 80.0f, 0.0f};            ///< 横滚平衡 PID

// ==== 跳跃（PID 增益）====
constexpr PidGains kLeftL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};       ///< 左腿蹬伸 PID
constexpr PidGains kRightL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};      ///< 右腿蹬伸 PID
constexpr PidGains kLeftL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};   ///< 左腿回收 PID
constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};  ///< 右腿回收 PID

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{20.0f, 0.0f, 0.0f, 15.0f, 0.0f};   ///< 左腿摆角速度 PID
constexpr PidGains kRightLegTurnPid{20.0f, 0.0f, 0.0f, 15.0f, 0.0f};  ///< 右腿摆角速度 PID

// ==== 上台阶（腿摆角 PID）====
constexpr PidGains kStairClimbThetaPid{40.0f, 0.0f, 8.0f, 60.0f,
                                       30.0f};  ///< 上台阶腿摆角 PID（位置环，跟踪 kStairClimbThetaTargetRad）
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

constexpr float kControlLoopDtS = 0.002f;  ///< 控制环周期 [s]

// -- 摇杆/键鼠输入归一化 --
constexpr std::int16_t kDr16AxisMaxAbs = 660;     ///< DR16 摇杆轴最大绝对值
constexpr float kRcStickMax = 660.0f;             ///< RC 摇杆最大值
constexpr float kTcMouseMax = 200.0f;             ///< 图传鼠标增量最大值
constexpr float kRcYawRateMaxRadS = -2.5f;        ///< RC 摇杆满偏时偏航积分速率 [rad/s]
constexpr float kRcPitchRateMaxRadS = 1.5f;       ///< RC 摇杆满偏时俯仰积分速率 [rad/s]
constexpr float kTcMouseYawRateMaxRadS = -2.0f;   ///< 图传鼠标满偏时偏航积分速率 [rad/s]
constexpr float kTcMousePitchRateMaxRadS = 1.0f;  ///< 图传鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kPitchTargetMinRad = -0.35f;      ///< RC 积分俯仰目标下限 [rad]
constexpr float kPitchTargetMaxRad = 0.25f;       ///< RC 积分俯仰目标上限 [rad]

// -- 云台启动归中判稳 --
constexpr float kGimbalStartupYawAlignErrorRad = 0.04f;           ///< 归中完成位置误差阈值 [rad]
constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;            ///< 归中完成速度阈值 [rad/s]
constexpr std::uint32_t kGimbalStartupYawAlignStableTicks = 50U;  ///< 归中判稳所需连续满足周期数

// -- 偏航就绪判稳 --
constexpr float kYawFollowDriveReadyErrorRad = 0.04f;           ///< 偏航就绪位置误差阈值 [rad]
constexpr float kYawFollowDriveReadyVelRadS = 0.25f;            ///< 偏航就绪速度阈值 [rad/s]
constexpr std::uint32_t kYawFollowDriveReadyStableTicks = 50U;  ///< 偏航就绪判稳所需连续周期数

// ==== 基本运动 ====
constexpr float kTargetForwardSpeedMaxMps = 2.f;                 ///< 最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 0.75f;        ///< 高腿长模式最大前进速度 [m/s]
constexpr float kVxInputDeadbandNorm = 0.1f;                     ///< 前进输入死区
constexpr float kVyInputDeadbandNorm = 0.1f;                     ///< 平移输入死区
constexpr float kYawFollowRampStepRadS = 0.05f;                  ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.4f;         ///< 位置锚定冻结速度阈值 [m/s]
constexpr float kYawFollowFixedTargetRad = -1.72f;               ///< 偏航跟随固定目标偏置角 [rad]
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;            ///< 偏航跟随侧向目标偏置角 [rad]
constexpr PidGains kYawFollowPid{8.2f, 0.0f, 1.2f, 6.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置 ====
constexpr float kExpectedThetaLlBiasRad = 0.11f;  ///< 期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRad = 0.11f;  ///< 期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = 0.f;     ///< 期望机体俯仰偏置 [rad]

// ==== 速度斜坡参数 ====
constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.008f};    ///< 低腿长速度斜坡
constexpr SdotRampParams kSdotRampMidLeg{0.006f, 0.003f};   ///< 中腿长速度斜坡
constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};  ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.05f;  ///< 小陀螺偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS = 7.0f;  ///< 小陀螺目标自旋角速度 [rad/s]
constexpr float kSpinTranslationGain = 1.2f;   ///< 小陀螺平移增益
constexpr float kSpinThetaLlBiasRad = 0.01f;   ///< 小陀螺时左腿摆角偏置 [rad]
constexpr float kSpinLegLengthBiasM = 0.0f;  ///< 小陀螺时腿长偏差（左+右-）[m]
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
constexpr float kLeftPhi1OffsetRad = -1.50f + M_PI;   ///< 左腿前关节零位偏移 [rad]
constexpr float kLeftPhi4OffsetRad = -1.50f;          ///< 左腿后关节零位偏移 [rad]
constexpr float kRightPhi1OffsetRad = -1.42f + M_PI;  ///< 右腿前关节零位偏移 [rad]
constexpr float kRightPhi4OffsetRad = -1.62f;         ///< 右腿后关节零位偏移 [rad]
}  // namespace state_estimator

// ── 腿部运动学 ──
namespace leg_kinematics {
constexpr float kDefaultDtS = 0.002f;  ///< 运动学默认解算周期 [s]
constexpr float kMinSin = 1e-5f;       ///< 正弦值下限（避免除零奇异）
constexpr float kMinLen = 1e-5f;       ///< 长度下限（避免除零奇异）
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
