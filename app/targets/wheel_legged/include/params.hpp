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
  float retract_theta_tolerance_rad;
  float hook_theta_tolerance_rad;
  float leg_length_tolerance_m;
  float settle_theta_tolerance_rad;
  float settle_theta_target_rad;
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

constexpr float kStairDescendLegLengthM = 0.16f;
constexpr float kStairDescendThetaBTriggerRad = 0.18f;

constexpr StairClimbParams kStairClimb{
    .high_leg_length_m = 0.30f,
    .hook_leg_length_m = 0.13f,
    .retract_leg_length_m = 0.13f,
    .settle_leg_length_m = 0.13f,
    .contact_theta_threshold_rad = 0.40f,
    .hook_theta_target_rad = 1.35f,
    .retract_theta_target_rad = 1.3f,
    .retract_theta_tolerance_rad = 0.2f,
    .hook_theta_tolerance_rad = 0.20f,
    .leg_length_tolerance_m = 0.01f,
    .settle_theta_tolerance_rad = 0.3f,
    .settle_theta_target_rad = 0.f,
    .hook_stable_ms = 180U,
    .retract_stable_ms = 180U,
    .settle_stable_ms = 1000U,
    .hook_timeout_ms = 1200U,
    .retract_timeout_ms = 1200U,
    .settle_timeout_ms = 2000U,
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
constexpr float kTargetForwardSpeedMaxNoScMps = 1.1f;     ///< 无超电最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.3f;  ///< 高腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxMidLegMps = 1.0f;   ///< F键中腿长模式最大前进速度 [m/s]
constexpr float kTargetSpeedBiasLowLegMps = 0.0f;         ///< 低腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegMps = 0.0f;         ///< C键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegFMps = 0.0f;        ///< F键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasHighLegMps = 0.0f;        ///< 高腿长目标速度偏置 [m/s]
constexpr float kVxInputDeadbandNorm = 0.1f;              ///< 前进输入死区（归一化值，低于此忽略）
constexpr float kVyInputDeadbandNorm = 0.1f;              ///< 平移输入死区（归一化值）
constexpr float kYawFollowRampStepRadS = 0.1f;            ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kYawFollowRampStepRadNoScS = 0.08f;  ///< 偏航跟随角速度斜坡步长（无超电）[(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.15f;  ///< 位置锚定冻结速度阈值 [m/s]（车速低于此值时锁定位置）
constexpr uint32_t kPositionHoldTimeoutTicks =
    1000U;  ///< 位置锚定超时 [ticks]（斜坡归零后最多等待此周期数，超时强制冻结）

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
constexpr float kExpectedThetaLlBiasRadLowLeg = -0.13f;    ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = -0.13;     ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLeg = -0.11f;    ///< 中腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLeg = -0.11f;    ///< 中腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadHighLeg = -0.08f;   ///< 高腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadHighLeg = -0.08f;   ///< 高腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = -0.168f;          ///< 期望机体俯仰偏置 [rad]
constexpr float kExpectedDisplacementBiasMLowLeg = 0.15f;  ///< 低腿长期望位移偏置 [m]
constexpr float kExpectedDisplacementBiasMMidLeg = 0.15f;  ///< 中腿长期望位移偏置 [m]

// ==== 速度斜坡参数（按腿长档位分级）====
constexpr SdotRampParams kSdotRampLowLeg{0.007f, 0.007f};   ///< 低腿长速度斜坡（加速/制动步长）
constexpr SdotRampParams kSdotRampMidLeg{0.005f, 0.005f};   ///< 中腿长速度斜坡(C 键触发)
constexpr SdotRampParams kSdotRampMidLegF{0.006f, 0.006f};  ///< 中腿长速度斜坡（F 键触发）
constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};  ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.1f;            ///< 小陀螺进入偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinExitYawRampStepRadS = 0.15f;       ///< 小陀螺退出偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS1 = 7.5f;          ///< 小陀螺目标自旋角速度 [rad/s] ≤55W
constexpr float kSpinTargetYawDotRadS2 = 7.5f;          ///< 小陀螺目标自旋角速度 [rad/s] 55-65W
constexpr float kSpinTargetYawDotRadS3 = 7.5f;          ///< 小陀螺目标自旋角速度 [rad/s] 65-75W
constexpr float kSpinTargetYawDotRadS4 = 7.5f;          ///< 小陀螺目标自旋角速度 [rad/s] >75W
constexpr float kSpinTargetYawDotRadNoScS1 = 6.0f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] ≤55W
constexpr float kSpinTargetYawDotRadNoScS2 = 6.0f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] 55-65W
constexpr float kSpinTargetYawDotRadNoScS3 = 6.0f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] 65-75W
constexpr float kSpinTargetYawDotRadNoScS4 = 6.0f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] >75W
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
constexpr PidGains kYawPositionPid{80.0f, 0.2, 3.f, 1000.0f, 4.0f};  ///< 自瞄偏航位置 PID（打装甲板）
constexpr PidGains kYawSpeedPid{0.7f, 0.0f, 0.0f, 10.0f, 0.3f};      ///< 自瞄偏航速度 PID（打装甲板）
constexpr PidGains kPitchPositionPid{80, 0.2f, 3.0f, 1000.0f, 4.f};  ///< 自瞄俯仰位置 PID（打装甲板）
constexpr PidGains kPitchSpeedPid{0.7f, 0.0f, 0.0f, 9.0f, 0.4f};     ///< 自瞄俯仰速度 PID（打装甲板）

constexpr PidGains kYawPositionPidRune{80.0f, 0.2, 3.f, 1000.0f, 4.0f};  ///< 自瞄偏航位置 PID（打符）
constexpr PidGains kYawSpeedPidRune{0.7f, 0.0f, 0.0f, 10.0f, 0.3f};      ///< 自瞄偏航速度 PID（打符）
constexpr PidGains kPitchPositionPidRune{80, 0.2f, 3.0f, 1000.0f, 4.f};  ///< 自瞄俯仰位置 PID（打符）
constexpr PidGains kPitchSpeedPidRune{0.7f, 0.0f, 0.0f, 9.0f, 0.4f};     ///< 自瞄俯仰速度 PID（打符）
}  // namespace aimbot

// ── 自瞄 + 小陀螺模式 PID ──
namespace aimbot_spin {
constexpr PidGains kYawPositionPid{80.0f, 0.2, 3.f, 1000.0f, 4.0f};  ///< 自瞄+小陀螺偏航位置 PID
constexpr PidGains kYawSpeedPid{0.7f, 0.0f, 0.0f, 10.0f, 0.3f};      ///< 自瞄+小陀螺偏航速度 PID
constexpr PidGains kPitchPositionPid{80, 0.2f, 3.0f, 1000.0f, 4.f};  ///< 自瞄+小陀螺俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.7f, 0.0f, 0.0f, 9.0f, 0.4f};     ///< 自瞄+小陀螺俯仰速度 PID

// ── 自旋偏航目标偏置（补偿小陀螺自旋时的角度滞后）──
constexpr float kYawTargetBiasSpeedThresholds[3] = {8.0f, 9.5f, 10.5f};  ///< 四档偏置的自旋速度分界 [rad/s]
constexpr float kYawTargetBiasRad[4] = {0.02f, 0.02f, 0.02f, 0.02f};     ///< 各档位偏航目标偏置 [rad]
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
constexpr float kFricSpeedTargetRpm = 6200.0f;                           ///< 摩擦轮目标转速 [rpm]
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

constexpr float kStairDescendLegLengthM = 0.16f;
constexpr float kStairDescendThetaBTriggerRad = 0.18f;

constexpr StairClimbParams kStairClimb{
    .high_leg_length_m = 0.33f,
    .hook_leg_length_m = 0.33f,
    .retract_leg_length_m = 0.1f,
    .settle_leg_length_m = 0.1f,
    .contact_theta_threshold_rad = 0.50f,
    .hook_theta_target_rad = 1.f,
    .retract_theta_target_rad = 0.f,
    .retract_theta_tolerance_rad = 0.4f,
    .hook_theta_tolerance_rad = 0.3f,
    .leg_length_tolerance_m = 0.05f,
    .settle_theta_tolerance_rad = 0.4f,
    .settle_theta_target_rad = 0.f,
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
constexpr float kJumpAutoPrepLegLengthM = 0.14f;         ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpAutoPushLegLengthM = 0.25f;         ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpAutoRecoverLegLengthM = 0.18f;      ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpAutoPushReachedLegLengthM = 0.26f;  ///< 蹬伸到位判定腿长 [m]

// ==== 基本运动（腿长档位）====
constexpr float kLowLegLengthM = 0.14f;              ///< 低腿长档位目标腿长 [m]
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
    -4.2745,  -36.307,  30.236,   64.668,  -32.448,  -24.832, -8.4163,  -54.93,  52.719,   101.54,   -61.798, -42.144,
    -0.53202, 3.0879,   -1.0567,  -2.6664, -0.87922, 2.0081,  -1.2206,  7.511,   -2.9488,  -5.9996,  -2.7354, 5.611,
    -17.925,  -92.992,  15.885,   84.223,  10.854,   -23.076, -0.88381, -9.4265, 4.2971,   -5.1429,  3.3956,  -5.6829,
    -3.3272,  -9.5433,  6.4699,   47.763,  -43.236,  -14.653, -0.31828, -4.1484, 0.47082,  13.067,   -19.693, 1.7763,
    -28.023,  48.647,   34.078,   -21.634, -30.297,  -36.396, -3.3213,  2.2628,  8.4448,   3.6266,   -7.9434, -8.8851,
    -4.2745,  30.236,   -36.307,  -24.832, -32.448,  64.668,  -8.4163,  52.719,  -54.93,   -42.144,  -61.798, 101.54,
    0.53202,  1.0567,   -3.0879,  -2.0081, 0.87922,  2.6664,  1.2206,   2.9488,  -7.511,   -5.611,   2.7354,  5.9996,
    -3.3272,  6.4699,   -9.5433,  -14.653, -43.236,  47.763,  -0.31828, 0.47082, -4.1484,  1.7763,   -19.693, 13.067,
    -17.925,  15.885,   -92.992,  -23.076, 10.854,   84.223,  -0.88381, 4.2971,  -9.4265,  -5.6829,  3.3956,  -5.1429,
    -28.023,  34.078,   48.647,   -36.396, -30.297,  -21.634, -3.3213,  8.4448,  2.2628,   -8.8851,  -7.9434, 3.6266,
    6.5573,   -14.508,  -0.48257, -22.109, 32.362,   0.41265, 11.693,   -20.178, -10.922,  -42.249,  63.062,  8.9434,
    -0.61429, -2.716,   -0.47342, 4.9946,  -1.4383,  1.5058,  -1.4026,  -6.8863, -0.89906, 12.578,   -4.1816, 3.4221,
    46.734,   -97.03,   4.4929,   75.408,  41.206,   -16.025, 1.8518,   1.5919,  -1.2936,  -8.839,   11.046,  -0.19367,
    -3.4741,  -16.851,  13.303,   11.457,  -40.059,  15.902,  -0.17582, 0.17405, 4.1903,   -5.0725,  -5.9192, 2.2057,
    -42.863,  -157.5,   50.882,   201.57,  26.133,   -79.23,  -1.9702,  -15.656, 3.9007,   15.606,   7.8948,  -7.2045,
    6.5573,   -0.48257, -14.508,  0.41265, 32.362,   -22.109, 11.693,   -10.922, -20.178,  8.9434,   63.062,  -42.249,
    0.61429,  0.47342,  2.716,    -1.5058, 1.4383,   -4.9946, 1.4026,   0.89906, 6.8863,   -3.4221,  4.1816,  -12.578,
    -3.4741,  13.303,   -16.851,  15.902,  -40.059,  11.457,  -0.17582, 4.1903,  0.17405,  2.2057,   -5.9192, -5.0725,
    46.734,   4.4929,   -97.03,   -16.025, 41.206,   75.408,  1.8518,   -1.2936, 1.5919,   -0.19367, 11.046,  -8.839,
    -42.863,  50.882,   -157.5,   -79.23,  26.133,   201.57,  -1.9702,  3.9007,  -15.656,  -7.2045,  7.8948,  15.606,
};
constexpr std::array<float, 240> kCtrlPMid{
    -2.2897,  -20.339, 16.638,   34.644,  -19.083,  -11.377, -5.8257,  -40.693, 37.633,   71.809,   -46.222, -25.271,
    -0.4408,  2.3564,  -0.99181, -1.7705, -0.86779, 1.8985,  -0.91642, 5.2685,  -2.5338,  -3.5446,  -2.5238, 4.8826,
    -13.43,   -89.152, 17.785,   85.477,  8.081,    -23.792, -1.0531,  -7.9072, 2.9785,   -2.7344,  2.8438,  -3.5474,
    -3.3954,  -4.4195, 0.23323,  29.287,  -37.152,  7.9743,  -0.32381, -2.4009, 0.41348,  8.3926,   -15.368, 2.412,
    -23.553,  33.833,  30.483,   -3.5721, -28.731,  -31.569, -2.7729,  1.6889,  6.942,    3.2576,   -6.6778, -6.9863,
    -2.2897,  16.638,  -20.339,  -11.377, -19.083,  34.644,  -5.8257,  37.633,  -40.693,  -25.271,  -46.222, 71.809,
    0.4408,   0.99181, -2.3564,  -1.8985, 0.86779,  1.7705,  0.91642,  2.5338,  -5.2685,  -4.8826,  2.5238,  3.5446,
    -3.3954,  0.23323, -4.4195,  7.9743,  -37.152,  29.287,  -0.32381, 0.41348, -2.4009,  2.412,    -15.368, 8.3926,
    -13.43,   17.785,  -89.152,  -23.792, 8.081,    85.477,  -1.0531,  2.9785,  -7.9072,  -3.5474,  2.8438,  -2.7344,
    -23.553,  30.483,  33.833,   -31.569, -28.731,  -3.5721, -2.7729,  6.942,   1.6889,   -6.9863,  -6.6778, 3.2576,
    5.0888,   -5.237,  -5.1063,  -26.427, 29.617,   2.7756,  12.003,   -9.5064, -19.704,  -60.951,  73.788,  12.439,
    -0.50003, -3.0708, -0.4497,  5.2846,  -1.5031,  1.3403,  -1.032,   -7.1561, -0.61969, 12.199,   -3.951,  2.4898,
    52.476,   -87.267, 3.5281,   40.029,  50.133,   -17.328, 3.5675,   -2.5779, -1.5728,  -3.729,   10.415,  -0.19357,
    -1.5928,  -21.6,   1.6202,   13.139,  -24.193,  20.243,  -0.1699,  0.46458, 3.4603,   -6.4239,  -1.9128, 0.67458,
    -36.203,  -181.09, 56.754,   224.46,  30.983,   -86.901, -1.653,   -16.223, 3.6618,   15.252,   9.1232,  -7.2868,
    5.0888,   -5.1063, -5.237,   2.7756,  29.617,   -26.427, 12.003,   -19.704, -9.5064,  12.439,   73.788,  -60.951,
    0.50003,  0.4497,  3.0708,   -1.3403, 1.5031,   -5.2846, 1.032,    0.61969, 7.1561,   -2.4898,  3.951,   -12.199,
    -1.5928,  1.6202,  -21.6,    20.243,  -24.193,  13.139,  -0.1699,  3.4603,  0.46458,  0.67458,  -1.9128, -6.4239,
    52.476,   3.5281,  -87.267,  -17.328, 50.133,   40.029,  3.5675,   -1.5728, -2.5779,  -0.19357, 10.415,  -3.729,
    -36.203,  56.754,  -181.09,  -86.901, 30.983,   224.46,  -1.653,   3.6618,  -16.223,  -7.2868,  9.1232,  15.252,
};
constexpr std::array<float, 240> kCtrlPHigh{
    -2.8767,  -25.363,    19.056,   38.418,   -16.834,  -14.207,  -4.2004,  -23.459,  23.052,     39.009,   -25.567,
    -16.161,  -0.36801,   1.6269,   -0.71459, -1.5042,  0.020494, 1.1432,   -1.1876,  5.4129,     -2.5212,  -4.8225,
    -0.17299, 4.091,      -7.8659,  -65.294,  13.886,   61.793,   3.1286,   -18.347,  -0.5964,    -4.7277,  2.1223,
    -2.5704,  1.802,      -2.5687,  -3.319,   3.6862,   -7.5445,  4.3481,   -6.0666,  11.866,     -0.31847, -0.90738,
    -0.15556, 3.9005,     -7.4459,  1.5918,   -19.382,  22.368,   29.271,   12.319,   -31.237,    -26.886,  -2.3221,
    1.5829,   5.6242,     2.8696,   -5.8072,  -5.4227,  -2.8767,  19.056,   -25.363,  -14.207,    -16.834,  38.418,
    -4.2004,  23.052,     -23.459,  -16.161,  -25.567,  39.009,   0.36801,  0.71459,  -1.6269,    -1.1432,  -0.020494,
    1.5042,   1.1876,     2.5212,   -5.4129,  -4.091,   0.17299,  4.8225,   -3.319,   -7.5445,    3.6862,   11.866,
    -6.0666,  4.3481,     -0.31847, -0.15556, -0.90738, 1.5918,   -7.4459,  3.9005,   -7.8659,    13.886,   -65.294,
    -18.347,  3.1286,     61.793,   -0.5964,  2.1223,   -4.7277,  -2.5687,  1.802,    -2.5704,    -19.382,  29.271,
    22.368,   -26.886,    -31.237,  12.319,   -2.3221,  5.6242,   1.5829,   -5.4227,  -5.8072,    2.8696,   9.9778,
    2.2853,   -23.563,    -58.158,  54.859,   14.57,    12.532,   0.61558,  -36.628,  -63.778,    77.274,   22.086,
    -0.34516, -3.1287,    -0.79313, 5.3015,   -1.3867,  1.3198,   -1.1094,  -10.564,  -2.3604,    17.767,   -4.8228,
    3.9317,   44.802,     -49.661,  3.9539,   6.2635,   45.185,   -17.943,  2.5584,   0.67204,    -2.003,   -3.2923,
    6.8263,   0.36406,    -0.75193, -33.349,  -9.1706,  36.305,   -17.306,  -0.50415, -0.0041152, -0.94611, 2.5751,
    -3.7672,  1.149,      -5.0554,  -33.343,  -191.46,  53.443,   214.77,   45.633,   -77.031,    -1.2941,  -17.29,
    2.613,    14.806,     10.813,   -5.5148,  9.9778,   -23.563,  2.2853,   14.57,    54.859,     -58.158,  12.532,
    -36.628,  0.61558,    22.086,   77.274,   -63.778,  0.34516,  0.79313,  3.1287,   -1.3198,    1.3867,   -5.3015,
    1.1094,   2.3604,     10.564,   -3.9317,  4.8228,   -17.767,  -0.75193, -9.1706,  -33.349,    -0.50415, -17.306,
    36.305,   -0.0041152, 2.5751,   -0.94611, -5.0554,  1.149,    -3.7672,  44.802,   3.9539,     -49.661,  -17.943,
    45.185,   6.2635,     2.5584,   -2.003,   0.67204,  0.36406,  6.8263,   -3.2923,  -33.343,    53.443,   -191.46,
    -77.031,  45.633,     214.77,   -1.2941,  2.613,    -17.29,   -5.5148,  10.813,   14.806,
};
constexpr std::array<float, 240> kCtrlPSpin{
    -2.93,    -25.073, 19.21,    37.876,  -17.953,  -13.361, -4.6306,    -24.535,  24.942,   41.453,  -29.446,  -16.063,
    -0.36417, 1.6091,  -0.73733, -1.4119, -0.12468, 1.2074,  -1.1742,    5.3846,   -2.6333,  -4.5029, -0.75711, 4.3855,
    -9.1469,  -72.349, 13.285,   71.917,  4.0181,   -17.497, -0.62704,   -4.9971,  2.186,    -2.7523, 1.9239,   -2.5028,
    -3.4264,  4.3406,  -7.2778,  3.0841,  -7.1507,  11.746,  -0.33538,   -0.9117,  -0.17564, 4.1081,  -8.2924,  2.0017,
    -19.707,  22.357,  29.537,   12.871,  -31.327,  -27.45,  -2.376,     1.5917,   5.8333,   2.9463,  -6.1064,  -5.5525,
    -2.93,    19.21,   -25.073,  -13.361, -17.953,  37.876,  -4.6306,    24.942,   -24.535,  -16.063, -29.446,  41.453,
    0.36417,  0.73733, -1.6091,  -1.2074, 0.12468,  1.4119,  1.1742,     2.6333,   -5.3846,  -4.3855, 0.75711,  4.5029,
    -3.4264,  -7.2778, 4.3406,   11.746,  -7.1507,  3.0841,  -0.33538,   -0.17564, -0.9117,  2.0017,  -8.2924,  4.1081,
    -9.1469,  13.285,  -72.349,  -17.497, 4.0181,   71.917,  -0.62704,   2.186,    -4.9971,  -2.5028, 1.9239,   -2.7523,
    -19.707,  29.537,  22.357,   -27.45,  -31.327,  12.871,  -2.376,     5.8333,   1.5917,   -5.5525, -6.1064,  2.9463,
    9.7477,   3.4605,  -23.013,  -59.971, 56.122,   12.518,  13.3,       2.3508,   -39.669,  -71.589, 86.132,   21.451,
    -0.35544, -3.0588, -0.71369, 5.1058,  -1.3635,  1.2114,  -1.145,     -10.385,  -2.0165,  17.159,  -4.7207,  3.4394,
    54.77,    -75.116, 4.5109,   29.87,   42.217,   -18.744, 2.7181,     0.79299,  -2.1651,  -3.5428, 7.0765,   0.25203,
    -1.1603,  -33.268, -8.1726,  36.238,  -13.141,  -4.3518, 0.00062661, -0.76961, 2.664,    -4.6146, 2.3356,   -5.7021,
    -32.021,  -197.57, 54.346,   222.82,  45.386,   -78.404, -1.1894,    -17.473,  2.3449,   14.748,  11.462,   -5.4773,
    9.7477,   -23.013, 3.4605,   12.518,  56.122,   -59.971, 13.3,       -39.669,  2.3508,   21.451,  86.132,   -71.589,
    0.35544,  0.71369, 3.0588,   -1.2114, 1.3635,   -5.1058, 1.145,      2.0165,   10.385,   -3.4394, 4.7207,   -17.159,
    -1.1603,  -8.1726, -33.268,  -4.3518, -13.141,  36.238,  0.00062661, 2.664,    -0.76961, -5.7021, 2.3356,   -4.6146,
    54.77,    4.5109,  -75.116,  -18.744, 42.217,   29.87,   2.7181,     -2.1651,  0.79299,  0.25203, 7.0765,   -3.5428,
    -32.021,  54.346,  -197.57,  -78.404, 45.386,   222.82,  -1.1894,    2.3449,   -17.473,  -5.4773, 11.462,   14.748,
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
constexpr PidGains kLeftLegTurnPid{15.0f, 0.f, 0.0f, 18.0f, 0.0f};         ///< 左腿摆角速度 PID
constexpr PidGains kRightLegTurnPid{15.0f, 0.f, 0.0f, 18.0f, 0.0f};        ///< 右腿摆角速度 PID
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
constexpr float kTcMouseYawRateMaxRadS = -3.0f;     ///< 图传鼠标满偏时偏航积分速率 [rad/s]
constexpr float kTcMousePitchRateMaxRadS = 1.5f;    ///< 图传鼠标满偏时俯仰积分速率 [rad/s]
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
constexpr float kTargetForwardSpeedMaxNoScMps = 1.2f;     ///< 无超电最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.1f;  ///< 高腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxMidLegMps = 1.0f;   ///< F键中腿长模式最大前进速度 [m/s]
constexpr float kTargetSpeedBiasLowLegMps = 0.0f;         ///< 低腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegMps = 0.0f;         ///< C键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegFMps = 0.0f;        ///< F键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasHighLegMps = 0.0f;        ///< 高腿长目标速度偏置 [m/s]
constexpr float kMaxSafeYawRateRadS = 4.5f;               ///< 摩擦圆最大安全偏航速率 [rad/s]
constexpr float kVxInputDeadbandNorm = 0.05f;             ///< 前进输入死区
constexpr float kVyInputDeadbandNorm = 0.05f;             ///< 平移输入死区
constexpr float kYawFollowRampStepRadS = 0.11f;           ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kYawFollowRampStepRadNoScS = 0.06f;  ///< 偏航跟随角速度斜坡步长（无超电）[(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.05f;  ///< 位置锚定冻结速度阈值 [m/s]
constexpr uint32_t kPositionHoldTimeoutTicks =
    500U;  ///< 位置锚定超时 [ticks]（斜坡归零后最多等待此周期数，超时强制冻结）

// ==== 落地减速（离地→落地时通过腿摆角辅助减速）====
constexpr float kLandingDecelThetaGain = -0.2f;               ///< 落地减速腿摆角增益 [rad/(m/s)]
constexpr float kLandingDecelThetaMaxRad = 0.3f;              ///< 落地减速腿摆角最大偏置 [rad]
constexpr float kLandingDecelThetaRampStepRad = 0.008f;       ///< 落地减速腿摆角每周期斜坡步长 [rad/tick]
constexpr std::uint32_t kLandingDecelOffGroundMinMs = 40U;    ///< 离地最短持续时间（防单帧误判）[ms]
constexpr std::uint32_t kLandingDecelStableDurationMs = 50U;  ///< 落地减速稳定保持时间 [ms]

constexpr float kYawFollowFixedTargetRad = -2.269f;                ///< 偏航跟随固定目标偏置角 [rad]
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;            ///< 偏航跟随侧向目标偏置角 [rad]
constexpr PidGains kYawFollowPid{26.0f, 0.0f, 3.f, 8.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置 ====
constexpr float kExpectedThetaLlBiasRadLowLeg = 0.085f;     ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = 0.085f;     ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLeg = 0.05f;      ///< 中腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLeg = 0.05f;      ///< 中腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadHighLeg = 0.06f;     ///< 高腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadHighLeg = 0.06f;     ///< 高腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = -0.005f;           ///< 期望机体俯仰偏置 [rad]
constexpr float kExpectedDisplacementBiasMLowLeg = 0.17f;   ///< 低腿长期望位移偏置 [m]
constexpr float kExpectedDisplacementBiasMMidLeg = 0.127f;  ///< 中腿长期望位移偏置 [m]

// ==== 速度斜坡参数 ====
constexpr SdotRampParams kSdotRampLowLeg{0.0065f, 0.007f};   ///< 低腿长速度斜坡
constexpr SdotRampParams kSdotRampMidLeg{0.0035f, 0.007f};   ///< 中腿长速度斜坡(C 键触发)
constexpr SdotRampParams kSdotRampMidLegF{0.0045f, 0.006f};  ///< 中腿长速度斜坡（F 键触发）
constexpr SdotRampParams kSdotRampHighLeg{0.005f, 0.005f};   ///< 高腿长速度斜坡

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
constexpr float kSpinTranslationGain = 0.25f;  ///< 小陀螺平移增益（系数2补偿 cos² 平均衰减，使平均车速=摇杆指令值）
constexpr float kSpinThetaLlBiasRad = 0.15f;  ///< 小陀螺时左腿摆角偏置 [rad]
constexpr float kSpinThetaLrBiasRad = 0.13f;  ///< 小陀螺时右腿摆角偏置 [rad]
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
constexpr PidGains kYawPositionPid{80.0f, 0.f, 1.5f, 10.0f, 2.2f};   ///< 自瞄偏航位置 PID（打装甲板）
constexpr PidGains kYawSpeedPid{0.65f, 0.0f, 0.0f, 10.0f, 0.f};      ///< 自瞄偏航速度 PID（打装甲板）
constexpr PidGains kPitchPositionPid{40.0f, 0.f, 1.5f, 10.0f, 2.f};  ///< 自瞄俯仰位置 PID（打装甲板）
constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 10.0f, 0.f};    ///< 自瞄俯仰速度 PID（打装甲板）

constexpr PidGains kYawPositionPidRune{80.0f, 0.f, 1.5f, 10.0f, 2.2f};   ///< 自瞄偏航位置 PID（打符）
constexpr PidGains kYawSpeedPidRune{0.65f, 0.0f, 0.0f, 10.0f, 0.f};      ///< 自瞄偏航速度 PID（打符）
constexpr PidGains kPitchPositionPidRune{40.0f, 0.f, 1.5f, 10.0f, 2.f};  ///< 自瞄俯仰位置 PID（打符）
constexpr PidGains kPitchSpeedPidRune{0.55f, 0.0f, 0.0f, 10.0f, 0.f};    ///< 自瞄俯仰速度 PID（打符）
}  // namespace aimbot

// ── 自瞄 + 小陀螺模式 PID ──
namespace aimbot_spin {
constexpr PidGains kYawPositionPid{80.0f, 0.f, 2.f, 10.0f, 5.f};     ///< 自瞄+小陀螺偏航位置 PID
constexpr PidGains kYawSpeedPid{0.55f, 0.0f, 0.0f, 10.0f, 0.f};      ///< 自瞄+小陀螺偏航速度 PID
constexpr PidGains kPitchPositionPid{40.0f, 0.f, 1.5f, 10.0f, 2.f};  ///< 自瞄+小陀螺俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 10.0f, 0.f};    ///< 自瞄+小陀螺俯仰速度 PID

// ── 自旋偏航目标偏置（补偿小陀螺自旋时的角度滞后）──
constexpr float kYawTargetBiasSpeedThresholds[3] = {8.0f, 9.5f, 10.5f};  ///< 四档偏置的自旋速度分界 [rad/s]
constexpr float kYawTargetBiasRad[4] = {-0.0f, -0.0f, -0.0f, -0.0f};     ///< 各档位偏航目标偏置 [rad]
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
constexpr float kLowLegLengthM = 0.15f;              ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.245f;             ///< 中腿长档位目标腿长 [m]
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
    -3.9735,  -34.223, 28.357,   60.22,   -31.203,  -21.995, -7.8748,  -51.866, 49.54,    94.751,  -59.34,  -37.474,
    -0.50665, 2.8814,  -1.0569,  -2.3991, -0.90413, 2.0244,  -1.164,   7.023,   -2.9268,  -5.3908, -2.7858, 5.6188,
    -16.562,  -92.448, 17.376,   84.373,  10.031,   -24.409, -0.98202, -8.9729, 3.9586,   -4.6861, 3.2911,  -5.0377,
    -3.4729,  -7.9469, 4.486,    41.823,  -43.288,  -5.7443, -0.33262, -3.6187, 0.49162,  11.786,  -19.036, 2.1646,
    -26.988,  45.008,  33.29,    -16.944, -30.472,  -35.011, -3.1928,  2.0711,  8.1445,   3.6311,  -7.7589, -8.4223,
    -3.9735,  28.357,  -34.223,  -21.995, -31.203,  60.22,   -7.8748,  49.54,   -51.866,  -37.474, -59.34,  94.751,
    0.50665,  1.0569,  -2.8814,  -2.0244, 0.90413,  2.3991,  1.164,    2.9268,  -7.023,   -5.6188, 2.7858,  5.3908,
    -3.4729,  4.486,   -7.9469,  -5.7443, -43.288,  41.823,  -0.33262, 0.49162, -3.6187,  2.1646,  -19.036, 11.786,
    -16.562,  17.376,  -92.448,  -24.409, 10.031,   84.373,  -0.98202, 3.9586,  -8.9729,  -5.0377, 3.2911,  -4.6861,
    -26.988,  33.29,   45.008,   -35.011, -30.472,  -16.944, -3.1928,  8.1445,  2.0711,   -8.4223, -7.7589, 3.6311,
    6.7453,   -12.69,  -2.4097,  -26.184, 35.365,   1.2996,  12.134,   -17.568, -14.352,  -48.879, 68.871,  10.419,
    -0.58254, -2.8213, -0.46717, 5.0916,  -1.4543,  1.4515,  -1.3313,  -7.1507, -0.86209, 12.813,  -4.1839, 3.2393,
    47.712,   -92.557, 4.4917,   63.75,   44.672,   -16.984, 2.335,    0.33433, -1.4364,  -7.2658, 11.13,   -0.1861,
    -2.9008,  -18.321, 9.2612,   11.73,   -34.565,  17.726,  -0.16995, 0.27379, 3.9821,   -5.6245, -4.6181, 1.7774,
    -41.579,  -164.94, 53.099,   209.47,  27.718,   -82.318, -1.8783,  -15.882, 3.7708,   15.484,  8.4604,  -7.204,
    6.7453,   -2.4097, -12.69,   1.2996,  35.365,   -26.184, 12.134,   -14.352, -17.568,  10.419,  68.871,  -48.879,
    0.58254,  0.46717, 2.8213,   -1.4515, 1.4543,   -5.0916, 1.3313,   0.86209, 7.1507,   -3.2393, 4.1839,  -12.813,
    -2.9008,  9.2612,  -18.321,  17.726,  -34.565,  11.73,   -0.16995, 3.9821,  0.27379,  1.7774,  -4.6181, -5.6245,
    47.712,   4.4917,  -92.557,  -16.984, 44.672,   63.75,   2.335,    -1.4364, 0.33433,  -0.1861, 11.13,   -7.2658,
    -41.579,  53.099,  -164.94,  -82.318, 27.718,   209.47,  -1.8783,  3.7708,  -15.882,  -7.204,  8.4604,  15.484,
};
constexpr std::array<float, 240> kCtrlPMid{
    -1.8832,  -17.359,  14.045,   29.411,  -15.801,  -9.891,  -4.8142,  -34.92,  31.858,   61.143,   -38.304, -22.061,
    -0.38062, 2.0121,   -0.83277, -1.5547, -0.58939, 1.5725,  -0.79777, 4.5142,  -2.1355,  -3.1511,  -1.7733, 4.0595,
    -11.023,  -80.722,  15.998,   77.576,  5.9512,   -21.397, -0.89231, -6.8528, 2.6143,   -2.3927,  2.3641,  -3.1809,
    -3.1296,  -3.1509,  -0.62356, 24.022,  -29.858,  7.8808,  -0.29382, -2.0324, 0.40393,  7.1221,   -12.848, 1.9022,
    -20.449,  26.836,   27.789,   2.283,   -26.398,  -28.607, -2.5012,  1.4781,  6.2126,   3.0911,   -5.9338, -6.3125,
    -1.8832,  14.045,   -17.359,  -9.891,  -15.801,  29.411,  -4.8142,  31.858,  -34.92,   -22.061,  -38.304, 61.143,
    0.38062,  0.83277,  -2.0121,  -1.5725, 0.58939,  1.5547,  0.79777,  2.1355,  -4.5142,  -4.0595,  1.7733,  3.1511,
    -3.1296,  -0.62356, -3.1509,  7.8808,  -29.858,  24.022,  -0.29382, 0.40393, -2.0324,  1.9022,   -12.848, 7.1221,
    -11.023,  15.998,   -80.722,  -21.397, 5.9512,   77.576,  -0.89231, 2.6143,  -6.8528,  -3.1809,  2.3641,  -2.3927,
    -20.449,  27.789,   26.836,   -28.607, -26.398,  2.283,   -2.5012,  6.2126,  1.4781,   -6.3125,  -5.9338, 3.0911,
    4.7048,   -4.1796,  -5.1179,  -25.25,  27.126,   3.0101,  11.123,   -7.7613, -18.724,  -57.915,  67.688,  12.256,
    -0.41689, -2.9163,  -0.45004, 5.0228,  -1.3306,  1.2448,  -0.8627,  -6.831,  -0.68315, 11.669,   -3.5463, 2.3722,
    47.801,   -70.665,  3.5859,   20.37,   46.368,   -16.532, 3.2856,   -1.9209, -1.4294,  -4.0946,  9.6013,  -0.19993,
    -1.1945,  -22.073,  -0.38861, 14.664,  -21.226,  20.177,  -0.15167, 0.31804, 3.2074,   -5.8261,  -1.8488, 0.55657,
    -31.11,   -167.9,   51.149,   203.75,  31.401,   -78.523, -1.4706,  -15.755, 3.5549,   14.733,   8.7352,  -6.9839,
    4.7048,   -5.1179,  -4.1796,  3.0101,  27.126,   -25.25,  11.123,   -18.724, -7.7613,  12.256,   67.688,  -57.915,
    0.41689,  0.45004,  2.9163,   -1.2448, 1.3306,   -5.0228, 0.8627,   0.68315, 6.831,    -2.3722,  3.5463,  -11.669,
    -1.1945,  -0.38861, -22.073,  20.177,  -21.226,  14.664,  -0.15167, 3.2074,  0.31804,  0.55657,  -1.8488, -5.8261,
    47.801,   3.5859,   -70.665,  -16.532, 46.368,   20.37,   3.2856,   -1.4294, -1.9209,  -0.19993, 9.6013,  -4.0946,
    -31.11,   51.149,   -167.9,   -78.523, 31.401,   203.75,  -1.4706,  3.5549,  -15.755,  -6.9839,  8.7352,  14.733,
};
constexpr std::array<float, 240> kCtrlPHigh{
    -2.8767,  -25.363,    19.056,   38.418,   -16.834,  -14.207,  -4.2004,  -23.459,  23.052,     39.009,   -25.567,
    -16.161,  -0.36801,   1.6269,   -0.71459, -1.5042,  0.020494, 1.1432,   -1.1876,  5.4129,     -2.5212,  -4.8225,
    -0.17299, 4.091,      -7.8659,  -65.294,  13.886,   61.793,   3.1286,   -18.347,  -0.5964,    -4.7277,  2.1223,
    -2.5704,  1.802,      -2.5687,  -3.319,   3.6862,   -7.5445,  4.3481,   -6.0666,  11.866,     -0.31847, -0.90738,
    -0.15556, 3.9005,     -7.4459,  1.5918,   -19.382,  22.368,   29.271,   12.319,   -31.237,    -26.886,  -2.3221,
    1.5829,   5.6242,     2.8696,   -5.8072,  -5.4227,  -2.8767,  19.056,   -25.363,  -14.207,    -16.834,  38.418,
    -4.2004,  23.052,     -23.459,  -16.161,  -25.567,  39.009,   0.36801,  0.71459,  -1.6269,    -1.1432,  -0.020494,
    1.5042,   1.1876,     2.5212,   -5.4129,  -4.091,   0.17299,  4.8225,   -3.319,   -7.5445,    3.6862,   11.866,
    -6.0666,  4.3481,     -0.31847, -0.15556, -0.90738, 1.5918,   -7.4459,  3.9005,   -7.8659,    13.886,   -65.294,
    -18.347,  3.1286,     61.793,   -0.5964,  2.1223,   -4.7277,  -2.5687,  1.802,    -2.5704,    -19.382,  29.271,
    22.368,   -26.886,    -31.237,  12.319,   -2.3221,  5.6242,   1.5829,   -5.4227,  -5.8072,    2.8696,   9.9778,
    2.2853,   -23.563,    -58.158,  54.859,   14.57,    12.532,   0.61558,  -36.628,  -63.778,    77.274,   22.086,
    -0.34516, -3.1287,    -0.79313, 5.3015,   -1.3867,  1.3198,   -1.1094,  -10.564,  -2.3604,    17.767,   -4.8228,
    3.9317,   44.802,     -49.661,  3.9539,   6.2635,   45.185,   -17.943,  2.5584,   0.67204,    -2.003,   -3.2923,
    6.8263,   0.36406,    -0.75193, -33.349,  -9.1706,  36.305,   -17.306,  -0.50415, -0.0041152, -0.94611, 2.5751,
    -3.7672,  1.149,      -5.0554,  -33.343,  -191.46,  53.443,   214.77,   45.633,   -77.031,    -1.2941,  -17.29,
    2.613,    14.806,     10.813,   -5.5148,  9.9778,   -23.563,  2.2853,   14.57,    54.859,     -58.158,  12.532,
    -36.628,  0.61558,    22.086,   77.274,   -63.778,  0.34516,  0.79313,  3.1287,   -1.3198,    1.3867,   -5.3015,
    1.1094,   2.3604,     10.564,   -3.9317,  4.8228,   -17.767,  -0.75193, -9.1706,  -33.349,    -0.50415, -17.306,
    36.305,   -0.0041152, 2.5751,   -0.94611, -5.0554,  1.149,    -3.7672,  44.802,   3.9539,     -49.661,  -17.943,
    45.185,   6.2635,     2.5584,   -2.003,   0.67204,  0.36406,  6.8263,   -3.2923,  -33.343,    53.443,   -191.46,
    -77.031,  45.633,     214.77,   -1.2941,  2.613,    -17.29,   -5.5148,  10.813,   14.806,
};
constexpr std::array<float, 240> kCtrlPSpin{
    -2.93,    -25.073, 19.21,    37.876,  -17.953,  -13.361, -4.6306,    -24.535,  24.942,   41.453,  -29.446,  -16.063,
    -0.36417, 1.6091,  -0.73733, -1.4119, -0.12468, 1.2074,  -1.1742,    5.3846,   -2.6333,  -4.5029, -0.75711, 4.3855,
    -9.1469,  -72.349, 13.285,   71.917,  4.0181,   -17.497, -0.62704,   -4.9971,  2.186,    -2.7523, 1.9239,   -2.5028,
    -3.4264,  4.3406,  -7.2778,  3.0841,  -7.1507,  11.746,  -0.33538,   -0.9117,  -0.17564, 4.1081,  -8.2924,  2.0017,
    -19.707,  22.357,  29.537,   12.871,  -31.327,  -27.45,  -2.376,     1.5917,   5.8333,   2.9463,  -6.1064,  -5.5525,
    -2.93,    19.21,   -25.073,  -13.361, -17.953,  37.876,  -4.6306,    24.942,   -24.535,  -16.063, -29.446,  41.453,
    0.36417,  0.73733, -1.6091,  -1.2074, 0.12468,  1.4119,  1.1742,     2.6333,   -5.3846,  -4.3855, 0.75711,  4.5029,
    -3.4264,  -7.2778, 4.3406,   11.746,  -7.1507,  3.0841,  -0.33538,   -0.17564, -0.9117,  2.0017,  -8.2924,  4.1081,
    -9.1469,  13.285,  -72.349,  -17.497, 4.0181,   71.917,  -0.62704,   2.186,    -4.9971,  -2.5028, 1.9239,   -2.7523,
    -19.707,  29.537,  22.357,   -27.45,  -31.327,  12.871,  -2.376,     5.8333,   1.5917,   -5.5525, -6.1064,  2.9463,
    9.7477,   3.4605,  -23.013,  -59.971, 56.122,   12.518,  13.3,       2.3508,   -39.669,  -71.589, 86.132,   21.451,
    -0.35544, -3.0588, -0.71369, 5.1058,  -1.3635,  1.2114,  -1.145,     -10.385,  -2.0165,  17.159,  -4.7207,  3.4394,
    54.77,    -75.116, 4.5109,   29.87,   42.217,   -18.744, 2.7181,     0.79299,  -2.1651,  -3.5428, 7.0765,   0.25203,
    -1.1603,  -33.268, -8.1726,  36.238,  -13.141,  -4.3518, 0.00062661, -0.76961, 2.664,    -4.6146, 2.3356,   -5.7021,
    -32.021,  -197.57, 54.346,   222.82,  45.386,   -78.404, -1.1894,    -17.473,  2.3449,   14.748,  11.462,   -5.4773,
    9.7477,   -23.013, 3.4605,   12.518,  56.122,   -59.971, 13.3,       -39.669,  2.3508,   21.451,  86.132,   -71.589,
    0.35544,  0.71369, 3.0588,   -1.2114, 1.3635,   -5.1058, 1.145,      2.0165,   10.385,   -3.4394, 4.7207,   -17.159,
    -1.1603,  -8.1726, -33.268,  -4.3518, -13.141,  36.238,  0.00062661, 2.664,    -0.76961, -5.7021, 2.3356,   -4.6146,
    54.77,    4.5109,  -75.116,  -18.744, 42.217,   29.87,   2.7181,     -2.1651,  0.79299,  0.25203, 7.0765,   -3.5428,
    -32.021,  54.346,  -197.57,  -78.404, 45.386,   222.82,  -1.1894,    2.3449,   -17.473,  -5.4773, 11.462,   14.748,
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
constexpr PidGains kLeftLegTurnPid{18.0f, 2.f, 0.0f, 25.0f, 10.0f};   ///< 左腿摆角速度 PID
constexpr PidGains kRightLegTurnPid{18.0f, 2.f, 0.0f, 25.0f, 10.0f};  ///< 右腿摆角速度 PID
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
constexpr float kGimbalStartupYawAlignErrorRad = 0.04f;           ///< 归中完成位置误差阈值 [rad]
constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;            ///< 归中完成速度阈值 [rad/s]
constexpr std::uint32_t kGimbalStartupYawAlignStableTicks = 50U;  ///< 归中判稳所需连续满足周期数

// -- 偏航就绪判稳 --
constexpr float kYawFollowDriveReadyErrorRad = 0.04f;           ///< 偏航就绪位置误差阈值 [rad]
constexpr float kYawFollowDriveReadyVelRadS = 0.25f;            ///< 偏航就绪速度阈值 [rad/s]
constexpr std::uint32_t kYawFollowDriveReadyStableTicks = 50U;  ///< 偏航就绪判稳所需连续周期数

// ==== 基本运动 ====
constexpr float kTargetForwardSpeedMaxMps = 2.1f;         ///< 最大前进速度 [m/s+]
constexpr float kTargetForwardSpeedMaxNoScMps = 1.f;      ///< 无超电最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.2f;  ///< 高腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxMidLegMps = 1.0f;   ///< F键中腿长模式最大前进速度 [m/s]
constexpr float kTargetSpeedBiasLowLegMps = 0.03f;        ///< 低腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegMps = 0.f;          ///< C键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegFMps = 0.f;         ///< F键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasHighLegMps = 0.0f;        ///< 高腿长目标速度偏置 [m/s]
constexpr float kMaxSafeYawRateRadS = 4.0f;               ///< 摩擦圆最大安全偏航速率 [rad/s]
constexpr float kVxInputDeadbandNorm = 0.1f;              ///< 前进输入死区
constexpr float kVyInputDeadbandNorm = 0.1f;              ///< 平移输入死区
constexpr float kYawFollowRampStepRadS = 0.1f;            ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kYawFollowRampStepRadNoScS = 0.06f;  ///< 偏航跟随角速度斜坡步长（无超电）[(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.05f;  ///< 位置锚定冻结速度阈值 [m/s]
constexpr uint32_t kPositionHoldTimeoutTicks =
    1000U;  ///< 位置锚定超时 [ticks]（斜坡归零后最多等待此周期数，超时强制冻结）

// ==== 落地减速（离地→落地时通过腿摆角辅助减速）====
constexpr float kLandingDecelThetaGain = 0.f;               ///< 落地减速腿摆角增益 [rad/(m/s)]
constexpr float kLandingDecelThetaMaxRad = 0.3f;            ///< 落地减速腿摆角最大偏置 [rad]
constexpr float kLandingDecelThetaRampStepRad = 0.01f;      ///< 落地减速腿摆角每周期斜坡步长 [rad/tick]
constexpr std::uint32_t kLandingDecelOffGroundMinMs = 40U;  ///< 离地最短持续时间（防单帧误判）[ms]
constexpr std::uint32_t kLandingDecelStableDurationMs = 400U;  ///< 落地减速稳定保持时间 [ms]

constexpr float kYawFollowFixedTargetRad = 0.f;                 ///< 偏航跟随固定目标偏置角 [rad]
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;           ///< 偏航跟随侧向目标偏置角 [rad]
constexpr PidGains kYawFollowPid{20.f, 0.0f, 3.f, 8.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置 ====
constexpr float kExpectedThetaLlBiasRadLowLeg = 0.01f;     ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = 0.01f;     ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLeg = 0.001f;    ///< 中腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLeg = 0.001f;    ///< 中腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadHighLeg = 0.02f;    ///< 高腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadHighLeg = 0.02f;    ///< 高腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = 0.02f;            ///< 期望机体俯仰偏置 [rad]
constexpr float kExpectedDisplacementBiasMLowLeg = 0.18f;  ///< 低腿长期望位移偏置 [m]
constexpr float kExpectedDisplacementBiasMMidLeg = 0.15f;  ///< 中腿长期望位移偏置 [m]

// ==== 速度斜坡参数 ====
constexpr SdotRampParams kSdotRampLowLeg{0.0065f, 0.007f};   ///< 低腿长速度斜坡
constexpr SdotRampParams kSdotRampMidLeg{0.0035f, 0.007f};   ///< 中腿长速度斜坡(C 键触发)
constexpr SdotRampParams kSdotRampMidLegF{0.0045f, 0.006f};  ///< 中腿长速度斜坡（F 键触发）
constexpr SdotRampParams kSdotRampHighLeg{0.006f, 0.005f};   ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.05f;           ///< 小陀螺进入偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinExitYawRampStepRadS = 0.15f;       ///< 小陀螺退出偏航角速度斜坡步长 [(rad/s)/周期]
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
constexpr float kSpinThetaLlBiasRad = 0.07f;            ///< 小陀螺时左腿摆角偏置 [rad]
constexpr float kSpinThetaLrBiasRad = 0.f;              ///< 小陀螺时右腿摆角偏置 [rad]
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
constexpr float kLeftPhi1OffsetRad = 1.38f + M_PI;   ///< 左腿前关节零位偏移 [rad]
constexpr float kLeftPhi4OffsetRad = 0.86f;          ///< 左腿后关节零位偏移 [rad]
constexpr float kRightPhi1OffsetRad = 1.26f+0.53926f + M_PI;  ///< 右腿前关节零位偏移 [rad]
constexpr float kRightPhi4OffsetRad = 1.02f+3.554876f;         ///< 右腿后关节零位偏移 [rad]
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
constexpr float kBulletBoundarySpeedMps = 20.f;  ///< 区分裁判系统返回值是否正确7

constexpr PidGains kYawPositionPid{70.0f, 0.f, 1.5f, 10.0f, 2.2f};  ///< 自瞄偏航位置 PID（打装甲板）
constexpr PidGains kYawSpeedPid{0.55f, 0.0f, 0.0f, 10.0f, 0.f};     ///< 自瞄偏航速度 PID（打装甲板）
constexpr PidGains kPitchPositionPid{40.0f, 1.f, 2.f, 30.0f, 2.f};  ///< 自瞄俯仰位置 PID（打装甲板）
constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 10.0f, 0.f};   ///< 自瞄俯仰速度 PID（打装甲板）

constexpr PidGains kYawPositionPidRune{60.0f, 0.f, 1.5f, 10.0f, 2.2f};   ///< 自瞄偏航位置 PID（打符）
constexpr PidGains kYawSpeedPidRune{0.55f, 0.0f, 0.0f, 10.0f, 0.f};      ///< 自瞄偏航速度 PID（打符）
constexpr PidGains kPitchPositionPidRune{45.0f, 0.f, 1.5f, 10.0f, 2.f};  ///< 自瞄俯仰位置 PID（打符）
constexpr PidGains kPitchSpeedPidRune{0.55f, 0.0f, 0.0f, 10.0f, 0.f};    ///< 自瞄俯仰速度 PID（打符）
}  // namespace aimbot

// ── 自瞄 + 小陀螺模式 PID ──
namespace aimbot_spin {
constexpr PidGains kYawPositionPid{70.0f, 0.f, 1.5f, 10.0f, 2.2f};   ///< 自瞄+小陀螺偏航位置 PID
constexpr PidGains kYawSpeedPid{0.55f, 0.0f, 0.0f, 10.0f, 0.f};      ///< 自瞄+小陀螺偏航速度 PID
constexpr PidGains kPitchPositionPid{40.0f, 0.f, 1.5f, 10.0f, 2.f};  ///< 自瞄+小陀螺俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 10.0f, 0.f};    ///< 自瞄+小陀螺俯仰速度 PID

// ── 自旋偏航目标偏置（补偿小陀螺自旋时的角度滞后）──
constexpr float kYawTargetBiasSpeedThresholds[3] = {8.0f, 9.5f, 10.5f};  ///< 四档偏置的自旋速度分界 [rad/s]
constexpr float kYawTargetBiasRad[4] = {0.02f, 0.02f, 0.02f, 0.02f};     ///< 各档位偏航目标偏置 [rad]
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
