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

inline YawSpeedFeedforward yaw_ff{0.002f, 0.5f};

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
constexpr  float kPitchGravityCompensationNm = 0.65f;

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
inline constexpr float kBoosterZeroPointRad = 0.11327f;     ///< 拨盘零位角度 [rad]
inline constexpr float kSegmentAngleRad = kPi / 3.f;     ///< 拨盘分段角度 [rad]
inline constexpr uint16_t kInitDelayTicks = 600;         ///< 初始化延迟周期数
inline constexpr uint16_t kShootDelayTicks = 360;        ///< 发射延迟周期数
inline constexpr float kStallThresholdRad = kPi / 18.f;  ///< 堵转判定角度阈值 [rad]
inline constexpr float kStallFallbackRad = kPi / 90.f;   ///< 堵转回退角度 [rad]
inline constexpr float kFwTargetSpeedRpm = 3400.0f;      ///< 摩擦轮目标转速 [rpm]  12m/s
// inline constexpr float kFwTargetSpeedRpm = 4690.0f;                       ///< 摩擦轮目标转速 [rpm]   16m/s
inline constexpr float kFricSpeedTargetRpm = kFwTargetSpeedRpm;  ///< alias for cross-variant compatibility
inline constexpr float kFwReadySpeedThresholdRpm = kFwTargetSpeedRpm - 200.f;  ///< 摩擦轮就绪判定转速 [rpm]
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
inline constexpr float kFwRpmFilterCutoffHz = 50.0f;  ///< 摩擦轮转速低通滤波截止频率 [Hz]
inline constexpr float kFricSpeedStepRpm = 20.0f;     ///< Z/X 键每次调整摩擦轮转速步长 [rpm]
}  // namespace shoot

// ── 底盘状态机 ──
namespace chassis_fsm {

constexpr float kStairDescendLegLengthM = 0.16f;
constexpr float kStairDescendThetaBTriggerRad = 0.18f;

constexpr StairClimbParams kStairClimb{
    .high_leg_length_m = 0.30f,
    .hook_leg_length_m = 0.38f,
    .retract_leg_length_m = 0.38f,
    .settle_leg_length_m = 0.38f,
    .contact_theta_threshold_rad = 0.40f,
    .hook_theta_target_rad = 1.8f,
    .retract_theta_target_rad = 0.7f,
    .retract_theta_tolerance_rad = 0.1f,
    .hook_theta_tolerance_rad = 0.13f,
    .leg_length_tolerance_m = 0.05f,
    .settle_theta_tolerance_rad = 0.3f,
    .settle_theta_target_rad = 0.f,
    .hook_stable_ms = 600U,
    .retract_stable_ms = 220U,
    .settle_stable_ms = 500U,
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
constexpr float kMidLegLengthM = 0.195f;      ///< 中腿长档位目标腿长 [m]
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
constexpr float kMidLegDipTargetLengthM = 0.17f;  ///< 下压目标腿长 [m]
constexpr uint16_t kMidLegDipHoldTicks = 500;     ///< 下压维持时间 [ticks @ 500Hz = 1s]

// ==== 物理参数（变体专属）====
constexpr float kBodyMassKg = 27.0f;  ///< 机体质量 [kg]

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.059f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵 — 40 组多项式系数）====
/// 由 MATLAB 离线拟合得到，p(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l² + p11*l_l*l_r + p02*l_r²
/// 共 40 行，对应 4×10 增益矩阵 K 的 40 个元素（按行主序展平）
/// 每行 6 个系数：[p00, p10, p01, p20, p11, p02]
constexpr std::array<float, 240> kCtrlPLow{
-4.5232,  -28.698,  23.998,  44.362,  -19.304,  -19.53,
     -7.7991,  -34.55,  37.412,  59.439,  -39.91,  -28.047,
     -0.35548,  1.5643,  -0.51092,  -1.8843,  0.25846,  0.75938,
     -2.9827,  13.258,  -4.4421,  -15.815,  2.0048,  6.6567,
     -15.826,  -86.367,  15.442,  84.812,  -0.0378,  -20.085,
     -0.87695,  -7.0311,  3.1184,  -5.0577,  2.5023,  -3.8326,
     -4.8131,  9.2047,  -7.9003,  -2.4639,  -0.41565,  3.5892,
     -0.42336,  -1.4139,  -0.78952,  5.8743,  -11.466,  2.2193,
     -21.736,  37.649,  28.686,  -11.969,  -34.125,  -27.228,
     -2.8523,  2.6204,  6.9903,  2.2991,  -7.6358,  -6.8418,
     -4.5232,  23.998,  -28.698,  -19.53,  -19.304,  44.362,
     -7.7991,  37.412,  -34.55,  -28.047,  -39.91,  59.439,
     0.35548,  0.51092,  -1.5643,  -0.75938,  -0.25846,  1.8843,
     2.9827,  4.4421,  -13.258,  -6.6567,  -2.0048,  15.815,
     -4.8131,  -7.9003,  9.2047,  3.5892,  -0.41565,  -2.4639,
     -0.42336,  -0.78952,  -1.4139,  2.2193,  -11.466,  5.8743,
     -15.826,  15.442,  -86.367,  -20.085,  -0.0378,  84.812,
     -0.87695,  3.1184,  -7.0311,  -3.8326,  2.5023,  -5.0577,
     -21.736,  28.686,  37.649,  -27.228,  -34.125,  -11.969,
     -2.8523,  6.9903,  2.6204,  -6.8418,  -7.6358,  2.2991,
     6.1423,  0.75322,  -14.537,  -30.717,  33.31,  6.7519,
     9.3523,  0.6559,  -26.661,  -43.885,  57.734,  12.18,
     -0.30479,  -1.3649,  -0.74606,  2.5208,  -0.60107,  1.0654,
     -2.5583,  -11.631,  -6.2056,  21.373,  -5.1609,  8.8298,
     45.83,  -84.913,  10.642,  95.568,  35.198,  -24.135,
     2.0563,  -0.52334,  -0.82853,  2.7158,  4.9392,  -0.89871,
     -3.6819,  -22.199,  -9.3188,  32.79,  -23.296,  -11.029,
     -0.24232,  -0.60156,  2.6955,  -2.6405,  1.4094,  -7.0983,
     -32.159,  -134.92,  51.37,  166.42,  14.278,  -62.908,
     -1.8068,  -13.332,  3.584,  12.555,  6.571,  -5.398,
     6.1423,  -14.537,  0.75322,  6.7519,  33.31,  -30.717,
     9.3523,  -26.661,  0.6559,  12.18,  57.734,  -43.885,
     0.30479,  0.74606,  1.3649,  -1.0654,  0.60107,  -2.5208,
     2.5583,  6.2056,  11.631,  -8.8298,  5.1609,  -21.373,
     -3.6819,  -9.3188,  -22.199,  -11.029,  -23.296,  32.79,
     -0.24232,  2.6955,  -0.60156,  -7.0983,  1.4094,  -2.6405,
     45.83,  10.642,  -84.913,  -24.135,  35.198,  95.568,
     2.0563,  -0.82853,  -0.52334,  -0.89871,  4.9392,  2.7158,
     -32.159,  51.37,  -134.92,  -62.908,  14.278,  166.42,
     -1.8068,  3.584,  -13.332,  -5.398,  6.571,  12.555,
};
constexpr std::array<float, 240> kCtrlPMid{
-2.2793,  -14.902,  12.583,  23.278,  -11.485,  -9.2314,
     -5.2063,  -23.567,  25.898,  41.664,  -30.001,  -17.469,
     -0.31735,  1.4568,  -0.434,  -1.648,  0.24533,  0.67208,
     -1.7514,  8.2112,  -2.6047,  -9.0952,  1.1101,  4.1091,
     -14.604,  -78.665,  10.29,  87.385,  -1.5411,  -13.087,
     -0.77879,  -5.0839,  2.1403,  -3.3824,  1.5244,  -2.4376,
     -3.6999,  8.7723,  -5.7972,  -6.9518,  2.7678,  5.8053,
     -0.34356,  -0.71615,  -0.3966,  3.7135,  -7.823,  1.9361,
     -18.858,  34.075,  22.288,  -12.479,  -25.778,  -21.876,
     -2.4009,  2.6598,  5.3375,  1.1933,  -5.9145,  -5.2069,
     -2.2793,  12.583,  -14.902,  -9.2314,  -11.485,  23.278,
     -5.2063,  25.898,  -23.567,  -17.469,  -30.001,  41.664,
     0.31735,  0.434,  -1.4568,  -0.67208,  -0.24533,  1.648,
     1.7514,  2.6047,  -8.2112,  -4.1091,  -1.1101,  9.0952,
     -3.6999,  -5.7972,  8.7723,  5.8053,  2.7678,  -6.9518,
     -0.34356,  -0.3966,  -0.71615,  1.9361,  -7.823,  3.7135,
     -14.604,  10.29,  -78.665,  -13.087,  -1.5411,  87.385,
     -0.77879,  2.1403,  -5.0839,  -2.4376,  1.5244,  -3.3824,
     -18.858,  22.288,  34.075,  -21.876,  -25.778,  -12.479,
     -2.4009,  5.3375,  2.6598,  -5.2069,  -5.9145,  1.1933,
     3.4787,  -0.43629,  -6.7793,  -18.733,  19.841,  2.7833,
     7.1638,  -1.332,  -18.219,  -35.453,  45.969,  8.046,
     -0.32112,  -1.4949,  -0.68706,  2.6442,  -0.48306,  1.027,
     -1.7707,  -8.5207,  -3.703,  14.969,  -2.8771,  5.5251,
     49.251,  -99.754,  8.4786,  100.56,  23.037,  -18.782,
     2.0572,  -1.6761,  -0.55105,  2.3808,  4.6485,  -0.87679,
     -2.9533,  -20.812,  -7.7943,  29.43,  -10.448,  -7.9356,
     -0.21349,  -0.80465,  2.1209,  -1.6903,  0.57976,  -4.8301,
     -29.021,  -131.04,  45.801,  165.01,  15.62,  -61.421,
     -1.8431,  -13.057,  3.9917,  13.18,  5.6823,  -6.1914,
     3.4787,  -6.7793,  -0.43629,  2.7833,  19.841,  -18.733,
     7.1638,  -18.219,  -1.332,  8.046,  45.969,  -35.453,
     0.32112,  0.68706,  1.4949,  -1.027,  0.48306,  -2.6442,
     1.7707,  3.703,  8.5207,  -5.5251,  2.8771,  -14.969,
     -2.9533,  -7.7943,  -20.812,  -7.9356,  -10.448,  29.43,
     -0.21349,  2.1209,  -0.80465,  -4.8301,  0.57976,  -1.6903,
     49.251,  8.4786,  -99.754,  -18.782,  23.037,  100.56,
     2.0572,  -0.55105,  -1.6761,  -0.87679,  4.6485,  2.3808,
     -29.021,  45.801,  -131.04,  -61.421,  15.62,  165.01,
     -1.8431,  3.9917,  -13.057,  -6.1914,  5.6823,  13.18,
};
constexpr std::array<float, 240> kCtrlPHigh{
-2.467,  -17.29,  14.239,  27.24,  -12.315,  -11.352,
     -5.128,  -25.714,  26.656,  44.449,  -28.651,  -19.92,
     -0.32149,  1.4986,  -0.43832,  -1.706,  0.27999,  0.67662,
     -1.775,  8.4196,  -2.5983,  -9.422,  1.3753,  4.0681,
     -12.548,  -73.393,  11.436,  75.61,  -1.2163,  -14.867,
     -0.7373,  -5.2718,  2.2717,  -3.5533,  1.6709,  -2.7926,
     -3.5981,  6.7091,  -5.4473,  -1.4655,  0.60076,  4.5827,
     -0.32836,  -1.0025,  -0.33004,  4.2921,  -8.1594,  1.5795,
     -18.805,  32.683,  23.147,  -10.323,  -26.076,  -22.79,
     -2.4108,  2.4678,  5.4505,  1.5302,  -5.8165,  -5.4582,
     -2.467,  14.239,  -17.29,  -11.352,  -12.315,  27.24,
     -5.128,  26.656,  -25.714,  -19.92,  -28.651,  44.449,
     0.32149,  0.43832,  -1.4986,  -0.67662,  -0.27999,  1.706,
     1.775,  2.5983,  -8.4196,  -4.0681,  -1.3753,  9.422,
     -3.5981,  -5.4473,  6.7091,  4.5827,  0.60076,  -1.4655,
     -0.32836,  -0.33004,  -1.0025,  1.5795,  -8.1594,  4.2921,
     -12.548,  11.436,  -73.393,  -14.867,  -1.2163,  75.61,
     -0.7373,  2.2717,  -5.2718,  -2.7926,  1.6709,  -3.5533,
     -18.805,  23.147,  32.683,  -22.79,  -26.076,  -10.323,
     -2.4108,  5.4505,  2.4678,  -5.4582,  -5.8165,  1.5302,
     4.1652,  -1.4159,  -7.8019,  -20.364,  22.255,  4.0379,
     7.785,  -2.9103,  -18.708,  -35.249,  46.582,  9.8443,
     -0.31334,  -1.5817,  -0.68676,  2.811,  -0.48778,  1.0329,
     -1.7271,  -8.9902,  -3.7277,  15.889,  -2.9038,  5.6026,
     41.813,  -74.598,  7.9154,  70.685,  26.382,  -18.424,
     1.9543,  -0.95663,  -0.65676,  1.1432,  4.9713,  -0.6828,
     -2.5062,  -21.69,  -7.6573,  29.061,  -15.104,  -3.1611,
     -0.19114,  -0.78465,  2.3055,  -1.838,  -0.088956,  -4.276,
     -29.104,  -129.06,  44.571,  160.54,  17.355,  -60.103,
     -1.7643,  -13.255,  3.8407,  13.232,  5.8975,  -5.99,
     4.1652,  -7.8019,  -1.4159,  4.0379,  22.255,  -20.364,
     7.785,  -18.708,  -2.9103,  9.8443,  46.582,  -35.249,
     0.31334,  0.68676,  1.5817,  -1.0329,  0.48778,  -2.811,
     1.7271,  3.7277,  8.9902,  -5.6026,  2.9038,  -15.889,
     -2.5062,  -7.6573,  -21.69,  -3.1611,  -15.104,  29.061,
     -0.19114,  2.3055,  -0.78465,  -4.276,  -0.088956,  -1.838,
     41.813,  7.9154,  -74.598,  -18.424,  26.382,  70.685,
     1.9543,  -0.65676,  -0.95663,  -0.6828,  4.9713,  1.1432,
     -29.104,  44.571,  -129.06,  -60.103,  17.355,  160.54,
     -1.7643,  3.8407,  -13.255,  -5.99,  5.8975,  13.232,
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
constexpr PidGains kLeftL0Pid{2500.0f, 0.1f, 130.0f, 130.0f, 10.0f};   ///< 左腿腿长 PID（常规）
constexpr PidGains kRightL0Pid{2500.0f, 0.1f, 130.0f, 130.0f, 10.0f};  ///< 右腿腿长 PID（常规）
constexpr PidGains kLeftL0PidStandup{6000.0f, 0.1f, 160.0f, 145.0f, 10.0f};   ///< 左腿腿长 PID（起立）
constexpr PidGains kRightL0PidStandup{6000.0f, 0.1f, 160.0f, 145.0f, 10.0f};  ///< 右腿腿长 PID（起立）
constexpr PidGains kRollPid{1000.0f, 0.1f, 20.0f, 100.0f, 40.0f};    ///< 横滚平衡 PID

// ==== 跳跃（PID 增益）====
constexpr float kJumpPushForceN = 250.0f;                                    ///< 蹬伸阶段单腿基础支撑力 [N]
constexpr PidGains kLeftL0PidJumpTwo{4000.0f, 0.0f, 60.0f, 120.0f, 0.0f};    ///< 左腿蹬伸 PID（JumpPush）
constexpr PidGains kRightL0PidJumpTwo{4000.0f, 0.0f, 60.0f, 120.0f, 0.0f};   ///< 右腿蹬伸 PID（JumpPush）
constexpr PidGains kLeftL0PidJumpThree{4000.0f, 0.f, 60.0f, 120.0f, 0.0f};   ///< 左腿回收 PID（JumpRecover）
constexpr PidGains kRightL0PidJumpThree{4000.0f, 0.f, 60.0f, 120.0f, 0.0f};  ///< 右腿回收 PID（JumpRecover）

// ==== 中腿长下压（PID 增益）====
constexpr PidGains kLeftL0PidDip{500.0f, 0.f, 130.0f, 180.0f, 0.0f};   ///< 左腿下压腿长 PID
constexpr PidGains kRightL0PidDip{500.0f, 0.f, 130.0f, 180.0f, 0.0f};  ///< 右腿下压腿长 PID

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{30.0f, 0.1f, 60.0f, 27.0f, 7.0f};   ///< 左腿摆角速度 PID（倒地恢复用）
constexpr PidGains kRightLegTurnPid{30.0f, 0.1f, 60.0f, 27.0f, 7.0f};  ///< 右腿摆角速度 PID（倒地恢复用）
constexpr PidGains kLeftLegTurnPidManual{30.0f, 0.1f, 60.0f, 27.0f, 7.0f};  ///< 左腿摆角速度 PID（手动倒地恢复）
constexpr PidGains kRightLegTurnPidManual{30.0f, 0.1f, 60.0f, 27.0f, 7.0f};  ///< 右腿摆角速度 PID（手动倒地恢复）

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
constexpr float kTcMouseYawRateMaxRadS = -6.0f;     ///< 图传鼠标满偏时偏航积分速率 [rad/s]
constexpr float kTcMousePitchRateMaxRadS = 3.0f;    ///< 图传鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kDr16MouseMax = 1600.0f;            ///< DR16 鼠标增量最大值（用于积分目标速率计算）
constexpr float kDr16MouseYawRateMaxRadS = -6.0f;   ///< DR16 鼠标满偏时偏航积分速率 [rad/s]
constexpr float kDr16MousePitchRateMaxRadS = 3.0f;  ///< DR16 鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kPitchTargetMinRad = -0.35f;        ///< RC 积分俯仰目标下限 [rad]
constexpr float kPitchTargetMaxRad = 0.7f;          ///< RC 积分俯仰目标上限 [rad]
constexpr float kKeyboardAccelRampStep = 0.004f;    ///< 键盘 WASD 加速斜坡步进（每周期，0→1 约 0.5s）
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
constexpr float kMaxSafeYawRateRadS = 3.5f;               ///< 摩擦圆最大安全偏航速率 [rad/s]
constexpr float kTargetForwardSpeedMaxMps = 2.43f;        ///< 最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.2f;  ///< 高腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxMidLegMps = 1.7f;   ///< F键中腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxNoScMps = 1.1f;     ///< 无超电最大前进速度 [m/s]
constexpr float kTargetSpeedBiasLowLegMps = 0.0f;         ///< 低腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegMps = 0.11f;         ///< C键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegFMps = 0.0f;        ///< F键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasHighLegMps = 0.0f;        ///< 高腿长目标速度偏置 [m/s]
constexpr float kVxInputDeadbandNorm = 0.1f;              ///< 前进输入死区（归一化值，低于此忽略）
constexpr float kVyInputDeadbandNorm = 0.1f;              ///< 平移输入死区（归一化值）
constexpr float kYawFollowRampStepRadS = 0.3f;            ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
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

constexpr float kYawFollowFixedTargetRad = -2.076f;      ///< 偏航跟随固定目标偏置角 [rad]（前进方向）
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;  ///< 偏航跟随侧向目标偏置角 [rad]（±π/2）
constexpr PidGains kYawFollowPid{40.0f, 0.0f, 4.f, 8.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置（腿摆角/机体俯仰）====
constexpr float kExpectedThetaLlBiasRadLowLeg = -0.05f;   ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = -0.05f;    ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLegC = -0.15f;  ///< 中腿长(C键)期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLegC = -0.15f;  ///< 中腿长(C键)期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLegF = -0.06f;  ///< 中腿长(F键)期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLegF = -0.06f;  ///< 中腿长(F键)期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadHighLeg = -0.03f;  ///< 高腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadHighLeg = -0.03f;  ///< 高腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = -0.088f;         ///< 期望机体俯仰偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLeg = -0.11f;   ///< 中腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLeg = -0.11f;   ///< 中腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedDisplacementBiasM = 0.0f;       ///< 期望位移偏置 [m]

// ==== 速度斜坡参数（按腿长档位分级）====
constexpr SdotRampParams kSdotRampLowLeg{0.0055f, 0.0055f};   ///< 低腿长速度斜坡（加速/制动步长）
constexpr SdotRampParams kSdotRampMidLeg{0.0055f, 0.0055f};   ///< 中腿长速度斜坡(C 键触发)
constexpr SdotRampParams kSdotRampMidLegF{0.005f, 0.005f};  ///< 中腿长速度斜坡（G 键触发）
constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};  ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.15f;            ///< 小陀螺进入偏航角速度斜坡步长 [(rad/s)/周期]
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
constexpr float kSpinTranslationGain = 0.3f;  ///< 小陀螺平移增益（将云台系前进指令投影到车体系的比例）
constexpr float kSpinThetaLlBiasRad = 0.02f;  ///< 小陀螺时左腿摆角偏置 [rad]
constexpr float kSpinThetaLrBiasRad = -0.05f;  ///< 小陀螺时右腿摆角偏置 [rad]
constexpr float kSpinLegLengthBiasM = -0.0f;   ///< 小陀螺时腿长偏差（左+右-）[m]
constexpr float kSpinThetaBBiasRad  = -0.065f;  ///< 小陀螺时俯仰目标偏置 [rad]

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
constexpr float kLeftPhi1OffsetRad = 2.8f + M_PI + 0.002f;                       ///< 左腿前关节零位偏移 [rad]
constexpr float kLeftPhi4OffsetRad = 0.4f-0.05;                              ///< 左腿后关节零位偏移 [rad]
constexpr float kRightPhi1OffsetRad = 2.1244f + M_PI + 0.13f - 0.068f + 0.019f-0.02;  ///< 右腿前关节零位偏移 [rad]
constexpr float kRightPhi4OffsetRad = 0.46f + 0.123f + 0.136-0.11f + 0.05f;           ///< 右腿后关节零位偏移 [rad]
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
constexpr PidGains kYawPositionPidRune{55.0f, 0.1, 2.f, 8.0f, 5.0f};  ///< 自瞄偏航位置 PID（打符）
constexpr PidGains kYawSpeedPidRune{0.8f, 0.05f, 0.0f, 6.4f, 0.6f};      ///< 自瞄偏航速度 PID（打符）
constexpr PidGains kPitchPositionPidRune{48, 0.f, 1.5f, 8.0f, 4.f};  ///< 自瞄俯仰位置 PID（打符）
constexpr PidGains kPitchSpeedPidRune{0.8f, 0.0f, 0.0f, 6.4f, 0.4f};     ///< 自瞄俯仰速度 PID（打符）
constexpr PidGains kYawPositionPid{55.0f, 0.1, 2.f, 8.0f, 5.0f};  ///< 自瞄偏航位置 PID
constexpr PidGains kYawSpeedPid{0.8f, 0.05f, 0.0f, 6.4f, 0.6f};      ///< 自瞄偏航速度 PID
constexpr PidGains kPitchPositionPid{48, 0.0f, 1.5f, 8.0f, 4.f};  ///< 自瞄俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.8f, 0.0f, 0.0f, 6.4f, 0.4f};     ///< 自瞄俯仰速度 PID
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
    .hook_leg_length_m = 0.3f,
    .retract_leg_length_m = 0.1f,
    .settle_leg_length_m = 0.1f,
    .contact_theta_threshold_rad = 0.50f,
    .hook_theta_target_rad = 1.1f,
    .retract_theta_target_rad = -0.1f,
    .retract_theta_tolerance_rad = 0.4f,
    .hook_theta_tolerance_rad = 0.1f,
    .leg_length_tolerance_m = 0.05f,
    .settle_theta_tolerance_rad = 0.3f,
    .settle_theta_target_rad = 0.f,
    .hook_stable_ms = 100U,
    .retract_stable_ms = 200U,
    .settle_stable_ms = 200U,
    .hook_timeout_ms = 10000U,
    .retract_timeout_ms = 10000U,
    .settle_timeout_ms = 30000U,
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
constexpr float kExpectedDisplacementBiasM = 0.15f;      ///< 期望位移偏置 [m]

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
constexpr float kSpinThetaLlBiasRad = 0.1f;   ///< 小陀螺时左腿摆角偏置 [rad]
constexpr float kSpinThetaLrBiasRad = 0.07f;  ///< 小陀螺时右腿摆角偏置 [rad]
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
    .retract_leg_length_m = 0.15f,
    .settle_leg_length_m = 0.16f,
    .contact_theta_threshold_rad = 0.50f,
    .hook_theta_target_rad = 0.80f,
    .retract_theta_target_rad = 1.f,
    .retract_theta_tolerance_rad = 0.2f,
    .hook_theta_tolerance_rad = 0.10f,
    .leg_length_tolerance_m = 0.02f,
    .settle_theta_tolerance_rad = 0.3f,
    .settle_theta_target_rad = 0.f,
    .hook_stable_ms = 20U,
    .retract_stable_ms = 50U,
    .settle_stable_ms = 100U,
    .hook_timeout_ms = 1200U,
    .retract_timeout_ms = 1200U,
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
constexpr float kLowLegLengthM = 0.14f;              ///< 低腿长档位目标腿长 [m]
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
constexpr float kTargetForwardSpeedMaxNoScMps = 1.1f;     ///< 无超电最大前进速度 [m/s]
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
constexpr float kYawFollowRampStepRadNoScS = 0.05f;  ///< 偏航跟随角速度斜坡步长（无超电）[(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.05f;  ///< 位置锚定冻结速度阈值 [m/s]
constexpr uint32_t kPositionHoldTimeoutTicks =
    1000U;  ///< 位置锚定超时 [ticks]（斜坡归零后最多等待此周期数，超时强制冻结）

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
constexpr float kExpectedDisplacementBiasM = 0.15f;        ///< 期望位移偏置 [m]

// ==== 速度斜坡参数 ====
constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.008f};    ///< 低腿长速度斜坡
constexpr SdotRampParams kSdotRampMidLeg{0.007f, 0.007f};   ///< 中腿长速度斜坡(C 键触发)
constexpr SdotRampParams kSdotRampMidLegF{0.01f, 0.008f};   ///< 中腿长速度斜坡（F 键触发）
constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};  ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.05f;           ///< 小陀螺进入偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinExitYawRampStepRadS = 0.15f;       ///< 小陀螺退出偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS1 = 8.f;           ///< 小陀螺目标自旋角速度 [rad/s] ≤55W
constexpr float kSpinTargetYawDotRadS2 = 8.f;           ///< 小陀螺目标自旋角速度 [rad/s] 55-65W
constexpr float kSpinTargetYawDotRadS3 = 8.f;           ///< 小陀螺目标自旋角速度 [rad/s] 65-75W
constexpr float kSpinTargetYawDotRadS4 = 8.f;           ///< 小陀螺目标自旋角速度 [rad/s] >75W
constexpr float kSpinTargetYawDotRadNoScS1 = 6.5f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] ≤55W
constexpr float kSpinTargetYawDotRadNoScS2 = 6.5f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] 55-65W
constexpr float kSpinTargetYawDotRadNoScS3 = 6.5f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] 65-75W
constexpr float kSpinTargetYawDotRadNoScS4 = 6.5f;      ///< 无超电小陀螺目标自旋角速度 [rad/s] >75W
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
constexpr float kBulletBoundarySpeedMps = 20.f;  ///< 区分裁判系统返回值是否正确7

constexpr PidGains kYawPositionPid{70.0f, 0.f, 1.5f, 10.0f, 2.2f};   ///< 自瞄偏航位置 PID（打装甲板）
constexpr PidGains kYawSpeedPid{0.55f, 0.0f, 0.0f, 10.0f, 0.f};      ///< 自瞄偏航速度 PID（打装甲板）
constexpr PidGains kPitchPositionPid{40.0f, 0.f, 1.5f, 10.0f, 2.f};  ///< 自瞄俯仰位置 PID（打装甲板）
constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 10.0f, 0.f};    ///< 自瞄俯仰速度 PID（打装甲板）

constexpr PidGains kYawPositionPidRune{60.0f, 0.f, 1.5f, 10.0f, 2.2f};   ///< 自瞄偏航位置 PID（打符）
constexpr PidGains kYawSpeedPidRune{0.55f, 0.0f, 0.0f, 10.0f, 0.f};      ///< 自瞄偏航速度 PID（打符）
constexpr PidGains kPitchPositionPidRune{45.0f, 0.f, 1.5f, 10.0f, 2.f};  ///< 自瞄俯仰位置 PID（打符）
constexpr PidGains kPitchSpeedPidRune{0.55f, 0.0f, 0.0f, 10.0f, 0.f};    ///< 自瞄俯仰速度 PID（打符）
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
