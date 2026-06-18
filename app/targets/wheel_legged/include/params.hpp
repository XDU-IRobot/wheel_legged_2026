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
constexpr float kPitchGravityCompensationNm = 0.65f;

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
inline constexpr float kBoosterZeroPointRad = 0.11327f;  ///< 拨盘零位角度 [rad]
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
    .hook_theta_target_rad = 1.45f,
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
constexpr StairClimbParams kStairClimbStep2{
    .high_leg_length_m = 0.30f,
    .hook_leg_length_m = 0.38f,
    .retract_leg_length_m = 0.38f,
    .settle_leg_length_m = 0.38f,
    .contact_theta_threshold_rad = 0.40f,
    .hook_theta_target_rad = 1.45f,
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
constexpr float kLowLegLengthM = 0.13f;        ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.18f;        ///< 中腿长档位目标腿长 [m]
constexpr float kHighLegLengthM = 0.3f;        ///< 高腿长档位目标腿长 [m]
constexpr float kCtrlCStairLegLengthM = 0.2f;  ///< Ctrl+C 上台阶预备模式目标腿长 [m]
constexpr float kLegLengthRampTimeS = 0.3f;    ///< 腿长切换斜坡时间 [s]（从低到高腿长的过渡时间）
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
constexpr float kStandupPhase0ThetaTargetRad = 1.3f;  ///< 起立 Phase 0/1 腿摆角目标 [rad]
constexpr float kStandupPhase1TargetLengthM = 0.1f;   ///< 起立 Phase 1 目标腿长 [m]
constexpr float kStandupPhase1ThetaTolRad = 0.4f;  ///< 起立 Phase 1 完成判定：摆角与目标差值容许 [rad]
constexpr float kStandupThetaRampStepRad = 0.02f;  ///< 起立摆角斜坡步长 [rad/周期]
constexpr float kPostureRollMinRad = -0.3f;        ///< 横滚角安全下限 [rad]
constexpr float kPostureRollMaxRad = 0.3f;         ///< 横滚角安全上限 [rad]
constexpr float kPostureThetaBMinRad = -0.58f;     ///< 机体俯仰角安全下限 [rad]
constexpr float kPostureThetaBMaxRad = 0.5f;       ///< 机体俯仰角安全上限 [rad]
constexpr float kPostureThetaLegMinRad = -0.9f;    ///< 腿摆角安全下限 [rad]
constexpr float kPostureThetaLegMaxRad = 1.55f;    ///< 腿摆角安全上限 [rad]
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
constexpr float kOffGroundSupportForceClampN = 100.0f;     ///< 离地时支持力限幅值 [N]

// -- 中腿长下压 --
constexpr float kMidLegDipTriggerLengthM = 0.27f;  ///< 中腿长模式下触发下压的腿长阈值 [m]
constexpr float kMidLegDipTargetLengthM = 0.21f;   ///< 下压目标腿长 [m]
constexpr uint16_t kMidLegDipHoldTicks = 500;      ///< 下压维持时间 [ticks @ 500Hz = 1s]

// ==== 物理参数（变体专属）====
constexpr float kBodyMassKg = 27.0f;  ///< 机体质量 [kg]

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.059f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵 — 40 组多项式系数）====
/// 由 MATLAB 离线拟合得到，p(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l² + p11*l_l*l_r + p02*l_r²
/// 共 40 行，对应 4×10 增益矩阵 K 的 40 个元素（按行主序展平）
/// 每行 6 个系数：[p00, p10, p01, p20, p11, p02]
constexpr std::array<float, 240> kCtrlPLow{
    -3.7122,  -27.256,   21.796,   41.514,    -18.838,  -16.452,   -6.2917, -30.692,   32.079,   52.307,   -36.167,
    -22.267,  -0.35879,  1.5664,   -0.6196,   -1.6074,  0.022042,  0.96962, -1.9778,   8.8003,   -3.619,   -8.8366,
    -0.15271, 5.7309,    -12.785,  -82.626,   14.245,   82.435,    4.0887,  -18.899,   -0.75548, -6.1243,  2.6978,
    -4.0635,  2.4116,    -3.1989,  -4.0482,   5.8441,   -7.1651,   2.908,   -6.328,    6.5257,   -0.38388, -1.2505,
    -0.39605, 5.2868,    -10.532,  2.1339,    -21.842,  31.378,    30.16,   -0.022537, -33.546,  -28.049,  -2.6752,
    2.1045,   6.5735,    2.6572,   -6.9955,   -6.2944,  -3.7122,   21.796,  -27.256,   -16.452,  -18.838,  41.514,
    -6.2917,  32.079,    -30.692,  -22.267,   -36.167,  52.307,    0.35879, 0.6196,    -1.5664,  -0.96962, -0.022042,
    1.6074,   1.9778,    3.619,    -8.8003,   -5.7309,  0.15271,   8.8366,  -4.0482,   -7.1651,  5.8441,   6.5257,
    -6.328,   2.908,     -0.38388, -0.39605,  -1.2505,  2.1339,    -10.532, 5.2868,    -12.785,  14.245,   -82.626,
    -18.899,  4.0887,    82.435,   -0.75548,  2.6978,   -6.1243,   -3.1989, 2.4116,    -4.0635,  -21.842,  30.16,
    31.378,   -28.049,   -33.546,  -0.022537, -2.6752,  6.5735,    2.1045,  -6.2944,   -6.9955,  2.6572,   8.2573,
    2.4743,   -19.777,   -46.409,  45.548,    10.606,   12.304,    2.134,   -36.049,   -62.704,  77.179,   18.726,
    -0.3465,  -2.1131,   -0.72779, 3.645,     -0.91663, 1.1238,    -1.9112, -11.961,   -3.856,   20.483,   -5.2299,
    5.9262,   55.629,    -93.565,  6.9071,    78.789,   38.74,     -20.578, 2.4756,    0.52034,  -1.7487,  -0.59263,
    6.1834,   -0.085935, -2.5567,  -27.776,   -7.497,   32.773,    -18.986, -7.687,    -0.11834, -0.458,   2.9268,
    -4.4591,  2.257,     -6.5701,  -33.946,   -175.36,  54.372,    207.16,  31.247,    -73.059,  -1.4785,  -15.785,
    2.6753,   13.843,    9.608,    -5.2178,   8.2573,   -19.777,   2.4743,  10.606,    45.548,   -46.409,  12.304,
    -36.049,  2.134,     18.726,   77.179,    -62.704,  0.3465,    0.72779, 2.1131,    -1.1238,  0.91663,  -3.645,
    1.9112,   3.856,     11.961,   -5.9262,   5.2299,   -20.483,   -2.5567, -7.497,    -27.776,  -7.687,   -18.986,
    32.773,   -0.11834,  2.9268,   -0.458,    -6.5701,  2.257,     -4.4591, 55.629,    6.9071,   -93.565,  -20.578,
    38.74,    78.789,    2.4756,   -1.7487,   0.52034,  -0.085935, 6.1834,  -0.59263,  -33.946,  54.372,   -175.36,
    -73.059,  31.247,    207.16,   -1.4785,   2.6753,   -15.785,   -5.2178, 9.608,     13.843,
};
constexpr std::array<float, 240> kCtrlPMid{
    -3.245,   -23.717,  19.29,    36.474,   -17.019,  -14.64,    -6.4068, -32.588, 33.476,   55.392,   -37.359,
    -23.585,  -0.35659, 1.566,    -0.63303, -1.55,    -0.068984, 1.0015,  -1.9653, 8.8129,   -3.7115,  -8.5075,
    -0.71093, 5.9467,   -13.945,  -88.943,  13.981,   89.482,    5.7476,  -18.879, -0.76813, -6.5238,  2.8009,
    -4.4935,  2.7126,   -3.3965,  -4.0066,  4.0706,   -5.3796,   8.6829,  -9.7314, 1.7621,   -0.37802, -1.5215,
    -0.3246,  5.9864,   -11.547,  1.9971,   -22.079,  30.078,    30.789,  1.8634,  -33.13,   -29.078,  -2.7161,
    1.9429,   6.7776,   2.9002,   -7.047,   -6.5309,  -3.245,    19.29,   -23.717, -14.64,   -17.019,  36.474,
    -6.4068,  33.476,   -32.588,  -23.585,  -37.359,  55.392,    0.35659, 0.63303, -1.566,   -1.0015,  0.068984,
    1.55,     1.9653,   3.7115,   -8.8129,  -5.9467,  0.71093,   8.5075,  -4.0066, -5.3796,  4.0706,   1.7621,
    -9.7314,  8.6829,   -0.37802, -0.3246,  -1.5215,  1.9971,    -11.547, 5.9864,  -13.945,  13.981,   -88.943,
    -18.879,  5.7476,   89.482,   -0.76813, 2.8009,   -6.5238,   -3.3965, 2.7126,  -4.4935,  -22.079,  30.789,
    30.078,   -29.078,  -33.13,   1.8634,   -2.7161,  6.7776,    1.9429,  -6.5309, -7.047,   2.9002,   6.9812,
    2.8554,   -16.302,  -41.279,  37.86,    9.4316,   12.32,     3.7131,  -35.014, -66.99,   75.708,   19.194,
    -0.35313, -2.09,    -0.69435, 3.5834,   -0.93383, 1.1126,    -1.9484, -11.852, -3.6416,  20.159,   -5.3274,
    5.83,     62.131,   -108.86,  7.042,    91.798,   38.17,     -20.109, 2.5345,  1.0108,   -1.7626,  -1.3465,
    6.3277,   0.022272, -2.861,   -26.108,  -5.4944,  28.419,    -23.584, -2.7571, -0.165,   0.051054, 3.1912,
    -5.5577,  1.8824,   -5.8966,  -33.123,  -177.62,  54.526,    209.18,  31.247,  -73.534,  -1.4341,  -15.71,
    2.6541,   13.552,   9.5906,   -5.2079,  6.9812,   -16.302,   2.8554,  9.4316,  37.86,    -41.279,  12.32,
    -35.014,  3.7131,   19.194,   75.708,   -66.99,   0.35313,   0.69435, 2.09,    -1.1126,  0.93383,  -3.5834,
    1.9484,   3.6416,   11.852,   -5.83,    5.3274,   -20.159,   -2.861,  -5.4944, -26.108,  -2.7571,  -23.584,
    28.419,   -0.165,   3.1912,   0.051054, -5.8966,  1.8824,    -5.5577, 62.131,  7.042,    -108.86,  -20.109,
    38.17,    91.798,   2.5345,   -1.7626,  1.0108,   0.022272,  6.3277,  -1.3465, -33.123,  54.526,   -177.62,
    -73.534,  31.247,   209.18,   -1.4341,  2.6541,   -15.71,    -5.2079, 9.5906,  13.552,
};
constexpr std::array<float, 240> kCtrlPHigh{
    -2.467,   -17.29,  14.239,   27.24,    -12.315,  -11.352, -5.128,   -25.714,  26.656,   44.449,  -28.651,   -19.92,
    -0.32149, 1.4986,  -0.43832, -1.706,   0.27999,  0.67662, -1.775,   8.4196,   -2.5983,  -9.422,  1.3753,    4.0681,
    -12.548,  -73.393, 11.436,   75.61,    -1.2163,  -14.867, -0.7373,  -5.2718,  2.2717,   -3.5533, 1.6709,    -2.7926,
    -3.5981,  6.7091,  -5.4473,  -1.4655,  0.60076,  4.5827,  -0.32836, -1.0025,  -0.33004, 4.2921,  -8.1594,   1.5795,
    -18.805,  32.683,  23.147,   -10.323,  -26.076,  -22.79,  -2.4108,  2.4678,   5.4505,   1.5302,  -5.8165,   -5.4582,
    -2.467,   14.239,  -17.29,   -11.352,  -12.315,  27.24,   -5.128,   26.656,   -25.714,  -19.92,  -28.651,   44.449,
    0.32149,  0.43832, -1.4986,  -0.67662, -0.27999, 1.706,   1.775,    2.5983,   -8.4196,  -4.0681, -1.3753,   9.422,
    -3.5981,  -5.4473, 6.7091,   4.5827,   0.60076,  -1.4655, -0.32836, -0.33004, -1.0025,  1.5795,  -8.1594,   4.2921,
    -12.548,  11.436,  -73.393,  -14.867,  -1.2163,  75.61,   -0.7373,  2.2717,   -5.2718,  -2.7926, 1.6709,    -3.5533,
    -18.805,  23.147,  32.683,   -22.79,   -26.076,  -10.323, -2.4108,  5.4505,   2.4678,   -5.4582, -5.8165,   1.5302,
    4.1652,   -1.4159, -7.8019,  -20.364,  22.255,   4.0379,  7.785,    -2.9103,  -18.708,  -35.249, 46.582,    9.8443,
    -0.31334, -1.5817, -0.68676, 2.811,    -0.48778, 1.0329,  -1.7271,  -8.9902,  -3.7277,  15.889,  -2.9038,   5.6026,
    41.813,   -74.598, 7.9154,   70.685,   26.382,   -18.424, 1.9543,   -0.95663, -0.65676, 1.1432,  4.9713,    -0.6828,
    -2.5062,  -21.69,  -7.6573,  29.061,   -15.104,  -3.1611, -0.19114, -0.78465, 2.3055,   -1.838,  -0.088956, -4.276,
    -29.104,  -129.06, 44.571,   160.54,   17.355,   -60.103, -1.7643,  -13.255,  3.8407,   13.232,  5.8975,    -5.99,
    4.1652,   -7.8019, -1.4159,  4.0379,   22.255,   -20.364, 7.785,    -18.708,  -2.9103,  9.8443,  46.582,    -35.249,
    0.31334,  0.68676, 1.5817,   -1.0329,  0.48778,  -2.811,  1.7271,   3.7277,   8.9902,   -5.6026, 2.9038,    -15.889,
    -2.5062,  -7.6573, -21.69,   -3.1611,  -15.104,  29.061,  -0.19114, 2.3055,   -0.78465, -4.276,  -0.088956, -1.838,
    41.813,   7.9154,  -74.598,  -18.424,  26.382,   70.685,  1.9543,   -0.65676, -0.95663, -0.6828, 4.9713,    1.1432,
    -29.104,  44.571,  -129.06,  -60.103,  17.355,   160.54,  -1.7643,  3.8407,   -13.255,  -5.99,   5.8975,    13.232,
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
constexpr PidGains kLeftL0Pid{2500.0f, 0.1f, 130.0f, 130.0f, 10.0f};          ///< 左腿腿长 PID（常规）
constexpr PidGains kRightL0Pid{2500.0f, 0.1f, 130.0f, 130.0f, 10.0f};         ///< 右腿腿长 PID（常规）
constexpr PidGains kLeftL0PidStandup{6300.0f, 0.1f, 160.0f, 145.0f, 10.0f};   ///< 左腿腿长 PID（起立）
constexpr PidGains kRightL0PidStandup{6300.0f, 0.1f, 160.0f, 145.0f, 10.0f};  ///< 右腿腿长 PID（起立）
constexpr PidGains kRollPid{1000.0f, 0.1f, 20.0f, 100.0f, 40.0f};             ///< 横滚平衡 PID

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
constexpr PidGains kLeftLegTurnPid{30.0f, 0.f, 60.0f, 15.0f, 9.0f};   ///< 左腿摆角速度 PID（倒地恢复用）
constexpr PidGains kRightLegTurnPid{30.0f, 0.f, 60.0f, 15.0f, 9.0f};  ///< 右腿摆角速度 PID（倒地恢复用）
constexpr PidGains kLeftLegAnglePidStandup{10.0f, 0.0f, 0.0f, 10.0f, 0.0f};   ///< 左腿摆角 PID（起立用）
constexpr PidGains kRightLegAnglePidStandup{10.0f, 0.0f, 0.0f, 10.0f, 0.0f};  ///< 右腿摆角 PID（起立用）
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
constexpr float kMaxSafeYawRateRadS = 3.5f;                  ///< 摩擦圆最大安全偏航速率 [rad/s]
constexpr float kTargetForwardSpeedMaxMps = 2.4f;            ///< 最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.2f;     ///< 高腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxMidLegMps = 1.7f;      ///< F键中腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxNoScMps = 1.1f;        ///< 无超电最大前进速度 [m/s]
constexpr float kTargetSpeedBiasLowLegMps = 0.0f;            ///< 低腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegMps = 0.f;             ///< C键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegFMps = 0.0f;           ///< F键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasHighLegMps = 0.0f;           ///< 高腿长目标速度偏置 [m/s]
constexpr float kTargetForwardSpeedMaxCtrlCStairMps = 1.2f;  ///< Ctrl+C 上台阶预备模式最大前进速度 [m/s]
constexpr float kTargetSpeedBiasCtrlCStairMps = 0.0f;        ///< Ctrl+C 上台阶预备模式速度偏置 [m/s]
constexpr float kVxInputDeadbandNorm = 0.1f;                 ///< 前进输入死区（归一化值，低于此忽略）
constexpr float kVyInputDeadbandNorm = 0.1f;                 ///< 平移输入死区（归一化值）
constexpr float kYawFollowRampStepRadS = 0.3f;               ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kYawFollowRampStepRadNoScS = 0.08f;  ///< 偏航跟随角速度斜坡步长（无超电）[(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.15f;  ///< 位置锚定冻结速度阈值 [m/s]（车速低于此值时锁定位置）
constexpr float kLqrStopDampingK = 0.1f;  ///< LQR 目标速度为零时主动阻尼系数
constexpr uint32_t kPositionHoldTimeoutTicks =
    1000U;  ///< 位置锚定超时 [ticks]（斜坡归零后最多等待此周期数，超时强制冻结）

// ==== 落地减速（离地→落地时通过腿摆角辅助减速）====
constexpr float kLandingDecelThetaGain = 0.f;                ///< 落地减速腿摆角增益 [rad/(m/s)]
constexpr float kLandingDecelThetaMaxRad = 0.3f;             ///< 落地减速腿摆角最大偏置 [rad]
constexpr float kLandingDecelThetaRampStepRad = 0.01f;       ///< 落地减速腿摆角每周期斜坡步长 [rad/tick]
constexpr std::uint32_t kLandingDecelOffGroundMinMs = 100U;  ///< 离地最短持续时间（防单帧误判）[ms]
constexpr std::uint32_t kLandingDecelStableDurationMs = 400U;  ///< 落地减速稳定保持时间 [ms]

constexpr float kYawFollowFixedTargetRad = -2.076f;    ///< 偏航跟随固定目标偏置角 [rad]（前进方向）
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;  ///< 偏航跟随侧向目标偏置角 [rad]（±π/2）
constexpr PidGains kYawFollowPid{40.0f, 0.0f, 4.f, 8.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置（腿摆角/机体俯仰）====
constexpr float kExpectedThetaLlBiasRadLowLeg = -0.05f;    ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = -0.05f;    ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLegC = -0.12f;   ///< 中腿长(C键)期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLegC = -0.12f;   ///< 中腿长(C键)期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLegF = -0.06f;   ///< 中腿长(F键)期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLegF = -0.06f;   ///< 中腿长(F键)期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadHighLeg = -0.03f;   ///< 高腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadHighLeg = -0.03f;   ///< 高腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = -0.088f;          ///< 期望机体俯仰偏置 [rad]
constexpr float kExpectedThetaBSpeedK = -0.03f;            ///< 期望机体俯仰速度系数（theta_b += k * s_dot）
constexpr float kExpectedDisplacementBiasM = 0.0f;         ///< 期望位移偏置 [m]
constexpr float kExpectedDisplacementBiasMLowLeg = -0.f;   ///< 低腿长期望位移偏置 [m]
constexpr float kExpectedDisplacementBiasMMidLeg = 0.0f;   ///< 中腿长期望位移偏置 [m]
constexpr float kExpectedDisplacementBiasMHighLeg = 0.0f;  ///< 高腿长期望位移偏置 [m]

// ==== 速度斜坡参数（按腿长档位分级）====
constexpr SdotRampParams kSdotRampLowLeg{0.007f, 0.007f};        ///< 低腿长速度斜坡（加速/制动步长）
constexpr SdotRampParams kSdotRampMidLeg{0.0060f, 0.006f};       ///< 中腿长速度斜坡(C 键触发)
constexpr SdotRampParams kSdotRampMidLegF{0.0055f, 0.0055f};     ///< 中腿长速度斜坡（G 键触发）
constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};       ///< 高腿长速度斜坡
constexpr SdotRampParams kSdotRampCtrlCStair{0.0055f, 0.0055f};  ///< Ctrl+C 上台阶预备模式速度斜坡

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
constexpr float kLeftPhi1OffsetRad = 2.8f + M_PI + 0.002f;  ///< 左腿前关节零位偏移 [rad]
constexpr float kLeftPhi4OffsetRad = 0.4f - 0.05;           ///< 左腿后关节零位偏移 [rad]
constexpr float kRightPhi1OffsetRad = 2.1244f + M_PI + 0.13f - 0.068f + 0.019f - 0.02;  ///< 右腿前关节零位偏移 [rad]
constexpr float kRightPhi4OffsetRad = 0.46f + 0.123f + 0.136 - 0.11f + 0.05f;  ///< 右腿后关节零位偏移 [rad]
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
constexpr uint8_t kRobotId = 1U;                                      ///< 机器人 ID（裁判系统回退值）
constexpr float kBulletSpeedMps = 11.5f;                              ///< 弹速 [m/s]（裁判系统回退值）
constexpr float kBulletDefaultSpeedMps = 11.5f;                       ///< 默认弹速
constexpr float kBulletBoundarySpeedMps = 10.5f;                      ///< 区分裁判系统返回值是否正确
constexpr PidGains kYawPositionPidRune{55.0f, 0.1, 2.f, 8.0f, 5.0f};  ///< 自瞄偏航位置 PID（打符）
constexpr PidGains kYawSpeedPidRune{0.8f, 0.05f, 0.0f, 6.4f, 0.6f};   ///< 自瞄偏航速度 PID（打符）
constexpr PidGains kPitchPositionPidRune{48, 0.f, 1.5f, 8.0f, 4.f};   ///< 自瞄俯仰位置 PID（打符）
constexpr PidGains kPitchSpeedPidRune{0.8f, 0.0f, 0.0f, 6.4f, 0.4f};  ///< 自瞄俯仰速度 PID（打符）
constexpr PidGains kYawPositionPid{55.0f, 0.1, 2.f, 8.0f, 5.0f};      ///< 自瞄偏航位置 PID
constexpr PidGains kYawSpeedPid{0.8f, 0.05f, 0.0f, 6.4f, 0.6f};       ///< 自瞄偏航速度 PID
constexpr PidGains kPitchPositionPid{48, 0.0f, 1.5f, 8.0f, 4.f};      ///< 自瞄俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.8f, 0.0f, 0.0f, 6.4f, 0.4f};      ///< 自瞄俯仰速度 PID
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

inline constexpr PidGains kYawPositionPid{32.0f, 0.f, 1.f, 10.0f, 0.0f};    ///< 偏航位置 PID
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
constexpr float kLowLegLengthM = 0.16f;              ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.23f;              ///< 中腿长档位目标腿长 [m]
constexpr float kHighLegLengthM = 0.33f;             ///< 高腿长档位目标腿长 [m]
constexpr float kCtrlCStairLegLengthM = 0.2f;        ///< Ctrl+C 上台阶预备模式目标腿长 [m]
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
constexpr float kStandupThetaThresholdRad = 0.9f;  ///< 起立完成判定：双腿摆角绝对值低于此值后允许轮端输出 [rad]
constexpr float kStandupPhase0ThetaTargetRad = 1.5f;  ///< 起立 Phase 0/1 腿摆角目标 [rad]
constexpr float kStandupPhase1TargetLengthM = 0.1f;   ///< 起立 Phase 1 目标腿长 [m]
constexpr float kStandupPhase1ThetaTolRad = 0.6f;   ///< 起立 Phase 1 完成判定：摆角与目标差值容许 [rad]
constexpr float kStandupThetaRampStepRad = 0.02f;   ///< 起立摆角斜坡步长 [rad/周期]
constexpr float kPostureRollMinRad = -1.f;          ///< 横滚角安全下限 [rad]
constexpr float kPostureRollMaxRad = 1.f;           ///< 横滚角安全上限 [rad]
constexpr float kPostureThetaBMinRad = -0.7f;       ///< 机体俯仰角安全下限 [rad]
constexpr float kPostureThetaBMaxRad = 0.7f;        ///< 机体俯仰角安全上限 [rad]
constexpr float kPostureThetaLegMinRad = -0.8f;     ///< 腿摆角安全下限 [rad]
constexpr float kPostureThetaLegMaxRad = 2.f;       ///< 腿摆角安全上限 [rad]
constexpr float kLegRecoverThetaDotTarget = -2.0f;  ///< 倒地恢复时腿摆角速度目标 [rad/s]
constexpr float kLegRecoverThetaDotRampStep = 0.008f;  ///< 倒地恢复腿摆角速度斜坡步长 [(rad/s)/周期]
constexpr float kManualRecoveryLegSpeedRadS = 0.5f;    ///< 手动倒地恢复腿摆角速度 [rad/s]
constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;    ///< 倒地恢复零力矩区间下限 [rad]
constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;    ///< 倒地恢复零力矩区间上限 [rad]
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

// ==== 物理参数（变体专属）====

constexpr float kBodyMassKg = 22.0f;  ///< 机体质量 [kg]

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.0198f;  ///< 横滚平衡目标角 [rad]

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
    -0.89916, -7.8431,  6.2342,   12.591,  -5.7131,  -5.0901, -4.381,   -33.888, 28.749,  55.525,   -28.118, -23.13,
    -0.41465, 1.9251,   -0.78929, -1.6687, -0.23525, 1.3043,  -1.3334,  6.4079,  -2.8207, -5.2747,  -1.163,  4.7313,
    -11.011,  -81.559,  14.015,   73.899,  9.1395,   -20.322, -0.6637,  -6.5578, 2.6105,  -4.3858,  3.122,   -3.6686,
    -3.158,   -3.8879,  -0.80124, 27.785,  -21.05,   -4.6079, -0.26946, -2.2981, 0.14417, 7.2833,   -11.667, 0.76359,
    -21.984,  24.695,   31.723,   9.7138,  -30.035,  -31.391, -2.581,   1.2716,  6.3956,  3.5043,   -5.8053, -6.554,
    -0.89916, 6.2342,   -7.8431,  -5.0901, -5.7131,  12.591,  -4.381,   28.749,  -33.888, -23.13,   -28.118, 55.525,
    0.41465,  0.78929,  -1.9251,  -1.3043, 0.23525,  1.6687,  1.3334,   2.8207,  -6.4079, -4.7313,  1.163,   5.2747,
    -3.158,   -0.80124, -3.8879,  -4.6079, -21.05,   27.785,  -0.26946, 0.14417, -2.2981, 0.76359,  -11.667, 7.2833,
    -11.011,  14.015,   -81.559,  -20.322, 9.1395,   73.899,  -0.6637,  2.6105,  -6.5578, -3.6686,  3.122,   -4.3858,
    -21.984,  31.723,   24.695,   -31.391, -30.035,  9.7138,  -2.581,   6.3956,  1.2716,  -6.554,   -5.8053, 3.5043,
    2.3999,   0.26553,  -4.7791,  -14.312, 11.971,   4.2921,  11.216,   1.0754,  -24.598, -64.886,  58.592,  21.134,
    -0.42541, -2.9442,  -0.69804, 4.9744,  -1.391,   1.4385,  -1.3667,  -9.9882, -1.9938, 16.738,   -4.9567, 4.3281,
    53.056,   -75.799,  5.7168,   35.819,  42.967,   -17.797, 2.455,    2.3975,  -1.6752, -5.8597,  7.222,   0.74538,
    -2.4153,  -23.198,  2.2125,   15.41,   -38.927,  20.426,  -0.25474, 1.1377,  3.8092,  -7.3993,  -2.2695, -0.91536,
    -34.058,  -185,     54.951,   214.27,  37.771,   -80.86,  -1.4545,  -16.236, 3.2294,  14.127,   8.9186,  -5.9463,
    2.3999,   -4.7791,  0.26553,  4.2921,  11.971,   -14.312, 11.216,   -24.598, 1.0754,  21.134,   58.592,  -64.886,
    0.42541,  0.69804,  2.9442,   -1.4385, 1.391,    -4.9744, 1.3667,   1.9938,  9.9882,  -4.3281,  4.9567,  -16.738,
    -2.4153,  2.2125,   -23.198,  20.426,  -38.927,  15.41,   -0.25474, 3.8092,  1.1377,  -0.91536, -2.2695, -7.3993,
    53.056,   5.7168,   -75.799,  -17.797, 42.967,   35.819,  2.455,    -1.6752, 2.3975,  0.74538,  7.222,   -5.8597,
    -34.058,  54.951,   -185,     -80.86,  37.771,   214.27,  -1.4545,  3.2294,  -16.236, -5.9463,  8.9186,  14.127,
};
constexpr std::array<float, 240> kCtrlPHigh{
    -0.89916, -7.8431,  6.2342,   12.591,  -5.7131,  -5.0901, -4.381,   -33.888, 28.749,  55.525,   -28.118, -23.13,
    -0.41465, 1.9251,   -0.78929, -1.6687, -0.23525, 1.3043,  -1.3334,  6.4079,  -2.8207, -5.2747,  -1.163,  4.7313,
    -11.011,  -81.559,  14.015,   73.899,  9.1395,   -20.322, -0.6637,  -6.5578, 2.6105,  -4.3858,  3.122,   -3.6686,
    -3.158,   -3.8879,  -0.80124, 27.785,  -21.05,   -4.6079, -0.26946, -2.2981, 0.14417, 7.2833,   -11.667, 0.76359,
    -21.984,  24.695,   31.723,   9.7138,  -30.035,  -31.391, -2.581,   1.2716,  6.3956,  3.5043,   -5.8053, -6.554,
    -0.89916, 6.2342,   -7.8431,  -5.0901, -5.7131,  12.591,  -4.381,   28.749,  -33.888, -23.13,   -28.118, 55.525,
    0.41465,  0.78929,  -1.9251,  -1.3043, 0.23525,  1.6687,  1.3334,   2.8207,  -6.4079, -4.7313,  1.163,   5.2747,
    -3.158,   -0.80124, -3.8879,  -4.6079, -21.05,   27.785,  -0.26946, 0.14417, -2.2981, 0.76359,  -11.667, 7.2833,
    -11.011,  14.015,   -81.559,  -20.322, 9.1395,   73.899,  -0.6637,  2.6105,  -6.5578, -3.6686,  3.122,   -4.3858,
    -21.984,  31.723,   24.695,   -31.391, -30.035,  9.7138,  -2.581,   6.3956,  1.2716,  -6.554,   -5.8053, 3.5043,
    2.3999,   0.26553,  -4.7791,  -14.312, 11.971,   4.2921,  11.216,   1.0754,  -24.598, -64.886,  58.592,  21.134,
    -0.42541, -2.9442,  -0.69804, 4.9744,  -1.391,   1.4385,  -1.3667,  -9.9882, -1.9938, 16.738,   -4.9567, 4.3281,
    53.056,   -75.799,  5.7168,   35.819,  42.967,   -17.797, 2.455,    2.3975,  -1.6752, -5.8597,  7.222,   0.74538,
    -2.4153,  -23.198,  2.2125,   15.41,   -38.927,  20.426,  -0.25474, 1.1377,  3.8092,  -7.3993,  -2.2695, -0.91536,
    -34.058,  -185,     54.951,   214.27,  37.771,   -80.86,  -1.4545,  -16.236, 3.2294,  14.127,   8.9186,  -5.9463,
    2.3999,   -4.7791,  0.26553,  4.2921,  11.971,   -14.312, 11.216,   -24.598, 1.0754,  21.134,   58.592,  -64.886,
    0.42541,  0.69804,  2.9442,   -1.4385, 1.391,    -4.9744, 1.3667,   1.9938,  9.9882,  -4.3281,  4.9567,  -16.738,
    -2.4153,  2.2125,   -23.198,  20.426,  -38.927,  15.41,   -0.25474, 3.8092,  1.1377,  -0.91536, -2.2695, -7.3993,
    53.056,   5.7168,   -75.799,  -17.797, 42.967,   35.819,  2.455,    -1.6752, 2.3975,  0.74538,  7.222,   -5.8597,
    -34.058,  54.951,   -185,     -80.86,  37.771,   214.27,  -1.4545,  3.2294,  -16.236, -5.9463,  8.9186,  14.127,
};
constexpr std::array<float, 240> kCtrlPSpin{
    -3.0039,  -24.696, 20.387,   41.33,   -23.885, -12.527, -7.3591,  -42.179, 42.466,  76.326,  -55.795, -24.977,
    -0.43257, 2.2555,  -1.0088,  -1.5814, -1.1625, 1.952,   -0.99432, 5.7113,  -2.9549, -3.4655, -3.8649, 5.7849,
    -20.703,  -109.16, 15.206,   120.02,  7.5653,  -19.987, -1.1245,  -8.1597, 3.1829,  -3.6678, 2.9966,  -3.3855,
    -4.5666,  1.0701,  2.695,    20.799,  -32.939, 1.8031,  -0.41493, -2.1558, 0.19571, 8.4482,  -16.776, 3.5541,
    -35.907,  61.955,  39.645,   -21.505, -39.458, -40.79,  -3.3661,  2.9334,  7.9258,  2.5682,  -8.1252, -7.7087,
    -3.0039,  20.387,  -24.696,  -12.527, -23.885, 41.33,   -7.3591,  42.466,  -42.179, -24.977, -55.795, 76.326,
    0.43257,  1.0088,  -2.2555,  -1.952,  1.1625,  1.5814,  0.99432,  2.9549,  -5.7113, -5.7849, 3.8649,  3.4655,
    -4.5666,  2.695,   1.0701,   1.8031,  -32.939, 20.799,  -0.41493, 0.19571, -2.1558, 3.5541,  -16.776, 8.4482,
    -20.703,  15.206,  -109.16,  -19.987, 7.5653,  120.02,  -1.1245,  3.1829,  -8.1597, -3.3855, 2.9966,  -3.6678,
    -35.907,  39.645,  61.955,   -40.79,  -39.458, -21.505, -3.3661,  7.9258,  2.9334,  -7.7087, -8.1252, 2.5682,
    6.1748,   -5.8244, -7.5637,  -32.636, 38.237,  3.0229,  13.863,   -8.2436, -30.286, -69.943, 91.864,  16.459,
    -0.51363, -2.8263, -0.38513, 4.701,   -1.3656, 1.145,   -1.1858,  -7.3745, -0.3405, 12.053,  -3.8211, 1.9211,
    80.506,   -177.26, 7.4929,   143.87,  40.345,  -21.868, 3.5485,   -2.1682, -2.0135, -3.4755, 10.169,  -0.0929,
    -4.5619,  -25.783, 6.5688,   20.821,  -15.302, 5.5668,  -0.17986, 0.12814, 3.5415,  -6.7561, 0.62129, -1.8365,
    -62.785,  -278.33, 92.854,   357.72,  37.178,  -140.01, -2.3828,  -19.441, 4.5648,  19.106,  10.779,  -9.2087,
    6.1748,   -7.5637, -5.8244,  3.0229,  38.237,  -32.636, 13.863,   -30.286, -8.2436, 16.459,  91.864,  -69.943,
    0.51363,  0.38513, 2.8263,   -1.145,  1.3656,  -4.701,  1.1858,   0.3405,  7.3745,  -1.9211, 3.8211,  -12.053,
    -4.5619,  6.5688,  -25.783,  5.5668,  -15.302, 20.821,  -0.17986, 3.5415,  0.12814, -1.8365, 0.62129, -6.7561,
    80.506,   7.4929,  -177.26,  -21.868, 40.345,  143.87,  3.5485,   -2.0135, -2.1682, -0.0929, 10.169,  -3.4755,
    -62.785,  92.854,  -278.33,  -140.01, 37.178,  357.72,  -2.3828,  4.5648,  -19.441, -9.2087, 10.779,  19.106,
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
constexpr PidGains kLeftLegTurnPid{18.0f, 0.f, 0.0f, 11.5f, 10.0f};          ///< 左腿摆角速度 PID
constexpr PidGains kRightLegTurnPid{18.0f, 0.f, 0.0f, 11.5f, 10.0f};         ///< 右腿摆角速度 PID
constexpr PidGains kLeftLegAnglePidStandup{10.0f, 0.0f, 0.0f, 6.0f, 0.0f};   ///< 左腿摆角 PID（起立用）
constexpr PidGains kRightLegAnglePidStandup{10.0f, 0.0f, 0.0f, 6.0f, 0.0f};  ///< 右腿摆角 PID（起立用）
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
constexpr float kGimbalStartupYawAlignErrorRad = 0.1f;            ///< 归中完成位置误差阈值 [rad]
constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;            ///< 归中完成速度阈值 [rad/s]
constexpr std::uint32_t kGimbalStartupYawAlignStableTicks = 10U;  ///< 归中判稳所需连续满足周期数

// -- 偏航就绪判稳 --
constexpr float kYawFollowDriveReadyErrorRad = 0.1f;            ///< 偏航就绪位置误差阈值 [rad]
constexpr float kYawFollowDriveReadyVelRadS = 0.25f;            ///< 偏航就绪速度阈值 [rad/s]
constexpr std::uint32_t kYawFollowDriveReadyStableTicks = 50U;  ///< 偏航就绪判稳所需连续周期数

// ==== 基本运动 ====
constexpr float kTargetForwardSpeedMaxMps = 2.22f;           ///< 最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxNoScMps = 1.2f;        ///< 无超电最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.5f;     ///< 高腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxMidLegMps = 1.0f;      ///< F键中腿长模式最大前进速度 [m/s]
constexpr float kTargetSpeedBiasLowLegMps = 0.f;             ///< 低腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegMps = 0.0f;            ///< C键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegFMps = 0.0f;           ///< F键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasHighLegMps = 0.0f;           ///< 高腿长目标速度偏置 [m/s]
constexpr float kTargetForwardSpeedMaxCtrlCStairMps = 1.3f;  ///< Ctrl+C 上台阶预备模式最大前进速度 [m/s]
constexpr float kTargetSpeedBiasCtrlCStairMps = 0.0f;        ///< Ctrl+C 上台阶预备模式速度偏置 [m/s]
constexpr float kMaxSafeYawRateRadS = 4.5f;                  ///< 摩擦圆最大安全偏航速率 [rad/s]
constexpr float kVxInputDeadbandNorm = 0.05f;                ///< 前进输入死区
constexpr float kVyInputDeadbandNorm = 0.05f;                ///< 平移输入死区
constexpr float kYawFollowRampStepRadS = 0.11f;              ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
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
constexpr float kYawFollowFixedTargetRad = 1.42f;             ///< 偏航跟随固定目标偏置角 [rad]
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;         ///< 偏航跟随侧向目标偏置角 [rad]
constexpr PidGains kYawFollowPid{32.0f, 0.0f, 3.f, 8.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置 ====
constexpr float kExpectedThetaLlBiasRadLowLeg = 0.02f;   ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = 0.02f;   ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLeg = 0.05f;   ///< 中腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLeg = 0.05f;   ///< 中腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadHighLeg = 0.02f;  ///< 高腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadHighLeg = 0.02f;  ///< 高腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = 0.025f;         ///< 期望机体俯仰偏置 [rad]
// constexpr float kExpectedDisplacementBiasMLowLeg = -0.08f;    ///< 低腿长期望位移偏置 [m]
constexpr float kExpectedDisplacementBiasMLowLeg = -0.09f;  ///< 低腿长期望位移偏置 [m]
constexpr float kExpectedDisplacementBiasMMidLeg = 0.127f;  ///< 中腿长期望位移偏置 [m]
constexpr float kExpectedDisplacementBiasMHighLeg = -0.2f;  ///< 高腿长期望位移偏置 [m]

// ==== 速度斜坡参数 ====
constexpr SdotRampParams kSdotRampLowLeg{0.0065f, 0.007f};      ///< 低腿长速度斜坡
constexpr SdotRampParams kSdotRampMidLeg{0.0035f, 0.007f};      ///< 中腿长速度斜坡(C 键触发)
constexpr SdotRampParams kSdotRampMidLegF{0.0045f, 0.006f};     ///< 中腿长速度斜坡（F 键触发）
constexpr SdotRampParams kSdotRampHighLeg{0.006f, 0.005f};      ///< 高腿长速度斜坡
constexpr SdotRampParams kSdotRampCtrlCStair{0.0045f, 0.006f};  ///< Ctrl+C 上台阶预备模式速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.08f;           ///< 小陀螺进入偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinExitYawRampStepRadS = 0.08f;       ///< 小陀螺退出偏航角速度斜坡步长 [(rad/s)/周期]
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
constexpr float kSpinThetaLlBiasRad = 0.12f;  ///< 小陀螺时左腿摆角偏置 [rad]
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
constexpr float kLowLegLengthM = 0.18f;              ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.245f;             ///< 中腿长档位目标腿长 [m]
constexpr float kHighLegLengthM = 0.35f;             ///< 高腿长档位目标腿长 [m]
constexpr float kCtrlCStairLegLengthM = 0.2f;        ///< Ctrl+C 上台阶预备模式目标腿长 [m]
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
constexpr float kStandupPhase1ThetaTolRad = 0.4f;   ///< 起立 Phase 1 完成判定：摆角与目标差值容许 [rad]
constexpr float kStandupThetaRampStepRad = 0.02f;   ///< 起立摆角斜坡步长 [rad/周期]
constexpr float kPostureRollMinRad = -0.5f;         ///< 横滚角安全下限 [rad]
constexpr float kPostureRollMaxRad = 0.5f;          ///< 横滚角安全上限 [rad]
constexpr float kPostureThetaBMinRad = -0.7f;       ///< 机体俯仰角安全下限 [rad]
constexpr float kPostureThetaBMaxRad = 0.7f;        ///< 机体俯仰角安全上限 [rad]
constexpr float kPostureThetaLegMinRad = -0.8f;     ///< 腿摆角安全下限 [rad]
constexpr float kPostureThetaLegMaxRad = 2.f;       ///< 摆角安全上限 [rad]
constexpr float kLegRecoverThetaDotTarget = -2.0f;  ///< 倒地恢复时腿摆角速度目标 [rad/s]
constexpr float kLegRecoverThetaDotRampStep = 0.06f;  ///< 倒地恢复腿摆角速度斜坡步长 [(rad/s)/周期]
constexpr float kManualRecoveryLegSpeedRadS = 0.5f;   ///< 手动倒地恢复腿摆角速度 [rad/s]
constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;   ///< 倒地恢复零力矩区间下限 [rad]
constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;   ///< 倒地恢复零力矩区间上限 [rad]

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

// ==== 物理参数（变体专属）====
constexpr float kBodyMassKg = 22.0f;  ///< 机体质量 [kg]

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.002f;  ///< 横滚平衡目标角 [rad]

// ==== 基本运动（LQR 增益矩阵）====
constexpr std::array<float, 240> kCtrlPLow{
    -4.8474, -28.617, 21.883,  40.383,  -12.065, -20.504, -8.019,   -33.402, 33.754,  53.044,  -30.813,  -28.479,
    -6.5925, 21.874,  -9.395,  -27.888, 2.6562,  12.321,  -4.7634,  16.928,  -8.1197, -20.066, 0.53838,  11.105,
    -13.415, -92.846, 20.706,  84.381,  10.945,  -28.236, -0.80742, -7.6917, 3.2504,  -5.2151, 3.4387,   -4.409,
    -5.5758, 8.252,   -15.387, 2.7688,  -1.5609, 0.66202, -0.47229, -1.4354, -1.2388, 5.6184,  -10.804,  0.33611,
    -25.009, 32.104,  37.666,  4.2434,  -45.503, -29.534, -3.0637,  2.2691,  7.5829,  2.888,   -8.0823,  -6.9605,
    -4.8474, 21.883,  -28.617, -20.504, -12.065, 40.383,  -8.019,   33.754,  -33.402, -28.479, -30.813,  53.044,
    6.5925,  9.395,   -21.874, -12.321, -2.6562, 27.888,  4.7634,   8.1197,  -16.928, -11.105, -0.53838, 20.066,
    -5.5758, -15.387, 8.252,   0.66202, -1.5609, 2.7688,  -0.47229, -1.2388, -1.4354, 0.33611, -10.804,  5.6184,
    -13.415, 20.706,  -92.846, -28.236, 10.945,  84.381,  -0.80742, 3.2504,  -7.6917, -4.409,  3.4387,   -5.2151,
    -25.009, 37.666,  32.104,  -29.534, -45.503, 4.2434,  -3.0637,  7.5829,  2.2691,  -6.9605, -8.0823,  2.888,
    10.137,  16.468,  -37.731, -61.266, 53.976,  19.926,  14.881,   18.714,  -57.707, -81.341, 90.02,    28.38,
    -4.73,   -28.112, -15.043, 54.565,  -17.957, 21.632,  -3.4593,  -22.344, -9.9326, 41.192,  -13.806,  13.441,
    58.914,  -80.325, 10.102,  83.023,  70.463,  -29.337, 2.7548,   2.1249,  -2.3171, 4.7358,  3.5221,   0.1199,
    -3.3992, -34.126, -14.27,  47.332,  -52.227, -20.22,  -0.21333, 0.1712,  3.7343,  -6.1457, 6.2871,   -14.375,
    -37.473, -188.01, 59.301,  205.76,  30.908,  -61.388, -1.401,   -15.243, 0.93539, 10.937,  10.865,   -2.0305,
    10.137,  -37.731, 16.468,  19.926,  53.976,  -61.266, 14.881,   -57.707, 18.714,  28.38,   90.02,    -81.341,
    4.73,    15.043,  28.112,  -21.632, 17.957,  -54.565, 3.4593,   9.9326,  22.344,  -13.441, 13.806,   -41.192,
    -3.3992, -14.27,  -34.126, -20.22,  -52.227, 47.332,  -0.21333, 3.7343,  0.1712,  -14.375, 6.2871,   -6.1457,
    58.914,  10.102,  -80.325, -29.337, 70.463,  83.023,  2.7548,   -2.3171, 2.1249,  0.1199,  3.5221,   4.7358,
    -37.473, 59.301,  -188.01, -61.388, 30.908,  205.76,  -1.401,   0.93539, -15.243, -2.0305, 10.865,   10.937,
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
constexpr float kTargetForwardSpeedMaxMps = 2.1f;            ///< 最大前进速度 [m/s+]
constexpr float kTargetForwardSpeedMaxNoScMps = 1.f;         ///< 无超电最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxHighLegMps = 1.2f;     ///< 高腿长模式最大前进速度 [m/s]
constexpr float kTargetForwardSpeedMaxMidLegMps = 1.0f;      ///< F键中腿长模式最大前进速度 [m/s]
constexpr float kTargetSpeedBiasLowLegMps = 0.1f;            ///< 低腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegMps = 0.f;             ///< C键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasMidLegFMps = 0.f;            ///< F键中腿长目标速度偏置 [m/s]
constexpr float kTargetSpeedBiasHighLegMps = -0.f;           ///< 高腿长目标速度偏置 [m/s]
constexpr float kTargetForwardSpeedMaxCtrlCStairMps = 1.2f;  ///< Ctrl+C 上台阶预备模式最大前进速度 [m/s]
constexpr float kTargetSpeedBiasCtrlCStairMps = 0.0f;        ///< Ctrl+C 上台阶预备模式速度偏置 [m/s]
constexpr float kMaxSafeYawRateRadS = 4.0f;                  ///< 摩擦圆最大安全偏航速率 [rad/s]
constexpr float kVxInputDeadbandNorm = 0.05f;                ///< 前进输入死区
constexpr float kVyInputDeadbandNorm = 0.05f;                ///< 平移输入死区
constexpr float kYawFollowRampStepRadS = 0.1f;               ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kYawFollowRampStepRadNoScS = 0.06f;  ///< 偏航跟随角速度斜坡步长（无超电）[(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.15f;  ///< 位置锚定冻结速度阈值 [m/s]
constexpr uint32_t kPositionHoldTimeoutTicks =
    1000U;  ///< 位置锚定超时 [ticks]（斜坡归零后最多等待此周期数，超时强制冻结）

// ==== 落地减速（离地→落地时通过腿摆角辅助减速）====
constexpr float kLandingDecelThetaGain = 0.f;               ///< 落地减速腿摆角增益 [rad/(m/s)]
constexpr float kLandingDecelThetaMaxRad = 0.3f;            ///< 落地减速腿摆角最大偏置 [rad]
constexpr float kLandingDecelThetaRampStepRad = 0.01f;      ///< 落地减速腿摆角每周期斜坡步长 [rad/tick]
constexpr std::uint32_t kLandingDecelOffGroundMinMs = 40U;  ///< 离地最短持续时间（防单帧误判）[ms]
constexpr std::uint32_t kLandingDecelStableDurationMs = 400U;  ///< 落地减速稳定保持时间 [ms]

constexpr float kYawFollowFixedTargetRad = -2.662f;             ///< 偏航跟随固定目标偏置角 [rad]
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;           ///< 偏航跟随侧向目标偏置角 [rad]
constexpr PidGains kYawFollowPid{20.f, 0.0f, 3.f, 8.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置 ====
constexpr float kExpectedThetaLlBiasRadLowLeg = -0.025f;    ///< 低腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadLowLeg = -0.025f;    ///< 低腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadMidLeg = -0.02f;     ///< 中腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadMidLeg = -0.02f;     ///< 中腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaLlBiasRadHighLeg = -0.02f;    ///< 高腿长期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRadHighLeg = -0.02f;    ///< 高腿长期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = -0.033f;           ///< 期望机体俯仰偏置 [rad]
constexpr float kExpectedDisplacementBiasMLowLeg = 0.078f;  ///< 低腿长期望位移偏置 [m]
constexpr float kExpectedDisplacementBiasMMidLeg = 0.08f;   ///< 中腿长期望位移偏置 [m]
constexpr float kExpectedDisplacementBiasMHighLeg = 0.0f;   ///< 高腿长期望位移偏置 [m]

// ==== 速度斜坡参数 ====
constexpr SdotRampParams kSdotRampLowLeg{0.0065f, 0.007f};      ///< 低腿长速度斜坡
constexpr SdotRampParams kSdotRampMidLeg{0.0035f, 0.007f};      ///< 中腿长速度斜坡(C 键触发)
constexpr SdotRampParams kSdotRampMidLegF{0.0045f, 0.006f};     ///< 中腿长速度斜坡（F 键触发）
constexpr SdotRampParams kSdotRampHighLeg{0.006f, 0.005f};      ///< 高腿长速度斜坡
constexpr SdotRampParams kSdotRampCtrlCStair{0.0045f, 0.006f};  ///< Ctrl+C 上台阶预备模式速度斜坡

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
