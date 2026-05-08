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
  float kp;       ///< 比例增益
  float ki;       ///< 积分增益
  float kd;       ///< 微分增益
  float max_out;  ///< 输出限幅
  float max_iout; ///< 积分输出限幅
};

// ══════════════════════════════════════════════════════════════════════════════
// 三变体完全相同的公共参数
// ══════════════════════════════════════════════════════════════════════════════
namespace common {

constexpr float kPi = 3.14159265358979323846f;

// ── 系统级参数 ──
namespace main {
inline constexpr float kControlLoopFrequencyHz = 500.0f;  ///< 主控制循环频率
}  // namespace main

namespace globals {
constexpr double kJointCanTxLimitHz = 4000.0;   ///< 关节电机 CAN 发送频率上限
constexpr double kWheelCanTxLimitHz = 4000.0;   ///< 轮毂电机 CAN 发送频率上限
constexpr double kGimbalCanTxLimitHz = 4000.0;  ///< 云台电机 CAN 发送频率上限

constexpr std::size_t kDr16UartRxBufferSize = 18;     ///< DR16 遥控器串口接收缓冲区大小
constexpr std::size_t kImuUartRxBufferSize = 518;     ///< IMU 串口接收缓冲区大小
constexpr std::size_t kRefereeUartRxBufferSize = 256; ///< 裁判系统串口接收缓冲区大小
}  // namespace globals

namespace gimbal {
constexpr float kDefaultDtS = 0.002f;                  ///< 云台控制默认周期 [s]
constexpr float kDmTorqueLimitNm = 10.0f;              ///< DM 电机力矩上限 [Nm]
constexpr float kPitchGravityCompensationNm = 1.3f;    ///< 俯仰重力补偿力矩 [Nm]
}  // namespace gimbal

// ── 底盘物理/机械参数 ──
namespace chassis {
constexpr float kControlDtS = 0.002f;  ///< 底盘控制周期 [s]（500Hz）

// -- 腿部连杆参数 --
constexpr float kLegL1M = 0.215f;  ///< 五连杆主动杆长度 [m]
constexpr float kLegL2M = 0.254f;  ///< 五连杆从动杆长度 [m]

// -- 弹簧模型参数（用于计算关节等效弹簧补偿力矩）--
constexpr float kSpringModelA = 1082.0f;   ///< 弹簧模型系数 A
constexpr float kSpringModelB = 1070.0f;   ///< 弹簧模型系数 B
constexpr float kSpringModelC = 404.0f;    ///< 弹簧模型系数 C
constexpr float kSpringModelD = 177.0f;    ///< 弹簧模型系数 D
constexpr float kSpringPhaseDivisor = 18.0f;  ///< 弹簧相位除数

// -- 质量/惯量/重力 --
constexpr float kLegMassKg = 2.3f;      ///< 单条腿质量 [kg]
constexpr float kGravityMps2 = 9.81f;   ///< 重力加速度 [m/s²]

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

// -- 起立/姿态安全/倒地恢复 --
constexpr float kStandupThetaThresholdRad = 0.85f;  ///< 起立完成判定：双腿摆角绝对值低于此值后允许轮端输出 [rad]
constexpr float kPostureRollMinRad = -0.5f;         ///< 横滚角安全下限 [rad]
constexpr float kPostureRollMaxRad = 0.5f;          ///< 横滚角安全上限 [rad]
constexpr float kPostureThetaBMinRad = -0.8f;       ///< 机体俯仰角安全下限 [rad]（变体可覆盖）
constexpr float kPostureThetaBMaxRad = 0.8f;        ///< 机体俯仰角安全上限 [rad]（变体可覆盖）
constexpr float kPostureThetaLegMinRad = -0.8f;     ///< 腿摆角安全下限 [rad]
constexpr float kPostureThetaLegMaxRad = 1.4f;      ///< 腿摆角安全上限 [rad]
constexpr float kLegRecoverThetaDotTarget = -2.0f;  ///< 倒地恢复时腿摆角速度目标 [rad/s]（负号表示前摆方向）
constexpr float kLegRecoverZeroTorqueMinRad = 0.0f; ///< 倒地恢复零力矩区间下限 [rad]
constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f; ///< 倒地恢复零力矩区间上限 [rad]

// -- 离地检测 --
constexpr float kOffGroundSupportForceThresholdN = 10.0f;  ///< 支撑力低于此值判定为离地 [N]
}  // namespace chassis

// ── 控制环公共参数（遥控器映射、云台归中判稳、小陀螺触发）──
namespace control_loop {
/// @brief 纵向速度斜坡参数（加速/制动步长分离）
struct SdotRampParams {
  float accel_step;   ///< 加速步长 [(m/s)/周期]
  float brake_step;   ///< 制动步长 [(m/s)/周期]
};

// -- 小陀螺触发阈值（DR16 拨轮值）--
constexpr std::int16_t kWheelSpinThreshold = 220;     ///< 拨轮超过此值触发小陀螺保持
constexpr std::int16_t kWheelActionThreshold = 320;   ///< 拨轮回中后快速负推超过此值触发跳跃
constexpr std::int16_t kWheelCenterThreshold = 80;    ///< 拨轮归中阈值（绝对值小于此值视为归中，重新就绪）

constexpr float kControlLoopDtS = 0.002f;             ///< 控制环周期 [s]

// -- 摇杆/键鼠输入归一化 --
constexpr std::int16_t kDr16AxisMaxAbs = 660;         ///< DR16 摇杆轴最大绝对值（用于归一化到 [-1,1]）
constexpr float kRcStickMax = 660.0f;                 ///< RC 摇杆最大值（用于积分目标速率计算）
constexpr float kTcMouseMax = 200.0f;                 ///< 图传鼠标增量最大值（用于积分目标速率计算）
constexpr float kRcYawRateMaxRadS = -2.5f;            ///< RC 摇杆满偏时偏航积分速率 [rad/s]
constexpr float kRcPitchRateMaxRadS = 1.5f;           ///< RC 摇杆满偏时俯仰积分速率 [rad/s]
constexpr float kTcMouseYawRateMaxRadS = -2.0f;       ///< 图传鼠标满偏时偏航积分速率 [rad/s]
constexpr float kTcMousePitchRateMaxRadS = 1.0f;      ///< 图传鼠标满偏时俯仰积分速率 [rad/s]
constexpr float kPitchTargetMinRad = -0.35f;          ///< RC 积分俯仰目标下限 [rad]
constexpr float kPitchTargetMaxRad = 0.25f;           ///< RC 积分俯仰目标上限 [rad]

// -- 云台启动归中判稳 --
constexpr float kGimbalStartupYawAlignErrorRad = 0.04f;      ///< 归中完成位置误差阈值 [rad]
constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;       ///< 归中完成速度阈值 [rad/s]
constexpr std::uint32_t kGimbalStartupYawAlignStableTicks = 50U;  ///< 归中判稳所需连续满足周期数

// -- 偏航就绪判稳（允许底盘纵向驱动前确认偏航跟踪到位）--
constexpr float kYawFollowDriveReadyErrorRad = 0.04f;       ///< 偏航就绪位置误差阈值 [rad]
constexpr float kYawFollowDriveReadyVelRadS = 0.25f;        ///< 偏航就绪速度阈值 [rad/s]
constexpr std::uint32_t kYawFollowDriveReadyStableTicks = 50U;  ///< 偏航就绪判稳所需连续周期数
}  // namespace control_loop

namespace actuators {
constexpr float kWheelCurrentClampAbs = 16000.0f;  ///< 轮电机电流限幅绝对值
}  // namespace actuators

// ── 图传 CAN 桥通信 ──
namespace remote_control_can_bridge {
constexpr std::uint16_t kRxStdIdA = 0x110;   ///< CAN 帧 A 标准 ID（键鼠/云台 IMU 数据）
constexpr std::uint16_t kRxStdIdB = 0x111;   ///< CAN 帧 B 标准 ID
constexpr std::uint16_t kRxStdIdC = 0x112;   ///< CAN 帧 C 标准 ID（欧拉角）
constexpr std::size_t kPayloadSizeA = 8U;    ///< 帧 A 数据长度
constexpr std::size_t kPayloadSizeB = 8U;    ///< 帧 B 数据长度
constexpr std::size_t kPayloadSizeC = 8U;    ///< 帧 C 数据长度
}  // namespace remote_control_can_bridge

// ── 状态估计 ──
namespace state_estimator {
constexpr float kDefaultDtS = 0.002f;                     ///< 估计器默认周期 [s]
constexpr float kDefaultExpectedSdotMps = 0.05f;          ///< 默认期望纵向速度 [m/s]
constexpr float kLegL1M = 0.215f;                         ///< 五连杆主动杆长度 [m]
constexpr float kLegL2M = 0.254f;                         ///< 五连杆从动杆长度 [m]
constexpr float kWheelRadiusM = 0.0575f;                  ///< 驱动轮半径 [m]
constexpr float kWheelReductionRatio = 17.0f / 268.0f;    ///< 轮电机到车轮的速度换算比
constexpr float kMaxValidSpeedMps = 8.0f;                 ///< 速度融合可信上限 [m/s]
constexpr float kThetaDotFilterCutoffHz = 8.0f;           ///< 腿摆角速度低通滤波截止频率 [Hz]

// -- IMU 加速度融合 --
constexpr float kImuAccelFilterSampleHz = 500.0f;         ///< 加速度低通滤波器采样频率 [Hz]
constexpr float kImuAccelFilterCutoffHz = 10.0f;          ///< 加速度低通滤波器截止频率 [Hz]
constexpr std::uint32_t kAccelBiasInitSamples = 1500U;    ///< 加速度零偏估计所需初始样本数
constexpr float kAccelZeroWheelSpeedThresholdMps = 0.02f; ///< 轮速零速判定阈值 [m/s]
constexpr float kAccelZeroHighThresholdMps2 = 0.5f;       ///< 加速度零偏校准上阈值 [m/s²]
constexpr float kAccelZeroLowThresholdMps2 = 0.2f;        ///< 加速度零偏校准下阈值 [m/s²]

// -- 卡尔曼滤波器矩阵（2 状态：速度、加速度）--
constexpr float kKalmanMinVariance = 1e-5f;
constexpr float kThetaPiHalf = 1.57079632679489661923f;
constexpr std::array<float, 4> kKalmanF{1.0f, kDefaultDtS, 0.0f, 1.0f};    ///< 状态转移矩阵 F
constexpr std::array<float, 4> kKalmanQ{0.0005f, 0.0f, 0.0f, 0.04f};       ///< 过程噪声协方差 Q
constexpr std::array<float, 4> kKalmanR{0.5f, 0.0f, 0.0f, 2.0f};           ///< 观测噪声协方差 R
constexpr std::array<float, 4> kKalmanP{10.0f, 0.0f, 0.0f, 10.0f};         ///< 初始估计协方差 P
constexpr std::array<float, 4> kKalmanH{1.0f, 0.0f, 0.0f, 1.0f};           ///< 观测矩阵 H
}  // namespace state_estimator

// ── 腿部运动学 ──
namespace leg_kinematics {
constexpr float kDefaultDtS = 0.002f;  ///< 运动学默认解算周期 [s]
constexpr float kMinSin = 1e-5f;       ///< 正弦值下限（避免除零奇异）
constexpr float kMinLen = 1e-5f;       ///< 长度下限（避免除零奇异）
}  // namespace leg_kinematics

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

constexpr float kPitchMinRad = -0.2f;  ///< 俯仰角下限 [rad]
constexpr float kPitchMaxRad = 0.25f;  ///< 俯仰角上限 [rad]

constexpr PidGains kYawPositionPid{24.0f, 0.0f, 0.0f, 1000.0f, 1.0f};    ///< 偏航位置 PID
constexpr PidGains kYawSpeedPid{1.f, 0.0f, 0.0f, 10.0f, 0.4f};           ///< 偏航速度 PID
constexpr PidGains kPitchPositionPid{25.0f, 0.0f, 0.0f, 1000.0f, 0.4f};  ///< 俯仰位置 PID
constexpr PidGains kPitchSpeedPid{2.f, 0.0f, 0.0f, 0.0f, 0.0f};          ///< 俯仰速度 PID
}  // namespace gimbal

// ── 发射机构（Hero：三摩擦轮 + DM 拨盘）──
namespace shoot {
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

inline constexpr int kFrictionWheelCount = 3;                 ///< 摩擦轮数量
inline constexpr float kBoosterZeroPointRad = 0.345f;         ///< 拨盘零位角度 [rad]
inline constexpr float kSegmentAngleRad = kPi / 3.f;          ///< 拨盘分段角度 [rad]
inline constexpr uint16_t kInitDelayTicks = 600;              ///< 初始化延迟周期数
inline constexpr uint16_t kShootDelayTicks = 360;             ///< 发射延迟周期数
inline constexpr float kStallThresholdRad = kPi / 18.f;       ///< 堵转判定角度阈值 [rad]
inline constexpr float kStallFallbackRad = kPi / 90.f;        ///< 堵转回退角度 [rad]
inline constexpr float kFwReadySpeedThresholdRpm = 4000.0f;   ///< 摩擦轮就绪判定转速 [rpm]
inline constexpr float kFwTargetSpeedRpm = 7000.0f;           ///< 摩擦轮目标转速 [rpm]
inline constexpr int16_t kFireDialThreshold = -100;           ///< 发射触发拨轮阈值
inline constexpr PidGains kBoosterPositionPid{60.f, 0.f, 560.f, 24.f, 0.f};   ///< 拨盘位置 PID
inline constexpr PidGains kBoosterSpeedPid{0.3f, 0.f, 0.02f, 6.4f, 0.f};     ///< 拨盘速度 PID
inline constexpr PidGains kFwSpeedPid{30.f, 0.01f, 0.f, 10000.f, 0.f};       ///< 摩擦轮速度 PID
inline constexpr uint16_t kFwMotor1Id = 0x01;  ///< 摩擦轮电机 1 CAN ID
inline constexpr uint16_t kFwMotor2Id = 0x02;  ///< 摩擦轮电机 2 CAN ID
inline constexpr uint16_t kFwMotor3Id = 0x03;  ///< 摩擦轮电机 3 CAN ID
inline constexpr uint16_t kBoosterMasterId = 0x10;  ///< 拨盘主电机 CAN ID
inline constexpr uint16_t kBoosterSlaveId = 0x09;   ///< 拨盘从电机 CAN ID
inline const DmMitSettings kBoosterDmSettings{0x10, 0x09, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
}  // namespace shoot

// ── 底盘状态机 ──
namespace chassis_fsm {

// ==== 上台阶 ====
constexpr float kStairClimbThetaThresholdRad = 0.5f;   ///< 双腿摆角均超过此值触发上台阶 [rad]
constexpr float kStairClimbLegLengthM = 0.16f;         ///< 上台阶目标腿长 [m]
constexpr float kStairClimbThetaTargetRad = 0.2f;      ///< 上台阶目标腿摆角（统一偏置） [rad]
constexpr std::uint32_t kStairClimbDurationMs = 400U;  ///< 上台阶最长持续时间 [ms]
constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.01f;  ///< 腿长到位容差 [m]
constexpr float kStairClimbThetaNearZeroThresholdRad = 0.08f;      ///< 摆角归零判定阈值（上台阶完成） [rad]
constexpr std::uint32_t kStairClimbPitchStableMs = 300U;           ///< 上台阶完成后俯仰稳定等待时间 [ms]

// ==== 倒地自启 ====
constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;       ///< 倒地确认时间（持续倒地超过此值进入自启） [ms]
constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;  ///< 自启超时（超时后强制停机） [ms]

// ==== 跳跃 ====
constexpr std::uint32_t kJumpPrepMs = 450U;        ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpPushMaxMs = 1000U;    ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpRecoverMs = 450U;     ///< 跳跃回收阶段持续时间 [ms]
constexpr float kJumpPrepLegLengthM = 0.13f;       ///< 跳跃预备阶段目标腿长 [m]（蓄力收腿）
constexpr float kJumpPushLegLengthM = 0.4f;        ///< 跳跃蹬伸阶段目标腿长 [m]（爆发推地）
constexpr float kJumpRecoverLegLengthM = 0.16f;    ///< 跳跃回收阶段目标腿长 [m]（落地缓冲）
constexpr float kJumpPushReachedLegLengthM = 0.365f;  ///< 蹬伸到位判定腿长 [m]（到达此值提前结束 push）

// ==== 基本运动（腿长档位）====
constexpr float kLowLegLengthM = 0.15f;    ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.22f;    ///< 中腿长档位目标腿长 [m]
constexpr float kHighLegLengthM = 0.34f;   ///< 高腿长档位目标腿长 [m]
constexpr float kLegLengthRampTimeS = 0.5f;  ///< 腿长切换斜坡时间 [s]（从低到高腿长的过渡时间）
}  // namespace chassis_fsm

// ── 底盘控制 ──
namespace chassis {
using namespace common::chassis;

// ==== 物理参数 ====
constexpr float kBodyMassKg = 24.0f;            ///< 机体质量 [kg]
constexpr float kSpringTorqueScale = 90.0f;     ///< 弹簧力矩缩放系数（调参用）

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.052f;  ///< 横滚平衡目标角 [rad]
constexpr float kPostureThetaBMinRad = -0.8f;    ///< 机体俯仰安全下限 [rad]
constexpr float kPostureThetaBMaxRad = 0.8f;     ///< 机体俯仰安全上限 [rad]

// ==== 基本运动（LQR 增益矩阵 — 40 组多项式系数）====
/// 由 MATLAB 离线拟合得到，p(l_l, l_r) = p00 + p10*l_l + p01*l_r + p20*l_l² + p11*l_l*l_r + p02*l_r²
/// 共 40 行，对应 4x10 增益矩阵 K 的 40 个元素（按行主序展平）
/// 每行 6 个系数：[p00, p10, p01, p20, p11, p02]
constexpr std::array<float, 240> kCtrlP{
    -0.83306, -5.1702, 3.9639,  7.1986,   -2.8365,  -2.9774,  -3.2283,  -14.938,  14.569,   23.723,   -14.737, -9.9482,
    -0.81055, 2.863,   -1.089,  -3.4804,  0.83266,  1.399,    -2.5853,  9.3913,   -3.7934,  -11.082,  2.2507,  5.0455,
    -12.3,    -78.693, 11.803,  89.993,   -2.4656,  -14.814,  -0.64798, -4.0995,  1.5913,   -2.5703,  1.151,   -1.8874,
    -4.4374,  10.82,   -10.556, -11.228,  13.601,   4.6118,   -0.34716, -0.16418, -0.53572, 1.7816,   -4.0662, 0.49091,
    -25.277,  37.872,  32.004,  -0.75349, -37.943,  -27.074,  -2.4722,  2.7258,   5.0256,   1.1312,   -5.5857, -4.5784,
    -0.83306, 3.9639,  -5.1702, -2.9774,  -2.8365,  7.1986,   -3.2283,  14.569,   -14.938,  -9.9482,  -14.737, 23.723,
    0.81055,  1.089,   -2.863,  -1.399,   -0.83266, 3.4804,   2.5853,   3.7934,   -9.3913,  -5.0455,  -2.2507, 11.082,
    -4.4374,  -10.556, 10.82,   4.6118,   13.601,   -11.228,  -0.34716, -0.53572, -0.16418, 0.49091,  -4.0662, 1.7816,
    -12.3,    11.803,  -78.693, -14.814,  -2.4656,  89.993,   -0.64798, 1.5913,   -4.0995,  -1.8874,  1.151,   -2.5703,
    -25.277,  32.004,  37.872,  -27.074,  -37.943,  -0.75349, -2.4722,  5.0256,   2.7258,   -4.5784,  -5.5857, 1.1312,
    2.0588,   2.5769,  -7.0205, -12.534,  11.718,   3.3683,   7.4556,   6.1892,   -26.658,  -40.376,  46.683,  12.73,
    -0.72175, -4.466,  -2.225,  8.0066,   -2.6818,  3.3191,   -2.3077,  -14.775,  -6.7921,  26.093,   -8.7142, 9.9612,
    63.482,   -113.17, 15.872,  102.99,   47.489,   -33.411,  2.5342,   -1.9481,  -0.4535,  5.6629,   4.0161,  -1.2781,
    -3.8787,  -37.776, -12.034, 56.789,   -33.164,  -16.399,  -0.2925,  -1.3576,  2.4025,   -0.84556, 1.1223,  -8.8394,
    -47.075,  -230.67, 78.522,  270.91,   30.395,   -95.402,  -2.3876,  -16.763,  5.0598,   15.783,   6.98,    -6.8333,
    2.0588,   -7.0205, 2.5769,  3.3683,   11.718,   -12.534,  7.4556,   -26.658,  6.1892,   12.73,    46.683,  -40.376,
    0.72175,  2.225,   4.466,   -3.3191,  2.6818,   -8.0066,  2.3077,   6.7921,   14.775,   -9.9612,  8.7142,  -26.093,
    -3.8787,  -12.034, -37.776, -16.399,  -33.164,  56.789,   -0.2925,  2.4025,   -1.3576,  -8.8394,  1.1223,  -0.84556,
    63.482,   15.872,  -113.17, -33.411,  47.489,   102.99,   2.5342,   -0.4535,  -1.9481,  -1.2781,  4.0161,  5.6629,
    -47.075,  78.522,  -230.67, -95.402,  30.395,   270.91,   -2.3876,  5.0598,   -16.763,  -6.8333,  6.98,    15.783,
};

// ==== 基本运动（PID 增益）====
constexpr PidGains kLeftL0Pid{8000.0f, 0.15f, 70000.0f, 170.0f, 30.0f};   ///< 左腿腿长 PID（常规）
constexpr PidGains kRightL0Pid{8000.0f, 0.15f, 70000.0f, 170.0f, 30.0f};  ///< 右腿腿长 PID（常规）
constexpr PidGains kRollPid{600.0f, 0.0f, 200.0f, 180.0f, 0.0f};          ///< 横滚平衡 PID

// ==== 跳跃（PID 增益）====
constexpr PidGains kLeftL0PidJumpTwo{8000.0f, 0.0f, 70000.0f, 250.0f, 0.0f};    ///< 左腿蹬伸 PID（JumpPush）
constexpr PidGains kRightL0PidJumpTwo{8000.0f, 0.0f, 70000.0f, 250.0f, 0.0f};   ///< 右腿蹬伸 PID（JumpPush）
constexpr PidGains kLeftL0PidJumpThree{6500.0f, 0.15f, 60000.0f, 190.0f, 30.0f}; ///< 左腿回收 PID（JumpRecover）
constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 60000.0f, 190.0f, 30.0f};///< 右腿回收 PID（JumpRecover）

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};   ///< 左腿摆角速度 PID（倒地恢复用）
constexpr PidGains kRightLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};  ///< 右腿摆角速度 PID（倒地恢复用）
}  // namespace chassis

// ── 控制环 ──
namespace control_loop {
using namespace common::control_loop;

// ==== 基本运动 ====
constexpr float kTargetForwardSpeedMaxMps = 1.8f;       ///< 最大前进速度 [m/s]
constexpr float kVxInputDeadbandNorm = 0.1f;             ///< 前进输入死区（归一化值，低于此忽略）
constexpr float kVyInputDeadbandNorm = 0.1f;             ///< 平移输入死区（归一化值）
constexpr float kYawFollowRampStepRadS = 0.05f;          ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.3f; ///< 位置锚定冻结速度阈值 [m/s]（车速低于此值时锁定位置）
constexpr float kYawFollowFixedTargetRad = 1.216f;       ///< 偏航跟随固定目标偏置角 [rad]（前进方向）
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;    ///< 偏航跟随侧向目标偏置角 [rad]（±π/2）
constexpr PidGains kYawFollowPid{8.0f, 0.0f, 1.f, 4.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置（腿摆角/机体俯仰）====
constexpr float kExpectedThetaLlBiasRad = -0.12f;   ///< 期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRad = -0.12f;   ///< 期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = -0.123f;   ///< 期望机体俯仰偏置 [rad]

// ==== 速度斜坡参数（按腿长档位分级）====
constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.008f};   ///< 低腿长速度斜坡（加速/制动步长）
constexpr SdotRampParams kSdotRampMidLeg{0.006f, 0.003f};  ///< 中腿长速度斜坡
constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f}; ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.005f;    ///< 小陀螺偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS = 6.0f;     ///< 小陀螺目标自旋角速度 [rad/s]
constexpr float kSpinTranslationGain = 1.0f;      ///< 小陀螺平移增益（将云台系前进指令投影到车体系的比例）
constexpr float kSpinThetaLlBiasRad = 0.0f;       ///< 小陀螺时左腿摆角偏置 [rad]
}  // namespace control_loop

namespace actuators {
using namespace common::actuators;

constexpr float kLeftWheelTorqueToCurrent = 2300.0f;   ///< 左轮力矩→电流转换系数
constexpr float kRightWheelTorqueToCurrent = 2300.0f;  ///< 右轮力矩→电流转换系数
}  // namespace actuators

// ── 状态估计（关节零位标定）──
namespace state_estimator {
using namespace common::state_estimator;

constexpr float kLeftPhi1OffsetRad = -0.05f + M_PI;   ///< 左腿前关节零位偏移 [rad]
constexpr float kLeftPhi4OffsetRad = -0.59 + 0.07f;   ///< 左腿后关节零位偏移 [rad]
constexpr float kRightPhi1OffsetRad = 3.04 + M_PI;    ///< 右腿前关节零位偏移 [rad]
constexpr float kRightPhi4OffsetRad = -2.17;          ///< 右腿后关节零位偏移 [rad]
}  // namespace state_estimator

namespace leg_kinematics {
using namespace common::leg_kinematics;
}
namespace remote_control_can_bridge {
using namespace common::remote_control_can_bridge;
}
namespace main {
using namespace common::main;
}

// ── 自瞄通信 ──
namespace aimbot {
constexpr uint8_t kRobotId = 1U;                ///< 机器人 ID（裁判系统回退值）
constexpr float kBulletSpeedMps = 11.8f;        ///< 弹速 [m/s]（裁判系统回退值）
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
inline constexpr int kFrictionWheelCount = 2;                ///< 摩擦轮数量
constexpr float kFricSpeedTargetRpm = 6900.0f;               ///< 摩擦轮目标转速 [rpm]
constexpr PidGains kFricSpeedPid{20.0f, 1.0f, 0.0f, 16000.0f, 2000.0f};  ///< 摩擦轮速度 PID
constexpr PidGains kDialSpeedPid{15.0f, 0.f, 0.2f, 2000.0f, 0.0f};       ///< 拨盘速度 PID
constexpr PidGains kDialPositionPid{8.0f, 0.f, 0.0f, 12000.0f, 0.0f};    ///< 拨盘位置 PID
constexpr int16_t kDialFireThreshold = -600;                 ///< 发射触发拨轮阈值
constexpr float kShootFrequencyHz = 10.0f;                   ///< 发射频率 [Hz]
}  // namespace shoot

// ── 底盘状态机 ──
namespace chassis_fsm {

// ==== 上台阶 ====
constexpr float kStairClimbThetaThresholdRad = 0.5f;    ///< 双腿摆角均超过此值触发上台阶 [rad]
constexpr float kStairClimbLegLengthM = 0.14f;          ///< 上台阶目标腿长 [m]
constexpr float kStairClimbThetaTargetRad = 0.2f;       ///< 上台阶目标腿摆角（统一偏置） [rad]
constexpr std::uint32_t kStairClimbDurationMs = 3000U;  ///< 上台阶最长持续时间 [ms]
constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.03f;  ///< 腿长到位容差 [m]
constexpr float kStairClimbThetaNearZeroThresholdRad = 0.1f;       ///< 摆角归零判定阈值（上台阶完成） [rad]
constexpr std::uint32_t kStairClimbPitchStableMs = 1000U;          ///< 上台阶完成后俯仰稳定等待时间 [ms]

// ==== 倒地自启 ====
constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;       ///< 倒地确认时间 [ms]
constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;  ///< 自启超时 [ms]

// ==== 跳跃 ====
constexpr std::uint32_t kJumpPrepMs = 200U;        ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpPushMaxMs = 1000U;    ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpRecoverMs = 250U;     ///< 跳跃回收阶段持续时间 [ms]
constexpr float kJumpPrepLegLengthM = 0.13f;       ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpPushLegLengthM = 0.22f;       ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpRecoverLegLengthM = 0.20f;    ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpPushReachedLegLengthM = 0.21f;  ///< 蹬伸到位判定腿长 [m]

// ==== 基本运动（腿长档位）====
constexpr float kLowLegLengthM = 0.127f;   ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.18f;    ///< 中腿长档位目标腿长 [m]
constexpr float kHighLegLengthM = 0.3f;    ///< 高腿长档位目标腿长 [m]
constexpr float kLegLengthRampTimeS = 0.8f;  ///< 腿长切换斜坡时间 [s]
}  // namespace chassis_fsm

// ── 底盘控制 ──
namespace chassis {
using namespace common::chassis;

// ==== 物理参数 ====
constexpr float kBodyMassKg = 22.0f;             ///< 机体质量 [kg]
constexpr float kSpringTorqueScale = 90.0f;      ///< 弹簧力矩缩放系数

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.003f;  ///< 横滚平衡目标角 [rad]
constexpr float kPostureThetaBMinRad = -0.7f;    ///< 机体俯仰安全下限 [rad]
constexpr float kPostureThetaBMaxRad = 0.7f;     ///< 机体俯仰安全上限 [rad]

// ==== 基本运动（LQR 增益矩阵）====
constexpr std::array<float, 240> kCtrlP{
    -3.5611,  -30.773,  24.204,  49.27,  -22.027,  -19.477,
    -5.9313,  -36.889,  34.858,  62.344,  -37.433,  -26.905,
    -0.38518,  1.8237,  -0.73941,  -1.668,  -0.18375,  1.2356,
    -1.5105,  7.3371,  -3.1349,  -6.4854,  -1.0271,  5.2894,
    -10.956,  -78.834,  16.086,  69.139,  7.6072,  -22.308,
    -0.72054,  -6.7038,  3.0018,  -4.3907,  2.8223,  -3.9011,
    -3.6446,  -0.68148,  -3.3396,  20.145,  -20.964,  2.93,
    -0.33916,  -2.0985,  0.045752,  7.2851,  -12.66,  1.5759,
    -23.053,  29.876,  32.954,  4.3946,  -33.814,  -31.656,
    -2.7417,  1.6265,  6.9444,  3.506,  -6.8174,  -6.9601,
    -3.5611,  24.204,  -30.773,  -19.477,  -22.027,  49.27,
    -5.9313,  34.858,  -36.889,  -26.905,  -37.433,  62.344,
    0.38518,  0.73941,  -1.8237,  -1.2356,  0.18375,  1.668,
    1.5105,  3.1349,  -7.3371,  -5.2894,  1.0271,  6.4854,
    -3.6446,  -3.3396,  -0.68148,  2.93,  -20.964,  20.145,
    -0.33916,  0.045752,  -2.0985,  1.5759,  -12.66,  7.2851,
    -10.956,  16.086,  -78.834,  -22.308,  7.6072,  69.139,
    -0.72054,  3.0018,  -6.7038,  -3.9011,  2.8223,  -4.3907,
    -23.053,  32.954,  29.876,  -31.656,  -33.814,  4.3946,
    -2.7417,  6.9444,  1.6265,  -6.9601,  -6.8174,  3.506,
    9.1291,  -1.9062,  -17.612,  -48.654,  47.114,  13.514,
    13.502,  -2.1689,  -33.382,  -67.639,  77.333,  24.071,
    -0.3831,  -2.6376,  -0.64534,  4.511,  -1.1575,  1.2208,
    -1.5003,  -10.756,  -2.3566,  18.277,  -4.8971,  4.5585,
    47.384,  -64.757,  3.9902,  32.704,  44.328,  -16.67,
    2.4081,  2.0579,  -2.1393,  -5.4513,  8.0842,  0.77762,
    -2.1265,  -25.244,  -1.4489,  20.509,  -27.359,  9.4924,
    -0.11488,  0.24722,  3.457,  -6.3006,  -0.48196,  -2.5744,
    -37.196,  -184.77,  55.678,  216.22,  38.939,  -81.1,
    -1.4163,  -16.78,  2.7189,  14.63,  10.281,  -5.6197,
    9.1291,  -17.612,  -1.9062,  13.514,  47.114,  -48.654,
    13.502,  -33.382,  -2.1689,  24.071,  77.333,  -67.639,
    0.3831,  0.64534,  2.6376,  -1.2208,  1.1575,  -4.511,
    1.5003,  2.3566,  10.756,  -4.5585,  4.8971,  -18.277,
    -2.1265,  -1.4489,  -25.244,  9.4924,  -27.359,  20.509,
    -0.11488,  3.457,  0.24722,  -2.5744,  -0.48196,  -6.3006,
    47.384,  3.9902,  -64.757,  -16.67,  44.328,  32.704,
    2.4081,  -2.1393,  2.0579,  0.77762,  8.0842,  -5.4513,
    -37.196,  55.678,  -184.77,  -81.1,  38.939,  216.22,
    -1.4163,  2.7189,  -16.78,  -5.6197,  10.281,  14.63,
};

// ==== 基本运动（PID 增益）====
constexpr PidGains kLeftL0Pid{7500.0f, 0.04f, 90000.0f, 170.0f, 10.0f};   ///< 左腿腿长 PID（常规）
constexpr PidGains kRightL0Pid{8500.0f, 0.04f, 90000.0f, 170.0f, 10.0f};  ///< 右腿腿长 PID（常规）
constexpr PidGains kRollPid{800.0f, 0.0f, 200.0f, 180.0f, 0.0f};          ///< 横滚平衡 PID

// ==== 跳跃（PID 增益）====
constexpr PidGains kLeftL0PidJumpTwo{7000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};    ///< 左腿蹬伸 PID
constexpr PidGains kRightL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};   ///< 右腿蹬伸 PID
constexpr PidGains kLeftL0PidJumpThree{7500.0f, 0.15f, 50000.0f, 170.0f, 30.0f}; ///< 左腿回收 PID
constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};///< 右腿回收 PID

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};   ///< 左腿摆角速度 PID
constexpr PidGains kRightLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};  ///< 右腿摆角速度 PID
}  // namespace chassis

// ── 控制环 ──
namespace control_loop {
using namespace common::control_loop;

// ==== 基本运动 ====
constexpr float kTargetForwardSpeedMaxMps = 2.1f;        ///< 最大前进速度 [m/s]
constexpr float kVxInputDeadbandNorm = 0.05f;             ///< 前进输入死区
constexpr float kVyInputDeadbandNorm = 0.05f;             ///< 平移输入死区
constexpr float kYawFollowRampStepRadS = 0.05f;           ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.35f; ///< 位置锚定冻结速度阈值 [m/s]
constexpr float kYawFollowFixedTargetRad = 0.f;           ///< 偏航跟随固定目标偏置角 [rad]
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;     ///< 偏航跟随侧向目标偏置角 [rad]
constexpr PidGains kYawFollowPid{10.0f, 0.0f, 2.2f, 10.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置 ====
constexpr float kExpectedThetaLlBiasRad = 0.13f;   ///< 期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRad = 0.13f;   ///< 期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = 0.0f;     ///< 期望机体俯仰偏置 [rad]

// ==== 速度斜坡参数 ====
constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.01f};    ///< 低腿长速度斜坡
constexpr SdotRampParams kSdotRampMidLeg{0.007f, 0.007f};  ///< 中腿长速度斜坡
constexpr SdotRampParams kSdotRampHighLeg{0.005f, 0.005f}; ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.02f;     ///< 小陀螺偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS = 6.0f;     ///< 小陀螺目标自旋角速度 [rad/s]
constexpr float kSpinTranslationGain = 1.0f;      ///< 小陀螺平移增益
constexpr float kSpinThetaLlBiasRad = 0.f;        ///< 小陀螺时左腿摆角偏置 [rad]
}  // namespace control_loop

namespace actuators {
using namespace common::actuators;

constexpr float kLeftWheelTorqueToCurrent = 2500.0f;   ///< 左轮力矩→电流转换系数
constexpr float kRightWheelTorqueToCurrent = 2300.0f;  ///< 右轮力矩→电流转换系数
}  // namespace actuators

// ── 状态估计（关节零位标定）──
namespace state_estimator {
using namespace common::state_estimator;

constexpr float kLeftPhi1OffsetRad = kPi - 2.94f;    ///< 左腿前关节零位偏移 [rad]
constexpr float kLeftPhi4OffsetRad = 0.59f;          ///< 左腿后关节零位偏移 [rad]
constexpr float kRightPhi1OffsetRad = kPi + 2.4f;    ///< 右腿前关节零位偏移 [rad]
constexpr float kRightPhi4OffsetRad = -1.87f;        ///< 右腿后关节零位偏移 [rad]
}  // namespace state_estimator

namespace leg_kinematics {
using namespace common::leg_kinematics;
}
namespace remote_control_can_bridge {
using namespace common::remote_control_can_bridge;
}
namespace main {
using namespace common::main;
}

// ── 自瞄通信 ──
namespace aimbot {
constexpr uint8_t kRobotId = 3U;                ///< 机器人 ID
constexpr float kBulletSpeedMps = 23.0f;        ///< 弹速 [m/s]
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

constexpr PidGains kYawPositionPid{25.0f, 0.0f, 0.05f, 10.0f, 1.0f};    ///< 偏航位置 PID
constexpr PidGains kYawSpeedPid{0.6f, 0.0f, 0.0f, 6.0f, 0.4f};          ///< 偏航速度 PID
constexpr PidGains kPitchPositionPid{26.0f, 0.0f, 0.1f, 10.0f, 0.4f};   ///< 俯仰位置 PID
constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 8.0f, 0.0f};       ///< 俯仰速度 PID
}  // namespace gimbal

// ── 发射机构（双摩擦轮 + M3508 拨盘）──
namespace shoot {
inline constexpr int kFrictionWheelCount = 2;                ///< 摩擦轮数量
constexpr float kFricSpeedTargetRpm = 6000.0f;               ///< 摩擦轮目标转速 [rpm]
constexpr PidGains kFricSpeedPid{20.0f, 1.0f, 0.0f, 16000.0f, 2000.0f};  ///< 摩擦轮速度 PID
constexpr PidGains kDialSpeedPid{10.0f, 0.5f, 0.0f, 16000.0f, 1000.0f};  ///< 拨盘速度 PID
constexpr PidGains kDialPositionPid{5.0f, 0.1f, 0.0f, 1500.0f, 500.0f};  ///< 拨盘位置 PID
constexpr int16_t kDialFireThreshold = -600;                 ///< 发射触发拨轮阈值
constexpr float kShootFrequencyHz = 24.0f;                   ///< 发射频率 [Hz]
}  // namespace shoot

// ── 底盘状态机 ──
namespace chassis_fsm {

// ==== 上台阶 ====
constexpr float kStairClimbThetaThresholdRad = 0.5f;    ///< 双腿摆角均超过此值触发上台阶 [rad]
constexpr float kStairClimbLegLengthM = 0.16f;          ///< 上台阶目标腿长 [m]
constexpr float kStairClimbThetaTargetRad = 0.3f;       ///< 上台阶目标腿摆角 [rad]
constexpr std::uint32_t kStairClimbDurationMs = 250U;   ///< 上台阶最长持续时间 [ms]
constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.03f;  ///< 腿长到位容差 [m]
constexpr float kStairClimbThetaNearZeroThresholdRad = 0.1f;       ///< 摆角归零判定阈值 [rad]
constexpr std::uint32_t kStairClimbPitchStableMs = 2000U;          ///< 上台阶完成后俯仰稳定等待时间 [ms]

// ==== 倒地自启 ====
constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;       ///< 倒地确认时间 [ms]
constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;  ///< 自启超时 [ms]

// ==== 跳跃 ====
constexpr std::uint32_t kJumpPrepMs = 250U;        ///< 跳跃预备阶段持续时间 [ms]
constexpr std::uint32_t kJumpPushMaxMs = 1000U;    ///< 跳跃蹬伸阶段最长持续时间 [ms]
constexpr std::uint32_t kJumpRecoverMs = 250U;     ///< 跳跃回收阶段持续时间 [ms]
constexpr float kJumpPrepLegLengthM = 0.13f;       ///< 跳跃预备阶段目标腿长 [m]
constexpr float kJumpPushLegLengthM = 0.25f;       ///< 跳跃蹬伸阶段目标腿长 [m]
constexpr float kJumpRecoverLegLengthM = 0.20f;    ///< 跳跃回收阶段目标腿长 [m]
constexpr float kJumpPushReachedLegLengthM = 0.25f;  ///< 蹬伸到位判定腿长 [m]

// ==== 基本运动（腿长档位）====
constexpr float kLowLegLengthM = 0.15f;    ///< 低腿长档位目标腿长 [m]
constexpr float kMidLegLengthM = 0.22f;    ///< 中腿长档位目标腿长 [m]
constexpr float kHighLegLengthM = 0.35f;   ///< 高腿长档位目标腿长 [m]
constexpr float kLegLengthRampTimeS = 0.5f;  ///< 腿长切换斜坡时间 [s]
}  // namespace chassis_fsm

// ── 底盘控制 ──
namespace chassis {
using namespace common::chassis;

// ==== 物理参数 ====
constexpr float kBodyMassKg = 18.0f;             ///< 机体质量 [kg]
constexpr float kSpringTorqueScale = 70.0f;      ///< 弹簧力矩缩放系数

// ==== 基本运动（横滚平衡）====
constexpr float kRollBalanceTargetRad = 0.f;     ///< 横滚平衡目标角 [rad]
constexpr float kPostureThetaBMinRad = -0.7f;    ///< 机体俯仰安全下限 [rad]
constexpr float kPostureThetaBMaxRad = 0.7f;     ///< 机体俯仰安全上限 [rad]

// ==== 基本运动（LQR 增益矩阵）====
constexpr std::array<float, 240> kCtrlP{
    -5.9739,  -45.91,   38.502,   78.117,  -40.234,  -28.916, -9.3012,  -49.076,  51.56,    90.193,   -63.065, -36.523,
    -0.47818, 2.6018,   -0.89571, -2.423,  -0.53511, 1.6694,  -1.5338,  8.6937,   -3.2917,  -7.7185,  -2.2948, 6.1694,
    -18.872,  -92.25,   16.559,   88.3,    7.4371,   -22.333, -1.0696,  -8.5995,  3.9464,   -5.0045,  3.0863,  -4.6606,
    -4.1492,  -0.31931, 0.82772,  23.107,  -32.251,  0.89166, -0.41616, -2.7839,  0.035074, 10.114,   -18.192, 3.2817,
    -27.815,  53.336,   31.835,   -28.501, -32.617,  -32.641, -3.3044,  2.8387,   8.2635,   2.7798,   -8.5728, -8.2573,
    -5.9739,  38.502,   -45.91,   -28.916, -40.234,  78.117,  -9.3012,  51.56,    -49.076,  -36.523,  -63.065, 90.193,
    0.47818,  0.89571,  -2.6018,  -1.6694, 0.53511,  2.423,   1.5338,   3.2917,   -8.6937,  -6.1694,  2.2948,  7.7185,
    -4.1492,  0.82772,  -0.31931, 0.89166, -32.251,  23.107,  -0.41616, 0.035074, -2.7839,  3.2817,   -18.192, 10.114,
    -18.872,  16.559,   -92.25,   -22.333, 7.4371,   88.3,    -1.0696,  3.9464,   -8.5995,  -4.6606,  3.0863,  -5.0045,
    -27.815,  31.835,   53.336,   -32.641, -32.617,  -28.501, -3.3044,  8.2635,   2.8387,   -8.2573,  -8.5728, 2.7798,
    8.5972,   -14.199,  -6.2941,  -32.689, 46.288,   2.1471,  11.78,    -13.709,  -20.715,  -46.464,  71.286,  11.826,
    -0.53043, -2.2396,  -0.58831, 3.9821,  -0.90289, 1.2424,  -1.6985,  -7.6658,  -1.7343,  13.525,   -3.4107, 3.844,
    48.892,   -103.3,   4.596,    95.161,  33.702,   -15.583, 2.2885,   -0.63372, -1.4604,  -3.4679,  9.1403,  -0.22252,
    -3.4647,  -18.032,  3.3037,   16.205,  -19.814,  3.1081,  -0.14795, -0.31165, 3.291,    -4.2672,  -1.7654, -1.3225,
    -43.78,   -160.3,   55.04,    207.76,  22.649,   -81.528, -2.0568,  -15.337,  3.7426,   15.224,   8.2198,  -6.9979,
    8.5972,   -6.2941,  -14.199,  2.1471,  46.288,   -32.689, 11.78,    -20.715,  -13.709,  11.826,   71.286,  -46.464,
    0.53043,  0.58831,  2.2396,   -1.2424, 0.90289,  -3.9821, 1.6985,   1.7343,   7.6658,   -3.844,   3.4107,  -13.525,
    -3.4647,  3.3037,   -18.032,  3.1081,  -19.814,  16.205,  -0.14795, 3.291,    -0.31165, -1.3225,  -1.7654, -4.2672,
    48.892,   4.596,    -103.3,   -15.583, 33.702,   95.161,  2.2885,   -1.4604,  -0.63372, -0.22252, 9.1403,  -3.4679,
    -43.78,   55.04,    -160.3,   -81.528, 22.649,   207.76,  -2.0568,  3.7426,   -15.337,  -6.9979,  8.2198,  15.224,
};

// ==== 基本运动（PID 增益）====
constexpr PidGains kLeftL0Pid{8000.0f, 0.15f, 50000.0f, 170.0f, 30.0f};   ///< 左腿腿长 PID（常规）
constexpr PidGains kRightL0Pid{8000.0f, 0.15f, 50000.0f, 170.0f, 30.0f};  ///< 右腿腿长 PID（常规）
constexpr PidGains kRollPid{800.0f, 0.0f, 80.0f, 80.0f, 0.0f};            ///< 横滚平衡 PID

// ==== 跳跃（PID 增益）====
constexpr PidGains kLeftL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};    ///< 左腿蹬伸 PID
constexpr PidGains kRightL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};   ///< 右腿蹬伸 PID
constexpr PidGains kLeftL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f}; ///< 左腿回收 PID
constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};///< 右腿回收 PID

// ==== 倒地自启（腿摆速度 PID）====
constexpr PidGains kLeftLegTurnPid{20.0f, 0.0f, 0.0f, 15.0f, 0.0f};   ///< 左腿摆角速度 PID
constexpr PidGains kRightLegTurnPid{20.0f, 0.0f, 0.0f, 15.0f, 0.0f};  ///< 右腿摆角速度 PID
}  // namespace chassis

// ── 控制环 ──
namespace control_loop {
using namespace common::control_loop;

// ==== 基本运动 ====
constexpr float kTargetForwardSpeedMaxMps = 2.f;          ///< 最大前进速度 [m/s]
constexpr float kVxInputDeadbandNorm = 0.1f;              ///< 前进输入死区
constexpr float kVyInputDeadbandNorm = 0.1f;              ///< 平移输入死区
constexpr float kYawFollowRampStepRadS = 0.05f;           ///< 偏航跟随角速度斜坡步长 [(rad/s)/周期]
constexpr float kPositionFreezeSpeedThresholdMps = 0.4f;  ///< 位置锚定冻结速度阈值 [m/s]
constexpr float kYawFollowFixedTargetRad = -1.72f;        ///< 偏航跟随固定目标偏置角 [rad]
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;     ///< 偏航跟随侧向目标偏置角 [rad]
constexpr PidGains kYawFollowPid{8.2f, 0.0f, 1.2f, 6.0f, 0.0f};  ///< 偏航跟随 PID

// ==== 期望状态偏置 ====
constexpr float kExpectedThetaLlBiasRad = 0.11f;   ///< 期望左腿摆角偏置 [rad]
constexpr float kExpectedThetaLrBiasRad = 0.11f;   ///< 期望右腿摆角偏置 [rad]
constexpr float kExpectedThetaBBiasRad = 0.f;      ///< 期望机体俯仰偏置 [rad]

// ==== 速度斜坡参数 ====
constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.008f};   ///< 低腿长速度斜坡
constexpr SdotRampParams kSdotRampMidLeg{0.006f, 0.003f};  ///< 中腿长速度斜坡
constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f}; ///< 高腿长速度斜坡

// ==== 小陀螺 ====
constexpr float kSpinYawRampStepRadS = 0.05f;     ///< 小陀螺偏航角速度斜坡步长 [(rad/s)/周期]
constexpr float kSpinTargetYawDotRadS = 7.0f;     ///< 小陀螺目标自旋角速度 [rad/s]
constexpr float kSpinTranslationGain = 1.2f;      ///< 小陀螺平移增益
constexpr float kSpinThetaLlBiasRad = 0.01f;      ///< 小陀螺时左腿摆角偏置 [rad]
}  // namespace control_loop

namespace actuators {
using namespace common::actuators;

constexpr float kLeftWheelTorqueToCurrent = 2436.5f;   ///< 左轮力矩→电流转换系数
constexpr float kRightWheelTorqueToCurrent = 2436.5f;  ///< 右轮力矩→电流转换系数
}  // namespace actuators

// ── 状态估计（关节零位标定）──
namespace state_estimator {
using namespace common::state_estimator;

constexpr float kLeftPhi1OffsetRad = -1.50f + M_PI;  ///< 左腿前关节零位偏移 [rad]
constexpr float kLeftPhi4OffsetRad = -1.50f;         ///< 左腿后关节零位偏移 [rad]
constexpr float kRightPhi1OffsetRad = -1.42f + M_PI; ///< 右腿前关节零位偏移 [rad]
constexpr float kRightPhi4OffsetRad = -1.62f;        ///< 右腿后关节零位偏移 [rad]
}  // namespace state_estimator

namespace leg_kinematics {
using namespace common::leg_kinematics;
}
namespace remote_control_can_bridge {
using namespace common::remote_control_can_bridge;
}
namespace main {
using namespace common::main;
}

// ── 自瞄通信 ──
namespace aimbot {
constexpr uint8_t kRobotId = 4U;                ///< 机器人 ID
constexpr float kBulletSpeedMps = 23.0f;        ///< 弹速 [m/s]
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
