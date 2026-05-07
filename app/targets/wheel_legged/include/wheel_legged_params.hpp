#pragma once

#include <array>
#include <cstdint>
#include <librm.hpp>

namespace wheel_legged::params {

#ifndef WHEEL_LEGGED_ROBOT_VARIANT
#define WHEEL_LEGGED_ROBOT_VARIANT 2
#endif

struct PidGains {
  float kp;
  float ki;
  float kd;
  float max_out;
  float max_iout;
};

// ── 三变体完全相同的公共参数 ──
namespace common {

constexpr float kPi = 3.14159265358979323846f;

namespace main {
inline constexpr float kControlLoopFrequencyHz = 500.0f;
}  // namespace main

namespace globals {
constexpr double kJointCanTxLimitHz = 4000.0;
constexpr double kWheelCanTxLimitHz = 4000.0;
constexpr double kGimbalCanTxLimitHz = 4000.0;

constexpr std::size_t kDr16UartRxBufferSize = 18;
constexpr std::size_t kImuUartRxBufferSize = 518;
constexpr std::size_t kRefereeUartRxBufferSize = 256;
}  // namespace globals

namespace gimbal {
constexpr float kDefaultDtS = 0.002f;
constexpr float kDmTorqueLimitNm = 10.0f;
constexpr float kPitchGravityCompensationNm = 1.3f;
}  // namespace gimbal

namespace chassis {
constexpr float kControlDtS = 0.002f;
constexpr float kStandupThetaThresholdRad = 0.85f;  ///< 起立完成判定：双腿 theta 低于此值时允许轮端输出
constexpr float kLegL1M = 0.215f;
constexpr float kLegL2M = 0.254f;
constexpr float kSpringModelA = 1082.0f;
constexpr float kSpringModelB = 1070.0f;
constexpr float kSpringModelC = 404.0f;
constexpr float kSpringModelD = 177.0f;
constexpr float kSpringPhaseDivisor = 18.0f;
constexpr float kLegMassKg = 2.3f;
constexpr float kGravityMps2 = 9.81f;
constexpr float kWheelRadiusM = 0.2025f;
constexpr float kOffGroundSupportForceThresholdN = 10.0f;
constexpr float kPostureRollMinRad = -0.5f;
constexpr float kPostureRollMaxRad = 0.5f;
constexpr float kPostureThetaLegMinRad = -0.8f;
constexpr float kPostureThetaLegMaxRad = 1.4f;
constexpr float kLegRecoverThetaDotTarget = -2.0f;
constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;
constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;

constexpr std::array<float, 24> kEtaLookupLegLengthM{
    0.11f, 0.12f, 0.13f, 0.14f, 0.15f, 0.16f, 0.17f, 0.18f, 0.19f, 0.20f, 0.21f, 0.22f,
    0.23f, 0.24f, 0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.30f, 0.31f, 0.32f, 0.33f, 0.34f,
};

constexpr std::array<float, 24> kEtaLookupLwM{
    0.061990f, 0.067466f, 0.072986f, 0.078550f, 0.084158f, 0.089810f, 0.095506f, 0.101246f,
    0.107030f, 0.112858f, 0.118730f, 0.124646f, 0.130606f, 0.136610f, 0.142658f, 0.148750f,
    0.154886f, 0.161066f, 0.167290f, 0.173558f, 0.179870f, 0.186226f, 0.192626f, 0.199070f,
};
}  // namespace chassis

namespace control_loop {
struct SdotRampParams {
  float accel_step;
  float brake_step;
};

constexpr std::int16_t kWheelSpinThreshold = 220;
constexpr std::int16_t kWheelActionThreshold = 320;
constexpr std::int16_t kWheelCenterThreshold = 80;
constexpr float kControlLoopDtS = 0.002f;
constexpr std::int16_t kDr16AxisMaxAbs = 660;
constexpr float kRcStickMax = 660.0f;
constexpr float kTcMouseMax = 200.0f;
constexpr float kRcYawRateMaxRadS = -2.5f;
constexpr float kRcPitchRateMaxRadS = 1.5f;
constexpr float kTcMouseYawRateMaxRadS = -2.0f;
constexpr float kTcMousePitchRateMaxRadS = 1.0f;
constexpr float kPitchTargetMinRad = -0.35f;
constexpr float kPitchTargetMaxRad = 0.25f;
constexpr float kGimbalStartupYawAlignErrorRad = 0.04f;
constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;
constexpr std::uint32_t kGimbalStartupYawAlignStableTicks = 50U;
constexpr float kYawFollowDriveReadyErrorRad = 0.04f;
constexpr float kYawFollowDriveReadyVelRadS = 0.25f;
constexpr std::uint32_t kYawFollowDriveReadyStableTicks = 50U;
}  // namespace control_loop

namespace actuators {
constexpr float kWheelCurrentClampAbs = 16000.0f;
}  // namespace actuators

namespace remote_control_can_bridge {
constexpr std::uint16_t kRxStdIdA = 0x110;
constexpr std::uint16_t kRxStdIdB = 0x111;
constexpr std::uint16_t kRxStdIdC = 0x112;
constexpr std::size_t kPayloadSizeA = 8U;
constexpr std::size_t kPayloadSizeB = 8U;
constexpr std::size_t kPayloadSizeC = 8U;
}  // namespace remote_control_can_bridge

namespace state_estimator {
constexpr float kDefaultDtS = 0.002f;
constexpr float kDefaultExpectedSdotMps = 0.05f;
constexpr float kLegL1M = 0.215f;
constexpr float kLegL2M = 0.254f;
constexpr float kWheelRadiusM = 0.0575f;
constexpr float kWheelReductionRatio = 17.0f / 268.0f;
constexpr float kMaxValidSpeedMps = 8.0f;
constexpr float kThetaDotFilterCutoffHz = 8.0f;
constexpr float kImuAccelFilterSampleHz = 500.0f;
constexpr float kImuAccelFilterCutoffHz = 10.0f;
constexpr std::uint32_t kAccelBiasInitSamples = 1500U;
constexpr float kAccelZeroWheelSpeedThresholdMps = 0.02f;
constexpr float kAccelZeroHighThresholdMps2 = 0.5f;
constexpr float kAccelZeroLowThresholdMps2 = 0.2f;
constexpr float kKalmanMinVariance = 1e-5f;
constexpr float kThetaPiHalf = 1.57079632679489661923f;

constexpr std::array<float, 4> kKalmanF{1.0f, kDefaultDtS, 0.0f, 1.0f};
constexpr std::array<float, 4> kKalmanQ{0.0005f, 0.0f, 0.0f, 0.04f};
constexpr std::array<float, 4> kKalmanR{0.5f, 0.0f, 0.0f, 2.0f};
constexpr std::array<float, 4> kKalmanP{10.0f, 0.0f, 0.0f, 10.0f};
constexpr std::array<float, 4> kKalmanH{1.0f, 0.0f, 0.0f, 1.0f};
}  // namespace state_estimator

namespace leg_kinematics {
constexpr float kDefaultDtS = 0.002f;
constexpr float kMinSin = 1e-5f;
constexpr float kMinLen = 1e-5f;
}  // namespace leg_kinematics

}  // namespace common

// ── Hero (variant 1) 特殊参数 ──
namespace hero {
using namespace common;

namespace globals {
using namespace common::globals;
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

constexpr std::uint16_t kLeftWheelId = 0x06;
constexpr std::uint16_t kRightWheelId = 0x05;

constexpr std::uint16_t kDmLfMasterId = 0x04;
constexpr std::uint16_t kDmLfSlaveId = 0x03;
constexpr std::uint16_t kDmLbMasterId = 0x06;
constexpr std::uint16_t kDmLbSlaveId = 0x05;
constexpr std::uint16_t kDmRfMasterId = 0x02;
constexpr std::uint16_t kDmRfSlaveId = 0x01;
constexpr std::uint16_t kDmRbMasterId = 0x08;
constexpr std::uint16_t kDmRbSlaveId = 0x07;

const DmMitSettings kDmLfSettings{kDmLfMasterId, kDmLfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmLbSettings{kDmLbMasterId, kDmLbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmRfSettings{kDmRfMasterId, kDmRfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmRbSettings{kDmRbMasterId, kDmRbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
}  // namespace globals

namespace gimbal {
using namespace common::gimbal;
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

const DmMitSettings kPitchMotorSettings{0x13, 0x12, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
const DmMitSettings kYawMotorSettings{0x21, 0x11, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};

constexpr float kPitchMinRad = -0.2f;
constexpr float kPitchMaxRad = 0.25f;

constexpr PidGains kYawPositionPid{24.0f, 0.0f, 0.0f, 1000.0f, 1.0f};
constexpr PidGains kYawSpeedPid{1.f, 0.0f, 0.0f, 10.0f, 0.4f};
constexpr PidGains kPitchPositionPid{25.0f, 0.0f, 0.0f, 1000.0f, 0.4f};
constexpr PidGains kPitchSpeedPid{2.f, 0.0f, 0.0f, 0.0f, 0.0f};
}  // namespace gimbal

namespace shoot {
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

inline constexpr int kFrictionWheelCount = 3;
inline constexpr float kBoosterZeroPointRad = 0.345f;
inline constexpr float kSegmentAngleRad = kPi / 3.f;
inline constexpr uint16_t kInitDelayTicks = 600;
inline constexpr uint16_t kShootDelayTicks = 360;
inline constexpr float kStallThresholdRad = kPi / 18.f;
inline constexpr float kStallFallbackRad = kPi / 90.f;
inline constexpr float kFwReadySpeedThresholdRpm = 4000.0f;
inline constexpr float kFwTargetSpeedRpm = 7000.0f;
inline constexpr int16_t kFireDialThreshold = -100;
inline constexpr PidGains kBoosterPositionPid{60.f, 0.f, 560.f, 24.f, 0.f};
inline constexpr PidGains kBoosterSpeedPid{0.3f, 0.f, 0.02f, 6.4f, 0.f};
inline constexpr PidGains kFwSpeedPid{30.f, 0.01f, 0.f, 10000.f, 0.f};
inline constexpr uint16_t kFwMotor1Id = 0x01;
inline constexpr uint16_t kFwMotor2Id = 0x02;
inline constexpr uint16_t kFwMotor3Id = 0x03;
inline constexpr uint16_t kBoosterMasterId = 0x10;
inline constexpr uint16_t kBoosterSlaveId = 0x09;
inline const DmMitSettings kBoosterDmSettings{0x10, 0x09, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
}  // namespace shoot

namespace chassis_fsm {
constexpr std::uint32_t kJumpPrepMs = 450U;
constexpr std::uint32_t kJumpPushMaxMs = 1000U;
constexpr std::uint32_t kJumpRecoverMs = 450U;
constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;
constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;

constexpr float kLowLegLengthM = 0.15f;
constexpr float kMidLegLengthM = 0.22f;
constexpr float kHighLegLengthM = 0.34f;
constexpr float kJumpPrepLegLengthM = 0.13f;
constexpr float kJumpPushLegLengthM = 0.4f;
constexpr float kJumpRecoverLegLengthM = 0.16f;
constexpr float kJumpPushReachedLegLengthM = 0.365f;
constexpr float kLegLengthRampTimeS = 0.5f;
constexpr float kStairClimbThetaThresholdRad = 0.5f;
constexpr float kStairClimbLegLengthM = 0.16f;
constexpr float kStairClimbThetaTargetRad = 0.2f;
constexpr std::uint32_t kStairClimbDurationMs = 400U;
constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.01f;
constexpr float kStairClimbThetaNearZeroThresholdRad = 0.08f;
constexpr std::uint32_t kStairClimbPitchStableMs = 300U;
}  // namespace chassis_fsm

namespace chassis {
using namespace common::chassis;

constexpr float kSpringTorqueScale = 90.0f;
constexpr float kBodyMassKg = 24.0f;
constexpr float kRollBalanceTargetRad = 0.052f;
constexpr float kPostureThetaBMinRad = -0.8f;
constexpr float kPostureThetaBMaxRad = 0.8f;

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

constexpr PidGains kLeftL0Pid{8000.0f, 0.15f, 70000.0f, 170.0f, 30.0f};
constexpr PidGains kRightL0Pid{8000.0f, 0.15f, 70000.0f, 170.0f, 30.0f};
constexpr PidGains kLeftL0PidJumpTwo{8000.0f, 0.0f, 70000.0f, 250.0f, 0.0f};
constexpr PidGains kRightL0PidJumpTwo{8000.0f, 0.0f, 70000.0f, 250.0f, 0.0f};
constexpr PidGains kLeftL0PidJumpThree{6500.0f, 0.15f, 60000.0f, 190.0f, 30.0f};
constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 60000.0f, 190.0f, 30.0f};
constexpr PidGains kRollPid{600.0f, 0.0f, 200.0f, 180.0f, 0.0f};
constexpr PidGains kLeftLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};
constexpr PidGains kRightLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};
}  // namespace chassis

namespace control_loop {
using namespace common::control_loop;

constexpr float kTargetForwardSpeedMaxMps = 1.8f;
constexpr float kVxInputDeadbandNorm = 0.1f;
constexpr float kVyInputDeadbandNorm = 0.1f;
constexpr float kYawFollowRampStepRadS = 0.05f;
constexpr float kPositionFreezeSpeedThresholdMps = 0.3f;
constexpr float kSpinYawRampStepRadS = 0.005f;
constexpr float kSpinTargetYawDotRadS = 6.0f;
constexpr float kSpinTranslationGain = 1.0f;
constexpr float kSpinThetaLlBiasRad = 0.0f;
constexpr float kYawFollowFixedTargetRad = 1.216f;
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;
constexpr float kExpectedThetaLlBiasRad = -0.12f;
constexpr float kExpectedThetaLrBiasRad = -0.12f;
constexpr float kExpectedThetaBBiasRad = -0.123f;

constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.008f};
constexpr SdotRampParams kSdotRampMidLeg{0.006f, 0.003f};
constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};

constexpr PidGains kYawFollowPid{8.0f, 0.0f, 1.f, 4.0f, 0.0f};
}  // namespace control_loop

namespace actuators {
using namespace common::actuators;

constexpr float kLeftWheelTorqueToCurrent = 2300.0f;
constexpr float kRightWheelTorqueToCurrent = 2300.0f;
}  // namespace actuators

namespace state_estimator {
using namespace common::state_estimator;

constexpr float kLeftPhi1OffsetRad = -0.05f + M_PI;
constexpr float kLeftPhi4OffsetRad = -0.59 + 0.07f;
constexpr float kRightPhi1OffsetRad = 3.04 + M_PI;
constexpr float kRightPhi4OffsetRad = -2.17;
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

namespace aimbot {
constexpr uint8_t kRobotId = 1U;
constexpr float kBulletSpeedMps = 11.8f;
constexpr PidGains kYawPositionPid{18.0f, 0.5f, 0.02f, 1000.0f, 2.0f};
constexpr PidGains kYawSpeedPid{0.8f, 0.0f, 0.0f, 10.0f, 0.3f};
constexpr PidGains kPitchPositionPid{20.0f, 0.5f, 0.02f, 1000.0f, 1.5f};
constexpr PidGains kPitchSpeedPid{1.5f, 0.0f, 0.0f, 10.0f, 0.3f};
}  // namespace aimbot

}  // namespace hero

// ── Infantry3 (variant 2) 特殊参数 ──
namespace infantry3 {
using namespace common;

namespace globals {
using namespace common::globals;
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

constexpr std::uint16_t kLeftWheelId = 0x02;
constexpr std::uint16_t kRightWheelId = 0x03;

constexpr std::uint16_t kDmLfMasterId = 0x17;
constexpr std::uint16_t kDmLfSlaveId = 0x07;
constexpr std::uint16_t kDmLbMasterId = 0x14;
constexpr std::uint16_t kDmLbSlaveId = 0x04;
constexpr std::uint16_t kDmRfMasterId = 0x16;
constexpr std::uint16_t kDmRfSlaveId = 0x06;
constexpr std::uint16_t kDmRbMasterId = 0x15;
constexpr std::uint16_t kDmRbSlaveId = 0x05;

constexpr std::uint16_t kFricLeftId = 0x02;
constexpr std::uint16_t kFricRightId = 0x04;
constexpr std::uint16_t kDialId = 0x01;

const DmMitSettings kDmLfSettings{kDmLfMasterId, kDmLfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmLbSettings{kDmLbMasterId, kDmLbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmRfSettings{kDmRfMasterId, kDmRfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmRbSettings{kDmRbMasterId, kDmRbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
}  // namespace globals

namespace gimbal {
using namespace common::gimbal;
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

const DmMitSettings kPitchMotorSettings{0x05, 0x04, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
const DmMitSettings kYawMotorSettings{0x10, 0x09, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};

constexpr float kPitchMinRad = -0.3f;
constexpr float kPitchMaxRad = 0.25f;

inline constexpr PidGains kYawPositionPid{25.0f, 0.0f, 0.05f, 10.0f, 1.0f};    ///< 偏航位置 PID
inline constexpr PidGains kYawSpeedPid{0.6f, 0.0f, 0.0f, 6.0f, 0.4f};          ///< 偏航速度 PID
inline constexpr PidGains kPitchPositionPid{23.0f, 0.0f, 0.15f, 10.0f, 0.4f};  ///< 俯仰位置 PID
inline constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 8.0f, 0.0f};       ///< 俯仰速度 PID
}  // namespace gimbal

namespace shoot {
inline constexpr int kFrictionWheelCount = 2;
constexpr float kFricSpeedTargetRpm = 6900.0f;
constexpr PidGains kFricSpeedPid{20.0f, 1.0f, 0.0f, 16000.0f, 2000.0f};
constexpr PidGains kDialSpeedPid{15.0f, 0.f, 0.2f, 2000.0f, 0.0f};
constexpr PidGains kDialPositionPid{8.0f, 0.f, 0.0f, 12000.0f, 0.0f};
constexpr int16_t kDialFireThreshold = -600;
constexpr float kShootFrequencyHz = 10.0f;
}  // namespace shoot

namespace chassis_fsm {
constexpr std::uint32_t kJumpPrepMs = 200U;
constexpr std::uint32_t kJumpPushMaxMs = 1000U;
constexpr std::uint32_t kJumpRecoverMs = 250U;
constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;
constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;

constexpr float kLowLegLengthM = 0.127f;
constexpr float kMidLegLengthM = 0.18f;
constexpr float kHighLegLengthM = 0.3f;
constexpr float kJumpPrepLegLengthM = 0.13f;
constexpr float kJumpPushLegLengthM = 0.22f;
constexpr float kJumpRecoverLegLengthM = 0.20f;
constexpr float kJumpPushReachedLegLengthM = 0.21f;
constexpr float kLegLengthRampTimeS = 0.5f;
constexpr float kStairClimbThetaThresholdRad = 0.5f;
constexpr float kStairClimbLegLengthM = 0.14f;
constexpr float kStairClimbThetaTargetRad = 0.2f;
constexpr std::uint32_t kStairClimbDurationMs = 3000U;
constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.03f;
constexpr float kStairClimbThetaNearZeroThresholdRad = 0.1f;
constexpr std::uint32_t kStairClimbPitchStableMs = 1000U;
}  // namespace chassis_fsm

namespace chassis {
using namespace common::chassis;

constexpr float kSpringTorqueScale = 90.0f;
constexpr float kBodyMassKg = 22.0f;
constexpr float kRollBalanceTargetRad = 0.003f;
constexpr float kPostureThetaBMinRad = -0.7f;
constexpr float kPostureThetaBMaxRad = 0.7f;

constexpr std::array<float, 240> kCtrlP{
    -3.4453,    -29.961,   23.245,   47.037,    -20.955, -18.206,  -5.4247,    -32.485, 31.129,    54.485,    -33.887,
    -23.204,    -0.36289,  1.672,    -0.70661,  -1.5208, -0.13492, 1.1651,     -1.4242, 6.733,     -2.9926,   -5.9207,
    -0.81123,   4.9865,    -9.9478,  -74.833,   15.323,  67.249,   6.2928,     -20.892, -0.6747,   -6.0053,   2.7133,
    -3.9397,    2.4861,    -3.428,   -3.588,    1.13,    -4.8555,  14.193,     -16.024, 6.0767,    -0.33916,  -1.657,
    -0.020045,  6.0915,    -11.042,  1.6444,    -21.912, 27.05,    32.007,     7.8292,  -33.632,   -30.118,   -2.6085,
    1.5988,     6.5445,    3.3226,   -6.5594,   -6.4544, -3.4453,  23.245,     -29.961, -18.206,   -20.955,   47.037,
    -5.4247,    31.129,    -32.485,  -23.204,   -33.887, 54.485,   0.36289,    0.70661, -1.672,    -1.1651,   0.13492,
    1.5208,     1.4242,    2.9926,   -6.733,    -4.9865, 0.81123,  5.9207,     -3.588,  -4.8555,   1.13,      6.0767,
    -16.024,    14.193,    -0.33916, -0.020045, -1.657,  1.6444,   -11.042,    6.0915,  -9.9478,   15.323,    -74.833,
    -20.892,    6.2928,    67.249,   -0.6747,   2.7133,  -6.0053,  -3.428,     2.4861,  -3.9397,   -21.912,   32.007,
    27.05,      -30.118,   -33.632,  7.8292,    -2.6085, 6.5445,   1.5988,     -6.4544, -6.5594,   3.3226,    9.8618,
    -0.0058622, -20.878,   -55.004,  52.333,    14.852,  13.626,   -0.31521,   -36.378, -69.809,   80.662,    24.507,
    -0.35502,   -2.6919,   -0.65176, 4.5717,    -1.1898, 1.1832,   -1.3914,    -10.977, -2.3737,   18.516,    -4.9983,
    4.3774,     48.014,    -62.452,  3.9316,    26.788,  44.918,   -17.163,    2.4601,  1.9122,    -2.2125,   -4.9742,
    7.7827,     0.71535,   -1.7573,  -27.911,   -4.0238, 24.961,   -23.219,    5.9696,  -0.073852, -0.049453, 3.1994,
    -5.8818,    0.50024,   -3.5729,  -35.997,   -189.92, 55.549,   218.51,     42.288,  -80.627,   -1.3261,   -17.074,
    2.533,      14.63,     10.792,   -5.465,    9.8618,  -20.878,  -0.0058622, 14.852,  52.333,    -55.004,   13.626,
    -36.378,    -0.31521,  24.507,   80.662,    -69.809, 0.35502,  0.65176,    2.6919,  -1.1832,   1.1898,    -4.5717,
    1.3914,     2.3737,    10.977,   -4.3774,   4.9983,  -18.516,  -1.7573,    -4.0238, -27.911,   5.9696,    -23.219,
    24.961,     -0.073852, 3.1994,   -0.049453, -3.5729, 0.50024,  -5.8818,    48.014,  3.9316,    -62.452,   -17.163,
    44.918,     26.788,    2.4601,   -2.2125,   1.9122,  0.71535,  7.7827,     -4.9742, -35.997,   55.549,    -189.92,
    -80.627,    42.288,    218.51,   -1.3261,   2.533,   -17.074,  -5.465,     10.792,  14.63,
};

constexpr PidGains kLeftL0Pid{7500.0f, 0.04f, 90000.0f, 170.0f, 10.0f};
constexpr PidGains kRightL0Pid{8500.0f, 0.04f, 90000.0f, 170.0f, 10.0f};
constexpr PidGains kLeftL0PidJumpTwo{7000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};
constexpr PidGains kRightL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};
constexpr PidGains kLeftL0PidJumpThree{7500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};
constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};
constexpr PidGains kRollPid{800.0f, 0.0f, 200.0f, 180.0f, 0.0f};
constexpr PidGains kLeftLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};
constexpr PidGains kRightLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};
}  // namespace chassis

namespace control_loop {
using namespace common::control_loop;

constexpr float kTargetForwardSpeedMaxMps = 2.1f;
constexpr float kVxInputDeadbandNorm = 0.05f;
constexpr float kVyInputDeadbandNorm = 0.05f;
constexpr float kYawFollowRampStepRadS = 0.05f;
constexpr float kPositionFreezeSpeedThresholdMps = 0.35f;
constexpr float kSpinYawRampStepRadS = 0.05f;
constexpr float kSpinTargetYawDotRadS = 6.0f;
constexpr float kSpinTranslationGain = 1.0f;
constexpr float kSpinThetaLlBiasRad = 0.01f;
constexpr float kYawFollowFixedTargetRad = 0.f;
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;
constexpr float kExpectedThetaLlBiasRad = 0.13f;
constexpr float kExpectedThetaLrBiasRad = 0.13f;
constexpr float kExpectedThetaBBiasRad = 0.0f;

constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.01f};
constexpr SdotRampParams kSdotRampMidLeg{0.007f, 0.007f};
constexpr SdotRampParams kSdotRampHighLeg{0.005f, 0.005f};

constexpr PidGains kYawFollowPid{10.0f, 0.0f, 2.2f, 10.0f, 0.0f};
}  // namespace control_loop

namespace actuators {
using namespace common::actuators;

constexpr float kLeftWheelTorqueToCurrent = 2500.0f;
constexpr float kRightWheelTorqueToCurrent = 2300.0f;
}  // namespace actuators

namespace state_estimator {
using namespace common::state_estimator;

constexpr float kLeftPhi1OffsetRad = kPi - 2.94f;
constexpr float kLeftPhi4OffsetRad = 0.59f;
constexpr float kRightPhi1OffsetRad = kPi + 2.4f;
constexpr float kRightPhi4OffsetRad = -1.87f;
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

namespace aimbot {
constexpr uint8_t kRobotId = 3U;
constexpr float kBulletSpeedMps = 23.0f;
constexpr PidGains kYawPositionPid{20.0f, 0.5f, 0.05f, 10.0f, 2.0f};
constexpr PidGains kYawSpeedPid{0.5f, 0.0f, 0.0f, 6.0f, 0.3f};
constexpr PidGains kPitchPositionPid{22.0f, 0.5f, 0.05f, 10.0f, 1.5f};
constexpr PidGains kPitchSpeedPid{0.45f, 0.0f, 0.0f, 8.0f, 0.3f};
}  // namespace aimbot

}  // namespace infantry3

// ── Infantry4 (variant 3) 特殊参数 ──
namespace infantry4 {
using namespace common;

namespace globals {
using namespace common::globals;
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

constexpr std::uint16_t kLeftWheelId = 0x06;
constexpr std::uint16_t kRightWheelId = 0x05;

constexpr std::uint16_t kDmLfMasterId = 0x02;
constexpr std::uint16_t kDmLfSlaveId = 0x01;
constexpr std::uint16_t kDmLbMasterId = 0x04;
constexpr std::uint16_t kDmLbSlaveId = 0x03;
constexpr std::uint16_t kDmRfMasterId = 0x06;
constexpr std::uint16_t kDmRfSlaveId = 0x05;
constexpr std::uint16_t kDmRbMasterId = 0x08;
constexpr std::uint16_t kDmRbSlaveId = 0x07;

constexpr std::uint16_t kFricLeftId = 0x07;
constexpr std::uint16_t kFricRightId = 0x08;
constexpr std::uint16_t kDialId = 0x07;

const DmMitSettings kDmLfSettings{kDmLfMasterId, kDmLfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmLbSettings{kDmLbMasterId, kDmLbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmRfSettings{kDmRfMasterId, kDmRfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
const DmMitSettings kDmRbSettings{kDmRbMasterId, kDmRbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
}  // namespace globals

namespace gimbal {
using namespace common::gimbal;
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

const DmMitSettings kPitchMotorSettings{0x12, 0x11, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
const DmMitSettings kYawMotorSettings{0x13, 0x03, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};

constexpr float kPitchMinRad = -0.3f;
constexpr float kPitchMaxRad = 0.3f;

constexpr PidGains kYawPositionPid{25.0f, 0.0f, 0.05f, 10.0f, 1.0f};
constexpr PidGains kYawSpeedPid{0.6f, 0.0f, 0.0f, 6.0f, 0.4f};
constexpr PidGains kPitchPositionPid{26.0f, 0.0f, 0.1f, 10.0f, 0.4f};
constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 8.0f, 0.0f};
}  // namespace gimbal

namespace shoot {
inline constexpr int kFrictionWheelCount = 2;
constexpr float kFricSpeedTargetRpm = 6000.0f;
constexpr PidGains kFricSpeedPid{20.0f, 1.0f, 0.0f, 16000.0f, 2000.0f};
constexpr PidGains kDialSpeedPid{10.0f, 0.5f, 0.0f, 16000.0f, 1000.0f};
constexpr PidGains kDialPositionPid{5.0f, 0.1f, 0.0f, 1500.0f, 500.0f};
constexpr int16_t kDialFireThreshold = -600;
constexpr float kShootFrequencyHz = 24.0f;
}  // namespace shoot

namespace chassis_fsm {
constexpr std::uint32_t kJumpPrepMs = 250U;
constexpr std::uint32_t kJumpPushMaxMs = 1000U;
constexpr std::uint32_t kJumpRecoverMs = 250U;
constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;
constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;

constexpr float kLowLegLengthM = 0.15f;
constexpr float kMidLegLengthM = 0.22f;
constexpr float kHighLegLengthM = 0.35f;
constexpr float kJumpPrepLegLengthM = 0.13f;
constexpr float kJumpPushLegLengthM = 0.25f;
constexpr float kJumpRecoverLegLengthM = 0.20f;
constexpr float kJumpPushReachedLegLengthM = 0.25f;
constexpr float kLegLengthRampTimeS = 0.5f;
constexpr float kStairClimbThetaThresholdRad = 0.5f;
constexpr float kStairClimbLegLengthM = 0.16f;
constexpr float kStairClimbThetaTargetRad = 0.3f;
constexpr std::uint32_t kStairClimbDurationMs = 250U;
constexpr std::uint32_t kStairClimbPitchStableMs = 2000U;
constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.03f;
constexpr float kStairClimbThetaNearZeroThresholdRad = 0.1f;
}  // namespace chassis_fsm

namespace chassis {
using namespace common::chassis;

constexpr float kSpringTorqueScale = 70.0f;
constexpr float kBodyMassKg = 18.0f;
constexpr float kRollBalanceTargetRad = 0.f;
constexpr float kPostureThetaBMinRad = -0.7f;
constexpr float kPostureThetaBMaxRad = 0.7f;

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

constexpr PidGains kLeftL0Pid{8000.0f, 0.15f, 50000.0f, 170.0f, 30.0f};
constexpr PidGains kRightL0Pid{8000.0f, 0.15f, 50000.0f, 170.0f, 30.0f};
constexpr PidGains kLeftL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};
constexpr PidGains kRightL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};
constexpr PidGains kLeftL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};
constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};
constexpr PidGains kRollPid{800.0f, 0.0f, 80.0f, 80.0f, 0.0f};
constexpr PidGains kLeftLegTurnPid{20.0f, 0.0f, 0.0f, 15.0f, 0.0f};
constexpr PidGains kRightLegTurnPid{20.0f, 0.0f, 0.0f, 15.0f, 0.0f};
}  // namespace chassis

namespace control_loop {
using namespace common::control_loop;

constexpr float kTargetForwardSpeedMaxMps = 2.f;
constexpr float kVxInputDeadbandNorm = 0.1f;
constexpr float kVyInputDeadbandNorm = 0.1f;
constexpr float kYawFollowRampStepRadS = 0.05f;
constexpr float kPositionFreezeSpeedThresholdMps = 0.4f;
constexpr float kSpinYawRampStepRadS = 0.05f;
constexpr float kSpinTargetYawDotRadS = 7.0f;
constexpr float kSpinTranslationGain = 1.2f;
constexpr float kSpinThetaLlBiasRad = 0.01f;
constexpr float kYawFollowFixedTargetRad = -1.72f;
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;
constexpr float kExpectedThetaLlBiasRad = 0.11f;
constexpr float kExpectedThetaLrBiasRad = 0.11f;
constexpr float kExpectedThetaBBiasRad = 0.f;

constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.008f};
constexpr SdotRampParams kSdotRampMidLeg{0.006f, 0.003f};
constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};

constexpr PidGains kYawFollowPid{8.2f, 0.0f, 1.2f, 6.0f, 0.0f};
}  // namespace control_loop

namespace actuators {
using namespace common::actuators;

constexpr float kLeftWheelTorqueToCurrent = 2436.5f;
constexpr float kRightWheelTorqueToCurrent = 2436.5f;
}  // namespace actuators

namespace state_estimator {
using namespace common::state_estimator;

constexpr float kLeftPhi1OffsetRad = -1.50f + M_PI;
constexpr float kLeftPhi4OffsetRad = -1.50f;
constexpr float kRightPhi1OffsetRad = -1.42f + M_PI;
constexpr float kRightPhi4OffsetRad = -1.62f;
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

namespace aimbot {
constexpr uint8_t kRobotId = 4U;
constexpr float kBulletSpeedMps = 23.0f;
constexpr PidGains kYawPositionPid{20.0f, 0.5f, 0.05f, 10.0f, 2.0f};
constexpr PidGains kYawSpeedPid{0.5f, 0.0f, 0.0f, 6.0f, 0.3f};
constexpr PidGains kPitchPositionPid{22.0f, 0.5f, 0.05f, 10.0f, 1.5f};
constexpr PidGains kPitchSpeedPid{0.45f, 0.0f, 0.0f, 8.0f, 0.3f};
}  // namespace aimbot

}  // namespace infantry4

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
