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
constexpr float kLockPointEnterInputThreshold = 0.08f;
constexpr float kLockPointExitInputThreshold = 0.12f;
constexpr std::uint32_t kLockPointMinDwellTicks = 100U;
constexpr float kLockPointFilteredSdotZeroThreshold = 1e-5f;
constexpr float kLockPointAlphaRiseStep = 0.015f;
constexpr float kLockPointAlphaFallStep = 0.018f;
constexpr float kYawFollowRampStepRadS = 0.05f;
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
inline constexpr PidGains kPitchPositionPid{26.0f, 0.0f, 0.05f, 10.0f, 0.4f};  ///< 俯仰位置 PID
inline constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 8.0f, 0.0f};       ///< 俯仰速度 PID
}  // namespace gimbal

namespace shoot {
inline constexpr int kFrictionWheelCount = 2;
constexpr float kFricSpeedTargetRpm = 6000.0f;
constexpr PidGains kFricSpeedPid{20.0f, 1.0f, 0.0f, 16000.0f, 2000.0f};
constexpr PidGains kDialSpeedPid{10.0f, 0.f, 0.0f, 2000.0f, 0.0f};
constexpr PidGains kDialPositionPid{5.0f, 0.f, 0.0f, 1500.0f, 0.0f};
constexpr int16_t kDialFireThreshold = -600;
constexpr float kShootFrequencyHz = 1.0f;
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
    -4.2214,  -30.753, 25.129,  48.885,  -22.672,  -19.467, -6.8658,  -34.141,  35.801,   59.656,   -40.377,   -25.848,
    -0.77615, 3.6689,  -1.1832, -3.9104, 0.22463,  1.9226,  -1.7685,  8.7367,   -3.1576,  -8.8793,  -0.013221, 5.2525,
    -14.076,  -80.556, 14.541,  78.773,  2.93,     -19.276, -0.87164, -6.575,   2.9119,   -4.0666,  2.4181,    -3.5158,
    -4.0014,  4.8923,  -4.8027, 5.8437,  -11.025,  6.1164,  -0.38154, -1.5261,  -0.30763, 6.1156,   -11.719,   2.3242,
    -22.629,  39.279,  28.243,  -13.282, -31.035,  -27.651, -2.7919,  2.5673,   6.6843,   2.1642,   -7.0677,   -6.6044,
    -4.2214,  25.129,  -30.753, -19.467, -22.672,  48.885,  -6.8658,  35.801,   -34.141,  -25.848,  -40.377,   59.656,
    0.77615,  1.1832,  -3.6689, -1.9226, -0.22463, 3.9104,  1.7685,   3.1576,   -8.7367,  -5.2525,  0.013221,  8.8793,
    -4.0014,  -4.8027, 4.8923,  6.1164,  -11.025,  5.8437,  -0.38154, -0.30763, -1.5261,  2.3242,   -11.719,   6.1156,
    -14.076,  14.541,  -80.556, -19.276, 2.93,     78.773,  -0.87164, 2.9119,   -6.575,   -3.5158,  2.4181,    -4.0666,
    -22.629,  28.243,  39.279,  -27.651, -31.035,  -13.282, -2.7919,  6.6843,   2.5673,   -6.6044,  -7.0677,   2.1642,
    7.3911,   -3.3225, -13.797, -35.397, 39.628,   8.0106,  10.597,   -4.0079,  -26.725,  -47.716,  64.247,    15.487,
    -0.78285, -3.8697, -1.4819, 6.7629,  -1.3144,  2.3561,  -1.7788,  -9.4618,  -3.1835,  16.329,   -3.5375,   5.0838,
    45.613,   -82.113, 5.8946,  75.142,  31.651,   -17.194, 2.2927,   -0.68969, -1.3678,  -0.19986, 6.2515,    -0.10509,
    -2.7041,  -22.429, -5.1453, 25.619,  -15.181,  -3.2146, -0.14859, -0.54937, 2.7018,   -3.3897,  0.49555,   -4.3112,
    -35.777,  -152.5,  51.402,  190.06,  23.114,   -71.77,  -1.8202,  -14.733,  3.4294,   14.188,   7.779,     -6.029,
    7.3911,   -13.797, -3.3225, 8.0106,  39.628,   -35.397, 10.597,   -26.725,  -4.0079,  15.487,   64.247,    -47.716,
    0.78285,  1.4819,  3.8697,  -2.3561, 1.3144,   -6.7629, 1.7788,   3.1835,   9.4618,   -5.0838,  3.5375,    -16.329,
    -2.7041,  -5.1453, -22.429, -3.2146, -15.181,  25.619,  -0.14859, 2.7018,   -0.54937, -4.3112,  0.49555,   -3.3897,
    45.613,   5.8946,  -82.113, -17.194, 31.651,   75.142,  2.2927,   -1.3678,  -0.68969, -0.10509, 6.2515,    -0.19986,
    -35.777,  51.402,  -152.5,  -71.77,  23.114,   190.06,  -1.8202,  3.4294,   -14.733,  -6.029,   7.779,     14.188,
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
constexpr float kLockPointEnterInputThreshold = 0.1f;
constexpr float kLockPointExitInputThreshold = 0.12f;
constexpr std::uint32_t kLockPointMinDwellTicks = 10U;
constexpr float kLockPointFilteredSdotZeroThreshold = 1e-5f;
constexpr float kLockPointAlphaRiseStep = 1.f;
constexpr float kLockPointAlphaFallStep = 1.f;
constexpr float kYawFollowRampStepRadS = 0.05f;
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

constexpr float kSpringTorqueScale = 75.0f;
constexpr float kBodyMassKg = 22.0f;
constexpr float kRollBalanceTargetRad = 0.003f;
constexpr float kPostureThetaBMinRad = -0.7f;
constexpr float kPostureThetaBMaxRad = 0.7f;

constexpr std::array<float, 240> kCtrlP{
   -3.5768,  -21.624,  18.53,  35.306,  -14.679,  -16.736,
     -5.3436,  -22.463,  25.319,  40.086,  -24.274,  -22.224,
     -0.42213,  2.1919,  -0.33498,  -2.917,  0.89947,  0.48124,
     -1.914,  10.082,  -1.6283,  -13.323,  4.1105,  2.3783,
     -10.635,  -67.429,  15.112,  69.643,  -15.477,  -17.761,
     -1.2179,  -5.6315,  2.1609,  -0.5621,  0.84763,  -2.8349,
     -3.9761,  10.28,  -6.9588,  -10.706,  12.103,  13.538,
     -0.32598,  -0.46792,  -0.64265,  2.6924,  -5.43,  1.6464,
     -12.561,  26.554,  15.288,  -14.08,  -20.373,  -15.588,
     -2.0762,  2.8753,  4.4387,  0.42789,  -5.1475,  -4.7145,
     -3.5768,  18.53,  -21.624,  -16.736,  -14.679,  35.306,
     -5.3436,  25.319,  -22.463,  -22.224,  -24.274,  40.086,
     0.42213,  0.33498,  -2.1919,  -0.48124,  -0.89947,  2.917,
     1.914,  1.6283,  -10.082,  -2.3783,  -4.1105,  13.323,
     -3.9761,  -6.9588,  10.28,  13.538,  12.103,  -10.706,
     -0.32598,  -0.64265,  -0.46792,  1.6464,  -5.43,  2.6924,
     -10.635,  15.112,  -67.429,  -17.761,  -15.477,  69.643,
     -1.2179,  2.1609,  -5.6315,  -2.8349,  0.84763,  -0.5621,
     -12.561,  15.288,  26.554,  -15.588,  -20.373,  -14.08,
     -2.0762,  4.4387,  2.8753,  -4.7145,  -5.1475,  0.42789,
     3.4379,  -6.7679,  -1.8279,  -8.6042,  17.348,  -1.7567,
     4.4691,  -8.3113,  -5.1749,  -10.321,  25.017,  -0.20635,
     -0.39953,  -1.2613,  -0.87232,  2.4238,  0.051759,  1.185,
     -1.8007,  -5.9406,  -4.0092,  11.322,  0.14691,  5.4318,
     20.243,  -17.94,  9.6357,  8.7585,  21.117,  -18.86,
     2.1922,  -4.8919,  0.26581,  5.8907,  3.7356,  -1.4218,
     -0.37255,  -22.225,  -18.716,  31.55,  -4.5528,  12.228,
     -0.19634,  -1.2609,  1.6207,  0.83895,  -1.0839,  -3.2189,
     -17.252,  -63.51,  26.778,  82.18,  6.2433,  -34.998,
     -1.6554,  -9.4357,  3.9589,  10.583,  2.8401,  -5.5362,
     3.4379,  -1.8279,  -6.7679,  -1.7567,  17.348,  -8.6042,
     4.4691,  -5.1749,  -8.3113,  -0.20635,  25.017,  -10.321,
     0.39953,  0.87232,  1.2613,  -1.185,  -0.051759,  -2.4238,
     1.8007,  4.0092,  5.9406,  -5.4318,  -0.14691,  -11.322,
     -0.37255,  -18.716,  -22.225,  12.228,  -4.5528,  31.55,
     -0.19634,  1.6207,  -1.2609,  -3.2189,  -1.0839,  0.83895,
     20.243,  9.6357,  -17.94,  -18.86,  21.117,  8.7585,
     2.1922,  0.26581,  -4.8919,  -1.4218,  3.7356,  5.8907,
     -17.252,  26.778,  -63.51,  -34.998,  6.2433,  82.18,
     -1.6554,  3.9589,  -9.4357,  -5.5362,  2.8401,  10.583,
};

constexpr PidGains kLeftL0Pid{6000.0f, 0.15f, 50000.0f, 170.0f, 30.0f};
constexpr PidGains kRightL0Pid{6000.0f, 0.15f, 50000.0f, 170.0f, 30.0f};
constexpr PidGains kLeftL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};
constexpr PidGains kRightL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};
constexpr PidGains kLeftL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};
constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};
constexpr PidGains kRollPid{500.0f, 0.0f, 80.0f, 80.0f, 0.0f};
constexpr PidGains kLeftLegTurnPid{20.0f, 0.0f, 0.0f, 15.0f, 0.0f};
constexpr PidGains kRightLegTurnPid{20.0f, 0.0f, 0.0f, 15.0f, 0.0f};
}  // namespace chassis

namespace control_loop {
using namespace common::control_loop;

constexpr float kTargetForwardSpeedMaxMps = 2.f;
constexpr float kVxInputDeadbandNorm = 0.1f;
constexpr float kVyInputDeadbandNorm = 0.1f;
constexpr float kLockPointEnterInputThreshold = 0.1f;
constexpr float kLockPointExitInputThreshold = 0.12f;
constexpr std::uint32_t kLockPointMinDwellTicks = 10U;
constexpr float kLockPointFilteredSdotZeroThreshold = 1e-5f;
constexpr float kLockPointAlphaRiseStep = 0.015f;
constexpr float kLockPointAlphaFallStep = 0.018f;
constexpr float kYawFollowRampStepRadS = 0.05f;
constexpr float kSpinYawRampStepRadS = 0.05f;
constexpr float kSpinTargetYawDotRadS = 7.0f;
constexpr float kSpinTranslationGain = 1.2f;
constexpr float kSpinThetaLlBiasRad = 0.01f;
constexpr float kYawFollowFixedTargetRad = -1.72f;
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;
constexpr float kExpectedThetaLlBiasRad = 0.105f;
constexpr float kExpectedThetaLrBiasRad = 0.105f;
constexpr float kExpectedThetaBBiasRad = 0.05f;

constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.008f};
constexpr SdotRampParams kSdotRampMidLeg{0.006f, 0.003f};
constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};

constexpr PidGains kYawFollowPid{8.0f, 0.0f, 1.2f, 6.0f, 0.0f};
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
