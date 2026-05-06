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
constexpr std::size_t kPayloadSizeA = 8U;
constexpr std::size_t kPayloadSizeB = 8U;
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
constexpr std::uint32_t kLockPointSpeedSettledTicks = 50U;
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

constexpr float kPitchMinRad = -0.2f;
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

constexpr float kLowLegLengthM = 0.15f;
constexpr float kMidLegLengthM = 0.21f;
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
    -4.2448,  -27.943,  24.322,   48.081,    -22.811,  -20.812, -7.3808, -34.992, 38.389,   65.198,    -42.541,
    -31.147,  -0.67131, 3.8281,   -0.59644,  -4.5997,  0.61137, 1.0939,  -1.5435, 9.1817,   -1.783,    -10.673,
    1.1528,   3.2836,   -17.237,  -73.41,    11.51,    72.6,    -2.0572, -14.979, -1.0135,  -6.4992,   2.9845,
    -4.127,   1.7761,   -3.7835,  -3.4261,   4.7568,   -0.7934, 4.2219,  -9.0128, 1.6866,   -0.33564,  -1.736,
    -0.23046, 6.6042,   -11.509,  2.0019,    -18.794,  43.53,   18.593,  -32.15,  -21.354,  -19.936,   -2.6642,
    3.4226,   5.8749,   0.53733,  -6.3349,   -6.1988,  -4.2448, 24.322,  -27.943, -20.812,  -22.811,   48.081,
    -7.3808,  38.389,   -34.992,  -31.147,   -42.541,  65.198,  0.67131, 0.59644, -3.8281,  -1.0939,   -0.61137,
    4.5997,   1.5435,   1.783,    -9.1817,   -3.2836,  -1.1528, 10.673,  -3.4261, -0.7934,  4.7568,    1.6866,
    -9.0128,  4.2219,   -0.33564, -0.23046,  -1.736,   2.0019,  -11.509, 6.6042,  -17.237,  11.51,     -73.41,
    -14.979,  -2.0572,  72.6,     -1.0135,   2.9845,   -6.4992, -3.7835, 1.7761,  -4.127,   -18.794,   18.593,
    43.53,    -19.936,  -21.354,  -32.15,    -2.6642,  5.8749,  3.4226,  -6.1988, -6.3349,  0.53733,   3.6792,
    -10.117,  1.0467,   -7.9563,  18.399,    -3.2531,  5.6247,  -12.837, -3.7072, -13.93,   32,        -0.69807,
    -0.71498, -1.9782,  -0.98018, 3.7112,    0.088883, 1.5526,  -1.626,  -5.0291, -2.3054,  9.3236,    -0.22558,
    3.716,    29.6,     -63.851,  5.7607,    69.982,   15.007,  -12.607, 1.5174,  -2.1169,  -0.030715, 0.85042,
    5.389,    -1.1938,  -2.6014,  -12.076,   -2.8847,  15.998,  -8.8749, 0.36243, -0.20479, -0.87095,  2.1866,
    -0.5102,  -2.293,   -1.4209,  -27.372,   -82.372,  32.363,  111.12,  7.5488,  -46.071,  -2.0412,   -10.549,
    4.1377,   11.895,   3.6102,   -6.3437,   3.6792,   1.0467,  -10.117, -3.2531, 18.399,   -7.9563,   5.6247,
    -3.7072,  -12.837,  -0.69807, 32,        -13.93,   0.71498, 0.98018, 1.9782,  -1.5526,  -0.088883, -3.7112,
    1.626,    2.3054,   5.0291,   -3.716,    0.22558,  -9.3236, -2.6014, -2.8847, -12.076,  0.36243,   -8.8749,
    15.998,   -0.20479, 2.1866,   -0.87095,  -1.4209,  -2.293,  -0.5102, 29.6,    5.7607,   -63.851,   -12.607,
    15.007,   69.982,   1.5174,   -0.030715, -2.1169,  -1.1938, 5.389,   0.85042, -27.372,  32.363,    -82.372,
    -46.071,  7.5488,   111.12,   -2.0412,   4.1377,   -10.549, -6.3437, 3.6102,  11.895,
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
constexpr std::uint32_t kLockPointSpeedSettledTicks = 30U;
constexpr float kLockPointAlphaRiseStep = 0.015f;
constexpr float kLockPointAlphaFallStep = 0.018f;
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

constexpr SdotRampParams kSdotRampLowLeg{0.005f, 0.005f};
constexpr SdotRampParams kSdotRampMidLeg{0.004f, 0.004f};
constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};

constexpr PidGains kYawFollowPid{8.0f, 0.0f, 1.2f, 6.0f, 0.0f};
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
constexpr PidGains kPitchPositionPid{26.0f, 0.0f, 0.05f, 10.0f, 0.4f};
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
    -2.5181,    -12.705,  10.428,   18.911,   -5.1252,  -10.765, -4.1698, -15.133,  16.533,     24.337,   -11.18,
    -16.725,    -0.43051, 1.7802,   -0.46132, -2.7242,  1.1215,  0.52638, -3.8637,  16.05,      -4.1793,  -24.513,
    10.14,      4.7772,   -7.9653,  -62.552,  16.875,   63.934,  -28.504, -18.326,  -1.0859,    -5.0116,  1.8802,
    -0.0021706, -0.16476, -2.5343,  -4.4015,  14.974,   -14.413, -24.047, 41.189,   13.852,     -0.36365, 0.33046,
    -1.2685,    0.20998,  -1.177,   0.89994,  -12.11,   23.556,  16.569,  -7.3446,  -25.101,    -15.189,  -1.9294,
    2.795,      3.9053,   0.51749,  -5.0412,  -4.0019,  -2.5181, 10.428,  -12.705,  -10.765,    -5.1252,  18.911,
    -4.1698,    16.533,   -15.133,  -16.725,  -11.18,   24.337,  0.43051, 0.46132,  -1.7802,    -0.52638, -1.1215,
    2.7242,     3.8637,   4.1793,   -16.05,   -4.7772,  -10.14,  24.513,  -4.4015,  -14.413,    14.974,   13.852,
    41.189,     -24.047,  -0.36365, -1.2685,  0.33046,  0.89994, -1.177,  0.20998,  -7.9653,    16.875,   -62.552,
    -18.326,    -28.504,  63.934,   -1.0859,  1.8802,   -5.0116, -2.5343, -0.16476, -0.0021706, -12.11,   16.569,
    23.556,     -15.189,  -25.101,  -7.3446,  -1.9294,  3.9053,  2.795,   -4.0019,  -5.0412,    0.51749,  2.5942,
    -1.6259,    -5.3416,  -7.2947,  13.548,   0.051142, 3.8691,  -3.3295, -8.8432,  -8.5796,    21.237,   0.91262,
    -0.30107,   -1.1274,  -1.0526,  2.4883,   -0.81891, 1.4468,  -2.6994, -10.236,  -9.486,     22.496,   -7.4299,
    13.007,     16.051,   6.4535,   14.814,   -15.165,  51.996,  -29.922, 2.2198,   -5.1395,    0.7406,   9.1345,
    3.8064,     -2.2901,  0.26192,  -30.078,  -29.369,  52.186,  -31.467, 9.8085,   -0.2866,    -1.7742,  1.6748,
    2.5492,     -1.2553,  -6.8634,  -17.553,  -68.205,  32.966,  83.851,  1.6665,   -34.333,    -1.7264,  -9.6002,
    4.4675,     10.328,   2.0002,   -4.9447,  2.5942,   -5.3416, -1.6259, 0.051142, 13.548,     -7.2947,  3.8691,
    -8.8432,    -3.3295,  0.91262,  21.237,   -8.5796,  0.30107, 1.0526,  1.1274,   -1.4468,    0.81891,  -2.4883,
    2.6994,     9.486,    10.236,   -13.007,  7.4299,   -22.496, 0.26192, -29.369,  -30.078,    9.8085,   -31.467,
    52.186,     -0.2866,  1.6748,   -1.7742,  -6.8634,  -1.2553, 2.5492,  16.051,   14.814,     6.4535,   -29.922,
    51.996,     -15.165,  2.2198,   0.7406,   -5.1395,  -2.2901, 3.8064,  9.1345,   -17.553,    32.966,   -68.205,
    -34.333,    1.6665,   83.851,   -1.7264,  4.4675,   -9.6002, -4.9447, 2.0002,   10.328,
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
constexpr std::uint32_t kLockPointSpeedSettledTicks = 30U;
constexpr float kLockPointAlphaRiseStep = 0.015f;
constexpr float kLockPointAlphaFallStep = 0.018f;
constexpr float kYawFollowRampStepRadS = 0.05f;
constexpr float kSpinYawRampStepRadS = 0.05f;
constexpr float kSpinTargetYawDotRadS = 7.0f;
constexpr float kSpinTranslationGain = 1.2f;
constexpr float kSpinThetaLlBiasRad = 0.01f;
constexpr float kYawFollowFixedTargetRad = 0.f;
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
