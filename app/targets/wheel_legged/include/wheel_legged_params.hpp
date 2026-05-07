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

constexpr PidGains kYawPositionPid{24.0f, 0.0f, 1.0f, 1000.0f, 1.0f};
constexpr PidGains kYawSpeedPid{1.f, 0.0f, 0.0f, 10.0f, 0.4f};
constexpr PidGains kPitchPositionPid{25.0f, 0.0f, 1.0f, 1000.0f, 0.4f};
constexpr PidGains kPitchSpeedPid{2.f, 0.0f, 0.0f, 10.0f, 0.0f};
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

constexpr float kLowLegLengthM = 0.14f;
constexpr float kMidLegLengthM = 0.185f;
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
constexpr float kMidLegLandingThresholdM = 0.14f;
constexpr std::uint32_t kMidLegLandingHoldMs = 200U;
}  // namespace chassis_fsm

namespace chassis {
using namespace common::chassis;

constexpr float kSpringTorqueScale = 105.0f;
constexpr float kBodyMassKg = 24.0f;
constexpr float kRollBalanceTargetRad = 0.025f;
constexpr float kPostureThetaBMinRad = -0.8f;
constexpr float kPostureThetaBMaxRad = 0.8f;

constexpr std::array<float, 240> kCtrlP{
-1.5468,  -11.315,  8.516,  16.038,  -5.9155,  -7.0138,
     -4.3901,  -24.461,  22.215,  37.798,  -20.621,  -17.081,
     -0.80964,  2.9629,  -1.3612,  -3.0213,  0.11841,  1.914,
     -2.3141,  8.7524,  -4.2418,  -8.5673,  -0.18351,  6.1178,
     -11.108,  -81.996,  14.445,  80.287,  6.676,  -19.485,
     -0.62943,  -5.4369,  2.2088,  -4.0923,  2.5798,  -2.9228,
     -4.2179,  4.083,  -7.9454,  8.756,  -2.8755,  -0.79268,
     -0.35064,  -1.1103,  -0.39638,  4.2991,  -7.9864,  0.4727,
     -26.374,  29.246,  37.512,  15.522,  -42.143,  -31.286,
     -2.6892,  1.8861,  6.2337,  2.7652,  -6.3247,  -5.8048,
     -1.5468,  8.516,  -11.315,  -7.0138,  -5.9155,  16.038,
     -4.3901,  22.215,  -24.461,  -17.081,  -20.621,  37.798,
     0.80964,  1.3612,  -2.9629,  -1.914,  -0.11841,  3.0213,
     2.3141,  4.2418,  -8.7524,  -6.1178,  0.18351,  8.5673,
     -4.2179,  -7.9454,  4.083,  -0.79268,  -2.8755,  8.756,
     -0.35064,  -0.39638,  -1.1103,  0.4727,  -7.9864,  4.2991,
     -11.108,  14.445,  -81.996,  -19.485,  6.676,  80.287,
     -0.62943,  2.2088,  -5.4369,  -2.9228,  2.5798,  -4.0923,
     -26.374,  37.512,  29.246,  -31.286,  -42.143,  15.522,
     -2.6892,  6.2337,  1.8861,  -5.8048,  -6.3247,  2.7652,
     4.9536,  6.5699,  -16.115,  -32.438,  26.075,  10.565,
     13.03,  13.044,  -44.952,  -77.386,  75.931,  27.957,
     -0.73814,  -5.5332,  -1.8992,  9.382,  -3.0584,  2.9834,
     -2.1185,  -16.501,  -4.9552,  27.529,  -8.916,  7.6704,
     69.068,  -100.17,  10.092,  66.171,  56.213,  -26.629,
     2.8162,  2.2269,  -2.1044,  -0.45348,  4.9946,  0.56876,
     -3.297,  -36.839,  -6.968,  38.624,  -41.763,  -1.7674,
     -0.2272,  0.30682,  3.5131,  -6.5952,  2.6953,  -8.3803,
     -47.602,  -261.38,  78.095,  287.32,  52.904,  -100.84,
     -1.7002,  -18.423,  2.7777,  14.59,  11.293,  -5.037,
     4.9536,  -16.115,  6.5699,  10.565,  26.075,  -32.438,
     13.03,  -44.952,  13.044,  27.957,  75.931,  -77.386,
     0.73814,  1.8992,  5.5332,  -2.9834,  3.0584,  -9.382,
     2.1185,  4.9552,  16.501,  -7.6704,  8.916,  -27.529,
     -3.297,  -6.968,  -36.839,  -1.7674,  -41.763,  38.624,
     -0.2272,  3.5131,  0.30682,  -8.3803,  2.6953,  -6.5952,
     69.068,  10.092,  -100.17,  -26.629,  56.213,  66.171,
     2.8162,  -2.1044,  2.2269,  0.56876,  4.9946,  -0.45348,
     -47.602,  78.095,  -261.38,  -100.84,  52.904,  287.32,
     -1.7002,  2.7777,  -18.423,  -5.037,  11.293,  14.59,
};

constexpr PidGains kLeftL0Pid{9000.0f, 0.15f, 70000.0f, 170.0f, 30.0f};
constexpr PidGains kRightL0Pid{9000.0f, 0.15f, 70000.0f, 170.0f, 30.0f};
constexpr PidGains kLeftL0PidJumpTwo{8000.0f, 0.0f, 70000.0f, 250.0f, 0.0f};
constexpr PidGains kRightL0PidJumpTwo{8000.0f, 0.0f, 70000.0f, 250.0f, 0.0f};
constexpr PidGains kLeftL0PidJumpThree{6500.0f, 0.15f, 60000.0f, 190.0f, 30.0f};
constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 60000.0f, 190.0f, 30.0f};
constexpr PidGains kRollPid{800.0f, 0.1f, 200.0f, 180.0f, 30.0f};
constexpr PidGains kLeftLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};
constexpr PidGains kRightLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};
}  // namespace chassis

namespace control_loop {
using namespace common::control_loop;

constexpr float kTargetForwardSpeedMaxMps = 2.3f;
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
constexpr float kYawFollowFixedTargetRad = -1.84f;
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;
constexpr float kExpectedThetaLlBiasRad = -0.01f;
constexpr float kExpectedThetaLrBiasRad = -0.01f;
constexpr float kExpectedThetaBBiasRad = -0.123f;

constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.008f};
constexpr SdotRampParams kSdotRampMidLeg{0.006f, 0.003f};
constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};

constexpr PidGains kYawFollowPid{8.0f, 0.0f, 1.f, 4.0f, 0.0f};
}  // namespace control_loop

namespace actuators {
using namespace common::actuators;

constexpr float kLeftWheelTorqueToCurrent = 2436.0f;
constexpr float kRightWheelTorqueToCurrent = 2436.0f;
}  // namespace actuators

namespace state_estimator {
using namespace common::state_estimator;

inline constexpr float kLeftPhi1OffsetRad = -0.05f + kPi + 0.09f;
inline constexpr float kLeftPhi4OffsetRad = -0.59 + 0.07f + 0.04f;
inline constexpr float kRightPhi1OffsetRad = 2.7f + kPi + 0.04;
inline constexpr float kRightPhi4OffsetRad = -2.11 + 0.015;
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
constexpr float kMidLegLandingThresholdM = 0.14f;
constexpr std::uint32_t kMidLegLandingHoldMs = 300U;
}  // namespace chassis_fsm

namespace chassis {
using namespace common::chassis;

constexpr float kSpringTorqueScale = 90.0f;
constexpr float kBodyMassKg = 22.0f;
constexpr float kRollBalanceTargetRad = 0.016f;
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
constexpr float kMidLegLandingThresholdM = 0.16f;
constexpr std::uint32_t kMidLegLandingHoldMs = 300U;
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