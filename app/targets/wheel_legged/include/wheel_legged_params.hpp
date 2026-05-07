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
constexpr float kHighLegLengthM = 0.325f;
constexpr float kJumpPrepLegLengthM = 0.13f;
constexpr float kJumpPushLegLengthM = 0.4f;
constexpr float kJumpRecoverLegLengthM = 0.16f;
constexpr float kJumpPushReachedLegLengthM = 0.365f;
constexpr float kLegLengthRampTimeS = 0.5f;
constexpr float kStairClimbThetaThresholdRad = 0.5f;
constexpr float kStairClimbLegLengthM = 0.16f;
constexpr float kStairClimbThetaTargetRad = 1.f;
constexpr std::uint32_t kStairClimbDurationMs = 400U;
constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.01f;
constexpr float kStairClimbThetaNearZeroThresholdRad = 0.08f;
constexpr std::uint32_t kStairClimbPitchStableMs = 300U;
}  // namespace chassis_fsm

namespace chassis {
using namespace common::chassis;

constexpr float kSpringTorqueScale = 105.0f;
constexpr float kBodyMassKg = 24.0f;
constexpr float kRollBalanceTargetRad = 0.025f;
constexpr float kPostureThetaBMinRad = -0.8f;
constexpr float kPostureThetaBMaxRad = 0.8f;

constexpr std::array<float, 240> kCtrlP{
-3.3216,  -23.892,  18.669,  34.707,  -14.728,  -14.349,
     -6.7988,  -33.491,  33.694,  54.905,  -36.385,  -23.617,
     -0.85716,  3.3289,  -1.5318,  -3.248,  -0.29215,  2.2935,
     -2.447,  9.8918,  -4.8358,  -9.1812,  -1.6197,  7.4312,
     -14.712,  -100.55,  15.974,  101.78,  9.0297,  -21.578,
     -0.76605,  -6.9557,  2.9234,  -4.9855,  3.2763,  -3.5857,
     -4.8311,  4.7421,  -7.0863,  10.459,  -9.8608,  -1.0781,
     -0.42566,  -1.5166,  -0.49172,  6.0462,  -11.993,  1.7067,
     -28.586,  33.643,  40.69,  12.525,  -44.978,  -35.398,
     -3.0334,  1.9889,  7.5289,  3.3748,  -7.8228,  -6.9874,
     -3.3216,  18.669,  -23.892,  -14.349,  -14.728,  34.707,
     -6.7988,  33.694,  -33.491,  -23.617,  -36.385,  54.905,
     0.85716,  1.5318,  -3.3289,  -2.2935,  0.29215,  3.248,
     2.447,  4.8358,  -9.8918,  -7.4312,  1.6197,  9.1812,
     -4.8311,  -7.0863,  4.7421,  -1.0781,  -9.8608,  10.459,
     -0.42566,  -0.49172,  -1.5166,  1.7067,  -11.993,  6.0462,
     -14.712,  15.974,  -100.55,  -21.578,  9.0297,  101.78,
     -0.76605,  2.9234,  -6.9557,  -3.5857,  3.2763,  -4.9855,
     -28.586,  40.69,  33.643,  -35.398,  -44.978,  12.525,
     -3.0334,  7.5289,  1.9889,  -6.9874,  -7.8228,  3.3748,
     9.1467,  10.414,  -27.567,  -59.092,  49.842,  16.152,
     16.865,  13.937,  -56.935,  -97.467,  104.69,  31.215,
     -0.81824,  -5.4934,  -1.7564,  9.1938,  -2.6618,  2.7232,
     -2.3479,  -16.479,  -4.4237,  27.04,  -7.6425,  6.7091,
     84.395,  -141.69,  9.1122,  110.88,  51.812,  -25.991,
     3.103,  3.0073,  -2.8918,  -1.8216,  5.9451,  0.78042,
     -3.8084,  -36.577,  -5.2449,  36.773,  -31.965,  -8.7031,
     -0.14705,  0.57532,  3.9223,  -8.6431,  5.4232,  -9.8387,
     -47.004,  -266.86,  78.586,  300.32,  51.532,  -102.35,
     -1.4079,  -18.945,  1.6502,  14.558,  13.489,  -4.4987,
     9.1467,  -27.567,  10.414,  16.152,  49.842,  -59.092,
     16.865,  -56.935,  13.937,  31.215,  104.69,  -97.467,
     0.81824,  1.7564,  5.4934,  -2.7232,  2.6618,  -9.1938,
     2.3479,  4.4237,  16.479,  -6.7091,  7.6425,  -27.04,
     -3.8084,  -5.2449,  -36.577,  -8.7031,  -31.965,  36.773,
     -0.14705,  3.9223,  0.57532,  -9.8387,  5.4232,  -8.6431,
     84.395,  9.1122,  -141.69,  -25.991,  51.812,  110.88,
     3.103,  -2.8918,  3.0073,  0.78042,  5.9451,  -1.8216,
     -47.004,  78.586,  -266.86,  -102.35,  51.532,  300.32,
     -1.4079,  1.6502,  -18.945,  -4.4987,  13.489,  14.558,
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

constexpr float kTargetForwardSpeedMaxMps = 2.1f;
constexpr float kVxInputDeadbandNorm = 0.1f;
constexpr float kVyInputDeadbandNorm = 0.1f;
constexpr float kYawFollowRampStepRadS = 0.05f;
constexpr float kPositionFreezeSpeedThresholdMps = 0.3f;
constexpr float kSpinYawRampStepRadS = 0.005f;
constexpr float kSpinTargetYawDotRadS = 9.0f;
constexpr float kSpinTranslationGain = 1.0f;
constexpr float kSpinThetaLlBiasRad = 0.0f;
constexpr float kYawFollowFixedTargetRad = -1.84f;
constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;
constexpr float kExpectedThetaLlBiasRad = 0.0f;
constexpr float kExpectedThetaLrBiasRad = 0.0f;
constexpr float kExpectedThetaBBiasRad = -0.123f;

constexpr SdotRampParams kSdotRampLowLeg{0.015f, 0.012f};
constexpr SdotRampParams kSdotRampMidLeg{0.010f, 0.008f};
constexpr SdotRampParams kSdotRampHighLeg{0.006f, 0.006f};

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
constexpr float kYawFollowRampStepRadS = 0.05f;
constexpr float kPositionFreezeSpeedThresholdMps = 0.3f;
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
