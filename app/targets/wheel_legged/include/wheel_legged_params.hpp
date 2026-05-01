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

namespace hero {

constexpr float kPi = 3.14159265358979323846f;

namespace main {
inline constexpr float kControlLoopFrequencyHz = 500.0f;
}

namespace globals {
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

inline constexpr double kJointCanTxLimitHz = 4000.0;
inline constexpr double kWheelCanTxLimitHz = 4000.0;
inline constexpr double kGimbalCanTxLimitHz = 4000.0;

inline constexpr std::uint16_t kDmLfMasterId = 0x17;
inline constexpr std::uint16_t kDmLfSlaveId = 0x07;
inline constexpr std::uint16_t kDmLbMasterId = 0x14;
inline constexpr std::uint16_t kDmLbSlaveId = 0x04;
inline constexpr std::uint16_t kDmRfMasterId = 0x16;
inline constexpr std::uint16_t kDmRfSlaveId = 0x06;
inline constexpr std::uint16_t kDmRbMasterId = 0x15;
inline constexpr std::uint16_t kDmRbSlaveId = 0x05;

inline constexpr std::uint16_t kLeftWheelId = 0x06;
inline constexpr std::uint16_t kRightWheelId = 0x05;

inline constexpr std::size_t kDr16UartRxBufferSize = 18;
inline constexpr std::size_t kImuUartRxBufferSize = 518;

inline const DmMitSettings kDmLfSettings{kDmLfMasterId, kDmLfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
inline const DmMitSettings kDmLbSettings{kDmLbMasterId, kDmLbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
inline const DmMitSettings kDmRfSettings{kDmRfMasterId, kDmRfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
inline const DmMitSettings kDmRbSettings{kDmRbMasterId, kDmRbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
}  // namespace globals

namespace gimbal {
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

inline const DmMitSettings kPitchMotorSettings{0x12, 0x11, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
inline const DmMitSettings kYawMotorSettings{0x13, 0x03, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};

inline constexpr float kDefaultDtS = 0.002f;
inline constexpr float kPitchMinRad = -0.2f;
inline constexpr float kPitchMaxRad = 0.25f;
inline constexpr float kDmTorqueLimitNm = 10.0f;
inline constexpr float kPitchGravityCompensationNm = 1.3f;

inline constexpr PidGains kYawPositionPid{15.0f, 0.0f, 0.05f, 10.0f, 1.0f};
inline constexpr PidGains kYawSpeedPid{0.6f, 0.0f, 0.0f, 6.0f, 0.4f};
inline constexpr PidGains kPitchPositionPid{13.0f, 0.0f, 0.05f, 10.0f, 0.4f};
inline constexpr PidGains kPitchSpeedPid{0.85f, 0.0f, 0.0f, 8.0f, 0.0f};
}  // namespace gimbal

namespace chassis_fsm {
inline constexpr std::uint32_t kJumpPrepMs = 450U;
inline constexpr std::uint32_t kJumpPushMaxMs = 1000U;
inline constexpr std::uint32_t kJumpRecoverMs = 450U;
inline constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;
inline constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;

inline constexpr float kLowLegLengthM = 0.15f;
inline constexpr float kMidLegLengthM = 0.19f;
inline constexpr float kHighLegLengthM = 0.25f;
inline constexpr float kJumpPrepLegLengthM = 0.13f;
inline constexpr float kJumpPushLegLengthM = 0.36f;
inline constexpr float kJumpRecoverLegLengthM = 0.20f;
inline constexpr float kJumpPushReachedLegLengthM = 0.30f;
inline constexpr float kLegLengthRampTimeS = 0.5f;
inline constexpr float kStairClimbThetaThresholdRad = 0.5f;
inline constexpr float kStairClimbLegLengthM = 0.16f;
inline constexpr float kStairClimbThetaTargetRad = 0.2f;
inline constexpr std::uint32_t kStairClimbDurationMs = 400U;
inline constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.01f;
inline constexpr float kStairClimbThetaNearZeroThresholdRad = 0.08f;
inline constexpr std::uint32_t kStairClimbPitchStableMs = 300U;
}  // namespace chassis_fsm

namespace chassis {
inline constexpr float kControlDtS = 0.002f;
inline constexpr float kLegL1M = 0.215f;
inline constexpr float kLegL2M = 0.254f;
inline constexpr float kSpringTorqueScale = 90.0f;
inline constexpr float kSpringModelA = 1082.0f;
inline constexpr float kSpringModelB = 1070.0f;
inline constexpr float kSpringModelC = 404.0f;
inline constexpr float kSpringModelD = 177.0f;
inline constexpr float kSpringPhaseDivisor = 18.0f;
inline constexpr float kBodyMassKg = 22.0f;
inline constexpr float kLegMassKg = 2.3f;
inline constexpr float kGravityMps2 = 9.81f;
inline constexpr float kWheelRadiusM = 0.2025f;
inline constexpr float kOffGroundSupportForceThresholdN = 10.0f;
inline constexpr float kRollBalanceTargetRad = 0.003f;
inline constexpr float kPostureThetaBMinRad = -0.7f;
inline constexpr float kPostureThetaBMaxRad = 0.7f;
inline constexpr float kPostureThetaLegMinRad = -0.8f;
inline constexpr float kPostureThetaLegMaxRad = 1.4f;
inline constexpr float kLegRecoverThetaDotTarget = -2.0f;
inline constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;
inline constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;

inline constexpr std::array<float, 24> kEtaLookupLegLengthM{
    0.11f, 0.12f, 0.13f, 0.14f, 0.15f, 0.16f, 0.17f, 0.18f, 0.19f, 0.20f, 0.21f, 0.22f,
    0.23f, 0.24f, 0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.30f, 0.31f, 0.32f, 0.33f, 0.34f,
};

inline constexpr std::array<float, 24> kEtaLookupLwM{
    0.061990f, 0.067466f, 0.072986f, 0.078550f, 0.084158f, 0.089810f, 0.095506f, 0.101246f,
    0.107030f, 0.112858f, 0.118730f, 0.124646f, 0.130606f, 0.136610f, 0.142658f, 0.148750f,
    0.154886f, 0.161066f, 0.167290f, 0.173558f, 0.179870f, 0.186226f, 0.192626f, 0.199070f,
};

inline constexpr std::array<float, 240> kCtrlP{
    -3.5024, -20.434, 16.164,  29.78,   -8.7177, -15.875, -6.0918,  -26.543,  26.454,   41.571,  -20.951, -24.434,
    -4.4068, 16.39,   -5.7828, -22.467, 5.7074,  7.3426,  -4.1161,  16.022,   -6.1749,  -21.125, 5.1843,  8.0634,
    -10.34,  -74.262, 17.554,  64.834,  -3.8299, -22.831, -0.68675, -6.1307,  2.6783,   -4.4923, 1.9695,  -3.732,
    -4.5551, 9.9611,  -14.849, -7.6613, 15.522,  5.8546,  -0.3776,  -0.96791, -1.1757,  3.8583,  -6.8914, 0.4104,
    -16.887, 24.424,  25.88,   0.21622, -32.19,  -22.861, -2.4568,  2.1532,   5.8936,   2.172,   -6.3814, -5.8094,
    -3.5024, 16.164,  -20.434, -15.875, -8.7177, 29.78,   -6.0918,  26.454,   -26.543,  -24.434, -20.951, 41.571,
    4.4068,  5.7828,  -16.39,  -7.3426, -5.7074, 22.467,  4.1161,   6.1749,   -16.022,  -8.0634, -5.1843, 21.125,
    -4.5551, -14.849, 9.9611,  5.8546,  15.522,  -7.6613, -0.3776,  -1.1757,  -0.96791, 0.4104,  -6.8914, 3.8583,
    -10.34,  17.554,  -74.262, -22.831, -3.8299, 64.834,  -0.68675, 2.6783,   -6.1307,  -3.732,  1.9695,  -4.4923,
    -16.887, 25.88,   24.424,  -22.861, -32.19,  0.21622, -2.4568,  5.8936,   2.1532,   -5.8094, -6.3814, 2.172,
    5.4759,  4.7941,  -17.204, -28.302, 28.744,  8.1302,  8.4521,   5.743,    -28.18,   -40.319, 49.011,  12.657,
    -2.8212, -16.398, -10.398, 33.1,    -11.37,  14.635,  -2.6404,  -16.616,  -9.6277,  32.503,  -11.856, 13.207,
    33.354,  -34.068, 12.18,   36.983,  60.764,  -29.192, 1.8307,   0.24393,  -0.55967, 4.2165,  4.4077,  -1.2287,
    -2.0954, -28.065, -17.005, 48.626,  -47.745, -10.058, -0.24059, -0.87551, 2.7174,   -1.2892, 0.86761, -9.4508,
    -23.046, -107.61, 39.685,  122.64,  12.286,  -40.321, -1.4437,  -11.881,  2.9206,   10.397,  5.5588,  -3.487,
    5.4759,  -17.204, 4.7941,  8.1302,  28.744,  -28.302, 8.4521,   -28.18,   5.743,    12.657,  49.011,  -40.319,
    2.8212,  10.398,  16.398,  -14.635, 11.37,   -33.1,   2.6404,   9.6277,   16.616,   -13.207, 11.856,  -32.503,
    -2.0954, -17.005, -28.065, -10.058, -47.745, 48.626,  -0.24059, 2.7174,   -0.87551, -9.4508, 0.86761, -1.2892,
    33.354,  12.18,   -34.068, -29.192, 60.764,  36.983,  1.8307,   -0.55967, 0.24393,  -1.2287, 4.4077,  4.2165,
    -23.046, 39.685,  -107.61, -40.321, 12.286,  122.64,  -1.4437,  2.9206,   -11.881,  -3.487,  5.5588,  10.397,
};

inline constexpr PidGains kLeftL0Pid{5900.0f, 0.04f, 95500.0f, 170.0f, 10.0f};
inline constexpr PidGains kRightL0Pid{5900.0f, 0.04f, 95500.0f, 170.0f, 10.0f};
inline constexpr PidGains kLeftL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};
inline constexpr PidGains kRightL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};
inline constexpr PidGains kLeftL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};
inline constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};
inline constexpr PidGains kRollPid{600.0f, 0.0f, 200.0f, 180.0f, 0.0f};
inline constexpr PidGains kLeftLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};
inline constexpr PidGains kRightLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};
}  // namespace chassis

namespace control_loop {
inline constexpr std::int16_t kWheelSpinThreshold = 220;
inline constexpr std::int16_t kWheelActionThreshold = 320;
inline constexpr std::int16_t kWheelCenterThreshold = 80;
inline constexpr float kControlLoopDtS = 0.002f;
inline constexpr std::int16_t kDr16AxisMaxAbs = 660;
inline constexpr float kTargetForwardSpeedMinMps = 1.5f;
inline constexpr float kTargetForwardSpeedMaxMps = 1.8f;
inline constexpr float kVxInputDeadbandNorm = 0.1f;
inline constexpr float kVyInputDeadbandNorm = 0.1f;
inline constexpr float kLockPointEnterSpeedThresholdMps = 0.30f;
inline constexpr float kLockPointExitSpeedThresholdMps = 0.55f;
inline constexpr float kLockPointEnterInputThreshold = 0.08f;
inline constexpr float kLockPointExitInputThreshold = 0.12f;
inline constexpr std::uint32_t kLockPointMinDwellTicks = 100U;
inline constexpr float kLockPointAlphaRiseStep = 0.015f;
inline constexpr float kLockPointAlphaFallStep = 0.018f;
inline constexpr float kRcStickMax = 660.0f;
inline constexpr float kRcYawRateMaxRadS = -2.5f;
inline constexpr float kRcPitchRateMaxRadS = 1.5f;
inline constexpr float kPitchTargetMinRad = -0.35f;
inline constexpr float kPitchTargetMaxRad = 0.25f;
inline constexpr float kYawFollowRampStepRadS = 0.05f;
inline constexpr float kSpinYawRampStepRadS = 0.05f;
inline constexpr float kSpinTargetYawDotRadS = 6.0f;
inline constexpr float kSpinTranslationGain = 1.0f;
inline constexpr float kSpinThetaLlBiasRad = 0.01f;
inline constexpr float kYawFollowFixedTargetRad = -1.72f;
inline constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;
inline constexpr float kGimbalStartupYawAlignErrorRad = 0.04f;
inline constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;
inline constexpr std::uint32_t kGimbalStartupYawAlignStableTicks = 50U;
inline constexpr float kYawFollowDriveReadyErrorRad = 0.04f;
inline constexpr float kYawFollowDriveReadyVelRadS = 0.25f;
inline constexpr std::uint32_t kYawFollowDriveReadyStableTicks = 50U;
inline constexpr float kExpectedThetaLlBiasRad = 0.0f;
inline constexpr float kExpectedThetaLrBiasRad = 0.0f;
inline constexpr float kExpectedThetaBBiasRad = 0.0f;

struct SdotRampParams {
  float accel_step;
  float brake_step;
};

inline constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.008f};
inline constexpr SdotRampParams kSdotRampMidLeg{0.006f, 0.003f};
inline constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};

inline constexpr PidGains kYawFollowPid{8.0f, 0.0f, 1.2f, 6.0f, 0.0f};
}  // namespace control_loop

namespace actuators {
inline constexpr float kLeftWheelTorqueToCurrent = 2500.0f;
inline constexpr float kRightWheelTorqueToCurrent = 2300.0f;
inline constexpr float kWheelCurrentClampAbs = 16000.0f;
}  // namespace actuators

namespace gimbal_can_bridge {
inline constexpr std::uint16_t kRxStdId = 0x119;
inline constexpr std::size_t kPayloadSize = 4U;
inline constexpr float kMilliScale = 0.001f;
}  // namespace gimbal_can_bridge

namespace state_estimator {
inline constexpr float kDefaultDtS = 0.002f;
inline constexpr float kDefaultExpectedSdotMps = 0.05f;
inline constexpr float kLegL1M = 0.215f;
inline constexpr float kLegL2M = 0.254f;
inline constexpr float kWheelRadiusM = 0.0575f;
inline constexpr float kWheelReductionRatio = 17.0f / 268.0f;
inline constexpr float kMaxValidSpeedMps = 8.0f;
inline constexpr float kLeftPhi1OffsetRad = kPi - 2.94f;
inline constexpr float kLeftPhi4OffsetRad = 0.59f;
inline constexpr float kRightPhi1OffsetRad = kPi + 2.4f;
inline constexpr float kRightPhi4OffsetRad = -1.87f;
inline constexpr float kThetaDotFilterCutoffHz = 8.0f;
inline constexpr float kImuAccelFilterSampleHz = 500.0f;
inline constexpr float kImuAccelFilterCutoffHz = 10.0f;
inline constexpr std::uint32_t kAccelBiasInitSamples = 1500U;
inline constexpr float kAccelZeroWheelSpeedThresholdMps = 0.02f;
inline constexpr float kAccelZeroHighThresholdMps2 = 0.5f;
inline constexpr float kAccelZeroLowThresholdMps2 = 0.2f;
inline constexpr float kKalmanMinVariance = 1e-5f;
inline constexpr float kThetaPiHalf = 1.57079632679489661923f;

inline constexpr std::array<float, 4> kKalmanF{1.0f, kDefaultDtS, 0.0f, 1.0f};
inline constexpr std::array<float, 4> kKalmanQ{0.0005f, 0.0f, 0.0f, 0.04f};
inline constexpr std::array<float, 4> kKalmanR{0.5f, 0.0f, 0.0f, 2.0f};
inline constexpr std::array<float, 4> kKalmanP{10.0f, 0.0f, 0.0f, 10.0f};
inline constexpr std::array<float, 4> kKalmanH{1.0f, 0.0f, 0.0f, 1.0f};
}  // namespace state_estimator

namespace leg_kinematics {
inline constexpr float kDefaultDtS = 0.002f;
inline constexpr float kMinSin = 1e-5f;
inline constexpr float kMinLen = 1e-5f;
}  // namespace leg_kinematics

}  // namespace hero

namespace infantry3 {

constexpr float kPi = 3.14159265358979323846f;

namespace main {
inline constexpr float kControlLoopFrequencyHz = 500.0f;
}

namespace globals {
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

inline constexpr double kJointCanTxLimitHz = 4000.0;
inline constexpr double kWheelCanTxLimitHz = 4000.0;
inline constexpr double kGimbalCanTxLimitHz = 4000.0;

inline constexpr std::uint16_t kDmLfMasterId = 0x17;
inline constexpr std::uint16_t kDmLfSlaveId = 0x07;
inline constexpr std::uint16_t kDmLbMasterId = 0x14;
inline constexpr std::uint16_t kDmLbSlaveId = 0x04;
inline constexpr std::uint16_t kDmRfMasterId = 0x16;
inline constexpr std::uint16_t kDmRfSlaveId = 0x06;
inline constexpr std::uint16_t kDmRbMasterId = 0x15;
inline constexpr std::uint16_t kDmRbSlaveId = 0x05;

inline constexpr std::uint16_t kLeftWheelId = 0x06;
inline constexpr std::uint16_t kRightWheelId = 0x05;

inline constexpr std::size_t kDr16UartRxBufferSize = 18;
inline constexpr std::size_t kImuUartRxBufferSize = 518;

inline const DmMitSettings kDmLfSettings{kDmLfMasterId, kDmLfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
inline const DmMitSettings kDmLbSettings{kDmLbMasterId, kDmLbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
inline const DmMitSettings kDmRfSettings{kDmRfMasterId, kDmRfSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
inline const DmMitSettings kDmRbSettings{kDmRbMasterId, kDmRbSlaveId, kPi, 45.0f, 54.0f, {0.0f, 500.0f}, {0.0f, 10.0f}};
}  // namespace globals

namespace gimbal {
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

inline const DmMitSettings kPitchMotorSettings{0x12, 0x11, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
inline const DmMitSettings kYawMotorSettings{0x13, 0x03, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};

inline constexpr float kDefaultDtS = 0.002f;
inline constexpr float kPitchMinRad = -0.2f;
inline constexpr float kPitchMaxRad = 0.25f;
inline constexpr float kDmTorqueLimitNm = 10.0f;
inline constexpr float kPitchGravityCompensationNm = 1.3f;

inline constexpr PidGains kYawPositionPid{15.0f, 0.0f, 0.05f, 10.0f, 1.0f};
inline constexpr PidGains kYawSpeedPid{0.6f, 0.0f, 0.0f, 6.0f, 0.4f};
inline constexpr PidGains kPitchPositionPid{13.0f, 0.0f, 0.05f, 10.0f, 0.4f};
inline constexpr PidGains kPitchSpeedPid{0.85f, 0.0f, 0.0f, 8.0f, 0.0f};
}  // namespace gimbal

namespace chassis_fsm {
inline constexpr std::uint32_t kJumpPrepMs = 450U;
inline constexpr std::uint32_t kJumpPushMaxMs = 1000U;
inline constexpr std::uint32_t kJumpRecoverMs = 450U;
inline constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;
inline constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;

inline constexpr float kLowLegLengthM = 0.15f;
inline constexpr float kMidLegLengthM = 0.21f;
inline constexpr float kHighLegLengthM = 0.3f;
inline constexpr float kJumpPrepLegLengthM = 0.13f;
inline constexpr float kJumpPushLegLengthM = 0.36f;
inline constexpr float kJumpRecoverLegLengthM = 0.20f;
inline constexpr float kJumpPushReachedLegLengthM = 0.30f;
inline constexpr float kLegLengthRampTimeS = 0.5f;
inline constexpr float kStairClimbThetaThresholdRad = 0.5f;
inline constexpr float kStairClimbLegLengthM = 0.14f;
inline constexpr float kStairClimbThetaTargetRad = 0.2f;
inline constexpr std::uint32_t kStairClimbDurationMs = 3000U;
inline constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.03f;
inline constexpr float kStairClimbThetaNearZeroThresholdRad = 0.1f;
inline constexpr std::uint32_t kStairClimbPitchStableMs = 1000U;
}  // namespace chassis_fsm

namespace chassis {
inline constexpr float kControlDtS = 0.002f;
inline constexpr float kLegL1M = 0.215f;
inline constexpr float kLegL2M = 0.254f;
inline constexpr float kSpringTorqueScale = 90.0f;
inline constexpr float kSpringModelA = 1082.0f;
inline constexpr float kSpringModelB = 1070.0f;
inline constexpr float kSpringModelC = 404.0f;
inline constexpr float kSpringModelD = 177.0f;
inline constexpr float kSpringPhaseDivisor = 18.0f;
inline constexpr float kBodyMassKg = 22.0f;
inline constexpr float kLegMassKg = 2.3f;
inline constexpr float kGravityMps2 = 9.81f;
inline constexpr float kWheelRadiusM = 0.2025f;
inline constexpr float kOffGroundSupportForceThresholdN = 10.0f;
inline constexpr float kRollBalanceTargetRad = 0.003f;
inline constexpr float kPostureThetaBMinRad = -0.7f;
inline constexpr float kPostureThetaBMaxRad = 0.7f;
inline constexpr float kPostureThetaLegMinRad = -0.8f;
inline constexpr float kPostureThetaLegMaxRad = 1.4f;
inline constexpr float kLegRecoverThetaDotTarget = -2.0f;
inline constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;
inline constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;

inline constexpr std::array<float, 24> kEtaLookupLegLengthM{
    0.11f, 0.12f, 0.13f, 0.14f, 0.15f, 0.16f, 0.17f, 0.18f, 0.19f, 0.20f, 0.21f, 0.22f,
    0.23f, 0.24f, 0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.30f, 0.31f, 0.32f, 0.33f, 0.34f,
};

inline constexpr std::array<float, 24> kEtaLookupLwM{
    0.061990f, 0.067466f, 0.072986f, 0.078550f, 0.084158f, 0.089810f, 0.095506f, 0.101246f,
    0.107030f, 0.112858f, 0.118730f, 0.124646f, 0.130606f, 0.136610f, 0.142658f, 0.148750f,
    0.154886f, 0.161066f, 0.167290f, 0.173558f, 0.179870f, 0.186226f, 0.192626f, 0.199070f,
};

inline constexpr std::array<float, 240> kCtrlP{
    -3.8622,  -34.977,  26.957,   54.988,  -22.639,  -23.131, -5.9075,   -38.957,  35.534,   63.637,  -35.357, -29.517,
    -0.72515, 3.3062,   -1.3834,  -3.0416, -0.24575, 2.2176,  -1.6546,   7.8217,   -3.5107,  -6.8318, -1.0017, 5.7146,
    -9.3486,  -78.037,  17.853,   60.654,  10.775,   -25.465, -0.68625,  -6.9934,  3.1576,   -4.8071, 3.2371,  -4.3488,
    -3.2658,  -3.545,   -5.0815,  25.665,  -23.385,  3.9157,  -0.31975,  -2.3667,  0.055362, 7.743,   -13.066, 1.13,
    -19.109,  18.638,   31.343,   13.681,  -31.121,  -30.06,  -2.5824,   0.93204,  6.9693,   4.2123,  -6.61,   -7.0455,
    -3.8622,  26.957,   -34.977,  -23.131, -22.639,  54.988,  -5.9075,   35.534,   -38.957,  -29.517, -35.357, 63.637,
    0.72515,  1.3834,   -3.3062,  -2.2176, 0.24575,  3.0416,  1.6546,    3.5107,   -7.8217,  -5.7146, 1.0017,  6.8318,
    -3.2658,  -5.0815,  -3.545,   3.9157,  -23.385,  25.665,  -0.31975,  0.055362, -2.3667,  1.13,    -13.066, 7.743,
    -9.3486,  17.853,   -78.037,  -25.465, 10.775,   60.654,  -0.68625,  3.1576,   -6.9934,  -4.3488, 3.2371,  -4.8071,
    -19.109,  31.343,   18.638,   -30.06,  -31.121,  13.681,  -2.5824,   6.9693,   0.93204,  -7.0455, -6.61,   4.2123,
    11.269,   1.9612,   -24.113,  -63.647, 54.893,   19.76,   15.012,    2.8409,   -38.659,  -80.543, 81.58,   29.503,
    -0.69205, -5.1575,  -1.2283,  8.8769,  -2.4532,  2.2769,  -1.5753,   -12.468,  -2.5165,  21.217,  -6.1915, 4.821,
    44.479,   -40.036,  -0.31607, 0.51685, 51.977,   -11.71,  2.5173,    3.5621,   -2.6662,  -6.7894, 8.0897,  1.514,
    -0.86784, -23.451,  -2.9857,  14.321,  -35.572,  13.886,  -0.084864, 0.94216,  3.8353,   -8.0855, -0.4502, -2.6056,
    -25.916,  -156.46,  39.544,   170.86,  41.97,    -57.711, -0.79292,  -15.828,  1.1672,   12.21,   11.192,  -3.5419,
    11.269,   -24.113,  1.9612,   19.76,   54.893,   -63.647, 15.012,    -38.659,  2.8409,   29.503,  81.58,   -80.543,
    0.69205,  1.2283,   5.1575,   -2.2769, 2.4532,   -8.8769, 1.5753,    2.5165,   12.468,   -4.821,  6.1915,  -21.217,
    -0.86784, -2.9857,  -23.451,  13.886,  -35.572,  14.321,  -0.084864, 3.8353,   0.94216,  -2.6056, -0.4502, -8.0855,
    44.479,   -0.31607, -40.036,  -11.71,  51.977,   0.51685, 2.5173,    -2.6662,  3.5621,   1.514,   8.0897,  -6.7894,
    -25.916,  39.544,   -156.46,  -57.711, 41.97,    170.86,  -0.79292,  1.1672,   -15.828,  -3.5419, 11.192,  12.21,
};

inline constexpr PidGains kLeftL0Pid{7500.0f, 0.04f, 90000.0f, 170.0f, 10.0f};
inline constexpr PidGains kRightL0Pid{7500.0f, 0.04f, 90000.0f, 170.0f, 10.0f};
inline constexpr PidGains kLeftL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};
inline constexpr PidGains kRightL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};
inline constexpr PidGains kLeftL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};
inline constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};
inline constexpr PidGains kRollPid{600.0f, 0.0f, 200.0f, 180.0f, 0.0f};
inline constexpr PidGains kLeftLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};
inline constexpr PidGains kRightLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};
}  // namespace chassis

namespace control_loop {
inline constexpr std::int16_t kWheelSpinThreshold = 220;
inline constexpr std::int16_t kWheelActionThreshold = 320;
inline constexpr std::int16_t kWheelCenterThreshold = 80;
inline constexpr float kControlLoopDtS = 0.002f;
inline constexpr std::int16_t kDr16AxisMaxAbs = 660;
inline constexpr float kTargetForwardSpeedMinMps = 2.1f;
inline constexpr float kTargetForwardSpeedMaxMps = 2.1f;
inline constexpr float kVxInputDeadbandNorm = 0.1f;
inline constexpr float kVyInputDeadbandNorm = 0.1f;
inline constexpr float kLockPointEnterSpeedThresholdMps = 0.30f;
inline constexpr float kLockPointExitSpeedThresholdMps = 0.55f;
inline constexpr float kLockPointEnterInputThreshold = 0.08f;
inline constexpr float kLockPointExitInputThreshold = 0.12f;
inline constexpr std::uint32_t kLockPointMinDwellTicks = 100U;
inline constexpr float kLockPointAlphaRiseStep = 0.015f;
inline constexpr float kLockPointAlphaFallStep = 0.018f;
inline constexpr float kRcStickMax = 660.0f;
inline constexpr float kRcYawRateMaxRadS = -2.5f;
inline constexpr float kRcPitchRateMaxRadS = 1.5f;
inline constexpr float kPitchTargetMinRad = -0.35f;
inline constexpr float kPitchTargetMaxRad = 0.25f;
inline constexpr float kYawFollowRampStepRadS = 0.05f;
inline constexpr float kSpinYawRampStepRadS = 0.05f;
inline constexpr float kSpinTargetYawDotRadS = 6.0f;
inline constexpr float kSpinTranslationGain = 1.0f;
inline constexpr float kSpinThetaLlBiasRad = 0.01f;
inline constexpr float kYawFollowFixedTargetRad = -1.72f;
inline constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;
inline constexpr float kGimbalStartupYawAlignErrorRad = 0.04f;
inline constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;
inline constexpr std::uint32_t kGimbalStartupYawAlignStableTicks = 50U;
inline constexpr float kYawFollowDriveReadyErrorRad = 0.04f;
inline constexpr float kYawFollowDriveReadyVelRadS = 0.25f;
inline constexpr std::uint32_t kYawFollowDriveReadyStableTicks = 50U;
inline constexpr float kExpectedThetaLlBiasRad = 0.13f;
inline constexpr float kExpectedThetaLrBiasRad = 0.13f;
inline constexpr float kExpectedThetaBBiasRad = 0.0f;

struct SdotRampParams {
  float accel_step;
  float brake_step;
};

inline constexpr SdotRampParams kSdotRampLowLeg{0.008f, 0.008f};
inline constexpr SdotRampParams kSdotRampMidLeg{0.006f, 0.006f};
inline constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};

inline constexpr PidGains kYawFollowPid{8.0f, 0.0f, 1.2f, 6.0f, 0.0f};
}  // namespace control_loop

namespace actuators {
inline constexpr float kLeftWheelTorqueToCurrent = 2500.0f;
inline constexpr float kRightWheelTorqueToCurrent = 2300.0f;
inline constexpr float kWheelCurrentClampAbs = 16000.0f;
}  // namespace actuators

namespace gimbal_can_bridge {
inline constexpr std::uint16_t kRxStdId = 0x119;
inline constexpr std::size_t kPayloadSize = 4U;
inline constexpr float kMilliScale = 0.001f;
}  // namespace gimbal_can_bridge

namespace state_estimator {
inline constexpr float kDefaultDtS = 0.002f;
inline constexpr float kDefaultExpectedSdotMps = 0.05f;
inline constexpr float kLegL1M = 0.215f;
inline constexpr float kLegL2M = 0.254f;
inline constexpr float kWheelRadiusM = 0.0575f;
inline constexpr float kWheelReductionRatio = 17.0f / 268.0f;
inline constexpr float kMaxValidSpeedMps = 8.0f;
inline constexpr float kLeftPhi1OffsetRad = kPi - 2.94f;
inline constexpr float kLeftPhi4OffsetRad = 0.59f;
inline constexpr float kRightPhi1OffsetRad = kPi + 2.4f;
inline constexpr float kRightPhi4OffsetRad = -1.87f;
inline constexpr float kThetaDotFilterCutoffHz = 8.0f;
inline constexpr float kImuAccelFilterSampleHz = 500.0f;
inline constexpr float kImuAccelFilterCutoffHz = 10.0f;
inline constexpr std::uint32_t kAccelBiasInitSamples = 1500U;
inline constexpr float kAccelZeroWheelSpeedThresholdMps = 0.02f;
inline constexpr float kAccelZeroHighThresholdMps2 = 0.5f;
inline constexpr float kAccelZeroLowThresholdMps2 = 0.2f;
inline constexpr float kKalmanMinVariance = 1e-5f;
inline constexpr float kThetaPiHalf = 1.57079632679489661923f;

inline constexpr std::array<float, 4> kKalmanF{1.0f, kDefaultDtS, 0.0f, 1.0f};
inline constexpr std::array<float, 4> kKalmanQ{0.0005f, 0.0f, 0.0f, 0.04f};
inline constexpr std::array<float, 4> kKalmanR{0.5f, 0.0f, 0.0f, 2.0f};
inline constexpr std::array<float, 4> kKalmanP{10.0f, 0.0f, 0.0f, 10.0f};
inline constexpr std::array<float, 4> kKalmanH{1.0f, 0.0f, 0.0f, 1.0f};
}  // namespace state_estimator

namespace leg_kinematics {
inline constexpr float kDefaultDtS = 0.002f;
inline constexpr float kMinSin = 1e-5f;
inline constexpr float kMinLen = 1e-5f;
}  // namespace leg_kinematics

}  // namespace infantry3

namespace infantry4 {

constexpr float kPi = 3.14159265358979323846f;

namespace main {
inline constexpr float kControlLoopFrequencyHz = 500.0f;
}

namespace globals {
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

inline constexpr double kJointCanTxLimitHz = 4000.0;   ///< 閸忓疇濡?CAN 閸欐垿鈧線顣堕悳鍥︾瑐闂?
inline constexpr double kWheelCanTxLimitHz = 4000.0;   ///< 鏉烆喗鐦?CAN 閸欐垿鈧線顣堕悳鍥︾瑐闂?
inline constexpr double kGimbalCanTxLimitHz = 4000.0;  ///< 娴滄垵褰?CAN 閸欐垿鈧線顣堕悳鍥︾瑐闂?

inline constexpr std::uint16_t kDmLfMasterId = 0x02;  ///< 瀹革箑澧犻懙澶稿瘜閻㈠灚婧€ ID
inline constexpr std::uint16_t kDmLfSlaveId = 0x01;   ///< 瀹革箑澧犻懙澶哥矤閻㈠灚婧€ ID
inline constexpr std::uint16_t kDmLbMasterId = 0x04;  ///< 瀹革箑鎮楅懙澶稿瘜閻㈠灚婧€ ID
inline constexpr std::uint16_t kDmLbSlaveId = 0x03;   ///< 瀹革箑鎮楅懙澶哥矤閻㈠灚婧€ ID
inline constexpr std::uint16_t kDmRfMasterId = 0x06;  ///< 閸欏啿澧犻懙澶稿瘜閻㈠灚婧€ ID
inline constexpr std::uint16_t kDmRfSlaveId = 0x05;   ///< 閸欏啿澧犻懙澶哥矤閻㈠灚婧€ ID
inline constexpr std::uint16_t kDmRbMasterId = 0x08;  ///< 閸欏啿鎮楅懙澶稿瘜閻㈠灚婧€ ID
inline constexpr std::uint16_t kDmRbSlaveId = 0x07;   ///< 閸欏啿鎮楅懙澶哥矤閻㈠灚婧€ ID

inline constexpr std::uint16_t kLeftWheelId = 0x06;   ///< 瀹革箒鐤嗗В鍌滄暩閺?ID
inline constexpr std::uint16_t kRightWheelId = 0x05;  ///< 閸欏疇鐤嗗В鍌滄暩閺?ID

inline constexpr std::size_t kDr16UartRxBufferSize =
    18;  ///< DR16 闁儲甯堕崳銊よ閸欙絾甯撮弨鍓佺处閸愭彃銇囩亸?
inline constexpr std::size_t kImuUartRxBufferSize = 518;  ///< IMU 娑撴彃褰涢幒銉︽暪缂傛挸鍟挎径褍鐨?

inline const DmMitSettings kDmLfSettings{
    kDmLfMasterId, kDmLfSlaveId,   kPi,          45.0f,
    54.0f,         {0.0f, 500.0f}, {0.0f, 10.0f}};  ///< 瀹革箑澧犻懙?DM 閻㈠灚婧€閸欏倹鏆?
inline const DmMitSettings kDmLbSettings{
    kDmLbMasterId, kDmLbSlaveId,   kPi,          45.0f,
    54.0f,         {0.0f, 500.0f}, {0.0f, 10.0f}};  ///< 瀹革箑鎮楅懙?DM 閻㈠灚婧€閸欏倹鏆?
inline const DmMitSettings kDmRfSettings{
    kDmRfMasterId, kDmRfSlaveId,   kPi,          45.0f,
    54.0f,         {0.0f, 500.0f}, {0.0f, 10.0f}};  ///< 閸欏啿澧犻懙?DM 閻㈠灚婧€閸欏倹鏆?
inline const DmMitSettings kDmRbSettings{
    kDmRbMasterId, kDmRbSlaveId,   kPi,          45.0f,
    54.0f,         {0.0f, 500.0f}, {0.0f, 10.0f}};  ///< 閸欏啿鎮楅懙?DM 閻㈠灚婧€閸欏倹鏆?
}  // namespace globals

namespace gimbal {
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

inline const DmMitSettings kPitchMotorSettings{
    0x05, 0x04, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};  ///< 娣囶垯璇濋悽鍨簚閸欏倹鏆?
inline const DmMitSettings kYawMotorSettings{
    0x10, 0x09, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};  ///< 閸嬪繗鍩呴悽鍨簚閸欏倹鏆?

inline constexpr float kDefaultDtS = 0.002f;  ///< 娴滄垵褰撮幒褍鍩楁妯款吇閺冨爼妫垮銉╂毐
inline constexpr float kPitchMinRad = -0.2f;  ///< 娣囶垯璇濈憴鎺嶇瑓闂?
inline constexpr float kPitchMaxRad = 0.25f;  ///< 娣囶垯璇濈憴鎺嶇瑐闂?
inline constexpr float kDmTorqueLimitNm = 10.0f;            ///< DM 閻㈠灚婧€閸旀稓鐓╅梽鎰煑
inline constexpr float kPitchGravityCompensationNm = 1.3f;  ///< 娣囶垯璇濋柌宥呭鐞涖儱浼╅崝娑氱叐

inline constexpr PidGains kYawPositionPid{25.0f, 0.0f, 0.05f, 10.0f, 1.0f};    ///< 閸嬪繗鍩呮担宥囩枂 PID
inline constexpr PidGains kYawSpeedPid{0.6f, 0.0f, 0.0f, 6.0f, 0.4f};          ///< 閸嬪繗鍩呴柅鐔峰 PID
inline constexpr PidGains kPitchPositionPid{26.0f, 0.0f, 0.05f, 10.0f, 0.4f};  ///< 娣囶垯璇濇担宥囩枂 PID
inline constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 8.0f, 0.0f};       ///< 娣囶垯璇濋柅鐔峰 PID
}  // namespace gimbal

namespace chassis_fsm {
inline constexpr std::uint32_t kJumpPrepMs = 450U;      ///< 鐠哄疇绌０鍕槵闂冭埖顔岄弮鍫曟毐
inline constexpr std::uint32_t kJumpPushMaxMs = 1000U;  ///< 鐠哄疇绌煫顑垮嚑閺堚偓婢堆勬闂€?
inline constexpr std::uint32_t kJumpRecoverMs = 450U;   ///< 鐠哄疇绌崶鐐存暪闂冭埖顔岄弮鍫曟毐
inline constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;        ///< 閸婃帒婀寸涵顔款吇閺冨爼鏆?
inline constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;  ///< 閼奉亝浠径宥堢Т閺冭埖妞傞梹?

inline constexpr float kLowLegLengthM = 0.15f;              ///< 娴ｅ氦鍚欓梹璺ㄦ窗閺?
inline constexpr float kMidLegLengthM = 0.20f;              ///< 娑擃叀鍚欓梹璺ㄦ窗閺?
inline constexpr float kHighLegLengthM = 0.35f;             ///< 妤傛鍚欓梹璺ㄦ窗閺?
inline constexpr float kJumpPrepLegLengthM = 0.13f;         ///< 鐠哄疇绌０鍕槵閼靛潡鏆?
inline constexpr float kJumpPushLegLengthM = 0.36f;         ///< 鐠哄疇绌煫顑垮嚑閼靛潡鏆?
inline constexpr float kJumpRecoverLegLengthM = 0.20f;      ///< 鐠哄疇绌崶鐐存暪閼靛潡鏆?
inline constexpr float kJumpPushReachedLegLengthM = 0.30f;  ///< 闊兛鍑犻崚銈呯暰閼靛潡鏆遍梼鍫濃偓?
inline constexpr float kLegLengthRampTimeS = 0.5f;          ///< 閼靛潡鏆遍崚鍥ㄥ床閺傛粌娼弮鍫曟？

inline constexpr float kStairClimbThetaThresholdRad =
    0.5f;  ///< 娑撳﹤褰撮梼鎯板悪閹藉棜顫楀Λ鈧ù瀣閸?//
inline constexpr float kStairClimbLegLengthM = 0.16f;  ///< 娑撳﹤褰撮梼鑸垫暪閼佃法娲伴弽?//
inline constexpr float kStairClimbThetaTargetRad =
    0.2f;  ///< 娑撳﹤褰撮梼鎯板悪閹藉棜顫楅惄顔界垼///---------------娑撳﹤褰撮梼璺哄棘閺?
inline constexpr std::uint32_t kStairClimbDurationMs =
    250U;  ///< 娑撳﹤褰撮梼鑸垫暪閼靛じ绻氶幐浣规闂€?//
inline constexpr float kStairClimbLegLengthNearTargetToleranceM =
    0.01f;  ///< 娑撳﹤褰撮梼鎯板悪闂€鎸庡复鏉╂垹娲伴弽鍥у灲鐎规岸妲囬崐?
inline constexpr float kStairClimbThetaNearZeroThresholdRad =
    0.08f;  ///< 娑撳﹤褰撮梼鎯板悪閹藉棜顫楅崶鐐搭劀閸掋倕鐣鹃梼鍫濃偓?
inline constexpr std::uint32_t kStairClimbPitchStableMs = 2000U;  ///< 娑撳﹤褰撮梼?pitch 缁嬪啿鐣剧粵澶婄窡閺冨爼鏆?//
}  // namespace chassis_fsm

namespace chassis {
inline constexpr float kControlDtS = 0.002f;         ///< 鎼存洜娲忛幒褍鍩楅弮鍫曟？濮濄儵鏆?
inline constexpr float kLegL1M = 0.215f;             ///< 娴滄棁绻涢弶鍡曠瑐濞堢敻鏆辨惔?
inline constexpr float kLegL2M = 0.254f;             ///< 娴滄棁绻涢弶鍡曠瑓濞堢敻鏆辨惔?
inline constexpr float kSpringTorqueScale = 90.0f;   ///< 瀵湱鍝€閸旀稓鐓╃紓鈺傛杹缁粯鏆?
inline constexpr float kSpringModelA = 1082.0f;      ///< 瀵湱鍝€濡€崇€烽崣鍌涙殶 A
inline constexpr float kSpringModelB = 1070.0f;      ///< 瀵湱鍝€濡€崇€烽崣鍌涙殶 B
inline constexpr float kSpringModelC = 404.0f;       ///< 瀵湱鍝€濡€崇€烽崣鍌涙殶 C
inline constexpr float kSpringModelD = 177.0f;       ///< 瀵湱鍝€濡€崇€烽崣鍌涙殶 D
inline constexpr float kSpringPhaseDivisor = 18.0f;  ///< 瀵湱鍝€濡€崇€烽惄闀愮秴闂勩倖鏆?
inline constexpr float kBodyMassKg = 22.0f;          ///< 鏉烇箒闊╃拹銊╁櫤
inline constexpr float kLegMassKg = 2.3f;            ///< 閸楁洝鍚欑拹銊╁櫤
inline constexpr float kGravityMps2 = 9.81f;         ///< 闁插秴濮忛崝鐘烩偓鐔峰
inline constexpr float kWheelRadiusM = 0.2025f;      ///< 鏉烆喗鐦嬮崡濠傜窞
inline constexpr float kOffGroundSupportForceThresholdN =
    10.0f;  ///< 缁傝婀撮弨顖涙嫼閸旀稑鍨界€规岸妲囬崐?
inline constexpr float kRollBalanceTargetRad = 0.003f;  ///< 濡亝绮撮獮瀹犮€€閻╊喗鐖ｇ憴?
inline constexpr float kPostureThetaBMinRad = -0.7f;    ///< 婵寧鈧焦婀侀弫?body pitch 娑撳妾?
inline constexpr float kPostureThetaBMaxRad = 0.7f;     ///< 婵寧鈧焦婀侀弫?body pitch 娑撳﹪妾?
inline constexpr float kPostureThetaLegMinRad =
    -0.8f;  ///< 婵寧鈧焦婀侀弫鍫ｅ悪閹藉棜顫楁稉瀣
inline constexpr float kPostureThetaLegMaxRad =
    1.4f;  ///< 婵寧鈧焦婀侀弫鍫ｅ悪閹藉棜顫楁稉濠囨
inline constexpr float kLegRecoverThetaDotTarget =
    -2.0f;  ///< 婵寧鈧焦浠径宥堝悪閹藉棜顫楅柅鐔峰閻╊喗鐖?
inline constexpr float kLegRecoverZeroTorqueMinRad =
    0.0f;  ///< 婵寧鈧焦浠径宥夋祩閸旀稓鐓╅崠娲？娑撳妾?
inline constexpr float kLegRecoverZeroTorqueMaxRad =
    1.4f;  ///< 婵寧鈧焦浠径宥夋祩閸旀稓鐓╅崠娲？娑撳﹪妾?

inline constexpr std::array<float, 24> kEtaLookupLegLengthM{
    0.11f, 0.12f, 0.13f, 0.14f, 0.15f, 0.16f, 0.17f, 0.18f, 0.19f, 0.20f, 0.21f, 0.22f,
    0.23f, 0.24f, 0.25f, 0.26f, 0.27f, 0.28f, 0.29f, 0.30f, 0.31f, 0.32f, 0.33f, 0.34f,
};

inline constexpr std::array<float, 24> kEtaLookupLwM{
    0.061990f, 0.067466f, 0.072986f, 0.078550f, 0.084158f, 0.089810f, 0.095506f, 0.101246f,
    0.107030f, 0.112858f, 0.118730f, 0.124646f, 0.130606f, 0.136610f, 0.142658f, 0.148750f,
    0.154886f, 0.161066f, 0.167290f, 0.173558f, 0.179870f, 0.186226f, 0.192626f, 0.199070f,
};

inline constexpr std::array<float, 240> kCtrlP{
    -0.58029, -2.5528,  1.9518,   3.5789,   -0.46423, -2.3134,  -2.2005,  -8.3085,  7.3717,   11.92,    -2.4309,
    -8.6938,  -0.36695, 1.3072,   -0.44543, -2.1009,  0.8663,   0.5222,   -4.9317,  17.609,   -6.0003,  -28.277,
    11.705,   7.026,    -5.0754,  -53.86,   14.542,   55.914,   -29.793,  -15.026,  -0.80591, -3.8308,  1.3799,
    0.41048,  -0.95335, -1.8342,  -3.8339,  15.723,   -17.188,  -28.613,  50.598,   12.348,   -0.32047, 0.69908,
    -1.3299,  -1.1112,  1.37,     0.26423,  -9.88,    17.178,   14.247,   -1.7829,  -22.238,  -11.965,  -1.595,
    2.3575,   2.933,    0.46058,  -4.1163,  -2.8264,  -0.58029, 1.9518,   -2.5528,  -2.3134,  -0.46423, 3.5789,
    -2.2005,  7.3717,   -8.3085,  -8.6938,  -2.4309,  11.92,    0.36695,  0.44543,  -1.3072,  -0.5222,  -0.8663,
    2.1009,   4.9317,   6.0003,   -17.609,  -7.026,   -11.705,  28.277,   -3.8339,  -17.188,  15.723,   12.348,
    50.598,   -28.613,  -0.32047, -1.3299,  0.69908,  0.26423,  1.37,     -1.1112,  -5.0754,  14.542,   -53.86,
    -15.026,  -29.793,  55.914,   -0.80591, 1.3799,   -3.8308,  -1.8342,  -0.95335, 0.41048,  -9.88,    14.247,
    17.178,   -11.965,  -22.238,  -1.7829,  -1.595,   2.933,    2.3575,   -2.8264,  -4.1163,  0.46058,  0.73668,
    -0.40249, -1.5474,  -1.5327,  3.5188,   -0.26472, 2.6764,   -1.9126,  -5.7685,  -4.6106,  12.91,    -0.65183,
    -0.20696, -1.0711,  -0.92691, 2.5161,   -1.1594,  1.3171,   -2.7815,  -14.473,  -12.493,  33.929,   -15.668,
    17.725,   12.309,   25.414,   17.122,   -44.23,   78.374,   -34.888,  2.0647,   -4.7777,  1.1318,   9.032,
    5.1163,   -2.8717,  0.99522,  -35.057,  -33.63,   68.446,   -58.917,  12.468,   -0.3039,  -2.1717,  1.6912,
    4.1545,   -3.3152,  -7.498,   -14.79,   -62.535,  30.308,   72.853,   -0.7513,  -26.157,  -1.6154,  -9.2102,
    4.599,    9.6907,   1.0272,   -4.2425,  0.73668,  -1.5474,  -0.40249, -0.26472, 3.5188,   -1.5327,  2.6764,
    -5.7685,  -1.9126,  -0.65183, 12.91,    -4.6106,  0.20696,  0.92691,  1.0711,   -1.3171,  1.1594,   -2.5161,
    2.7815,   12.493,   14.473,   -17.725,  15.668,   -33.929,  0.99522,  -33.63,   -35.057,  12.468,   -58.917,
    68.446,   -0.3039,  1.6912,   -2.1717,  -7.498,   -3.3152,  4.1545,   12.309,   17.122,   25.414,   -34.888,
    78.374,   -44.23,   2.0647,   1.1318,   -4.7777,  -2.8717,  5.1163,   9.032,    -14.79,   30.308,   -62.535,
    -26.157,  -0.7513,  72.853,   -1.6154,  4.599,    -9.2102,  -4.2425,  1.0272,   9.6907,
};

inline constexpr PidGains kLeftL0Pid{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};   ///< 瀹革箒鍚欓梹鎸庡付閸?PID
inline constexpr PidGains kRightL0Pid{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};  ///< 閸欏疇鍚欓梹鎸庡付閸?PID
inline constexpr PidGains kLeftL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f,
                                            0.0f};  ///< 瀹革箒鍚欑捄瀹犵┈闊兛鍑?PID
inline constexpr PidGains kRightL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f,
                                             0.0f};  ///< 閸欏疇鍚欑捄瀹犵┈闊兛鍑?PID
inline constexpr PidGains kLeftL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f,
                                              30.0f};  ///< 瀹革箒鍚欑捄瀹犵┈閸ョ偞鏁?PID
inline constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f,
                                               30.0f};                 ///< 閸欏疇鍚欑捄瀹犵┈閸ョ偞鏁?PID
inline constexpr PidGains kRollPid{500.0f, 0.0f, 80.0f, 80.0f, 0.0f};  ///< 濡亝绮撮獮瀹犮€€ PID
inline constexpr PidGains kLeftLegTurnPid{20.0f, 0.0f, 0.0f, 15.0f,
                                          0.0f};  ///< 瀹革箒鍚欓幗鍡氼潡閹垹顦?PID
inline constexpr PidGains kRightLegTurnPid{20.0f, 0.0f, 0.0f, 15.0f,
                                           0.0f};  ///< 閸欏疇鍚欓幗鍡氼潡閹垹顦?PID
}  // namespace chassis

namespace control_loop {
inline constexpr std::int16_t kWheelSpinThreshold = 220;    ///< 閹枫劏鐤嗛懛顏呮闂冨牆鈧?
inline constexpr std::int16_t kWheelActionThreshold = 320;  ///< 閹枫劏鐤嗛崝銊ょ稊鐟欙箑褰傞梼鍫濃偓?
inline constexpr std::int16_t kWheelCenterThreshold = 80;   ///< 閹枫劏鐤嗚ぐ鎺嶈厬閸掋倕鐣鹃梼鍫濃偓?
inline constexpr float kControlLoopDtS = 0.002f;            ///< 娑撶粯甯堕崚璺烘儕閻滎垰鎳嗛張?
inline constexpr std::int16_t kDr16AxisMaxAbs = 660;        ///< DR16 閹藉洦娼岄張鈧径褏绮风€电懓鈧?
inline constexpr float kTargetForwardSpeedMinMps =
    1.5f;  ///< 閺堚偓鐏忓骏绱欑拹鐔锋倻閿涘娲伴弽鍥偓鐔峰楠炲懎鈧?
inline constexpr float kTargetForwardSpeedMaxMps = 1.8f;  ///< 閺堚偓婢堆冨鏉╂稓娲伴弽鍥偓鐔峰
inline constexpr float kVxInputDeadbandNorm =
    0.1f;  ///< 閸撳秷绻樻潏鎾冲弳濮濊灏敍鍫濈秺娑撯偓閸栨牭绱?
inline constexpr float kVyInputDeadbandNorm =
    0.1f;  ///< 娓氀冩倻鏉堟挸鍙嗗璇插隘閿涘牆缍婃稉鈧崠鏍电礆
inline constexpr float kLockPointEnterSpeedThresholdMps =
    0.30f;  ///< 鏉╂稑鍙嗙€规氨鍋ｉ柨浣哥暰闁喎瀹抽梼鍫濃偓?
inline constexpr float kLockPointExitSpeedThresholdMps =
    0.55f;  ///< 闁偓閸戝搫鐣鹃悙褰掓敚鐎规岸鈧喎瀹抽梼鍫濃偓?
inline constexpr float kLockPointEnterInputThreshold = 0.08f;  ///< 鏉╂稑鍙嗙€规氨鍋ｉ柨浣哥暰鏉堟挸鍙嗛梼鍫濃偓?
inline constexpr float kLockPointExitInputThreshold =
    0.12f;  ///< 闁偓閸戝搫鐣鹃悙褰掓敚鐎规俺绶崗銉╂閸?
inline constexpr std::uint32_t kLockPointMinDwellTicks = 100U;  ///< 鐎规氨鍋ｉ柨浣哥暰閺堚偓鐏忓繘鈹楅悾娆忔噯閺堢喐鏆?
inline constexpr float kLockPointAlphaRiseStep =
    0.015f;  ///< 鐎规氨鍋ｉ柨浣哥暰 alpha 娑撳﹤宕屽銉╂毐
inline constexpr float kLockPointAlphaFallStep =
    0.018f;                                   ///< 鐎规氨鍋ｉ柨浣哥暰 alpha 娑撳妾峰銉╂毐
inline constexpr float kRcStickMax = 660.0f;  ///< 闁儲甯堕崳銊︽啚閺夊棙娓舵径褍鈧?
inline constexpr float kRcPitchRateMaxRadS =
    1.5f;  ///< 闁儲甯堕崳銊ゅ垔娴犳媽顫楅柅鐔峰娑撳﹪妾?
inline constexpr float kRcYawRateMaxRadS =
    -2.5f;  ///< 闁儲甯堕崳銊ヤ焊閼割亣顫楅柅鐔峰娑撳﹪妾?
inline constexpr float kPitchTargetMinRad = -0.35f;     ///< 娣囶垯璇濋惄顔界垼娑撳妾?
inline constexpr float kPitchTargetMaxRad = 0.25f;      ///< 娣囶垯璇濋惄顔界垼娑撳﹪妾?
inline constexpr float kYawFollowRampStepRadS = 0.05f;  ///< 閸嬪繗鍩呯捄鐔兼閺傛粌娼銉╂毐
inline constexpr float kSpinYawRampStepRadS = 0.05f;    ///< spin yaw ramp step
inline constexpr float kSpinTargetYawDotRadS = 6.0f;    ///< spin target yaw rate
inline constexpr float kSpinTranslationGain = 1.0f;     ///< spin translation gain
inline constexpr float kSpinThetaLlBiasRad = 0.01f;     ///< spin theta_ll bias
inline constexpr float kYawFollowFixedTargetRad = 0.f;  ///< 閸嬪繗鍩呯捄鐔兼閸ュ搫鐣鹃惄顔界垼鐟?
inline constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;  ///< 閸嬪繗鍩呯捄鐔兼娓氀冩倻閸嬪繒些
inline constexpr float kGimbalStartupYawAlignErrorRad =
    0.04f;  ///< 娴滄垵褰撮崥顖氬З閸嬪繗鍩呯€电懓鍣拠顖氭▕闂冨牆鈧?
inline constexpr float kGimbalStartupYawAlignVelRadS =
    0.25f;  ///< 娴滄垵褰撮崥顖氬З閸嬪繗鍩呯€电懓鍣柅鐔峰闂冨牆鈧?
inline constexpr std::uint32_t kGimbalStartupYawAlignStableTicks =
    50U;  ///< 娴滄垵褰撮崥顖氬З閸嬪繗鍩呯€电懓鍣粙鍐茬暰閸涖劍婀￠弫?
inline constexpr float kYawFollowDriveReadyErrorRad =
    0.04f;  ///< 閸嬪繗鍩呯捄鐔兼鐏忚京鍗庣拠顖氭▕闂冨牆鈧?
inline constexpr float kYawFollowDriveReadyVelRadS =
    0.25f;  ///< 閸嬪繗鍩呯捄鐔兼鐏忚京鍗庨柅鐔峰闂冨牆鈧?
inline constexpr std::uint32_t kYawFollowDriveReadyStableTicks =
    50U;  ///< 閸嬪繗鍩呯捄鐔兼鐏忚京鍗庣粙鍐茬暰閸涖劍婀￠弫?
inline constexpr float kExpectedThetaLlBiasRad = 0.105f;  ///< 瀹革箒鍚欓幗鍡氼潡閺堢喐婀滈崑蹇曠枂
inline constexpr float kExpectedThetaLrBiasRad = 0.105f;  ///< 閸欏疇鍚欓幗鍡氼潡閺堢喐婀滈崑蹇曠枂
inline constexpr float kExpectedThetaBBiasRad = 0.05f;    ///< 鏉烇箒闊╂穱顖欒瘽閺堢喐婀滈崑蹇曠枂

struct SdotRampParams {
  float accel_step;
  float brake_step;
};

inline constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.008f};
inline constexpr SdotRampParams kSdotRampMidLeg{0.006f, 0.003f};
inline constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};

inline constexpr PidGains kYawFollowPid{8.0f, 0.0f, 1.2f, 6.0f, 0.0f};  ///< Yaw follow PID
}  // namespace control_loop

namespace actuators {
inline constexpr float kLeftWheelTorqueToCurrent = 2500.0f;
inline constexpr float kRightWheelTorqueToCurrent = 2300.0f;  ///< Right wheel torque-to-current factor
inline constexpr float kWheelCurrentClampAbs = 16000.0f;      ///< Wheel current clamp absolute value
}  // namespace actuators

namespace gimbal_can_bridge {
inline constexpr std::uint16_t kRxStdId = 0x119;  ///< 娴滄垵褰?CAN 閸欏秹顩幒銉︽暪閺嶅洤鍣敮?ID
inline constexpr std::size_t kPayloadSize = 4U;   ///< 娴滄垵褰?CAN 閸欏秹顩弫鐗堝祦闂€鍨
inline constexpr float kMilliScale = 0.001f;      ///< 娴滄垵褰?CAN 閸欏秹顩В顐㈠礋娴ｅ秶缂夐弨?
}  // namespace gimbal_can_bridge

namespace state_estimator {
inline constexpr float kDefaultDtS =
    0.002f;  ///< 閻樿埖鈧椒鍙婄拋锟犵帛鐠併倖妞傞梻瀛橆劄闂€?
inline constexpr float kDefaultExpectedSdotMps = 0.05f;        ///< 姒涙顓婚張鐔告箿闁喎瀹?
inline constexpr float kLegL1M = 0.215f;                       ///< 娴滄棁绻涢弶鍡曠瑐濞堢敻鏆辨惔?
inline constexpr float kLegL2M = 0.254f;                       ///< 娴滄棁绻涢弶鍡曠瑓濞堢敻鏆辨惔?
inline constexpr float kWheelRadiusM = 0.0575f;                ///< 鏉烆喗鐦嬮崡濠傜窞
inline constexpr float kWheelReductionRatio = 17.0f / 268.0f;  ///< 鏉烆喗鐦嬮崙蹇涒偓鐔哥槷
inline constexpr float kMaxValidSpeedMps = 8.0f;               ///< 閺堚偓婢堆勬箒閺佸牓鈧喎瀹?
inline constexpr float kLeftPhi1OffsetRad = -1.50f + M_PI;     ///< 瀹革箒鍚?phi1 闂嗘湹缍呴崑蹇曅?
inline constexpr float kLeftPhi4OffsetRad = -1.50f;            ///< 瀹革箒鍚?phi4 闂嗘湹缍呴崑蹇曅?
inline constexpr float kRightPhi1OffsetRad = -1.42f + M_PI;    ///< 閸欏疇鍚?phi1 闂嗘湹缍呴崑蹇曅?
inline constexpr float kRightPhi4OffsetRad = -1.62f;           ///< 閸欏疇鍚?phi4 闂嗘湹缍呴崑蹇曅?
inline constexpr float kThetaDotFilterCutoffHz =
    8.0f;  ///< 閼垫寧鎲滅憴鎺椻偓鐔峰濠娿倖灏濋崳銊﹀焻濮濄垽顣堕悳?
inline constexpr float kImuAccelFilterSampleHz =
    500.0f;  ///< IMU 閸旂娀鈧喎瀹崇拋鈩冩姢濞夈垹娅掗柌鍥ㄧ壉閻?
inline constexpr float kImuAccelFilterCutoffHz =
    10.0f;  ///< IMU 閸旂娀鈧喎瀹崇拋鈩冩姢濞夈垹娅掗幋顏咁剾妫版垹宸?
inline constexpr std::uint32_t kAccelBiasInitSamples =
    1500U;  ///< 閸旂娀鈧喎瀹崇拋锟犳祩閸嬪繐鍨垫慨瀣闁插洦鐗遍弫?
inline constexpr float kAccelZeroWheelSpeedThresholdMps =
    0.02f;  ///< 閸旂娀鈧喎瀹崇拋锟犳祩閸嬪繑鐖ｇ€规俺鐤嗛柅鐔兼閸?
inline constexpr float kAccelZeroHighThresholdMps2 =
    0.5f;  ///< 閸旂娀鈧喎瀹崇拋锟犳祩閸嬪繑鐖ｇ€规岸鐝梼鍫濃偓?
inline constexpr float kAccelZeroLowThresholdMps2 =
    0.2f;  ///< 閸旂娀鈧喎瀹崇拋锟犳祩閸嬪繑鐖ｇ€规矮缍嗛梼鍫濃偓?
inline constexpr float kKalmanMinVariance =
    1e-5f;  ///< 閸椻€崇毜閺囧吋鎶ゅ▔銏犳珤閺堚偓鐏忓繑鏌熷?
inline constexpr float kThetaPiHalf = 1.57079632679489661923f;  ///< pi/2 鐢悂鍣?

inline constexpr std::array<float, 4> kKalmanF{1.0f, kDefaultDtS, 0.0f,
                                               1.0f};  ///< 閸椻€崇毜閺囪偐濮搁幀浣芥祮缁夎崵鐓╅梼?
inline constexpr std::array<float, 4> kKalmanQ{
    0.0005f, 0.0f, 0.0f, 0.04f};  ///< 閸椻€崇毜閺囪壈绻冪粙瀣珨婢规澘宕楅弬鐟版▕
inline constexpr std::array<float, 4> kKalmanR{
    0.5f, 0.0f, 0.0f, 2.0f};  ///< 閸椻€崇毜閺囪壈顫囧ù瀣珨婢规澘宕楅弬鐟版▕
inline constexpr std::array<float, 4> kKalmanP{
    10.0f, 0.0f, 0.0f, 10.0f};  ///< 閸椻€崇毜閺囩厧鍨垫慨瀣嚖瀹割喖宕楅弬鐟版▕
inline constexpr std::array<float, 4> kKalmanH{1.0f, 0.0f, 0.0f,
                                               1.0f};  ///< 閸椻€崇毜閺囪壈顫囧ù瀣叐闂?
}  // namespace state_estimator

namespace leg_kinematics {
inline constexpr float kDefaultDtS = 0.002f;  ///< 閼佃儻绻嶉崝銊ヮ劅姒涙顓婚弮鍫曟？濮濄儵鏆?
inline constexpr float kMinSin =
    1e-5f;  ///< 閺堚偓鐏忓繑顒滃锕€鈧》绱欓弫鏉库偓闂寸箽閹躲倧绱?
inline constexpr float kMinLen = 1e-5f;  ///< 閺堚偓鐏忓繘鏆辨惔锔肩礄閺佹澘鈧棿绻氶幎銈忕礆
}  // namespace leg_kinematics

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
