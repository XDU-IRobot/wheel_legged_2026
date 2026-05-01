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
inline constexpr float kSpinYawRampStepRadS = 0.005f;
inline constexpr float kSpinYawTargetOffsetRad = 0.55f;
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
inline constexpr float kWheelTorqueToCurrent = 2436.5f;
inline constexpr float kWheelCurrentClampAbs = 16000.0f;
}  // namespace actuators


namespace remote_control_can_bridge {
inline constexpr std::uint16_t kRxStdIdA = 0x110;
inline constexpr std::uint16_t kRxStdIdB = 0x111;
inline constexpr std::size_t kPayloadSizeA = 8U;
inline constexpr std::size_t kPayloadSizeB = 8U;
}  // namespace remote_control_can_bridge

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
inline constexpr float kStairClimbLegLengthM = 0.16f;
inline constexpr float kStairClimbThetaTargetRad = 0.2f;
inline constexpr std::uint32_t kStairClimbDurationMs = 400U;
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
    -1.7536,  -8.5963, 7.6363,  13.317,  -5.0025, -7.1595, -7.8569,  -34.422,  33.604,   55.096,  -25.455,  -30.722,
    -1.412,   5.7351,  -1.5938, -8.5599, 2.4571,  2.1146,  -6.3355,  26.019,   -7.4286,  -38.531, 10.996,   9.9427,
    -17.697,  -93.095, 19.591,  87.301,  -20.439, -23.648, -1.0539,  -8.3951,  3.1865,   -5.8755, 1.9465,   -4.4536,
    -6.7816,  17.829,  -12.765, -21.167, 29.847,  0.64978, -0.44142, -1.3045,  -1.9889,  4.8847,  -8.671,   1.2316,
    -19.776,  38.572,  26.093,  -16.219, -33.668, -27.161, -2.8949,  3.1364,   6.593,    1.8681,  -7.2219,  -7.0297,
    -1.7536,  7.6363,  -8.5963, -7.1595, -5.0025, 13.317,  -7.8569,  33.604,   -34.422,  -30.722, -25.455,  55.096,
    1.412,    1.5938,  -5.7351, -2.1146, -2.4571, 8.5599,  6.3355,   7.4286,   -26.019,  -9.9427, -10.996,  38.531,
    -6.7816,  -12.765, 17.829,  0.64978, 29.847,  -21.167, -0.44142, -1.9889,  -1.3045,  1.2316,  -8.671,   4.8847,
    -17.697,  19.591,  -93.095, -23.648, -20.439, 87.301,  -1.0539,  3.1865,   -8.3951,  -4.4536, 1.9465,   -5.8755,
    -19.776,  26.093,  38.572,  -27.161, -33.668, -16.219, -2.8949,  6.593,    3.1364,   -7.0297, -7.2219,  1.8681,
    0.98692,  0.66247, -2.9466, -5.0219, 5.9389,  0.71383, 4.2217,   2.7,      -13.123,  -21.057, 26.444,   3.2147,
    -0.73585, -2.519,  -2.7492, 5.5157,  -1.8533, 3.8488,  -3.3043,  -11.559,  -12.371,  25.071,  -8.5545,  17.242,
    27.937,   -44.386, 21.339,  65.854,  45.7,    -38.34,  1.5936,   -2.2997,  0.86771,  7.409,   3.5388,   -2.6507,
    -4.7288,  -19.992, -17.214, 43.676,  -41.798, -8.3752, -0.47025, -0.83783, 2.1865,   0.28918, -0.16986, -8.3941,
    -26.895,  -87.904, 49.967,  112.8,   -7.1115, -51.104, -2.2375,  -10.096,  5.3281,   10.636,  1.7557,   -5.8204,
    0.98692,  -2.9466, 0.66247, 0.71383, 5.9389,  -5.0219, 4.2217,   -13.123,  2.7,      3.2147,  26.444,   -21.057,
    0.73585,  2.7492,  2.519,   -3.8488, 1.8533,  -5.5157, 3.3043,   12.371,   11.559,   -17.242, 8.5545,   -25.071,
    -4.7288,  -17.214, -19.992, -8.3752, -41.798, 43.676,  -0.47025, 2.1865,   -0.83783, -8.3941, -0.16986, 0.28918,
    27.937,   21.339,  -44.386, -38.34,  45.7,    65.854,  1.5936,   0.86771,  -2.2997,  -2.6507, 3.5388,   7.409,
    -26.895,  49.967,  -87.904, -51.104, -7.1115, 112.8,   -2.2375,  5.3281,   -10.096,  -5.8204, 1.7557,   10.636,
};

inline constexpr PidGains kLeftL0Pid{5900.0f, 0.04f, 65500.0f, 170.0f, 10.0f};
inline constexpr PidGains kRightL0Pid{5900.0f, 0.04f, 65500.0f, 170.0f, 10.0f};
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
inline constexpr float kSpinYawRampStepRadS = 0.005f;
inline constexpr float kSpinYawTargetOffsetRad = 0.55f;
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
inline constexpr float kWheelTorqueToCurrent = 2436.5f;
inline constexpr float kWheelCurrentClampAbs = 16000.0f;
}  // namespace actuators


namespace remote_control_can_bridge {
inline constexpr std::uint16_t kRxStdIdA = 0x110;
inline constexpr std::uint16_t kRxStdIdB = 0x111;
inline constexpr std::size_t kPayloadSizeA = 8U;
inline constexpr std::size_t kPayloadSizeB = 8U;
}  // namespace remote_control_can_bridge

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

inline constexpr double kJointCanTxLimitHz = 4000.0;   ///< 关节 CAN 发送频率上限
inline constexpr double kWheelCanTxLimitHz = 4000.0;   ///< 轮毂 CAN 发送频率上限
inline constexpr double kGimbalCanTxLimitHz = 4000.0;  ///< 云台 CAN 发送频率上限

inline constexpr std::uint16_t kDmLfMasterId = 0x02;  ///< 左前腿主电机 ID
inline constexpr std::uint16_t kDmLfSlaveId = 0x01;   ///< 左前腿从电机 ID
inline constexpr std::uint16_t kDmLbMasterId = 0x04;  ///< 左后腿主电机 ID
inline constexpr std::uint16_t kDmLbSlaveId = 0x03;   ///< 左后腿从电机 ID
inline constexpr std::uint16_t kDmRfMasterId = 0x06;  ///< 右前腿主电机 ID
inline constexpr std::uint16_t kDmRfSlaveId = 0x05;   ///< 右前腿从电机 ID
inline constexpr std::uint16_t kDmRbMasterId = 0x08;  ///< 右后腿主电机 ID
inline constexpr std::uint16_t kDmRbSlaveId = 0x07;   ///< 右后腿从电机 ID

inline constexpr std::uint16_t kLeftWheelId = 0x06;   ///< 左轮毂电机 ID
inline constexpr std::uint16_t kRightWheelId = 0x05;  ///< 右轮毂电机 ID

inline constexpr std::size_t kDr16UartRxBufferSize = 18;  ///< DR16 遥控器串口接收缓冲大小
inline constexpr std::size_t kImuUartRxBufferSize = 518;

inline const DmMitSettings kDmLfSettings{kDmLfMasterId, kDmLfSlaveId,   kPi,          45.0f,
                                         54.0f,         {0.0f, 500.0f}, {0.0f, 10.0f}};  ///< 左前腿 DM 电机参数
inline const DmMitSettings kDmLbSettings{kDmLbMasterId, kDmLbSlaveId,   kPi,          45.0f,
                                         54.0f,         {0.0f, 500.0f}, {0.0f, 10.0f}};  ///< 左后腿 DM 电机参数
inline const DmMitSettings kDmRfSettings{kDmRfMasterId, kDmRfSlaveId,   kPi,          45.0f,
                                         54.0f,         {0.0f, 500.0f}, {0.0f, 10.0f}};  ///< 右前腿 DM 电机参数
inline const DmMitSettings kDmRbSettings{kDmRbMasterId, kDmRbSlaveId,   kPi,          45.0f,
                                         54.0f,         {0.0f, 500.0f}, {0.0f, 10.0f}};  ///< 右后腿 DM 电机参数
}  // namespace globals

namespace gimbal {
using DmMitSettings = rm::device::DmMotorSettings<rm::device::DmMotorControlMode::kMit>;

inline const DmMitSettings kPitchMotorSettings{0x05, 0x04,         kPi,       30.f,
                                               10.f, {0.f, 500.f}, {0.f, 5.f}};  ///< 俯仰电机参数
inline const DmMitSettings kYawMotorSettings{0x10, 0x09, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};  ///< 偏航电机参数

inline constexpr float kDefaultDtS = 0.002f;                ///< 云台控制默认时间步长
inline constexpr float kPitchMinRad = -0.2f;                ///< 俯仰角下限
inline constexpr float kPitchMaxRad = 0.25f;                ///< 俯仰角上限
inline constexpr float kDmTorqueLimitNm = 10.0f;            ///< DM 电机力矩限制
inline constexpr float kPitchGravityCompensationNm = 1.3f;  ///< 俯仰重力补偿力矩

inline constexpr PidGains kYawPositionPid{25.0f, 0.0f, 0.05f, 10.0f, 1.0f};    ///< 偏航位置 PID
inline constexpr PidGains kYawSpeedPid{0.6f, 0.0f, 0.0f, 6.0f, 0.4f};          ///< 偏航速度 PID
inline constexpr PidGains kPitchPositionPid{26.0f, 0.0f, 0.05f, 10.0f, 0.4f};  ///< 俯仰位置 PID
inline constexpr PidGains kPitchSpeedPid{0.55f, 0.0f, 0.0f, 8.0f, 0.0f};       ///< 俯仰速度 PID
}  // namespace gimbal

namespace chassis_fsm {
inline constexpr std::uint32_t kJumpPrepMs = 450U;                   ///< 跳跃预备阶段时长
inline constexpr std::uint32_t kJumpPushMaxMs = 1000U;               ///< 跳跃蹬伸最大时长
inline constexpr std::uint32_t kJumpRecoverMs = 450U;                ///< 跳跃回收阶段时长
inline constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;        ///< 倒地确认时长
inline constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;  ///< 自恢复超时时长

inline constexpr float kLowLegLengthM = 0.15f;              ///< 低腿长目标
inline constexpr float kMidLegLengthM = 0.22f;              ///< 中腿长目标
inline constexpr float kHighLegLengthM = 0.35f;             ///< 高腿长目标
inline constexpr float kJumpPrepLegLengthM = 0.13f;         ///< 跳跃预备腿长
inline constexpr float kJumpPushLegLengthM = 0.25f;         ///< 跳跃蹬伸腿长
inline constexpr float kJumpRecoverLegLengthM = 0.20f;      ///< 跳跃回收腿长
inline constexpr float kJumpPushReachedLegLengthM = 0.25f;  ///< 蹬伸判定腿长阈值
inline constexpr float kLegLengthRampTimeS = 0.5f;          ///< 腿长切换斜坡时间

inline constexpr float kStairClimbThetaThresholdRad = 0.5f;  ///< 上台阶腿摆角检测阈值///
inline constexpr float kStairClimbLegLengthM = 0.16f;        ///< 上台阶收腿目标///
inline constexpr float kStairClimbThetaTargetRad = 0.2f;  ///< 上台阶腿摆角目标///---------------上台阶参数
inline constexpr std::uint32_t kStairClimbDurationMs = 250U;      ///< 上台阶收腿保持时长///
inline constexpr std::uint32_t kStairClimbPitchStableMs = 2000U;  ///< 上台阶 pitch 稳定等待时长///
}  // namespace chassis_fsm

namespace chassis {
inline constexpr float kControlDtS = 0.002f;                      ///< 底盘控制时间步长
inline constexpr float kLegL1M = 0.215f;                          ///< 五连杆上段长度
inline constexpr float kLegL2M = 0.254f;                          ///< 五连杆下段长度
inline constexpr float kSpringTorqueScale = 90.0f;                ///< 弹簧力矩缩放系数
inline constexpr float kSpringModelA = 1082.0f;                   ///< 弹簧模型参数 A
inline constexpr float kSpringModelB = 1070.0f;                   ///< 弹簧模型参数 B
inline constexpr float kSpringModelC = 404.0f;                    ///< 弹簧模型参数 C
inline constexpr float kSpringModelD = 177.0f;                    ///< 弹簧模型参数 D
inline constexpr float kSpringPhaseDivisor = 18.0f;               ///< 弹簧模型相位除数
inline constexpr float kBodyMassKg = 22.0f;                       ///< 车身质量
inline constexpr float kLegMassKg = 2.3f;                         ///< 单腿质量
inline constexpr float kGravityMps2 = 9.81f;                      ///< 重力加速度
inline constexpr float kWheelRadiusM = 0.2025f;                   ///< 轮毂半径
inline constexpr float kOffGroundSupportForceThresholdN = 10.0f;  ///< 离地支撑力判定阈值
inline constexpr float kRollBalanceTargetRad = 0.003f;            ///< 横滚平衡目标角
inline constexpr float kPostureThetaBMinRad = -0.7f;              ///< 姿态有效 body pitch 下限
inline constexpr float kPostureThetaBMaxRad = 0.7f;               ///< 姿态有效 body pitch 上限
inline constexpr float kPostureThetaLegMinRad = -0.8f;            ///< 姿态有效腿摆角下限
inline constexpr float kPostureThetaLegMaxRad = 1.4f;             ///< 姿态有效腿摆角上限
inline constexpr float kLegRecoverThetaDotTarget = -2.0f;         ///< 姿态恢复腿摆角速度目标
inline constexpr float kLegRecoverZeroTorqueMinRad = 0.0f;        ///< 姿态恢复零力矩区间下限
inline constexpr float kLegRecoverZeroTorqueMaxRad = 1.4f;        ///< 姿态恢复零力矩区间上限

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
   -1.5204,  -7.4399,  5.9657,  10.931,  -2.435,  -6.5271,
     -3.1013,  -11.561,  11.803,  17.888,  -6.4299,  -12.832,
     -0.42057,  1.713,  -0.43919,  -2.6572,  1.1556,  0.47966,
     -3.7751,  15.439,  -3.9645,  -23.912,  10.456,  4.3276,
     -6.2929,  -58.498,  14.865,  60.498,  -30.302,  -15.406,
     -0.99437,  -4.412,  1.5395,  0.37628,  -0.58181,  -2.0773,
     -4.1093,  15.057,  -13.9,  -25.533,  45.668,  11.692,
     -0.32787,  0.49905,  -1.2098,  -0.44629,  0.16794,  0.57922,
     -10.709,  20.329,  14.648,  -5.2165,  -22.337,  -13.449,
     -1.7369,  2.6214,  3.3041,  0.36694,  -4.3927,  -3.3969,
     -1.5204,  5.9657,  -7.4399,  -6.5271,  -2.435,  10.931,
     -3.1013,  11.803,  -11.561,  -12.832,  -6.4299,  17.888,
     0.42057,  0.43919,  -1.713,  -0.47966,  -1.1556,  2.6572,
     3.7751,  3.9645,  -15.439,  -4.3276,  -10.456,  23.912,
     -4.1093,  -13.9,  15.057,  11.692,  45.668,  -25.533,
     -0.32787,  -1.2098,  0.49905,  0.57922,  0.16794,  -0.44629,
     -6.2929,  14.865,  -58.498,  -15.406,  -30.302,  60.498,
     -0.99437,  1.5395,  -4.412,  -2.0773,  -0.58181,  0.37628,
     -10.709,  14.648,  20.329,  -13.449,  -22.337,  -5.2165,
     -1.7369,  3.3041,  2.6214,  -3.3969,  -4.3927,  0.36694,
     1.7119,  -1.4778,  -3.1162,  -3.7328,  8.1833,  -0.30904,
     3.2227,  -3.5514,  -6.354,  -5.151,  15.857,  0.020885,
     -0.2875,  -1.1755,  -1.0683,  2.6119,  -0.87825,  1.4638,
     -2.579,  -10.662,  -9.6352,  23.6,  -7.9564,  13.17,
     13.475,  17.277,  15.67,  -31.49,  53.511,  -30.286,
     2.1864,  -5.2063,  0.90571,  9.1,  3.7912,  -2.3732,
     0.81483,  -31.998,  -31.668,  56.374,  -34.871,  14.568,
     -0.27991,  -1.9008,  1.5981,  3.1173,  -1.9279,  -6.5117,
     -15.684,  -63.057,  30.402,  76.674,  1.1552,  -30.692,
     -1.6615,  -9.3074,  4.514,  10.138,  1.5171,  -4.8147,
     1.7119,  -3.1162,  -1.4778,  -0.30904,  8.1833,  -3.7328,
     3.2227,  -6.354,  -3.5514,  0.020885,  15.857,  -5.151,
     0.2875,  1.0683,  1.1755,  -1.4638,  0.87825,  -2.6119,
     2.579,  9.6352,  10.662,  -13.17,  7.9564,  -23.6,
     0.81483,  -31.668,  -31.998,  14.568,  -34.871,  56.374,
     -0.27991,  1.5981,  -1.9008,  -6.5117,  -1.9279,  3.1173,
     13.475,  15.67,  17.277,  -30.286,  53.511,  -31.49,
     2.1864,  0.90571,  -5.2063,  -2.3732,  3.7912,  9.1,
     -15.684,  30.402,  -63.057,  -30.692,  1.1552,  76.674,
     -1.6615,  4.514,  -9.3074,  -4.8147,  1.5171,  10.138,
};

inline constexpr PidGains kLeftL0Pid{6000.0f, 0.15f, 50000.0f, 170.0f, 30.0f};            ///< 左腿长控制 PID
inline constexpr PidGains kRightL0Pid{6000.0f, 0.15f, 50000.0f, 170.0f, 30.0f};           ///< 右腿长控制 PID
inline constexpr PidGains kLeftL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};       ///< 左腿跳跃蹬伸 PID
inline constexpr PidGains kRightL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};      ///< 右腿跳跃蹬伸 PID
inline constexpr PidGains kLeftL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};   ///< 左腿跳跃回收 PID
inline constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};  ///< 右腿跳跃回收 PID
inline constexpr PidGains kRollPid{500.0f, 0.0f, 80.0f, 80.0f, 0.0f};                     ///< 横滚平衡 PID
inline constexpr PidGains kLeftLegTurnPid{20.0f, 0.0f, 0.0f, 15.0f, 0.0f};                ///< 左腿摆角恢复 PID
inline constexpr PidGains kRightLegTurnPid{20.0f, 0.0f, 0.0f, 15.0f, 0.0f};               ///< 右腿摆角恢复 PID
}  // namespace chassis

namespace control_loop {
inline constexpr std::int16_t kWheelSpinThreshold = 220;                 ///< 拨轮自旋阈值
inline constexpr std::int16_t kWheelActionThreshold = 320;               ///< 拨轮动作触发阈值
inline constexpr std::int16_t kWheelCenterThreshold = 80;                ///< 拨轮归中判定阈值
inline constexpr float kControlLoopDtS = 0.002f;                         ///< 主控制循环周期
inline constexpr std::int16_t kDr16AxisMaxAbs = 660;                     ///< DR16 摇杆最大绝对值
inline constexpr float kTargetForwardSpeedMaxMps = 1.8f;                 ///< 最大前进目标速度
inline constexpr float kVxInputDeadbandNorm = 0.1f;                      ///< 前进输入死区（归一化）
inline constexpr float kVyInputDeadbandNorm = 0.1f;                      ///< 侧向输入死区（归一化）
inline constexpr float kLockPointEnterSpeedThresholdMps = 0.30f;         ///< 进入定点锁定速度阈值
inline constexpr float kLockPointExitSpeedThresholdMps = 0.55f;          ///< 退出定点锁定速度阈值
inline constexpr float kLockPointEnterInputThreshold = 0.08f;            ///< 进入定点锁定输入阈值
inline constexpr float kLockPointExitInputThreshold = 0.12f;             ///< 退出定点锁定输入阈值
inline constexpr std::uint32_t kLockPointMinDwellTicks = 100U;           ///< 定点锁定最小驻留周期数
inline constexpr float kLockPointAlphaRiseStep = 0.015f;                 ///< 定点锁定 alpha 上升步长
inline constexpr float kLockPointAlphaFallStep = 0.018f;                 ///< 定点锁定 alpha 下降步长
inline constexpr float kRcStickMax = 660.0f;                             ///< 遥控器摇杆最大值
inline constexpr float kRcPitchRateMaxRadS = 1.5f;                       ///< 遥控器俯仰角速度上限
inline constexpr float kRcYawRateMaxRadS = -2.5f;                        ///< 遥控器偏航角速度上限
inline constexpr float kPitchTargetMinRad = -0.35f;                      ///< 俯仰目标下限
inline constexpr float kPitchTargetMaxRad = 0.25f;                       ///< 俯仰目标上限
inline constexpr float kYawFollowRampStepRadS = 0.05f;                   ///< 偏航跟随斜坡步长
inline constexpr float kSpinYawRampStepRadS = 0.005f;                    ///< 自旋偏航斜坡步长
inline constexpr float kSpinYawTargetOffsetRad = 0.55f;                  ///< 自旋偏航目标偏移
inline constexpr float kYawFollowFixedTargetRad = 0.f;                   ///< 偏航跟随固定目标角
inline constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;             ///< 偏航跟随侧向偏移
inline constexpr float kGimbalStartupYawAlignErrorRad = 0.04f;           ///< 云台启动偏航对准误差阈值
inline constexpr float kGimbalStartupYawAlignVelRadS = 0.25f;            ///< 云台启动偏航对准速度阈值
inline constexpr std::uint32_t kGimbalStartupYawAlignStableTicks = 50U;  ///< 云台启动偏航对准稳定周期数
inline constexpr float kYawFollowDriveReadyErrorRad = 0.04f;             ///< 偏航跟随就绪误差阈值
inline constexpr float kYawFollowDriveReadyVelRadS = 0.25f;              ///< 偏航跟随就绪速度阈值
inline constexpr std::uint32_t kYawFollowDriveReadyStableTicks = 50U;    ///< 偏航跟随就绪稳定周期数
inline constexpr float kExpectedThetaLlBiasRad = 0.105f;                 ///< 左腿摆角期望偏置
inline constexpr float kExpectedThetaLrBiasRad = 0.105f;                 ///< 右腿摆角期望偏置
inline constexpr float kExpectedThetaBBiasRad = 0.05f;                   ///< 车身俯仰期望偏置

struct SdotRampParams {
  float accel_step;
  float brake_step;
};

inline constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.008f};
inline constexpr SdotRampParams kSdotRampMidLeg{0.006f, 0.003f};
inline constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};

inline constexpr PidGains kYawFollowPid{8.0f, 0.0f, 1.2f, 6.0f, 0.0f};  ///< 偏航跟随 PID
}  // namespace control_loop

namespace actuators {
inline constexpr float kWheelTorqueToCurrent = 2436.5f;   ///< 轮毂力矩-电流转换系数
inline constexpr float kWheelCurrentClampAbs = 16000.0f;  ///< 轮毂电流限幅绝对值
}  // namespace actuators

namespace gimbal_can_bridge {
inline constexpr std::uint16_t kRxStdId = 0x119;  ///< 云台 CAN 反馈接收标准帧 ID
inline constexpr std::size_t kPayloadSize = 4U;   ///< 云台 CAN 反馈数据长度
inline constexpr float kMilliScale = 0.001f;      ///< 云台 CAN 反馈毫单位缩放
}  // namespace gimbal_can_bridge

namespace remote_control_can_bridge {
inline constexpr std::uint16_t kRxStdIdA = 0x110;
inline constexpr std::uint16_t kRxStdIdB = 0x111;
inline constexpr std::size_t kPayloadSizeA = 8U;
inline constexpr std::size_t kPayloadSizeB = 8U;
}  // namespace remote_control_can_bridge

namespace state_estimator {
inline constexpr float kDefaultDtS = 0.002f;                      ///< 状态估计默认时间步长
inline constexpr float kDefaultExpectedSdotMps = 0.05f;           ///< 默认期望速度
inline constexpr float kLegL1M = 0.215f;                          ///< 五连杆上段长度
inline constexpr float kLegL2M = 0.254f;                          ///< 五连杆下段长度
inline constexpr float kWheelRadiusM = 0.0575f;                   ///< 轮毂半径
inline constexpr float kWheelReductionRatio = 17.0f / 268.0f;     ///< 轮毂减速比
inline constexpr float kMaxValidSpeedMps = 8.0f;                  ///< 最大有效速度
inline constexpr float kLeftPhi1OffsetRad = -1.50f + M_PI;        ///< 左腿 phi1 零位偏移
inline constexpr float kLeftPhi4OffsetRad = -1.50f;               ///< 左腿 phi4 零位偏移
inline constexpr float kRightPhi1OffsetRad = -1.42f + M_PI;       ///< 右腿 phi1 零位偏移
inline constexpr float kRightPhi4OffsetRad = -1.62f;              ///< 右腿 phi4 零位偏移
inline constexpr float kThetaDotFilterCutoffHz = 8.0f;            ///< 腿摆角速度滤波器截止频率
inline constexpr float kImuAccelFilterSampleHz = 500.0f;          ///< IMU 加速度计滤波器采样率
inline constexpr float kImuAccelFilterCutoffHz = 10.0f;           ///< IMU 加速度计滤波器截止频率
inline constexpr std::uint32_t kAccelBiasInitSamples = 1500U;     ///< 加速度计零偏初始化采样数
inline constexpr float kAccelZeroWheelSpeedThresholdMps = 0.02f;  ///< 加速度计零偏标定轮速阈值
inline constexpr float kAccelZeroHighThresholdMps2 = 0.5f;        ///< 加速度计零偏标定高阈值
inline constexpr float kAccelZeroLowThresholdMps2 = 0.2f;         ///< 加速度计零偏标定低阈值
inline constexpr float kKalmanMinVariance = 1e-5f;                ///< 卡尔曼滤波器最小方差
inline constexpr float kThetaPiHalf = 1.57079632679489661923f;    ///< pi/2 常量

inline constexpr std::array<float, 4> kKalmanF{1.0f, kDefaultDtS, 0.0f, 1.0f};  ///< 卡尔曼状态转移矩阵
inline constexpr std::array<float, 4> kKalmanQ{0.0005f, 0.0f, 0.0f, 0.04f};     ///< 卡尔曼过程噪声协方差
inline constexpr std::array<float, 4> kKalmanR{0.5f, 0.0f, 0.0f, 2.0f};         ///< 卡尔曼观测噪声协方差
inline constexpr std::array<float, 4> kKalmanP{10.0f, 0.0f, 0.0f, 10.0f};       ///< 卡尔曼初始误差协方差
inline constexpr std::array<float, 4> kKalmanH{1.0f, 0.0f, 0.0f, 1.0f};         ///< 卡尔曼观测矩阵
}  // namespace state_estimator

namespace leg_kinematics {
inline constexpr float kDefaultDtS = 0.002f;  ///< 腿运动学默认时间步长
inline constexpr float kMinSin = 1e-5f;       ///< 最小正弦值（数值保护）
inline constexpr float kMinLen = 1e-5f;       ///< 最小长度（数值保护）
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
