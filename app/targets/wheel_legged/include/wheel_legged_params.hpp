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

inline constexpr std::uint16_t kDmLfMasterId = 0x04;
inline constexpr std::uint16_t kDmLfSlaveId = 0x03;
inline constexpr std::uint16_t kDmLbMasterId = 0x06;
inline constexpr std::uint16_t kDmLbSlaveId = 0x05;
inline constexpr std::uint16_t kDmRfMasterId = 0x02;
inline constexpr std::uint16_t kDmRfSlaveId = 0x01;
inline constexpr std::uint16_t kDmRbMasterId = 0x08;
inline constexpr std::uint16_t kDmRbSlaveId = 0x07;

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

inline const DmMitSettings kPitchMotorSettings{0x13, 0x12, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};
inline const DmMitSettings kYawMotorSettings{0x21, 0x11, kPi, 30.f, 10.f, {0.f, 500.f}, {0.f, 5.f}};

inline constexpr float kDefaultDtS = 0.002f;
inline constexpr float kPitchMinRad = -0.2f;
inline constexpr float kPitchMaxRad = 0.25f;
inline constexpr float kDmTorqueLimitNm = 10.0f;
inline constexpr float kPitchGravityCompensationNm = 1.3f;

inline constexpr PidGains kYawPositionPid{24.0f, 0.0f, 0.0f, 1000.0f, 1.0f};
inline constexpr PidGains kYawSpeedPid{1.f, 0.0f, 0.0f, 10.0f, 0.4f};
inline constexpr PidGains kPitchPositionPid{25.0f, 0.0f, 0.0f, 1000.0f, 0.4f};
inline constexpr PidGains kPitchSpeedPid{2.f, 0.0f, 0.0f, 10.0f, 0.0f};
}  // namespace gimbal

namespace chassis_fsm {
inline constexpr std::uint32_t kJumpPrepMs = 450U;
inline constexpr std::uint32_t kJumpPushMaxMs = 1000U;
inline constexpr std::uint32_t kJumpRecoverMs = 450U;
inline constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;
inline constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;

inline constexpr float kLowLegLengthM = 0.13f;
inline constexpr float kMidLegLengthM = 0.2f;
inline constexpr float kHighLegLengthM = 0.27f;
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
inline constexpr float kBodyMassKg = 24.0f;
inline constexpr float kLegMassKg = 2.3f;
inline constexpr float kGravityMps2 = 9.81f;
inline constexpr float kWheelRadiusM = 0.2025f;
inline constexpr float kOffGroundSupportForceThresholdN = 10.0f;
inline constexpr float kRollBalanceTargetRad = 0.052f;
inline constexpr float kPostureThetaBMinRad = -0.8f;
inline constexpr float kPostureThetaBMaxRad = 0.8f;
inline constexpr float kPostureRollMinRad = -0.5f;
inline constexpr float kPostureRollMaxRad = 0.5f;
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
    -2.6445,  -12.6,   10.251,  17.766,  -7.4619, -7.4266, -5.567,   -16.821, 20.891,  29.849,  -24.519, -13.014,
    -1.1474,  3.906,   -1.3866, -5.7547, 1.5533,  1.8049,  -5.1477,  17.774,  -6.5037, -25.904, 6.7679,  8.5849,
    -14.685,  -80.684, 16.187,  97.32,   -12.948, -19.771, -0.79047, -4.8123, 2.1206,  -2.6328, 0.47748, -2.2372,
    -6.2642,  23.242,  -18.645, -39.999, 29.701,  17.373,  -0.46091, 0.40931, -1.3627, 0.95392, -4.1794, 1.7985,
    -23.123,  45.231,  28.673,  -18.674, -40.503, -23.06,  -2.6116,  3.5853,  5.4235,  0.48667, -7.047,  -4.761,
    -2.6445,  10.251,  -12.6,   -7.4266, -7.4619, 17.766,  -5.567,   20.891,  -16.821, -13.014, -24.519, 29.849,
    1.1474,   1.3866,  -3.906,  -1.8049, -1.5533, 5.7547,  5.1477,   6.5037,  -17.774, -8.5849, -6.7679, 25.904,
    -6.2642,  -18.645, 23.242,  17.373,  29.701,  -39.999, -0.46091, -1.3627, 0.40931, 1.7985,  -4.1794, 0.95392,
    -14.685,  16.187,  -80.684, -19.771, -12.948, 97.32,   -0.79047, 2.1206,  -4.8123, -2.2372, 0.47748, -2.6328,
    -23.123,  28.673,  45.231,  -23.06,  -40.503, -18.674, -2.6116,  5.4235,  3.5853,  -4.761,  -7.047,  0.48667,
    3.3863,   3.9763,  -12.618, -17.304, 22.588,  1.7443,  6.41,     4.4976,  -25.383, -28.464, 46.741,  5.2784,
    -0.72971, -3.7267, -2.7337, 7.6209,  -2.8115, 3.8989,  -3.2797,  -17.031, -12.174, 34.489,  -12.733, 17.178,
    45.535,   -85.851, 23.018,  108.45,  60.323,  -47.798, 2.0473,   -2.8389, 0.26134, 9.2948,  4.0662,  -2.7437,
    -4.7976,  -35.096, -19.404, 70.847,  -38.918, -30.004, -0.33195, -2.0164, 2.093,   1.5668,  1.9951,  -11.945,
    -39.57,   -155.35, 69.09,   190.95,  3.2277,  -70.566, -2.412,   -13.532, 5.0765,  13.273,  4.6463,  -5.8032,
    3.3863,   -12.618, 3.9763,  1.7443,  22.588,  -17.304, 6.41,     -25.383, 4.4976,  5.2784,  46.741,  -28.464,
    0.72971,  2.7337,  3.7267,  -3.8989, 2.8115,  -7.6209, 3.2797,   12.174,  17.031,  -17.178, 12.733,  -34.489,
    -4.7976,  -19.404, -35.096, -30.004, -38.918, 70.847,  -0.33195, 2.093,   -2.0164, -11.945, 1.9951,  1.5668,
    45.535,   23.018,  -85.851, -47.798, 60.323,  108.45,  2.0473,   0.26134, -2.8389, -2.7437, 4.0662,  9.2948,
    -39.57,   69.09,   -155.35, -70.566, 3.2277,  190.95,  -2.412,   5.0765,  -13.532, -5.8032, 4.6463,  13.273,
};

inline constexpr PidGains kLeftL0Pid{8000.0f, 0.15f, 70000.0f, 170.0f, 30.0f};
inline constexpr PidGains kRightL0Pid{8000.0f, 0.15f, 70000.0f, 170.0f, 30.0f};
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
inline constexpr float kLockPointCaptureSpeedThresholdMps = 0.02f;
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
inline constexpr float kExpectedThetaLlBiasRad = -0.12f;
inline constexpr float kExpectedThetaLrBiasRad = -0.12f;
inline constexpr float kExpectedThetaBBiasRad = -0.123f;

struct SdotRampParams {
  float accel_step;
  float brake_step;
};

inline constexpr SdotRampParams kSdotRampLowLeg{0.01f, 0.008f};
inline constexpr SdotRampParams kSdotRampMidLeg{0.006f, 0.003f};
inline constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};

inline constexpr PidGains kYawFollowPid{10.0f, 0.0f, 1.f, 4.0f, 0.0f};
}  // namespace control_loop

namespace actuators {
inline constexpr float kLeftWheelTorqueToCurrent = 2300.0f;
inline constexpr float kRightWheelTorqueToCurrent = 2300.0f;
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
inline constexpr float kLeftPhi1OffsetRad = -0.05f + M_PI;
inline constexpr float kLeftPhi4OffsetRad = -0.59 + 0.07f;
inline constexpr float kRightPhi1OffsetRad = 3.04 + M_PI;
inline constexpr float kRightPhi4OffsetRad = -2.17;
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
inline constexpr std::uint32_t kJumpPrepMs = 200U;
inline constexpr std::uint32_t kJumpPushMaxMs = 1000U;
inline constexpr std::uint32_t kJumpRecoverMs = 250U;
inline constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;
inline constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;

inline constexpr float kLowLegLengthM = 0.15f;
inline constexpr float kMidLegLengthM = 0.21f;
inline constexpr float kHighLegLengthM = 0.3f;
inline constexpr float kJumpPrepLegLengthM = 0.13f;
inline constexpr float kJumpPushLegLengthM = 0.22f;
inline constexpr float kJumpRecoverLegLengthM = 0.20f;
inline constexpr float kJumpPushReachedLegLengthM = 0.21f;
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
inline constexpr float kPostureRollMinRad = -0.5f;
inline constexpr float kPostureRollMaxRad = 0.5f;
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

inline constexpr PidGains kLeftL0Pid{7500.0f, 0.04f, 90000.0f, 170.0f, 10.0f};
inline constexpr PidGains kRightL0Pid{8500.0f, 0.04f, 90000.0f, 170.0f, 10.0f};
inline constexpr PidGains kLeftL0PidJumpTwo{7000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};
inline constexpr PidGains kRightL0PidJumpTwo{6000.0f, 0.0f, 40000.0f, 250.0f, 0.0f};
inline constexpr PidGains kLeftL0PidJumpThree{7500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};
inline constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};
inline constexpr PidGains kRollPid{800.0f, 0.0f, 200.0f, 180.0f, 0.0f};
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
inline constexpr float kVxInputDeadbandNorm = 0.05f;
inline constexpr float kVyInputDeadbandNorm = 0.05f;
inline constexpr float kLockPointEnterSpeedThresholdMps = 0.30f;
inline constexpr float kLockPointExitSpeedThresholdMps = 0.55f;
inline constexpr float kLockPointCaptureSpeedThresholdMps = 1.f;
inline constexpr float kLockPointEnterInputThreshold = 0.1f;
inline constexpr float kLockPointExitInputThreshold = 0.12f;
inline constexpr std::uint32_t kLockPointMinDwellTicks = 10U;
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

inline constexpr SdotRampParams kSdotRampLowLeg{0.005f, 0.005f};
inline constexpr SdotRampParams kSdotRampMidLeg{0.004f, 0.004f};
inline constexpr SdotRampParams kSdotRampHighLeg{0.003f, 0.003f};

inline constexpr PidGains kYawFollowPid{8.0f, 0.0f, 1.2f, 6.0f, 0.0f};
}  // namespace control_loop

namespace actuators {
inline constexpr float kLeftWheelTorqueToCurrent = 2500.0f;
inline constexpr float kRightWheelTorqueToCurrent = 2300.0f;
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
inline constexpr std::uint32_t kJumpPrepMs = 250U;                   ///< 跳跃预备阶段时长
inline constexpr std::uint32_t kJumpPushMaxMs = 1000U;               ///< 跳跃蹬伸最大时长
inline constexpr std::uint32_t kJumpRecoverMs = 250U;                ///< 跳跃回收阶段时长
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
inline constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.03f;  ///< 上台阶收腿到位判定容差
inline constexpr float kStairClimbThetaNearZeroThresholdRad = 0.1f;       ///< 上台阶摆角归零判定阈值

}  // namespace chassis_fsm

namespace chassis {
inline constexpr float kControlDtS = 0.002f;                      ///< 底盘控制时间步长
inline constexpr float kLegL1M = 0.215f;                          ///< 五连杆上段长度
inline constexpr float kLegL2M = 0.254f;                          ///< 五连杆下段长度
inline constexpr float kSpringTorqueScale = 80.0f;                ///< 弹簧力矩缩放系数
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
inline constexpr float kPostureRollMinRad = -0.5f;               ///< 姿态有效 roll 下限
inline constexpr float kPostureRollMaxRad = 0.5f;                ///< 姿态有效 roll 上限
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
    -1.591,   -7.6413, 6.188,    11.268,   -2.5759,  -6.7423,  -3.2384,  -11.829, 12.233,  18.401,   -6.7737, -13.243,
    -0.43397, 1.7776,  -0.44445, -2.7788,  1.2093,   0.4847,   -3.8951,  16.021,  -4.0117, -25.005,  10.941,  4.3729,
    -6.5657,  -59.575, 15.056,   62.048,   -31.914,  -15.446,  -1.0388,  -4.5059, 1.5729,  0.39936,  -0.6459, -2.1145,
    -4.2419,  15.785,  -13.869,  -26.983,  47.155,   11.76,    -0.33777, 0.54193, -1.2556, -0.52199, 0.17372, 0.65107,
    -10.874,  21.193,  14.693,   -6.0893,  -22.795,  -13.562,  -1.7617,  2.7216,  3.3308,  0.30522,  -4.4868, -3.4385,
    -1.591,   6.188,   -7.6413,  -6.7423,  -2.5759,  11.268,   -3.2384,  12.233,  -11.829, -13.243,  -6.7737, 18.401,
    0.43397,  0.44445, -1.7776,  -0.4847,  -1.2093,  2.7788,   3.8951,   4.0117,  -16.021, -4.3729,  -10.941, 25.005,
    -4.2419,  -13.869, 15.785,   11.76,    47.155,   -26.983,  -0.33777, -1.2556, 0.54193, 0.65107,  0.17372, -0.52199,
    -6.5657,  15.056,  -59.575,  -15.446,  -31.914,  62.048,   -1.0388,  1.5729,  -4.5059, -2.1145,  -0.6459, 0.39936,
    -10.874,  14.693,  21.193,   -13.562,  -22.795,  -6.0893,  -1.7617,  3.3308,  2.7216,  -3.4385,  -4.4868, 0.30522,
    1.6628,   -1.5432, -2.972,   -3.4631,  8.0836,   -0.48456, 3.1242,   -3.629,  -6.0672, -4.7018,  15.613,  -0.31132,
    -0.29402, -1.1382, -1.0861,  2.5531,   -0.85725, 1.4888,   -2.6374,  -10.326, -9.7952, 23.069,   -7.7657, 13.394,
    13.319,   16.354,  16.102,   -29.276,  52.528,   -30.743,  2.1637,   -5.2879, 0.95415, 9.2789,   3.7343,  -2.437,
    0.67876,  -31.519, -31.717,  55.723,   -33.89,   14.449,   -0.29282, -1.8755, 1.5848,  3.1069,   -1.8457, -6.5439,
    -15.963,  -62.387, 30.972,   76.483,   0.3788,   -31.335,  -1.6996,  -9.227,  4.6037,  10.14,    1.3882,  -4.9127,
    1.6628,   -2.972,  -1.5432,  -0.48456, 8.0836,   -3.4631,  3.1242,   -6.0672, -3.629,  -0.31132, 15.613,  -4.7018,
    0.29402,  1.0861,  1.1382,   -1.4888,  0.85725,  -2.5531,  2.6374,   9.7952,  10.326,  -13.394,  7.7657,  -23.069,
    0.67876,  -31.717, -31.519,  14.449,   -33.89,   55.723,   -0.29282, 1.5848,  -1.8755, -6.5439,  -1.8457, 3.1069,
    13.319,   16.102,  16.354,   -30.743,  52.528,   -29.276,  2.1637,   0.95415, -5.2879, -2.437,   3.7343,  9.2789,
    -15.963,  30.972,  -62.387,  -31.335,  0.3788,   76.483,   -1.6996,  4.6037,  -9.227,  -4.9127,  1.3882,  10.14,
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
inline constexpr float kTargetForwardSpeedMinMps = 0.6f;                 ///< 最小前进目标速度
inline constexpr float kTargetForwardSpeedMaxMps = 1.8f;                 ///< 最大前进目标速度
inline constexpr float kVxInputDeadbandNorm = 0.1f;                      ///< 前进输入死区（归一化）
inline constexpr float kVyInputDeadbandNorm = 0.1f;                      ///< 侧向输入死区（归一化）
inline constexpr float kLockPointEnterSpeedThresholdMps = 0.30f;         ///< 进入定点锁定速度阈值
inline constexpr float kLockPointExitSpeedThresholdMps = 0.55f;          ///< 退出定点锁定速度阈值
inline constexpr float kLockPointCaptureSpeedThresholdMps = 0.02f;       ///< 捕获锁点参考位置的速度阈值
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
inline constexpr float kSpinTargetYawDotRadS = 10.0f;                   ///< 自旋目标偏航角速度
inline constexpr float kSpinTranslationGain = 1.2f;                     ///< 自旋平移增益
inline constexpr float kSpinThetaLlBiasRad = 0.0f;                      ///< 自旋左腿摆角偏置
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
inline constexpr float kLeftWheelTorqueToCurrent = 2436.5f;   ///< 左轮毂力矩-电流转换系数
inline constexpr float kRightWheelTorqueToCurrent = 2436.5f;  ///< 右轮毂力矩-电流转换系数
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
