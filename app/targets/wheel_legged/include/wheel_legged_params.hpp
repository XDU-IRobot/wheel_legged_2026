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
inline constexpr PidGains kPitchSpeedPid{2.f, 0.0f, 0.0f, 0.0f, 0.0f};
}  // namespace gimbal

namespace chassis_fsm {
inline constexpr std::uint32_t kJumpPrepMs = 450U;
inline constexpr std::uint32_t kJumpPushMaxMs = 1000U;
inline constexpr std::uint32_t kJumpRecoverMs = 450U;
inline constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;
inline constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;

inline constexpr float kLowLegLengthM = 0.15f;
inline constexpr float kMidLegLengthM = 0.22f;
inline constexpr float kHighLegLengthM = 0.34f;
inline constexpr float kJumpPrepLegLengthM = 0.13f;
inline constexpr float kJumpPushLegLengthM = 0.4f;
inline constexpr float kJumpRecoverLegLengthM = 0.16f;
inline constexpr float kJumpPushReachedLegLengthM = 0.365f;
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
-0.83306,  -5.1702,  3.9639,  7.1986,  -2.8365,  -2.9774,
     -3.2283,  -14.938,  14.569,  23.723,  -14.737,  -9.9482,
     -0.81055,  2.863,  -1.089,  -3.4804,  0.83266,  1.399,
     -2.5853,  9.3913,  -3.7934,  -11.082,  2.2507,  5.0455,
     -12.3,  -78.693,  11.803,  89.993,  -2.4656,  -14.814,
     -0.64798,  -4.0995,  1.5913,  -2.5703,  1.151,  -1.8874,
     -4.4374,  10.82,  -10.556,  -11.228,  13.601,  4.6118,
     -0.34716,  -0.16418,  -0.53572,  1.7816,  -4.0662,  0.49091,
     -25.277,  37.872,  32.004,  -0.75349,  -37.943,  -27.074,
     -2.4722,  2.7258,  5.0256,  1.1312,  -5.5857,  -4.5784,
     -0.83306,  3.9639,  -5.1702,  -2.9774,  -2.8365,  7.1986,
     -3.2283,  14.569,  -14.938,  -9.9482,  -14.737,  23.723,
     0.81055,  1.089,  -2.863,  -1.399,  -0.83266,  3.4804,
     2.5853,  3.7934,  -9.3913,  -5.0455,  -2.2507,  11.082,
     -4.4374,  -10.556,  10.82,  4.6118,  13.601,  -11.228,
     -0.34716,  -0.53572,  -0.16418,  0.49091,  -4.0662,  1.7816,
     -12.3,  11.803,  -78.693,  -14.814,  -2.4656,  89.993,
     -0.64798,  1.5913,  -4.0995,  -1.8874,  1.151,  -2.5703,
     -25.277,  32.004,  37.872,  -27.074,  -37.943,  -0.75349,
     -2.4722,  5.0256,  2.7258,  -4.5784,  -5.5857,  1.1312,
     2.0588,  2.5769,  -7.0205,  -12.534,  11.718,  3.3683,
     7.4556,  6.1892,  -26.658,  -40.376,  46.683,  12.73,
     -0.72175,  -4.466,  -2.225,  8.0066,  -2.6818,  3.3191,
     -2.3077,  -14.775,  -6.7921,  26.093,  -8.7142,  9.9612,
     63.482,  -113.17,  15.872,  102.99,  47.489,  -33.411,
     2.5342,  -1.9481,  -0.4535,  5.6629,  4.0161,  -1.2781,
     -3.8787,  -37.776,  -12.034,  56.789,  -33.164,  -16.399,
     -0.2925,  -1.3576,  2.4025,  -0.84556,  1.1223,  -8.8394,
     -47.075,  -230.67,  78.522,  270.91,  30.395,  -95.402,
     -2.3876,  -16.763,  5.0598,  15.783,  6.98,  -6.8333,
     2.0588,  -7.0205,  2.5769,  3.3683,  11.718,  -12.534,
     7.4556,  -26.658,  6.1892,  12.73,  46.683,  -40.376,
     0.72175,  2.225,  4.466,  -3.3191,  2.6818,  -8.0066,
     2.3077,  6.7921,  14.775,  -9.9612,  8.7142,  -26.093,
     -3.8787,  -12.034,  -37.776,  -16.399,  -33.164,  56.789,
     -0.2925,  2.4025,  -1.3576,  -8.8394,  1.1223,  -0.84556,
     63.482,  15.872,  -113.17,  -33.411,  47.489,  102.99,
     2.5342,  -0.4535,  -1.9481,  -1.2781,  4.0161,  5.6629,
     -47.075,  78.522,  -230.67,  -95.402,  30.395,  270.91,
     -2.3876,  5.0598,  -16.763,  -6.8333,  6.98,  15.783,
};

inline constexpr PidGains kLeftL0Pid{8000.0f, 0.15f, 70000.0f, 170.0f, 30.0f};
inline constexpr PidGains kRightL0Pid{8000.0f, 0.15f, 70000.0f, 170.0f, 30.0f};
inline constexpr PidGains kLeftL0PidJumpTwo{8000.0f, 0.0f, 70000.0f, 250.0f, 0.0f};
inline constexpr PidGains kRightL0PidJumpTwo{8000.0f, 0.0f, 70000.0f, 250.0f, 0.0f};
inline constexpr PidGains kLeftL0PidJumpThree{6500.0f, 0.15f, 60000.0f, 190.0f, 30.0f};
inline constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 60000.0f, 190.0f, 30.0f};
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
inline constexpr float kSpinYawRampStepRadS = 0.005f;
inline constexpr float kSpinTargetYawDotRadS = 6.0f;
inline constexpr float kSpinTranslationGain = 1.0f;
inline constexpr float kSpinThetaLlBiasRad = 0.0f;
inline constexpr float kYawFollowFixedTargetRad = 1.216f;
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

inline constexpr PidGains kYawFollowPid{8.0f, 0.0f, 1.f, 4.0f, 0.0f};
}  // namespace control_loop

namespace actuators {
inline constexpr float kLeftWheelTorqueToCurrent = 2300.0f;
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
inline constexpr std::size_t kImuUartRxBufferSize = 518;  ///< IMU 串口接收缓冲大小

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
inline constexpr float kMidLegLengthM = 0.20f;              ///< 中腿长目标
inline constexpr float kHighLegLengthM = 0.35f;             ///< 高腿长目标
inline constexpr float kJumpPrepLegLengthM = 0.13f;         ///< 跳跃预备腿长
inline constexpr float kJumpPushLegLengthM = 0.36f;         ///< 跳跃蹬伸腿长
inline constexpr float kJumpRecoverLegLengthM = 0.20f;      ///< 跳跃回收腿长
inline constexpr float kJumpPushReachedLegLengthM = 0.30f;  ///< 蹬伸判定腿长阈值
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

inline constexpr PidGains kLeftL0Pid{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};            ///< 左腿长控制 PID
inline constexpr PidGains kRightL0Pid{6500.0f, 0.15f, 50000.0f, 170.0f, 30.0f};           ///< 右腿长控制 PID
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
