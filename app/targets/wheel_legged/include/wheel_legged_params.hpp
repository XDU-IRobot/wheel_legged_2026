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
inline constexpr float kPitchMinRad = -0.35f;
inline constexpr float kPitchMaxRad = 0.7f;
inline constexpr float kDmTorqueLimitNm = 10.0f;
inline constexpr float kPitchGravityCompensationNm = 1.3f;

inline constexpr PidGains kYawPositionPid{24.0f, 0.0f, 0.0f, 1000.0f, 1.0f};
inline constexpr PidGains kYawSpeedPid{1.f, 0.0f, 0.0f, 10.0f, 0.4f};
inline constexpr PidGains kPitchPositionPid{25.0f, 0.0f, 0.0f, 1000.0f, 0.4f};
inline constexpr PidGains kPitchSpeedPid{2.f, 0.0f, 0.0f, 10.0f, 0.0f};
// ---------- 发射机构参数 (hero: 3 摩擦轮 + DM 拨盘) ----------
inline constexpr float kBoosterZeroPointRad = 0.345f;
inline constexpr float kSegmentAngleRad = kPi / 3.f;
inline constexpr uint16_t kInitDelayTicks = 600;
inline constexpr uint16_t kShootDelayTicks = 360;
inline constexpr float kStallThresholdRad = kPi / 18.f;
inline constexpr float kStallFallbackRad = kPi / 90.f;
inline constexpr int16_t kFireDialThreshold = -100;
inline constexpr float kFwReadySpeedThresholdRpm = 4000.0f;
inline constexpr float kFwTargetSpeedRpm = 7000.0f;

inline constexpr PidGains kBoosterPositionPid{60.f, 0.f, 560.f, 24.f, 0.f};
inline constexpr PidGains kBoosterSpeedPid{0.3f, 0.f, 0.02f, 6.4f, 0.f};
inline constexpr PidGains kFwSpeedPid{30.f, 0.01f, 0.f, 10000.f, 0.f};  // 摩擦轮 PID 占位

inline constexpr std::uint16_t kDmBoosterMasterId = 0x10;
inline constexpr std::uint16_t kDmBoosterSlaveId = 0x09;
inline const DmMitSettings kBoosterDmSettings{kDmBoosterMasterId, kDmBoosterSlaveId, kPi, 30.f, 10.f,
                                              {0.f, 500.f},       {0.f, 5.f}};

inline constexpr std::uint16_t kFwMotor1Id = 0x01;
inline constexpr std::uint16_t kFwMotor2Id = 0x02;
inline constexpr std::uint16_t kFwMotor3Id = 0x03;
}  // namespace gimbal

namespace chassis_fsm {
inline constexpr std::uint32_t kJumpPrepMs = 300U;
inline constexpr std::uint32_t kJumpPushMaxMs = 1000U;
inline constexpr std::uint32_t kJumpRecoverMs = 300U;
inline constexpr std::uint32_t kRecoveryFallConfirmMs = 220U;
inline constexpr std::uint32_t kRecoverySelfRightTimeoutMs = 2200U;

inline constexpr float kLowLegLengthM = 0.13f;
inline constexpr float kMidLegLengthM = 0.19f;
inline constexpr float kHighLegLengthM = 0.34f;
inline constexpr float kJumpPrepLegLengthM = 0.13f;
inline constexpr float kJumpPushLegLengthM = 0.27f;
inline constexpr float kJumpRecoverLegLengthM = 0.22f;
inline constexpr float kJumpPushReachedLegLengthM = 0.25f;
inline constexpr float kLegLengthRampTimeS = 0.5f;
inline constexpr float kStairClimbThetaThresholdRad = 0.4f;
inline constexpr float kStairClimbLegLengthM = 0.14f;
inline constexpr float kStairClimbThetaTargetRad = 0.35f;
inline constexpr std::uint32_t kStairClimbDurationMs = 180U;
inline constexpr float kStairClimbLegLengthNearTargetToleranceM = 0.01f;
inline constexpr float kStairClimbThetaNearZeroThresholdRad = 0.08f;
inline constexpr std::uint32_t kStairClimbPitchStableMs = 150U;
}  // namespace chassis_fsm

namespace chassis {
inline constexpr float kControlDtS = 0.002f;
inline constexpr float kLegL1M = 0.215f;
inline constexpr float kLegL2M = 0.254f;
inline constexpr float kSpringTorqueScale = 105.0f;
inline constexpr float kSpringModelA = 1082.0f;
inline constexpr float kSpringModelB = 1070.0f;
inline constexpr float kSpringModelC = 404.0f;
inline constexpr float kSpringModelD = 177.0f;
inline constexpr float kSpringPhaseDivisor = 18.0f;
inline constexpr float kBodyMassKg = 24.0f;
inline constexpr float kLegMassKg = 2.3f;
inline constexpr float kGravityMps2 = 9.81f;
inline constexpr float kWheelRadiusM = 0.2025f;
inline constexpr float kOffGroundSupportForceThresholdN = 30.0f;
inline constexpr float kMidLegOffGroundRecoverLengthM = 0.16f;
inline constexpr uint32_t kMidLegOffGroundRecoverMs = 3000U;
inline constexpr uint32_t kMidLegOffGroundConfirmMs = 100U;
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
-2.6582,  -17.684,  14.93,  26.816,  -13.843,  -10.466,
     -7.0212,  -33.032,  35.192,  56.456,  -41.158,  -22.679,
     -0.91246,  3.815,  -1.5983,  -3.5819,  -0.45637,  2.5137,
     -1.856,  8.3659,  -3.9715,  -7.1268,  -2.2441,  6.5355,
     -18.828,  -114.92,  13.108,  127.02,  7.1808,  -17.726,
     -0.8572,  -7.1299,  2.8512,  -4.8698,  3.1255,  -3.3057,
     -4.4227,  4.1973,  -3.1239,  11.24,  -11.697,  -3.85,
     -0.41674,  -1.4847,  -0.37763,  6.1853,  -12.748,  2.3501,
     -25.633,  32.975,  34.812,  4.8617,  -36.311,  -33.152,
     -2.9352,  2.118,  7.2793,  2.908,  -7.5932,  -6.8455,
     -2.6582,  14.93,  -17.684,  -10.466,  -13.843,  26.816,
     -7.0212,  35.192,  -33.032,  -22.679,  -41.158,  56.456,
     0.91246,  1.5983,  -3.815,  -2.5137,  0.45637,  3.5819,
     1.856,  3.9715,  -8.3659,  -6.5355,  2.2441,  7.1268,
     -4.4227,  -3.1239,  4.1973,  -3.85,  -11.697,  11.24,
     -0.41674,  -0.37763,  -1.4847,  2.3501,  -12.748,  6.1853,
     -18.828,  13.108,  -114.92,  -17.726,  7.1808,  127.02,
     -0.8572,  2.8512,  -7.1299,  -3.3057,  3.1255,  -4.8698,
     -25.633,  34.812,  32.975,  -33.152,  -36.311,  4.8617,
     -2.9352,  7.2793,  2.118,  -6.8455,  -7.5932,  2.908,
     5.4276,  5.4469,  -13.749,  -37.657,  30.385,  7.5521,
     13.166,  10.708,  -39.806,  -83.531,  83.479,  20.474,
     -0.96521,  -5.3376,  -1.8863,  8.85,  -2.4241,  3.0963,
     -1.9764,  -11.912,  -3.0933,  19.153,  -5.212,  5.0843,
     96.472,  -190.83,  9.0778,  162.69,  36.092,  -22.604,
     3.0806,  1.0461,  -1.9596,  -1.0285,  6.0004,  0.029215,
     -3.9636,  -28.92,  -2.898,  29.186,  -23.764,  -2.4579,
     -0.23497,  0.69241,  3.5511,  -7.5851,  3.5353,  -6.9025,
     -38.365,  -223.79,  67.911,  264.54,  36.4,  -92.584,
     -1.5661,  -17.055,  2.8109,  14.449,  10.622,  -5.8442,
     5.4276,  -13.749,  5.4469,  7.5521,  30.385,  -37.657,
     13.166,  -39.806,  10.708,  20.474,  83.479,  -83.531,
     0.96521,  1.8863,  5.3376,  -3.0963,  2.4241,  -8.85,
     1.9764,  3.0933,  11.912,  -5.0843,  5.212,  -19.153,
     -3.9636,  -2.898,  -28.92,  -2.4579,  -23.764,  29.186,
     -0.23497,  3.5511,  0.69241,  -6.9025,  3.5353,  -7.5851,
     96.472,  9.0778,  -190.83,  -22.604,  36.092,  162.69,
     3.0806,  -1.9596,  1.0461,  0.029215,  6.0004,  -1.0285,
     -38.365,  67.911,  -223.79,  -92.584,  36.4,  264.54,
     -1.5661,  2.8109,  -17.055,  -5.8442,  10.622,  14.449,
};

inline constexpr PidGains kLeftL0PidLow{5500.0f, 1.5f, 700.0f, 170.0f, 30.0f};
inline constexpr PidGains kRightL0PidLow{5500.0f, 1.5f, 700.0f, 170.0f, 30.0f};
inline constexpr PidGains kLeftL0PidMid{5000.0f, 1.5f, 80000.0f, 170.0f, 30.0f};
inline constexpr PidGains kRightL0PidMid{5000.0f, 1.5f, 80000.0f, 170.0f, 30.0f};
inline constexpr PidGains kLeftL0PidHigh{4500.0f, 1.2f, 500.0f, 160.0f, 25.0f};
inline constexpr PidGains kRightL0PidHigh{4500.0f, 1.2f, 500.0f, 160.0f, 25.0f};
inline constexpr PidGains kLeftL0PidJumpTwo{8000.0f, 0.0f, 70000.0f, 250.0f, 0.0f};
inline constexpr PidGains kRightL0PidJumpTwo{8000.0f, 0.0f, 70000.0f, 250.0f, 0.0f};
inline constexpr PidGains kLeftL0PidJumpThree{6500.0f, 0.15f, 60000.0f, 190.0f, 30.0f};
inline constexpr PidGains kRightL0PidJumpThree{6500.0f, 0.15f, 60000.0f, 190.0f, 30.0f};
inline constexpr PidGains kRollPid{800.0f, 0.5f, 50.0f, 180.0f, 20.0f};
inline constexpr PidGains kLeftLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};
inline constexpr PidGains kRightLegTurnPid{32.0f, 0.0f, 10.0f, 20.0f, 0.0f};
}  // namespace chassis

namespace control_loop {
inline constexpr std::int16_t kWheelSpinThreshold = 220;
inline constexpr std::int16_t kWheelActionThreshold = 320;
inline constexpr std::int16_t kWheelCenterThreshold = 80;
inline constexpr float kControlLoopDtS = 0.002f;
inline constexpr std::int16_t kDr16AxisMaxAbs = 660;
inline constexpr float kTargetForwardSpeedMinMps = 2.3f;
inline constexpr float kTargetForwardSpeedMaxMps = 2.3f;
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
inline constexpr float kPitchTargetMaxRad = 0.7f;
inline constexpr float kYawFollowRampStepRadS = 0.05f;
inline constexpr float kSpinYawRampStepRadS = 0.02f;
inline constexpr float kSpinTargetYawDotRadS = 8.0f;
inline constexpr float kSpinTranslationGain = 1.0f;
inline constexpr float kSpinThetaLlBiasRad = 0.0f;
inline constexpr float kYawFollowFixedTargetRad = 1.216f;
inline constexpr float kYawFollowSideOffsetRad = 0.5f * kPi;
inline constexpr float kGimbalStartupYawAlignErrorRad = 0.08f;
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
inline constexpr float kLeftWheelTorqueToCurrent = 2436.0f;
inline constexpr float kRightWheelTorqueToCurrent = 2436.0f;
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
inline constexpr float kLeftPhi1OffsetRad = -0.05f + kPi;
inline constexpr float kLeftPhi4OffsetRad = -0.59 + 0.07f;
inline constexpr float kRightPhi1OffsetRad = 2.7f + kPi;
inline constexpr float kRightPhi4OffsetRad = -2.11;
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
inline constexpr float kMidLegLengthM = 0.18f;
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
inline constexpr float kMidLegOffGroundRecoverLengthM = 0.16f;
inline constexpr uint32_t kMidLegOffGroundRecoverMs = 3000U;
inline constexpr uint32_t kMidLegOffGroundConfirmMs = 200U;
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

inline constexpr PidGains kLeftL0PidLow{8000.0f, 0.05f, 100000.0f, 180.0f, 12.0f};
inline constexpr PidGains kRightL0PidLow{9000.0f, 0.05f, 100000.0f, 180.0f, 12.0f};
inline constexpr PidGains kLeftL0PidMid{7500.0f, 0.04f, 90000.0f, 170.0f, 10.0f};
inline constexpr PidGains kRightL0PidMid{8500.0f, 0.04f, 90000.0f, 170.0f, 10.0f};
inline constexpr PidGains kLeftL0PidHigh{6800.0f, 0.03f, 80000.0f, 150.0f, 8.0f};
inline constexpr PidGains kRightL0PidHigh{7800.0f, 0.03f, 80000.0f, 150.0f, 8.0f};
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
                                               10.f, {0.f, 500.f}, {0.f, 5.f}};                       ///< 俯仰电机参数
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

inline constexpr float kStairClimbThetaThresholdRad = 0.5f;       ///< 上台阶腿摆角检测阈值///
inline constexpr float kStairClimbLegLengthM = 0.16f;             ///< 上台阶收腿目标///
inline constexpr float kStairClimbThetaTargetRad = 0.2f;          ///< 上台阶腿摆角目标///---------------上台阶参数
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
inline constexpr float kMidLegOffGroundRecoverLengthM = 0.16f;    ///< 中腿长离地恢复目标腿长
inline constexpr uint32_t kMidLegOffGroundRecoverMs = 300U;       ///< 中腿长离地恢复保持时间
inline constexpr uint32_t kMidLegOffGroundConfirmMs = 100U;      ///< 中腿长离地确认时间
inline constexpr float kRollBalanceTargetRad = 0.003f;            ///< 横滚平衡目标角
inline constexpr float kPostureThetaBMinRad = -0.7f;              ///< 姿态有效 body pitch 下限
inline constexpr float kPostureThetaBMaxRad = 0.7f;               ///< 姿态有效 body pitch 上限
inline constexpr float kPostureRollMinRad = -0.5f;                ///< 姿态有效 roll 下限
inline constexpr float kPostureRollMaxRad = 0.5f;                 ///< 姿态有效 roll 上限
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

inline constexpr PidGains kLeftL0PidLow{6500.0f, 0.18f, 55000.0f, 180.0f, 35.0f};         ///< 低腿长 PID
inline constexpr PidGains kRightL0PidLow{6500.0f, 0.18f, 55000.0f, 180.0f, 35.0f};        ///< 低腿长 PID
inline constexpr PidGains kLeftL0PidMid{6000.0f, 0.15f, 50000.0f, 170.0f, 30.0f};         ///< 中腿长 PID
inline constexpr PidGains kRightL0PidMid{6000.0f, 0.15f, 50000.0f, 170.0f, 30.0f};        ///< 中腿长 PID
inline constexpr PidGains kLeftL0PidHigh{5300.0f, 0.12f, 45000.0f, 150.0f, 25.0f};        ///< 高腿长 PID
inline constexpr PidGains kRightL0PidHigh{5300.0f, 0.12f, 45000.0f, 150.0f, 25.0f};       ///< 高腿长 PID
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
inline constexpr float kSpinTargetYawDotRadS = 10.0f;                    ///< 自旋目标偏航角速度
inline constexpr float kSpinTranslationGain = 1.2f;                      ///< 自旋平移增益
inline constexpr float kSpinThetaLlBiasRad = 0.0f;                       ///< 自旋左腿摆角偏置
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
inline constexpr float kWheelCurrentClampAbs = 16000.0f;      ///< 轮毂电流限幅绝对值
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
inline constexpr float kLeftPhi1OffsetRad = -1.50f + kPi;         ///< 左腿 phi1 零位偏移
inline constexpr float kLeftPhi4OffsetRad = -1.50f;               ///< 左腿 phi4 零位偏移
inline constexpr float kRightPhi1OffsetRad = -1.42f + kPi;        ///< 右腿 phi1 零位偏移
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
