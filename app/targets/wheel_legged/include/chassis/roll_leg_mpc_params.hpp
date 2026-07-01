#pragma once

#include <cstdint>

namespace chassis::roll_leg_mpc_params {

// Runtime scheduling.
inline constexpr std::uint8_t kUpdatePeriodTicks = 5U;  // 100 Hz MPC inside the 500 Hz chassis loop.

// ADMM solver settings.
inline constexpr int kMaxIter = 60;
inline constexpr float kAbsPriStateTol = 0.05f;
inline constexpr float kAbsPriInputTolN = 5.0f;
inline constexpr float kAbsDuaStateTol = 0.05f;
inline constexpr float kAbsDuaInputTolN = 5.0f;

// Output limits.
inline constexpr float kForceMinN = -300.0f;
inline constexpr float kForceMaxN = 300.0f;
inline constexpr float kForceSlewRateNPerS = 20000.0f;

// MPC enable/safety window.
inline constexpr float kLegSafeMinM = 0.08f;
inline constexpr float kLegSafeMaxM = 0.42f;
inline constexpr float kCosMin = 0.5f;
inline constexpr float kThetaMpcMaxRad = 0.7853982f;
inline constexpr float kRollMpcMaxRad = 0.35f;

// Static model generation settings.
inline constexpr float kModelDtS = 0.01f;
inline constexpr int kModelHorizon = 15;
inline constexpr float kModelRho = 1.0f;
inline constexpr float kSupportHalfWidthM = 0.2025f;
inline constexpr float kModelLegMinM = 0.14f;
inline constexpr float kModelLegMaxM = 0.35f;
inline constexpr float kModelLegStepM = 0.03f;
inline constexpr float kWheelRadiusM = 0.0575f;
inline constexpr float kBodyComHeightOffsetM = 0.0f;
inline constexpr float kRollInertiaKgM2 = 0.0f;

// Linearized continuous model shaping.
inline constexpr float kADl = -8.0f;
inline constexpr float kADd = -10.0f;
inline constexpr float kADroll = -4.0f;

// MPC state/input weights.
inline constexpr float kQL = 200000.0f;
inline constexpr float kQDl = 50.0f;
inline constexpr float kQD = 300.0f;
inline constexpr float kQDd = 30.0f;
inline constexpr float kQRoll = 5000.0f;
inline constexpr float kQDroll = 300.0f;
inline constexpr float kQAy = 0.0f;
inline constexpr float kRLeft = 0.01f;
inline constexpr float kRRight = 0.01f;

}  // namespace chassis::roll_leg_mpc_params
