#include "include/chassis/roll_leg_mpc.hpp"

#include <algorithm>
#include <cmath>

namespace {

using Mpc = chassis::RollLegMpc;
namespace StaticData = chassis::roll_leg_mpc_static_data;

constexpr rm::f32 kMinPositive = 1.0e-5f;

enum StateIndex : int {
  kEL = 0,
  kDL = 1,
  kD = 2,
  kDD = 3,
  kERoll = 4,
  kDRoll = 5,
  kAy = 6,
};

enum InputIndex : int {
  kDFLeft = 0,
  kDFRight = 1,
};

rm::f32 ClampFinite(const rm::f32 value, const rm::f32 fallback = 0.0f) {
  return std::isfinite(value) ? value : fallback;
}

rm::f32 SafeAbsLimit(const rm::f32 value, const rm::f32 limit) {
  const rm::f32 lim = std::fabs(limit);
  return std::clamp(value, -lim, lim);
}

rm::f32 SlewLimit(const rm::f32 target, const rm::f32 previous, const rm::f32 max_delta) {
  return previous + std::clamp(target - previous, -max_delta, max_delta);
}

rm::f32 SafeCos(const rm::f32 angle_rad, const rm::f32 cos_min) { return std::max(std::cos(angle_rad), cos_min); }

rm::f32 DefaultEffectiveLegMassKg(const rm::f32 body_mass_kg, const rm::f32 leg_mass_kg) {
  return 0.5f * body_mass_kg + 0.5f * leg_mass_kg;
}

bool IsSafeLegLength(const Mpc::Input &input, const Mpc::Config &config) {
  return input.left_leg_length_m > config.leg_safe_min_m && input.left_leg_length_m < config.leg_safe_max_m &&
         input.right_leg_length_m > config.leg_safe_min_m && input.right_leg_length_m < config.leg_safe_max_m;
}

const StaticData::ModelData &SelectModelForLegLength(const rm::f32 leg_length_m) {
  const rm::f32 leg = std::clamp(leg_length_m, StaticData::kLegLengthMinM, StaticData::kLegLengthMaxM);
  int best_index = 0;
  rm::f32 best_error = std::fabs(leg - StaticData::kModels[0].nominal_leg_length_m);
  for (int i = 1; i < StaticData::kNumModels; ++i) {
    const rm::f32 error = std::fabs(leg - StaticData::kModels[i].nominal_leg_length_m);
    if (error < best_error) {
      best_index = i;
      best_error = error;
    }
  }
  return StaticData::kModels[best_index];
}

void BuildState(const Mpc::Input &input, rm::f32 (&x)[Mpc::kNumStates]) {
  const rm::f32 L_c = 0.5f * (input.left_leg_length_m + input.right_leg_length_m);
  const rm::f32 dL_c = 0.5f * (input.left_leg_length_dot_mps + input.right_leg_length_dot_mps);
  const rm::f32 D = input.right_leg_length_m - input.left_leg_length_m;
  const rm::f32 dD = input.right_leg_length_dot_mps - input.left_leg_length_dot_mps;
  const rm::f32 e_roll = input.roll_rad - input.target_roll_rad;
  const rm::f32 a_y = input.forward_speed_mps * input.yaw_rate_rad_s;

  x[kEL] = L_c - input.target_leg_length_m;
  x[kDL] = dL_c;
  x[kD] = D;
  x[kDD] = dD;
  x[kERoll] = e_roll;
  x[kDRoll] = input.roll_rate_rad_s;
  x[kAy] = a_y;
}

void FillStateDebug(const rm::f32 (&x)[Mpc::kNumStates], Mpc::Output &output) {
  output.e_L = x[kEL];
  output.dL_c = x[kDL];
  output.D = x[kD];
  output.dD = x[kDD];
  output.e_roll = x[kERoll];
  output.droll = x[kDRoll];
  output.a_y = x[kAy];
}

}  // namespace

namespace chassis {

RollLegMpc::Config RollLegMpc::MakeDefaultConfig() {
  Config config{};
  return config;
}

bool RollLegMpc::Init(const Config &config) {
  config_ = config;
  last_output_ = {};
  last_output_.initialized = false;
  last_output_.fallback_reason = FallbackReason::kSetupFailed;
  last_left_force_n_ = 0.0f;
  last_right_force_n_ = 0.0f;
  static_solver_.Reset();

  if (StaticData::kDtS <= 0.0f || StaticData::kHorizon < 2 || config_.max_iter < 0) {
    initialized_ = false;
    return false;
  }

  initialized_ = true;
  last_output_.initialized = true;
  last_output_.fallback_reason = FallbackReason::kNone;
  return true;
}

void RollLegMpc::Reset() {
  last_output_ = {};
  last_output_.initialized = initialized_;
  last_output_.fallback_reason = initialized_ ? FallbackReason::kNone : FallbackReason::kNotInitialized;
  last_left_force_n_ = 0.0f;
  last_right_force_n_ = 0.0f;
  static_solver_.Reset();
}

RollLegMpc::Output RollLegMpc::Update(const Input &input) {
  Output output{};
  output.initialized = initialized_;

  if (!initialized_) {
    output.fallback_reason = FallbackReason::kNotInitialized;
    last_output_ = output;
    return output;
  }

  const rm::f32 avg_leg_length_m = 0.5f * (input.left_leg_length_m + input.right_leg_length_m);
  const StaticData::ModelData &model = SelectModelForLegLength(avg_leg_length_m);
  output.model_leg_length_m = model.nominal_leg_length_m;
  output.model_com_height_m = model.model_com_height_m;
  output.model_roll_inertia_kg_m2 = model.model_roll_inertia_kg_m2;

  rm::f32 x0[kNumStates]{};
  BuildState(input, x0);
  FillStateDebug(x0, output);

  const bool input_finite = std::isfinite(input.left_leg_length_m) && std::isfinite(input.right_leg_length_m) &&
                            std::isfinite(input.left_leg_length_dot_mps) &&
                            std::isfinite(input.right_leg_length_dot_mps) && std::isfinite(input.left_leg_theta_rad) &&
                            std::isfinite(input.right_leg_theta_rad) && std::isfinite(input.roll_rad) &&
                            std::isfinite(input.roll_rate_rad_s) && std::isfinite(input.forward_speed_mps) &&
                            std::isfinite(input.yaw_rate_rad_s) && std::isfinite(input.target_leg_length_m) &&
                            std::isfinite(input.target_roll_rad) && std::isfinite(input.left_effective_mass_kg) &&
                            std::isfinite(input.right_effective_mass_kg);
  if (!input_finite) {
    output.fallback_reason = FallbackReason::kInvalidInput;
    last_output_ = output;
    return output;
  }

  if (!IsSafeLegLength(input, config_)) {
    output.fallback_reason = FallbackReason::kUnsafeLegLength;
    last_output_ = output;
    return output;
  }

  if (std::fabs(input.left_leg_theta_rad) > config_.theta_mpc_max_rad ||
      std::fabs(input.right_leg_theta_rad) > config_.theta_mpc_max_rad) {
    output.fallback_reason = FallbackReason::kUnsafeLegAngle;
    last_output_ = output;
    return output;
  }

  if (std::fabs(output.e_roll) > config_.roll_mpc_max_rad) {
    output.fallback_reason = FallbackReason::kUnsafeRoll;
    last_output_ = output;
    return output;
  }

  output.cos_left = SafeCos(input.left_leg_theta_rad, config_.cos_min);
  output.cos_right = SafeCos(input.right_leg_theta_rad, config_.cos_min);

  const rm::f32 left_mass_kg = input.left_effective_mass_kg > kMinPositive
                                   ? input.left_effective_mass_kg
                                   : DefaultEffectiveLegMassKg(config_.body_mass_kg, config_.leg_mass_kg);
  const rm::f32 right_mass_kg = input.right_effective_mass_kg > kMinPositive
                                    ? input.right_effective_mass_kg
                                    : DefaultEffectiveLegMassKg(config_.body_mass_kg, config_.leg_mass_kg);
  output.gravity_left_n = left_mass_kg * config_.gravity_mps2 / output.cos_left;
  output.gravity_right_n = right_mass_kg * config_.gravity_mps2 / output.cos_right;

  output.u_min_left_n = config_.force_min_n - output.gravity_left_n;
  output.u_max_left_n = config_.force_max_n - output.gravity_left_n;
  output.u_min_right_n = config_.force_min_n - output.gravity_right_n;
  output.u_max_right_n = config_.force_max_n - output.gravity_right_n;

  RollLegMpcStaticSolver::Input static_input{};
  for (int i = 0; i < kNumStates; ++i) {
    static_input.x0[i] = x0[i];
  }
  static_input.u_min[kDFLeft] = output.u_min_left_n;
  static_input.u_min[kDFRight] = output.u_min_right_n;
  static_input.u_max[kDFLeft] = output.u_max_left_n;
  static_input.u_max[kDFRight] = output.u_max_right_n;

  RollLegMpcStaticSolver::Settings settings{};
  settings.max_iter = config_.max_iter;
  settings.abs_pri_state_tol = config_.abs_pri_state_tol;
  settings.abs_pri_input_tol_n = config_.abs_pri_input_tol_n;
  settings.abs_dua_state_tol = config_.abs_dua_state_tol;
  settings.abs_dua_input_tol_n = config_.abs_dua_input_tol_n;
  settings.always_return_last_iterate = true;
  const RollLegMpcStaticSolver::Output solve_output = static_solver_.Solve(model, static_input, settings);
  output.solver_iterations = solve_output.iterations;
  if (!solve_output.usable) {
    output.fallback_reason = FallbackReason::kSolveFailed;
    last_output_ = output;
    return output;
  }

  output.dF_left_n = ClampFinite(solve_output.u0[kDFLeft]);
  output.dF_right_n = ClampFinite(solve_output.u0[kDFRight]);

  rm::f32 left_force = std::clamp(output.gravity_left_n + output.dF_left_n, config_.force_min_n, config_.force_max_n);
  rm::f32 right_force =
      std::clamp(output.gravity_right_n + output.dF_right_n, config_.force_min_n, config_.force_max_n);

  const rm::f32 dt_s = (input.dt_s > 0.0f) ? input.dt_s : StaticData::kDtS;
  const rm::f32 max_force_delta = std::max(config_.force_slew_rate_n_per_s * dt_s, 0.0f);
  if (last_output_.active) {
    left_force = SlewLimit(left_force, last_left_force_n_, max_force_delta);
    right_force = SlewLimit(right_force, last_right_force_n_, max_force_delta);
  }

  output.left_force_n = SafeAbsLimit(left_force, config_.force_max_n);
  output.right_force_n = SafeAbsLimit(right_force, config_.force_max_n);
  output.dF_left_n = output.left_force_n - output.gravity_left_n;
  output.dF_right_n = output.right_force_n - output.gravity_right_n;
  output.active = true;
  output.solved = solve_output.converged;
  output.fallback_reason = FallbackReason::kNone;

  last_left_force_n_ = output.left_force_n;
  last_right_force_n_ = output.right_force_n;
  last_output_ = output;
  return output;
}

}  // namespace chassis
