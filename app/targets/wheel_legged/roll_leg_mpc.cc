#include "include/chassis/roll_leg_mpc.hpp"

#include <algorithm>
#include <cmath>

#include <tinympc/tiny_api.hpp>

namespace {

constexpr tinytype kLargeBound = 1.0e17;
constexpr rm::f32 kMinPositive = 1.0e-5f;

using Mpc = chassis::RollLegMpc;

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

tinyMatrix BuildContinuousA(const Mpc::Config &config) {
  tinyMatrix a = tinyMatrix::Zero(Mpc::kNumStates, Mpc::kNumStates);
  a(kEL, kDL) = 1.0;
  a(kDL, kDL) = config.a_dL;
  a(kD, kDD) = 1.0;
  a(kDD, kDD) = config.a_dD;
  a(kERoll, kDRoll) = 1.0;
  a(kDRoll, kD) = config.a_Drho;
  a(kDRoll, kERoll) = config.a_rho;
  a(kDRoll, kDRoll) = config.a_drho;
  a(kDRoll, kAy) = config.b_ay;
  return a;
}

tinyMatrix BuildContinuousB(const Mpc::Config &config) {
  tinyMatrix b = tinyMatrix::Zero(Mpc::kNumStates, Mpc::kNumInputs);
  b(kDL, kDFLeft) = config.b_sum;
  b(kDL, kDFRight) = config.b_sum;
  b(kDD, kDFLeft) = -config.b_D;
  b(kDD, kDFRight) = config.b_D;
  b(kDRoll, kDFLeft) = config.b_lrho;
  b(kDRoll, kDFRight) = config.b_rrho;
  return b;
}

tinyVector BuildState(const Mpc::Input &input) {
  const rm::f32 L_c = 0.5f * (input.left_leg_length_m + input.right_leg_length_m);
  const rm::f32 dL_c = 0.5f * (input.left_leg_length_dot_mps + input.right_leg_length_dot_mps);
  const rm::f32 D = input.right_leg_length_m - input.left_leg_length_m;
  const rm::f32 dD = input.right_leg_length_dot_mps - input.left_leg_length_dot_mps;
  const rm::f32 e_roll = input.roll_rad - input.target_roll_rad;
  const rm::f32 a_y = input.forward_speed_mps * input.yaw_rate_rad_s;

  tinyVector x = tinyVector::Zero(Mpc::kNumStates);
  x(kEL) = L_c - input.target_leg_length_m;
  x(kDL) = dL_c;
  x(kD) = D;
  x(kDD) = dD;
  x(kERoll) = e_roll;
  x(kDRoll) = input.roll_rate_rad_s;
  x(kAy) = a_y;
  return x;
}

void FillStateDebug(const tinyVector &x, Mpc::Output &output) {
  output.e_L = static_cast<rm::f32>(x(kEL));
  output.dL_c = static_cast<rm::f32>(x(kDL));
  output.D = static_cast<rm::f32>(x(kD));
  output.dD = static_cast<rm::f32>(x(kDD));
  output.e_roll = static_cast<rm::f32>(x(kERoll));
  output.droll = static_cast<rm::f32>(x(kDRoll));
  output.a_y = static_cast<rm::f32>(x(kAy));
}

void DeleteSolver(TinySolver *solver) {
  if (solver == nullptr) {
    return;
  }
  delete solver->solution;
  delete solver->settings;
  delete solver->cache;
  delete solver->work;
  delete solver;
}

}  // namespace

namespace chassis {

RollLegMpc::~RollLegMpc() { ReleaseSolver(); }

RollLegMpc::Config RollLegMpc::MakeDefaultConfig() {
  Config config{};
  ApplyPhysicalModel(config, config.com_height_m);
  return config;
}

void RollLegMpc::ApplyPhysicalModel(Config &config, const rm::f32 com_height_m) {
  const rm::f32 body_mass_kg = std::max(config.body_mass_kg, kMinPositive);
  const rm::f32 leg_mass_kg = std::max(config.leg_mass_kg, kMinPositive);
  const rm::f32 total_mass_kg = body_mass_kg + 2.0f * leg_mass_kg;
  const rm::f32 half_width_m = std::max(config.support_half_width_m, 0.05f);
  const rm::f32 model_height_m = std::max(com_height_m, 0.05f);
  const rm::f32 default_roll_inertia_kg_m2 = total_mass_kg * half_width_m * half_width_m;
  const rm::f32 roll_inertia_kg_m2 =
      std::max(config.roll_inertia_kg_m2 > kMinPositive ? config.roll_inertia_kg_m2 : default_roll_inertia_kg_m2, 0.1f);

  config.com_height_m = model_height_m;
  config.roll_inertia_kg_m2 = roll_inertia_kg_m2;
  config.b_sum = 1.0f / total_mass_kg;
  config.b_D = 1.0f / std::max(leg_mass_kg, 1.0f);
  config.a_rho = total_mass_kg * config.gravity_mps2 * model_height_m / roll_inertia_kg_m2;
  config.a_Drho = -config.a_rho / half_width_m;
  config.b_lrho = -half_width_m / roll_inertia_kg_m2;
  config.b_rrho = half_width_m / roll_inertia_kg_m2;
  config.b_ay = total_mass_kg * model_height_m / roll_inertia_kg_m2;
}

bool RollLegMpc::Init(const Config &config) {
  ReleaseSolver();
  config_ = config;
  last_output_ = {};
  last_output_.initialized = false;
  last_output_.fallback_reason = FallbackReason::kSetupFailed;
  last_left_force_n_ = 0.0f;
  last_right_force_n_ = 0.0f;

  if (config_.dt_s <= 0.0f || config_.horizon < 2) {
    initialized_ = false;
    return false;
  }

  const tinyMatrix a_continuous = BuildContinuousA(config_);
  const tinyMatrix b_continuous = BuildContinuousB(config_);
  const tinyMatrix a_discrete =
      tinyMatrix::Identity(kNumStates, kNumStates) + static_cast<tinytype>(config_.dt_s) * a_continuous;
  const tinyMatrix b_discrete = static_cast<tinytype>(config_.dt_s) * b_continuous;
  const tinyVector f_discrete = tinyVector::Zero(kNumStates);

  tinyVector q_diag(kNumStates);
  q_diag << config_.q_L, config_.q_dL, config_.q_D, config_.q_dD, config_.q_roll, config_.q_droll, config_.q_ay;

  tinyVector r_diag(kNumInputs);
  r_diag << config_.r_left, config_.r_right;

  TinySolver *solver = nullptr;
  const int setup_status = tiny_setup(&solver, a_discrete, b_discrete, f_discrete, q_diag.asDiagonal(),
                                      r_diag.asDiagonal(), config_.rho, kNumStates, kNumInputs, config_.horizon, 0);
  if (setup_status != 0 || solver == nullptr) {
    DeleteSolver(solver);
    initialized_ = false;
    return false;
  }

  const tinyMatrix x_min = tinyMatrix::Constant(kNumStates, config_.horizon, -kLargeBound);
  const tinyMatrix x_max = tinyMatrix::Constant(kNumStates, config_.horizon, kLargeBound);
  const tinyMatrix u_min = tinyMatrix::Constant(kNumInputs, config_.horizon - 1, -kLargeBound);
  const tinyMatrix u_max = tinyMatrix::Constant(kNumInputs, config_.horizon - 1, kLargeBound);
  const int bound_status = tiny_set_bound_constraints(solver, x_min, x_max, u_min, u_max);
  if (bound_status != 0) {
    DeleteSolver(solver);
    initialized_ = false;
    return false;
  }

  solver->settings->max_iter = config_.max_iter;
  solver->settings->abs_pri_tol = config_.abs_pri_tol;
  solver->settings->abs_dua_tol = config_.abs_dua_tol;
  solver->settings->check_termination = 1;
  solver->settings->en_state_bound = 0;
  solver->settings->en_input_bound = 1;

  solver_ = solver;
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
}

RollLegMpc::Output RollLegMpc::Update(const Input &input) {
  Output output{};
  output.initialized = initialized_;
  output.model_com_height_m = config_.com_height_m;
  output.model_roll_inertia_kg_m2 = config_.roll_inertia_kg_m2;

  if (!initialized_ || solver_ == nullptr) {
    output.fallback_reason = FallbackReason::kNotInitialized;
    last_output_ = output;
    return output;
  }

  auto *solver = static_cast<TinySolver *>(solver_);
  const tinyVector x0 = BuildState(input);
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

  tinyMatrix u_min = tinyMatrix::Zero(kNumInputs, config_.horizon - 1);
  tinyMatrix u_max = tinyMatrix::Zero(kNumInputs, config_.horizon - 1);
  for (int i = 0; i < config_.horizon - 1; ++i) {
    u_min(kDFLeft, i) = output.u_min_left_n;
    u_min(kDFRight, i) = output.u_min_right_n;
    u_max(kDFLeft, i) = output.u_max_left_n;
    u_max(kDFRight, i) = output.u_max_right_n;
  }

  const tinyMatrix x_min = tinyMatrix::Constant(kNumStates, config_.horizon, -kLargeBound);
  const tinyMatrix x_max = tinyMatrix::Constant(kNumStates, config_.horizon, kLargeBound);
  if (tiny_set_bound_constraints(solver, x_min, x_max, u_min, u_max) != 0) {
    output.fallback_reason = FallbackReason::kBoundUpdateFailed;
    last_output_ = output;
    return output;
  }

  const int x0_status = tiny_set_x0(solver, x0);
  if (x0_status != 0) {
    output.fallback_reason = FallbackReason::kInvalidInput;
    last_output_ = output;
    return output;
  }

  const int solve_status = tiny_solve(solver);
  output.solver_status = solver->work != nullptr ? solver->work->status : solve_status;
  output.solver_iterations = solver->solution != nullptr ? solver->solution->iter : 0;
  if (solve_status != 0 || solver->solution == nullptr || solver->solution->solved == 0 ||
      solver->solution->u.cols() <= 0) {
    output.fallback_reason = FallbackReason::kSolveFailed;
    last_output_ = output;
    return output;
  }

  output.dF_left_n = ClampFinite(static_cast<rm::f32>(solver->solution->u(kDFLeft, 0)));
  output.dF_right_n = ClampFinite(static_cast<rm::f32>(solver->solution->u(kDFRight, 0)));

  rm::f32 left_force = std::clamp(output.gravity_left_n + output.dF_left_n, config_.force_min_n, config_.force_max_n);
  rm::f32 right_force =
      std::clamp(output.gravity_right_n + output.dF_right_n, config_.force_min_n, config_.force_max_n);

  const rm::f32 dt_s = (input.dt_s > 0.0f) ? input.dt_s : config_.dt_s;
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
  output.solved = true;
  output.fallback_reason = FallbackReason::kNone;

  last_left_force_n_ = output.left_force_n;
  last_right_force_n_ = output.right_force_n;
  last_output_ = output;
  return output;
}

void RollLegMpc::ReleaseSolver() {
  DeleteSolver(static_cast<TinySolver *>(solver_));
  solver_ = nullptr;
  initialized_ = false;
}

}  // namespace chassis
