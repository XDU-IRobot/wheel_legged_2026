#pragma once

#include <cstdint>

#include <librm/core/typedefs.hpp>

#include "roll_leg_mpc_params.hpp"
#include "roll_leg_mpc_static_solver.hpp"

namespace chassis {

class RollLegMpc {
 public:
  static constexpr int kNumStates = 7;
  static constexpr int kNumInputs = 2;

  enum class FallbackReason : std::uint8_t {
    kNone = 0,
    kNotInitialized,
    kInvalidInput,
    kUnsafeLegLength,
    kUnsafeLegAngle,
    kUnsafeRoll,
    kSetupFailed,
    kSolveFailed,
  };

  struct Config {
    int max_iter{roll_leg_mpc_params::kMaxIter};
    rm::f32 abs_pri_state_tol{roll_leg_mpc_params::kAbsPriStateTol};
    rm::f32 abs_pri_input_tol_n{roll_leg_mpc_params::kAbsPriInputTolN};
    rm::f32 abs_dua_state_tol{roll_leg_mpc_params::kAbsDuaStateTol};
    rm::f32 abs_dua_input_tol_n{roll_leg_mpc_params::kAbsDuaInputTolN};

    rm::f32 body_mass_kg{22.0f};
    rm::f32 leg_mass_kg{2.3f};
    rm::f32 gravity_mps2{9.81f};
    rm::f32 roll_balance_target_rad{0.0f};

    rm::f32 force_min_n{roll_leg_mpc_params::kForceMinN};
    rm::f32 force_max_n{roll_leg_mpc_params::kForceMaxN};
    rm::f32 force_slew_rate_n_per_s{roll_leg_mpc_params::kForceSlewRateNPerS};

    rm::f32 leg_safe_min_m{roll_leg_mpc_params::kLegSafeMinM};
    rm::f32 leg_safe_max_m{roll_leg_mpc_params::kLegSafeMaxM};
    rm::f32 cos_min{roll_leg_mpc_params::kCosMin};
    rm::f32 theta_mpc_max_rad{roll_leg_mpc_params::kThetaMpcMaxRad};
    rm::f32 roll_mpc_max_rad{roll_leg_mpc_params::kRollMpcMaxRad};
  };

  struct Input {
    rm::f32 left_leg_length_m{0.0f};
    rm::f32 right_leg_length_m{0.0f};
    rm::f32 left_leg_length_dot_mps{0.0f};
    rm::f32 right_leg_length_dot_mps{0.0f};

    rm::f32 left_leg_theta_rad{0.0f};
    rm::f32 right_leg_theta_rad{0.0f};

    rm::f32 roll_rad{0.0f};
    rm::f32 roll_rate_rad_s{0.0f};

    rm::f32 forward_speed_mps{0.0f};
    rm::f32 yaw_rate_rad_s{0.0f};

    rm::f32 target_leg_length_m{0.0f};
    rm::f32 target_roll_rad{0.0f};
    rm::f32 left_effective_mass_kg{0.0f};
    rm::f32 right_effective_mass_kg{0.0f};
    rm::f32 dt_s{0.0f};
  };

  struct Output {
    bool initialized{false};
    bool active{false};
    bool solved{false};
    FallbackReason fallback_reason{FallbackReason::kNotInitialized};
    int solver_iterations{0};

    rm::f32 left_force_n{0.0f};
    rm::f32 right_force_n{0.0f};
    rm::f32 dF_left_n{0.0f};
    rm::f32 dF_right_n{0.0f};
    rm::f32 gravity_left_n{0.0f};
    rm::f32 gravity_right_n{0.0f};
    rm::f32 cos_left{1.0f};
    rm::f32 cos_right{1.0f};

    rm::f32 e_L{0.0f};
    rm::f32 dL_c{0.0f};
    rm::f32 D{0.0f};
    rm::f32 dD{0.0f};
    rm::f32 e_roll{0.0f};
    rm::f32 droll{0.0f};
    rm::f32 a_y{0.0f};

    rm::f32 u_min_left_n{0.0f};
    rm::f32 u_max_left_n{0.0f};
    rm::f32 u_min_right_n{0.0f};
    rm::f32 u_max_right_n{0.0f};
    rm::f32 model_leg_length_m{0.0f};
    rm::f32 model_com_height_m{0.0f};
    rm::f32 model_roll_inertia_kg_m2{0.0f};
  };

  RollLegMpc() = default;
  ~RollLegMpc() = default;

  RollLegMpc(const RollLegMpc &) = delete;
  RollLegMpc &operator=(const RollLegMpc &) = delete;

  [[nodiscard]] static Config MakeDefaultConfig();

  bool Init(const Config &config = MakeDefaultConfig());
  void Reset();
  [[nodiscard]] Output Update(const Input &input);

  [[nodiscard]] bool initialized() const { return initialized_; }
  [[nodiscard]] const Config &config() const { return config_; }
  [[nodiscard]] rm::f32 solver_dt_s() const { return roll_leg_mpc_static_data::kDtS; }
  [[nodiscard]] const Output &last_output() const { return last_output_; }

 private:
  RollLegMpcStaticSolver static_solver_{};
  Config config_{};
  Output last_output_{};
  bool initialized_{false};
  rm::f32 last_left_force_n_{0.0f};
  rm::f32 last_right_force_n_{0.0f};
};

}  // namespace chassis
