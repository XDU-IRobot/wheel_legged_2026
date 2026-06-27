#pragma once

#include <cstdint>

#include <librm.hpp>

#include "../params.hpp"

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
    kBoundUpdateFailed,
    kSolveFailed,
  };

  struct Config {
    rm::f32 dt_s{0.01f};
    int horizon{15};
    rm::f32 rho{1.0f};
    int max_iter{60};
    rm::f32 abs_pri_tol{1e-3f};
    rm::f32 abs_dua_tol{1e-3f};

    rm::f32 body_mass_kg{wheel_legged::params::active::chassis::kBodyMassKg};
    rm::f32 leg_mass_kg{wheel_legged::params::active::chassis::kLegMassKg};
    rm::f32 gravity_mps2{wheel_legged::params::active::chassis::kGravityMps2};
    rm::f32 support_half_width_m{wheel_legged::params::active::chassis::kWheelRadiusM};
    rm::f32 com_height_m{0.28f};
    rm::f32 body_com_height_offset_m{0.0f};
    rm::f32 roll_inertia_kg_m2{0.0f};
    rm::f32 roll_balance_target_rad{wheel_legged::params::active::chassis::kRollBalanceTargetRad};

    rm::f32 force_min_n{0.0f};
    rm::f32 force_max_n{600.0f};
    rm::f32 force_slew_rate_n_per_s{20000.0f};

    rm::f32 leg_safe_min_m{0.08f};
    rm::f32 leg_safe_max_m{0.42f};
    rm::f32 cos_min{0.5f};
    rm::f32 theta_mpc_max_rad{0.7853982f};
    rm::f32 roll_mpc_max_rad{0.35f};

    rm::f32 a_dL{-8.0f};
    rm::f32 b_sum{0.0f};
    rm::f32 a_dD{-10.0f};
    rm::f32 b_D{0.0f};
    rm::f32 a_Drho{0.0f};
    rm::f32 a_rho{0.0f};
    rm::f32 a_drho{-4.0f};
    rm::f32 b_lrho{0.0f};
    rm::f32 b_rrho{0.0f};
    rm::f32 b_ay{0.0f};

    rm::f32 q_L{500.0f};
    rm::f32 q_dL{50.0f};
    rm::f32 q_D{300.0f};
    rm::f32 q_dD{30.0f};
    rm::f32 q_roll{5000.0f};
    rm::f32 q_droll{300.0f};
    rm::f32 q_ay{0.0f};
    rm::f32 r_left{0.01f};
    rm::f32 r_right{0.01f};
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
    rm::f32 target_roll_rad{wheel_legged::params::active::chassis::kRollBalanceTargetRad};
    rm::f32 left_effective_mass_kg{0.0f};
    rm::f32 right_effective_mass_kg{0.0f};
    rm::f32 dt_s{0.0f};
  };

  struct Output {
    bool initialized{false};
    bool active{false};
    bool solved{false};
    FallbackReason fallback_reason{FallbackReason::kNotInitialized};
    int solver_status{0};
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
    rm::f32 model_com_height_m{0.0f};
    rm::f32 model_roll_inertia_kg_m2{0.0f};
  };

  RollLegMpc() = default;
  ~RollLegMpc();

  RollLegMpc(const RollLegMpc &) = delete;
  RollLegMpc &operator=(const RollLegMpc &) = delete;

  [[nodiscard]] static Config MakeDefaultConfig();
  static void ApplyPhysicalModel(Config &config, rm::f32 com_height_m);

  bool Init(const Config &config = MakeDefaultConfig());
  void Reset();
  [[nodiscard]] Output Update(const Input &input);

  [[nodiscard]] bool initialized() const { return initialized_; }
  [[nodiscard]] const Config &config() const { return config_; }
  [[nodiscard]] const Output &last_output() const { return last_output_; }

 private:
  void ReleaseSolver();

  void *solver_{nullptr};
  Config config_{};
  Output last_output_{};
  bool initialized_{false};
  rm::f32 last_left_force_n_{0.0f};
  rm::f32 last_right_force_n_{0.0f};
};

}  // namespace chassis
