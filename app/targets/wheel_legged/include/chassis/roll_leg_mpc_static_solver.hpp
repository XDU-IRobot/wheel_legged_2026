#pragma once

#include <cstdint>

#include <librm/core/typedefs.hpp>

#include "roll_leg_mpc_static_data.hpp"

namespace chassis {

class RollLegMpcStaticSolver {
 public:
  static constexpr int kNumStates = roll_leg_mpc_static_data::kNumStates;
  static constexpr int kNumInputs = roll_leg_mpc_static_data::kNumInputs;
  static constexpr int kHorizon = roll_leg_mpc_static_data::kHorizon;
  using ModelData = roll_leg_mpc_static_data::ModelData;

  struct Settings {
    int max_iter{20};
    rm::f32 abs_pri_state_tol{0.05f};
    rm::f32 abs_pri_input_tol_n{5.0f};
    rm::f32 abs_dua_state_tol{0.05f};
    rm::f32 abs_dua_input_tol_n{5.0f};
    bool always_return_last_iterate{true};
  };

  struct Input {
    rm::f32 x0[kNumStates]{};
    rm::f32 u_min[kNumInputs]{};
    rm::f32 u_max[kNumInputs]{};
  };

  struct Output {
    rm::f32 u0[kNumInputs]{};
    int iterations{0};
    bool converged{false};
    bool usable{false};
    rm::f32 primal_residual_state{0.0f};
    rm::f32 primal_residual_input{0.0f};
    rm::f32 dual_residual_state{0.0f};
    rm::f32 dual_residual_input{0.0f};
  };

  struct Workspace {
    rm::f32 x[kNumStates][kHorizon]{};
    rm::f32 u[kNumInputs][kHorizon - 1]{};
    rm::f32 q[kNumStates][kHorizon]{};
    rm::f32 r[kNumInputs][kHorizon - 1]{};
    rm::f32 p[kNumStates][kHorizon]{};
    rm::f32 d[kNumInputs][kHorizon - 1]{};
    rm::f32 v[kNumStates][kHorizon]{};
    rm::f32 vnew[kNumStates][kHorizon]{};
    rm::f32 z[kNumInputs][kHorizon - 1]{};
    rm::f32 znew[kNumInputs][kHorizon - 1]{};
    rm::f32 g[kNumStates][kHorizon]{};
    rm::f32 y[kNumInputs][kHorizon - 1]{};
  };

  RollLegMpcStaticSolver() = default;

  void Reset();
  [[nodiscard]] Output Solve(const ModelData &model, const Input &input);
  [[nodiscard]] Output Solve(const ModelData &model, const Input &input, const Settings &settings);
  [[nodiscard]] const Workspace &workspace() const { return workspace_; }

 private:
  Workspace workspace_{};
};

}  // namespace chassis
