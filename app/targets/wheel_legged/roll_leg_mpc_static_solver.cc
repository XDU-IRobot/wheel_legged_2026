#include "include/chassis/roll_leg_mpc_static_solver.hpp"

#include <algorithm>
#include <cmath>

namespace {

using Solver = chassis::RollLegMpcStaticSolver;

rm::f32 AbsMax(const rm::f32 value, const rm::f32 current) { return std::max(std::fabs(value), current); }

rm::f32 ClampInput(const rm::f32 value, const rm::f32 lower, const rm::f32 upper) {
  return std::clamp(value, std::min(lower, upper), std::max(lower, upper));
}

void ZeroWorkspace(Solver::Workspace &work) {
  for (int i = 0; i < Solver::kNumStates; ++i) {
    for (int k = 0; k < Solver::kHorizon; ++k) {
      work.x[i][k] = 0.0f;
      work.q[i][k] = 0.0f;
      work.p[i][k] = 0.0f;
      work.v[i][k] = 0.0f;
      work.vnew[i][k] = 0.0f;
      work.g[i][k] = 0.0f;
    }
  }

  for (int i = 0; i < Solver::kNumInputs; ++i) {
    for (int k = 0; k < Solver::kHorizon - 1; ++k) {
      work.u[i][k] = 0.0f;
      work.r[i][k] = 0.0f;
      work.d[i][k] = 0.0f;
      work.z[i][k] = 0.0f;
      work.znew[i][k] = 0.0f;
      work.y[i][k] = 0.0f;
    }
  }
}

void UpdateLinearCost(Solver::Workspace &work) {
  for (int k = 0; k < Solver::kHorizon; ++k) {
    for (int i = 0; i < Solver::kNumStates; ++i) {
      work.q[i][k] = -chassis::roll_leg_mpc_static_data::kRho * (work.vnew[i][k] - work.g[i][k]);
    }
  }

  for (int k = 0; k < Solver::kHorizon - 1; ++k) {
    for (int i = 0; i < Solver::kNumInputs; ++i) {
      work.r[i][k] = -chassis::roll_leg_mpc_static_data::kRho * (work.znew[i][k] - work.y[i][k]);
    }
  }

  const int terminal = Solver::kHorizon - 1;
  for (int i = 0; i < Solver::kNumStates; ++i) {
    work.p[i][terminal] = -chassis::roll_leg_mpc_static_data::kRho * (work.vnew[i][terminal] - work.g[i][terminal]);
  }
}

void BackwardPassGrad(const Solver::ModelData &model, Solver::Workspace &work) {
  for (int k = Solver::kHorizon - 2; k >= 0; --k) {
    rm::f32 tmp_u[Solver::kNumInputs]{};
    for (int i = 0; i < Solver::kNumInputs; ++i) {
      tmp_u[i] = work.r[i][k] + model.bpf[i];
      for (int j = 0; j < Solver::kNumStates; ++j) {
        tmp_u[i] += model.bdyn[j][i] * work.p[j][k + 1];
      }
    }

    for (int i = 0; i < Solver::kNumInputs; ++i) {
      work.d[i][k] = 0.0f;
      for (int j = 0; j < Solver::kNumInputs; ++j) {
        work.d[i][k] += model.quu_inv[i][j] * tmp_u[j];
      }
    }

    for (int i = 0; i < Solver::kNumStates; ++i) {
      rm::f32 value = work.q[i][k] + model.apf[i];
      for (int j = 0; j < Solver::kNumStates; ++j) {
        value += model.am_bk_t[i][j] * work.p[j][k + 1];
      }
      for (int j = 0; j < Solver::kNumInputs; ++j) {
        value -= model.kinf[j][i] * work.r[j][k];
      }
      work.p[i][k] = value;
    }
  }
}

void ForwardPass(const Solver::ModelData &model, Solver::Workspace &work) {
  for (int k = 0; k < Solver::kHorizon - 1; ++k) {
    for (int i = 0; i < Solver::kNumInputs; ++i) {
      rm::f32 value = -work.d[i][k];
      for (int j = 0; j < Solver::kNumStates; ++j) {
        value -= model.kinf[i][j] * work.x[j][k];
      }
      work.u[i][k] = value;
    }

    for (int i = 0; i < Solver::kNumStates; ++i) {
      rm::f32 value = model.fd[i];
      for (int j = 0; j < Solver::kNumStates; ++j) {
        value += model.adyn[i][j] * work.x[j][k];
      }
      for (int j = 0; j < Solver::kNumInputs; ++j) {
        value += model.bdyn[i][j] * work.u[j][k];
      }
      work.x[i][k + 1] = value;
    }
  }
}

void UpdateSlack(Solver::Workspace &work, const Solver::Input &input) {
  for (int k = 0; k < Solver::kHorizon; ++k) {
    for (int i = 0; i < Solver::kNumStates; ++i) {
      work.vnew[i][k] = work.x[i][k] + work.g[i][k];
    }
  }

  for (int k = 0; k < Solver::kHorizon - 1; ++k) {
    for (int i = 0; i < Solver::kNumInputs; ++i) {
      work.znew[i][k] = ClampInput(work.u[i][k] + work.y[i][k], input.u_min[i], input.u_max[i]);
    }
  }
}

void UpdateDual(Solver::Workspace &work) {
  for (int k = 0; k < Solver::kHorizon; ++k) {
    for (int i = 0; i < Solver::kNumStates; ++i) {
      work.g[i][k] += work.x[i][k] - work.vnew[i][k];
    }
  }

  for (int k = 0; k < Solver::kHorizon - 1; ++k) {
    for (int i = 0; i < Solver::kNumInputs; ++i) {
      work.y[i][k] += work.u[i][k] - work.znew[i][k];
    }
  }
}

Solver::Output ComputeResiduals(const Solver::Workspace &work) {
  Solver::Output output{};
  for (int k = 0; k < Solver::kHorizon; ++k) {
    for (int i = 0; i < Solver::kNumStates; ++i) {
      output.primal_residual_state = AbsMax(work.x[i][k] - work.vnew[i][k], output.primal_residual_state);
      output.dual_residual_state = AbsMax(chassis::roll_leg_mpc_static_data::kRho * (work.v[i][k] - work.vnew[i][k]),
                                          output.dual_residual_state);
    }
  }

  for (int k = 0; k < Solver::kHorizon - 1; ++k) {
    for (int i = 0; i < Solver::kNumInputs; ++i) {
      output.primal_residual_input = AbsMax(work.u[i][k] - work.znew[i][k], output.primal_residual_input);
      output.dual_residual_input = AbsMax(chassis::roll_leg_mpc_static_data::kRho * (work.z[i][k] - work.znew[i][k]),
                                          output.dual_residual_input);
    }
  }
  return output;
}

void SavePreviousSlack(Solver::Workspace &work) {
  for (int k = 0; k < Solver::kHorizon; ++k) {
    for (int i = 0; i < Solver::kNumStates; ++i) {
      work.v[i][k] = work.vnew[i][k];
    }
  }

  for (int k = 0; k < Solver::kHorizon - 1; ++k) {
    for (int i = 0; i < Solver::kNumInputs; ++i) {
      work.z[i][k] = work.znew[i][k];
    }
  }
}

bool HasFiniteInput(const Solver::Input &input) {
  for (int i = 0; i < Solver::kNumStates; ++i) {
    if (!std::isfinite(input.x0[i])) {
      return false;
    }
  }
  for (int i = 0; i < Solver::kNumInputs; ++i) {
    if (!std::isfinite(input.u_min[i]) || !std::isfinite(input.u_max[i])) {
      return false;
    }
  }
  return true;
}

}  // namespace

namespace chassis {

void RollLegMpcStaticSolver::Reset() { ZeroWorkspace(workspace_); }

RollLegMpcStaticSolver::Output RollLegMpcStaticSolver::Solve(const ModelData &model, const Input &input) {
  return Solve(model, input, Settings{});
}

RollLegMpcStaticSolver::Output RollLegMpcStaticSolver::Solve(const ModelData &model, const Input &input,
                                                             const Settings &settings) {
  Output output{};
  if (!HasFiniteInput(input)) {
    return output;
  }

  for (int i = 0; i < kNumStates; ++i) {
    workspace_.x[i][0] = input.x0[i];
  }

  const int max_iter = std::max(settings.max_iter, 0);
  for (int iter = 0; iter < max_iter; ++iter) {
    UpdateLinearCost(workspace_);
    BackwardPassGrad(model, workspace_);
    ForwardPass(model, workspace_);
    UpdateSlack(workspace_, input);
    UpdateDual(workspace_);

    output = ComputeResiduals(workspace_);
    output.iterations = iter + 1;
    output.converged = output.primal_residual_state < settings.abs_pri_state_tol &&
                       output.primal_residual_input < settings.abs_pri_input_tol_n &&
                       output.dual_residual_state < settings.abs_dua_state_tol &&
                       output.dual_residual_input < settings.abs_dua_input_tol_n;
    if (output.converged) {
      break;
    }
    SavePreviousSlack(workspace_);
  }

  if (max_iter == 0) {
    output = ComputeResiduals(workspace_);
  }

  for (int i = 0; i < kNumInputs; ++i) {
    output.u0[i] = workspace_.znew[i][0];
  }
  output.usable = output.converged || settings.always_return_last_iterate;
  return output;
}

}  // namespace chassis
