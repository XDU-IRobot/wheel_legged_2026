#include "include/chassis/chassis.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr rm::f32 kDefaultDtS = 0.002f;
constexpr rm::f32 kLegLengthKp = 3200.0f;
constexpr rm::f32 kLegLengthKd = 90.0f;
constexpr rm::f32 kRollCouplingKp = 120.0f;
constexpr rm::f32 kMaxLegForceN = 450.0f;
constexpr rm::f32 kMaxWheelTauNm = 8.0f;

rm::f32 ClampAbs(const rm::f32 value, const rm::f32 abs_limit) {
  const rm::f32 lim = std::fabs(abs_limit);
  return rm::modules::Clamp(value, -lim, lim);
}

bool IsSafeStopMode(const chassis::Fsm::State mode) {
  return mode == chassis::Fsm::State::kDisabled || mode == chassis::Fsm::State::kStandby;
}

}  // namespace

void chassis::Chassis::Init() {
  ChassisStateEstimatorConfig cfg{};
  state_estimator_.Init(cfg);
  SafeStop();
}

void chassis::Chassis::SafeStop() {
  output_.lf_tau = 0.0f;
  output_.lb_tau = 0.0f;
  output_.rf_tau = 0.0f;
  output_.rb_tau = 0.0f;
  output_.lw_tau = 0.0f;
  output_.rw_tau = 0.0f;
  output_.left_support_force_n = 0.0f;
  output_.right_support_force_n = 0.0f;
}

void chassis::Chassis::Update(const UpdateInput &input) {
  ChassisStateEstimatorInput estimator_input = input.estimator_input;
  estimator_input.dt_s = (estimator_input.dt_s > 0.0f) ? estimator_input.dt_s : kDefaultDtS;
  estimator_input.use_wheel_speed_direct = input.spin_enable;

  state_estimator_.Update(estimator_input);
  const ChassisStateEstimatorOutput &state_output = state_estimator_.GetOutput();

  const CalibratedLegKinematicsInput &leg_input = state_output.calibrated_leg_input;
  left_leg_.SetPhi1(leg_input.left.phi1_rad);
  left_leg_.SetPhi4(leg_input.left.phi4_rad);
  left_leg_.SetWPhi1(leg_input.left.w_phi1_rad_s);
  left_leg_.SetWPhi4(leg_input.left.w_phi4_rad_s);
  left_leg_.Update(estimator_input.dt_s);

  right_leg_.SetPhi1(leg_input.right.phi1_rad);
  right_leg_.SetPhi4(leg_input.right.phi4_rad);
  right_leg_.SetWPhi1(leg_input.right.w_phi1_rad_s);
  right_leg_.SetWPhi4(leg_input.right.w_phi4_rad_s);
  right_leg_.Update(estimator_input.dt_s);

  output_.current_state = state_output.current;
  output_.mean_leg_length_m = 0.5f * (state_output.left_leg_length_m + state_output.right_leg_length_m);
  output_.speed_mps = state_output.fused_speed_mps;

  if (!input.enable_output || !input.run_chassis_update || IsSafeStopMode(input.fsm_mode)) {
    SafeStop();
    return;
  }

  ComputeActuatorTorque(input, state_output);
}

void chassis::Chassis::ComputeActuatorTorque(const UpdateInput &input,
                                             const ChassisStateEstimatorOutput &state_output) {
  const wbr::CurrentState current = state_output.current;
  base_torque_ = lqr_controller_.ComputeControl(current, input.expected);

  const rm::f32 leg_length_avg = 0.5f * (state_output.left_leg_length_m + state_output.right_leg_length_m);
  const rm::f32 leg_length_dot_avg = 0.5f * (left_leg_.l0_dot() + right_leg_.l0_dot());

  const rm::f32 base_leg_force = rm::modules::Clamp(
      kLegLengthKp * (input.target_leg_length_m - leg_length_avg) - kLegLengthKd * leg_length_dot_avg, -kMaxLegForceN,
      kMaxLegForceN);
  const rm::f32 roll_coupling = rm::modules::Clamp(-kRollCouplingKp * current.theta_b, -120.0f, 120.0f);

  const rm::f32 left_force = base_leg_force + roll_coupling;
  const rm::f32 right_force = base_leg_force - roll_coupling;

  output_.left_support_force_n = left_force;
  output_.right_support_force_n = right_force;

  const rm::f32 t_bl_cmd = -base_torque_.t_bl;
  const rm::f32 t_br_cmd = -base_torque_.t_br;

  output_.lb_tau = left_leg_.jacobi_00() * left_force + left_leg_.jacobi_01() * t_bl_cmd;
  output_.lf_tau = left_leg_.jacobi_10() * left_force + left_leg_.jacobi_11() * t_bl_cmd;
  output_.rb_tau = right_leg_.jacobi_00() * right_force + right_leg_.jacobi_01() * t_br_cmd;
  output_.rf_tau = right_leg_.jacobi_10() * right_force + right_leg_.jacobi_11() * t_br_cmd;

  output_.lf_tau = -output_.lf_tau;
  output_.lb_tau = -output_.lb_tau;

  output_.lw_tau = ClampAbs(-base_torque_.t_wl, kMaxWheelTauNm);
  output_.rw_tau = ClampAbs(base_torque_.t_wr, kMaxWheelTauNm);

  if (input.fsm_mode == Fsm::State::kJumpRecover) {
    output_.lw_tau = 0.0f;
    output_.rw_tau = 0.0f;
  }
}
