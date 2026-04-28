#pragma once

#include "chassis_state.hpp"
#include "fsm.hpp"
#include "lqr_controllers.hpp"

namespace chassis {

class Chassis {
 public:
  struct UpdateInput {
    ChassisStateEstimatorInput estimator_input{};
    wbr::ExpectedState expected{};
    Fsm::State fsm_mode{Fsm::State::kDisabled};
    bool enable_output{false};
    bool run_chassis_update{false};
    bool spin_enable{false};
    rm::f32 target_leg_length_m{0.18f};
  };

  struct UpdateOutput {
    rm::f32 lf_tau{0.0f};
    rm::f32 lb_tau{0.0f};
    rm::f32 rf_tau{0.0f};
    rm::f32 rb_tau{0.0f};
    rm::f32 lw_tau{0.0f};
    rm::f32 rw_tau{0.0f};

    rm::f32 left_support_force_n{0.0f};
    rm::f32 right_support_force_n{0.0f};
    rm::f32 mean_leg_length_m{0.0f};
    rm::f32 speed_mps{0.0f};
    rm::f32 wheel_speed_mps{0.0f};
    rm::f32 raw_wheel_speed_mps{0.0f};
    rm::f32 raw_accel_speed_mps{0.0f};
    rm::f32 current_speed_mps{0.0f};

    wbr::CurrentState current_state{};
  };

  void Init();
  void Update(const UpdateInput &input);
  void SafeStop();

  [[nodiscard]] const UpdateOutput &GetOutput() const { return output_; }

 private:
  void ComputeActuatorTorque(const UpdateInput &input, const ChassisStateEstimatorOutput &state_output);
  void CalSupportForce();

  struct TunableParams {
    rm::f32 leg_target_length_m{0.18f};
  };

  ChassisStateEstimator state_estimator_{};
  wbr::WbrController lqr_controller_{};
  wbr::LegKinematics left_leg_{0.215f, 0.254f};
  wbr::LegKinematics right_leg_{0.215f, 0.254f};
  wbr::MotorTorque base_torque_{};
  TunableParams params_{};

  rm::f32 left_force_{0.0f};
  rm::f32 right_force_{0.0f};
  rm::f32 l_spring_torque_{0.0f};
  rm::f32 r_spring_torque_{0.0f};
  rm::f32 imu_roll_{0.0f};

  rm::f32 lf_real_torque_{0.0f};
  rm::f32 lb_real_torque_{0.0f};
  rm::f32 rf_real_torque_{0.0f};
  rm::f32 rb_real_torque_{0.0f};

  rm::f32 left_support_force_est_n_{0.0f};
  rm::f32 right_support_force_est_n_{0.0f};
  rm::f32 left_l0_dot_prev_{0.0f};
  rm::f32 right_l0_dot_prev_{0.0f};

  rm::modules::PID left_l0_pid_{};
  rm::modules::PID right_l0_pid_{};
  rm::modules::PID left_l0_pid_jump_two_{};
  rm::modules::PID right_l0_pid_jump_two_{};
  rm::modules::PID left_l0_pid_jump_three_{};
  rm::modules::PID right_l0_pid_jump_three_{};
  rm::modules::PID roll_pid_{};
  rm::modules::PID left_leg_turn_pid_{};
  rm::modules::PID right_leg_turn_pid_{};

  UpdateOutput output_{};
};

}  // namespace chassis
