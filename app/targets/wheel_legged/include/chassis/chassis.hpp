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

    wbr::CurrentState current_state{};
  };

  void Init();
  void Update(const UpdateInput &input);
  void SafeStop();

  [[nodiscard]] const UpdateOutput &GetOutput() const { return output_; }

 private:
  void ComputeActuatorTorque(const UpdateInput &input,
                             const ChassisStateEstimatorOutput &state_output);

  ChassisStateEstimator state_estimator_{};
  wbr::WbrController lqr_controller_{};
  wbr::LegKinematics left_leg_{0.215f, 0.254f};
  wbr::LegKinematics right_leg_{0.215f, 0.254f};
  wbr::MotorTorque base_torque_{};
  UpdateOutput output_{};
};

}  // namespace chassis
