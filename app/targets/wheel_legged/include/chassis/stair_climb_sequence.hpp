#pragma once

#include "../fsm_common.hpp"

#include <cstdint>

namespace chassis {

class StairClimbSequence {
 public:
  struct Input {
    bool start{false};
    bool cancel{false};
    bool use_step2_params{false};
    bool output_enabled{false};
    bool posture_valid{true};
    float mean_leg_length_m{0.0f};
    float theta_ll_rad{0.0f};
    float theta_lr_rad{0.0f};
    float theta_ll_dot_rad_s{0.0f};
    float theta_lr_dot_rad_s{0.0f};
    float theta_b_rad{0.0f};
    float theta_b_dot_rad_s{0.0f};
    float roll_rad{0.0f};
    uint32_t tick_ms{0U};
  };

  struct Output {
    wheel_legged::StairPhase phase{wheel_legged::StairPhase::kIdle};
    wheel_legged::StairAbortReason abort_reason{wheel_legged::StairAbortReason::kNone};
    wheel_legged::ChassisMotionTarget target{};
    bool controls_motion{false};
    bool running{false};
    bool succeeded{false};
    bool aborted{false};
    bool trigger_standup{false};
    uint32_t phase_elapsed_ms{0U};
    uint32_t stable_elapsed_ms{0U};
    float theta_ll_error_rad{0.0f};
    float theta_lr_error_rad{0.0f};
    float leg_length_error_m{0.0f};
  };

  void Reset();
  const Output &Update(const Input &input);
  [[nodiscard]] const Output &output() const { return output_; }

 private:
  void EnterPhase(wheel_legged::StairPhase phase, uint32_t tick_ms);
  void Abort(wheel_legged::StairAbortReason reason, uint32_t tick_ms);
  bool StableFor(bool condition, uint32_t duration_ms, uint32_t tick_ms);
  void UpdateOutput(const Input &input);

  bool use_step2_params_{false};
  wheel_legged::StairPhase phase_{wheel_legged::StairPhase::kIdle};
  wheel_legged::StairAbortReason abort_reason_{wheel_legged::StairAbortReason::kNone};
  uint32_t phase_enter_tick_ms_{0U};
  uint32_t stable_start_tick_ms_{0U};
  bool stable_active_{false};
  float settle_theta_ramp_target_{0.0f};
  Output output_{};
};

}  // namespace chassis
