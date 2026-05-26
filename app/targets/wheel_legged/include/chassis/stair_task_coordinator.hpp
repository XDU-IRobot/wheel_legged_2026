#pragma once

#include "../fsm_common.hpp"

#include <cstdint>

namespace chassis {

class StairTaskCoordinator {
 public:
  struct Input {
    wheel_legged::StairTaskRequest request{wheel_legged::StairTaskRequest::kNone};
    bool contact_detected{false};
    bool high_leg_ready{false};
    bool posture_valid{true};
    bool output_enabled{false};
    bool sequence_succeeded{false};
    bool sequence_aborted{false};
    wheel_legged::StairAbortReason sequence_abort_reason{wheel_legged::StairAbortReason::kNone};
  };

  struct Output {
    wheel_legged::StairTaskMode mode{wheel_legged::StairTaskMode::kIdle};
    wheel_legged::StairAbortReason abort_reason{wheel_legged::StairAbortReason::kNone};
    uint8_t requested_attempts{0U};
    uint8_t completed_attempts{0U};
    bool task_active{false};
    bool request_high_leg{false};
    bool force_low_leg{false};
    bool start_sequence{false};
    bool cancel_sequence{false};
    bool reset_sequence{false};
    bool recovery_required{false};
  };

  void Reset();
  const Output &Update(const Input &input);
  [[nodiscard]] const Output &output() const { return output_; }

 private:
  void Arm(uint8_t attempts, bool continuous);
  void Abort(wheel_legged::StairAbortReason reason, bool recovery_required);
  void RefreshOutput();
  bool IsActive() const;

  wheel_legged::StairTaskMode mode_{wheel_legged::StairTaskMode::kIdle};
  wheel_legged::StairAbortReason abort_reason_{wheel_legged::StairAbortReason::kNone};
  uint8_t requested_attempts_{0U};
  uint8_t completed_attempts_{0U};
  bool continuous_{false};
  bool hold_low_until_release_{false};
  bool recovery_required_{false};
  bool contact_released_{true};
  Output output_{};
};

}  // namespace chassis
