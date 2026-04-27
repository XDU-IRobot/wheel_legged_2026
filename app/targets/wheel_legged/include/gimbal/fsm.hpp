#pragma once

#include <cstdint>

namespace gimbal {

class Fsm {
 public:
  enum class State {
    kDisabled,
    kSafe,
    kManualControl,
    kHostControl,
    kRecoveryAlign,
  };

  enum class TargetSource {
    kRemote,
    kHost,
  };

  struct Input {
    bool input_valid{false};
    bool enable_request{false};
    bool safe_request{false};
    bool host_target_valid{false};
    bool chassis_recovery_active{false};
    bool fire_request{false};
  };

  struct Output {
    struct ControlOutput {
      bool gimbal_enable{false};
      bool align_to_chassis_forward{false};
      bool fire_allowed{false};
      bool shoot_request{false};
      TargetSource target_source{TargetSource::kRemote};
    };

    State mode{State::kDisabled};
    bool state_changed{false};
    ControlOutput control{};
  };

  void Init();
  void Transit(State new_mode);
  Output Update(const Input &input);

  [[nodiscard]] const Output &output() const { return output_; }
  [[nodiscard]] State mode() const { return mode_; }

 private:
  State mode_{State::kDisabled};
  Output output_{};
};

}  // namespace gimbal

