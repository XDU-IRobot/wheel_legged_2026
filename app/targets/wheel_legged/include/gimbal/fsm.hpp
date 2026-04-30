#pragma once

#include "../fsm_common.hpp"

#include <cstdint>

namespace gimbal {

class Fsm {
 public:
  enum class State : uint8_t {
    kDisabled = 0,
    kServiceWithFire,
    kServiceSafe,
    kCombat,
    kRecoveryAlign,
    kStartupAlign,
  };

  struct Input {
    wheel_legged::ModeRequest request{};
    bool chassis_recovery_active{false};
    bool startup_align_complete{false};
  };

  struct Output {
    struct ControlOutput {
      bool gimbal_enable{false};
      bool align_to_chassis_forward{false};
      bool fire_allowed{false};
      bool shoot_request{false};
      wheel_legged::TargetSource active_target_source{wheel_legged::TargetSource::kRc};
      wheel_legged::GimbalTarget gimbal_target{};
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
