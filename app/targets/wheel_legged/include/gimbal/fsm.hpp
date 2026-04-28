#pragma once

#include "../fsm_common.hpp"

#include <cstdint>

/**
 * @file  targets/wheel_legged/include/gimbal/fsm.hpp
 * @brief 云台模式状态机接口
 */

namespace gimbal {

class Fsm {
 public:
  /**
   * @brief 云台状态
   */
  enum class State : uint8_t {
    kDisabled = 0,
    kServiceWithFire,
    kServiceSafe,
    kCombat,
    kRecoveryAlign,
  };

  /**
   * @brief 状态机输入
   */
  struct Input {
    wheel_legged::ModeRequest request{};
    bool chassis_recovery_active{false};  ///< 底盘恢复状态标志
  };

  /**
   * @brief 状态机输出
   */
  struct Output {
    struct ControlOutput {
      bool gimbal_enable{false};                                                         ///< 云台使能
      bool align_to_chassis_forward{false};                                              ///< 是否对齐底盘前向
      bool fire_allowed{false};                                                          ///< 是否允许开火
      bool shoot_request{false};                                                         ///< 本周期开火请求
      wheel_legged::TargetSource active_target_source{wheel_legged::TargetSource::kRc};  ///< 目标来源
      wheel_legged::GimbalTarget gimbal_target{};                                        ///< 本周期目标
    };

    State mode{State::kDisabled};  ///< 当前状态
    bool state_changed{false};     ///< 状态是否变化
    ControlOutput control{};       ///< 控制动作
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
