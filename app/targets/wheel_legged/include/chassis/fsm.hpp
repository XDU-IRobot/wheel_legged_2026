#pragma once

#include "../fsm_common.hpp"
#include "../wheel_legged_params.hpp"

#include <cstdint>

/**
 * @file  chassis/fsm.hpp
 * @brief 底盘模式状态机接口（纯输入 -> 纯输出）
 */

namespace chassis {

class Fsm {
 public:
  /**
   * @brief 底盘状态模式
   */
  enum class State : uint8_t {
    kDisabled = 0,
    kLowLeg,
    kMidLeg,
    kHighLeg,
    kSpin,
    kJumpPrep,
    kJumpPush,
    kJumpRecover,
    kRecoveryFallCheck,
    kRecoverySelfRight,
  };

  /**
   * @brief 状态机输入
   * @note  由 control_loop 生成统一语义请求并传入。
   */
  struct Input {
    wheel_legged::ModeRequest request{};
  };

  /**
   * @brief 状态机输出
   */
  struct Output {
    /**
     * @brief 控制动作输出
     */
    struct ControlOutput {
      bool enable_dm{false};                                                 ///< DM 电机使能
      bool run_chassis_update{false};                                        ///< 是否执行底盘控制更新
      bool spin_enable{false};                                               ///< 速度估计是否使用轮速直通
      bool recovery_enable{false};                                           ///< 是否使能恢复逻辑
      bool safe_output_required{true};                                       ///< 是否要求安全输出（全零）
      wheel_legged::LegProfile leg_profile{wheel_legged::LegProfile::kLow};  ///< 当前腿长语义档位
      float target_leg_length_m{wheel_legged::params::active::chassis_fsm::kLowLegLengthM};  ///< 目标腿长
      uint8_t jump_phase{0};  ///< 跳跃阶段编号，0 表示非跳跃
    };

    State mode{State::kDisabled};  ///< 当前模式
    bool state_changed{false};     ///< 本周期是否发生状态变化
    ControlOutput control{};       ///< 控制动作
  };

  ~Fsm();

  /** @brief 初始化状态机 */
  void Init();

  /** @brief 状态迁移并刷新输出 */
  void Transit(State new_mode);

  /**
   * @brief 单步更新
   * @param input 状态机输入
   * @return 当前输出
   */
  Output Update(const Input &input);

  [[nodiscard]] const Output &output() const { return output_; }
  [[nodiscard]] State mode() const { return mode_; }

 private:
  State mode_{State::kDisabled};
  wheel_legged::LegProfile requested_leg_profile_{wheel_legged::LegProfile::kLow};
  uint32_t state_enter_tick_ms_{0};
  Output output_{};
};

}  // namespace chassis

using Fsm = chassis::Fsm;
