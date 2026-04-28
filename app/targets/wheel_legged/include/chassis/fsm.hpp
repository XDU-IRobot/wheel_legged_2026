#pragma once

#include "librm/core/typedefs.hpp"

#include <cstdint>

/**
 * @file  chassis/fsm.hpp
 * @brief 底盘模式状态机接口（纯输入 -> 纯输出）
 */

namespace chassis {

using namespace rm;

class Fsm {
 public:
  /**
   * @brief 目标腿长档位
   */
  enum class LegLengthMode {
    kLow,
    kMid,
    kHigh,
  };

  /**
   * @brief 底盘状态模式
   */
  enum class State {
    kDisabled,
    kStandby,
    kBalance,
    kSpin,
    kJumpPrep,
    kJumpPush,
    kJumpRecover,
    kRecoveryFallCheck,
    kRecoverySelfRight,
  };

  /**
   * @brief 状态机输入
   * @note  上层需将遥控器、故障检测等原始量转换成该输入
   */
  struct Input {
    bool input_valid{false};                            ///< 输入是否有效
    bool force_enable{false};                           ///< 是否使能力控
    LegLengthMode leg_length_mode{LegLengthMode::kLow}; ///< 目标腿长档位
    bool spin_enable{false};                            ///< 小陀螺开关
    bool jump_trigger{false};                           ///< 跳跃触发沿
    bool fall_detected{false};                          ///< 是否检测到跌倒
    bool upright_stable{true};                          ///< 是否恢复稳定直立
    float current_leg_length_m{0.0f};                   ///< 当前腿长
    uint32_t tick_ms{0};                                ///< 系统时基（毫秒）
  };

  /**
   * @brief 状态机输出
   */
  struct Output {
    /**
     * @brief 控制动作输出
     */
    struct ControlOutput {
      bool enable_dm{false};            ///< DM 电机使能
      bool run_chassis_update{false};   ///< 是否执行底盘控制更新
      bool spin_enable{false};          ///< 速度估计是否使用轮速直通
      bool recovery_enable{false};      ///< 是否使能恢复逻辑
      bool safe_output_required{true};  ///< 是否要求安全输出（全零）
      float target_leg_length_m{0.18f}; ///< 目标腿长
      uint8_t jump_phase{0};            ///< 跳跃阶段编号，0 表示非跳跃
    };

    State mode{State::kDisabled}; ///< 当前模式
    bool state_changed{false};    ///< 本周期是否发生状态变化
    ControlOutput control{};      ///< 控制动作
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
  struct EtlImpl;
  State mode_{State::kDisabled};
  LegLengthMode leg_length_mode_{LegLengthMode::kLow};
  EtlImpl *etl_impl_{nullptr};
  Output output_{};
};

}  // namespace chassis

using Fsm = chassis::Fsm;
