#pragma once

#include "../fsm_common.hpp"

#include <cstdint>

/**
 * @file  targets/wheel_legged/include/gimbal/fsm.hpp
 * @brief 云台模式状态机接口
 */

namespace gimbal {

/**
 * @brief 云台状态机
 */
class Fsm {
 public:
  /**
   * @brief 云台工作模式
   */
  enum class State : uint8_t {
    kDisabled = 0,          ///< 关闭云台输出
    kServiceWithFire,       ///< 维护模式，允许发射链路
    kServiceSafe,           ///< 维护模式，禁止发射链路
    kCombat,                ///< 战斗模式
    kRecoveryAlign,         ///< 底盘恢复时云台对齐车体前方
    kRecoveryYawCentering,  ///< 恢复前偏航归中 + pitch 到上限
    kStartupAlign,          ///< 上电/重新使能后的偏航归中
    kIdent,                 ///< 辨识模式：五次谐波轨迹 + 单位置环 PID + 串口采集
    kFfVerify,              ///< 前馈验证模式：纯动力学前馈跟随五次谐波轨迹
  };

  /**
   * @brief 状态机输入
   */
  struct Input {
    wheel_legged::GimbalFsmInput request{};
  };

  /**
   * @brief 状态机输出
   */
  struct Output {
    /**
     * @brief 云台控制动作
     */
    struct ControlOutput {
      bool gimbal_enable{false};             ///< 是否使能云台电机
      bool align_to_chassis_forward{false};  ///< 是否忽略目标并对齐车体前方
      wheel_legged::TargetSource active_target_source{wheel_legged::TargetSource::kRc};  ///< 实际采用的目标来源
      wheel_legged::GimbalTarget gimbal_target{};  ///< 实际下发给云台控制器的目标
      wheel_legged::GimbalTestProfile gimbal_test_profile{wheel_legged::GimbalTestProfile::kNormal};  ///< 测试子模式
    };

    State mode{State::kDisabled};  ///< 当前模式
    bool state_changed{false};     ///< 本周期是否发生状态变化
    ControlOutput control{};       ///< 控制动作
  };

  /** @brief 初始化状态机 */
  void Init();

  /** @brief 状态迁移 */
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
  Output output_{};
};

}  // namespace gimbal
