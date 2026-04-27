#pragma once

/**
 * @file  wl_infantry/fsm/fsm.hpp
 * @brief 整车统一状态机接口
 */

#include <cstdint>
#include <memory>

/**
 * @brief 整车统一状态机
 * @note  负责解析遥控输入、倒地/跳跃时序以及底盘/云台使能约束。
 */
class Fsm {
 public:
  struct EtlImpl;

  /** @brief 整车工作域请求。 */
  enum class DomainRequest : uint8_t {
    kDisabled = 0,
    kService,
    kCombat,
  };

  /** @brief 服务态固定配置。 */
  enum class ServiceProfile : uint8_t {
    kGimbalOnlyWithFire = 0,
    kChassisAndGimbalWithFire,
    kChassisAndGimbalSafe,
  };

  /** @brief 腿长档位。 */
  enum class LegProfile : uint8_t {
    kLow = 0,
    kMid,
    kHigh,
  };

  /** @brief 云台目标源枚举。 */
  enum class TargetSource {
    kRc,
    kHost,
  };

  /**
   * @brief 云台目标角
   */
  struct GimbalTarget {
    float yaw_rad{0.0f};
    float pitch_rad{0.0f};
  };

  /**
   * @brief 整车主状态枚举
   */
  enum class State {
    kDisabled,
    kServiceGimbalOnlyWithFire,
    kServiceWithFireLowLeg,
    kServiceWithFireMidLeg,
    kServiceWithFireHighLeg,
    kServiceWithFireSpin,
    kServiceWithFireJumpPrep,
    kServiceWithFireJumpPush,
    kServiceWithFireJumpRecover,
    kServiceSafeLowLeg,
    kServiceSafeMidLeg,
    kServiceSafeHighLeg,
    kServiceSafeSpin,
    kServiceSafeJumpPrep,
    kServiceSafeJumpPush,
    kServiceSafeJumpRecover,
    kCombatLowLeg,
    kCombatMidLeg,
    kCombatHighLeg,
    kCombatSpin,
    kCombatJumpPrep,
    kCombatJumpPush,
    kCombatJumpRecover,
    kRecoveryFallCheck,
    kRecoverySelfRight,
  };

  /**
   * @brief FSM 输入
   */
  struct Input {
    bool input_valid{false};
    DomainRequest domain_request{DomainRequest::kDisabled};
    ServiceProfile service_profile{ServiceProfile::kGimbalOnlyWithFire};
    LegProfile leg_request{LegProfile::kLow};

    bool spin_hold{false};
    bool jump_trigger{false};
    bool fire_request{false};
    float current_leg_length_m{0.0f};

    TargetSource target_source{TargetSource::kRc};
    GimbalTarget rc_target{};
    GimbalTarget host_target{};
    bool host_target_valid{false};

    bool fall_detected{false};
    uint32_t fall_detected_hold_ms{0};
    bool upright_stable{false};

    uint32_t tick_ms{0};
  };

  /**
   * @brief FSM 输出
   */
  struct Output {
    /** @brief 底盘控制位。 */
    struct ChassisControl {
      bool chassis_enable{false};
      bool spin_mode{false};
      bool recovery_mode{false};
      LegProfile leg_profile{LegProfile::kLow};
      float target_leg_length_m{0.18f};
    };

    /** @brief 云台控制位。 */
    struct GimbalControl {
      bool gimbal_enable{false};
      /** @brief 是否允许火控子状态机进入武器工作态。 */
      bool fire_allowed{false};
      /** @brief 当前拍是否请求触发一次射击动作。 */
      bool shoot_request{false};
      /** @brief 恢复态时要求云台优先与底盘正方向对齐。 */
      bool align_to_chassis_forward{false};
      TargetSource active_target_source{TargetSource::kRc};
      GimbalTarget gimbal_target{};
    };

    State state{State::kDisabled};
    bool state_changed{false};
    ChassisControl chassis{};
    GimbalControl gimbal{};
  };

  Fsm();
  ~Fsm();

  /** @brief 初始化状态机内部状态。 */
  void Init();
  /**
   * @brief 更新状态机
   * @param input 当前拍输入
   * @return 当前拍状态与控制输出
   */
  Output Update(const Input& input);

 private:
  void Transit(State next_state, uint32_t tick_ms);
  [[nodiscard]] Output BuildOutput(const Input& input) const;

  State state_{State::kDisabled};
  Output output_{};
  uint32_t state_enter_tick_ms_{0};
  State jump_exit_low_state_{State::kDisabled};
  std::unique_ptr<EtlImpl> etl_impl_{};
};
