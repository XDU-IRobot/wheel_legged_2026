#pragma once
#include "librm/core/typedefs.hpp"
#include <cstdint>

/**
 * @file  fsm/fsm.hpp
 * @brief 底盘模式状态机接口（纯输入 -> 纯输出）
 */

using namespace rm;

class Fsm {
public:
    enum class LegLengthMode {
        kLow,
        kMid,
        kHigh,
    };

    /**
     * @brief 状态机模式
     */
    enum class State {
        kNoForce,  // 无力模式
        kNormalLowLeg,
        kNormalMidLeg,
        kNormalHighLeg,
        kSmallGyro,
        kJumpRetract1,
        kJumpExtend,
        kJumpRetract2,
    };

    /**
     * @brief 状态机输入
     * @note  由上层把传感器/遥控状态转换为业务输入
     */
    struct Input {
        bool input_valid{false};
        bool force_enable{false};
        LegLengthMode leg_length_mode{LegLengthMode::kLow};
        bool small_gyro_enable{false};
        bool jump_trigger{false};
        float current_leg_length_m{0.0f};
        uint32_t tick_ms{0};
    };

    /**
     * @brief 状态机输出
     */
    struct Output {
        /**
         * @brief 控制动作输出
         * @note  供任务层统一执行，避免在任务层写状态分支
         */
        struct ControlOutput {
            bool enable_dm{false};
            bool run_chassis_update{false};
            bool small_gyro_enable{false};
            float target_leg_length_m{0.18f};
            uint8_t jump_phase{0};
        };

        State mode{State::kNoForce};
        bool state_changed{false};
        ControlOutput control{};
    };

    ~Fsm();

    /**
     * @brief 初始化状态机
     */
    void Init();

    /**
     * @brief 执行状态迁移并更新输出
     */
    void Transit(State new_mode);

    /**
     * @brief 单步更新
     * @param input 状态机输入
     * @return 当前输出
     */
    Output Update(const Input &input);

    [[nodiscard]] const Output &output() const { return output_; }
    [[nodiscard]] State mode() const { return mode_; }

    State mode_{State::kNoForce};

private:
    struct EtlImpl;
    EtlImpl* etl_impl_{nullptr};
    Output output_{};
};