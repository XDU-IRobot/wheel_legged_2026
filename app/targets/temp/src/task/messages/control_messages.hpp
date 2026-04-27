#pragma once

#include "../../fsm/fsm.hpp"
#include "../../chassis/state/chassis_state.hpp"
#include "wbr/wbr_controller.hpp"

/**
 * @file  task/messages/control_messages.hpp
 * @brief 任务间控制消息定义
 */

namespace tasking {

/**
 * @brief 消息通用头
 */
struct MsgHeader {
  // 消息生成时间戳（ms）。
  uint32_t tick_ms{0};
  // 同源消息序号。
  uint16_t seq{0};
  // 来源任务标识，便于后续扩展追踪。
  uint16_t source_id{0};
};

/**
 * @brief 通信输入消息（遥控器/后续云台）
 */
struct CommInputMsg {
  MsgHeader h{};
  bool link_ok{false};
  bool remote_mid_force{false};
  bool force_enable{false};
  Fsm::LegLengthMode leg_length_mode{Fsm::LegLengthMode::kLow};
  bool small_gyro_enable{false};
  bool jump_trigger{false};
  float vx_cmd{0.0f};
  float yaw_cmd{0.0f};
};

/**
 * @brief 状态机输出消息
 */
struct FsmOutputMsg {
  MsgHeader h{};
  Fsm::State mode{Fsm::State::kNoForce};
  bool state_changed{false};
  Fsm::Output::ControlOutput control{};
};

/**
 * @brief 状态估计消息（预留给通信回传或上层监控）
 */
struct StateEstimateMsg {
  MsgHeader h{};
  wbr::CurrentState current{};
  float wheel_speed_mps{0.0f};
  float fused_speed_mps{0.0f};
};

/**
 * @brief WBR 模型力矩输出消息
 */
struct WbrTorqueMsg {
  MsgHeader h{};
  wbr::MotorTorque base{};
};

/**
 * @brief 电机反馈消息
 * @note  由电机任务产生，供计算任务做状态估计。
 */
struct MotorFeedbackMsg {
  MsgHeader h{};
  LegJointFeedback left_leg{};
  LegJointFeedback right_leg{};
  WheelFeedback wheel{};
  bool valid{false};
};

/**
 * @brief 六电机目标力矩消息
 */
struct TorqueCmd6 {
  MsgHeader h{};
  // 执行控制位：由计算任务统一下发给电机任务。
  bool enable_dm{false};
  uint8_t jump_phase{0U};  // 跳跃阶段：1=下蹲, 2=起跳伸直, 3=空中缓冲/落地

  float lf_tau{0.0f};
  float lb_tau{0.0f};
  float rf_tau{0.0f};
  float rb_tau{0.0f};
  float lw_tau{0.0f};
  float rw_tau{0.0f};
  bool valid{false};
};

}  // namespace tasking
