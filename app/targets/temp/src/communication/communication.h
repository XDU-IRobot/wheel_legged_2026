#ifndef SRC_COMMUNICATION_COMMUNICATION_H
#define SRC_COMMUNICATION_COMMUNICATION_H

#include <array>

#include "librm.hpp"

/**
 * @file  communication/communication.h
 * @brief 底盘侧 CAN 通信协议封装
 * @note  保持与上位机/云台的既有协议格式一致：0x59, 7 字节有效载荷。
 */

namespace communication {

/**
 * @brief 底盘模式
 */
enum class ChassisState : rm::u8 {
  kUnable = 0x00,
  kFollow = 0x01,
  kRotate = 0x02,
  kJump = 0x03,
};

/**
 * @brief 腿长档位
 */
enum class LegLength : rm::u8 {
  kLow = 0x00,
  kNormal = 0x01,
  kHigh = 0x02,
};

/**
 * @brief 底盘控制命令
 */
struct ChassisCommand {
  ChassisState state{ChassisState::kUnable};
  LegLength leg_length{LegLength::kNormal};
  rm::i8 move_x{0};
  rm::i8 move_y{0};
};

/**
 * @brief UI 命令
 */
struct UICommand {
  rm::u8 ui1{0};
  rm::u8 ui2{0};
  rm::u8 ui3{0};
  rm::u8 ui4{0};
};

/**
 * @brief 完整通信命令
 */
struct Command {
  ChassisCommand chassis{};
  UICommand ui{};
};

/**
 * @brief 底盘侧 CAN 接收器
 * @note  默认订阅 ID 0x59，与云台发送端保持一致。
 */
class ChassisReceive : public rm::device::CanDevice {
 public:
  explicit ChassisReceive(rm::hal::CanInterface &can, rm::u32 rx_std_id = 0x59U);
  ChassisReceive() = delete;
  ~ChassisReceive() override = default;

  void RxCallback(const rm::hal::CanFrame *msg) override;

  [[nodiscard]] Command getCommand() const;

 protected:
  Command command_{};
};

/**
 * @brief 通信适配层（预留发送到云台的扩展接口）
 */
class Communication final : public ChassisReceive {
 public:
  explicit Communication(rm::hal::CanInterface &can, rm::u32 rx_std_id = 0x59U) : ChassisReceive(can, rx_std_id) {}

  void SendCommandToGimbal();

 private:
  std::array<rm::u8, 8> tx_buf_gimbal_{};
};

}  // namespace communication

#endif  // SRC_COMMUNICATION_COMMUNICATION_H
