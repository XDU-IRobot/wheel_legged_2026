#pragma once

#include <array>
#include <cstdint>

#include <librm.hpp>
#include "wheel_legged_params.hpp"

/**
 * @file  targets/wheel_legged/include/gimbal_to_chassis_rx_bridge.hpp
 * @brief 接收云台 GimbalToChassisTxBridge 转发的惯导 + VT03 键鼠 CAN 数据
 */

/**
 * @brief 云台→底盘 CAN 接收桥
 * @note  与云台端 GimbalToChassisTxBridge 协议一致：
 *        - 0x110: [0-1]pitch_millirad [2-3]yaw_millirad [4-5]mouse_x [6-7]mouse_y
 *        - 0x111: [0-1]mouse_z [2]left_button [3]right_button [4-5]keyboard_value [6-7]reserved
 */
class GimbalToChassisRxBridge final : public rm::device::CanDevice {
 public:
  static constexpr rm::u16 kRxStdIdA = wheel_legged::params::active::remote_control_can_bridge::kRxStdIdA;
  static constexpr rm::u16 kRxStdIdB = wheel_legged::params::active::remote_control_can_bridge::kRxStdIdB;
  static constexpr rm::usize kPayloadSizeA = wheel_legged::params::active::remote_control_can_bridge::kPayloadSizeA;
  static constexpr rm::usize kPayloadSizeB = wheel_legged::params::active::remote_control_can_bridge::kPayloadSizeB;

  explicit GimbalToChassisRxBridge(rm::hal::CanInterface &can) : CanDevice(can, kRxStdIdA, kRxStdIdB) {}

  void RxCallback(const rm::hal::CanFrame *msg) override {
    if (msg == nullptr) {
      ReportStatus(kFault);
      return;
    }
    if (msg->rx_std_id == kRxStdIdA && msg->dlc >= kPayloadSizeA) {
      pitch_rad_ = MilliI16ToRad(UnpackI16(&msg->data[0]));
      yaw_rad_ = MilliI16ToRad(UnpackI16(&msg->data[2]));
      mouse_x_ = UnpackI16(&msg->data[4]);
      mouse_y_ = UnpackI16(&msg->data[6]);
      frame_count_++;
      ReportStatus(kOk);
    } else if (msg->rx_std_id == kRxStdIdB && msg->dlc >= kPayloadSizeB) {
      mouse_z_ = UnpackI16(&msg->data[0]);
      left_button_ = (msg->data[2] != 0);
      right_button_ = (msg->data[3] != 0);
      keyboard_value_ = UnpackU16(&msg->data[4]);
      frame_count_++;
      kbd_frame_count_++;
      ReportStatus(kOk);
    } else {
      ReportStatus(kFault);
    }
  }

  [[nodiscard]] rm::f32 pitch_rad() const { return pitch_rad_; }
  [[nodiscard]] rm::f32 yaw_rad() const { return yaw_rad_; }
  [[nodiscard]] rm::i16 mouse_x() const { return mouse_x_; }
  [[nodiscard]] rm::i16 mouse_y() const { return mouse_y_; }
  [[nodiscard]] rm::i16 mouse_z() const { return mouse_z_; }
  [[nodiscard]] bool left_button() const { return left_button_; }
  [[nodiscard]] bool right_button() const { return right_button_; }
  [[nodiscard]] rm::u16 keyboard_value() const { return keyboard_value_; }
  [[nodiscard]] rm::u32 frame_count() const { return frame_count_; }
  [[nodiscard]] rm::u32 keyboard_frame_count() const { return kbd_frame_count_; }

 private:
  static rm::i16 UnpackI16(const rm::u8 *in) {
    const rm::u16 raw = static_cast<rm::u16>((static_cast<rm::u16>(in[0]) << 8) | static_cast<rm::u16>(in[1]));
    return static_cast<rm::i16>(raw);
  }

  static rm::u16 UnpackU16(const rm::u8 *in) {
    return static_cast<rm::u16>((static_cast<rm::u16>(in[0]) << 8) | static_cast<rm::u16>(in[1]));
  }

  static rm::f32 MilliI16ToRad(rm::i16 value) { return static_cast<rm::f32>(value) * 0.001f; }

  rm::f32 pitch_rad_{0.0f};
  rm::f32 yaw_rad_{0.0f};
  rm::i16 mouse_x_{0};
  rm::i16 mouse_y_{0};
  rm::i16 mouse_z_{0};
  bool left_button_{false};
  bool right_button_{false};
  rm::u16 keyboard_value_{0};
  rm::u32 frame_count_{0};
  rm::u32 kbd_frame_count_{0};
};
