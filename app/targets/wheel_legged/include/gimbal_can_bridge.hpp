#pragma once

#include <array>
#include <cstdint>

#include <librm.hpp>
#include "params.hpp"

/**
 * @file  targets/wheel_legged/include/gimbal_can_bridge.hpp
 * @brief 接收云台 GimbalToChassisTxBridge 转发的惯导 + VT03 键鼠 CAN 数据
 */

struct EmyRobotHP {
  rm::u16 hero_1_HP;
  rm::u16 engineer_2_HP;
  rm::u16 standard_3_HP;
  rm::u16 standard_4_HP;
  rm::u16 sentry_7_HP;
};

/**
 * @brief 云台→底盘 CAN 接收桥
 * @note  与云台端 GimbalToChassisTxBridge 协议一致：
 *        - 0x110: [0..1]vt03_online [2..3]gyro_z [4..5]gyro_x [6]mouse_left [7]mouse_right
 *        - 0x111: [0..1]mouse_x [2..3]mouse_y [4..5]mouse_z [6..7]keyboard_key
 *        - 0x112: [0-1]quat_w [2-3]quat_x [4-5]quat_y [6-7]quat_z  (int16, scale 32767)
 *        - 0x113: [0-1]hero_1_HP [2-3]engineer_2_HP [4-5]standard_3_HP [6-7]standard_4_HP
 *        - 0x114: [0-1]sentry_7_HP
 */
class GimbalToChassisRxBridge final : public rm::device::CanDevice {
 public:
  static constexpr rm::u16 kRxStdIdA = wheel_legged::params::active::remote_control_can_bridge::kRxStdIdA;
  static constexpr rm::u16 kRxStdIdB = wheel_legged::params::active::remote_control_can_bridge::kRxStdIdB;
  static constexpr rm::u16 kRxStdIdC = wheel_legged::params::active::remote_control_can_bridge::kRxStdIdC;
  static constexpr rm::u16 kRxStdIdD = wheel_legged::params::active::remote_control_can_bridge::kRxStdIdD;
  static constexpr rm::u16 kRxStdIdE = wheel_legged::params::active::remote_control_can_bridge::kRxStdIdE;
  static constexpr rm::usize kPayloadSizeA = wheel_legged::params::active::remote_control_can_bridge::kPayloadSizeA;
  static constexpr rm::usize kPayloadSizeB = wheel_legged::params::active::remote_control_can_bridge::kPayloadSizeB;
  static constexpr rm::usize kPayloadSizeC = wheel_legged::params::active::remote_control_can_bridge::kPayloadSizeC;
  static constexpr rm::usize kPayloadSizeD = wheel_legged::params::active::remote_control_can_bridge::kPayloadSizeD;
  static constexpr rm::usize kPayloadSizeE = wheel_legged::params::active::remote_control_can_bridge::kPayloadSizeE;

  explicit GimbalToChassisRxBridge(rm::hal::CanInterface &can)
      : CanDevice(can, kRxStdIdA, kRxStdIdB, kRxStdIdC, kRxStdIdD, kRxStdIdE) {}

  void RxCallback(const rm::hal::CanFrame *msg) override {
    if (msg == nullptr) {
      ReportStatus(kFault);
      return;
    }
    if (msg->rx_std_id == kRxStdIdA && msg->dlc >= kPayloadSizeA) {
      vt03_online_ = (msg->data[0] != 0);
      gyro_z_rad_s_ = MilliI16ToRad(UnpackI16(&msg->data[2]));
      gyro_x_rad_s_ = MilliI16ToRad(UnpackI16(&msg->data[4]));
      left_button_ = (msg->data[6] != 0);
      right_button_ = (msg->data[7] != 0);
      frame_count_++;
      ReportStatus(kOk);
    } else if (msg->rx_std_id == kRxStdIdB && msg->dlc >= kPayloadSizeB) {
      mouse_x_ = UnpackI16(&msg->data[0]);
      mouse_y_ = UnpackI16(&msg->data[2]);
      // mouse_z_ = UnpackI16(&msg->data[4]);
      mouse_z_ = 0;
      keyboard_value_ = UnpackU16(&msg->data[6]);
      frame_count_++;
      kbd_frame_count_++;
      ReportStatus(kOk);
    } else if (msg->rx_std_id == kRxStdIdC && msg->dlc >= kPayloadSizeC) {
      f32 q[4] = {
          I16ToQuat(UnpackI16(&msg->data[0])),  // w
          I16ToQuat(UnpackI16(&msg->data[2])),  // x
          I16ToQuat(UnpackI16(&msg->data[4])),  // y
          I16ToQuat(UnpackI16(&msg->data[6])),  // z
      };
      f32 euler[3];
      rm::modules::QuatToEuler(q, euler);
      euler_roll_rad_ = euler[1];
      euler_pitch_rad_ = euler[0];
      euler_yaw_rad_ = euler[2];
      pitch_rad_ = euler_pitch_rad_;
      yaw_rad_ = euler_yaw_rad_;
      frame_count_++;
      ReportStatus(kOk);
    } else if (msg->rx_std_id == kRxStdIdD && msg->dlc >= kPayloadSizeD) {
      robot_hp_.hero_1_HP = UnpackU16(&msg->data[0]);
      robot_hp_.engineer_2_HP = UnpackU16(&msg->data[2]);
      robot_hp_.standard_3_HP = UnpackU16(&msg->data[4]);
      robot_hp_.standard_4_HP = UnpackU16(&msg->data[6]);
      frame_count_++;
      ReportStatus(kOk);
    } else if (msg->rx_std_id == kRxStdIdE && msg->dlc >= kPayloadSizeE) {
      robot_hp_.sentry_7_HP = UnpackU16(&msg->data[0]);
      fric_left_rpm_ = UnpackI16(&msg->data[2]);
      fric_right_rpm_ = UnpackI16(&msg->data[4]);
      fric_speed_target_rpm_ = UnpackU16(&msg->data[6]);
      frame_count_++;
      ReportStatus(kOk);
    } else {
      ReportStatus(kFault);
    }
  }

  [[nodiscard]] bool vt03_online() const { return vt03_online_; }
  [[nodiscard]] rm::f32 pitch_rad() const { return pitch_rad_; }
  [[nodiscard]] rm::f32 yaw_rad() const { return yaw_rad_; }
  [[nodiscard]] rm::f32 gyro_z_rad_s() const { return gyro_z_rad_s_; }
  [[nodiscard]] rm::f32 gyro_x_rad_s() const { return gyro_x_rad_s_; }
  [[nodiscard]] rm::f32 euler_yaw_rad() const { return euler_yaw_rad_; }
  [[nodiscard]] rm::f32 euler_pitch_rad() const { return euler_pitch_rad_; }
  [[nodiscard]] rm::f32 euler_roll_rad() const { return euler_roll_rad_; }
  [[nodiscard]] rm::i16 mouse_x() const { return mouse_x_; }
  [[nodiscard]] rm::i16 mouse_y() const { return mouse_y_; }
  [[nodiscard]] rm::i16 mouse_z() const { return mouse_z_; }
  [[nodiscard]] bool left_button() const { return left_button_; }
  [[nodiscard]] bool right_button() const { return right_button_; }
  [[nodiscard]] rm::u16 keyboard_value() const { return keyboard_value_; }
  [[nodiscard]] rm::u32 frame_count() const { return frame_count_; }
  [[nodiscard]] rm::u32 keyboard_frame_count() const { return kbd_frame_count_; }
  [[nodiscard]] const EmyRobotHP &robot_hp() const { return robot_hp_; }
  [[nodiscard]] rm::i16 fric_left_rpm() const { return fric_left_rpm_; }
  [[nodiscard]] rm::i16 fric_right_rpm() const { return fric_right_rpm_; }
  [[nodiscard]] rm::u16 fric_speed_target_rpm() const { return fric_speed_target_rpm_; }

 private:
  static rm::i16 UnpackI16(const rm::u8 *in) {
    const rm::u16 raw = static_cast<rm::u16>((static_cast<rm::u16>(in[0]) << 8) | static_cast<rm::u16>(in[1]));
    return static_cast<rm::i16>(raw);
  }

  static rm::u16 UnpackU16(const rm::u8 *in) {
    return static_cast<rm::u16>((static_cast<rm::u16>(in[0]) << 8) | static_cast<rm::u16>(in[1]));
  }

  static rm::f32 MilliI16ToRad(rm::i16 value) { return static_cast<rm::f32>(value) * 0.001f; }

  static rm::f32 I16ToQuat(rm::i16 value) { return static_cast<rm::f32>(value) / 32767.0f; }

  static constexpr rm::f32 kDegToRad = wheel_legged::params::active::kPi / 180.0f;

  bool vt03_online_{false};
  rm::f32 pitch_rad_{0.0f};
  rm::f32 yaw_rad_{0.0f};
  rm::f32 gyro_z_rad_s_{0.0f};
  rm::f32 gyro_x_rad_s_{0.0f};
  rm::f32 euler_yaw_rad_{0.0f};
  rm::f32 euler_pitch_rad_{0.0f};
  rm::f32 euler_roll_rad_{0.0f};
  rm::i16 mouse_x_{0};
  rm::i16 mouse_y_{0};
  rm::i16 mouse_z_{0};
  bool left_button_{false};
  bool right_button_{false};
  rm::u16 keyboard_value_{0};
  rm::u32 frame_count_{0};
  rm::u32 kbd_frame_count_{0};
  EmyRobotHP robot_hp_{};
  rm::i16 fric_left_rpm_{0};
  rm::i16 fric_right_rpm_{0};
  rm::u16 fric_speed_target_rpm_{0};
};

/**
 * @brief 底盘→云台 CAN 发送桥
 * @note  发送 combat_mode 标志位给云台控制摩擦轮
 *        - 0x120: [0] combat_mode (1=kCombat, 0=其他)
 */
class ChassisToGimbalTxBridge final : public rm::device::CanDevice {
 public:
  static constexpr rm::u16 kTxStdId = 0x120;
  static constexpr rm::usize kPayloadSize = 8U;

  explicit ChassisToGimbalTxBridge(rm::hal::CanInterface &can) : CanDevice(can, kTxStdId) {}

  void SetCombatMode(bool combat) { combat_mode_ = combat; }

  void RxCallback(const rm::hal::CanFrame *msg) override {}

  bool QueueSend() {
    tx_data_[0] = combat_mode_ ? 1 : 0;
    // for (rm::usize i = 1; i < kPayloadSize; ++i) {
    //   tx_data_[i] = 0;
    // }
    can_->Write(kTxStdId, tx_data_.data(), tx_data_.size());
    ReportStatus(kOk);
    return true;
  }

 private:
  bool combat_mode_{false};
  std::array<rm::u8, kPayloadSize> tx_data_{};
};
