#pragma once

#include <array>

#include <librm.hpp>

/**
 * @file  targets/wheel_legged/include/gimbal_can_feedback_rx_bridge.hpp
 * @brief 接收云台惯导 CAN 反馈并转换为控制环角度输入
 */

/**
 * @brief 云台惯导反馈接收桥
 * @note  载荷格式为 pitch/yaw 两个大端 int16，单位为 0.001rad。
 */
class GimbalCanFeedbackRxBridge final : public rm::device::CanDevice {
 public:
  static constexpr rm::u16 kRxStdId0 = 0x119;      ///< 云台惯导反馈标准帧 ID
  static constexpr rm::usize kPayloadSize = 4U;    ///< pitch/yaw 两个 int16

  /** @brief 绑定 CAN 总线并注册接收 ID */
  explicit GimbalCanFeedbackRxBridge(rm::hal::CanInterface &can) : CanDevice(can, kRxStdId0) {}

  /**
   * @brief CAN 接收回调
   * @param msg 原始 CAN 帧
   */
  void RxCallback(const rm::hal::CanFrame *msg) override {
    if (msg == nullptr || msg->dlc < kPayloadSize) {
      ReportStatus(kFault);
      return;
    }

    rx_payload_[0] = msg->data[0];
    rx_payload_[1] = msg->data[1];
    rx_payload_[2] = msg->data[2];
    rx_payload_[3] = msg->data[3];

    pitch_rad_ = MilliI16ToUnit(UnpackI16BigEndian(&rx_payload_[0]));
    yaw_rad_ = MilliI16ToUnit(UnpackI16BigEndian(&rx_payload_[2]));

    frame_count_++;
    ReportStatus(kOk);
  }

  [[nodiscard]] rm::f32 pitch_rad() const { return pitch_rad_; }
  [[nodiscard]] rm::f32 yaw_rad() const { return yaw_rad_; }
  [[nodiscard]] rm::u32 frame_count() const { return frame_count_; }

 private:
  /** @brief 解包大端有符号 16 位整数 */
  static rm::i16 UnpackI16BigEndian(const rm::u8 *in) {
    const rm::u16 raw = static_cast<rm::u16>((static_cast<rm::u16>(in[0]) << 8) | static_cast<rm::u16>(in[1]));
    return static_cast<rm::i16>(raw);
  }

  /** @brief 将毫弧度 int16 转为弧度浮点数 */
  static rm::f32 MilliI16ToUnit(rm::i16 value) { return static_cast<rm::f32>(value) * 0.001f; }

  std::array<rm::u8, kPayloadSize> rx_payload_{};  ///< 最近一次接收载荷
  rm::f32 pitch_rad_{0.0f};                        ///< 云台惯导俯仰角
  rm::f32 yaw_rad_{0.0f};                          ///< 云台惯导偏航角
  rm::u32 frame_count_{0U};                        ///< 有效接收帧计数
};
