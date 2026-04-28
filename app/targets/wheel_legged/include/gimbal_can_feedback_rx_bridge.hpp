#pragma once

#include <array>

#include <librm.hpp>

class GimbalCanFeedbackRxBridge final : public rm::device::CanDevice {
 public:
  static constexpr rm::u16 kRxStdId0 = 0x119;
  static constexpr rm::usize kPayloadSize = 4U;

  explicit GimbalCanFeedbackRxBridge(rm::hal::CanInterface &can) : CanDevice(can, kRxStdId0) {}

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
  static rm::i16 UnpackI16BigEndian(const rm::u8 *in) {
    const rm::u16 raw = static_cast<rm::u16>((static_cast<rm::u16>(in[0]) << 8) | static_cast<rm::u16>(in[1]));
    return static_cast<rm::i16>(raw);
  }

  static rm::f32 MilliI16ToUnit(rm::i16 value) { return static_cast<rm::f32>(value) * 0.001f; }

  std::array<rm::u8, kPayloadSize> rx_payload_{};
  rm::f32 pitch_rad_{0.0f};
  rm::f32 yaw_rad_{0.0f};
  rm::u32 frame_count_{0U};
};
