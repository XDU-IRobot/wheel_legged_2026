#pragma once

#include <array>

#include <librm.hpp>

class DypCanRxBridge final : public rm::device::CanDevice {
 public:
  static constexpr rm::u16 kRxStdId = 0x120;
  static constexpr rm::usize kPayloadSize = 6U;

  explicit DypCanRxBridge(rm::hal::CanInterface& can) : CanDevice(can, kRxStdId) {}

  void RxCallback(const rm::hal::CanFrame* msg) override {
    if (msg == nullptr || msg->dlc < kPayloadSize) {
      ReportStatus(kFault);
      return;
    }

    distance_raw_left_   = UnpackU16BigEndian(&msg->data[0]);
    distance_raw_right_  = UnpackU16BigEndian(&msg->data[2]);
    last_result_left_    = msg->data[4];
    last_result_right_   = msg->data[5];

    frame_count_++;
    ReportStatus(kOk);
  }

  [[nodiscard]] rm::u16 distance_raw_left()  const { return distance_raw_left_; }
  [[nodiscard]] rm::u16 distance_raw_right() const { return distance_raw_right_; }
  [[nodiscard]] rm::u8  last_result_left()   const { return last_result_left_; }
  [[nodiscard]] rm::u8  last_result_right()  const { return last_result_right_; }
  [[nodiscard]] rm::u32 frame_count()        const { return frame_count_; }

 private:
  static rm::u16 UnpackU16BigEndian(const rm::u8* in) {
    return static_cast<rm::u16>((static_cast<rm::u16>(in[0]) << 8) | static_cast<rm::u16>(in[1]));
  }

  std::array<rm::u8, kPayloadSize> rx_payload_{};
  rm::u16 distance_raw_left_{0U};
  rm::u16 distance_raw_right_{0U};
  rm::u8  last_result_left_{0U};
  rm::u8  last_result_right_{0U};
  rm::u32 frame_count_{0U};
};
