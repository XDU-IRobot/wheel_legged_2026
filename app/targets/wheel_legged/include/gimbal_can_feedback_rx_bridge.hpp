#pragma once

#include <array>

#include <librm.hpp>
#include "wheel_legged_params.hpp"

/**
 * @file  targets/wheel_legged/include/gimbal_can_feedback_rx_bridge.hpp
 * @brief 鎺ユ敹浜戝彴鎯 CAN 鍙嶉骞惰浆鎹负鎺у埗鐜搴﹁緭鍏?
 */

/**
 * @brief 浜戝彴鎯鍙嶉鎺ユ敹妗?
 * @note  杞借嵎鏍煎紡涓?pitch/yaw 涓や釜澶х int16锛屽崟浣嶄负 0.001rad銆?
 */
class GimbalCanFeedbackRxBridge final : public rm::device::CanDevice {
 public:
  static constexpr rm::u16 kRxStdId0 = wheel_legged::params::active::gimbal_can_bridge::kRxStdId;    ///< 浜戝彴鎯鍙嶉鏍囧噯甯?ID
  static constexpr rm::usize kPayloadSize = wheel_legged::params::active::gimbal_can_bridge::kPayloadSize;  ///< pitch/yaw 涓や釜 int16

  /** @brief 缁戝畾 CAN 鎬荤嚎骞舵敞鍐屾帴鏀?ID */
  explicit GimbalCanFeedbackRxBridge(rm::hal::CanInterface &can) : CanDevice(can, kRxStdId0) {}

  /**
   * @brief CAN 鎺ユ敹鍥炶皟
   * @param msg 鍘熷 CAN 甯?
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
  /** @brief 瑙ｅ寘澶х鏈夌鍙?16 浣嶆暣鏁?*/
  static rm::i16 UnpackI16BigEndian(const rm::u8 *in) {
    const rm::u16 raw = static_cast<rm::u16>((static_cast<rm::u16>(in[0]) << 8) | static_cast<rm::u16>(in[1]));
    return static_cast<rm::i16>(raw);
  }

  /** @brief 灏嗘寮у害 int16 杞负寮у害娴偣鏁?*/
  static rm::f32 MilliI16ToUnit(rm::i16 value) {
    return static_cast<rm::f32>(value) * wheel_legged::params::active::gimbal_can_bridge::kMilliScale;
  }

  std::array<rm::u8, kPayloadSize> rx_payload_{};  ///< 鏈€杩戜竴娆℃帴鏀惰浇鑽?
  rm::f32 pitch_rad_{0.0f};                        ///< 浜戝彴鎯淇话瑙?
  rm::f32 yaw_rad_{0.0f};                          ///< 浜戝彴鎯鍋忚埅瑙?
  rm::u32 frame_count_{0U};                        ///< 鏈夋晥鎺ユ敹甯ц鏁?
};

