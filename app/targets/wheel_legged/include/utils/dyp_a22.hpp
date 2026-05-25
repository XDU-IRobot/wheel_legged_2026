/*
  Copyright (c) 2026 XDU-IRobot

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

/**
 * @file  librm/device/sensor/dyp_a22.hpp
 * @brief DYP-A22 UART受控输出驱动
 */

#ifndef LIBRM_DEVICE_SENSOR_DYP_A22_HPP
#define LIBRM_DEVICE_SENSOR_DYP_A22_HPP

#include "librm/core/typedefs.hpp"
#include "librm/device/device.hpp"
#include "librm/hal/serial.hpp"

namespace rm::device {

/**
 * @brief DYP-A22 UART受控输出传感器
 *
 * 通信参数(默认):
 * - 115200, 8N1, TTL
 *
 * 触发逻辑:
 * - 对RX引脚发送任意串口数据即可触发一次测量
 *
 * 返回帧格式:
 * - Byte0: 0xFF
 * - Byte1: Data_H
 * - Byte2: Data_L
 * - Byte3: SUM = (0xFF + Data_H + Data_L) & 0xFF
 */
class DypA22 : public Device {
 public:
  enum class DistanceUnit : u8 {
    kMillimeter,   ///< 距离单位为mm
    kMicrosecond,  ///< 距离单位为us(可通过 /5.75 换算成mm)
  };

  enum class Result : u8 {
    kUnknown,
    kValid,
    kInterference,  ///< 0xFFFE
    kOutOfRange,    ///< 0xFFFD
    kChecksumError,
  };

  DypA22() = delete;
  explicit DypA22(hal::SerialInterface &serial, u8 trigger_byte = 0x01);

  void Start() const;
  void TriggerOnce() const;
  void SetTriggerByte(u8 trigger_byte);
  void SetDistanceUnit(DistanceUnit unit);

  [[nodiscard]] u16 distance_raw() const;
  [[nodiscard]] f32 distance_mm() const;
  [[nodiscard]] bool data_valid() const;
  [[nodiscard]] Result last_result() const;
  [[nodiscard]] u16 frame_count() const;

 private:
  void RxCallback(etl::span<const u8> data);
  void ProcessByte(u8 byte);
  void ParseFrame(u8 data_h, u8 data_l, u8 sum);

  hal::SerialInterface *serial_;

  static constexpr u8 kFrameHeader = 0xFF;
  static constexpr u16 kInterferenceCode = 0xFFFE;
  static constexpr u16 kOutOfRangeCode = 0xFFFD;

  enum class ParseState : u8 {
    kWaitHeader,
    kDataHigh,
    kDataLow,
    kChecksum,
  };

  ParseState parse_state_{ParseState::kWaitHeader};
  u8 frame_data_h_{0};
  u8 frame_data_l_{0};

  u8 trigger_byte_{0x01};
  DistanceUnit distance_unit_{DistanceUnit::kMillimeter};
  Result last_result_{Result::kUnknown};
  u16 distance_raw_{0};
  u16 frame_count_{0};
};

}  // namespace rm::device

#endif  // LIBRM_DEVICE_SENSOR_DYP_A22_HPP
