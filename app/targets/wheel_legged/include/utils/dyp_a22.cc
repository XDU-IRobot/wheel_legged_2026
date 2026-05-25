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
 * @file  librm/device/sensor/dyp_a22.cc
 * @brief DYP-A22 UART受控输出驱动
 */

#include "dyp_a22.hpp"

namespace rm::device {

DypA22::DypA22(hal::SerialInterface &serial, u8 trigger_byte) : serial_(&serial), trigger_byte_(trigger_byte) {
  serial_->AttachRxCallback(std::bind(&DypA22::RxCallback, this, std::placeholders::_1));
}

void DypA22::Start() const { serial_->Start(); }

void DypA22::TriggerOnce() const { serial_->Write(&trigger_byte_, 1); }

void DypA22::SetTriggerByte(u8 trigger_byte) { trigger_byte_ = trigger_byte; }

void DypA22::SetDistanceUnit(DistanceUnit unit) { distance_unit_ = unit; }

u16 DypA22::distance_raw() const { return distance_raw_; }

f32 DypA22::distance_mm() const {
  if (distance_unit_ == DistanceUnit::kMicrosecond) {
    return static_cast<f32>(distance_raw_) / 5.75f;
  }
  return static_cast<f32>(distance_raw_);
}

bool DypA22::data_valid() const { return last_result_ == Result::kValid; }

DypA22::Result DypA22::last_result() const { return last_result_; }

u16 DypA22::frame_count() const { return frame_count_; }

void DypA22::RxCallback(etl::span<const u8> data) {
  for (const auto byte : data) {
    ProcessByte(byte);
  }
}

void DypA22::ProcessByte(u8 byte) {
  switch (parse_state_) {
    case ParseState::kWaitHeader:
      if (byte == kFrameHeader) {
        parse_state_ = ParseState::kDataHigh;
      }
      break;
    case ParseState::kDataHigh:
      frame_data_h_ = byte;
      parse_state_ = ParseState::kDataLow;
      break;
    case ParseState::kDataLow:
      frame_data_l_ = byte;
      parse_state_ = ParseState::kChecksum;
      break;
    case ParseState::kChecksum:
      ParseFrame(frame_data_h_, frame_data_l_, byte);
      parse_state_ = ParseState::kWaitHeader;
      break;
  }
}

void DypA22::ParseFrame(u8 data_h, u8 data_l, u8 sum) {
  const u8 expected_sum = static_cast<u8>((kFrameHeader + data_h + data_l) & 0xFF);
  if (sum != expected_sum) {
    last_result_ = Result::kChecksumError;
    return;
  }

  const u16 value = static_cast<u16>((static_cast<u16>(data_h) << 8) | data_l);
  distance_raw_ = value;
  frame_count_++;

  if (value == kInterferenceCode) {
    last_result_ = Result::kInterference;
    ReportStatus(kFault);
    return;
  }

  if (value == kOutOfRangeCode) {
    last_result_ = Result::kOutOfRange;
    ReportStatus(kOk);
    return;
  }

  last_result_ = Result::kValid;
  ReportStatus(kOk);
}

}  // namespace rm::device
