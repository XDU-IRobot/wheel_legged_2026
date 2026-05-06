#pragma once

#include <etl/span.h>

#include "librm/device/referee/referee.hpp"
#include "librm/hal/serial_interface.hpp"

/**
 * @file  targets/wheel_legged/include/referee_rx_bridge.hpp
 * @brief 裁判系统串口接收桥，将 UART 字节流接入 librm Referee 解析器
 */

namespace wheel_legged {

/**
 * @brief 裁判系统串口接收桥
 * @note  仿照 DR16 串口回调模式，通过 AsyncReadable::AttachRxCallback 将
 *        裁判系统 UART 的 DMA+IDLE 接收数据逐个字节喂给 Referee 解析。
 */
template <rm::device::RefereeRevision revision>
class RefereeRxBridge {
 public:
  using RefereeType = rm::device::Referee<revision>;

  /**
   * @param referee 裁判系统解析器实例（位于 SharedResources 中）
   * @param serial  UART 串口对象（如 referee_uart）
   */
  explicit RefereeRxBridge(RefereeType &referee, rm::hal::AsyncReadable &serial)
      : referee_(&referee), serial_(&serial) {
    serial_->AttachRxCallback([this](etl::span<const rm::u8> data) { RxCallback(data); });
  }

  /**
   * @brief 启动串口接收
   */
  void Begin() { serial_->Start(); }

  /**
   * @brief 访问裁判系统解析后的协议数据
   */
  [[nodiscard]] const auto &data() const { return referee_->data(); }

  /**
   * @brief 裁判系统设备的当前在线状态
   */
  [[nodiscard]] rm::device::Device::Status online_status() const { return referee_->online_status(); }

  /**
   * @brief 近 10 轮平均丢包率（百分比）
   */
  [[nodiscard]] rm::f32 loss_rate() const { return referee_->loss_rate(); }

 private:
  void RxCallback(etl::span<const rm::u8> data) {
    for (const rm::u8 byte : data) {
      *referee_ << byte;
    }
  }

  RefereeType *referee_;
  rm::hal::AsyncReadable *serial_;
};

}  // namespace wheel_legged
