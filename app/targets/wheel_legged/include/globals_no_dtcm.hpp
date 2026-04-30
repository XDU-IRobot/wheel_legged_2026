#pragma once

/**
 * @file  globals_no_dtcm.hpp
 * @brief 放置在 DTCM 之外（RAM_D1/.sram4）的 DMA 相关全局资源
 */

#include <librm.hpp>

#include "wheel_legged_params.hpp"
#include "usart.h"

/**
 * @brief 非 DTCM 共享资源
 * @note  STM32H7 的 DTCM 仅 CPU 可访问，DMA 相关对象必须放在 DTCM 之外。
 *        链接脚本中 .sram4 已映射到 RAM_D1。
 */
struct SharedResourcesNoDtcm {
  rm::hal::stm32::Uart rc_uart{huart5, wheel_legged::params::active::globals::kDr16UartRxBufferSize,
                               rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};  ///< DR16 接收 UART
  rm::hal::stm32::Uart imu_uart{huart10, wheel_legged::params::active::globals::kImuUartRxBufferSize,
                                rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};  ///< 底盘惯导 UART
};

extern __attribute__((section(".sram4"))) SharedResourcesNoDtcm globals_no_dtcm;
