#pragma once

/**
 * @file  globals_no_dtcm.hpp
 * @brief 放置于 DTCM 外（RAM_D1/.sram4）的 DMA 相关全局资源
 */

#include <librm.hpp>

#include "usart.h"

/**
 * STM32H7 的 DTCM 仅 CPU 可访问，DMA 相关对象需放在 DTCM 外。
 * linker script 已提供：
 *   .sram4 (NOLOAD) : { *(.sram4) } >RAM_D1
 */
struct SharedResourcesNoDtcm {
  rm::hal::stm32::Uart rc_uart{huart5, 18, rm::hal::stm32::UartMode::kNormal,
                               rm::hal::stm32::UartMode::kDma};
  rm::hal::stm32::Uart imu_uart{huart10, 518, rm::hal::stm32::UartMode::kNormal, rm::hal::stm32::UartMode::kDma};
};

extern __attribute__((section(".sram4"))) SharedResourcesNoDtcm globals_no_dtcm;
