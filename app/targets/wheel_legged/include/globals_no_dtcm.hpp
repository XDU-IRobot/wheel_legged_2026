#pragma once

/**
 * @file  globals_no_dtcm.hpp
 * @brief Non-DTCM shared resources for DMA related peripherals.
 */

#include <librm.hpp>

#include "wheel_legged_params.hpp"
#include "usart.h"

/**
 * @brief Shared resources that must live outside DTCM.
 */
struct SharedResourcesNoDtcm {
  rm::hal::Serial<wheel_legged::params::active::globals::kDr16UartRxBufferSize> rc_uart{
      huart5, false, true};
  rm::hal::Serial<wheel_legged::params::active::globals::kImuUartRxBufferSize> imu_uart{
      huart10, false, true};
};

extern __attribute__((section(".sram4"))) SharedResourcesNoDtcm globals_no_dtcm;
