#pragma once

/**
 * @file  globals_no_dtcm.hpp
 * @brief Non-DTCM shared resources for DMA related peripherals.
 */

#include <librm.hpp>

#include "params.hpp"
#include "usart.h"

/**
 * @brief Shared resources that must live outside DTCM.
 */
struct SharedResourcesNoDtcm {
  rm::hal::Serial<wheel_legged::params::active::globals::kDr16UartRxBufferSize> rc_uart{huart5, false, true};
  rm::hal::Serial<wheel_legged::params::active::globals::kImuUartRxBufferSize> imu_uart{huart10, false, true};
  rm::hal::Serial<wheel_legged::params::active::globals::kRefereeUartRxBufferSize> referee_uart{huart1, false, true};
  rm::hal::Serial<wheel_legged::params::common::gimbal_ident_common::kIdentUartTxBufSize> ident_uart{huart7, false,
                                                                                                     false};
  rm::hal::Serial<wheel_legged::params::active::globals::kDypUartRxBufferSize> dyp_left_uart{huart8, false, false};
  rm::hal::Serial<wheel_legged::params::active::globals::kDypUartRxBufferSize> dyp_right_uart{huart9, false, false};
};

extern __attribute__((section(".sram4"))) SharedResourcesNoDtcm globals_no_dtcm;
