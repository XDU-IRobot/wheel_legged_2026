#include "common/bsp/timer_task_scheduler.hpp"

#include <etl/delegate.h>
#include "include/ai/policy_runner.hpp"
#include "include/globals.hpp"
#include "include/globals_no_dtcm.hpp"

#include "main.h"
#include "tim.h"

/**
 * @file  targets/wheel_legged/main.cc
 * @brief 应用入口：初始化共享资源并启动 500Hz 控制任务
 */
SharedResources *globals{nullptr};
__attribute__((section(".sram4"))) SharedResourcesNoDtcm globals_no_dtcm;
namespace {
SharedResources g_globals;

using Tof = rm::device::Vl53l4cd;
using TofMode = wheel_legged::TofMode;

void DisableAllTofs() {
  // XSHUT is active-low. Sensors on the same I2C bus retain the default 0x29 address,
  // so exactly one sensor per bus may be enabled.
  HAL_GPIO_WritePin(LEFT_FRONT_TOF_XSHUT_GPIO_Port, LEFT_FRONT_TOF_XSHUT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RIGHT_FRONT_TOF_XSHUT_GPIO_Port, RIGHT_FRONT_TOF_XSHUT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LEFT_DOWN_TOF_XSHUT_GPIO_Port, LEFT_DOWN_TOF_XSHUT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RIGHT_DOWN_TOF_XSHUT_GPIO_Port, RIGHT_DOWN_TOF_XSHUT_Pin, GPIO_PIN_RESET);
}

void EnableTofPair(const TofMode mode) {
  if (mode == TofMode::kStairDescend) {
    HAL_GPIO_WritePin(LEFT_DOWN_TOF_XSHUT_GPIO_Port, LEFT_DOWN_TOF_XSHUT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RIGHT_DOWN_TOF_XSHUT_GPIO_Port, RIGHT_DOWN_TOF_XSHUT_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(LEFT_FRONT_TOF_XSHUT_GPIO_Port, LEFT_FRONT_TOF_XSHUT_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RIGHT_FRONT_TOF_XSHUT_GPIO_Port, RIGHT_FRONT_TOF_XSHUT_Pin, GPIO_PIN_SET);
  }
}

std::optional<Tof> &LeftTofForMode(SharedResources &g, const TofMode mode) {
  return mode == TofMode::kStairDescend ? g.left_down_tof : g.left_front_tof;
}

std::optional<Tof> &RightTofForMode(SharedResources &g, const TofMode mode) {
  return mode == TofMode::kStairDescend ? g.right_down_tof : g.right_front_tof;
}

void SwitchTofMode(SharedResources &g, const TofMode target_mode) {
  g.tof_mode_ready = false;

  auto &old_left = LeftTofForMode(g, g.active_tof_mode);
  auto &old_right = RightTofForMode(g, g.active_tof_mode);
  if (old_left.has_value() && old_left->ranging()) (void)old_left->Stop();
  if (old_right.has_value() && old_right->ranging()) (void)old_right->Stop();

  DisableAllTofs();
  HAL_Delay(2U);
  EnableTofPair(target_mode);
  HAL_Delay(10U);

  auto &new_left = LeftTofForMode(g, target_mode);
  auto &new_right = RightTofForMode(g, target_mode);
  const auto left_error = new_left.has_value() ? new_left->Begin() : Tof::Error::kNotStarted;
  const auto right_error = new_right.has_value() ? new_right->Begin() : Tof::Error::kNotStarted;

  g.active_tof_mode = target_mode;
  g.tof_init_error_mask =
      static_cast<std::uint8_t>((left_error == Tof::Error::kOk ? 0U : 1U) | (right_error == Tof::Error::kOk ? 0U : 2U));
  g.tof_mode_ready = g.tof_init_error_mask == 0U;
  g.tof_switch_count = g.tof_switch_count + 1U;
}

void PollActiveTofs(SharedResources &g) {
  if (!g.tof_mode_ready) return;
  auto &left = LeftTofForMode(g, g.active_tof_mode);
  auto &right = RightTofForMode(g, g.active_tof_mode);

  const std::uint32_t poll_start_cycles = DWT->CYCCNT;
  if (left.has_value()) {
    const std::uint32_t start_cycles = DWT->CYCCNT;
    (void)left->Poll();
    const std::uint32_t elapsed_cycles = DWT->CYCCNT - start_cycles;
    g.tof_left_poll_last_us =
        static_cast<std::uint32_t>((static_cast<std::uint64_t>(elapsed_cycles) * 1000000ULL) / SystemCoreClock);
  }
  if (right.has_value()) {
    const std::uint32_t start_cycles = DWT->CYCCNT;
    (void)right->Poll();
    const std::uint32_t elapsed_cycles = DWT->CYCCNT - start_cycles;
    g.tof_right_poll_last_us =
        static_cast<std::uint32_t>((static_cast<std::uint64_t>(elapsed_cycles) * 1000000ULL) / SystemCoreClock);
  }

  const std::uint32_t elapsed_cycles = DWT->CYCCNT - poll_start_cycles;
  const std::uint32_t elapsed_us =
      static_cast<std::uint32_t>((static_cast<std::uint64_t>(elapsed_cycles) * 1000000ULL) / SystemCoreClock);
  g.tof_poll_last_us = elapsed_us;
  if (elapsed_us > g.tof_poll_max_us) g.tof_poll_max_us = elapsed_us;
}

void RequestTofPoll() { g_globals.tof_poll_request_count = g_globals.tof_poll_request_count + 1U; }
}  // namespace

extern "C" {

/**
 * @brief 应用主函数（由 C 层启动）
 */
void AppMain() {
  DisableAllTofs();

  globals = &g_globals;
  globals->Init();
  if constexpr (wheel_legged::params::active::tof::kEnabled) {
    SwitchTofMode(*globals, wheel_legged::TofMode::kAutoJump);
  }
  wheel_legged::ai::PolicyTestInit();

  // .sram4 is NOLOAD and is not cleared by the startup code. Reset cumulative
  // timing counters explicitly before the control timer starts.
  wl_debug.control_jitter_over_100us_count = 0U;
  wl_debug.control_overrun_count = 0U;

  TimerTaskScheduler mainloop{&htim13};
  mainloop.AddTask(wheel_legged::params::active::main::kControlLoopFrequencyHz,
                   etl::delegate<void()>::create<ControlLoop>(), "ControlLoop");
  if constexpr (wheel_legged::params::active::tof::kEnabled) {
    mainloop.AddTask(wheel_legged::params::active::tof::kPollRequestFrequencyHz,
                     etl::delegate<void()>::create<RequestTofPoll>(), "TofPollRequest");
  }
  (void)mainloop.Start();

  [[maybe_unused]] std::uint32_t serviced_tof_poll_request_count = 0U;
  for (;;) {
    if constexpr (wheel_legged::params::active::tof::kEnabled) {
      if (globals->requested_tof_mode != globals->active_tof_mode) {
        SwitchTofMode(*globals, globals->requested_tof_mode);
      }
    }

    if (globals->joint_can.has_value()) {
      (void)globals->joint_can->Process();
    }
    if (globals->wheel_can.has_value()) {
      (void)globals->wheel_can->Process();
    }
    if (globals->gimbal_can.has_value()) {
      (void)globals->gimbal_can->Process();
    }

    if constexpr (wheel_legged::params::active::tof::kEnabled) {
      const std::uint32_t requested_tof_polls = globals->tof_poll_request_count;
      if (requested_tof_polls != serviced_tof_poll_request_count) {
        const std::uint32_t pending_count = requested_tof_polls - serviced_tof_poll_request_count;
        if (pending_count > 1U) {
          globals->tof_poll_coalesced_count = globals->tof_poll_coalesced_count + pending_count - 1U;
        }
        serviced_tof_poll_request_count = requested_tof_polls;
        PollActiveTofs(*globals);
        globals->tof_poll_process_count = globals->tof_poll_process_count + 1U;
      }
    }
    // wheel_legged::ai::PolicyTestPoll();
  }
}
}
