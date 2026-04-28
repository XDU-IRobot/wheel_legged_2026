#include "common/bsp/timer_task_scheduler.hpp"

#include <etl/delegate.h>

#include "include/globals.hpp"
#include "include/globals_no_dtcm.hpp"

#include "tim.h"

/**
 * @file  targets/wheel_legged/main.cc
 * @brief 应用入口：初始化共享资源并启动 500Hz 控制任务
 */

namespace {

constexpr float kControlLoopFrequencyHz = 500.0f;

}  // namespace

SharedResources *globals{nullptr};
__attribute__((section(".sram4"))) SharedResourcesNoDtcm globals_no_dtcm;
namespace {
SharedResources g_globals;
}

extern "C" {

/**
 * @brief 应用主函数（由 C 层启动）
 */
void AppMain() {
  globals = &g_globals;
  globals->Init();

  TimerTaskScheduler mainloop{&htim13};
  mainloop.AddTask(kControlLoopFrequencyHz, etl::delegate<void()>::create<ControlLoop>(), "ControlLoop");
  (void)mainloop.Start();

  for (;;) {
    {
      static uint32_t loop_count = 0;
      static uint32_t last_ms = 0;
      loop_count++;
      const uint32_t now_ms = HAL_GetTick();
      const uint32_t elapsed = now_ms - last_ms;
      if (elapsed >= 500) {
        wl_fm_can_loop_freq_hz = static_cast<float>(loop_count) * 1000.0f / static_cast<float>(elapsed);
        loop_count = 0;
        last_ms = now_ms;
      }
    }

    if (globals->joint_can.has_value()) {
      (void)globals->joint_can->Process();
    }
    if (globals->wheel_can.has_value()) {
      (void)globals->wheel_can->Process();
    }
  }
}
}
