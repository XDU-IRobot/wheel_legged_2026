#include "common/bsp/timer_task_scheduler.hpp"

#include <etl/delegate.h>

#include "include/globals.hpp"
#include "include/globals_no_dtcm.hpp"

#include "tim.h"

namespace {

constexpr float kControlLoopFrequencyHz = 500.0f;

}  // namespace

SharedResources *globals{nullptr};
__attribute__((section(".sram4"))) SharedResourcesNoDtcm globals_no_dtcm;
namespace {
SharedResources g_globals;
}

extern "C" {

void AppMain() {
  globals = &g_globals;
  globals->Init();

  TimerTaskScheduler mainloop{&htim13};
  mainloop.AddTask(kControlLoopFrequencyHz, etl::delegate<void()>::create<ControlLoop>(), "ControlLoop");
  (void)mainloop.Start();

  for (;;) {
    // globals->ServiceCanQueues();
  }
}
}
