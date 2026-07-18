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

void SelectActiveTofs() {
  // XSHUT is active-low. Keep only one sensor enabled on each I2C bus.
  HAL_GPIO_WritePin(TOF_I2C2_PD14_XSHUT_GPIO_Port, TOF_I2C2_PD14_XSHUT_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TOF_I2C2_PD15_XSHUT_GPIO_Port, TOF_I2C2_PD15_XSHUT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TOF_I2C1_PE14_XSHUT_GPIO_Port, TOF_I2C1_PE14_XSHUT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TOF_I2C1_PE0_XSHUT_GPIO_Port, TOF_I2C1_PE0_XSHUT_Pin, GPIO_PIN_SET);
}
}  // namespace

extern "C" {

/**
 * @brief 应用主函数（由 C 层启动）
 */
void AppMain() {
  SelectActiveTofs();

  globals = &g_globals;
  globals->Init();
  wheel_legged::ai::PolicyTestInit();

  TimerTaskScheduler mainloop{&htim13};
  mainloop.AddTask(wheel_legged::params::active::main::kControlLoopFrequencyHz,
                   etl::delegate<void()>::create<ControlLoop>(), "ControlLoop");
  (void)mainloop.Start();

  for (;;) {
    SelectActiveTofs();

    if (globals->joint_can.has_value()) {
      (void)globals->joint_can->Process();
    }
    if (globals->wheel_can.has_value()) {
      (void)globals->wheel_can->Process();
    }
    if (globals->gimbal_can.has_value()) {
      (void)globals->gimbal_can->Process();
    }
    if (globals->tof.has_value()) {
      (void)globals->tof->Poll();
    }
    if (globals->tof_i2c1.has_value()) {
      (void)globals->tof_i2c1->Poll();
    }
    wheel_legged::ai::PolicyTestPoll();
  }
}
}
