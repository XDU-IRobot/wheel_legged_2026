#pragma once

#include <chrono>
#include <cstdint>

#include <librm.hpp>

#include "globals_no_dtcm.hpp"

#include "chassis/chassis.hpp"
#include "chassis/fsm.hpp"
#include "gimbal/fsm.hpp"
#include "librm/device/remote/dr16.hpp"

/**
 * @brief 板级全局共享资源
 */
struct SharedResources {
  /**
   * @brief DTCM 外的 DMA 相关资源指针
   */
  SharedResourcesNoDtcm *no_dtcm{&globals_no_dtcm};

  /**
   * @brief 遥控器
   *
   * 遥控器接在底盘 MCU 本地，因此直接使用 no_dtcm 中的 rc_uart。
   */
  rm::device::DR16 dr16{no_dtcm->rc_uart};

  /**
   * @brief 底盘状态机
   */
  chassis::Fsm chassis_fsm{};

  /**
   * @brief 底盘控制模块
   */
  chassis::Chassis chassis{};

  /**
   * @brief 云台状态机
   */
  gimbal::Fsm gimbal_fsm{};

  /**
   * @brief 获取单例实例
   */
  static SharedResources &GetInstance() {
    static SharedResources *shared_resources_instance{nullptr};

    if (shared_resources_instance == nullptr) {
      shared_resources_instance = new SharedResources;
      shared_resources_instance->Init();
    }

    return *shared_resources_instance;
  }

  /**
   * @brief 初始化所有全局对象
   */
  void Init() {
    // dr16.SetHeartbeatTimeout(std::chrono::milliseconds(50));

    dr16.Begin();

    chassis_fsm.Init();
    chassis.Init();
    gimbal_fsm.Init();
  }
};

/**
 * @brief 全局资源指针
 *
 * 建议在 main.cc 中完成赋值：
 *
 *   globals = &Globals::GetInstance();
 *
 */
extern SharedResources *globals;

extern "C" {
extern volatile uint32_t wl_fm_tick_ms;

extern volatile uint8_t wl_fm_chassis_mode;
extern volatile uint8_t wl_fm_gimbal_mode;

extern volatile uint8_t wl_fm_chassis_state_changed;
extern volatile uint8_t wl_fm_gimbal_state_changed;

extern volatile uint8_t wl_fm_dr16_online;
extern volatile int32_t wl_fm_dr16_switch_l;
extern volatile int32_t wl_fm_dr16_switch_r;
extern volatile int16_t wl_fm_dr16_dial;

extern volatile uint8_t wl_fm_dr16_enable_request;
extern volatile uint8_t wl_fm_dr16_spin_request;
extern volatile uint8_t wl_fm_dr16_jump_trigger_edge;

extern volatile float wl_fm_chassis_leg_length_m;
extern volatile float wl_fm_chassis_speed_mps;
}

/**
 * @brief 主控制循环
 *
 * 由 main.cc 中的定时器任务调度器以固定周期调用。
 */
void ControlLoop();
