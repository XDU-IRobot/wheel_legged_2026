#pragma once

#include <cstdint>
#include <optional>
#include "fdcan.h"
#include "stm32h7xx_it.h"
#include "usart.h"
#include "spi.h"
#include <librm.hpp>

#include "params.hpp"
#include "globals_no_dtcm.hpp"

#include "chassis/chassis.hpp"
#include "chassis/fsm.hpp"
#include "gimbal_can_bridge.hpp"
#include "utils/dyp_a22.hpp"
#include "utils/aimbot_can.hpp"
#include "librm/device/supercap/gk_supercap.hpp"
#include "librm/device/referee/referee.hpp"
#include "../ui/referee_user.hpp"
#include "gimbal/fsm.hpp"
#include "gimbal/gimbal.hpp"
#include "gimbal/gimbal_ident.hpp"
#include "librm/device/remote/dr16.hpp"
#include "common/sd_spi.hpp"
#include "common/sd_log.hpp"
#if WHEEL_LEGGED_ROBOT_VARIANT == 1
#include "gimbal/shoot_3fric.hpp"
#else
#include "gimbal/shoot_2fric.hpp"
#endif

/**
 * @file  targets/wheel_legged/include/globals.hpp
 * @brief 轮腿目标共享资源与调试导出变量
 */

/**
 * @brief 任务间共享的设备、控制器与状态机对象
 * @note  与硬件相关的成员在 Init() 中延迟构造，避免静态初始化阶段硬件访问异常
 */
struct SharedResources {
  using DmMitMotor = rm::device::DmMotor<rm::device::DmMotorControlMode::kMit>;

  SharedResourcesNoDtcm *no_dtcm{&globals_no_dtcm};  ///< DMA 相关外设资源

  rm::device::DR16 dr16{no_dtcm->rc_uart};  ///< 遥控器接收机

  // 保持值语义成员，实际构造延后到 Init()。
  std::optional<rm::hal::ThrottledCan<>> joint_can{};         ///< 腿部 DM 电机 CAN
  std::optional<rm::hal::ThrottledCan<>> wheel_can{};         ///< 轮毂/偏航电机 CAN
  std::optional<rm::hal::ThrottledCan<>> gimbal_can{};        ///< 云台俯仰与云台惯导 CAN
  std::optional<DmMitMotor> dm_lf{};                          ///< 左前腿关节 DM 电机
  std::optional<DmMitMotor> dm_lb{};                          ///< 左后腿关节 DM 电机
  std::optional<DmMitMotor> dm_rf{};                          ///< 右前腿关节 DM 电机
  std::optional<DmMitMotor> dm_rb{};                          ///< 右后腿关节 DM 电机
  std::optional<rm::device::M3508> left_wheel{};              ///< 左轮 M3508
  std::optional<rm::device::M3508> right_wheel{};             ///< 右轮 M3508
  std::optional<rm::device::HipnucImu> chassis_imu{};         ///< 底盘惯导
  std::optional<GimbalToChassisRxBridge> gimbal_rx{};         ///< 云台→底盘 CAN 桥（惯导+键鼠）
  std::optional<ChassisToGimbalTxBridge> chassis_tx{};        ///< 底盘→云台 CAN 桥（combat标志）
  std::optional<rm::device::AimbotCanCommunicator> aimbot{};  ///< 自瞄 CAN 通信 (gimbal_can)
  std::optional<rm::device::Referee<rm::device::RefereeRevision::kNewV120>> referee{};         ///< 裁判系统串口w
  std::optional<rm::device::RefereeUser<rm::device::RefereeRevision::kNewV120>> subReferee{};  ///< 裁判子协议
  std::optional<rm::device::GkSupercap> supercap{};  ///< 超级电容 (wheel_can)
  std::optional<rm::device::DypA22> dyp_left{};      ///< DYP 左超声波 (UART8)
  std::optional<rm::device::DypA22> dyp_right{};     ///< DYP 右超声波 (UART9)
  std::optional<SdSpi> sd_card{};                   ///< SD 卡 (SPI1 + PE15 CS)
  std::optional<SdLogger> sd_logger{};              ///< SD 卡日志记录器

  std::optional<DmMitMotor> yaw_motor{};    ///< 云台偏航 DM 电机
  std::optional<DmMitMotor> pitch_motor{};  ///< 云台俯仰 DM 电机

#if WHEEL_LEGGED_ROBOT_VARIANT == 1
  std::optional<DmMitMotor> booster_motor{};  ///< DM 拨盘 (wheel_can, hero)
  wheel_legged::ShootController shoot_controller{};
#else
  std::optional<rm::device::M3508> dial{};  ///< 拨盘 (wheel_can)
  Shoot shoot{};                            ///< 发射机构状态机
#endif

  chassis::Fsm chassis_fsm{};          ///< 底盘状态机
  chassis::Chassis chassis{};          ///< 底盘控制器
  gimbal::Fsm gimbal_fsm{};            ///< 云台状态机
  gimbal::Gimbal gimbal{};             ///< 云台控制器
  gimbal::GimbalIdent gimbal_ident{};  ///< 云台辨识/前馈验证控制器

  bool ui_refresh_key{false};  ///< E 键按下时 UI 刷新使能（由 input 语义折叠设置）

  /**
   * @brief 懒加载单例入口
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
   * @brief 初始化外设对象、状态机与控制器
   */
  void Init() {
    const auto prepare_uart_rx_to_idle_dma = [](UART_HandleTypeDef &huart, const IRQn_Type uart_irqn,
                                                const IRQn_Type dma_rx_irqn) {
      // Clear stale UART/DMA state before librm starts ReceiveToIdle mode.
      (void)HAL_UART_AbortReceive(&huart);
      (void)HAL_UART_Abort(&huart);
      __HAL_UART_CLEAR_FLAG(&huart, UART_CLEAR_PEF);
      __HAL_UART_CLEAR_FLAG(&huart, UART_CLEAR_FEF);
      __HAL_UART_CLEAR_FLAG(&huart, UART_CLEAR_NEF);
      __HAL_UART_CLEAR_FLAG(&huart, UART_CLEAR_OREF);
      __HAL_UART_SEND_REQ(&huart, UART_RXDATA_FLUSH_REQUEST);
      huart.ErrorCode = HAL_UART_ERROR_NONE;
      huart.RxState = HAL_UART_STATE_READY;
      huart.gState = HAL_UART_STATE_READY;
      HAL_NVIC_ClearPendingIRQ(uart_irqn);
      HAL_NVIC_ClearPendingIRQ(dma_rx_irqn);
    };

    prepare_uart_rx_to_idle_dma(huart5, UART5_IRQn, DMA1_Stream0_IRQn);
    prepare_uart_rx_to_idle_dma(huart10, USART10_IRQn, DMA1_Stream5_IRQn);
    prepare_uart_rx_to_idle_dma(huart1, USART1_IRQn, DMA2_Stream0_IRQn);
    prepare_uart_rx_to_idle_dma(huart7, UART7_IRQn, DMA1_Stream3_IRQn);
    prepare_uart_rx_to_idle_dma(huart8, UART8_IRQn, DMA2_Stream3_IRQn);
    prepare_uart_rx_to_idle_dma(huart9, UART9_IRQn, DMA2_Stream2_IRQn);

    const auto prepare_uart_tx_dma = [](UART_HandleTypeDef &huart, const IRQn_Type uart_irqn,
                                        const IRQn_Type dma_tx_irqn) {
      (void)HAL_UART_AbortTransmit(&huart);
      __HAL_UART_CLEAR_FLAG(&huart, UART_FLAG_TC);
      huart.ErrorCode = HAL_UART_ERROR_NONE;
      huart.gState = HAL_UART_STATE_READY;
      HAL_NVIC_ClearPendingIRQ(uart_irqn);
      HAL_NVIC_ClearPendingIRQ(dma_tx_irqn);
    };

    prepare_uart_tx_dma(huart1, USART1_IRQn, DMA2_Stream1_IRQn);

    dr16.Begin();

    if (!joint_can.has_value()) {
      joint_can.emplace(wheel_legged::params::active::globals::kJointCanTxLimitHz, hfdcan1);
      joint_can->SetFilter(0, 0);
      joint_can->Begin();
    }
    if (!wheel_can.has_value()) {
      wheel_can.emplace(wheel_legged::params::active::globals::kWheelCanTxLimitHz, hfdcan2);
      wheel_can->SetFilter(0, 0);
      wheel_can->Begin();
    }
    if (!gimbal_can.has_value()) {
      gimbal_can.emplace(wheel_legged::params::active::globals::kGimbalCanTxLimitHz, hfdcan3);
      gimbal_can->SetFilter(0, 0);
      gimbal_can->Begin();
    }
    if (!gimbal_rx.has_value()) {
      gimbal_rx.emplace(*gimbal_can);
    }
    if (!chassis_tx.has_value()) {
      chassis_tx.emplace(*gimbal_can);
    }
    if (!aimbot.has_value()) {
      aimbot.emplace(*gimbal_can);
    }
    if (!referee.has_value()) {
      referee.emplace();
      no_dtcm->referee_uart.AttachRxCallback([this](etl::span<const rm::u8> data) {
        for (const auto byte : data) {
          *referee << byte;
        }
      });
      no_dtcm->referee_uart.Start();
    }
    if (!subReferee.has_value()) {
      subReferee.emplace(*referee);
      referee->AttachCallback([this](rm::u16 cmd_id, rm::u8 seq) { subReferee->AttachCallback(cmd_id, seq); });
    }
    if (!supercap.has_value()) {
      supercap.emplace(*wheel_can);
    }
    if (!dyp_left.has_value()) {
      dyp_left.emplace(no_dtcm->dyp_left_uart);
      dyp_left->Start();
    }
    if (!dyp_right.has_value()) {
      dyp_right.emplace(no_dtcm->dyp_right_uart);
      dyp_right->Start();
    }

    if (!dm_lf.has_value()) {
      dm_lf.emplace(*joint_can, wheel_legged::params::active::globals::kDmLfSettings);
      dm_lb.emplace(*joint_can, wheel_legged::params::active::globals::kDmLbSettings);
      dm_rf.emplace(*joint_can, wheel_legged::params::active::globals::kDmRfSettings);
      dm_rb.emplace(*joint_can, wheel_legged::params::active::globals::kDmRbSettings);
    }

    if (!left_wheel.has_value()) {
      left_wheel.emplace(*wheel_can, wheel_legged::params::active::globals::kLeftWheelId);
      right_wheel.emplace(*wheel_can, wheel_legged::params::active::globals::kRightWheelId);
    }

    if (!pitch_motor.has_value()) {
      pitch_motor.emplace(*gimbal_can, wheel_legged::params::active::gimbal::kPitchMotorSettings);
    }

    if (!yaw_motor.has_value()) {
      yaw_motor.emplace(*wheel_can, wheel_legged::params::active::gimbal::kYawMotorSettings);
    }

#if WHEEL_LEGGED_ROBOT_VARIANT == 1
    if (!booster_motor.has_value()) {
      booster_motor.emplace(*wheel_can, wheel_legged::params::active::shoot::kBoosterDmSettings, true);
      shoot_controller.Init(&*booster_motor);
    }
#else
    if (!dial.has_value()) {
      dial.emplace(*wheel_can, wheel_legged::params::active::globals::kDialId);
    }
#endif

    if (!chassis_imu.has_value()) {
      chassis_imu.emplace(no_dtcm->imu_uart);
      chassis_imu->Begin();
    }

    chassis_fsm.Init();
    chassis.Init();
    gimbal_fsm.Init();
    gimbal.Init();
    gimbal_ident.Init();

#if WHEEL_LEGGED_ROBOT_VARIANT != 1
    shoot.Init();
#endif


    if (!sd_card.has_value()) {
      sd_card.emplace(hspi1, MCP_CS_GPIO_Port, MCP_CS_Pin);

      // ── 修复 SPI1 配置以适应外接 SD 卡模块 ──
      // 1. 启用 KeepIOState：HAL 调用间引脚不浮空，SD 卡不会误判总线错误
      // 2. 关闭 NSSPulse：软件 CS 模式下脉冲会干扰时序
      const uint32_t saved_br = hspi1.Init.BaudRatePrescaler;
      const uint32_t saved_keepio = hspi1.Init.MasterKeepIOState;
      const uint32_t saved_nsspulse = hspi1.Init.NSSPMode;

      hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;  // ~390kHz, SD 初始化要求 ≤400kHz
      hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
      hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
      HAL_SPI_Init(&hspi1);

      // 3. MISO (PB4) 启用内部上拉，防止无卡或卡不驱动时浮空
      GPIO_InitTypeDef miso_cfg = {};
      miso_cfg.Pin = GPIO_PIN_4;
      miso_cfg.Mode = GPIO_MODE_AF_PP;
      miso_cfg.Pull = GPIO_PULLUP;
      miso_cfg.Speed = GPIO_SPEED_FREQ_LOW;
      miso_cfg.Alternate = GPIO_AF5_SPI1;
      HAL_GPIO_Init(GPIOB, &miso_cfg);

      if (sd_card->Init() != SdSpi::Error::kNone) {
        // SD 卡初始化失败：卡不在位或硬件问题，静默降级
        hspi1.Init.BaudRatePrescaler = saved_br;
        hspi1.Init.MasterKeepIOState = saved_keepio;
        hspi1.Init.NSSPMode = saved_nsspulse;
        HAL_SPI_Init(&hspi1);
        sd_card.reset();
      } else {
        // 初始化完成，恢复高速分频（保持 KeepIOState 和 NSSPulse 修复）
        hspi1.Init.BaudRatePrescaler = saved_br;
        HAL_SPI_Init(&hspi1);

        // ── SD Logger 测试配置：100Hz / 60s / 开机 1s 后 auto-start ──
        {
          SdLogger::Config log_cfg;
          log_cfg.decimation = 5;       // 500Hz / 5 = 100Hz
          log_cfg.max_records = 0;      // auto-calc
          log_cfg.max_duration_s = 120;  // 120 秒
          log_cfg.base_lba = 2048;
          log_cfg.sd = &*sd_card;

          sd_logger.emplace();
          sd_logger->Configure(log_cfg);

          extern float sdlog_test_val_a;
          sd_logger->RegisterField("test_val_a", SdLogger::FieldType::kFloat32, &sdlog_test_val_a);
        }
      }
    }
  }
};

extern SharedResources *globals;

#include "debug.hpp"

void ControlLoop();
