#pragma once

#include <cstdint>

#include "../include/chassis/chassis.hpp"
#include "../include/gimbal/gimbal.hpp"
#include "input_resolver.hpp"

/**
 * @file  targets/wheel_legged/control_loop/debug_export.hpp
 * @brief 将本周期的关键内部量填充到 DebugSnapshot（SRAM4，供调试器/DMA 读取）
 */

/**
 * @brief 填充 DebugSnapshot
 * @param tick_ms                当前系统 tick
 * @param input                  本周期输入快照
 * @param chassis_output         底盘 FSM 输出
 * @param gimbal_output          云台 FSM 输出
 * @param chassis_control_output 底盘控制器输出
 * @param gimbal_control_output  云台控制器输出
 * @note  本函数在每个控制周期末调用一次，将分散在各阶段的关键量集中写入 wl_debug 结构体。
 *        wl_debug 放置在 SRAM4 段，可由调试器通过 SWD/JTAG 或 DMA 直接读取，
 *        无需主循环主动发送，零运行时开销。
 */
void UpdateDebugSnapshot(uint32_t tick_ms, const wheel_legged::control_loop::InputSnapshot &input,
                         const chassis::Fsm::Output &chassis_output, const gimbal::Fsm::Output &gimbal_output,
                         const chassis::Chassis::UpdateOutput &chassis_control_output,
                         const gimbal::Gimbal::UpdateOutput &gimbal_control_output);
