#pragma once

#include "task/runtime/task_context.hpp"

/**
 * @file  task/api/task_entrypoints.hpp
 * @brief 四个任务模块的初始化与更新入口声明
 */

namespace tasking {

/**
 * @brief 通信任务初始化
 */
void CommTaskInit(TaskContext &ctx);

/**
 * @brief 通信任务单步更新
 */
void CommTaskUpdate(TaskContext &ctx);

/**
 * @brief 状态机任务初始化
 */
void FsmTaskInit(TaskContext &ctx);

/**
 * @brief 状态机任务单步更新
 */
void FsmTaskUpdate(TaskContext &ctx);

/**
 * @brief 力矩总计算任务初始化
 */
void TorqueComputeTaskInit(TaskContext &ctx);

/**
 * @brief 力矩总计算任务单步更新
 */
void TorqueComputeTaskUpdate(TaskContext &ctx);

/**
 * @brief 电机输出任务初始化
 */
void MotorOutputTaskInit(TaskContext &ctx);

/**
 * @brief 电机输出任务单步更新
 */
void MotorOutputTaskUpdate(TaskContext &ctx);

}  // namespace tasking
