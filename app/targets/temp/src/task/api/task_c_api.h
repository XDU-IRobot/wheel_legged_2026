#ifndef SRC_TASK_TASK_C_API_H
#define SRC_TASK_TASK_C_API_H


/**
 * @file  task/api/task_c_api.h
 * @brief 任务 C 接口（供 FreeRTOS 线程入口调用）
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化任务队列上下文和消息队列
 */
void TaskQueuesInit(void);

/**
 * @brief 通信任务初始化
 */
void CommTaskInitC(void);

/**
 * @brief 通信任务单步更新
 */
void CommTaskUpdateC(void);

/**
 * @brief 力矩计算任务初始化
 */
void TorqueTaskInitC(void);

/**
 * @brief 力矩计算任务单步更新
 */
void TorqueTaskUpdateC(void);

/**
 * @brief 状态机任务初始化
 */
void FsmTaskInitC(void);

/**
 * @brief 状态机任务单步更新
 */
void FsmTaskUpdateC(void);

/**
 * @brief 电机输出任务初始化
 */
void MotorTaskInitC(void);

/**
 * @brief 电机输出任务单步更新
 */
void MotorTaskUpdateC(void);


#ifdef __cplusplus
}
#endif

#endif // SRC_TASK_TASK_C_API_H




