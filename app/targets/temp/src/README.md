# balance_chassis_2026

基于 STM32 + FreeRTOS 的轮腿底盘代码，当前采用“四任务 + 消息队列”架构，将通信、状态机、控制计算和电机下发解耦，便于扩展。

## 1. 当前代码架构

核心源码位于 `src/`，按功能分层：

- `src/fsm/`：状态机逻辑（模式决策与控制意图输出）
- `src/chassis/`：状态估计、控制器与力矩合成
- `src/task/`：任务调度接口、消息定义、运行时上下文、任务实现
- `src/communication/`：通信模块（当前链路中预留）
- `Core/Src/freertos.c`：任务创建和循环调度入口

### 1.1 `src/task` 目录说明

`src/task` 已拆分为四层：

- `src/task/api/`
  - `task_c_api.h`：C 接口（给 `freertos.c` 调用）
  - `task_entrypoints.hpp`：C++ 任务入口声明
- `src/task/messages/`
  - `control_messages.hpp`：任务间消息结构体
- `src/task/runtime/`
  - `task_context.hpp`：全局任务上下文与队列句柄
  - `task_queue_runtime.cc`：队列初始化（`TaskQueuesInit()`）
- `src/task/workers/`
  - `comm_task.cc`：通信输入任务
  - `fsm_task.cc`：状态机任务
  - `torque_task.cc`：控制计算任务
  - `motor_output_task.cc`：电机输出任务

## 2. 任务职责与边界

### 2.1 CommTask

- 采集遥控输入（DR16）
- 生成 `CommInputMsg` 并发布

### 2.2 FsmTask

- 消费 `CommInputMsg`
- 调用 `Fsm::Update()` 做模式转移
- 输出 `FsmOutputMsg`（如 `enable_dm`、`run_chassis_update`、目标腿长等）

### 2.3 TorqueComputeTask

- 消费通信消息、状态机输出、电机反馈
- 调用 `Chassis::Update()`
- 生成 `TorqueCmd6`（六电机目标力矩）

### 2.4 MotorOutputTask

- 读取 `TorqueCmd6` 并下发 DM + 轮电机
- 处理超时/无效命令的安全回零
- 发布 `MotorFeedbackMsg` 供下一拍计算使用

## 3. 数据流向（详细）

### 3.1 队列拓扑

`TaskContext::Queues` 中定义了 5 条主队列：

- `comm_to_fsm`：`CommInputMsg`
- `comm_to_compute`：`CommInputMsg`
- `fsm_to_compute`：`FsmOutputMsg`
- `motor_to_compute`：`MotorFeedbackMsg`
- `compute_to_motor`：`TorqueCmd6`

### 3.2 主控制链路

```text
DR16输入
  -> CommTask
  -> CommInputMsg
  -> (comm_to_fsm, comm_to_compute)

comm_to_fsm
  -> FsmTask
  -> FsmOutputMsg
  -> fsm_to_compute

(comm_to_compute + fsm_to_compute + motor_to_compute + IMU)
  -> TorqueComputeTask
  -> TorqueCmd6
  -> compute_to_motor

compute_to_motor
  -> MotorOutputTask
  -> 电机下发
```

### 3.3 反馈闭环

```text
MotorOutputTask
  -> 读取电机反馈
  -> MotorFeedbackMsg
  -> motor_to_compute
  -> TorqueComputeTask（下一拍控制）
```

该闭环保证“执行反馈”参与下一周期控制计算。

### 3.4 队列策略

`QueuePutLatest()` 使用 latest-wins 语义：

- 队列满时弹出旧数据，写入最新数据
- 适用于状态流，避免旧包堆积导致控制滞后
- 未测试

## 4. 初始化与时序

启动时序（见 `Core/Src/freertos.c`）：

1. `TaskQueuesInit()` 创建任务队列
2. 创建四个业务线程
3. 每个线程先执行 `*TaskInitC()`，再进入 `*TaskUpdateC()` 周期循环

当前任务循环使用 `vTaskDelayUntil(&last, 2)`，即 2 ms 周期（500 Hz 基准节拍）。

## 5. 算法模块

- `src/fsm/`：只负责模式转移和控制意图，不直接操作硬件
- `src/chassis/state/`：状态估计
- `src/chassis/wbr/`：模型控制输出
- `src/chassis/chassis.*`：整合估计与控制，输出可执行力矩

## 6. 扩展建议

- 新增任务：放在 `src/task/workers/`，并在 `src/task/api/task_entrypoints.hpp` 声明
- 新增消息：优先扩展 `src/task/messages/control_messages.hpp`
- 新增通信源（云台/裁判系统）：在 `CommTask` 统一映射到 `CommInputMsg`

