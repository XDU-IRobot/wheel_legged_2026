# 轮腿机器人嵌入式控制固件

## 系统概述

本项目是轮腿机器人嵌入式控制固件，运行于 STM32H7 平台。核心为 500Hz 主控制循环，负责底盘 LQR 控制、云台双轴随动、发射机构状态机与外设通信。

支持三种机器人变体，通过编译宏 `WHEEL_LEGGED_ROBOT_VARIANT` 切换：

| 宏值 | 机型 | 发射机构 | 主要差异 |
|------|------|----------|----------|
| 1 | Hero | 三摩擦轮 + DM 拨盘 | 3×M3508 PID + DM MIT 位置控制 |
| 2 | Infantry3 | 双摩擦轮 + M3508 拨盘 | Shoot2Fric 通用控制器 |
| 3 | Infantry4 | 双摩擦轮 + M3508 拨盘 | 参数独立，结构同 Infantry3 |

---

## 代码目录结构

```
app/targets/wheel_legged/
├── main.cc                          # 应用入口，启动 500Hz 调度
├── control.cc                       # 500Hz 主控制循环（核心编排）
├── chassis_fsm.cc                   # 底盘状态机实现（12 状态）
├── gimbal_fsm.cc                    # 云台状态机实现（6 状态）
├── chassis.cc                       # 底盘控制与估计器实现 (LQR + 运动学)
├── input.cc                         # 硬件输入采集 + DR16 语义折叠
├── state_ctx.cc                     # 驾驶输入、偏航跟随、定点锁定
├── debug.cc                         # 调试快照填充
├── shoot_2fric.cc                   # 双摩擦轮发射机构 (infantry3/4)
├── kalman.cc                        # 通用矩阵卡尔曼滤波器（CMSIS-DSP）
├── aimbot_can.cc                    # 自瞄 CAN 通信（TX:0x150, RX:0x170）
│
├── include/
│   ├── globals.hpp                  # SharedResources（所有外设/控制器实例）
│   ├── globals_no_dtcm.hpp          # 非 DTCM 外设（UART DMA 缓冲）
│   ├── fsm_common.hpp               # 共享枚举 + ModeRequest + FSM 输入结构体
│   ├── actuators.hpp                # Actuators 执行器适配层（反馈+下发）
│   ├── debug.hpp                    # DebugSnapshot 结构体定义（SRAM4，512B）+ 填充函数
│   ├── params.hpp                   # 全部参数（按变体分 namespace）
│   ├── input.hpp                    # 硬件输入采集、DR16/图传语义折叠与 FSM 输入构建
│   ├── state_ctx.hpp                # 底盘跨周期控制状态、偏航跟随、定点锁定与速率斜坡
│   ├── gimbal_can_bridge.hpp        # 云台→底盘 CAN 桥接收
│   ├── referee_rx.hpp               # 裁判系统串口桥
│   │
│   ├── chassis/
│   │   ├── fsm.hpp                  # 底盘状态机接口
│   │   ├── chassis.hpp              # 底盘控制器接口（LQR 主类）
│   │   ├── state.hpp                # 状态估计器 + 期望/当前状态向量
│   │   ├── leg.hpp                  # 五连杆腿部运动学解算
│   │   └── lqr.hpp                  # LQR 调节器（增益按腿长多项式插值）
│   │
│   ├── gimbal/
│   │   ├── fsm.hpp                  # 云台状态机接口
│   │   ├── gimbal.hpp               # 云台双轴 PID 控制器
│   │   ├── shoot_2fric.hpp          # Shoot 双摩擦轮控制器 (infantry)
│   │   └── shoot_3fric.hpp          # ShootController 三摩擦轮状态机 (hero)
│   │
│   └── utils/
│       ├── dyp_can.hpp              # DYP 超声波 CAN 接收桥
│       ├── kalman.hpp               # 通用矩阵卡尔曼滤波器（CMSIS-DSP）
│       └── aimbot_can.hpp           # 自瞄 CAN 通信（TX:0x150, RX:0x170）
│
├── docs/
│   ├── README.md                    # 本文档：上手阅读指南
│   ├── DATA_FLOW.md                 # 数据流详解
│   ├── REFACTOR_PLAN.md             # 架构设计说明
│   ├── CONTROL_LOOP_OPTIMIZATIONS.md # 控制器优化详解（位置锚定、斜坡、小陀螺）
│
├── 待办.md                          # 结构化待办列表（P0/P1/P2）
│
└── debug/
    ├── can.pmpx                     # CAN 总线调试配置
    └── debug.pmpx                   # 调试器配置
```

---

## 如何开始阅读这份代码

### 第一步：理解启动流程

1. **`main.cc`** — 应用入口，约 50 行。完成三件事：
   - 构造 `SharedResources` 并调用 `Init()`（初始化所有外设/CAN/电机/状态机）
   - 用 `TimerTaskScheduler` 将 `ControlLoop()` 绑定到 500Hz 硬件定时器
   - 主循环持续驱动三条 CAN 总线的 `Process()`（收发轮询）

### 第二步：理解数据流

2. **`docs/DATA_FLOW.md`** — 完整的 9 阶段数据流图。对照 `control.cc` 阅读。
3. **`control.cc`** — 核心编排文件（~500 行）。`ControlLoop()` 函数按 9 个阶段分步执行，每个阶段调用一个子模块。

### 第三步：理解类型系统

4. **`include/fsm_common.hpp`** — 所有共享枚举和 FSM 输入结构体：
   - `DomainRequest` / `ServiceProfile` / `LegProfile` / `CombatProfile` / `TargetSource` — 整车语义枚举
   - `ModeRequest` — 旧版统一语义请求（@deprecated，正向 ChassisFsmInput/GimbalFsmInput 迁移）
   - `ChassisFsmInput` / `GimbalFsmInput` — 新版分体 FSM 输入

### 第四步：理解硬件层

5. **`include/globals.hpp`** — `SharedResources` 结构体，包含所有 `std::optional<>` 外设成员和延迟构造逻辑。
6. **`include/actuators.hpp`** — `Actuators` 类，负责传感器反馈采集（`FillEstimatorInput`）和电机命令统一下发（`Apply*` 系列）。

### 第五步：深入各子系统（按需）

| 子系统 | 入口文件 | 关键类型 |
|--------|----------|----------|
| 底盘状态机 | `include/chassis/fsm.hpp` + `fsm.cc` | 12 状态，FSM::Input/Output |
| 底盘控制 | `include/chassis/chassis.hpp` + `chassis.cc` | LQR + 运动学 + 支撑力 |
| 云台状态机 | `include/gimbal/fsm.hpp` + `gimbal_fsm.cc` | 6 状态，启动归中 |
| 云台控制 | `include/gimbal/gimbal.hpp` | 双环 PID * 2 轴 |
| 发射机构 (hero) | `include/gimbal/shoot_3fric.hpp` | 5 状态拨盘 + 3 路 PID |
| 发射机构 (infantry) | `include/gimbal/shoot_2fric.hpp` + `shoot_2fric.cc` | Shoot2Fric 控制器 |
| 输入解析 | `include/input.hpp` + `input.cc` | DR16 + 图传语义折叠 |
| 跨周期状态 | `include/state_ctx.hpp` + `state_ctx.cc` | 偏航跟随 + 定点锁定 |
| 调试导出 | `include/debug.hpp` + `debug.cc` | SRAM4 512B 快照 |
| 参数系统 | `include/params.hpp` | `params::active` 变体切换 |

---

## 核心设计原则

### 1. 数据单向流动

硬件反馈 → 语义输入 → FSM 决策 → 控制器解算 → 执行器输出。中间层之间通过明确的结构体传递，不跨层直接访问硬件。

### 2. 执行器集中下发

所有电机 IO（DM MIT 命令 + M3508 电流）集中在 `Actuators` 类中。控制器只负责计算力矩/电流值，通过 `*Output` 结构体返回。

### 3. FSM 与控制器分离

状态机 (FSM) 负责"做什么"（enable_dm, spin_enable, target_leg_length），控制器负责"怎么做"（LQR 计算力矩, PID 跟踪目标）。FSM 输出结构体 (`ControlOutput`) 是二者的契约接口。

### 4. 变体隔离

- 共用逻辑写一次（`control.cc` 主函数、FSM、底盘/云台控制器）
- 差异部分用 `#if WHEEL_LEGGED_ROBOT_VARIANT` 守卫
- 参数按变体分 namespace：`params::hero` / `params::infantry3` / `params::infantry4`
- `params::active` 根据编译宏自动 alias 到当前变体

### 5. 跨周期状态独立封装

`ChassisStateContext` 结构体承载所有需跨周期保持的控制状态（偏航跟随 PID、速率滤波、定点锁定 alpha、归中状态等），避免在 `ControlLoop()` 中散落大量 `static` 变量。

---

## 关键参数位置

```cpp
// 变体切换
params::active::*                    // ← 所有控制参数从这里引用

// 参数分类
params::active::control_loop::*      // 控制循环 (dt, 死区, 斜坡速率, 定点锁定阈值)
params::active::chassis_fsm::*       // 底盘状态机 (腿长, 跳越/恢复定时)
params::active::chassis::*           // 底盘控制器 (LQR 矩阵, 腿运动学)
params::active::gimbal::*            // 云台控制器 (PID 增益, 力矩限幅)
params::active::shoot::*             // 发射机构 (摩擦轮/拨盘 PID, 射击频率)
params::active::actuators::*         // 执行器 (电流限幅, 力矩-电流转换)
params::active::globals::*           // 外设 (CAN 限频, 电机 ID/设置)
```

---

## 通用规则

- 底盘 DM 电机断开连接时, 输出零力矩（不判错, 直接设零）
- 非 combat 域下, 拨轮触发跳跃; combat 域下拨轮用于发射（不跳跃）
- 上位机自瞄/图传未接入时, 使用 RC 积分目标; 有上位机输入时优先上位机
- 所有 PID 使用 librm 标准接口, 圆形 PID 自动处理角度回绕
