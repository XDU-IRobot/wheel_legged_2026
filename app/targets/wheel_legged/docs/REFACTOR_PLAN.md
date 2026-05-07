# 轮腿机器人控制软件架构设计

## 设计目标

本固件架构围绕以下目标设计：

1. **可调试性** — 单周期内所有关键变量可通过调试器直接观察（DebugSnapshot, SRAM4）
2. **变体可维护** — 三款机器人共用控制框架，差异仅通过宏和参数隔离
3. **控制逻辑清晰** — 数据单向流动，各层职责明确，跨模块依赖最小化
4. **安全第一** — 任何通信中断/输入丢失时自动退回关闭状态，输出零力矩

---

## 分层架构

```
┌──────────────────────────────────────────────────────────┐
│                    应用层 (main.cc)                       │
│  AppMain(): 构造 SharedResources, 启动 500Hz 定时任务      │
├──────────────────────────────────────────────────────────┤
│                  编排层 (control_loop.cc)                  │
│  ControlLoop(): 9 阶段流水线, 串联所有子系统                │
├──────────┬──────────┬──────────┬──────────┬──────────────┤
│ 输入解析 │ 状态机层  │ 控制层   │ 执行器层  │ 调试层       │
│ input_   │ chassis/ │ chassis/ │ actuators│ debug_       │
│ resolver │ gimbal   │ gimbal/  │ .hpp     │ snapshot.hpp │
│ .cc      │ FSM      │ shoot    │          │ debug_export │
├──────────┴──────────┴──────────┴──────────┴──────────────┤
│                  硬件抽象层 (librm)                        │
│  DR16, DM_Motor, M3508, HipnucImu, CanDevice, PID        │
├──────────────────────────────────────────────────────────┤
│                  HAL 层 (STM32Cube)                       │
│  FDCAN, UART+DMA, TIM, GPIO                              │
└──────────────────────────────────────────────────────────┘
```

### 层间通信契约

每层之间通过**明确的结构体类型**传递数据，禁止跨层绕过：

| 从 | 到 | 结构体 |
|----|----|--------|
| 硬件 | 输入解析 | `Dr16RawInput`, `TcRemoteInput`, `ChassisStateEstimatorInput` |
| 输入解析 | 状态机 | `ChassisFsmInput`, `GimbalFsmInput` |
| 状态机 | 控制器 | `Fsm::Output::ControlOutput` (*2) |
| 状态机+传感器 | 云台控制 | `Gimbal::UpdateInput` |
| 输入解析+FSM+跨周期 | 底盘控制 | `Chassis::UpdateInput` |
| 控制器 | 执行器 | `Chassis::UpdateOutput`, `Gimbal::UpdateOutput`, `ShootOutput` |
| 全部 | 调试 | `DebugSnapshot` (SRAM4) |

---

## 核心设计决策

### 决策 1：ModeRequest -> ChassisFsmInput / GimbalFsmInput 拆分

**问题**: 最初使用单一的 `ModeRequest` 结构体传入两个状态机，导致底盘 FSM 能读到云台相关字段（rc_target 等），云台 FSM 能读到跳跃字段等。

**方案**: 在 `BuildChassisFsmInput()` 和 `BuildGimbalFsmInput()` 中将 `ModeRequest` 拆分为两个精简结构体：
- `ChassisFsmInput`: 仅含底盘决策所需字段（spin_hold, jump_trigger, leg_request, fall_detected 等）
- `GimbalFsmInput`: 仅含云台决策所需字段（rc_target, host_target, target_source, startup_align_complete 等）

**收益**: 接口清晰，每个 FSM 只能看到自己的业务字段；后续可彻底移除 `ModeRequest` 中间层。

---

### 决策 2：ShootOutput 解耦

**问题**: Shoot 控制器最初直接访问 `SharedResources` 中的电机对象并下发 CAN 命令。

**方案**: Shoot 只返回 `ShootOutput` 结构体（三个电流值），由 `Actuators::ApplyShootOutput()` 统一下发：
```cpp
ShootOutput { fric_left_current, fric_right_current, dial_current }
```

**收益**: 发射控制器与硬件解耦，可在无硬件环境单独测试控制逻辑。

---

### 决策 3：ChassisStateContext 跨周期状态封装

**问题**: 偏航跟随 PID、速率滤波、定点锁定 alpha、归中判稳计数器等作为 `static` 散落在 `ControlLoop()` 函数中，难以追踪和测试。

**方案**: 统一封装在 `ChassisStateContext` 中，包含：
- 滤波后的 s_dot / yaw_dot
- 定点锁定状态（target, alpha, s_ref, 去抖计数器）
- 偏航跟随目标选择与就绪判稳
- 云台启动归中状态
- 模式切换时的重置逻辑 `ResetOnModeChange()`

**收益**: 跨周期状态一目了然；模式切换重置规则集中在一处。

---

### 决策 4：DebugSnapshot 替换 volatile 全局变量

**问题**: 原代码有约 100 个 `volatile` 全局变量散布各处，占用内存、编译器无法优化、代码耦合严重。

**方案**: 定义 512 字节 packed struct `DebugSnapshot`，放置在 SRAM4 段：
- 调试器/DMA 可直接读取整块内存
- 主循环在每个周期末集中填充
- 不需要主动通过 UART/CAN 发送

**收益**: 零主循环开销的可观测性；512 字节限制保证 DMA 效率。

---

### 决策 5：参数命名空间多态

**问题**: 三款机器人参数不同，但控制代码相同。

**方案**: 参数在 `wheel_legged_params.hpp` 中按 namespace 组织：
```
params::common::control_loop::*  <- 共用参数
params::hero::*                  <- Hero 专属
params::infantry3::*             <- Infantry3 专属
params::infantry4::*             <- Infantry4 专属
params::active::*                <- 编译期 alias 到当前变体
```

所有控制代码只引用 `params::active::*`，变体切换时自动绑定到正确参数集。

**收益**: 零运行时开销的变体切换；新增变体只需添加 namespace 和 cmake 宏。

---

### 决策 6：Actuators 集中 IO 适配

**问题**: 电机 CAN 命令下发分散在多个文件中。

**方案**: `Actuators` 类承担三个职责：
1. **反馈采集** — `FillEstimatorInput()` 把 DM/M3508/IMU 反馈填入状态估计器输入
2. **命令下发** — `ApplyChassisOutput()`, `ApplyGimbalOutput()`, `ApplyShootOutput()`
3. **使能锁存** — 首次 enable 时发送 DM enable 指令（避免每周期重复发送）

**收益**: 更换硬件（如 DM 换 M3508）只需修改 Actuators；电机 IO 与控制器解耦。

---

### 决策 7：常量化控制参数

所有控制循环参数通过 `constexpr` 提取到函数顶部，禁止在逻辑代码中使用魔术数字。

```cpp
constexpr float kControlLoopDtS = ns::control_loop::kControlLoopDtS;
constexpr float kVxInputDeadbandNorm = ns::control_loop::kVxInputDeadbandNorm;
// ...
```

**收益**: 参数调整时可快速定位；编译期常量零运行时开销。

---

### 决策 8：基于 LQR 状态反馈的类 PI 纵向控制

**问题**: LQR 是全状态反馈调节器（本质为 PD 控制），没有积分项。单独使用 LQR 做速度控制时，摇杆归中后车体无法锚定位置，会随扰动漂移。传统方案是外加一个位置环 PID（I 项积分），但引入额外的调参和积分饱和问题。

**方案**: 不引入显式积分器，而是通过管理 LQR 期望状态中的 `expected_s`（期望位移）实现位置锚定：

- **正常行驶时**：`expected_s` 实时跟随 `current_s`，位移误差 `s - expected_s = 0`，LQR 仅调节速度。等效纯比例速度控制。
- **摇杆归中后**：`target_s_dot` 设为零，速度经斜坡衰减。`expected_s` 沿速度衰减路径积分（`expected_s += filtered_s_dot × dt`）。当 `filtered_s_dot` 到达零且车速静止时，`expected_s` 冻结为位置锚点。此时 `s - expected_s` 非零，LQR 同时调节位移和速度——等效 PI 控制。
- **重新推动摇杆**：`expected_s` 立刻重置为 `current_s`，位移误差归零，回到纯速度控制。

```
驾驶中:   expected_s = current_s    → 误差 = 0          → LQR = P（速度控制）
归中后:   expected_s 沿斜坡积分      → 误差逐步建立       → LQR = PD 过渡
停止后:   expected_s 冻结为锚点      → 误差持续 = 位置偏差 → LQR ≈ PI（位置锚定）
```

**收益**: 
- 零参数的"积分"效果：不需要位置环 PID 的 kp/ki/kd 调参
- 天然抗积分饱和：锚点在车实际停下的位置，不会累积
- LQR 一次解算所有通道（位移、速度、腿摆角、俯仰角等 10 维状态）的耦合控制，比独立的位置 PID + 速度 PID 更协调
- 自动适配不同腿长的制动特性（速度斜坡参数已按腿长分档）

**实现位置**: `control_loop.cc:338-355`（7i 阶段），详见 `docs/DATA_FLOW.md` 阶段 7。

---

## 状态机设计

### 底盘 12 状态层级

```
            ┌──────────┐
            │ kDisabled│ <- 输入失效/关闭
            └────┬─────┘
                 │ domain_request != kDisabled
         ┌───────┼───────┐
         v       v       v
    ┌────────┐┌────────┐┌────────┐
    │kLowLeg ││kMidLeg ││kHighLeg│ <- 腿长档位
    └───┬────┘└───┬────┘└───┬────┘
        │         │         │
   ┌────┼────┐    │    ┌────┤
   v    v    │    │    v    │
┌────┐┌────┐│    │ ┌──────┐│
│Jump││Spin││    │ │Stair ││
│    ││    ││    │ │Climb ││
└────┘└────┘│    │ └──────┘│
            │    │         │
            v    v         v
      ┌──────────────────────┐
      │ kRecoveryFallCheck   │ <- 倒地检测
      │ kRecoverySelfRight   │
      └──────────────────────┘
```

- **常态**: Low/Mid/High 三档腿长 (DR16 switch_r)
- **Spin**: 小陀螺，车体以恒定角速度旋转
- **Jump**: 三段式跳跃 (Prep->Push->Recover)
- **Stair**: 楼梯攀爬 (Climb->Done->HighLeg)
- **Recovery**: 倒地恢复 (FallCheck->SelfRight->LowLeg/Disabled)

### 云台 6 状态层级

```
┌──────────┐     ┌───────────────┐
│ kDisabled│---->│ kStartupAlign │ <- 首次使能必经归中
└──────────┘     └───────┬───────┘
                         │ align_complete
         ┌───────────────┼───────────────┐
         v               v               v
   ┌──────────┐   ┌──────────┐   ┌────────┐
   │ kService │   │ kService │   │kCombat │
   │ WithFire │   │ Safe     │   │        │
   └──────────┘   └──────────┘   └────────┘
         │               │               │
         └───────────────┼───────────────┘
                         │ chassis_recovery_active
                         v
                  ┌──────────────┐
                  │kRecoveryAlign│ <- 底盘恢复时对齐车头
                  └──────────────┘
```

---

## 安全设计

### 输入丢失 -> 零输出

- DR16 离线且无可信图传输入 -> `input_valid = false` -> FSM 立即退回 `kDisabled` -> 所有电机输出清零
- `chassis_output_enable` 级联条件：FSM enable + posture_valid + gimbal_startup_align_complete，任意不满足即零输出

### 姿态异常保护

- `chassis_control_output.posture_valid` 为 false 时：
  - 底盘输出 disable（DM 失能，轮毂零电流）
  - 云台输出 disable（防止异常姿态下发错误目标）

### 偏航就绪保护

- 偏航角未跟踪到位（`yaw_follow_drive_ready` = false）时，`target_s_dot = 0`，车体不纵向移动
- 防止云台朝向与车头方向不一致时车体误动

### 发射安全

- 非 Combat 域下 `shoot_enabled = false`（摩擦轮停转，拨盘停转）
- Hero: 只有 `enter_shoot`（进入 Combat 且 FSM 初始化完成）才使能摩擦轮和拨盘

---

## 未完成项 (@todo)

以下功能已预留接口，数据通路完整，但上游数据源尚未接入：

| 功能 | 当前状态 | 接入位置 |
|------|----------|----------|
| 上位机自瞄目标 | 已部分接入（aimbot CAN 通信已工作） | `input_resolver.cc:259-268` |
| 维护域策略细分 | 硬编码 kChassisAndGimbalSafe | `input_resolver.cc` — 需附加 DR16 通道 |
| 倒地自起 | 执行端已接入，触发端未实现 | `chassis.cc` 自起立力矩已就位；`input_resolver.cc` 中 `fall_detected` 恒 false，需接入 IMU 俯仰/横滚阈值判断 |
| 跳跃阶段控制 | 已接入 | `chassis.cc` — ComputeActuatorTorque() 三阶段跳跃力矩 |

---

## 关键不变量

- `ControlLoop()` 必须在 2ms 内完成（500Hz 定时器约束）
- `DebugSnapshot` <= 512 bytes（保证单次 DMA 传输）
- CAN 总线不直接在主循环发送；由 `DjiMotorBase::SendCommand()` 统一触发
- `SharedResources::Init()` 幂等：重复调用不重复构造（`std::optional::has_value()` 守卫）
- PID 的 `SetCircular` 在所有角度 PID 上启用，自动处理 +/-pi 回绕
