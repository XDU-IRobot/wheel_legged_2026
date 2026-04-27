# `wheel_legged` 重构架构标准文档

本文档用于指导 `wheel_legged` 工程重构，是后续代码整理、模块拆分和状态机解耦的统一标准。

当前文档重点不是记录历史实现，而是明确重构后的目标架构、文件职责、控制链路、状态机边界和硬件约定。后续提交代码时，应尽量以本文档作为目录组织和模块职责划分依据。

---

## 1. 项目目标范围

项目名称统一改为：

```text
wheel_legged
```

当前目标仍然是单板整车控制：

- 底盘、云台、火控共用一套主控制循环；
- 遥控器接在底盘 MCU 本地；
- 底盘 IMU 接在底盘 MCU 本地串口；
- 云台 IMU 通过 CAN 反馈到底盘控制板；
- 上位机发送云台 `yaw / pitch` 目标命令也通过 CAN 反馈到底盘控制板；
- 上位机控制接口当前先预留，不要求立刻完成完整闭环；
- 控制周期固定为 **2 ms**。

本次重构的核心目标是：

1. 保留“底盘、云台、火控共用一套主控制循环”的整体设计；
2. 将底盘控制、云台控制、火控控制从一个复杂整车状态机中逐步解耦；
3. 将当前状态机简化，并拆分为底盘状态机和云台状态机；
4. 将外设原始数据、模型内部状态量、控制输出和硬件下发边界划清楚；
5. 为后续接入上位机目标、视觉目标、键鼠输入和更完整火控流程预留接口。

---

## 2. 总体设计原则

重构后的代码应遵守以下原则：

1. **主循环只做调度，不堆控制细节。**  
   `main.cc` 负责定时器和入口，`control_loop.cc` 负责每 2 ms 调用各模块 `update()`。

2. **模块内部封装自己的状态和算法。**  
   底盘控制细节放在 `chassis/`，云台和火控细节放在 `gimbal/`，滤波和工具函数放在 `utils/`。

3. **状态机分层解耦。**  
   底盘 FSM 只决定底盘工作模式；云台 FSM 只决定云台和火控模式。二者通过 `control_loop.cc` 中的统一输入和输出进行协调，不直接互相调用。

4. **外设原始值不能直接污染控制算法。**  
   电机、IMU、遥控器等原始值应先被整理成语义化输入或模型状态，再进入 LQR、PID、运动学和状态机。

5. **DMA 相关全局资源不得放在 DTCM。**  
   STM32H7 的 DTCM 仅 CPU 可访问，DMA 相关对象必须放在 DTCM 外的 RAM 区域中。

6. **控制周期固定为 2 ms。**  
   所有周期控制逻辑默认以 `dt = 0.002 s` 为基础，不应在不同模块中随意定义不同控制周期。

---

## 3. 目标文件架构

重构后的目录结构以当前规划为准：

```text
wheel_legged/
├── include/
│   ├── chassis/
│   │   ├── chassis_state.hpp
│   │   ├── fsm.hpp
│   │   ├── leg_kinematics.hpp
│   │   └── lqr_controllers.hpp
│   ├── gimbal/
│   │   ├── fsm.hpp
│   │   └── gimbal.hpp
│   ├── utils/
│   │   ├── kalman_filter.cc
│   │   └── kalman_filter.hpp
│   ├── globals.hpp
│   └── globals_no_dtcm.hpp
├── chassis.cc
├── control_loop.cc
└── main.cc
```

说明：

- `include/` 放模块对外头文件；
- `chassis.cc` 放底盘模块实现；
- `control_loop.cc` 放 2 ms 主控制链路；
- `main.cc` 放入口、初始化和定时器调度；
- `globals.hpp` 负责整合常规全局对象；
- `globals_no_dtcm.hpp` 专门放 DMA 相关、不能进入 DTCM 的全局对象。

---

## 4. 顶层文件职责

### 4.1 `main.cc`

`main.cc` 是工程入口文件，职责应尽量简单。

主要负责：

- 初始化底层硬件；
- 初始化 `Globals` 中的设备对象和控制模块；
- 配置并启动 2 ms 定时器任务调度器；
- 将定时器任务接入 `control_loop.cc` 中的主循环函数；
- 不直接编写复杂控制逻辑；
- 不直接写底盘、云台、火控算法细节。

推荐职责边界：

```text
main.cc
  -> system init
  -> globals init
  -> timer scheduler init
  -> register ControlLoop()
  -> start scheduler
```

`main.cc` 中不建议出现大量状态机判断、LQR 解算、PID 解算或 CAN 命令拼装代码。

---

### 4.2 `control_loop.cc`

`control_loop.cc` 是整车 2 ms 控制链路的核心调度文件。

主要负责：

- 按固定顺序读取或刷新输入；
- 调用底盘状态机 `chassis::Fsm`；
- 调用云台状态机 `gimbal::Fsm`；
- 调用底盘控制模块 `Chassis::Update()`；
- 调用云台控制模块 `Gimbal::Update()`；
- 汇总底盘、云台、火控输出；
- 调用硬件对象完成命令下发；
- 刷新调试数据。

`control_loop.cc` 的定位是“控制流程编排层”，而不是“算法实现层”。

推荐单拍流程：

```text
ControlLoop, 2 ms
  -> update raw feedback
  -> build input snapshot
  -> update chassis FSM
  -> update gimbal FSM
  -> update chassis controller
  -> update gimbal controller and fire controller
  -> apply actuator command
  -> update debug snapshot
```

更具体的链路可以理解为：

```text
Motor / CAN / UART feedback
  -> input snapshot
  -> chassis_state update
  -> chassis FSM update
  -> gimbal FSM update
  -> chassis controller update
  -> gimbal controller update
  -> actuator command output
```

---

### 4.3 `globals.hpp`

`globals.hpp` 用于整合普通全局对象。

主要放置：

- 电机对象；
- CAN 通讯对象；
- 串口遥控器对象；
- 底盘 IMU 串口对象；
- 上位机通讯对象；
- 云台 IMU CAN 通讯对象；
- 底盘控制模块对象；
- 云台控制模块对象；
- 状态机对象；
- 调试数据对象。

`globals.hpp` 的目标是让 `main.cc` 和 `control_loop.cc` 不需要到处声明零散全局变量，而是通过统一的全局对象集合访问设备和模块。

建议组织形式：

```cpp
struct Globals {
  // communication devices
  // sensors
  // motors
  // chassis module
  // gimbal module
  // debug data
};

extern Globals g;
```

注意：普通 CPU 访问对象可以放在这里，但 DMA buffer、DMA 接收结构、部分 CAN/UART DMA 相关内存不应放在这里，应放入 `globals_no_dtcm.hpp`。

---

### 4.4 `globals_no_dtcm.hpp`

`globals_no_dtcm.hpp` 专门放置 DMA 相关全局资源。

原因：

STM32H7 的 DTCM 仅 CPU 可访问，DMA 无法访问 DTCM。如果将 UART DMA、CAN DMA 或其他 DMA 相关 buffer 放入 DTCM，可能出现：

- DMA 接收不到数据；
- 数据偶发错误；
- CPU 看到的数据不更新；
- 调试时表现为外设正常但内存无变化。

因此 DMA 相关对象必须放在 DTCM 外，例如 `RAM_D1 / .sram4`。

linker script 已提供：

```ld
.sram4 (NOLOAD) : { *(.sram4) } >RAM_D1
```

建议在 `globals_no_dtcm.hpp` 中集中声明类似资源：

```cpp
// Example only
extern uint8_t rc_dma_buffer[];
extern uint8_t imu_dma_buffer[];
extern uint8_t can_rx_buffer[];
```

并在对应定义处显式放入 `.sram4`：

```cpp
__attribute__((section(".sram4"))) uint8_t rc_dma_buffer[...];
```

原则：

- DMA 会读写的 buffer 放在 `globals_no_dtcm.hpp` 管理；
- 普通控制对象放在 `globals.hpp` 管理；
- 不要为了方便把所有全局对象都塞进 `.sram4`；
- 不要让 DMA buffer 随机分散在各个 `.cc` 文件中。

---

## 5. `chassis/` 文件夹职责

`include/chassis/` 主要封装底盘相关模型、状态机、运动学和 LQR 控制器。

### 5.1 `include/chassis/chassis_state.hpp`

`chassis_state.hpp` 负责将外设原始值转换成底盘控制模型需要的内部状态量。

输入来源包括：

- 关节电机原始反馈；
- 轮毂电机原始反馈；
- 底盘 IMU 姿态、角速度、加速度；
- 遥控器或上位机给出的速度/姿态目标；
- 必要时的云台反馈量。

输出应是底盘控制算法可以直接使用的模型状态，例如：

- 机体俯仰角；
- 机体俯仰角速度；
- 轮系位置或等效位移；
- 轮系速度；
- 腿部角度；
- 腿部角速度；
- 当前腿长；
- 左右腿几何量；
- LQR 所需状态向量。

这里还应包含速度融合逻辑，例如：

- 轮速观测；
- IMU 加速度积分观测；
- 卡尔曼滤波融合；
- 对异常加速度或轮速打滑情况做必要抑制。

职责边界：

- 可以做状态估计；
- 可以做单位换算和坐标转换；
- 可以做速度融合；
- 不应直接下发电机电流；
- 不应直接决定整车工作模式。

---

### 5.2 `include/chassis/fsm.hpp`

`chassis/fsm.hpp` 是底盘状态机。

它只负责底盘工作模式，不负责云台模式，也不负责火控节拍。

建议底盘 FSM 先保持简单，后续根据调试需要再扩展。推荐初始状态集合：

```text
ChassisFsm
├── Disabled          // 底盘失能
├── Standby           // 底盘待机，电机可保持安全输出
├── Balance           // 正常平衡/行走
├── Spin              // 小陀螺/旋转模式
├── JumpPrep          // 跳跃准备
├── JumpPush          // 跳跃蹬伸
├── JumpRecover       // 跳跃恢复
├── RecoveryFallCheck // 倒地检测/恢复判断
└── RecoverySelfRight // 自恢复动作
```

底盘 FSM 输入建议包括：

- 遥控器底盘使能请求；
- 腿长档位请求；
- 速度指令；
- spin 请求；
- jump 触发；
- 倒地检测结果；
- 直立稳定判据；
- 电机/IMU 是否在线。

底盘 FSM 输出建议包括：

- `chassis_enable`；
- `target_leg_length`；
- `target_vx`；
- `target_vy`；
- `target_yaw_rate`；
- `spin_enable`；
- `jump_phase`；
- `recovery_enable`；
- `safe_output_required`。

底盘 FSM 不应关心：

- 云台 yaw/pitch 跟踪细节；
- 摩擦轮是否启动；
- 拨盘什么时候单发；
- 上位机目标角具体如何滤波。

当前迁移阶段补充说明（与 `app/targets/temp/src/fsm` 语义对齐）：

- 虽然状态命名已切换为 `Disabled/Balance/Spin/Jump*`，但迁移逻辑保持与旧 FSM 一致：
  - `force_enable=false` 回退到底盘失能态；
  - `jump_trigger` 采用上升沿触发，驱动 `JumpPrep -> JumpPush -> JumpRecover`；
  - 非跳跃阶段由 `spin_enable` 决定 `Balance/Spin`。
- 旧语义与新命名可按下表理解：

| temp/fsm 旧语义 | wheel_legged 当前命名 |
|---|---|
| `kNoForce` | `kDisabled` |
| `kNormalLowLeg / Mid / High` | `kBalance` + `LegLengthMode::kLow / kMid / kHigh` |
| `kSmallGyro` | `kSpin` |
| `kJumpRetract1 / Extend / Retract2` | `kJumpPrep / kJumpPush / kJumpRecover` |

- `RecoveryFallCheck / RecoverySelfRight` 当前仍作为预留状态保留在接口中，但主迁移链路不主动进入，避免偏离 `temp/fsm` 行为。

---

### 5.3 `include/chassis/leg_kinematics.hpp`

`leg_kinematics.hpp` 放腿部运动学模型。

主要负责：

- 根据关节角计算腿长；
- 根据关节角计算腿部姿态角；
- 计算雅可比矩阵；
- 将虚拟力或虚拟力矩映射到关节力矩；
- 为 VMC、LQR 或其他底盘控制器提供几何接口。

如果后续更换腿部连杆构型，例如由五连杆改为“髋关节电机 + 四连杆膝关节驱动”，应优先修改本文件中的运动学和雅可比计算，而不是在 `chassis.cc` 中写临时几何公式。

---

### 5.4 `include/chassis/lqr_controllers.hpp`

`lqr_controllers.hpp` 放底盘动力学和 LQR 控制器相关内容。

主要负责：

- 定义 LQR 状态向量；
- 定义控制输入向量；
- 保存离线计算得到的 LQR 增益；
- 根据当前腿长选择或拟合对应增益；
- 根据当前状态和目标状态计算控制输出。

推荐职责边界：

- LQR 增益表、拟合多项式和控制律放在这里；
- 底盘状态估计不要放在这里；
- 电机对象不要放在这里；
- CAN 下发不要放在这里。

---

### 5.5 `chassis.cc`

`chassis.cc` 是底盘模块实现文件。

主要负责：

- 接收 `chassis/fsm.hpp` 输出的底盘目标模式；
- 调用 `chassis_state.hpp` 得到模型内部状态量；
- 调用 `leg_kinematics.hpp` 完成腿部几何解算；
- 调用 `lqr_controllers.hpp` 完成 LQR 或等效底盘控制解算；
- 合成左右腿关节力矩和左右轮力矩；
- 输出一个统一的 `ChassisOutput` 给 `control_loop.cc` 调用。

`chassis.cc` 应向外提供类似接口：

```cpp
class Chassis {
 public:
  void Init();
  ChassisOutput Update(const ChassisInput& input);
};
```

`control_loop.cc` 只应调用 `Chassis::Update()`，不应直接进入底盘内部函数拼接控制量。

---

## 6. `gimbal/` 文件夹职责

`include/gimbal/` 主要封装云台状态机、云台姿态控制和火控逻辑。

### 6.1 `include/gimbal/fsm.hpp`

`gimbal/fsm.hpp` 是云台状态机。

它只负责云台和火控相关模式，不负责底盘平衡、腿长、跳跃和倒地恢复。

建议云台 FSM 先保持简单，推荐初始状态集合：

```text
GimbalFsm
├── Disabled       // 云台失能
├── Safe           // 云台安全态，火控关闭
├── ManualControl  // 遥控器控制 yaw/pitch
├── HostControl    // 上位机/视觉目标控制 yaw/pitch，当前先预留
└── RecoveryAlign  // 底盘恢复时云台对正到底盘前向
```

云台 FSM 输入建议包括：

- 遥控器云台使能请求；
- 上位机目标是否有效；
- 遥控器 yaw/pitch 目标；
- 上位机 yaw/pitch 目标；
- 底盘是否处于恢复态；
- 是否允许火控；
- 当前是否有射击请求。

云台 FSM 输出建议包括：

- `gimbal_enable`；
- `target_source`，例如 `Remote` 或 `Host`；
- `target_yaw`；
- `target_pitch`；
- `align_to_chassis_forward`；
- `fire_allowed`；
- `shoot_request`。

云台 FSM 不应关心：

- 腿长档位；
- LQR 状态量；
- 底盘轮速融合；
- 跳跃力矩细节。

---

### 6.2 火控子状态机建议

火控建议作为云台模块内部的子状态机，而不是继续塞进整车主状态机。

初始可保持三态：

```text
FireFsm
├── Safe        // 火控关闭
├── Armed       // 摩擦轮开启，等待射击请求
└── SingleShot  // 单发拨盘输出一拍或一个短节拍
```

后续可以扩展为：

```text
FireFsm
├── Safe
├── SpinUp
├── Ready
├── SingleShot
├── Burst
└── Cooldown
```

当前阶段建议先实现简单版本，避免状态机过早复杂化。

---

### 6.3 `include/gimbal/gimbal.hpp`

`gimbal.hpp` 定义云台模块对外接口和数据结构。

主要包括：

- 云台输入结构；
- 云台输出结构；
- yaw 控制器；
- pitch 控制器；
- 火控输出；
- 云台 IMU 反馈；
- 上位机目标接口预留。

建议输出内容包括：

- yaw 电机控制量；
- pitch 电机控制量；
- 左右摩擦轮控制量；
- 拨盘控制量；
- 当前云台状态；
- 当前火控状态。

---

## 7. `utils/` 文件夹职责

`include/utils/` 存放通用工具。

### 7.1 `kalman_filter.hpp` / `kalman_filter.cc`

卡尔曼滤波器实现放在这里。

当前主要服务于底盘速度融合，例如：

- 轮速观测；
- IMU 加速度观测；
- 速度估计；
- 低频漂移抑制；
- 高频噪声抑制。

后续如果云台或其他模块也需要滤波器，应优先复用 `utils/` 中的通用实现，而不是在各模块中复制一份滤波代码。

### 7.2 后续可加入的工具

后续可以继续加入：

- 一阶低通滤波器；
- 斜坡函数；
- 限幅函数；
- 死区函数；
- 角度归一化函数；
- PID 通用模板；
- 坐标变换工具。

原则是：只有多个模块都会用到的通用代码才放入 `utils/`。如果只服务于底盘，则放 `chassis/`；只服务于云台，则放 `gimbal/`。

---

## 8. 状态机解耦方案

当前重构方向是将原先复杂的整车状态机拆成两个相对独立的状态机：

```text
control_loop.cc
├── chassis::Fsm
└── gimbal::Fsm
```

### 8.1 解耦前的问题

如果所有逻辑都塞进一个整车状态机，容易出现：

- 底盘腿长、跳跃、恢复和火控模式混在一起；
- 云台是否允许射击影响到底盘状态命名；
- 状态数量快速膨胀；
- 新增上位机目标或视觉目标时要修改大量叶子状态；
- 调试时很难判断问题属于底盘、云台还是火控。

### 8.2 解耦后的边界

解耦后建议边界如下：

| 模块 | 负责 | 不负责 |
|---|---|---|
| `chassis::Fsm` | 底盘使能、腿长、平衡、Spin、Jump、Recovery | 云台 yaw/pitch、火控节拍 |
| `gimbal::Fsm` | 云台使能、目标源选择、恢复对正、火控许可 | LQR、腿长、轮速融合 |
| `control_loop.cc` | 调度两个 FSM，传递必要共享信息 | 不写具体算法 |
| `chassis.cc` | 底盘状态估计和控制解算 | 不判断火控状态 |
| `gimbal.hpp/.cc` | 云台控制和火控输出 | 不改底盘状态 |

### 8.3 两个 FSM 的信息交互

两个 FSM 不建议直接互相持有指针或互相调用。

推荐通过 `control_loop.cc` 传递有限共享信息：

```text
InputSnapshot
  -> chassis::Fsm::Input
  -> chassis::Fsm::Output
  -> gimbal::Fsm::Input
  -> gimbal::Fsm::Output
```

例如：

- 底盘处于 `RecoverySelfRight` 时，`control_loop.cc` 可以给云台 FSM 一个 `chassis_recovery_active = true`；
- 云台 FSM 决定进入 `RecoveryAlign`，让云台对正到底盘前向；
- 云台是否允许火控由云台 FSM 管理，不反向改变底盘状态。

这样可以保证：

- 底盘问题主要查 `chassis/`；
- 云台/火控问题主要查 `gimbal/`；
- 两者耦合点集中在 `control_loop.cc`，便于调试。

---

## 9. 输入与目标源约定

### 9.1 本地输入

本地输入包括：

- 遥控器；
- 底盘 IMU；
- 底盘电机反馈；
- 云台电机反馈；
- 云台 IMU CAN 反馈。

遥控器原始通道值不建议直接进入状态机。应先转换为语义化请求，例如：

- 底盘使能请求；
- 云台使能请求；
- 腿长档位请求；
- spin 请求；
- jump 请求；
- fire 请求；
- `vx / vy / yaw_rate` 指令；
- `yaw / pitch` 目标增量或目标角。

### 9.2 上位机目标源预留

当前预留上位机通过 CAN 发送：

- `target_yaw`；
- `target_pitch`；
- `target_valid`；
- 后续可扩展目标距离、目标速度、是否开火等字段。

当前只要求接口预留，不要求完整实现。

建议在云台 FSM 中保留目标源枚举：

```text
TargetSource
├── Remote
└── Host
```

当 `host_target_valid == true` 且当前模式允许上位机控制时，云台 FSM 可以选择 `Host` 作为目标源；否则回到 `Remote`。

### 9.3 DR16 语义化映射（当前实现约定）

当前 `wheel_legged` 使用 DR16 左拨杆、右拨杆与拨轮，先做“语义化”再喂给 FSM。
当前已接入真实 `rm::device::DR16`（`globals_no_dtcm.rc_uart`），在线判据基于 `DR16::online_status()==kOk`。

#### 9.3.1 左拨杆（`switch_l`）

- `kDown`：底盘失能请求（`chassis_enable_request=false`），云台失能请求（`gimbal_enable_request=false`）。
- `kMid`：底盘使能（允许进入 `Balance/Spin/Jump`），云台使能。
- `kUp`：底盘使能，云台使能。

说明：左拨杆作为整车总使能门控。

为了与 `temp/fsm` 语义保持一致，`switch_l` 仅在 `kMid/kUp` 时认为是“使能请求”，`kDown/kUnknown` 均按失能处理。

#### 9.3.2 右拨杆（`switch_r`）

右拨杆一杆双义：同时决定底盘腿长档位与云台工作档位。

- 对底盘腿长请求：
  - `kDown` -> `LegLengthMode::kLow`
  - `kMid` -> `LegLengthMode::kMid`
  - `kUp` -> `LegLengthMode::kHigh`
- 对云台状态请求：
  - `kDown` -> `gimbal_safe_request=true`（对应 `Safe`）
  - `kMid` -> 普通遥控模式（`ManualControl`）
  - `kUp` -> 上位机目标模式请求（当且仅当 `gimbal_host_target_external_valid=true` 时进入 `HostControl`）

#### 9.3.3 拨轮（`dial`）

拨轮用于 `jump` 与 `spin`，采用阈值 + 滞回 + 上升沿触发：

- `jump`（单次触发）：
  - 触发阈值：`dial >= +420`
  - 释放阈值：`dial <= +260`
  - 只有从“未锁存”到“触发阈值以上”的上升沿产生一次 `chassis_jump_request=true`
- `spin`（保持请求）：
  - 使能阈值：`dial <= -360`
  - 解除阈值：`dial >= -220`
  - 在滞回区间内保持上次状态，输出 `chassis_spin_request`

#### 9.3.4 安全与优先级

- DR16 不在线或输入无效：`input_valid=false`，FSM 回退到失能/安全态。
- 当左拨杆为 `kDown` 时，`jump/spin` 请求被门控为无效。
- `gimbal_host_target_valid` 由两部分共同决定：
  - 右拨杆在 `kUp`；
  - 上位机目标有效（`gimbal_host_target_external_valid=true`）。

该约定的目标是让 `control_loop.cc` 只做“输入语义化 + FSM 编排”，不直接混入底盘/云台控制算法细节。

其中底盘语义信号与旧 `temp/fsm` 输入字段的对应关系如下：

| DR16 语义化后字段 | temp/fsm 输入语义 |
|---|---|
| `chassis_enable_request` | `force_enable` |
| `chassis_leg_length_mode` | `leg_length_mode` |
| `chassis_spin_request` | `small_gyro_enable` |
| `chassis_jump_request`（上升沿） | `jump_trigger` |

---

## 10. 输出约定

### 10.1 底盘输出

底盘模块最终输出应包括：

- 左前/左后或左腿关节力矩；
- 右前/右后或右腿关节力矩；
- 左轮力矩或电流；
- 右轮力矩或电流；
- 当前估计状态；
- 当前腿长；
- 当前底盘 FSM 状态；
- 调试量。

### 10.2 云台与火控输出

云台模块最终输出应包括：

- yaw 电机力矩/电流；
- pitch 电机力矩/电流；
- 摩擦轮电流；
- 拨盘电流；
- 当前云台 FSM 状态；
- 当前火控状态；
- 当前目标源；
- yaw/pitch 目标角和反馈角。

### 10.3 硬件下发

硬件下发建议统一放在 `control_loop.cc` 的末尾或由 `Globals` 中的硬件封装对象完成。

原则：

- 控制模块只输出“想要的控制量”；
- 硬件封装层负责将控制量映射到具体 CAN ID、电机类型和发送函数；
- 不要在 LQR 或 PID 内部直接调用 CAN 发送。

---

## 11. 2 ms 控制周期约定

控制周期固定为：

```text
dt = 0.002 s
frequency = 500 Hz
```

要求：

- 定时器调度周期为 2 ms；
- `control_loop.cc` 每 2 ms 执行一次；
- LQR、速度估计、PID、状态机计时均按 2 ms 推进；
- 如果某些低频任务不需要 500 Hz，应在主循环中做分频，不应改变主控制周期。

例如：

```text
2 ms task:
  - chassis control
  - gimbal control
  - motor command output

10 ms sub-task:
  - low frequency debug update
  - slow host status report
```

---

## 12. 硬件与总线约定

硬件约定沿用当前文档中的设置。

### 12.1 底盘侧

- DR16 遥控器：本地串口；
- 底盘 IMU：本地串口；
- 四个关节 DM 电机：`hfdcan1`；
- 两个轮毂 M3508 电机：`hfdcan2`。

### 12.2 云台侧

- pitch DM 电机：`hfdcan3`；
- yaw DM 电机：`hfdcan2`；
- 左右摩擦轮：`hfdcan3`；
- 拨盘电机：`hfdcan2`；
- 云台 IMU 反馈帧：`hfdcan3`。

### 12.3 上位机通信

上位机发送云台目标命令通过 CAN 反馈到底盘控制板。

当前先预留接口，建议字段包括：

- `target_yaw`；
- `target_pitch`；
- `target_valid`；
- `timestamp` 或帧计数；
- 后续可选 `fire_request`。

具体 CAN 通道和 CAN ID 后续可根据硬件接线与现有 ID 分配进一步确定。

---

## 13. 推荐重构步骤

为了降低一次性重构风险，建议按以下顺序推进：

### 第一步：改名和目录整理

- 将项目统一命名为 `wheel_legged`；
- 按本文档建立目录；
- 将旧文件逐步移动到目标结构中；
- 保证工程能编译通过。

### 第二步：抽出 `globals.hpp` 和 `globals_no_dtcm.hpp`

- 普通全局对象进入 `globals.hpp`；
- DMA 相关 buffer 进入 `globals_no_dtcm.hpp`；
- 检查 linker script 中 `.sram4` 是否生效；
- 确认遥控器、IMU、CAN 接收数据正常。

### 第三步：整理 `control_loop.cc`

- 固定 2 ms 调用顺序；
- 将主循环改成调用各模块 `update()`；
- 删除主循环中过多的临时控制逻辑。

### 第四步：拆分底盘 FSM

- 新建 `include/chassis/fsm.hpp`；
- 只保留底盘相关状态；
- 将腿长、Spin、Jump、Recovery 放入底盘 FSM；
- 输出底盘控制目标给 `chassis.cc`。

### 第五步：拆分云台 FSM

- 新建 `include/gimbal/fsm.hpp`；
- 将云台使能、目标源选择、恢复对正、火控许可放入云台 FSM；
- 将火控节拍留在云台模块内部。

### 第六步：整理底盘状态估计

- 将电机和 IMU 原始值转换逻辑放入 `chassis_state.hpp`；
- 将速度融合逻辑统一到该层或调用 `utils/kalman_filter`；
- 保证 LQR 输入状态来源清晰。

### 第七步：预留上位机目标接口

- 在全局通讯对象中预留上位机 CAN 接收；
- 在云台输入中加入 `host_target`；
- 在云台 FSM 中保留 `HostControl` 或 `TargetSource::Host`；
- 当前可以先不启用完整上位机控制闭环。

---

## 14. 代码维护边界

后续修改时建议遵守以下边界：

| 修改内容 | 优先修改位置 |
|---|---|
| 定时器周期、主循环入口 | `main.cc` |
| 控制流程调用顺序 | `control_loop.cc` |
| 全局设备对象 | `globals.hpp` |
| DMA buffer | `globals_no_dtcm.hpp` |
| 底盘状态机 | `include/chassis/fsm.hpp` |
| 底盘模型状态估计 | `include/chassis/chassis_state.hpp` |
| 腿部运动学 | `include/chassis/leg_kinematics.hpp` |
| LQR 控制器 | `include/chassis/lqr_controllers.hpp` |
| 底盘控制实现 | `chassis.cc` |
| 云台状态机 | `include/gimbal/fsm.hpp` |
| 云台和火控控制 | `include/gimbal/gimbal.hpp` 及对应实现 |
| 卡尔曼滤波等通用工具 | `include/utils/` |

---

## 15. 当前总结

`wheel_legged` 重构后的核心架构可以概括为：

```text
main.cc
  -> 2 ms timer scheduler
  -> control_loop.cc
      -> chassis::Fsm
      -> gimbal::Fsm
      -> Chassis::Update()
      -> Gimbal::Update()
      -> actuator command output
```

其中：

- `main.cc` 只负责初始化和定时调度；
- `control_loop.cc` 只负责编排单拍控制流程；
- `globals.hpp` 统一管理普通全局对象；
- `globals_no_dtcm.hpp` 统一管理 DMA 安全内存；
- `chassis/` 负责底盘状态估计、运动学、LQR 和底盘 FSM；
- `gimbal/` 负责云台状态机、yaw/pitch 控制和火控；
- `utils/` 负责卡尔曼滤波器等通用工具。

本次重构的重点不是一次性实现所有功能，而是先建立清晰的模块边界。只要边界稳定，后续无论继续扩展上位机目标、视觉控制、键鼠输入、复杂火控流程，还是更换腿部连杆构型，都可以在对应模块内局部修改，而不需要重新推翻整车架构。
