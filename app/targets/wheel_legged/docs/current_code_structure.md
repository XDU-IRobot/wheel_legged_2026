# wheel_legged 当前代码结构与重构建议

本文基于 `app/targets/wheel_legged` 当前源码整理，重点说明真实代码结构、控制数据流、状态机转换关系，以及后续重构建议。

## 1. 总体结构

`wheel_legged` 目标目前是一个 500 Hz 单主循环控制程序。主入口在 `main.cc`，硬件对象、控制器、状态机集中保存在 `SharedResources`，主控制逻辑集中在 `control.cc::ControlLoop()`。

核心目录如下：

```text
app/targets/wheel_legged
|-- main.cc                         # AppMain，初始化资源，注册 500 Hz ControlLoop
|-- control.cc                      # 主控制循环，当前最复杂的调度与业务融合点
|-- input.cc                        # DR16/图传键鼠/自瞄反馈到语义请求的折叠层
|-- chassis_fsm.cc                  # 底盘模式状态机
|-- gimbal_fsm.cc                   # 云台模式状态机
|-- chassis.cc                      # 底盘控制器，状态估计、LQR、腿长/摆角/轮力矩输出
|-- state_ctx.cc                    # 控制环跨周期状态，yaw-follow、速度斜坡、位置保持等
|-- stair_task_coordinator.cc       # 台阶任务协调器，负责 Arm/Executing/BetweenSteps 等任务状态
|-- stair_climb_sequence.cc         # 单次台阶动作序列，负责 Hook/Retract/Settle
|-- shoot_2fric.cc                  # 非 hero 版本双摩擦轮发射机构实现
|-- aimbot_can.cc                   # 自瞄 CAN 通信编码
|-- debug.cc                        # 调试快照导出
|-- include
|   |-- globals.hpp                 # SharedResources，硬件与控制对象总容器
|   |-- actuators.hpp               # 执行器适配层，反馈采集和电机命令下发
|   |-- params.hpp                  # 所有变体参数，按 active 命名空间选择
|   |-- input.hpp                   # 输入快照、语义状态、FSM 输入构造接口
|   |-- fsm_common.hpp              # Domain/Leg/Combat/Stair/GimbalTarget 等共享语义
|   |-- state_ctx.hpp               # ControlLoop 跨周期上下文
|   |-- chassis/
|   |   |-- fsm.hpp                 # 底盘 FSM 接口和状态定义
|   |   |-- chassis.hpp             # 底盘控制器接口
|   |   |-- state.hpp               # 底盘状态估计器
|   |   |-- lqr.hpp                 # LQR 状态、控制器和系数
|   |   |-- leg.hpp                 # 五连杆运动学
|   |   |-- stair_task_coordinator.hpp
|   |   |-- stair_climb_sequence.hpp
|   |-- gimbal/
|   |   |-- fsm.hpp                 # 云台 FSM 接口和状态定义
|   |   |-- gimbal.hpp              # 云台双轴控制器
|   |   |-- gimbal_ident.hpp        # 云台辨识/前馈验证
|   |   |-- shoot_2fric.hpp
|   |   |-- shoot_3fric.hpp
|   |-- utils/
|-- ui/                             # 裁判 UI、UI 快照和调度
```

### 1.1 关键对象关系

`SharedResources` 是全局资源容器，包含：

| 类别 | 主要成员 | 职责 |
|---|---|---|
| 通信与传感器 | `dr16`、`joint_can`、`wheel_can`、`gimbal_can`、`chassis_imu`、`gimbal_rx`、`aimbot`、`referee`、`supercap`、`dyp_left/right` | 接收遥控器、惯导、图传桥、自瞄、裁判、超声波和 CAN 设备 |
| 执行器 | `dm_lf/lb/rf/rb`、`left_wheel/right_wheel`、`yaw_motor/pitch_motor`、发射电机 | 最终被 `Actuators` 写入命令 |
| 控制与状态机 | `chassis_fsm`、`chassis`、`gimbal_fsm`、`gimbal`、`gimbal_ident`、`shoot`/`shoot_controller` | 模式决策和控制计算 |

`Actuators` 是硬件适配层：

- `FillEstimatorInput()` 从 DM 关节、电机轮速、底盘 IMU、yaw 电机编码器填充 `ChassisStateEstimatorInput`。
- `ApplyChassisOutput()` 把 `Chassis::UpdateOutput` 的关节力矩和轮端力矩转换成 DM MIT 命令和 M3508 电流。
- `ApplyGimbalOutput()` 把 `Gimbal::UpdateOutput` 的 yaw/pitch 力矩下发到云台 DM。
- `ApplyShootOutput()` 下发双摩擦轮和拨盘电流。

## 2. 主控制数据流

`AppMain()` 中执行：

1. `globals = &g_globals`
2. `globals->Init()`
3. 使用 `TimerTaskScheduler` 将 `ControlLoop()` 注册到 500 Hz。
4. 主 `for(;;)` 循环只负责 `joint_can/wheel_can/gimbal_can.Process()`。

所以业务控制实际都在 `ControlLoop()` 内。当前代码注释中把主循环分成 9 个阶段，实际数据流如下：

```text
硬件反馈/遥控/图传/自瞄/裁判
        |
        v
UpdateRawFeedbackAndInputSnapshot()
        |
        v
InputSnapshot + ModeRequest
        |
        +--> StairTaskCoordinator + StairClimbSequence
        |
        +--> BuildChassisFsmInput() --> chassis_fsm.Update()
        |
        +--> BuildGimbalFsmInput()  --> gimbal_fsm.Update()
        |
        v
Gimbal::UpdateInput ------------------> Gimbal::Update() --> Actuators::ApplyGimbalOutput()
        |
        v
Shoot / Supercap / Ident UART
        |
        v
Chassis::UpdateInput
        |
        +--> yaw-follow、速度斜坡、位置保持、期望状态生成
        |
        v
Chassis::Update() --> Actuators::ApplyChassisOutput()
        |
        v
Aimbot TX + DebugSnapshot + UI Snapshot
```

### 2.1 输入采集和语义折叠

入口：`UpdateRawFeedbackAndInputSnapshot()`。

它做了几类事情：

1. 调用 `Actuators::FillEstimatorInput()`，采集底盘估计器所需反馈：
   - 四个腿部 DM 关节位置、速度、反馈力矩。
   - 左右轮 M3508 rpm 转换为车体方向轮速。
   - 底盘 IMU roll/pitch/yaw、gyro、acc。
   - yaw 电机编码器角度。

2. 读取 DR16 原始值到 `Dr16RawInput`：
   - 左右拨杆、四个摇杆通道、拨轮、鼠标、键盘位。

3. 读取图传键鼠 `TcRemoteInput`：
   - 如果 `gimbal_rx->vt03_online()`，使用图传桥 CAN 数据。
   - 否则如果 DR16 在线，把 DR16 鼠标/键盘回退包装成 `tc_remote`，并置 `tc_from_dr16=true`。

4. 使能边沿处理：
   - 从 disabled 进入 enabled 时，清除中腿长保持、Ctrl+C 台阶预备、自动小跳、aim mode 等跨周期状态。

5. 调用 `ResolveInputSemantics()` 生成 `ModeRequest`。

6. 处理自动小跳：
   - 使用 DYP 左右超声波平均距离。
   - 满足低腿长、距离低于阈值并保持足够时间后，置 `mode_request.auto_jump_triggered=true`。

7. 处理 host target：
   - 自瞄模式下，如果 `aimbot` 在线、NUC 启动且 `aimbot_state()!=0`，把 yaw/pitch 目标写入 `mode_request.host_target`。
   - Host 目标丢失时，从当前云台姿态接管回 RC 目标，避免跳回旧目标。

8. 填充云台 IMU 数据：
   - `gimbal_imu_yaw_rad/pitch_rad/gyro_z/gyro_x` 来自 `gimbal_rx`。

### 2.2 `ModeRequest` 的来源规则

`ModeRequest` 是当前输入层输出给两个 FSM 的统一语义请求，主要包含：

- 整车工作域：`DomainRequest::kDisabled/kService/kCombat`
- 腿长请求：`LegProfile::kLow/kMid/kHigh`
- 台阶任务：`StairTaskRequest`
- 底盘动作：`spin_hold`、`spin_dir`、`jump_trigger`、`auto_jump_triggered`
- 云台目标：`rc_target`、`host_target`、`target_source`
- 战斗子模式：`CombatProfile::kNormal/kAutoAimAmmo/kAutoAimFuSmall/kAutoAimFuBig`
- 恢复相关：`fall_detected`、`upright_stable`、`recovery_manual_mode` 等

DR16 优先级高于图传键鼠：

| 输入源 | 主要映射 |
|---|---|
| DR16 左拨杆 | Down=Disabled，Mid=Service，Up=Combat |
| DR16 右拨杆 | 普通域下 Low/Mid/High；Combat 下 Normal/AutoAimAmmo/AutoAimFuSmall |
| DR16 拨轮 | 正向小陀螺；非 Combat 下反向小陀螺 |
| 图传 Q | disabled/enabled 切换 |
| 图传 Ctrl+Q | standby/enabled 切换 |
| 图传 C | 中腿长保持切换，并重置 yaw 正方向 |
| 图传 Ctrl+C | 台阶预备中腿长模式 |
| 图传 V/B | 单台阶/双台阶任务 |
| 图传 Shift | 小陀螺 |
| 图传滚轮下 | 低腿长跳跃 |
| 图传滚轮上 | stair descend 模式切换 |
| 图传右键 | 自瞄保持，配合 G 循环选择 aim mode |
| 图传 R | 云台目标加 pi，并触发底盘驱动方向翻转 |
| 图传 Z 长按 | 倒地自启自动/手动模式切换 |
| 图传 A/D | 手动恢复模式下控制左右腿摆角速度 |

注意：`ModeRequest` 当前仍然是“旧版统一请求”，文件里已有注释标记 deprecated，后面又定义了 `ChassisFsmInput` 和 `GimbalFsmInput`。这说明代码已经开始向分离输入迁移，但目前仍保留旧结构作为中间对象。

### 2.3 台阶任务数据流

台阶逻辑分为两层：

1. `StairTaskCoordinator`
   - 管任务生命周期：`Idle -> Armed -> Executing -> BetweenSteps -> Succeeded/Aborted -> Idle`
   - 根据 V/B/连续高腿长请求、接触检测、高腿长就绪、姿态有效性和 sequence 结果决定是否启动/取消单次序列。

2. `StairClimbSequence`
   - 管一次台阶动作：`Idle -> Hook -> Retract -> Settle -> Succeeded/Aborted`
   - 输出 `ChassisMotionTarget`，覆盖底盘运动目标。

它们在 `ControlLoop()` 中位于 chassis FSM 之前。结果会回写到 `chassis_input`：

- `request_high_leg` 强制 `leg_request=kHigh`
- `force_low_leg` 强制 `leg_request=kLow`
- `task_active` 写入 `stair_task_active`
- `recovery_required` 写入 `stair_task_recovery_required`
- `completed_attempts>0` 写入 `stair_step2`

### 2.4 底盘控制数据流

底盘 FSM 只决定“当前是什么模式”和若干控制开关，不直接算力矩。

`chassis_fsm.Update()` 输出：

- `mode`
- `enable_dm`
- `run_chassis_update`
- `spin_enable`
- `recovery_enable`
- `safe_output_required`
- `leg_profile`
- `target_leg_length_m`
- `jump_phase`

之后 `ControlLoop()` 继续生成 `Chassis::UpdateInput`：

1. 基础字段：
   - `fsm_mode = chassis_output.mode`
   - `enable_output = chassis_output_enable`
   - `run_chassis_update = chassis_output.control.run_chassis_update`
   - `spin_enable = chassis_output.control.spin_enable`
   - `motion_target.leg_length_m = target_leg_length_m`

2. 台阶 sequence 激活时覆盖 `motion_target`。

3. 根据当前模式和输入生成期望状态：
   - 驱动输入归一化：`ResolveDriveInput()`
   - yaw-follow 对齐模式：正前、右侧、左侧
   - yaw-follow 目标选择：`SelectNearestYawTarget()`
   - 速度目标：普通行驶、小陀螺投影速度、spin exit、standby、台阶中置零
   - 位置保持：速度归零后冻结 `expected_s`
   - 腿摆角目标：spin、jump、stair、高腿、中腿、低腿分别赋值
   - yaw 角速度：spin 直接跟目标角速度，否则用 yaw-follow PID

4. 调用 `globals->chassis.Update(chassis_update_input)`。

`Chassis::Update()` 内部再执行：

- 状态估计器更新：关节运动学、轮速/IMU 速度融合、姿态与腿长状态。
- 腿长目标斜坡。
- LQR 系数选择。
- 腿长 PID、roll PID、恢复站立、台阶摆角 PID、LQR 输出。
- `ComputeActuatorTorque()` 把控制量转换为四个关节力矩和左右轮力矩。

最后 `Actuators::ApplyChassisOutput()` 下发四个 DM 关节 MIT 力矩和左右轮 M3508 电流。

### 2.5 云台控制数据流

`BuildGimbalFsmInput()` 从 `InputSnapshot` 和底盘 FSM 输出构造云台 FSM 输入：

- 输入有效性、工作域、service profile、combat profile
- 目标源和目标角
- gimbal test profile
- chassis recovery active
- startup align complete

`gimbal_fsm.Update()` 输出：

- `mode`
- `gimbal_enable`
- `align_to_chassis_forward`
- `active_target_source`
- `gimbal_target`
- `gimbal_test_profile`

之后 `ControlLoop()` 再补充和覆盖云台控制输入：

- 姿态无效时强制关云台，但 `RecoveryYawCentering` 例外。
- `StartupAlign` 时覆盖 yaw 目标为最近车头方向，使用 yaw 电机编码器反馈。
- `RecoveryYawCentering` 时 yaw 归中，pitch 到上限。
- 退出 recovery yaw centering 时，把 RC 目标同步到当前姿态，避免目标跳变。
- 小陀螺模式下给云台一个 chassis yaw rate 前馈。
- 自瞄模式时传入 NUC yaw/pitch 速度加速度，但当前 `gimbal.hpp` 中自瞄前馈实际被置零。

`Gimbal::Update()` 内部：

- 根据 `use_yaw_motor_feedback` 选择 yaw 反馈来源：启动归中/恢复时用 yaw 电机，否则用云台 IMU。
- 根据 `aimbot_mode`、`aimbot_is_rune`、`spin_hold` 切 PID profile。
- 辨识/前馈验证模式走 `GimbalIdent`。
- 普通模式走双轴位置/速度控制和重力/动力学前馈。
- 输出 yaw/pitch torque。

### 2.6 发射、自瞄 TX、UI 和 Debug

发射机构逻辑在 `ControlLoop()` 中直接分支：

- `WHEEL_LEGGED_ROBOT_VARIANT == 1`：三摩擦轮 hero，使用 `ShootController`。
- 其他版本：双摩擦轮和 M3508 拨盘，使用 `Shoot`。

触发条件来自：

- gimbal FSM 是否在 `kCombat`
- DR16 拨轮或图传左键
- host 自瞄反馈 bit
- 裁判热量、冷却、枪口初速

自瞄 TX 在底盘控制之后执行：

- 从 `gimbal_rx` 取欧拉角 yaw/pitch/roll。
- 根据 `combat_profile` 编码 mode。
- 从裁判系统读取 robot_id 和 bullet speed。
- 调用 `aimbot->UpdateControl()` 发给 NUC。

UI 和 debug 快照在控制环末尾更新，数据来自 `InputSnapshot`、两个 FSM 输出、底盘/云台控制输出、裁判系统和调试字段。

## 3. 底盘 FSM 转换

底盘状态定义：

| 状态 | 含义 |
|---|---|
| `kDisabled` | 关闭底盘输出 |
| `kLowLeg` | 低腿长正常模式 |
| `kMidLeg` | 中腿长正常模式 |
| `kHighLeg` | 高腿长正常模式 |
| `kSpin` | 小陀螺 |
| `kSpinExitPending` | 小陀螺退出等待 yaw 对齐 |
| `kJumpPrep` | 跳跃预备 |
| `kJumpPush` | 跳跃蹬伸 |
| `kJumpRecover` | 跳跃回收 |
| `kRecoveryFallCheck` | 姿态异常确认 |
| `kRecoverySelfRight` | 倒地自启/恢复 |
| `kStairTask` | 台阶任务由 sequence 接管 |
| `kStandby` | 低腿长姿态，轮力矩禁用语义 |
| `kStairDescend` | 下台阶/收腿模式 |

### 3.1 全局优先转换

任意状态下，如果 `input_valid=false` 或 `domain_request=kDisabled`：

```text
any -> kDisabled
```

进入 `kDisabled` 时会清除 `spin_lock_low_`。输出为：

- `enable_dm=false`
- `run_chassis_update=false`
- `safe_output_required=true`

### 3.2 正常腿长模式

正常稳定态由 `leg_request` 决定：

```text
LegProfile::kLow  -> kLowLeg
LegProfile::kMid  -> kMidLeg
LegProfile::kHigh -> kHighLeg
standby=true      -> kStandby
stair_descend_active=true 且非 standby -> kStairDescend
```

从 `kDisabled` 进入时：

```text
kDisabled -> kStairTask            if stair_task_active
kDisabled -> requested_stable_state otherwise
```

### 3.3 `kLowLeg`

```text
kLowLeg -> kRecoveryFallCheck if fall_detected
kLowLeg -> kStandby           if standby
kLowLeg -> kStairTask         if stair_task_active
kLowLeg -> kJumpPrep          if auto_jump_triggered, jump profile = Mid
kLowLeg -> kJumpPrep          if jump_trigger, jump profile = Low
kLowLeg -> kSpin              if spin_hold && |current_s_dot| < spin entry threshold
kLowLeg -> requested_stable_state otherwise
```

### 3.4 `kMidLeg`

```text
kMidLeg -> kRecoveryFallCheck if fall_detected
kMidLeg -> kStandby           if standby
kMidLeg -> kStairTask         if stair_task_active
kMidLeg -> kJumpPrep          if jump_trigger, jump profile = Mid
kMidLeg -> kSpin              if spin_hold && |current_s_dot| < spin entry threshold
kMidLeg -> requested_stable_state otherwise
```

### 3.5 `kHighLeg`

```text
kHighLeg -> kRecoveryFallCheck if fall_detected
kHighLeg -> kStandby           if standby
kHighLeg -> kStairTask         if stair_task_active
kHighLeg -> kSpin              if spin_hold && |current_s_dot| < spin entry threshold
kHighLeg -> requested_stable_state otherwise
```

### 3.6 `kStandby`

```text
kStandby -> kRecoveryFallCheck if fall_detected
kStandby -> requested_stable_state if standby=false
```

### 3.7 小陀螺

```text
kSpin -> kRecoveryFallCheck if fall_detected
kSpin -> kStandby           if standby
kSpin -> kSpinExitPending   if spin_hold=false

kSpinExitPending -> kRecoveryFallCheck if fall_detected
kSpinExitPending -> kStandby           if standby
kSpinExitPending -> kSpin              if spin_hold=true
kSpinExitPending -> requested_stable_state
    if spin_exit_yaw_aligned || elapsed >= kSpinExitTimeoutMs
```

退出 `kSpin` 时 `spin_lock_low_` 会锁住低腿长，直到检测到手动切换腿长请求才解锁。这是一个隐藏状态，复杂度不在 enum 中体现。

### 3.8 跳跃流程

跳跃流程是时间和腿长反馈驱动：

```text
kJumpPrep -> kJumpPush
    if elapsed >= prep_ms

kJumpPush -> kJumpRecover
    if current_leg_length_m >= reached_m || elapsed >= push_max_ms

kJumpRecover -> requested_stable_state
    if elapsed >= recover_min_ms && !off_ground
    or elapsed >= recover_ms
```

`jump_leg_profile_` 在进入 `kJumpPrep` 前锁定：

- 低腿长手动跳：`kLow`
- 自动小跳或中腿长跳：`kMid`

### 3.9 恢复流程

```text
kRecoveryFallCheck -> kRecoverySelfRight
    if fall_detected_hold_ms >= kRecoveryFallConfirmMs

kRecoveryFallCheck -> requested_stable_state
    if !fall_detected

kRecoverySelfRight -> kDisabled
    if elapsed >= kRecoverySelfRightTimeoutMs

kRecoverySelfRight -> requested_normal_state
    if upright_stable
```

实际恢复控制细节不在 FSM 中，而在 `Chassis::Update()` 内部处理，包括 standup phase、theta recovery phase、manual recovery speed 等。

### 3.10 台阶和下台阶

`kStairTask`：

```text
kStairTask -> kRecoveryFallCheck if fall_detected || stair_task_recovery_required
kStairTask -> kStandby           if standby
kStairTask -> kSpin              if spin_hold
kStairTask -> requested_stable_state if !stair_task_active
```

`kStairDescend`：

```text
kStairDescend -> kRecoveryFallCheck if fall_detected
kStairDescend -> kStandby           if standby
kStairDescend -> kStairTask         if stair_task_active
kStairDescend -> kSpin              if spin_hold
kStairDescend -> requested_normal_state if !stair_descend_active
```

`kStairDescend` 内还有 `stair_descend_retracted_` 隐藏状态：

- 刚进入时腿长为 `kStairDescendLegLengthM`。
- 当 `theta_b_rad > kStairDescendThetaBTriggerRad` 后锁定为低腿长。
- 离开状态后清除。

## 4. 云台 FSM 转换

云台状态定义：

| 状态 | 含义 |
|---|---|
| `kDisabled` | 关闭云台输出 |
| `kServiceWithFire` | 维护模式，允许发射链路 |
| `kServiceSafe` | 维护模式，禁止发射链路 |
| `kCombat` | 战斗模式 |
| `kRecoveryAlign` | 底盘恢复时云台对齐车体前方 |
| `kRecoveryYawCentering` | 恢复前 yaw 归中，pitch 到上限 |
| `kStartupAlign` | 上电或重新使能后的 yaw 归中 |
| `kIdent` | 云台辨识模式 |
| `kFfVerify` | 前馈验证模式 |

### 4.1 输入失效

```text
any -> kDisabled
    if input_valid=false || domain_request=kDisabled
```

### 4.2 测试模式优先级

只要输入有效：

```text
any -> kIdent    if gimbal_test_profile=kIdent
any -> kFfVerify if gimbal_test_profile=kFfVerify
```

这两个模式绕过普通 domain/service/combat 路由。

### 4.3 恢复优先级

普通路由中，恢复优先于 combat/service：

```text
any -> kRecoveryYawCentering if yaw_centering_for_recovery
any -> kRecoveryAlign        if chassis_recovery_active
```

`kRecoveryYawCentering` 特意绕过 `startup_align_complete` 限制，避免底盘 theta 恢复需要 yaw 归中时被卡在 `kStartupAlign`。

### 4.4 启动归中

```text
kDisabled -> kStartupAlign
    if input valid and not disabled

kStartupAlign -> normal_mode
    if startup_align_complete

non-disabled normal/test exit -> kStartupAlign
    if startup_align_complete=false
```

`startup_align_complete` 不是云台 FSM 自己计算的，而是 `ControlLoop()` 根据 `Gimbal::UpdateOutput::yaw_centered`、yaw 电机速度和稳定计数更新。

### 4.5 普通模式路由

```text
normal_mode = kCombat
    if domain_request=kCombat

normal_mode = kServiceSafe
    if domain_request=kService && service_profile=kChassisAndGimbalSafe

normal_mode = kServiceWithFire
    if domain_request=kService && service_profile=kChassisAndGimbalWithFire
```

云台 FSM 输出目标源选择：

```text
active_target_source = Host
    if target_source=Host && host_target_valid

active_target_source = Rc
    otherwise
```

## 5. 台阶子状态机转换

### 5.1 `StairTaskCoordinator`

任务状态：

```text
kIdle
  -> kArmed
       on kArmSingle/kArmDouble/kArmContinuous

kArmed
  -> kExecuting
       if high_leg_ready && contact_detected
       and non-hero variant requires contact_released

kExecuting
  -> kAborted
       if sequence_aborted
  -> kBetweenSteps
       if sequence_succeeded && (continuous || completed_attempts < requested_attempts)
  -> kSucceeded
       if sequence_succeeded && requested attempts completed

kBetweenSteps
  -> kArmed
       if !contact_detected

kSucceeded/kAborted
  -> kIdle
       next update, with reset_sequence=true
```

任意 active 状态下：

```text
active -> kAborted if kCancel
active -> kAborted if output_enabled=false
active -> kAborted if posture_valid=false
```

`Output` 会告诉主循环：

- 是否启动/取消/重置 sequence。
- 是否强制高腿待命。
- 是否任务 active。
- 是否要求恢复。

### 5.2 `StairClimbSequence`

动作阶段：

```text
kIdle -> kHook
    if start && not running

kHook -> kRetract
    if both theta reach hook target and stable enough
kHook -> kAborted
    if timeout/output disabled/posture invalid/cancel

kRetract -> kSettle
    if both theta reach retract target and stable enough
kRetract -> kAborted
    if timeout/output disabled/posture invalid/cancel

kSettle -> kSucceeded
    if settled stable enough
kSettle -> kAborted
    if timeout/output disabled/posture invalid/cancel
```

输出的 `ChassisMotionTarget`：

- running/succeeded/aborted 时 `use_stair_theta_controller=true`。
- running 时 `disable_wheel_torque=true`。
- `Hook/Retract/Settle` 分别输出不同腿长和左右腿摆角目标。

## 6. 当前复杂度和混乱点

### 6.1 `ControlLoop()` 是事实上的 God Function

`ControlLoop()` 当前承担了：

- 输入采集触发
- 台阶任务调度
- 底盘 FSM 输入修正
- 裁判电源裁决
- 云台 FSM
- 云台启动归中状态维护
- 云台控制输入覆盖
- 发射机构控制
- 底盘 yaw-follow
- 底盘速度和位置保持
- 底盘期望状态生成
- 自瞄 TX
- debug/UI 快照

这导致状态来源分散。比如底盘是否输出不仅由 `chassis_fsm.Output` 决定，还受到：

- 裁判电源管理
- 云台 startup align 是否完成
- 姿态有效性
- 台阶任务 active
- hidden latch，例如 `spin_lock_low_`、`stair_descend_retracted_`、`ctx.flip_180_in_progress`

### 6.2 FSM 状态和隐藏状态混在一起

底盘 FSM enum 看起来有 14 个状态，但真实行为还依赖多个隐藏状态：

- `spin_lock_low_`
- `ctrl_c_stair_`
- `stair_step2_`
- `stair_descend_retracted_`
- `jump_leg_profile_`
- `state_enter_tick_ms_`
- `ctx.spin_exit_recovery`
- `ctx.flip_180_in_progress`
- `ctx.yaw_follow_drive_ready`
- `Chassis` 内部 standup/recovery phase

这会让“当前车到底在什么行为阶段”很难只看 FSM state 得出。

### 6.3 输入层语义过多

`ResolveInputSemantics()` 同时处理：

- 输入优先级
- 按键上升沿
- 键盘模式 latch
- 腿长、台阶、standby、spin、jump
- 自瞄目标源
- 云台积分目标
- 发射摩擦轮速度调节
- 恢复手动模式

这些逻辑很多并不属于同一抽象层，后续维护容易互相影响。

### 6.4 参数文件过大

`params.hpp` 目前有约 1900 行，`k*` 常量近千处，并混合：

- common 参数
- hero/infantry3/infantry4 三套变体参数
- 电机 ID 和硬件配置
- 控制器 PID/LQR 参数
- 输入映射阈值
- 台阶流程参数
- 发射参数
- inline 运行时对象，例如 hero 的 `yaw_ff`
- 编译期 active namespace 选择

这样做的问题是：

- 很难判断参数是否仍被引用。
- 变体之间大量重复，调参时容易改错 namespace。
- 一些废弃功能的参数仍保留，增加误读成本。
- 参数和运行时对象混在一起，不利于做配置生成或静态检查。

### 6.5 废弃代码和注释块较多

当前源码中存在大量注释掉的控制分支、前馈尝试、直接置零命令、旧功能残留。例如：

- `actuators.hpp` 中多处注释掉的电机置零或关节关闭命令。
- `control.cc` 中摩擦圆限速、落地减速整段注释。
- `gimbal.hpp` 中自瞄前馈速度/加速度输入被注释，实际置零。
- `fsm_common.hpp` 保留 deprecated 的 `ModeRequest`。

这类代码不只是“占地方”，更主要的问题是让读代码的人无法判断它们是临时调试、实验失败、待恢复功能，还是已经废弃。

## 7. 重构建议

建议按低风险到高风险分阶段做，不建议一口气重写状态机。当前代码能跑的行为很多，直接推倒会很难验证。

### 阶段 1：冻结现有行为，补最小回归观测

目标是先让重构可验证。

建议：

1. 为 `chassis_fsm.cc` 和 `gimbal_fsm.cc` 建立纯函数单元测试或桌面仿真测试。
2. 用表驱动 case 覆盖关键转换：
   - disabled enable
   - low/mid/high 切换
   - spin enter/exit
   - jump prep/push/recover
   - recovery
   - stair task active/release
   - gimbal startup/recovery/test/combat
3. 给 `ResolveInputSemantics()` 加输入到 `ModeRequest` 的测试，尤其是 Q/C/V/B/R/Z/鼠标滚轮。
4. 把 `DebugSnapshot` 中关键字段作为人工回归清单。

### 阶段 2：拆 `ControlLoop()`，不改行为

只移动代码，尽量不改逻辑。

建议拆出这些函数或小类：

```text
ControlLoop()
|-- UpdateInputs()
|-- UpdateStairTask()
|-- UpdateModeFsms()
|-- UpdateGimbalControl()
|-- UpdateShootControl()
|-- UpdateChassisControl()
|-- UpdateAimbotTx()
|-- UpdateUiAndDebug()
```

其中 `UpdateChassisControl()` 再继续拆：

```text
BuildChassisUpdateInput()
UpdateChassisModeContext()
ResolveDriveCommand()
UpdateYawFollow()
UpdatePositionHold()
BuildExpectedState()
```

这一阶段的目标不是变优雅，而是先把 1000 多行主循环切成能单独阅读和测试的块。

### 阶段 3：把输入语义拆成多个 reducer

当前 `ResolveInputSemantics()` 应拆成几个互不污染的 reducer：

```text
RawInputReader
InputPriorityResolver
DomainReducer
LegModeReducer
StairCommandReducer
GimbalTargetReducer
CombatProfileReducer
RecoveryInputReducer
ShootTuningReducer
```

推荐输出结构从一个大 `ModeRequest` 改成领域化请求：

```cpp
struct VehicleRequest {
  DomainRequest domain;
  bool standby;
};

struct ChassisRequest {
  LegProfile leg;
  bool spin_hold;
  float spin_dir;
  bool jump_trigger;
  bool auto_jump_triggered;
  StairCommand stair;
  RecoveryCommand recovery;
};

struct GimbalRequest {
  CombatProfile combat_profile;
  TargetSource target_source;
  GimbalTarget rc_target;
  GimbalTarget host_target;
  bool host_target_valid;
  GimbalTestProfile test_profile;
};

struct ShootRequest {
  bool manual_fire;
  float fric_speed_target_rpm;
};
```

`ModeRequest` 可以保留一个过渡版本，但新代码不再继续往里面塞字段。

### 阶段 4：收敛状态机层级

底盘 FSM 现在把普通腿长、spin、jump、recovery、stair descend、stair task 都放在一个 enum 中。可以改成两层：

```text
ChassisMode:
  Disabled
  Standby
  Normal
  Spin
  Jump
  Recovery
  Stair
  StairDescend

NormalLeg:
  Low
  Mid
  High

JumpPhase:
  Prep
  Push
  Recover

RecoveryPhase:
  FallCheck
  SelfRight

StairPhase:
  Coordinator/Sequence existing states
```

这样 `LowLeg/MidLeg/HighLeg` 不再和 `JumpPrep/RecoverySelfRight` 平铺，输出控制也能更清楚地按 mode + substate 组合生成。

短期内也可以先做轻量版本：

- 保留现有 enum。
- 把 hidden state 全部显式放进 `Fsm::Output::DebugState`。
- 在文档和 debug 中展示 `spin_lock_low`、`jump_leg_profile`、`stair_descend_retracted`。

### 阶段 5：清理参数

不要直接按感觉删参数。建议先建立引用清单：

1. 用脚本扫描 `params::active::*` 和 `params::<variant>::*` 的引用。
2. 生成 `params_usage.csv`：
   - 参数名
   - 定义 namespace
   - 引用文件
   - 引用次数
   - 是否仅注释引用
   - 所属功能
3. 先删除 0 引用参数。
4. 再把参数按功能拆分：

```text
params/common.hpp
params/hardware.hpp
params/input.hpp
params/chassis_fsm.hpp
params/chassis_control.hpp
params/gimbal.hpp
params/shoot.hpp
params/stair.hpp
params/variant_hero.hpp
params/variant_infantry3.hpp
params/variant_infantry4.hpp
```

更理想的做法是每个模块只 include 自己的参数头，不再全项目 include 一个 139 KB 的 `params.hpp`。

### 阶段 6：处理废弃功能代码

建议建立明确规则：

- 临时调试代码不得长期注释保留，改为 `#if WHEEL_LEGGED_ENABLE_xxx_EXPERIMENT`。
- 实验功能必须有 owner、用途、启用方式和删除日期。
- 已废弃代码直接删，依赖 git 历史找回。
- 对“可能要恢复”的代码放到 `docs/archive/` 说明设计，不留在主控制路径附近。

优先清理：

1. `control.cc` 中整段注释掉的落地减速、摩擦圆限速。
2. `actuators.hpp` 中注释掉的置零命令。
3. `gimbal.hpp` 中自瞄前馈输入实际置零的旧逻辑。
4. `fsm_common.hpp` 中 deprecated `ModeRequest` 的迁移尾巴。

## 8. 推荐的最终结构

目标结构可以演进为：

```text
app/targets/wheel_legged
|-- main.cc
|-- control_loop/
|   |-- control_loop.cc
|   |-- control_loop.hpp
|   |-- input_pipeline.cc
|   |-- fsm_pipeline.cc
|   |-- chassis_pipeline.cc
|   |-- gimbal_pipeline.cc
|   |-- shoot_pipeline.cc
|   |-- debug_pipeline.cc
|-- input/
|   |-- raw_input_reader.hpp
|   |-- input_semantics.hpp
|   |-- domain_reducer.hpp
|   |-- chassis_request_reducer.hpp
|   |-- gimbal_request_reducer.hpp
|-- chassis/
|   |-- fsm.*
|   |-- controller.*
|   |-- estimator.*
|   |-- yaw_follow.*
|   |-- position_hold.*
|   |-- stair/
|-- gimbal/
|   |-- fsm.*
|   |-- controller.*
|   |-- ident.*
|-- platform/
|   |-- globals.hpp
|   |-- actuators.hpp
|   |-- hardware_init.cc
|-- params/
|-- ui/
```

重构后的依赖方向应固定为：

```text
hardware/platform -> input snapshot -> request reducers -> FSM -> controllers -> actuators
```

禁止反向依赖，例如 controller 不应读取 DR16，FSM 不应读取硬件，输入层不应关心 LQR 细节。

## 9. 最小可落地计划

建议优先做这 5 件事：

1. 给 `chassis_fsm`、`gimbal_fsm`、`StairTaskCoordinator`、`StairClimbSequence` 加表驱动测试。
2. 把 `ControlLoop()` 机械拆分成 6 到 8 个静态函数，保持行为不变。
3. 把 `ResolveInputSemantics()` 拆出 `DomainReducer`、`StairCommandReducer`、`GimbalTargetReducer`。
4. 生成参数引用清单，先删 0 引用参数和明显废弃注释块。
5. 把 hidden FSM state 暴露到 debug，先让调试时能看见真实状态。

这样做的收益最大：不需要马上重写控制算法，就能把阅读、调试和删参数的风险明显降下来。
