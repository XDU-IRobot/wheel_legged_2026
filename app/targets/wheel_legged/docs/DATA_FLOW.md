# 轮腿机器人控制数据流详解

## 概览

本文档跟踪 500Hz 控制环从硬件传感器到电机输出的完整数据流。每个周期经历 9 个流水线阶段，数据单向流动：**原始硬件 → 语义输入 → 状态机决策 → 控制器解算 → 执行器输出**。

---

## 数据流总览图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         500Hz ControlLoop()                              │
├─────────────────────────────────────────────────────────────────────────┤
│ (1) 硬件采集      (2) 语义输入        (3) FSM 决策      (4) 云台控制    │
│ ┌──────────┐   ┌──────────────┐   ┌─────────────┐   ┌──────────────┐   │
│ │ CAN_Bus  │──▶│UpdateRawFeed-│──▶│ChassisFsmIn-│──▶│Gimbal::Update│   │
│ │ DR16_UART│   │backAndInput- │   │put / Gimbal-│   │  → GimbalUp- │   │
│ │ IMU_UART │   │Snapshot()    │   │FsmInput     │   │  dateOutput  │   │
│ │ C-Board  │   │ →InputSnap-  │   │→ FSM::Update│   │              │   │
│ └──────────┘   │   shot{}     │   │→ FSM::Output│   └──────┬───────┘   │
│                  └──────┬───────┘   └──────┬──────┘          │          │
│                         │                  │                 │          │
│ (5) 发射机构控制 ───────┼──────────────────┼─────────────────┘          │
│ ┌──────────────────────┐│                  │                            │
│ │ Hero: ShootController││   (6) 启动归中判断                            │
│ │ Inf : Shoot::Update  ││   ┌──────────────────────────┐                │
│ │ → ShootOutput        ││   │ gimbal_startup_align     │                │
│ └──────────┬───────────┘│   │ _complete 闭环判稳        │                │
│            │            │   └──────────┬───────────────┘                │
│            │            │              │                                │
│   (7) 底盘控制 ◀────────┼──────────────┘                                │
│   ┌────────────────────┐│                                               │
│   │ DriveInput + 偏航  ││   (8) 执行器统一下发                          │
│   │  跟随 + 定点锁定   ││   ┌──────────────────────────┐                │
│   │ → Chassis::Update  ││   │ Actuators::              │                │
│   │ → ChassisUpdate-   ││   │  ApplyChassisOutput()    │                │
│   │   Output           ││   │  ApplyGimbalOutput()     │                │
│   └────────┬───────────┘│   │  ApplyShootOutput()      │                │
│            │            │   │ → DM MIT / M3508 CAN     │                │
│            └────────────┼──▶└──────────────────────────┘                │
│                         │                                               │
│   (9) 调试导出 ◀────────┘                                               │
│   ┌─────────────────────────────────┐                                   │
│   │ UpdateDebugSnapshot()           │                                   │
│   │ → wl_debug (SRAM4, DMA 可读)    │                                   │
│   └─────────────────────────────────┘                                   │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 阶段 1：硬件反馈采集

**代码位置**: `UpdateRawFeedbackAndInputSnapshot()` → `Actuators::FillEstimatorInput()`

### 输入（硬件层）

| 硬件设备 | 接口 | 数据项 | 目标字段 |
|----------|------|--------|----------|
| DM 关节电机 ×4 | FDCAN1 (joint_can) | pos, vel, tau | `estimator_input.{left_leg,right_leg}.{front,back}` |
| M3508 轮毂 ×2 | FDCAN2 (wheel_can) | rpm → rad/s | `estimator_input.wheel.{left_rad_s,right_rad_s}` |
| Hipnuc IMU | UART10 (imu_uart) | roll, pitch, yaw, gyro, accel | `estimator_input.imu.*` |
| DM 偏航电机 | FDCAN2 (wheel_can) | pos (MIT 反馈) | `estimator_input.yaw_motor_rad` |
| 云台 C 板 CAN | FDCAN3 (gimbal_can) | 惯导 + 键鼠 | `gimbal_rx` (下文详述) |
| DR16 遥控器 | UART5 (rc_uart) | 摇杆/拨杆/拨轮原始值 | `Dr16RawInput` |

### 输出

- `InputSnapshot.estimator_input : ChassisStateEstimatorInput` — 供底盘状态估计器使用
- `Dr16RawInput` — 6 通道原始值（switch_l, switch_r, right_x, right_y, left_x, left_y, dial）
- `TcRemoteInput` — 图传键鼠数据（mouse_x, mouse_y, left_button, keyboard_value）

---

## 阶段 2：语义输入折叠

**代码位置**: `ResolveInputSemantics()` in `input_resolver.cc`

将遥控器/图传的**原始开关/摇杆值**折叠为整车级**语义请求**（解耦：后续模块不再直接读取 DR16）。

### 输入 → 输出映射

| 原始输入 | 判断逻辑 | 语义输出 |
|----------|----------|----------|
| `switch_l` | Up→Combat, Mid→Service, Down→Disabled | `domain_request : DomainRequest` |
| `switch_r` (Combat) | Down→Normal+LowLeg, Mid→AutoAimNoMove+LowLeg, Up→AutoAimWithMove+MidLeg | `combat_profile`, `leg_request` |
| `switch_r` (非 Combat) | Down→Low, Mid→Mid, Up→High | `leg_request` |
| `dial` | >= wheel_spin_threshold 或 Shift 键 | `spin_hold` |
| `dial` (非 Combat) | <= -wheel_action_threshold 上升沿 | `jump_trigger` |
| left_x / mouse_x 积分 | 速率 × dt 累加 + WrapToPi | `rc_target.yaw_rad` |
| left_y / mouse_y 积分 | 速率 × dt 累加 + Clamp | `rc_target.pitch_rad` |
| keyboard C 键 | 上升沿切换 | `tc_state.mid_leg_hold` |

### ModeRequest 结构体字段

```
ModeRequest {
  input_valid, domain_request, service_profile, leg_request
  spin_hold, jump_trigger
  current_leg_length_m, theta_ll_rad, theta_lr_rad  ← 回灌自上周期
  combat_profile, target_source
  rc_target, host_target, host_target_valid
  fall_detected, fall_detected_hold_ms, upright_stable
  tick_ms
}
```

### @todo 项（尚未接入）

- `service_profile` 硬编码为 `kChassisAndGimbalSafe`
- `fall_detected` 始终为 false（IMU 倒地检测未接入）

### 已部分接入

- `host_target` / `host_target_valid`：自瞄 CAN 通信已工作（`input_resolver.cc:259-268`），NUC 启动后 `host_target_valid` 置 true，云台在 AutoAim 模式下可切换为上位机目标。尚未与 NUC 实车联调验证。

---

## 阶段 3：FSM 分支

**代码位置**: `BuildChassisFsmInput()` → `chassis_fsm.Update()`, `BuildGimbalFsmInput()` → `gimbal_fsm.Update()`

ModeRequest 被拆分为两个独立的结构体，分别送入底盘和云台状态机，实现决策隔离。

### ChassisFsmInput（底盘状态机）

```
input_valid, domain_request, leg_request, combat_profile
spin_hold, jump_trigger
current_leg_length_m, theta_ll_rad, theta_lr_rad  ← 从上周期底盘输出回灌
fall_detected, fall_detected_hold_ms, upright_stable
tick_ms
```

### GimbalFsmInput（云台状态机）

```
input_valid, domain_request, service_profile, combat_profile
target_source, rc_target, host_target, host_target_valid
chassis_recovery_active  ← 从底盘 FSM 输出派生
startup_align_complete   ← 从 control_loop 内部判稳派生
```

### 底盘状态机转移（12 状态）

```
kDisabled ──▶ kLowLeg / kMidLeg / kHighLeg  (domain != disabled)
kLowLeg   ──▶ kSpin              (spin_hold)
          ──▶ kJumpPrep          (jump_trigger, non-combat)
          ──▶ kRecoveryFallCheck (fall_detected)
kMidLeg   ──▶ kSpin / kRecoveryFallCheck / normal
kHighLeg  ──▶ kSpin / kRecoveryFallCheck / kStairClimb (theta 超阈值)
kSpin     ──▶ normal / kRecoveryFallCheck
kJumpPrep ──▶ kJumpPush          (计时到)
kJumpPush ──▶ kJumpRecover       (腿长到或超时)
kJumpRecover ─▶ kLowLeg          (计时到)
kRecoveryFallCheck ─▶ kRecoverySelfRight (倒地确认) / kLowLeg (恢复)
kRecoverySelfRight ─▶ kLowLeg (upright_stable) / kDisabled (超时)
kStairClimb ──▶ kStairClimbDone  (腿到位+计时) / kRecoveryFallCheck
kStairClimbDone ─▶ kHighLeg      (pitch 稳定计时到)
```

### 云台状态机转移（6 状态）

```
kDisabled    ──▶ kStartupAlign    (首次使能)
kStartupAlign ─▶ normal mode     (startup_align_complete)
             ──▶ kDisabled       (输入失效)
normal       ──▶ kDisabled       (输入失效)
             ──▶ kRecoveryAlign  (chassis_recovery_active)
kRecoveryAlign ─▶ normal mode    (恢复结束)
```

### FSM 输出字段

**chassis::Fsm::Output::ControlOutput:**
```
enable_dm, run_chassis_update, spin_enable
leg_profile, target_leg_length_m, theta_leg_target_rad
recovery_enable, safe_output_required, jump_phase
```

**gimbal::Fsm::Output::ControlOutput:**
```
gimbal_enable, align_to_chassis_forward
active_target_source, gimbal_target
```

---

## 阶段 4：云台控制

**代码位置**: `gimbal.Update()` in `gimbal/gimbal.hpp`

### 输入：`Gimbal::UpdateInput`

| 字段 | 来源 | 说明 |
|------|------|------|
| `yaw_motor` / `pitch_motor` | SharedResources | DM 电机对象指针 |
| `gimbal_enable` | gimbal_fsm output | 是否使能 |
| `align_to_chassis_forward` | gimbal_fsm output | 是否对车头 |
| `use_yaw_motor_feedback` | control_loop | 启动归中阶段用电机编码器 |
| `target` | gimbal_fsm output | 角度目标 |
| `chassis_yaw_rad` | 底盘 IMU | 对齐车头时的目标源 |
| `chassis_pitch_rad` | 底盘 IMU | 俯仰重力补偿 |
| `yaw_motor_rad` | DM 反馈 | 电机编码器值 |
| `gimbal_imu_yaw/pitch_rad` | CAN 桥 | 云台惯导值 |

### 控制逻辑

- **反馈选择**: 启动归中阶段用偏航电机编码器（`use_yaw_motor_feedback`），正常阶段用云台惯导
- **目标选择**: 对齐车头时目标 = 底盘 IMU yaw，否则 = FSM 下发的 target
- **PID**: 双环（位置外环 + 速度内环）× 2 轴（yaw + pitch）
- **俯仰前馈**: `pitch_gravity_ff = kPitchGravityCompensationNm × cos(chassis_pitch)`
- **输出限幅**: DM 力矩限幅 ±kDmTorqueLimitNm

### 输出：`Gimbal::UpdateOutput`

```
gimbal_enabled, yaw_target_rad, yaw_pos_rad, yaw_vel_rad_s, yaw_cmd_torque_nm
pitch_target_rad, pitch_pos_rad, pitch_vel_rad_s, pitch_cmd_torque_nm
```

---

## 阶段 5：发射机构控制（变体分支）

**代码位置**: `control_loop.cc` 发射机构段

### Hero (VARIANT == 1)：三摩擦轮 + DM 拨盘

使用 `ShootController` 类 (`include/gimbal/shoot_3fric.hpp`)，内置 5 状态机：

```
kStop ── enter_shoot ──▶ kInitialize ── 初始化完成 ──▶ kReady
kReady ── fire_trigger ─▶ kShooting ── 拨盘到位 ──▶ kCooling
kCooling ─▶ kReady (继续) / kStop (退出)
```

- 摩擦轮：3 路独立速度 PID，退出时反转制动
- 拨盘：位置 PID（圆形）+ 速度 PID（MIT 力矩模式），分段角度控制
- 进入条件：`gimbal_output.mode == kCombat`

### Infantry3/4 (VARIANT != 1)：双摩擦轮 + M3508 拨盘

使用 `Shoot` 类 (`include/gimbal/shoot.hpp`)，内部使用 `Shoot2Fric` 控制器：

- 输入：`fric_left_rpm, fric_right_rpm, dial_encoder, dial_rpm, dr16_dial, mouse_left`
- 输出：`ShootOutput { fric_left_current, fric_right_current, dial_current }`
- 下发由 `Actuators::ApplyShootOutput()` 统一处理

---

## 阶段 6：启动归中判稳

**代码位置**: `control_loop.cc` 云台启动归中段

- `gimbal_startup_align_active` 且首次进入 → 锁定最近的车头方向为归中目标
- 偏航电机角误差 < 阈值 且 角速度 < 阈值 持续 N 个周期 → `startup_align_complete = true`
- 完成后将 RC 积分目标对齐到当前云台惯导角（消除归中过程的目标漂移）
- 底盘在归中未完成前暂缓输出（`chassis_output_enable` 联动）

---

## 阶段 7：底盘控制

**代码位置**: `control_loop.cc` 底盘控制段（7a—7m）→ `chassis.Update()`

阶段 7 是底盘控制的核心。它接收 FSM 输出和驾驶输入，通过速度斜坡、位置锚定和 LQR 状态反馈，计算出轮毂和腿关节的力矩指令。整个流程分 13 个子阶段。

---

### 7a. 底盘控制器输入组装

将 FSM 输出、状态估计器输入和跨周期期望状态打包为 `Chassis::UpdateInput`：

```
estimator_input, expected（期望状态向量）, fsm_mode
enable_output, run_chassis_update, spin_enable, target_leg_length_m
```

---

### 7b. 模式切换处理

底盘 FSM 模式变化时：
- 调用 `ctx.ResetOnModeChange()` 重置所有跨周期状态（滤波值、PID、偏航目标等）
- 跨小陀螺边界时（普通↔spin），`filtered_yaw_dot` 继承当前物理 `phi_dot`，实现无缝角速度过渡

---

### 7c. 驾驶输入解析

```
DR16 右摇杆 Y  → forward_input_norm  (前进/后退)
DR16 右摇杆 X  → side_input_norm     (平移)
TcRemote WASD  → forward/side (DR16 离线时替代)
```

DR16 在线时优先摇杆；DR16 离线时降级为图传键盘 WASD。

---

### 7d. 偏航跟随模式选择

根据驾驶输入方向选择车头对准模式：

| 模式 | 触发条件 | 目标偏航角偏移 |
|------|----------|----------------|
| kForward | 前进/后退有输入 或无输入 | 0° |
| kSidePositive | 仅右平移 | +kYawFollowSideOffsetRad |
| kSideNegative | 仅左平移 | -kYawFollowSideOffsetRad |

---

### 7e. 偏航目标更新

- 首次进入或模式变化时，选择离当前偏航电机角最近的方向（两个候选差 π），避免大幅回旋
- 偏航跟随 PID 按 `kYawFollowPid` 参数跟踪目标角

---

### 7f. 纵向速度目标计算

```
spin模式: target_s_dot = 0（走小陀螺投影通道）
偏航未就绪: target_s_dot = 0（等待偏航对齐）
有前进输入: target_s_dot = drive_sign × kTargetForwardSpeedMaxMps × forward_input_norm
有平移输入: target_s_dot = drive_sign × kTargetForwardSpeedMaxMps × side_input_norm
```

---

### 7g. 偏航就绪判稳

偏航电机角误差 < 阈值 **且** 角速度 < 阈值 持续 ≥ N 个周期（500Hz 下 100ms），`yaw_follow_drive_ready` 才置 true。
偏航未就绪时禁止底盘纵向驱动，防止车头未对准就移动。

---

### 7h. 目标纵向速度（汇总）

综合 7f 和 7g 的结果：
- Spin 模式：`spin_target_s_dot` = 云台前进指令经 `cos(gimbal_heading)` 投影 × 增益
- 偏航未就绪：`target_s_dot = 0`
- 正常行驶：`target_s_dot` 由 7f 计算

---

### 7i. 纵向位置 I 项管理（类 PI 控制的核心）

这是整个底盘纵向控制的关键设计。LQR 是状态反馈调节器（本质为 PD），通过巧妙地管理 `expected_s`（期望位移），在不引入显式积分器的情况下实现了位置无静差锁定。等效为"PI 风格"的 LQR 控制。

**核心思想：**

```
正常行驶： expected_s 实时跟随 current_s，位移误差为零
         → LQR 仅调节速度误差（s_dot - expected_s_dot）
         → 等效纯 P（比例）速度控制

摇杆归中： target_s_dot = 0，filtered_s_dot 经斜坡逐步衰减
         expected_s 沿速度斜坡积分，构建平滑减速轨迹
         filtered_s_dot 归零后 expected_s 冻结于停止位置
         → LQR 同时调节位移误差（s - expected_s）和速度误差
         → 等效 PI 控制：位置被锚定，LQR 产生持续力消除静差
```

**实现细节（`control_loop.cc:338-355`）：**

```
can_hold_position = 底盘使能 && 非Disabled && 非Spin

如果 (不能保持位置 或 驾驶员有指令):
    integrate_position = false
    expected_s = current_s          // 位移误差归零，仅速度控制生效

否则如果 (filtered_s_dot != 0):
    integrate_position = true
    expected_s += filtered_s_dot * dt   // 沿速度衰减轨迹积分

否则如果 (integrate_position && 物理速度已静止):
    expected_s = current_s          // snap 到实际停住位置
    integrate_position = false      // 冻结为锚点
```

**为什么这样设计：**
- 不依赖物理车速估计做锚点触发（估计器可能有延迟或噪声）
- 自动适配不同腿长的制动速率（斜坡参数已按腿长分档）
- 位置锚点在车实际停下的位置，无累积误差
- LQR 增益矩阵中 `s` 通道的反馈即等效 P（位置环），`s_dot` 通道即等效 D（速度环），共同作用等效 PI
- 不需要单独的位置环 PID，LQR 已经一次解算所有通道的耦合控制

---

### 7j. 纵向速度斜坡

将 `target_s_dot` 经 `RampValueToTarget()` 平滑到 `filtered_s_dot`。斜坡参数按腿长分档（低/中/高腿），且加速/制动非对称（制动步长更小，减速更平缓）。Spin 模式跳过斜坡，直接使用物理速度。

---

### 7k. 期望状态填充

将 control_loop 层计算出的期望值填入 LQR 的 `ExpectedState`：

| 状态分量 | 来源 | 说明 |
|---------|------|------|
| `expected.s` | `ctx.expected_s` | 7i 管理的位置锚点 |
| `expected.s_dot` | `ctx.filtered_s_dot`（7j）或 `spin_target_s_dot` | 斜坡后的速度目标 |
| `expected.phi` | `current_state.phi` | 偏航角不干预（跟随实际值） |
| `expected.phi_dot` | `ctx.filtered_yaw_dot`（7l）或 `kSpinTargetYawDotRadS` | 偏航角速度目标 |
| `expected.theta_{ll,lr}` | FSM 下发或默认偏置 | 腿摆角目标 |
| `expected.theta_b` | `kExpectedThetaBBiasRad` | 期望俯仰角（通常为 0） |

离地时（中/高腿长 `off_ground`），`expected.s_dot` 强制使用物理速度，避免 LQR 对悬空轮端施加不合理的力矩。

---

### 7l. 偏航角速度控制

- **Spin 模式**：`filtered_yaw_dot` 斜坡到 `kSpinTargetYawDotRadS`（6~7 rad/s），步长极小（0.005~0.05），缓慢加速
- **偏航跟随模式**：PID 输出目标 yaw_dot，经斜坡到 `filtered_yaw_dot`
- **其他**：`filtered_yaw_dot` 清零

---

### 7m. 底盘控制器执行

调用 `chassis.Update(chassis_update_input)`，内部流程：

1. **状态估计更新**：卡尔曼滤波器融合轮速和 IMU 加速度，输出 12 维当前状态向量
2. **腿部运动学刷新**：五连杆正向运动学解算腿长 l0、摆角 theta、雅可比矩阵
3. **支撑力估计**：关节力矩经雅可比反推 → 重力+动力学补偿 → 左右腿支撑力
4. **姿态安全检查**：`posture_valid` 判断（俯仰/横滚/腿摆角在安全范围）
5. **起立判稳**：双腿 theta 均 < 阈值后锁存 `standup_complete`，解锁轮端力矩
6. **腿长斜坡平滑**：目标腿长按 `kLegLengthRampTimeS`（0.5s）平滑过渡
7. **力矩合成**（`ComputeActuatorTorque`）：

```
LQR 解算:  u = -K(l_l, l_r) × (current - expected)
           → base_torque = {t_wl, t_wr, t_bl, t_br}

腿长控制:  leg_length_force = PID(目标腿长 - 平均腿长)
           + gravity_ff（重力前馈）
           + roll_pid（横滚补偿）
           + inertial_ff（惯性补偿）
           + spring_torque（弹簧补偿）

跳跃时:    阶段1收腿 → PID+弹簧，阶段2蹬伸 → 专用高输出PID，阶段3回收 → 专用回收PID

力→关节力矩: tau_joint = J^T × [force; torque]
            （雅可比转置将竖直力+髋力矩映射到前/后关节）

轮端力矩:   output.lw_tau = -base_torque.t_wl
           output.rw_tau =  base_torque.t_wr
           （离地/跳跃回收/上台阶/起立未完成时强制为零）
```

### 底盘控制器输出：`Chassis::UpdateOutput`

```
lf_tau, lb_tau, rf_tau, rb_tau    ← 腿部 DM 关节力矩 (Nm)
lw_tau, rw_tau                    ← 轮毂力矩 (Nm)
left_support_force_n, right_support_force_n
mean_leg_length_m, speed_mps
posture_valid, off_ground_in_mid_high_leg, standup_complete
current_state (12 维状态向量)
```

---

## 阶段 8：执行器统一下发

**代码位置**: `include/actuators.hpp` — `Actuators` 类

所有电机 IO 在此集中，不分散在各控制器中。

### ApplyChassisOutput

```
DM 使能/失能锁存 (首次 enable 时发 enable 指令)
→ SendDmMitCommand(lf_tau, lb_tau, rf_tau, rb_tau)
→ wheel_tau * kTorqueToCurrent → M3508 SetCurrent
→ DjiMotorBase::SendCommand(wheel_can)
```

### ApplyGimbalOutput

```
DM 使能/失能锁存
→ SendGimbalMitCommand(yaw_tau, pitch_tau)
```

### ApplyShootOutput (仅 infantry)

```
fric_left/right → M3508 SetCurrent
dial → M3508 SetCurrent
→ DjiMotorBase::SendCommand(gimbal_can)
→ DjiMotorBase::SendCommand(wheel_can)
```

---

## 阶段 9：调试快照导出

**代码位置**: `debug_export.cc` → `UpdateDebugSnapshot()`

`DebugSnapshot` (512 bytes, `__attribute__((section(".sram4")))`) 包含所有关键内部量：

| 类别 | 字段 |
|------|------|
| 时间戳/状态机 | tick_ms, chassis_fsm_state, gimbal_fsm_state, state_changed |
| DR16 原始 | switch_l/r, dial, left_x/y, right_x/y, online |
| DR16 语义 | enable_request, spin_request, jump_trigger_edge |
| 电机反馈 | 4*关节 pos/vel/tau, 2*轮速, yaw/pitch pos/vel |
| 电机输出 | 4*DM 力矩, 2*轮力矩, yaw/pitch target/cmd |
| IMU | 底盘 9 轴, 云台 2 轴 |
| 状态向量 | 12 维 (s, s_dot, phi, phi_dot, theta_ll, theta_ll_dot, theta_lr, theta_lr_dot, theta_b, theta_b_dot, l_l, l_r) |
| 期望值 | expected_s/s_dot, gimbal_target_yaw/pitch |
| 底盘状态 | mean_leg_length, left/right_leg_length, speed, support_force, posture_valid, off_ground |
| 定点锁定 | lock_point_enabled/request/captured/rising_edge/speed_below_threshold |
| DYP 超声波 | distance_raw left/right, result left/right, frame_count |
| 发射 | hero: booster_pos, fw_rpm*3 / infantry: shoot_enabled |
| 图传 | tc_mid_leg_hold |

SRAM4 段可由调试器/DMA 直接读取，不占用主循环 CPU 时间。

---

## 关键结构体索引

| 结构体 | 文件 | 定位 |
|--------|------|------|
| `DebugSnapshot` | `include/debug.hpp` | 全局调试快照（SRAM4） |
| `SharedResources` | `include/globals.hpp` | 所有外设/控制器/状态机实例 |
| `ModeRequest` | `include/fsm_common.hpp` | 旧版统一语义请求 (@deprecated) |
| `ChassisFsmInput` | `include/fsm_common.hpp` | 底盘 FSM 专用输入 |
| `GimbalFsmInput` | `include/fsm_common.hpp` | 云台 FSM 专用输入 |
| `InputSnapshot` | `include/input.hpp` | 单周期硬件+语义快照 |
| `ChassisStateContext` | `include/state_ctx.hpp` | 控制环跨周期状态 |
| `Chassis::UpdateInput` | `include/chassis/chassis.hpp` | 底盘控制器输入 |
| `Chassis::UpdateOutput` | `include/chassis/chassis.hpp` | 底盘控制器输出 |
| `Gimbal::UpdateInput` | `include/gimbal/gimbal.hpp` | 云台控制器输入 |
| `Gimbal::UpdateOutput` | `include/gimbal/gimbal.hpp` | 云台控制器输出 |
| `ShootOutput` | `include/gimbal/shoot_2fric.hpp` | 双摩擦轮发射输出 |
| `ShootController` | `include/gimbal/shoot_3fric.hpp` | 三摩擦轮发射状态机 |
| `ChassisStateEstimatorInput` | `include/chassis/state.hpp` | 状态估计器传感器输入 |
