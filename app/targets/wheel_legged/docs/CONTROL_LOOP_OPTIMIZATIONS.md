# control_loop 控制器优化详解

本文档详解 `control_loop.cc` 阶段 7（底盘控制）中的三项关键控制器逻辑：定点锁定、速度斜坡处理、小陀螺出入过渡。

---

## 一、定点锁定

### 1.1 概述

定点锁定在松开摇杆时自动将底盘锚定在当前纵向位置，防止惯性漂移。通过带滞回的 enters/exit 阈值、最小 dwell 时长、位置捕获门控和 alpha blend 实现平滑无抖动的锁定行为。

### 1.2 参数表

| 参数 | Hero (V1) | Infantry3 (V2) | Infantry4 (V3) | 作用 |
|---|---|---|---|---|
| `kLockPointEnterInputThreshold` | 0.08 | 0.10 | 0.10 | 摇杆幅值低于此值请求锁定 |
| `kLockPointExitInputThreshold` | 0.12 | 0.12 | 0.12 | 摇杆幅值高于此值请求退出锁定 |
| `kLockPointCaptureSpeedThresholdMps` | 0.02 | 1.0 | 1.0 | 车速低于此值时允许捕获锁定参考位置 |
| `kLockPointMinDwellTicks` | 100 | 10 | 10 | 状态切换前必须保持的最小 tick 数 |
| `kLockPointAlphaRiseStep` | 0.015 | 0.015 | 0.015 | 每周期 alpha 上升量（进入锁定） |
| `kLockPointAlphaFallStep` | 0.018 | 0.018 | 0.018 | 每周期 alpha 下降量（退出锁定） |

### 1.3 状态机

```
            摇杆 < EnterThreshold
    ┌───────── 且 speed < CaptureThreshold ─────────┐
    │         且 dwell >= MinDwellTicks             │
    ▼                                               │
┌────────┐    摇杆 > ExitThreshold                 ┌────────┐
│ UNLOCK │    且 dwell >= MinDwellTicks             │  LOCK  │
│ α → 0  │ ──────────────────────────────────────► │ α → 1  │
└────────┘                                          └────────┘
    ▲                                                 │
    └─────────────────────────────────────────────────┘
```

### 1.4 核心机制

**滞回双阈值** — `EnterThreshold` (0.08-0.10) 严格小于 `ExitThreshold` (0.12)。松开摇杆时，幅值必须先降到 EnterThreshold 以下才请求锁定；摇杆回推时，幅值必须升到 ExitThreshold 以上才退出锁定。两个阈值之间的死区约 0.02-0.04，防止边界抖动导致反复切换。

**最小驻留时长** — `kLockPointMinDwellTicks` 要求状态切换后必须停留至少 N 个周期（Hero: 100 ticks，即 200ms；步兵: 10 ticks，即 20ms），才能再次切换。Hero 的 200ms 更长是因为其底盘惯量更大，需要更长判断时间避免误切换。

**位置捕获门控** — 进入锁定时，参考位置不是简单地取当前 s，而是要求同时满足三个条件才捕获：
1. `request_lock == true`（摇杆归中）
2. `speed_below_threshold`（当前车速 < 阈值）
3. `lock_point_captured == false`（尚未捕获）

这确保 `lock_point_s_ref` 记录的是一个"静止"位置，而不是减速过程中的瞬时位置。

**alpha blend 平滑过渡** — 锁定/解锁不是瞬间切换，而是通过 alpha ∈ [0, 1] 的渐进混合：
```
expected_s      = α · s_ref + (1-α) · s_current    // 位置：锁定占据当前，解锁跟随当前
expected_s_dot  = (1-α) · filtered_s_dot            // 速度：α 越大，速度越接近 0
```
alpha 上升（进入锁定）用 0.015/step，下降（退出锁定）用 0.018/step。下降略快于上升，让驾驶员"给油就走"，而锁定进入则稍缓避免顿挫。

**alpha 近零时持续刷新参考位置** — 当 alpha < 0.02 且未在请求锁定时，`lock_point_s_ref` 持续跟随当前 `s`。这保证解锁期间参考位置不"过期"（一直跟随当前真实位置），下次锁定捕获时总能拿到最新的位置作为参考。

### 1.5 使能条件

定点锁定仅在以下条件同时满足时工作：
- `chassis_output_enable == true`
- 底盘不在 kDisabled 状态
- 底盘不在 kSpin 状态（小陀螺有自己的控制逻辑）

---

## 二、速度斜坡处理

### 2.1 概述

底盘纵向速度和偏航角速度均通过带斜率限制的斜坡函数平滑变化，而非阶跃跳变。纵向速度斜坡按腿长档位区分参数（低腿/中腿/高腿），并在加速/制动方向上采用非对称步长。

### 2.2 纵向速度斜坡

#### 参数表

| 腿长档位 | accel_step | brake_step | 含义 |
|---|---|---|---|
| LowLeg | 0.01 (Hero) / 0.005 (Inf) | 0.008 (Hero) / 0.005 (Inf) | 低腿：加速快，制动稍慢 |
| MidLeg | 0.006 (Hero) / 0.004 (Inf) | 0.003 (Hero) / 0.004 (Inf) | 中腿：中等速率 |
| HighLeg | 0.003 (Hero/Inf) | 0.003 (Hero/Inf) | 高腿：对称，整体较慢 |

#### 算法

```
RampValueToTarget(target, value):
  direction_changed = (value * target) < 0        // 方向翻转
  magnitude_reduced = |target| < |value|          // 减速
  step = (direction_changed OR magnitude_reduced) ? brake_step : accel_step

  if value < target:  value = min(value + step, target)
  if value > target:  value = max(value - step, target)
```

**设计意图**：
- 制动优先于加速：方向翻转或大小缩减时用更小的步长（brake_step ≤ accel_step），让减速更平缓，避免轮胎打滑或车身前倾
- 低腿档加速步长最大：低重心时稳定性好，可承受更大加速度
- 高腿档步长最小且对称：高重心时加速/减速都需要保守，避免倾倒

### 2.3 偏航角速度斜坡

偏航角速度有两个独立的斜坡通道：

| 通道 | 步长 | 目标 | 使用场景 |
|---|---|---|---|
| 偏航跟随 (YawFollow) | 0.05 rad/s² | PID 输出 | 正常行驶时的车头朝向控制 |
| 小陀螺 (Spin) | 0.005 rad/s² | 6.0-7.0 rad/s | 小陀螺旋转 |

**偏航跟随通道** — PID 实时输出目标 yaw dot，每周期 ramp 到该值。步长较大 (0.05)，因为正常行驶需要快速响应朝向纠偏。

**小陀螺通道** — 目标恒定为常数 (6-7 rad/s)，步长极小 (0.005)，因为小陀螺需要缓慢加速到目标转速。进入小陀螺时角速度从当前值开始 ramp（继承跨周期状态），退出时 ramp 归零。

### 2.4 小陀螺纵向速度特殊处理

小陀螺使能时 (`spin_control_enabled == true`)，纵向速度不经过斜坡函数，直接使用当前状态速度作为滤波值：
```
ctx.filtered_s_dot = current_state.s_dot
```
这是为了不让底盘 LQR 控制器被 ramp lag 干扰，在小陀螺期间直接跟随物理状态。

---

## 三、小陀螺出入过渡

### 3.1 触发条件

小陀螺由两个输入源触发（OR 关系）：
- **DR16 拨轮** ≥ `kWheelSpinThreshold`
- **图传键盘 Shift 键** 按下

触发后 `mode_request.spin_hold` 为 true，底盘 FSM 转入或保持 kSpin 状态。

### 3.2 跨小陀螺边界的角速度继承

当底盘模式在小陀螺 ↔ 普通状态之间切换时：

```
cross_spin = (last_was_spin != now_is_spin)
if (cross_spin):
    ctx.filtered_yaw_dot = chassis_control_output.current_state.phi_dot
```

**为什么这样做**：底盘 LQR 状态估计器的 `phi_dot`（车体偏航角速度）是经过状态滤波器融合的物理量，切换 yaw dot 控制目标时如果直接用 ramp 从零开始，会产生角速度阶梯跳变——底盘电机会突然减速或加速，造成车体抖动。直接继承当前估计的物理角速度作为 ramp 起始值，实现"无缝切换"。

这在两个方向上都生效：
- **进入小陀螺**：从当前的偏航跟随角速度（通常接近 0）继承，ramp 到 6-7 rad/s
- **退出小陀螺**：从当前的小陀螺角速度（可能 6 rad/s）继承，ramp 回零或 PID 目标

### 3.3 小陀螺平移投影

小陀螺时车体在旋转，驾驶员希望"相对于云台系"移动。算法将云台系前进指令投影到车体系：

```
gimbal_heading = -gimbal_imu_yaw_rad                           // 云台航向（相对于车头）
vx_gimbal = forward_input_norm                                 // 摇杆前推量
s_dot_cmd = vx_gimbal * cos(gimbal_heading)                    // 投影到车体系 x
spin_target_s_dot = kSpinTranslationGain * s_dot_cmd           // 增益缩放
```

**数学直觉**：云台指向正前方时 (gimbal_heading ≈ 0)，摇杆前推 → 全速前进。云台指向侧面时 (gimbal_heading ≈ ±π/2)，cos → 0，前进指令不产生纵向速度——这是正确的，因为此时"前进"在车体系下是横向移动，由 LQR 的侧向通道处理。

### 3.4 小陀螺专属偏置

小陀螺时腿摆角期望值独立设置：
```
expected.theta_ll = kSpinThetaLlBiasRad    // 0.0 (Hero) / 0.01 (Inf)
expected.theta_lr = 0.0
```
普通模式使用 FSM 下发的 `theta_leg_target_rad` 或默认值。小陀螺强制左右不对称偏置（左腿略大），用于补偿旋转离心力导致的腿受力不均。

### 3.5 小陀螺期间的连锁状态重置

进入小陀螺后，以下状态被抑制或重置：
```
yaw_follow_drive_ready = false        // 重置偏航就绪
yaw_follow_drive_ready_stable_ticks = 0
lock_point_target = false             // 禁用定点锁定（spin 本身不需要）
```
退出小陀螺时（通过 `ResetOnModeChange`），所有跨周期状态被重置：滤波值、PID、锁定 alpha、偏航目标。

---

## 四、偏航跟随驱动就绪门控

### 4.1 动机

底盘纵向驱动必须等待偏航电机先对准目标方向。如果在偏航未对准时就驱动底盘前进，车体会朝错误方向移动。

### 4.2 判稳逻辑

偏航跟随到位判断 (`IsYawFollowDriveReady`)：
```
|yaw_target - yaw_motor| < 0.04 rad     // 角度误差 < 2.3°
|yaw_motor_vel| < 0.25 rad/s             // 角速度 < 14°/s
```
两个条件同时满足 ≥ 50 个连续周期（500Hz 下 = 100ms），`yaw_follow_drive_ready` 才置为 true。

只有 `yaw_follow_drive_ready == true` 时，纵向速度目标才被允许 non-zero。

### 4.3 复位条件

以下任一情况导致就绪复位：
- 无遥控器输入或工作域进入 kDisabled
- 偏航跟随对齐模式切换（如从前进切换到侧向）
- 进入小陀螺

---

## 五、整体数据流总结

```
摇杆/键盘输入
    │
    ├── ResolveDriveInput ─── forward/side 归一化
    │
    ├── [偏航支路] ─── PID(偏航电机 → 目标角) ─── RampYawDot ─── expected.phi_dot
    │
    ├── [纵向支路] ─── target_s_dot ─── RampValueToTarget ─── filtered_s_dot
    │                                        │
    │                              [定点锁定 alpha blend]
    │                                        │
    │                              expected_s_dot = (1-α) · filtered_s_dot
    │                              expected_s     = α · s_ref + (1-α) · s_current
    │
    ├── [小陀螺分支]
    │     ├── phi_dot: Ramp → kSpinTargetYawDotRadS
    │     ├── s_dot:   投影 cos(云台航向) + spin 增益
    │     └── theta:   spin 专属偏置
    │
    └── Chassis.Update(expected) ─── LQR 解算轮/腿力矩
```

## 六、变体差异速查

| 机制 | Hero (V1) | Infantry3 (V2) | Infantry4 (V3) |
|---|---|---|---|
| 锁点 Dwell | 200ms (100 ticks) | 20ms (10 ticks) | 20ms (10 ticks) |
| 锁点速度阈值 | 0.02 m/s | 1.0 m/s | 1.0 m/s |
| 锁点输入进入阈值 | 0.08 | 0.10 | 0.10 |
| 小陀螺目标角速度 | 6.0 rad/s | 6.0 rad/s | 7.0 rad/s |
| 小陀螺Yaw ramp步长 | 0.005 | 0.05 | 0.05 |
| 小陀螺平移增益 | 1.0 | 1.0 | 1.2 |
| Spin theta ll bias | 0.0 rad | 0.01 rad | 0.01 rad |
| SdotRamp LowLeg | 0.01/0.008 | 0.005/0.005 | 0.01/0.008 |
