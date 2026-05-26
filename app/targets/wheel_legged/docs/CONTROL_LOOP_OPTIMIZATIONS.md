# control_loop 控制器优化详解

本文档详解 `control.cc` 阶段 7（底盘控制）中的三项关键控制器逻辑：位置锚定（类 PI 控制）、速度斜坡处理、小陀螺出入过渡。

---

## 一、位置锚定 — 类 PI 纵向控制

### 1.1 概述

LQR 是全状态反馈调节器，本质为 PD 控制，没有积分项。单独使用时，摇杆归中后车体会随扰动漂移。本方案不引入显式位置环 PID，而是通过管理 LQR 期望状态中的 `expected_s`（期望位移）实现位置锚定，等效于"类 PI"控制。

**核心思想**：正常行驶时 `expected_s` 实时跟随当前位移（误差为零，LQR 仅做速度控制）；摇杆归中后 `expected_s` 沿速度衰减路径积分，最终冻结为位置锚点（误差持续存在，LQR 同时调节位移和速度，等效 PI）。

### 1.2 与旧方案的区别

旧方案使用 alpha-blend 定点锁定（`alpha × s_ref + (1-alpha) × current_s`），需要 7 个可调参数（滞回双阈值、消抖 tick、零速判稳 tick、alpha 上升/下降步长等）。新方案通过速度斜坡输出 `filtered_s_dot` 驱动 `expected_s` 的积分，将位置锚定与速度斜坡统一：**斜坡走完 = 位置已锚定**。仅需 `kPositionFreezeSpeedThresholdMps` 一个参数确认物理静止。

### 1.3 参数表

| 参数 | Hero (V1) | Infantry3 (V2) | Infantry4 (V3) | 作用 |
|---|---|---|---|---|
| `kPositionFreezeSpeedThresholdMps` | 0.30 | 0.35 | 0.40 | 物理车速低于此值时确认静止，snap 锚点 |

### 1.4 时序

```
时间轴 →

摇杆归中 → target_s_dot 设为零 → filtered_s_dot 经斜坡逐步衰减 → 到达零速
                                                                      │
                                                        expected_s 沿斜坡路径积分
                                                        到达零速+物理静止后冻结为锚点
```

**状态机：**
```
驾驶员有指令 → integrate_position = false, expected_s = current_s（无位置误差）
摇杆归中     → integrate_position = true,  expected_s += filtered_s_dot × dt
filtered_s_dot=0 且 物理车速<阈值 → expected_s snap 到 current_s, integrate_position = false（冻结锚点）
```

### 1.5 为什么等效 PI

LQR 输出 `u = -K × (x - x_ref)`。当 `expected_s` 冻结于锚点而实际位置因扰动偏离时，`s - expected_s` 非零：

- **K 矩阵中 s 通道的反馈** → 等效 P（比例位置环），产生回位力
- **K 矩阵中 s_dot 通道的反馈** → 等效 D（速度阻尼），抑制振荡
- 两者的联合作用 → 等效 PI 控制，无静差地锚定位置

与显式 PI 相比的优势：
- 不需要额外的位置环 PID 调参（无 kp/ki/kd）
- 天然抗积分饱和（锚点在车实际停下的位置）
- LQR 一次解算 10 维状态的耦合控制，比独立 PID 更协调

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
普通模式使用腿型默认偏置；台阶序列通过 `motion_target` 独立下发摆角目标。小陀螺强制左右不对称偏置（左腿略大），用于补偿旋转离心力导致的腿受力不均。

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
    │                              [驾驶中] expected_s = current_s（误差=0）
    │                              [归中后] expected_s += filtered_s_dot × dt
    │                              [停止后] expected_s 冻结为锚点
    │                                        │
    │                              expected_s_dot = filtered_s_dot
    │
    ├── [小陀螺分支]
    │     ├── phi_dot: Ramp → kSpinTargetYawDotRadS
    │     ├── s_dot:   投影 cos(云台航向) + spin 增益
    │     │            expected_s_dot = spin_target_s_dot
    │     └── theta:   spin 专属偏置
    │
    └── Chassis::Update(expected) ─── LQR 解算 u = -K × (x - x_ref)
          ├── 轮力矩: base_torque.t_wl/t_wr
          └── 腿力矩: J^T × [leg_length_force + gravity_ff + roll_pid + spring; base_torque.t_bl/t_br]
```

## 六、变体差异速查

| 机制 | Hero (V1) | Infantry3 (V2) | Infantry4 (V3) |
|---|---|---|---|
| 位置锚定冻结阈值 | 0.30 m/s | 0.35 m/s | 0.40 m/s |
| 小陀螺目标角速度 | 6.0 rad/s | 6.0 rad/s | 7.0 rad/s |
| 小陀螺Yaw ramp步长 | 0.005 | 0.05 | 0.05 |
| 小陀螺平移增益 | 1.0 | 1.0 | 1.2 |
| Spin theta ll bias | 0.0 rad | 0.01 rad | 0.01 rad |
| SdotRamp LowLeg | 0.01/0.008 | 0.005/0.005 | 0.01/0.008 |
