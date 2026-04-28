# wheel_legged 状态机迁移与拆分定义

本文记录将 `app/targets/temp/fsm` 中的整车状态机语义迁移到 `wheel_legged` 的方案。目标不是继续保留一个整车大状态机，而是把 `temp` 的语义拆成底盘 FSM 和云台 FSM 两条边界清晰的状态链路。

## 1. 迁移目标

`temp/fsm` 当前把整车状态定义为一个 ETL HFSM，叶子状态同时编码：

- 整车工作域：`Disabled / Service / Combat`
- 服务模式：`ChassisAndGimbalWithFire / ChassisAndGimbalSafe`
- 底盘运动：`LowLeg / MidLeg / HighLeg / Spin / JumpPrep / JumpPush / JumpRecover`
- 恢复流程：`RecoveryFallCheck / RecoverySelfRight`
- 云台和火控许可：云台使能、目标源选择、是否允许开火

`wheel_legged` 新方案保留这些语义，但拆成：

```text
control_loop.cc
  -> 语义化输入 ModeRequest
  -> chassis::Fsm   // 底盘运动状态
  -> gimbal::Fsm    // 云台/火控/目标源状态
  -> chassis / gimbal 控制器
```

两个 FSM 不互相持有指针，也不直接调用对方。需要共享的信息由 `control_loop.cc` 在单个控制周期内显式传递。

## 2. 公共语义定义

建议抽出公共语义枚举，供 `control_loop.cc`、`chassis::Fsm` 和 `gimbal::Fsm` 共用。

```cpp
enum class DomainRequest : uint8_t {
  kDisabled = 0,
  kService,
  kCombat,
};

enum class ServiceProfile : uint8_t {
  kChassisAndGimbalWithFire = 0,
  kChassisAndGimbalSafe,
};

enum class LegProfile : uint8_t {
  kLow = 0,
  kMid,
  kHigh,
};

enum class TargetSource : uint8_t {
  kRc = 0,
  kHost,
};
```

建议统一输入快照保留 `temp` 的字段语义：

```cpp
struct ModeRequest {
  bool input_valid{false};
  DomainRequest domain_request{DomainRequest::kDisabled};
  ServiceProfile service_profile{ServiceProfile::kChassisAndGimbalWithFire};
  LegProfile leg_request{LegProfile::kLow};

  bool spin_hold{false};
  bool jump_trigger{false};
  bool fire_request{false};
  float current_leg_length_m{0.0f};

  TargetSource target_source{TargetSource::kRc};
  GimbalTarget rc_target{};
  GimbalTarget host_target{};
  bool host_target_valid{false};

  bool fall_detected{false};
  uint32_t fall_detected_hold_ms{0};
  bool upright_stable{false};

  uint32_t tick_ms{0};
};
```

`jump_trigger` 按一次性触发语义处理。DR16、键鼠或上位机输入应在进入 FSM 之前完成边沿检测或锁存释放。

## 3. temp 语义摘要

`temp/fsm` 的核心规则如下：

- `input_valid == false` 或 `domain_request == kDisabled` 时进入 `kDisabled`。
- `kService + kChassisAndGimbalWithFire` 进入 `kServiceWithFire*` 系列，底盘和云台使能，允许开火。
- `kService + kChassisAndGimbalSafe` 进入 `kServiceSafe*` 系列，底盘和云台使能，不允许开火。
- `kCombat` 进入 `kCombat*` 系列，底盘和云台使能，允许开火。
- 普通行走状态按腿长请求分为 `LowLeg / MidLeg / HighLeg`。
- `spin_hold == true` 时从普通行走状态进入对应 `Spin`，释放后回到当前请求的普通行走状态。
- `jump_trigger == true` 且当前请求为低腿长时进入跳跃链路：`JumpPrep -> JumpPush -> JumpRecover`。
- 跳跃准备持续 `450 ms`；蹬伸阶段在腿长达到 `0.30 m` 或持续 `1000 ms` 后进入恢复；跳跃恢复持续 `450 ms`。
- 非跳跃状态检测到跌倒后进入 `RecoveryFallCheck`；跌倒保持 `220 ms` 后进入 `RecoverySelfRight`；恢复稳定后回到当前请求的低腿长状态；自恢复超时 `2200 ms` 后进入 `Disabled`。

`temp` 使用的目标腿长常量：

| 语义 | 目标腿长 |
|---|---:|
| LowLeg | `0.18 m` |
| MidLeg | `0.24 m` |
| HighLeg | `0.30 m` |
| JumpPrep | `0.13 m` |
| JumpPush | `0.36 m` |
| JumpRecover | `0.20 m` |

## 4. 新底盘 FSM 定义

底盘 FSM 只表达底盘运动和恢复状态，不表达火控许可，也不区分云台目标源。

```cpp
namespace chassis {

class Fsm {
 public:
  enum class State : uint8_t {
    kDisabled = 0,
    kLowLeg,
    kMidLeg,
    kHighLeg,
    kSpin,
    kJumpPrep,
    kJumpPush,
    kJumpRecover,
    kRecoveryFallCheck,
    kRecoverySelfRight,
  };
};

}  // namespace chassis
```

### 4.1 底盘输入

底盘输入建议从公共 `ModeRequest` 派生：

- `input_valid`
- `domain_request`
- `service_profile`
- `leg_request`
- `spin_hold`
- `jump_trigger`
- `current_leg_length_m`
- `fall_detected`
- `fall_detected_hold_ms`
- `upright_stable`
- `tick_ms`

`domain_request == kDisabled` 时，底盘请求解析为 `kDisabled`。

### 4.2 底盘转移规则

底盘状态转移保留 `temp` 的运动语义：

- 失能优先级最高：输入无效或整车失能时，进入 `kDisabled`。
- 普通请求由 `leg_request` 解析为 `kLowLeg / kMidLeg / kHighLeg`。
- `spin_hold` 只在普通腿长状态中进入 `kSpin`；释放后回到当前请求的普通腿长状态。
- `jump_trigger` 只从低腿长请求进入 `kJumpPrep`，并记录跳跃结束后要返回的低腿长上下文。
- 跳跃链路按 `temp` 的时间和腿长阈值推进。
- 非跳跃状态中允许进入恢复链路：`kRecoveryFallCheck -> kRecoverySelfRight`。
- 恢复完成后回到当前请求的低腿长状态；恢复超时回到 `kDisabled`。

### 4.3 底盘状态描述

| 底盘状态 | 说明 |
|---|---|
| `kDisabled` | 底盘完全失能；DM 不使能，输出安全模式。 |
| `kLowLeg` | 正常移动低腿长模式，目标腿长 `0.18 m`。 |
| `kMidLeg` | 正常移动中腿长模式，目标腿长 `0.24 m`。 |
| `kHighLeg` | 正常移动高腿长模式，目标腿长 `0.30 m`。 |
| `kSpin` | 小陀螺模式；轮系执行自旋语义，腿长保持当前请求档位。 |
| `kJumpPrep` | 跳跃蓄力阶段，收腿至 `0.13 m`。 |
| `kJumpPush` | 跳跃蹬伸阶段，伸腿目标 `0.36 m`。 |
| `kJumpRecover` | 跳跃恢复阶段，腿长回收至 `0.20 m`。 |
| `kRecoveryFallCheck` | 跌倒确认阶段，等待确认是否进入自恢复。 |
| `kRecoverySelfRight` | 自恢复执行阶段，底盘主动翻正并限制其他动作。 |

### 4.4 底盘输出

```cpp
struct ControlOutput {
  bool enable_dm{false};
  bool run_chassis_update{false};
  bool spin_enable{false};
  bool recovery_enable{false};
  bool safe_output_required{true};
  LegProfile leg_profile{LegProfile::kLow};
  float target_leg_length_m{0.18f};
  uint8_t jump_phase{0};
};
```

输出映射：

| 底盘状态 | enable/run | spin | recovery | leg_profile | target_leg_length | jump_phase |
|---|---|---|---|---|---:|---:|
| `kDisabled` | false | false | false | Low | `0.18` | 0 |
| `kLowLeg` | true | false | false | Low | `0.18` | 0 |
| `kMidLeg` | true | false | false | Mid | `0.24` | 0 |
| `kHighLeg` | true | false | false | High | `0.30` | 0 |
| `kSpin` | true | true | false | 当前请求 | 按当前请求 | 0 |
| `kJumpPrep` | true | false | false | Low | `0.13` | 1 |
| `kJumpPush` | true | false | false | Low | `0.36` | 2 |
| `kJumpRecover` | true | false | false | Low | `0.20` | 3 |
| `kRecoveryFallCheck` | true | false | true | Low | `0.18` | 0 |
| `kRecoverySelfRight` | true | false | true | Low | `0.18` | 0 |

现有 `kStandby / kBalance` 不再作为对外状态保留。`kBalance + LegLengthMode` 合并为显式的 `kLowLeg / kMidLeg / kHighLeg`，以贴近 `temp` 的叶子状态语义。

## 5. 新云台 FSM 定义

云台 FSM 负责整车工作域中与云台、火控、目标源相关的部分。

```cpp
namespace gimbal {

class Fsm {
 public:
  enum class State : uint8_t {
    kDisabled = 0,
    kServiceWithFire,
    kServiceSafe,
    kCombat,
    kRecoveryAlign,
  };
};

}  // namespace gimbal
```

### 5.1 云台输入

云台输入建议从公共 `ModeRequest` 派生：

- `input_valid`
- `domain_request`
- `service_profile`
- `target_source`
- `rc_target`
- `host_target`
- `host_target_valid`
- `fire_request`
- `chassis_recovery_active`

其中 `chassis_recovery_active` 由 `control_loop.cc` 根据底盘 FSM 输出生成，云台 FSM 不直接访问底盘对象。

### 5.2 云台转移规则

- 输入无效或整车失能时进入 `kDisabled`。
- 底盘处于恢复状态时进入 `kRecoveryAlign`，云台对齐底盘前向，火控关闭。
- `domain_request == kService` 时：
  - `kChassisAndGimbalWithFire` -> `kServiceWithFire`
  - `kChassisAndGimbalSafe` -> `kServiceSafe`
- `domain_request == kCombat` 时进入 `kCombat`。
- `TargetSource::kHost` 不是云台公开状态，只影响输出目标源；当上位机目标无效时回退到 `kRc`。

### 5.3 云台状态描述

| 云台状态 | 说明 |
|---|---|
| `kDisabled` | 云台失能；不跟随、不瞄准、不允许开火。 |
| `kServiceWithFire` | 服务域带火控模式；允许开火，目标源可在 RC/Host 间切换。 |
| `kServiceSafe` | 服务域安全模式；云台使能但禁止开火。 |
| `kCombat` | 作战模式；云台使能并允许开火，目标源可在 RC/Host 间切换。 |
| `kRecoveryAlign` | 底盘恢复联动模式；云台对齐底盘前向，禁止开火。 |

### 5.4 云台输出

```cpp
struct ControlOutput {
  bool gimbal_enable{false};
  bool fire_allowed{false};
  bool shoot_request{false};
  bool align_to_chassis_forward{false};
  TargetSource active_target_source{TargetSource::kRc};
  GimbalTarget gimbal_target{};
};
```

输出映射：

| 云台状态 | gimbal_enable | fire_allowed | shoot_request | align_to_chassis_forward |
|---|---|---|---|---|
| `kDisabled` | false | false | false | false |
| `kServiceWithFire` | true | true | `fire_request` | false |
| `kServiceSafe` | true | false | false | false |
| `kCombat` | true | true | `fire_request` | false |
| `kRecoveryAlign` | true | false | false | true |

目标源规则：

- `target_source == kHost && host_target_valid == true` 时，输出 `active_target_source = kHost`，目标为 `host_target`。
- 其他情况输出 `active_target_source = kRc`，目标为 `rc_target`。

现有 `kManualControl / kHostControl` 不再作为公开状态保留，改为 `active_target_source` 输出字段。

## 6. temp 状态到拆分状态的映射

拆分后，`temp` 的叶子状态可以由云台状态和底盘状态组合还原。

| temp 状态 | gimbal::State | chassis::State |
|---|---|---|
| `kDisabled` | `kDisabled` | `kDisabled` |
| `kServiceWithFireLowLeg` | `kServiceWithFire` | `kLowLeg` |
| `kServiceWithFireMidLeg` | `kServiceWithFire` | `kMidLeg` |
| `kServiceWithFireHighLeg` | `kServiceWithFire` | `kHighLeg` |
| `kServiceWithFireSpin` | `kServiceWithFire` | `kSpin` |
| `kServiceWithFireJumpPrep` | `kServiceWithFire` | `kJumpPrep` |
| `kServiceWithFireJumpPush` | `kServiceWithFire` | `kJumpPush` |
| `kServiceWithFireJumpRecover` | `kServiceWithFire` | `kJumpRecover` |
| `kServiceSafeLowLeg` | `kServiceSafe` | `kLowLeg` |
| `kServiceSafeMidLeg` | `kServiceSafe` | `kMidLeg` |
| `kServiceSafeHighLeg` | `kServiceSafe` | `kHighLeg` |
| `kServiceSafeSpin` | `kServiceSafe` | `kSpin` |
| `kServiceSafeJumpPrep` | `kServiceSafe` | `kJumpPrep` |
| `kServiceSafeJumpPush` | `kServiceSafe` | `kJumpPush` |
| `kServiceSafeJumpRecover` | `kServiceSafe` | `kJumpRecover` |
| `kCombatLowLeg` | `kCombat` | `kLowLeg` |
| `kCombatMidLeg` | `kCombat` | `kMidLeg` |
| `kCombatHighLeg` | `kCombat` | `kHighLeg` |
| `kCombatSpin` | `kCombat` | `kSpin` |
| `kCombatJumpPrep` | `kCombat` | `kJumpPrep` |
| `kCombatJumpPush` | `kCombat` | `kJumpPush` |
| `kCombatJumpRecover` | `kCombat` | `kJumpRecover` |
| `kRecoveryFallCheck` | `kRecoveryAlign` | `kRecoveryFallCheck` |
| `kRecoverySelfRight` | `kRecoveryAlign` | `kRecoverySelfRight` |

调试时可以新增一个只读的组合状态枚举或字符串，用于复现 `temp` 的整车状态视角；控制逻辑仍以两个 FSM 输出为准。

## 7. control_loop 侧协调规则

`control_loop.cc` 的职责是把原始输入转成公共语义输入，并调度两个 FSM：

```text
DR16 / host / sensor feedback
  -> ModeRequest
  -> chassis::Fsm::Update()
  -> chassis_recovery_active
  -> gimbal::Fsm::Update()
  -> chassis controller / gimbal controller
  -> actuator output
```

协调规则：

- `ModeRequest` 是唯一的上层语义输入，两个 FSM 都只读这个快照。
- `chassis_recovery_active = chassis_state == kRecoveryFallCheck || chassis_state == kRecoverySelfRight`。
- 云台恢复对齐只由 `chassis_recovery_active` 触发，不反向修改底盘状态。
- 火控许可只由云台 FSM 输出，底盘 FSM 不判断是否允许开火。
- 底盘使能只由底盘 FSM 输出，云台 FSM 不直接控制底盘电机。

## 8. 预计代码更改范围

后续实现时建议按以下文件推进：

| 文件 | 更改内容 |
|---|---|
| `app/targets/wheel_legged/include/chassis/fsm.hpp` | 替换底盘状态枚举、输入输出定义，使用 `temp` 语义 |
| `app/targets/wheel_legged/fsm.cc` | 改成与 `temp` 类似的 ETL HFSM 或等价层级实现 |
| `app/targets/wheel_legged/include/gimbal/fsm.hpp` | 替换云台状态枚举、输入输出定义 |
| `app/targets/wheel_legged/gimbal_fsm.cc` | 按 Service/Combat/Safe/RecoveryAlign 生成云台和火控输出 |
| `app/targets/wheel_legged/control_loop.cc` | 新增公共语义输入构建、调度两个 FSM、生成组合调试状态 |
| `app/targets/wheel_legged/chassis.cc` | 适配新的底盘状态枚举和目标腿长常量 |

建议优先完成 FSM 接口和调试输出，再迁移控制器调用，避免一次性同时修改状态定义、控制算法和硬件下发。

## 9. 与当前实现的主要差异

- `wheel_legged` 当前底盘 FSM 的 `kBalance + LegLengthMode` 改为显式 `kLowLeg / kMidLeg / kHighLeg`。
- 当前底盘腿长目标 `0.15 / 0.20 / 0.23` 改为 `temp` 的 `0.18 / 0.24 / 0.30`。
- 当前云台 `kManualControl / kHostControl` 改为输出字段 `active_target_source`，云台状态改为表达 Service/Combat/Safe/Fire 许可。
- 不再保留“云台和火控可用、底盘不使能”的服务态语义。
- `Recovery` 不属于云台本体运动状态，但云台 FSM 用 `kRecoveryAlign` 响应底盘恢复状态，关闭火控并对齐底盘前向。

## 10. DR16 与状态机语义映射（定义）

本节给出当前方案下的 **DR16 -> ModeRequest -> chassis/gimbal FSM** 映射定义。映射基线参考 `app/targets/temp/inputs/inputs.cc`，并满足本次约束：不提供“云台和火控可用、底盘不使能”模式。

### 10.1 输入有效性

- `dr16.online == false`：`input_valid = false`，两 FSM 进入 `kDisabled`。
- `dr16.online == true`：`input_valid = true`，按下述规则解析。

### 10.2 左拨杆 `switch_l` -> `DomainRequest`

| DR16 左拨杆 | DomainRequest | 语义 |
|---|---|---|
| `Up` | `kCombat` | 作战域：底盘/云台使能，火控按云台状态机许可。 |
| `Mid` | `kService` | 服务域：底盘/云台使能。 |
| `Down` / `Unknown` | `kDisabled` | 整车失能。 |

说明：与 `temp/inputs` 的 `ToDomainRequest()` 一致。

### 10.3 右拨杆 `switch_r` -> `LegProfile`

| DR16 右拨杆 | LegProfile |
|---|---|
| `Down` / `Unknown` | `kLow` |
| `Mid` | `kMid` |
| `Up` | `kHigh` |

说明：与 `temp/inputs` 的 `ToLegRequest()` 一致。

### 10.4 拨轮 `dial` -> `spin_hold / jump_trigger`

沿用 `temp/inputs` 阈值与锁存：

- `spin_hold = (dial >= 220)`。
- `wheel_action_trigger` 触发条件：`dial <= -320` 且动作已上膛。
- 动作上膛复位条件：`abs(dial) <= 80`。
- 当前 `wheel_action_mode = kJump`，因此 `jump_trigger = wheel_action_trigger`。
- `fire_request` 不由 DR16 拨轮触发（保持 `false`，后续可由上位机或键鼠路径补充）。

### 10.5 `ServiceProfile` 与 gimbal/chassis 语义

- DR16 语义下不提供 `GimbalOnly` 分支。
- `domain_request == kService` 时默认 `service_profile = kChassisAndGimbalWithFire`。
- `kChassisAndGimbalSafe` 保留为可扩展语义（例如上位机/键鼠注入），但不由当前 DR16 三档直接进入。

因此 DR16 直控下允许的整车组合为：

- `kDisabled + kDisabled`
- `kServiceWithFire + {kLowLeg, kMidLeg, kHighLeg, kSpin, kJumpPrep, kJumpPush, kJumpRecover, kRecoveryFallCheck, kRecoverySelfRight}`
- `kCombat + {kLowLeg, kMidLeg, kHighLeg, kSpin, kJumpPrep, kJumpPush, kJumpRecover, kRecoveryFallCheck, kRecoverySelfRight}`

## 11. 未在本阶段实现的内容

本文只定义迁移方案和新语义，不修改现有 C++ 实现。后续实现前还需要确认：

- 公共语义枚举放在独立头文件，还是分别放在两个 FSM 头文件中。
- `fall_detected_hold_ms` 由状态估计层提供，还是由 `control_loop.cc` 计时生成。
- 是否需要保留一个 `TempEquivalentState` 只读调试枚举，用于和 `temp/fsm` 的日志、示波变量对齐。
