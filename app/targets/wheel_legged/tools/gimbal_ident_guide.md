# 云台动力学参数辨识指南

## 概述

云台辨识用于标定二自由度云台的 9 个动力学参数，为前馈控制提供准确的模型。Pitch 重力与摩擦首先通过多速度正反扫角联合辨识，再辨识惯量和耦合参数，最后综合验证。

### 9 参数一览

| 索引 | 符号 | 物理意义 | 辨识步骤 |
|------|------|----------|----------|
| theta[0] | I1zz_com | Yaw 轴等效惯量（含 Pitch 耦合分量） | Step 4 coupling |
| theta[1] | I2xx_com | Pitch 轴绕 x 的耦合惯量 | Step 4 coupling |
| theta[2] | I2yy_com | Pitch 轴等效惯量 | Step 3 pitch-inertia |
| theta[3] | m2·l2x | 水平偏心距 × Pitch 体质量 | Step 2 friction 联合辨识 |
| theta[4] | m2·l2z | 垂直偏心距 × Pitch 体质量 | Step 2 friction 联合辨识 |
| theta[5] | fv1 | Yaw 粘性摩擦系数 | Step 2 friction |
| theta[6] | fc1 | Yaw 库仑摩擦系数 | Step 2 friction |
| theta[7] | fv2 | Pitch 粘性摩擦系数 | Step 2 friction |
| theta[8] | fc2 | Pitch 库仑摩擦系数 | Step 2 friction |

---

## 依赖环境

- Python 3.8+
- `numpy`, `pandas`, `scipy`, `scikit-learn`, `pyserial`

```bash
pip install numpy pandas scipy scikit-learn pyserial
```

---

## 整体流程

```
Step 1 (gravity)      → 静态重力曲线检查（可选）
        ↓
Step 2 (friction)     → Pitch 重力/偏置/摩擦联合辨识 + Yaw 摩擦辨识
        ↓
Step 3 (pitch-inertia) → theta[2]
        ↓
Step 4 (coupling)     → theta[0], theta[1]
        ↓
Step 5 (verify)       → 综合验证全部 9 参数
```

Pitch 的 theta[3]、theta[4]、常值偏置 C、theta[7]、theta[8] 由 Step 2
同一批多速度正反扫角数据联合求解。Step 1 不再是必需的参数输入，只用于静态交叉验证。

---

## 激活辨识模式

辨识模式由**遥控器拨杆组合**激活，仅改 `params.hpp` 中的 `kIdentSubMode` 不会自动运行。

| 遥控器操作 | 模式 |
|------------|------|
| 左拨杆 **DOWN** + 右拨杆 **UP** | 辨识模式 (`kIdent`) |
| 左拨杆 **DOWN** + 右拨杆 **MID** | 前馈验证模式 (`kFfVerify`) |
| 左拨杆其他位置 | 正常工作模式 |

**每次上电后需要重新拨杆激活。**

---

## Step 1 — 静态重力项辨识 (theta[3], theta[4])

**目的**：利用 Pitch 轴在多个不同角度静止悬停时的力矩数据，分离出云台重心偏心引起的重力矩。

**原理**：

理想、无静摩擦时，Pitch 轴静止力矩满足：

```
tau2 = -g * theta[3] * cos(q2) - g * theta[4] * sin(q2)
```

实际静止状态还存在方向相关的静摩擦，因此本步骤采用往返平均，只用于检查重力曲线和联合辨识结果，不再作为 Step 2 的必需输入。

### 操作步骤

#### 1.1 修改固件配置

在 `params.hpp` 中，找到对应机器人的 `gimbal_ident` 命名空间，将子模式设为 `kGravity`：

```cpp
static constexpr IdentSubMode kIdentSubMode = IdentSubMode::kGravity;
```

确认以下参数适合你的云台：

```cpp
constexpr float kGravityPitchAngles[] = {...};  // Pitch 静止角度序列 (rad)
constexpr size_t kGravityAngleCount = ...;       // 角度数量
constexpr float kGravityHoldDuration = 2.0f;     // 每角度测量时间 (s)
constexpr float kGravitySettleDuration = 0.5f;   // 到位稳定时间 (s)
```

- 角度序列应覆盖 Pitch 轴的工作行程，建议 7~9 个均匀分布的角度
- 测量时间建议 ≥ 2s，确保力矩数据稳定

#### 1.2 编译烧录固件，上电运行

上电后，**遥控器左拨杆 DOWN + 右拨杆 UP** 激活辨识模式。云台先按数组顺序正向扫描全部 Pitch 角度，再按相反顺序扫描回起点；转折端不重复停留。除两端外，每个角度都会分别从两个方向进入并测量，可减小静摩擦和线缆迟滞对重力项的影响。全程 Yaw 轴保持当前位置不动。

若角度数组包含 `N` 个角度，一次辨识共记录 `2N-1` 个静止段。请等待云台反向返回起始角度后再停止采集。

#### 1.3 采集数据

用串口线连接 STM32 的辨识串口（UART7），运行数据采集脚本：

```bash
python serial_collect.py gravity --port COM6
```

脚本将数据保存到 `gravity_static.csv`。**等待云台完成全部角度后按 Ctrl+C 停止**。

CSV 列名：`time, q1, q2, dq1, dq2, tau1, tau2`

- q1/q2 = Yaw/Pitch 电机位置 (rad)
- dq1/dq2 = Yaw/Pitch 电机速度 (rad/s)
- tau1/tau2 = Yaw/Pitch 电机输出力矩 (Nm)

#### 1.4 运行辨识

```bash
python identify_gimbal.py gravity gravity_static.csv
```

**输出示例**：
```
Step 1: 静态重力项辨识  →  theta[3], theta[4]
  使用静止段数: 7
    [0] q2=-40.0°  tau2=0.1234  (行 50-200)
    [1] q2=-30.0°  tau2=0.0890  (行 250-400)
    ...

  拟合: tau2 = A*cos(q2) + B*sin(q2) + C
    A  = -0.265432
    B  = 0.254321
    C  = 0.001234
    R² = 0.9876

  辨识结果:
    theta[3] (m2*l2x, 水平偏心) = 0.027061
    theta[4] (m2*l2z, 垂直偏心) = -0.025930
```

**注意**：如果常值偏置 C 过大（> 0.1 * max(|A|,|B|)），说明可能存在电流零偏、线缆拉力或编码器零位问题，需要检查。

---

## Step 2 — Pitch 重力/摩擦联合辨识 + Yaw 摩擦辨识

**目的**：让 Pitch 以多个正负恒定速度扫过同一个角度范围，同时辨识重力、常值偏置、粘性摩擦和库仑摩擦；随后使用多个速度档辨识 Yaw 摩擦。

**原理**：

Pitch 匀速运动时惯性项为零，联合模型为：

```
tau2 = A*cos(q2) + B*sin(q2) + C
     + fv2*dq2 + fc2*tanh(dq2 / 0.02)
```

其中 `A=-g*theta[3]`、`B=-g*theta[4]`。`tanh(dq/0.02)` 用于平滑近似符号函数，0.02 rad/s 必须与 C++ 动力学模型一致。

> 注意：摩擦基函数已由旧版 `tanh(dq)` 改为 `tanh(dq/0.02)`，旧的 theta[6]/theta[8] 数值不能直接沿用，必须重新辨识。

### 操作步骤

#### 2.1 修改固件配置

```cpp
static constexpr IdentSubMode kIdentSubMode = IdentSubMode::kFriction;
```

确认摩擦参数：

```cpp
constexpr float kFrictionPitchVelocitiesRadS[] = {0.05f, 0.10f, 0.20f, 0.30f};
constexpr size_t kFrictionPitchVelocityCount = 4;
constexpr float kFrictionYawVelocitiesRadS[] = {0.05f, 0.10f, 0.20f, 0.30f};
constexpr size_t kFrictionYawVelocityCount = 4;
constexpr float kFrictionYawTravelRad = 1.0f;
constexpr float kFrictionPauseDuration = 0.5f;
```

- 每个 Pitch 速度档均执行 Top→Bottom 正扫和 Bottom→Top 反扫
- 准备、端点停顿和转向阶段不发送 CSV
- 多个速度幅值用于分离粘性摩擦与库仑摩擦

#### 2.2 编译烧录，上电运行

上电后，**遥控器左拨杆 DOWN + 右拨杆 UP** 激活辨识模式。云台先移动到 Pitch Top 保护端，再按速度数组逐档完成正反向全行程扫描；完成后回中，再按多个速度档完成 Yaw 正反转。

#### 2.3 采集数据

```bash
python serial_collect.py friction --port COM6
```

保存到 `friction_sweep.csv`。**等待全部扫角完成后按 Ctrl+C 停止**。

#### 2.4 运行辨识

```bash
python identify_gimbal.py friction friction_sweep.csv
```

输出会同时打印 theta[3]、theta[4]、C、theta[5..8]，以及下一步所需的完整命令。

---

## Step 3 — Pitch 惯量辨识 (theta[2])

**目的**：Yaw 轴固定不动，Pitch 轴依次执行多个频率的正弦运动，通过同一角度下不同加速度幅值解除重力与惯量耦合。

**原理**：

```
tau2 = A*cos(q2) + B*sin(q2) + C
     + I2yy_com*ddq2 + fv2*dq2 + fc2*tanh(dq2/vs)
```

固件记录 DWT 实测控制周期、目标位置/速度/加速度、频率和周期编号。脚本对实际位置做逐频率谐波拟合并解析求导，不再直接差分原始 DM 速度；随后对全部频率做带物理约束的鲁棒联合回归。

### 操作步骤

#### 3.1 修改固件配置

```cpp
static constexpr IdentSubMode kIdentSubMode = IdentSubMode::kPitchInertia;
```

确认参数：

```cpp
constexpr float kPitchInertiaFrequenciesHz[] = {0.4f, 0.7f, 1.0f};
constexpr float kPitchInertiaAmplitude = 0.15f;
constexpr uint16_t kPitchInertiaWarmupCycles = 2;
constexpr uint16_t kPitchInertiaRecordCycles = 6;
```

#### 3.2 编译烧录，上电运行

上电后，**遥控器左拨杆 DOWN + 右拨杆 UP** 激活辨识模式。Pitch 先平滑回中，然后依次执行 0.4/0.7/1.0 Hz；每档前 2 个周期只用于稳定，随后 6 个周期标记为有效数据。

#### 3.3 采集数据

```bash
# 保存到自定义文件名以便区分
python serial_collect.py dynamic --port COM6 --output dynamic_pitch.csv
```

#### 3.4 运行辨识

```bash
python identify_gimbal.py pitch-inertia dynamic_pitch.csv \
    --theta34 0.027061,-0.025930 \
    --theta78 0.3,0.04 \
    --pitch-bias 0.10
```

PowerShell 中负数逗号参数应使用 `--theta34=-0.1,0.09` 形式，续行符应使用反引号而不是反斜杠。`--theta34/--theta78/--pitch-bias` 用于旧 CSV 回退和结果对照；新版 CSV 的主结果来自多频联合回归。

脚本会同时打印实测控制周期、各频率位置拟合质量、位置导数与 DM 速度比例、分频率惯量、正负加速度惯量和残差偏置。只有这些检查通过时才建议采用 theta[2]。

---

## Step 4 — 耦合惯量辨识 (theta[0], theta[1]) — 两步法

**目的**：在不同 Pitch 固定角度下，Yaw 做大振幅正弦加减速（接近 360°），采用两步法辨识 Yaw-Pitch 耦合惯量。

**原理**：

Yaw 轴的有效惯量随 Pitch 角度变化。当 Pitch 固定 (dq2≈0) 时：

```
I_eff(q2) = I1zz_com * sin²(q2) + I2xx_com * cos²(q2)
          = I2xx_com + (I1zz_com - I2xx_com) * sin²(q2)

tau1 = I_eff(q2) * ddq1 + fv1*dq1 + fc1*tanh(dq1/vs)
```

旧版全局双参数回归在单个角度块内存在共线性问题（两个 regressor 是同一正弦波的不同缩放）。**两步法**避免了这个问题：

- **Step A — 每角度独立拟合 I_eff**：在每个 Pitch 角度下，扣除摩擦后做单参数回归 `tau1_residual = I_eff * ddq1`。单参数回归无共线性问题。
- **Step B — 跨角度拟合 I1zz / I2xx**：将各角度的 I_eff 对 sin²(q2) 做线性回归，截距=I2xx，斜率=I1zz-I2xx。

### 关键参数说明

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| `kCouplingFreqHz` | **1.0 Hz** | 越高 ddq1 信号越强（∝ω²），但需确保 PID 能跟踪 |
| `kCouplingAmplitude` | **2.5 rad (143°)** | 大振幅接近 360° 峰峰值，提升谐波拟合质量 |
| `kCouplingPitchAngles` | 覆盖全行程 | 角度范围和数量决定 Step B 的可靠性 |
| `kCouplingDurationPerAngle` | 8.0 s | 1 Hz 下 2 warmup + 6 record 周期 |

> **注意**：大振幅下 Yaw 电机位置会跨过 ±π 跳变。固件的 `WrapYawTarget` 已正确处理目标 wrapping；Python 脚本在谐波拟合前用 `np.unwrap()` 恢复连续位置信号。

### 操作步骤

#### 4.1 修改固件配置

```cpp
static constexpr IdentSubMode kIdentSubMode = IdentSubMode::kCoupling;
```

确认参数（以步兵 3 号为例，`params.hpp` 中 `infantry3::gimbal_ident` 命名空间）：

```cpp
constexpr float kCouplingPitchAngles[] = {0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f, 1.1f};
constexpr size_t kCouplingAngleCount = 11;
constexpr float kCouplingFreqHz = 1.0f;            // Yaw 正弦频率 [Hz]
constexpr float kCouplingAmplitude = 2.5f;          // Yaw 正弦幅值 [rad] (~143°, 峰峰值 ~286°)
constexpr float kCouplingDurationPerAngle = 8.0f;   // 每角度持续时间 [s]
constexpr uint16_t kCouplingWarmupCycles = 2;
constexpr uint16_t kCouplingRecordCycles = 6;
```

- Pitch 角度应覆盖云台工作行程（建议 ≥ 30° 范围，≥ 5 个角度）
- 大振幅前请确认云台机械限位允许 yaw 轴 ±160° 以上的摆动范围

#### 4.2 编译烧录，上电运行

上电后，**遥控器左拨杆 DOWN + 右拨杆 UP** 激活辨识模式。云台依次在每个 Pitch 角度停留，期间 Yaw 做 1 Hz 大振幅正弦摆动。全程数据由固件自动分段（`frequency_index` 区分角度块，`valid_for_fit` 标记有效周期）。

#### 4.3 采集数据

**只需一次采集**，固件自动处理全部角度：

```bash
python serial_collect.py dynamic --port COM6 --output coupling_sweep.csv
```

等待云台走完所有角度后按 Ctrl+C 停止。单个 CSV 文件包含全部角度的数据。

#### 4.4 运行辨识

```bash
python identify_gimbal.py coupling coupling_sweep.csv --theta56 0.2,0.03
```

`--theta56` 填入 Step 2 辨识得到的 theta[5], theta[6]（Yaw 摩擦）。

**输出示例**：

```
Step 4: Yaw-Pitch coupling inertia (two-step method)

  --- Step A: Per-angle harmonic fit & I_eff regression ---
  ddq1_ref not available; using harmonic-fit ddq1 from q1 position.

  index   q2[deg]         I_eff     R2(A)      q-R2    A[rad]       n
  -----  --------  ------------  --------  --------  --------  ------
      0     -24.0     -0.069811    0.1042    0.9998     0.497     661
      1     -17.7     -0.072753    0.1168    0.9998     0.497     662
    ...

  --- Step B: Cross-angle fit I_eff = I2xx + (I1zz - I2xx) * sin^2(q2) ---

    Model: I_eff = I2xx + (I1zz - I2xx) * sin^2(q2)
    I2xx (intercept)     = -0.067453  kg.m2  -> theta[1]
    I1zz - I2xx (slope)  = -0.014865  kg.m2
    I1zz                 = -0.082318  kg.m2  -> theta[0]
    R^2(B)               = 0.3317
```

**输出解读**：

- **q-R²**：谐波拟合质量，应 > 0.95。偏低说明 yaw PID 跟踪不良（频率太高或振幅太大）
- **R²(A)**：每角度 I_eff 回归的决定系数。低频小振幅时偏低是正常的（信噪比不足），提高激励频率后可显著改善
- **R²(B)**：跨角度拟合的决定系数。受两个因素影响：q2 角度范围和 I_eff 估计精度
- **res%**：每角度 I_eff 与 Step B 预测值的残差百分比，应 < 10%
- 如果 Step B 的 R² < 0.5，说明 I1zz 和 I2xx 分离不可靠，需要更大的 q2 范围或更高的激励频率

---

## Step 5 — 综合验证

**目的**：使用全部 9 个参数，在五次谐波综合轨迹上对比模型预测力矩和实测力矩，评估辨识质量。

### 操作步骤

#### 5.1 修改固件配置

```cpp
static constexpr IdentSubMode kIdentSubMode = IdentSubMode::kHarmonic;
```

使用五次谐波轨迹同时激励 Yaw 和 Pitch 轴。

#### 5.2 编译烧录，上电运行

上电后，**遥控器左拨杆 DOWN + 右拨杆 UP** 激活辨识模式，云台做五次谐波运动。采集数据：

```bash
python serial_collect.py dynamic --port COM6 --output dynamic_harmonics.csv
```

#### 5.3 运行验证

```bash
python identify_gimbal.py verify dynamic_harmonics.csv \
    --theta 0.010,0.005,0.030,0.027,-0.026,0.200,0.030,0.300,0.040
```

9 个数值依次对应 theta[0] ~ theta[8]，即前四步的汇总结果。

**输出解读**：

- **R²**：越接近 1 越好。Yaw 和 Pitch 均应 ≥ 0.6，理想情况下 > 0.9
- **RMSE**：均方根误差。越小越好，通常在 0.1 Nm 量级
- **MAE**：平均绝对误差
- 脚本会自动给出诊断建议（如 R² 偏低应检查哪些参数）

---

## 参数导入固件

验证通过后，将 9 个参数填入 `params.hpp` 中对应机器人的 `kIdentTheta` 数组：

```cpp
constexpr float kIdentTheta[9] = {
    0.0100000f,  // theta[0] I1zz_com
    0.0050000f,  // theta[1] I2xx_com
    0.0300000f,  // theta[2] I2yy_com
    0.0270000f,  // theta[3] m2*l2x
   -0.0260000f,  // theta[4] m2*l2z
    0.2000000f,  // theta[5] fv1
    0.0300000f,  // theta[6] fc1
    0.3000000f,  // theta[7] fv2
    0.0400000f,  // theta[8] fc2
};
```

`identify_gimbal.py verify` 的最后一行会直接输出可复制的 C++ 初始化列表格式。

---

## 快速命令速查

```bash
# Step 1: 静态重力
python serial_collect.py gravity --port COM6
python identify_gimbal.py gravity gravity_static.csv

# Step 2: Pitch 重力/摩擦联合辨识 + Yaw 摩擦
python serial_collect.py friction --port COM6
python identify_gimbal.py friction friction_sweep.csv

# Step 3: Pitch 惯量
python serial_collect.py dynamic --port COM6 --output dynamic_pitch.csv
python identify_gimbal.py pitch-inertia dynamic_pitch.csv --theta34 <θ3>,<θ4> --theta78 <θ7>,<θ8> --pitch-bias <C>

# Step 4: 耦合惯量 (两步法，单次采集)
python serial_collect.py dynamic --port COM6 --output coupling_sweep.csv
python identify_gimbal.py coupling coupling_sweep.csv --theta56 <θ5>,<θ6>

# Step 5: 综合验证
python serial_collect.py dynamic --port COM6 --output dynamic_harmonics.csv
python identify_gimbal.py verify dynamic_harmonics.csv --theta θ0,θ1,...,θ8
```

---

## 故障排查

| 现象 | 可能原因 | 解决方法 |
|------|----------|----------|
| Step 1 静止段检测 < 3 | Pitch 角度不够多或数据噪声大 | 降低 `--dq-threshold`，增加角度数量 |
| Step 1 常值偏置 C 过大 | 电流零偏 / 编码器零位 / 线缆拉力 | 检查电机零点标定，排除线缆干扰 |
| Step 3 theta[2] 为负值 | 谐波拟合质量差或摩擦参数不准 | 检查 q-R² 是否 > 0.95，复查 theta[7]/theta[8] |
| Step 4 R²(A) 偏低 | 激励频率太低导致 ddq1 信噪比不足 | 提高 kCouplingFreqHz（建议 ≥ 1.0 Hz） |
| Step 4 R²(B) 偏低 | q2 角度范围太小或 I_eff 估计噪声大 | 增大 Pitch 角度范围（≥ 30°），增加角度数量 |
| Step 4 q-R² 偏低 | Yaw PID 跟踪不良 | 降低激励频率或振幅，检查 PID 参数 |
| Step 5 R² 偏低 (Yaw) | 耦合惯量或摩擦不准 | 复查 Step 2 的 theta[5]/theta[6] 和 Step 4 |
| Step 5 R² 偏低 (Pitch) | 重力、惯量或摩擦不准 | 复查 Step 1/2/3 |
| 力矩可能饱和 | 激励幅值过大 | 减小谐波幅值或降低加速度 |
| 串口无数据 | 串口号错误或波特率不匹配 | 检查 `--port` 参数 (默认 COM6)，确认 115200 |
