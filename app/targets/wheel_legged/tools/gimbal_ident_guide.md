# 云台动力学参数辨识指南

## 概述

云台辨识用于标定二自由度云台的 9 个动力学参数，为前馈控制提供准确的模型。辨识采用**分步递进**策略：先辨识独立的参数（重力、摩擦），再以此为基础辨识耦合参数（惯量），最后综合验证。

### 9 参数一览

| 索引 | 符号 | 物理意义 | 辨识步骤 |
|------|------|----------|----------|
| theta[0] | I1zz_com | Yaw 轴等效惯量（含 Pitch 耦合分量） | Step 4 coupling |
| theta[1] | I2xx_com | Pitch 轴绕 x 的耦合惯量 | Step 4 coupling |
| theta[2] | I2yy_com | Pitch 轴等效惯量 | Step 3 pitch-inertia |
| theta[3] | m2·l2x | 水平偏心距 × Pitch 体质量 | Step 1 gravity |
| theta[4] | m2·l2z | 垂直偏心距 × Pitch 体质量 | Step 1 gravity |
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
Step 1 (gravity)      → theta[3], theta[4]
        ↓
Step 2 (friction)     → theta[5], theta[6], theta[7], theta[8]
        ↓
Step 3 (pitch-inertia) → theta[2]
        ↓
Step 4 (coupling)     → theta[0], theta[1]
        ↓
Step 5 (verify)       → 综合验证全部 9 参数
```

每一步的输出是下一步的输入，依序进行。

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

Pitch 轴静止时（dq2 ≈ 0），电机力矩仅用于抵消重力：

```
tau2 = -g * theta[3] * cos(q2) - g * theta[4] * sin(q2)
```

通过采集多个 Pitch 角度的 tau2 数据，使用最小二乘法拟合出 theta[3]（水平偏心）和 theta[4]（垂直偏心）。

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

上电后，**遥控器左拨杆 DOWN + 右拨杆 UP** 激活辨识模式。云台将依次移动到每个 Pitch 角度并停留测量。全程 Yaw 轴保持当前位置不动。

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

## Step 2 — 低速摩擦项辨识 (theta[5..8])

**目的**：在重力项已知的前提下，通过 Yaw 和 Pitch 轴的分段匀速运动，辨识粘性摩擦和库仑摩擦。

**原理**：

匀速运动时惯性项为零，扣除重力后剩余的力矩主要由摩擦贡献：

```
tau = fv * dq + fc * tanh(dq)
```

其中 tanh 用于平滑近似符号函数（避免在 dq=0 处不连续）。

### 操作步骤

#### 2.1 修改固件配置

```cpp
static constexpr IdentSubMode kIdentSubMode = IdentSubMode::kFriction;
```

确认摩擦参数：

```cpp
constexpr float kFrictionVelocities[] = {0.05f, 0.10f, 0.20f, 0.40f};      // 正速度档 (rad/s)
constexpr float kFrictionVelocitiesNeg[] = {-0.05f, -0.10f, -0.20f, -0.40f}; // 负速度档
constexpr size_t kFrictionVelocityCount = 4;
constexpr float kFrictionConstVelDuration = 2.0f;  // 每段匀速持续时间 (s)
constexpr float kFrictionAccel = 0.5f;              // 加减速加速度 (rad/s²)
```

- 速度档位应覆盖低速范围（0.05 ~ 0.4 rad/s），保证摩擦效应显著
- 加速度不宜过大，避免惯性项干扰

#### 2.2 编译烧录，上电运行

上电后，**遥控器左拨杆 DOWN + 右拨杆 UP** 激活辨识模式。云台先扫 Yaw 轴（正转速 4 档 → 负转速 4 档），再扫 Pitch 轴（正负交替 8 档）。

#### 2.3 采集数据

```bash
python serial_collect.py friction --port COM6
```

保存到 `friction_sweep.csv`。**等待全部扫角完成后按 Ctrl+C 停止**。

#### 2.4 运行辨识

```bash
python identify_gimbal.py friction friction_sweep.csv --theta34 0.027061,-0.025930
```

其中 `--theta34` 填入 Step 1 辨识得到的 theta[3], theta[4]。

**输出将打印下一步所需的 `--theta78` 参数**。

---

## Step 3 — Pitch 惯量辨识 (theta[2])

**目的**：Yaw 轴固定不动，Pitch 轴做正弦加减速运动，从扣除了重力和摩擦的力矩残差中辨识 Pitch 等效惯量。

**原理**：

```
tau2_residual = tau2 - tau2_gravity - tau2_friction = I2yy_com * ddq2
```

线性回归 ddq2 → tau2_residual，斜率即为 theta[2]。

### 操作步骤

#### 3.1 修改固件配置

```cpp
static constexpr IdentSubMode kIdentSubMode = IdentSubMode::kPitchInertia;
```

确认参数：

```cpp
constexpr float kPitchInertiaFreqHz = 0.2f;     // 正弦频率 (Hz)
constexpr float kPitchInertiaAmplitude = 0.25f;  // 正弦幅值 (rad)
```

#### 3.2 编译烧录，上电运行

上电后，**遥控器左拨杆 DOWN + 右拨杆 UP** 激活辨识模式。云台 Yaw 锁定在 0 位，Pitch 绕中心角做正弦摆动。

#### 3.3 采集数据

```bash
# 保存到自定义文件名以便区分
python serial_collect.py dynamic --port COM6 --output dynamic_pitch.csv
```

#### 3.4 运行辨识

```bash
python identify_gimbal.py pitch-inertia dynamic_pitch.csv \
    --theta34 0.027061,-0.025930 \
    --theta78 0.3,0.04
```

`--theta78` 填入 Step 2 辨识得到的 theta[7], theta[8]（Pitch 摩擦）。

**输出将打印 theta[2]**。如果结果为负值，检查 ddq 信号质量（是否由噪声差分得到）或重力/摩擦扣除是否正确。

---

## Step 4 — 耦合惯量辨识 (theta[0], theta[1])

**目的**：在不同 Pitch 固定角度下，Yaw 做正弦加减速，辨识 Yaw-Pitch 耦合惯量项。

**原理**：

C++ 模型中 Yaw 轴的惯性力矩有两个分量，分别与 theta[0] 和 theta[1] 成正比。在不同 q2 角度下，二者的贡献不同，从而可以分离：

```
tau1_residual = theta[0] * (sin²(q2)*ddq1 + sin(2q2)*dq1*dq2)
              + theta[1] * (cos²(q2)*ddq1 - sin(2q2)*dq1*dq2)
```

### 操作步骤

#### 4.1 修改固件配置

```cpp
static constexpr IdentSubMode kIdentSubMode = IdentSubMode::kCoupling;
```

确认参数：

```cpp
constexpr float kCouplingPitchAngles[] = {0.70f, 0.94f, 1.20f, 1.50f};  // 多个 Pitch 固定角 (rad)
constexpr size_t kCouplingAngleCount = 4;
constexpr float kCouplingFreqHz = 0.2f;           // Yaw 正弦频率 (Hz)
constexpr float kCouplingAmplitude = 0.5f;         // Yaw 正弦幅值 (rad)
constexpr float kCouplingDurationPerAngle = 5.0f;  // 每角度持续时间 (s)
```

- Pitch 角度范围应尽量大（建议 ≥ 20°），否则 theta[0]/theta[1] 辨识不准

#### 4.2 编译烧录，上电运行

上电后，**遥控器左拨杆 DOWN + 右拨杆 UP** 激活辨识模式。云台依次在每个 Pitch 角度停留，每次停留期间 Yaw 做正弦摆动。

#### 4.3 采集数据

**重要**：需要分别采集每个 Pitch 角度下的数据（通过多次运行或分段时间裁剪），或使用一个包含全部角度的长数据文件后用脚本自动分段。

由于脚本支持多文件输入，推荐分次采集：

```bash
# Pitch = -40° 时
python serial_collect.py dynamic --port COM6 --output q2_m40.csv
# Pitch = 0° 时
python serial_collect.py dynamic --port COM6 --output q2_0.csv
# Pitch = +40° 时
python serial_collect.py dynamic --port COM6 --output q2_p40.csv
```

#### 4.4 运行辨识

```bash
python identify_gimbal.py coupling q2_m40.csv q2_0.csv q2_p40.csv \
    --theta56 0.2,0.03
```

`--theta56` 填入 Step 2 辨识得到的 theta[5], theta[6]（Yaw 摩擦）。

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

# Step 2: 低速摩擦
python serial_collect.py friction --port COM6
python identify_gimbal.py friction friction_sweep.csv --theta34 <θ3>,<θ4>

# Step 3: Pitch 惯量
python serial_collect.py dynamic --port COM6 --output dynamic_pitch.csv
python identify_gimbal.py pitch-inertia dynamic_pitch.csv --theta34 <θ3>,<θ4> --theta78 <θ7>,<θ8>

# Step 4: 耦合惯量
python serial_collect.py dynamic --port COM6 --output q2_m40.csv   # Pitch=-40°
python serial_collect.py dynamic --port COM6 --output q2_0.csv     # Pitch=0°
python serial_collect.py dynamic --port COM6 --output q2_p40.csv   # Pitch=+40°
python identify_gimbal.py coupling q2_m40.csv q2_0.csv q2_p40.csv --theta56 <θ5>,<θ6>

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
| Step 3 theta[2] 为负值 | ddq 信号由噪声差分得到 | 降低 SG 滤波的 window 参数，检查轨迹跟踪 |
| Step 4 theta[0] 不准 | q2 角度范围太小 | 增大 Pitch 固定角度范围（≥ ±30°） |
| Step 5 R² 偏低 (Yaw) | 耦合惯量或摩擦不准 | 复查 Step 2 的 theta[5]/theta[6] 和 Step 4 |
| Step 5 R² 偏低 (Pitch) | 重力、惯量或摩擦不准 | 复查 Step 1/2/3 |
| 力矩可能饱和 | 激励幅值过大 | 减小谐波幅值或降低加速度 |
| 串口无数据 | 串口号错误或波特率不匹配 | 检查 `--port` 参数 (默认 COM6)，确认 115200 |
