# Roll 轴与腿长全力分配 MPC 控制计划

## 1. 目标与边界

本文档描述一版用于轮腿机器人 `roll` 轴与腿长控制的 MPC 方案。该方案不是在现有腿长 PID、roll PID、重力前馈和转向惯性补偿外面再叠加一层修正，而是希望由 MPC 统一生成最终左右腿虚拟支持力：

```text
F_l, F_r = MPC(x, ref, v_x, omega_z, L_l, L_r)
```

然后继续复用现有 VMC / 雅可比映射：

```text
tau_l = J_l^T [F_l, T_p_l]^T
tau_r = J_r^T [F_r, T_p_r]^T
```

本方案的控制边界如下：

- MPC 接管常规支撑状态下的腿长控制、重力平衡、roll 控制和转向惯性预测。
- MPC 输出最终虚拟支持力 `F_l`、`F_r`，而不是 PID 的增量修正。
- LQR 的轮端力矩 `T_wl/T_wr` 与髋部等效摆角力矩 `T_bl/T_br` 第一版保留。
- VMC 雅可比映射第一版保留。
- 弹簧补偿不进入 MPC，不作为平衡力、不作为扰动模型、不作为代价项。
- 起立、跳跃、离地恢复、倒地自起、台阶任务等特殊动作第一版不由该 MPC 接管。
- 第一版直接集成 TinyMPC C++ 库，使用固定 `alpha = 0` 的 `A/B` 矩阵；腿摆角只用于重力平衡点与安全检查。

现有常规分支大致为：

```text
F_l =
  leg_length_pid_l
+ gravity_l
+ roll_pid
- turn_inertial_l
+ spring_l

F_r =
  leg_length_pid_r
+ gravity_r
- roll_pid
+ turn_inertial_r
+ spring_r
```

本方案希望将其中以下项统一进 MPC：

```text
leg_length_pid_l/r
gravity_l/r
roll_pid
turn_inertial_l/r
```

以下项不进入 MPC：

```text
spring_l/r
```

如果实车弹簧或机械弹性确实影响明显，第一版将其视为未建模扰动，由 MPC 反馈和后续系统辨识吸收。

## 2. 状态与坐标定义

左右腿等效腿长：

```text
L_l      左腿等效腿长
L_r      右腿等效腿长
dL_l     左腿腿长速度
dL_r     右腿腿长速度
```

平均腿长和腿长差：

```text
L_c  = (L_l + L_r) / 2
D    = L_r - L_l

dL_c = (dL_l + dL_r) / 2
dD   = dL_r - dL_l
```

左右腿长可由 `L_c` 和 `D` 还原：

```text
L_l = L_c - D / 2
L_r = L_c + D / 2
```

roll 轴状态：

```text
rho   = roll angle
drho  = roll angular velocity
```

### 2.1 腿摆角与 cos_min

腿向支持力 `F_l`、`F_r` 沿腿长方向（径向），并非竖直方向。当腿不竖直时，径向力只有一部分用于支撑重力，其余分量沿水平方向（该水平分量影响 pitch，第一版不建模，由 LQR 的髋力矩处理）。

定义腿摆角（腿相对世界系竖直方向的夹角）：

```text
alpha_l = theta_ll     左腿倾角（0 = 竖直向下）
alpha_r = theta_lr     右腿倾角（0 = 竖直向下）

alpha_avg = (alpha_l + alpha_r) / 2
```

`theta_ll`、`theta_lr` 来自现有状态估计器（`CurrentState`），定义已包含机体俯仰角 `theta_b` 的贡献，因此 `alpha_l`、`alpha_r` 就是腿在世界系中的绝对倾角。

径向力与竖直分量的关系：

```text
F_l_vertical = F_l * cos(alpha_l)
F_r_vertical = F_r * cos(alpha_r)
```

当 `alpha_l = alpha_r = 0`（腿竖直），`cos = 1`，模型退化为原文档的简化形式。当腿前摆或后摆（典型范围 ±30° 即 ±0.52 rad），`cos` 降至约 0.87，不可忽略。

第一版 TinyMPC 使用固定 `alpha = 0` 的 `A/B` 矩阵，不在线更新动力学矩阵。腿摆角只用于：

```text
1. 重力平衡点 F_g_l / F_g_r
2. MPC 启用安全检查
```

实车腿摆角不会超过 60 度，因此加入：

```text
cos_min = cos(60 deg) = 0.5
cos_l = max(cos(alpha_l), cos_min)
cos_r = max(cos(alpha_r), cos_min)
```

如果检测到腿摆角接近大角度工作区间，第一版应直接 fallback，而不是让固定 AB 的 MPC 继续接管。建议：

```text
theta_mpc_max = 45 deg
```

机械上限 60 度只作为 `cos_min = 0.5` 的数值保护。

腿长目标：

```text
L_ref = 当前 FSM / control_loop 给出的目标腿长
```

roll 目标：

```text
rho_ref = 标定后的 roll 平衡角
```

第一版可复用当前参数中的 `kRollBalanceTargetRad`，而不是强行取 0。这样可以兼容 IMU 安装偏置、机械左右不对称和实车静态平衡偏置。

误差状态：

```text
e_L   = L_c - L_ref
e_rho = rho - rho_ref
```

MPC 状态定义为：

```text
x = [
  e_L,
  dL_c,
  D,
  dD,
  e_rho,
  drho,
  a_y
]^T
```

其中 `a_y = v_x * omega_z` 作为第 7 个状态，是为了贴合 TinyMPC 的固定线性系统形式：

```text
x[k+1] = A x[k] + B u[k]
```

预测时域内令：

```text
a_y[k+1] = a_y[k]
```

## 3. 控制输入定义

MPC 最终输出左右腿虚拟支持力：

```text
F_l  左腿虚拟支持力
F_r  右腿虚拟支持力
```

为了适配 TinyMPC 的输入 box constraint，第一版不使用 `dF_sum/dF_diff` 作为优化输入，而直接使用左右腿力相对重力平衡点的增量：

```text
u = [
  dF_l,
  dF_r
]^T
```

考虑腿摆角后，径向力需要更大才能提供同样的竖直支撑。使用带下限的 `cos`：

```text
cos_l = max(cos(alpha_l), cos_min)
cos_r = max(cos(alpha_r), cos_min)
cos_min = 0.5
```

定义重力平衡力：

```text
F_g_l = m_l_eff(L_l) * g / cos_l
F_g_r = m_r_eff(L_r) * g / cos_r
```

物理含义：当腿倾斜 alpha 角时，径向力 `F` 的竖直分量为 `F * cos(alpha)`。为支撑重力 `m_eff * g`，需要径向力 `m_eff * g / cos(alpha)`。

第一版可以使用简化质量分配：

```text
m_l_eff = 0.5 * m
m_r_eff = 0.5 * m
```

更贴近当前代码的版本可复用 `eta(L)` 查表：

```text
m_l_eff(L_l) = 0.5 * m_body + eta(L_l) * m_leg
m_r_eff(L_r) = 0.5 * m_body + eta(L_r) * m_leg

F_g_l = m_l_eff(L_l) * g / cos_l
F_g_r = m_r_eff(L_r) * g / cos_r
```

注意：
- 这里不加入任何弹簧补偿项。
- 当 `alpha_l ≠ alpha_r` 时，即使 `m_l_eff = m_r_eff`，`F_g_l` 与 `F_g_r` 也可能不同。

最终输出力：

```text
F_l = F_g_l + dF_l
F_r = F_g_r + dF_r
```

这样，重力平衡已经在 MPC 的输入定义中完成，不再由外部重力前馈叠加。

内部仍可用于日志换算：

```text
F_sum  = F_l + F_r
F_diff = F_r - F_l
```

使用左右腿力增量作为优化输入的好处是，最终力约束可以直接写成输入边界：

```text
F_min - F_g_l <= dF_l <= F_max - F_g_l
F_min - F_g_r <= dF_r <= F_max - F_g_r
```

## 4. 转向惯性补偿在固定 MPC 中的表达

当前代码中已有与转向相关的惯性补偿，核心量可理解为：

```text
v_x * omega_z
```

其中：

```text
v_x      机器人纵向速度，可来自 current.s_dot / fused_speed_mps
omega_z  yaw rate，可来自 IMU gyro_z / current.phi_dot
```

定义横向加速度扰动：

```text
a_y = v_x * omega_z
```

在物理上，`a_y` 是转向时的横向加速度；质心高度为 `h` 时，它会产生 roll 方向的惯性力矩。后续 roll 动力学中用 `b_ay * a_y` 表示这一项。

为了直接适配 TinyMPC 的固定离散线性系统：

```text
x[k+1] = A_d x[k] + B_d u[k]
```

第一版不再单独写扰动矩阵 `E w`，而是把 `a_y` 放入状态向量第 7 维：

```text
x = [e_L, dL_c, D, dD, e_rho, drho, a_y]^T
```

并在预测时域内假设当前横向加速度保持常值：

```text
dot(a_y) = 0
a_y[k+1] = a_y[k]
```

这样 TinyMPC 仍然只求解标准固定 `A/B` 问题，但 MPC 可以在预测中看到转向惯性对 roll 的持续影响，从而提前生成左右腿力差，而不是继续在 MPC 外部手写转向惯性前馈。

后续如果希望使用未来速度指令和 yaw 指令形成 `a_y[k+i]` 序列，需要扩展为时变参考或多模型策略；这不放入第一版固定 TinyMPC 方案。

## 5. 动力学模型

第一版 TinyMPC 使用固定 `alpha = 0` 的动力学矩阵。也就是说，`A/B` 不随腿摆角变化；腿摆角只进入重力平衡点和安全检查。

状态：

```text
x = [
  e_L,
  dL_c,
  D,
  dD,
  e_rho,
  drho,
  a_y
]^T
```

输入：

```text
u = [
  dF_l,
  dF_r
]^T
```

### 5.1 平均腿长动力学

平均腿长主要由左右径向力增量之和控制。围绕重力平衡点线性化：

```text
dot(e_L)  = dL_c
dot(dL_c) = a_dL * dL_c + b_sum * (dF_l + dF_r)
```

其中：

```text
a_dL  < 0
b_sum > 0
```

理论初值：

```text
b_sum ~= 1 / m_L
```

注意：这里不使用 `b_sum / cos(alpha_avg)`。如果状态仍是径向平均腿长 `L_c`，且输入是径向力增量，那么围绕 `F_g = mg / cos(alpha)` 平衡点线性化后，`cos(alpha)` 会在 `ddh = ddL_c * cos(alpha)` 与 `F_vertical = F * cos(alpha)` 中相互抵消。第一版固定 AB 下直接使用 `b_sum`。

如果后续要控制竖直高度：

```text
h_c = L_c * cos(alpha_avg)
```

则需要重新定义状态和 `B` 矩阵，而不是简单把 `b_sum` 除以 `cos`。

### 5.2 腿长差动力学

腿长差：

```text
D = L_r - L_l
```

左右力增量对腿长差的作用：

```text
dot(D)  = dD
dot(dD) = a_dD * dD + b_D * (dF_r - dF_l)
```

其中：

```text
a_dD < 0
```

`b_D` 的符号必须通过日志或小幅激励确认。按直觉，若右腿力增量更大且右腿更容易伸长，则 `b_D > 0`；但实际会受 VMC 符号、腿部机构方向和编码器定义影响。

### 5.3 Roll 轴动力学

roll 轴使用刚体转动方程：

```text
I_roll * dd(rho) = M_roll
```

第一版固定 AB 下，roll 动力学写为：

```text
dot(e_rho) = drho

dot(drho) =
    a_Drho * D
  + a_rho  * e_rho
  + a_drho * drho
  + b_lrho * dF_l
  + b_rrho * dF_r
  + b_ay   * a_y
```

在 `alpha = 0` 时，左右腿力增量产生的 roll 力矩近似为：

```text
M_force = -b/2 * dF_l + b/2 * dF_r
```

因此：

```text
b_lrho = -sigma_F * b / (2 * I_roll)
b_rrho =  sigma_F * b / (2 * I_roll)
```

其中 `sigma_F` 由项目坐标系和 VMC 力方向决定。

腿长差改变支撑平面：

```text
rho_support ~= D / b
```

重力倾倒项近似：

```text
a_rho * (e_rho - D / b)
```

因此理论上：

```text
a_Drho ~= -a_rho / b
```

但 `a_Drho` 的符号也必须通过实车日志确认。

转向横向加速度扰动：

```text
a_y = v_x * omega_z
```

其 roll 加速度项：

```text
b_ay * a_y
```

理论初值：

```text
b_ay ~= sigma_ay * m * h / I_roll
```

`h` 是质心高度。第一版可取常数 `h0`，后续可随腿长和姿态调度。

## 6. 连续状态空间模型

连续模型：

```text
dot(x) = A x + B u
```

其中 `a_y` 已作为第 7 个状态，不再单独写 `E w`。

状态顺序：

```text
x = [e_L, dL_c, D, dD, e_rho, drho, a_y]^T
```

输入顺序：

```text
u = [dF_l, dF_r]^T
```

连续矩阵：

```text
A =
[
  0      1       0        0       0       0        0
  0      a_dL    0        0       0       0        0
  0      0       0        1       0       0        0
  0      0       0        a_dD    0       0        0
  0      0       0        0       0       1        0
  0      0       a_Drho   0       a_rho   a_drho   b_ay
  0      0       0        0       0       0        0
]
```

```text
B =
[
  0        0
  b_sum    b_sum
  0        0
 -b_D      b_D
  0        0
  b_lrho   b_rrho
  0        0
]
```

最后一行 `dot(a_y) = 0`，即预测时域内横向加速度保持常值。

参数说明：

```text
a_dL      平均腿长速度阻尼
b_sum     单腿径向力增量对平均腿长加速度的贡献

a_dD      腿长差速度阻尼
b_D       左右径向力差对腿长差加速度的增益

a_Drho    腿长差对 roll 加速度的影响
a_rho     roll 重力倾倒项
a_drho    roll 角速度阻尼
b_lrho    左腿力增量对 roll 加速度的增益
b_rrho    右腿力增量对 roll 加速度的增益
b_ay      转向横向加速度对 roll 加速度的扰动增益
```

理论初值：

```text
b_sum  ~= 1 / m_L
b_D    ~= sigma_D / m_D
a_rho  ~= sigma_g * m * g * h / I_roll
a_Drho ~= -a_rho / b
a_drho ~= -c_roll / I_roll
b_lrho ~= -sigma_F * b / (2 * I_roll)
b_rrho ~=  sigma_F * b / (2 * I_roll)
b_ay   ~= sigma_ay * m * h / I_roll
```

其中 `sigma_D`、`sigma_g`、`sigma_F`、`sigma_ay` 均需要通过实车日志或仿真激励确认。

## 7. 腿摆角在第一版中的处理

第一版固定 AB：

```text
alpha = 0
cos_l = cos_r = 1
```

TinyMPC 内部不在线更新 `A_d/B_d`。实际运行时，腿摆角只用于：

```text
F_g_l = m_l_eff * g / max(cos(theta_ll), cos_min)
F_g_r = m_r_eff * g / max(cos(theta_lr), cos_min)
```

以及 MPC 工作区间判断：

```text
abs(theta_ll), abs(theta_lr) < theta_mpc_max
```

第一版建议：

```text
cos_min = 0.5        // 60 deg
theta_mpc_max = 45 deg
```

如果后续要考虑腿摆角对 roll 控制效率的影响，可按 `cos(alpha_avg)` 分档生成多套固定 solver，而不是每周期在线修改 TinyMPC 的 `A/B`：

```text
model_0: alpha = 0 deg
model_1: alpha = 20 deg
model_2: alpha = 40 deg
```

## 8. TinyMPC 固定模型离散化

TinyMPC 使用离散线性模型：

```text
x[k+1] = A_d x[k] + B_d u[k]
```

第一版用欧拉离散：

```text
A_d = I + dt * A
B_d = dt * B
```

推荐第一版参数：

```text
dt_mpc = 0.01 s
N      = 15
```

预测时域：

```text
T = dt_mpc * N = 150 ms
```

底层 VMC / 电机输出仍按当前 500 Hz 控制周期运行。MPC 可 100 Hz 更新一次，两个 MPC 周期之间保持上一次输出，或对 `F_l/F_r` 做外部变化率限幅。

## 9. 代价函数

MPC 代价函数：

```text
J = sum_{i=0}^{N-1} [
    q_L      * e_L[i]^2
  + q_dL     * dL_c[i]^2
  + q_D      * D[i]^2
  + q_dD     * dD[i]^2
  + q_rho    * e_rho[i]^2
  + q_drho   * drho[i]^2
  + q_ay     * a_y[i]^2
  + r_l      * dF_l[i]^2
  + r_r      * dF_r[i]^2
]
+ terminal_cost
```

TinyMPC 标准问题中直接设置：

```text
Q = diag([q_L, q_dL, q_D, q_dD, q_rho, q_drho, q_ay])
R = diag([r_l, r_r])
```

第一版参考权重：

```text
Q = diag([
  500,    // e_L
  50,     // dL_c
  300,    // D
  30,     // dD
  5000,   // e_rho
  300,    // drho
  0       // a_y, 扰动状态通常不惩罚
])

R = diag([
  0.01,   // dF_l
  0.01    // dF_r
])
```

若希望抑制输入变化率，第一版建议在 TinyMPC 外部对最终 `F_l/F_r` 做 slew-rate limit。后续若要把变化率纳入 QP，可扩展状态保存上一帧输入并优化 `Delta u`。

调参方向：

```text
平均腿长跟踪慢      增大 q_L 或减小 r_l/r_r
腿长上下振荡明显    增大 q_dL
roll 恢复慢         增大 q_rho 或减小 r_l/r_r
roll 抖动明显       增大 q_drho 或增大外部 slew-rate 限制
左右腿动作太猛      增大 q_D, q_dD, r_l/r_r
支持力过大          增大 r_l/r_r
```

## 10. 约束设计

### 10.1 输入约束

优化输入是左右腿力增量：

```text
F_l = F_g_l + dF_l
F_r = F_g_r + dF_r
```

支持力约束可转为输入 box constraint：

```text
F_min - F_g_l <= dF_l <= F_max - F_g_l
F_min - F_g_r <= dF_r <= F_max - F_g_r
```

TinyMPC 支持输入上下界，因此第一版直接使用这组 bounds。若集成时不方便每周期更新 bounds，可以先使用保守固定 bounds，再在 TinyMPC 输出后对最终 `F_l/F_r` 做安全限幅。

### 10.2 支持力变化率约束

第一版在 TinyMPC 外部做最终力变化率限制：

```text
|F_l[k] - F_l[k-1]| <= dF_max
|F_r[k] - F_r[k-1]| <= dF_max
```

### 10.3 腿长约束

左右腿长：

```text
L_l = L_ref + e_L - D / 2
L_r = L_ref + e_L + D / 2
```

第一版可以不把腿长约束塞进 TinyMPC 的 state bounds，而是作为外部启用条件：

```text
L_safe_min < L_l, L_r < L_safe_max
```

后续如果需要预测内约束，再接入 TinyMPC 的状态 bounds。

### 10.4 Roll 安全约束

第一版也作为外部安全条件处理：

```text
|e_rho| < rho_mpc_max
```

若当前 roll 超过安全阈值，退出 MPC，进入原有 recovery / safe 控制。

### 10.5 腿摆角安全约束

实车腿摆角不会超过 60 度，数值保护：

```text
cos_min = 0.5
cos_l = max(cos(theta_ll), cos_min)
cos_r = max(cos(theta_lr), cos_min)
```

MPC 工作区建议更保守：

```text
theta_mpc_max = 45 deg
```

若检测到：

```text
abs(theta_ll) > theta_mpc_max
abs(theta_lr) > theta_mpc_max
```

第一版直接 fallback。

## 11. TinyMPC C++ 直接集成方案

### 11.1 选择 TinyMPC 的原因

本问题规模很小：

```text
n = 7 states
m = 2 inputs
N = 10 ~ 20
```

TinyMPC 面向资源受限平台，官方文档中说明其标准形式为离散线性系统：

```text
x[k+1] = A x[k] + B u[k]
```

并通过状态/输入代价与 box constraints 组成 MPC 问题。TinyMPC 的求解器内部利用 LQR / Riccati 结构加速 primal update，适合嵌入式实时控制。

第一版直接集成 TinyMPC C++ 库，而不是使用 Python codegen。这样便于在工程中逐步接入、调参和观察 solver 状态。

官方资料：

- TinyMPC examples: https://tinympc.org/get-started/examples/
- Obtaining the model: https://tinympc.org/get-started/model/
- Inside TinyMPC solver: https://tinympc.org/solver/solver/

### 11.2 TinyMPC 问题形式映射

TinyMPC 需要：

```text
A_d       7x7 离散系统矩阵
B_d       7x2 离散输入矩阵
Q         7x7 状态权重
R         2x2 输入权重
N         horizon
x_min/x_max 可选状态约束
u_min/u_max 可选输入约束
x0        当前状态
x_ref     参考状态，第一版为 0
u_ref     参考输入，第一版为 0
```

本项目映射：

```text
x0 = [e_L, dL_c, D, dD, e_rho, drho, a_y]^T
x_ref = 0
u_ref = 0
u = [dF_l, dF_r]^T
```

TinyMPC 输出第一步控制：

```text
u0 = [dF_l, dF_r]^T
```

最终力：

```text
F_l = F_g_l + dF_l
F_r = F_g_r + dF_r
```

### 11.3 固定 AB 的第一版配置

第一版固定 `alpha = 0`，所以 `A_d/B_d` 在初始化时计算一次：

```text
A_d = I + dt_mpc * A
B_d = dt_mpc * B
```

运行时每周期只更新：

```text
x0
u_min/u_max 可选
x_ref = 0
u_ref = 0
```

不在线更新 `A/B`。

### 11.4 工程封装建议

新增独立控制器文件：

```text
app/targets/wheel_legged/include/chassis/roll_leg_mpc.hpp
app/targets/wheel_legged/roll_leg_mpc.cc
```

推荐接口：

```cpp
struct RollLegMpcInput {
  float left_leg_length_m;
  float right_leg_length_m;
  float left_leg_length_dot_mps;
  float right_leg_length_dot_mps;

  float left_leg_theta_rad;
  float right_leg_theta_rad;

  float roll_rad;
  float roll_rate_rad_s;

  float forward_speed_mps;
  float yaw_rate_rad_s;

  float target_leg_length_m;
  float target_roll_rad;
};

struct RollLegMpcOutput {
  bool solved;
  bool active;
  uint8_t fallback_reason;

  float left_force_n;
  float right_force_n;

  float dF_left_n;
  float dF_right_n;
  float gravity_left_n;
  float gravity_right_n;

  float e_L;
  float dL_c;
  float D;
  float dD;
  float e_roll;
  float droll;
  float a_y;
};
```

`Chassis` 中增加成员：

```cpp
RollLegMpc roll_leg_mpc_{};
```

在 `Chassis::Init()` 中初始化 TinyMPC workspace。

在 `ComputeActuatorTorque()` 中，只在常规分支调用：

```cpp
const auto mpc_out = roll_leg_mpc_.Update(mpc_input);
```

第一阶段 shadow 模式：

```cpp
// 仍使用原 PID 输出控制
left_force_ = old_left_force;
right_force_ = old_right_force;

// 只记录 mpc_out.left_force_n / right_force_n
```

第二阶段接管模式：

```cpp
if (mpc_out.solved && mpc_control_enabled) {
  left_force_ = mpc_out.left_force_n;
  right_force_ = mpc_out.right_force_n;
} else {
  left_force_ = old_left_force;
  right_force_ = old_right_force;
}
```

### 11.5 从现有代码取状态

当前 `ComputeActuatorTorque()` 中可以直接获得：

```text
L_l        = left_leg_.l0()
L_r        = right_leg_.l0()
dL_l       = left_leg_.l0_dot()
dL_r       = right_leg_.l0_dot()
theta_ll   = state_output.current.theta_ll
theta_lr   = state_output.current.theta_lr
roll       = imu_roll_
v_x        = state_output.current.s_dot
yaw_rate   = state_output.current.phi_dot
L_ref      = params_.leg_target_length_m
rho_ref    = kRollBalanceTargetRad
```

还需要保存 roll rate：

```text
roll_rate = estimator_input.imu.gyro_x_rad_s
```

当前 `Chassis` 内部只保存了 `imu_roll_`、`imu_acc_x_mps2_`、`imu_acc_z_mps2_` 等，建议增加成员：

```cpp
rm::f32 imu_gyro_x_rad_s_{0.0f};
```

并在 `Update()` 中赋值。

## 12. 每周期运行流程

```text
1. 读取当前状态
   L_l, L_r
   dL_l, dL_r
   theta_ll, theta_lr
   roll, roll_rate
   v_x, yaw_rate
   L_ref, rho_ref

2. 安全检查
   cos_l = max(cos(theta_ll), cos_min)
   cos_r = max(cos(theta_lr), cos_min)
   若腿摆角、腿长、roll 超出 MPC 工作区间，fallback

3. 状态变换
   L_c  = (L_l + L_r) / 2
   D    = L_r - L_l
   dL_c = (dL_l + dL_r) / 2
   dD   = dL_r - dL_l

4. 构造 TinyMPC 状态
   e_L   = L_c - L_ref
   e_rho = roll - rho_ref
   a_y   = v_x * yaw_rate
   x0 = [e_L, dL_c, D, dD, e_rho, roll_rate, a_y]^T

5. 计算重力平衡点
   F_g_l = m_l_eff(L_l) * g / cos_l
   F_g_r = m_r_eff(L_r) * g / cos_r

6. 设置输入约束
   u_min_l = F_min - F_g_l
   u_max_l = F_max - F_g_l
   u_min_r = F_min - F_g_r
   u_max_r = F_max - F_g_r

7. 调用 TinyMPC solve
   u0 = [dF_l, dF_r]^T

8. 还原最终力
   F_l = F_g_l + dF_l
   F_r = F_g_r + dF_r

9. 外部安全限幅和变化率限制
   clamp(F_l, F_min, F_max)
   clamp(F_r, F_min, F_max)
   slew-rate limit F_l/F_r

10. 输入 VMC
    tau_l = J_l^T [F_l, T_p_l]^T
    tau_r = J_r^T [F_r, T_p_r]^T
```

## 13. 与现有代码的接入关系

当前常规分支中，下列逻辑应逐步被 MPC 替代：

```text
left_l0_pid_
right_l0_pid_
roll_pid_
gravity_ff_left/right
inertial_ff_left/right
```

第一版启用条件建议严格限制：

```text
fsm_mode in {kLowLeg, kMidLeg, kHighLeg}
posture_valid == true
standup_complete == true
not jump state
not stair task special control
not recovery mode
not off_ground_in_mid_high_leg
leg length inside safe range
abs(theta_ll/theta_lr) < theta_mpc_max
TinyMPC solved successfully
```

若任一条件不满足，回退到现有 PID / LQR / VMC 流程。

MPC 接管后，常规分支应变为：

```text
base_torque = LQR(...)

F_l, F_r = RollLegMpc(...)

lw_tau = -base_torque.t_wl
rw_tau =  base_torque.t_wr

t_bl_cmd = -base_torque.t_bl
t_br_cmd = -base_torque.t_br

joint_tau_l = J_l^T [F_l, t_bl_cmd]^T
joint_tau_r = J_r^T [F_r, t_br_cmd]^T
```

弹簧补偿不进入 MPC。若后续出于执行器保护或机械补偿需要保留，必须将其定义为 MPC 外部执行层补偿，并在日志中单独记录，避免与 MPC 模型混在一起。

## 14. Shadow 模式与日志

Shadow 模式下：

```text
TinyMPC 正常读取状态、求解、输出 F_l_mpc / F_r_mpc
实际电机仍使用现有 PID 分支输出
记录现有控制等效 F_l_pid / F_r_pid 和 MPC 输出对比
```

建议新增或复用 debug 量：

```text
mpc_enabled
mpc_solved
mpc_fallback_reason

mpc_e_L
mpc_dL_c
mpc_D
mpc_dD
mpc_e_roll
mpc_droll
mpc_a_y

mpc_cos_l
mpc_cos_r
mpc_F_g_l
mpc_F_g_r
mpc_dF_l
mpc_dF_r
mpc_F_l
mpc_F_r

pid_equiv_F_l
pid_equiv_F_r
pid_equiv_F_sum
pid_equiv_F_diff
```

重点检查：

```text
腿长低于目标时，dF_l + dF_r 是否增大
roll 右倾时，dF_l/dF_r 方向是否会扶正
加速转向时，MPC 是否提前产生抗侧倾力差
F_l/F_r 是否在安全范围内
输出变化率是否可接受
```

## 15. 参数辨识

需要辨识的参数：

```text
a_dL, b_sum
a_dD, b_D
a_Drho, a_rho, a_drho, b_lrho, b_rrho, b_ay
```

其中最关键的符号：

```text
b_D
b_lrho / b_rrho
b_ay
a_Drho
```

从日志构造：

```text
L_c  = (L_l + L_r) / 2
D    = L_r - L_l
dL_c = (dL_l + dL_r) / 2
dD   = dL_r - dL_l

F_g_l = m_l_eff * g / cos_l
F_g_r = m_r_eff * g / cos_r

dF_l = F_l - F_g_l
dF_r = F_r - F_g_r

a_y = v_x * omega_z
```

通过滤波 / 平滑微分得到：

```text
ddL_c
ddD
ddrho
```

平均腿长模型：

```text
ddL_c = a_dL * dL_c + b_sum * (dF_l + dF_r)
```

腿长差模型：

```text
ddD = a_dD * dD + b_D * (dF_r - dF_l)
```

Roll 模型：

```text
ddrho =
    a_Drho * D
  + a_rho  * e_rho
  + a_drho * drho
  + b_lrho * dF_l
  + b_rrho * dF_r
  + b_ay   * a_y
```

正则最小二乘：

```text
theta = (X^T X + lambda I)^(-1) X^T Y
```

## 16. 实施阶段

### 阶段 0：模型参数和日志准备

- 确认 `roll` 正方向、`gyro_x` 正方向、`D = L_r - L_l` 正方向。
- 确认当前 `left_force_ / right_force_` 的物理符号。
- 增加 `roll_rate = gyro_x` 的内部保存。
- 新增 MPC shadow debug 字段。
- 在现有 PID 控制下采集常规站立、低速直行、低速转向、高速转向数据。

### 阶段 1：固定 AB + TinyMPC shadow

- 直接集成 TinyMPC C++ 库。
- 实现 7 状态、2 输入、固定 `A_d/B_d` 的 solver。
- 使用理论参数初值。
- 只 shadow，不控制电机。
- 对比 MPC 输出和现有 PID 等效输出。
- 校准 `b_D`、`b_lrho/b_rrho`、`b_ay`、`a_Drho` 的符号。

### 阶段 2：常规支撑小权重接管

- 只在低速、低腿长变化、姿态稳定时启用。
- 保守设置 `F_min/F_max` 和外部变化率限制。
- `q_rho/q_L` 从小开始，逐步提高。
- 一旦 TinyMPC 无解、姿态异常、支撑力异常，立即 fallback 到现有 PID。

### 阶段 3：辨识参数替换理论参数

- 使用日志辨识 `A/B` 参数。
- 替换理论初值。
- 分别验证直行、转向、左右扰动、不同腿长档位下的响应。

### 阶段 4：多模型或调度

第一版固定 `alpha = 0`。后续若需要考虑腿摆角对 roll 控制效率的影响，可按 `cos(alpha_avg)` 分档生成多套固定 solver：

```text
model_0: alpha = 0 deg
model_1: alpha = 20 deg
model_2: alpha = 40 deg
```

运行时根据当前腿摆角选择 solver，而不是每周期在线改 TinyMPC 的 `A/B`。

## 17. 风险点

### 17.1 符号错误

最危险的是左右力差对 roll 的符号错误。现有代码常规分支表现为：

```text
left_force  += roll_pid_out
right_force -= roll_pid_out
```

即：

```text
F_diff = F_r - F_l
F_diff += -2 * roll_pid_out
```

这说明项目坐标下 `F_diff` 与 roll 恢复的直觉符号可能和文档公式相反。必须通过 shadow 和小幅激励确认。

### 17.2 固定 AB 模型误差

第一版 TinyMPC 使用 `alpha = 0` 的固定 AB。腿摆角较大时：

- 重力平衡点通过 `mg/cos(theta)` 修正。
- 但动力学增益仍按腿竖直处理。
- 因此大摆角下模型误差会增加。

建议通过启用条件限制 MPC 工作区间：

```text
abs(theta_ll), abs(theta_lr) < theta_mpc_max
```

第一版建议 `theta_mpc_max = 45 deg`，机械上限 60 度只作为 `cos_min = 0.5` 的保护。

### 17.3 TinyMPC 求解失败或迭代超时

若 TinyMPC 求解失败或迭代超时，应：

- 使用上一帧安全输出，或
- 立即 fallback 到现有 PID 分支。

不要在控制闭环里使用未收敛的异常输出。

### 17.4 与 LQR 耦合

LQR 仍在输出轮端和髋部摆角力矩，MPC 输出腿向支持力。二者通过腿部姿态、轮地接触和机体姿态耦合。第一版不要同时大幅提高 LQR 和 MPC 的 aggressive 程度，应先保证 MPC 的 `F_l/F_r` 平滑。

## 18. 最终推荐架构

最终常规支撑控制结构：

```text
state estimator
  -> L_l, L_r, dL_l, dL_r, theta_ll, theta_lr, roll, roll_rate, v_x, omega_z

control_loop
  -> L_ref, rho_ref

roll-leg TinyMPC
  -> dF_l, dF_r
  -> F_l = F_g_l + dF_l
  -> F_r = F_g_r + dF_r

existing LQR
  -> T_wl, T_wr, T_bl, T_br

VMC / Jacobian mapping
  -> joint torques
```

常规模式下不再使用：

```text
leg length PID
roll PID
external gravity feedforward
external turn inertial feedforward
```

弹簧补偿不进入 MPC。若后续出于执行器保护或机械补偿需要保留，必须将其定义为 MPC 外部执行层补偿，并在日志中单独记录，避免与 MPC 模型混在一起。
