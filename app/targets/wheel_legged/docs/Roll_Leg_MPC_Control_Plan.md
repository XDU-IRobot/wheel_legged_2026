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
  drho
]^T
```

## 3. 控制输入定义

MPC 最终输出左右腿虚拟支持力：

```text
F_l  左腿虚拟支持力
F_r  右腿虚拟支持力
```

内部使用总力和差力：

```text
F_sum  = F_l + F_r
F_diff = F_r - F_l
```

反解：

```text
F_l = (F_sum - F_diff) / 2
F_r = (F_sum + F_diff) / 2
```

为了把重力前馈纳入 MPC，定义重力平衡力：

```text
F_g_l = m_l_eff(L_l) * g
F_g_r = m_r_eff(L_r) * g
```

第一版可以使用简化形式：

```text
F_g_l = 0.5 * m * g
F_g_r = 0.5 * m * g
```

更贴近当前代码的版本可复用 `eta(L)` 查表：

```text
m_l_eff(L_l) = 0.5 * m_body + eta(L_l) * m_leg
m_r_eff(L_r) = 0.5 * m_body + eta(L_r) * m_leg

F_g_l = m_l_eff(L_l) * g
F_g_r = m_r_eff(L_r) * g
```

注意：这里不加入任何弹簧补偿项。

定义平衡输入：

```text
F_sum_eq  = F_g_l + F_g_r
F_diff_eq = F_g_r - F_g_l
```

MPC 的优化输入使用相对平衡点的增量：

```text
u = [
  dF_sum,
  dF_diff
]^T
```

最终输出力：

```text
F_sum  = F_sum_eq  + dF_sum
F_diff = F_diff_eq + dF_diff

F_l = (F_sum - F_diff) / 2
F_r = (F_sum + F_diff) / 2
```

这样，重力平衡已经在 MPC 的输入定义中完成，不再由外部重力前馈叠加。

## 4. 转向扰动定义

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

该扰动作为 MPC 已知扰动进入预测模型：

```text
w = a_y
```

第一版预测时域内可假设：

```text
w[k+i] = w[k]
```

后续可根据速度指令和 yaw 指令构造扰动序列：

```text
w[k+i] = v_x_ref[k+i] * omega_z_ref[k+i]
```

## 5. 平均腿长动力学

平均腿长主要由总支持力控制。近似物理关系：

```text
m_L * ddL_c = F_sum_vertical - m * g
```

若腿接近竖直，第一版可近似：

```text
F_sum_vertical ~= F_sum
```

由于 `F_sum_eq` 已包含重力平衡，有：

```text
ddL_c = b_sum * dF_sum
```

考虑机构阻尼、电机响应、腿长速度反馈效果，可加入速度阻尼：

```text
dot(e_L)  = dL_c
dot(dL_c) = a_dL * dL_c + b_sum * dF_sum
```

参数符号：

```text
a_dL  < 0
b_sum > 0
```

理论初值：

```text
b_sum ~= 1 / m_L
```

其中 `m_L` 是腿长方向等效质量。第一版可取整机参与垂向运动的等效质量，后续通过辨识修正。

这里不显式加入：

```text
a_L * e_L
```

因为腿长恢复由 MPC 代价函数中的 `q_L * e_L^2` 产生，对应原腿长 PID 的 P 项。若后续辨识发现系统本体或底层执行结构存在明显自然恢复，再考虑加入 `a_L`。

## 6. 腿长差动力学

腿长差：

```text
D = L_r - L_l
```

支持力差：

```text
F_diff = F_r - F_l
```

围绕重力差平衡点，使用增量输入：

```text
dF_diff = F_diff - F_diff_eq
```

近似动力学：

```text
dot(D)  = dD
dot(dD) = a_dD * dD + b_D * dF_diff
```

参数符号：

```text
a_dD < 0
```

`b_D` 的符号必须用日志或小幅开环激励确认。直觉上，若 `F_diff > 0` 表示右腿支持力更大，右腿更容易相对左腿伸展，则 `b_D > 0`；但实车符号会受到机构定义、腿向力方向、编码器方向和 VMC 符号影响，不能仅凭文档确定。

可写成：

```text
b_D = sigma_D / m_D
```

其中：

```text
sigma_D = +1 或 -1
m_D     = 腿长差方向等效质量
```

## 7. Roll 轴动力学

roll 轴使用刚体转动方程：

```text
I_roll * dd(rho) = M_roll
```

roll 力矩来源包括：

```text
重力倾倒力矩
roll 阻尼力矩
左右腿长差改变支撑平面产生的力矩
左右支持力差产生的控制力矩
转向横向惯性扰动力矩
```

设：

```text
b       左右轮 / 腿接触点横向距离
h       质心高度
I_roll  整机绕 roll 轴转动惯量
m       整机等效质量
```

### 7.1 支持力差项

左右腿接触点相对中心近似为：

```text
left  = -b / 2
right = +b / 2
```

支持力差产生 roll 力矩：

```text
M_force ~= (b / 2) * F_diff
```

围绕平衡点使用 `dF_diff`：

```text
dd(rho)_force = b_rho * dF_diff
```

理论初值：

```text
b_rho ~= sigma_F * b / (2 * I_roll)
```

其中 `sigma_F` 由项目坐标系和 VMC 力方向决定。

### 7.2 重力倾倒项

小角度下，roll 偏角导致质心横向偏移：

```text
y_com ~= h * rho
```

重力产生的 roll 力矩：

```text
M_gravity ~= m * g * h * rho
```

得到：

```text
dd(rho)_gravity = a_rho * e_rho
```

理论初值：

```text
a_rho ~= sigma_g * m * g * h / I_roll
```

对于轮腿倒立结构，该项常表现为发散项，即偏了之后会继续偏，某些符号定义下 `a_rho > 0`。

### 7.3 腿长差项

左右腿长差会改变支撑平面。近似：

```text
rho_support ~= D / b
```

重力项更合理地写作：

```text
dd(rho)_gravity ~= a_rho * (e_rho - k_D * D / b)
```

第一版取：

```text
k_D = 1
```

展开得到：

```text
dd(rho)_gravity = a_rho * e_rho + a_Drho * D
```

其中：

```text
a_Drho ~= -a_rho / b
```

实际符号仍需结合 IMU roll 方向、`D = L_r - L_l` 定义和 VMC 输出方向校准。

### 7.4 Roll 阻尼项

roll 角速度阻尼：

```text
dd(rho)_damping = a_drho * drho
```

一般：

```text
a_drho < 0
```

理论：

```text
a_drho = -c_roll / I_roll
```

### 7.5 转向横向惯性项

转向横向加速度：

```text
a_y = v_x * omega_z
```

横向惯性力：

```text
F_inertia = m * a_y
```

作用在质心高度 `h` 处，产生 roll 扰动力矩：

```text
M_turn ~= m * h * a_y
```

得到：

```text
dd(rho)_turn = b_ay * a_y
```

理论初值：

```text
b_ay ~= sigma_ay * m * h / I_roll
```

符号由 yaw rate、纵向速度、roll 正方向共同决定。

稳态转向时，若希望：

```text
e_rho = 0
D     = 0
drho  = 0
```

则需要：

```text
b_rho * dF_diff_turn + b_ay * a_y = 0
```

因此：

```text
dF_diff_turn = -b_ay / b_rho * a_y
```

代入理论初值：

```text
b_rho ~= b / (2 * I_roll)
b_ay  ~= m * h / I_roll
```

得到：

```text
dF_diff_turn ~= -2 * m * h / b * a_y
```

这就是现有转向惯性补偿在 MPC 里的物理意义：MPC 通过扰动预测提前生成合适的 `dF_diff`，而不是由外部手写 `inertial_ff_left/right` 叠加。

### 7.6 完整 roll 动力学

综合得到：

```text
dot(e_rho) = drho

dot(drho) =
    a_Drho * D
  + a_rho  * e_rho
  + a_drho * drho
  + b_rho  * dF_diff
  + b_ay   * a_y
```

## 8. 连续状态空间模型

状态：

```text
x = [
  e_L,
  dL_c,
  D,
  dD,
  e_rho,
  drho
]^T
```

输入：

```text
u = [
  dF_sum,
  dF_diff
]^T
```

扰动：

```text
w = a_y = v_x * omega_z
```

连续模型：

```text
dot(x) = A x + B u + E w
```

其中：

```text
A =
[
  0      1       0        0       0       0
  0      a_dL    0        0       0       0
  0      0       0        1       0       0
  0      0       0        a_dD    0       0
  0      0       0        0       0       1
  0      0       a_Drho   0       a_rho   a_drho
]
```

```text
B =
[
  0       0
  b_sum   0
  0       0
  0       b_D
  0       0
  0       b_rho
]
```

```text
E =
[
  0
  0
  0
  0
  0
  b_ay
]
```

参数说明：

```text
a_dL    平均腿长速度阻尼
b_sum   总支持力增量对平均腿长加速度的增益

a_dD    腿长差速度阻尼
b_D     支持力差增量对腿长差加速度的增益

a_Drho  腿长差对 roll 加速度的影响
a_rho   roll 重力倾倒项
a_drho  roll 角速度阻尼
b_rho   支持力差对 roll 加速度的增益
b_ay    转向横向加速度对 roll 加速度的扰动增益
```

理论初值：

```text
b_sum  ~= 1 / m_L
b_D    ~= sigma_D / m_D
a_rho  ~= sigma_g * m * g * h / I_roll
a_Drho ~= -a_rho / b
a_drho ~= -c_roll / I_roll
b_rho  ~= sigma_F * b / (2 * I_roll)
b_ay   ~= sigma_ay * m * h / I_roll
```

其中 `sigma_D`、`sigma_g`、`sigma_F`、`sigma_ay` 均需要通过实车日志或仿真激励确认。

## 9. 离散模型

MPC 使用离散模型：

```text
x[k+1] = A_d x[k] + B_d u[k] + E_d w[k]
```

第一版可使用欧拉离散：

```text
A_d = I + dt * A
B_d = dt * B
E_d = dt * E
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

若算力允许，可尝试：

```text
dt_mpc = 0.005 s
N      = 20
T      = 100 ms
```

底层 VMC / 电机输出仍按当前 500 Hz 控制周期运行。MPC 可 100 Hz 更新一次，两个 MPC 周期之间保持上一次输出，或对 `F_l/F_r` 做线性插值 / 变化率限幅。

## 10. 代价函数

MPC 代价函数：

```text
J = sum_{i=0}^{N-1} [
    q_L      * e_L[i]^2
  + q_dL     * dL_c[i]^2
  + q_D      * D[i]^2
  + q_dD     * dD[i]^2
  + q_rho    * e_rho[i]^2
  + q_drho   * drho[i]^2
  + r_sum    * dF_sum[i]^2
  + r_diff   * dF_diff[i]^2
  + rd_sum   * Delta dF_sum[i]^2
  + rd_diff  * Delta dF_diff[i]^2
]
+ terminal_cost
```

输入变化率：

```text
Delta dF_sum[i]  = dF_sum[i]  - dF_sum[i-1]
Delta dF_diff[i] = dF_diff[i] - dF_diff[i-1]
```

含义对应：

```text
q_L       原腿长 PID 的 P 效果
q_dL      原腿长 PID 的 D 效果
q_rho     原 roll PID 的 P 效果
q_drho    原 roll PID 的 D 效果
q_D       抑制左右腿长差
q_dD      抑制左右腿长差变化速度
r_sum     限制总支持力增量
r_diff    限制左右支持力差增量
rd_sum    平滑总支持力
rd_diff   平滑支持力差
```

初始权重可参考：

```text
Q = diag([
  500,    // e_L
  50,     // dL_c
  300,    // D
  30,     // dD
  5000,   // e_rho
  300     // drho
])

R = diag([
  0.01,   // dF_sum
  0.01    // dF_diff
])

R_delta = diag([
  0.1,    // Delta dF_sum
  0.1     // Delta dF_diff
])
```

实际调参方向：

```text
平均腿长跟踪慢      增大 q_L 或减小 r_sum
腿长上下振荡明显    增大 q_dL 或 rd_sum
roll 恢复慢         增大 q_rho 或减小 r_diff
roll 抖动明显       增大 q_drho 或 rd_diff
左右腿动作太猛      增大 q_D, q_dD, rd_diff
支持力过大          增大 r_sum, r_diff
```

## 11. 约束设计

### 11.1 支持力约束

最终力：

```text
F_sum  = F_sum_eq  + dF_sum
F_diff = F_diff_eq + dF_diff

F_l = (F_sum - F_diff) / 2
F_r = (F_sum + F_diff) / 2
```

支持力约束：

```text
F_min <= F_l <= F_max
F_min <= F_r <= F_max
```

写成输入线性约束：

```text
F_min <= (F_sum_eq + dF_sum - F_diff_eq - dF_diff) / 2 <= F_max
F_min <= (F_sum_eq + dF_sum + F_diff_eq + dF_diff) / 2 <= F_max
```

第一版可设：

```text
F_min = 0
F_max = 根据电机力矩、雅可比奇异性和当前 PID 输出经验确定
```

在离地、支撑力异常或腿长接近奇异位形时，应额外收紧 `F_max` 或直接退出 MPC。

### 11.2 支持力变化率约束

建议约束最终左右力：

```text
|F_l[k] - F_l[k-1]| <= dF_max
|F_r[k] - F_r[k-1]| <= dF_max
```

也可约束内部变量：

```text
|dF_sum[k]  - dF_sum[k-1]|  <= dF_sum_max
|dF_diff[k] - dF_diff[k-1]| <= dF_diff_max
```

第一版若求解器实现更简单，可先使用内部变量变化率约束。

### 11.3 腿长约束

左右腿长：

```text
L_l = L_ref + e_L - D / 2
L_r = L_ref + e_L + D / 2
```

约束：

```text
L_min <= L_ref + e_L - D / 2 <= L_max
L_min <= L_ref + e_L + D / 2 <= L_max
```

### 11.4 腿长速度约束

左右腿长速度：

```text
dL_l = dL_c - dD / 2
dL_r = dL_c + dD / 2
```

约束：

```text
|dL_c - dD / 2| <= dL_max
|dL_c + dD / 2| <= dL_max
```

### 11.5 Roll 安全约束

```text
|e_rho| <= rho_max
```

建议使用软约束：

```text
|e_rho| <= rho_max + epsilon
epsilon >= 0
```

并在代价中加入：

```text
q_epsilon * epsilon^2
```

这样在大扰动或异常姿态下 QP 不会轻易无解。

## 12. 每周期运行流程

MPC 每个周期执行：

```text
1. 读取当前状态
   L_l, L_r
   dL_l, dL_r
   roll, roll_rate
   v_x, omega_z
   L_ref, rho_ref

2. 状态变换
   L_c  = (L_l + L_r) / 2
   D    = L_r - L_l
   dL_c = (dL_l + dL_r) / 2
   dD   = dL_r - dL_l

3. 构造误差状态
   e_L   = L_c - L_ref
   e_rho = roll - rho_ref

4. 计算重力平衡点
   F_g_l = m_l_eff(L_l) * g
   F_g_r = m_r_eff(L_r) * g
   F_sum_eq  = F_g_l + F_g_r
   F_diff_eq = F_g_r - F_g_l

5. 计算扰动
   w = a_y = v_x * omega_z

6. 构造预测模型
   x[k+1] = A_d x[k] + B_d u[k] + E_d w[k]

7. 求解 QP
   得到 u[0] = [dF_sum, dF_diff]^T

8. 还原最终虚拟支持力
   F_sum  = F_sum_eq  + dF_sum
   F_diff = F_diff_eq + dF_diff
   F_l = (F_sum - F_diff) / 2
   F_r = (F_sum + F_diff) / 2

9. 安全限幅和变化率限制
   clamp / slew-rate limit F_l, F_r

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
fsm_mode in {kLowLeg, kMidLeg, kHighLeg, kSpin 可选}
posture_valid == true
standup_complete == true
not jump state
not stair task special control
not recovery mode
not off_ground_in_mid_high_leg
leg length inside safe range
QP solved successfully
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

弹簧补偿不进入 MPC。是否在 MPC 输出之后仍由外部叠加弹簧补偿，需要单独实车决策；如果严格执行“弹簧补偿不进 MPC 且不影响力命令”，则不叠加。若为了保护机械或抵消已知机构力矩保留外部弹簧项，应明确它是 VMC 输出后的执行器补偿，而不是 MPC 模型的一部分。

## 14. Shadow 模式与日志

为了确认模型符号和量级，必须先实现 shadow 模式。

Shadow 模式下：

```text
MPC 正常读取状态、求解 QP、输出 F_l_mpc / F_r_mpc
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

mpc_F_sum_eq
mpc_F_diff_eq
mpc_dF_sum
mpc_dF_diff
mpc_F_l
mpc_F_r

pid_equiv_F_l
pid_equiv_F_r
pid_equiv_F_sum
pid_equiv_F_diff

mpc_pred_roll_1
mpc_pred_roll_N
mpc_pred_Lc_1
mpc_pred_Lc_N
```

重点检查：

```text
roll 右倾时，MPC 给出的 F_diff 方向是否会扶正
加速转向时，MPC 是否提前产生反向 F_diff_turn
腿长低于目标时，dF_sum 是否增大
F_l/F_r 是否在安全范围内
输出变化率是否可接受
```

## 15. 参数辨识

### 15.1 需要辨识的参数

```text
a_dL, b_sum
a_dD, b_D
a_Drho, a_rho, a_drho, b_rho, b_ay
```

其中最关键的符号：

```text
b_D
b_rho
b_ay
a_Drho
```

### 15.2 数据构造

从日志构造：

```text
L_c  = (L_l + L_r) / 2
D    = L_r - L_l
dL_c = (dL_l + dL_r) / 2
dD   = dL_r - dL_l

F_sum  = F_l + F_r
F_diff = F_r - F_l

F_sum_eq  = F_g_l + F_g_r
F_diff_eq = F_g_r - F_g_l

dF_sum  = F_sum  - F_sum_eq
dF_diff = F_diff - F_diff_eq

a_y = v_x * omega_z
```

通过滤波 / 平滑微分得到：

```text
ddL_c
ddD
ddrho
```

### 15.3 平均腿长模型辨识

模型：

```text
ddL_c = a_dL * dL_c + b_sum * dF_sum
```

构造：

```text
Y_L = ddL_c
X_L = [dL_c, dF_sum]
theta_L = [a_dL, b_sum]^T
```

最小二乘：

```text
theta_L = (X_L^T X_L + lambda I)^(-1) X_L^T Y_L
```

### 15.4 腿长差模型辨识

模型：

```text
ddD = a_dD * dD + b_D * dF_diff
```

构造：

```text
Y_D = ddD
X_D = [dD, dF_diff]
theta_D = [a_dD, b_D]^T
```

### 15.5 Roll 模型辨识

模型：

```text
ddrho =
    a_Drho * D
  + a_rho  * e_rho
  + a_drho * drho
  + b_rho  * dF_diff
  + b_ay   * a_y
```

构造：

```text
Y_rho = ddrho
X_rho = [D, e_rho, drho, dF_diff, a_y]
theta_rho = [a_Drho, a_rho, a_drho, b_rho, b_ay]^T
```

正则最小二乘：

```text
theta_rho = (X_rho^T X_rho + lambda I)^(-1) X_rho^T Y_rho
```

## 16. 实施阶段

### 阶段 0：模型参数和日志准备

- 确认 `roll` 正方向、`gyro_x` 正方向、`D = L_r - L_l` 正方向。
- 确认当前 `left_force_ / right_force_` 的物理符号。
- 新增 MPC shadow debug 字段。
- 在现有 PID 控制下采集常规站立、低速直行、低速转向、高速转向数据。

### 阶段 1：固定参数 shadow MPC

- 实现 6 状态、2 输入、1 扰动线性 MPC。
- 使用理论参数初值。
- 只 shadow，不控制电机。
- 对比 MPC 输出和现有 PID 等效输出。
- 校准 `b_D`、`b_rho`、`b_ay`、`a_Drho` 的符号。

### 阶段 2：常规支撑小权重接管

- 只在低速、低腿长变化、姿态稳定时启用。
- 保守设置 `F_min/F_max` 和变化率约束。
- `q_rho/q_L` 从小开始，逐步提高。
- 一旦 QP 无解、姿态异常、支撑力异常，立即 fallback 到现有 PID。

### 阶段 3：辨识参数替换理论参数

- 使用日志辨识 `A/B/E` 参数。
- 替换理论初值。
- 分别验证直行、转向、左右扰动、不同腿长档位下的响应。

### 阶段 4：腿长调度

质心高度和 roll 参数随腿长变化：

```text
h = h(L_c)
a_rho(L_c)  ~= m * g * h(L_c) / I_roll
b_ay(L_c)   ~= m * h(L_c) / I_roll
b_rho(L_c)  可根据腿姿态和接触几何修正
```

将固定参数 MPC 扩展为按 `L_c` 更新 `A/B/E` 的 LTV-MPC。

### 阶段 5：地形 / 单边桥扩展

估计左右地面高度差：

```text
Dz_g_hat ~= b * e_rho - D
```

低通滤波：

```text
Dz_g_hat[k] =
  beta * Dz_g_hat[k-1]
+ (1 - beta) * (b * e_rho[k] - D[k])
```

可引入腿长差参考：

```text
D_ref = k_ground * Dz_g_hat
```

或扩展 roll 动力学：

```text
dot(drho) = ... + b_g * Dz_g_hat
```

## 17. 风险点

### 17.1 符号错误

最危险的是 `F_diff` 对 roll 的符号错误。现有代码常规分支表现为：

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

### 17.2 QP 无解

同时约束支持力、腿长、腿速和 roll，可能在大扰动下无解。应：

- roll 使用软约束。
- 腿长安全边界留裕量。
- QP 失败时使用上一帧安全输出或立即 fallback。

### 17.3 模型过于理想

第一版模型未包含：

```text
弹簧补偿
轮地接触非线性
侧向滑移
腿部机构摩擦
电机延迟
雅可比奇异附近增益变化
```

因此需要保守权重、限幅和 fallback。

### 17.4 与 LQR 耦合

LQR 仍在输出轮端和髋部摆角力矩，MPC 输出腿向支持力。二者通过腿部姿态、轮地接触和机体姿态耦合。第一版不要同时大幅提高 LQR 和 MPC 的 aggressive 程度，应先保证 MPC 的 `F_l/F_r` 平滑。

## 18. 最终推荐架构

最终常规支撑控制结构：

```text
state estimator
  -> L_l, L_r, dL_l, dL_r, roll, roll_rate, v_x, omega_z

control_loop
  -> L_ref, rho_ref

roll-leg MPC
  -> F_l, F_r

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

