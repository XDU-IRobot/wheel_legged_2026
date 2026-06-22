# WL VMC Sim2Real 部署说明

本文档说明 `IRobot-WL-Velocity-VMC-Flat-v0` / `IRobot-WL-Velocity-VMC-Rough-v0` 训练出来的 VMC policy 如何部署到实机，重点说明 policy 输入输出、单位、缩放、正方向和 URDF 坐标系的对应关系。

相关源码：

- URDF：`source/IRobot_wl/data/Robots/wl/wl_description/urdf/wl.urdf`
- 机器人参数：`source/IRobot_wl/IRobot_wl/assets/wl.py`
- VMC action：`source/IRobot_wl/IRobot_wl/tasks/manager_based/locomotion/velocity/mdp/vmc.py`
- VMC observation：`source/IRobot_wl/IRobot_wl/tasks/manager_based/locomotion/velocity/config/wheeled/wl/vmc_rough_env_cfg.py`
- ONNX 导出：`scripts/rsl_rl/export_onnx.py`

## 1. 部署整体流程

推荐实机控制链路如下：

```text
实机传感器
  -> 关节位置/速度、IMU角速度、IMU姿态/重力方向、速度指令
  -> 按训练时的 observation 顺序和缩放组 policy 输入
  -> ONNX policy 推理
  -> 得到 6 维 VMC action
  -> 按训练里的 VMC 公式转换为 6 个关节力矩
  -> 电机力矩控制/底层电流控制
```

这里的 policy 不是直接输出电机角度，也不是直接输出电机力矩。policy 输出的是任务空间 action：

```text
[theta_L, L0_L, wheel_L, theta_R, L0_R, wheel_R]
```

VMC 控制器再把腿角、腿长、轮速目标转换成 URDF 关节力矩。

## 2. URDF 坐标系和正方向

### 2.1 base_link 坐标系

本文所有方向按 `wl.urdf` 中 `base_link` 定义说明。部署时建议把实机机体系也对齐为：

```text
+X：机器人前方
+Y：机器人左侧
+Z：机器人上方
```

URDF 中左右腿安装位置也符合这个约定：

- 左腿 hip joint：`lf0_Joint` origin 的 `y = +0.16652`
- 右腿 hip joint：`rf0_Joint` origin 的 `y = -0.17680`
- 左轮：`l_wheel_Joint` origin 的 `y = +0.14186`
- 右轮：`r_wheel_Joint` origin 的 `y = -0.14186`

### 2.2 关节和 link 顺序

训练中腿关节逻辑顺序是：

```text
[lf0_Joint, lf1_Joint, rf0_Joint, rf1_Joint]
```

轮关节逻辑顺序是：

```text
[l_wheel_Joint, r_wheel_Joint]
```

Isaac articulation 的完整 joint order 在代码注释中写成：

```text
[lf0_Joint, rf0_Joint, lf1_Joint, rf1_Joint, l_wheel_Joint, r_wheel_Joint]
```

部署端不要混用这两个顺序。VMC 函数的 `leg_joint_indices` 按 `[lf0, lf1, rf0, rf1]` 传入，最终输出力矩时再写回完整电机顺序。

### 2.3 URDF 关节轴

URDF 中所有腿关节和轮关节的 `<axis xyz="0 0 1" />`，也就是各自 joint frame 的 `+Z` 轴为正转轴。正方向按右手定则定义：右手拇指指向 joint frame 的 `+Z`，四指弯曲方向为关节位置、速度和力矩的正方向。

需要注意 hip joint 的 joint frame 相对 `base_link` 有旋转：

```text
lf0_Joint origin rpy = [-pi/2, 0, 0]
rf0_Joint origin rpy = [+pi/2, 0, 0]
```

因此左右 hip 的 URDF 正转轴映射到 base frame 后方向相反：

- `lf0_Joint` 的正轴大致是 base `+Y`
- `rf0_Joint` 的正轴大致是 base `-Y`

这也是 VMC 代码里右腿要做镜像符号处理的原因。

轮关节 `l_wheel_Joint` 和 `r_wheel_Joint` 的 origin `rpy="0 0 0"`，轴均是各自父 link frame 的 `+Z`。部署时以电机编码器的正方向为准，但必须校准到 URDF joint 正方向。

## 3. Policy 输入

ONNX 导出的 sequence policy 有两个输入：

```text
observations:         shape = [batch, 27]
observation_history: shape = [batch, 135]
```

其中 `observation_history` 是最近 5 帧 policy observation 拼接：

```text
[obs(t-4), obs(t-3), obs(t-2), obs(t-1), obs(t)]
```

如果刚上电没有历史，先用当前帧重复填满 5 帧，或者用 0 初始化后逐帧推进。更稳妥的是重复当前帧，避免 encoder 一开始看到非真实的全零历史。

### 3.1 单帧 observation 顺序

单帧 27 维输入顺序如下，所有值都必须使用训练时同样的缩放：

| 维度 | 名称 | 单位 | 输入给 policy 的值 | 正方向/含义 |
| --- | --- | --- | --- | --- |
| 0:3 | `base_ang_vel` | rad/s | `base_ang_vel_b * 0.25` | base 坐标系角速度 `[wx, wy, wz]`，`+wz` 为左转/yaw 正方向 |
| 3:6 | `projected_gravity` | 1 | `g_b` | 世界重力方向投影到 base 坐标系，直立时约 `[0, 0, -1]` |
| 6 | `cmd_lin_x` | m/s | `vx_cmd * 2.0` | base `+X` 前进为正 |
| 7 | `cmd_yaw` | rad/s | `yaw_rate_cmd * 0.25` | base `+Z` 左转为正 |
| 8 | `height_cmd` | m | `height_cmd * 5.0` | 目标机身高度，默认 `0.23 m`，输入值默认 `1.15` |
| 9:11 | `leg_angle` | rad | `[theta0_L, theta0_R]` | 任务空间腿角，0 表示竖直向下 |
| 11:13 | `leg_angle_dot` | rad/s | `[theta0_dot_L, theta0_dot_R] * 0.05` | 腿角速度 |
| 13:15 | `leg_length` | m | `[L0_L, L0_R] * 5.0` | hip 到轮心的等效腿长 |
| 15:17 | `leg_length_dot` | m/s | `[L0_dot_L, L0_dot_R] * 0.25` | 腿长变化率，伸长为正 |
| 17:19 | `wheel_pos` | rad | `[-q_l_wheel, q_r_wheel]` | 训练观测中左轮位置取负，右轮不取负 |
| 19:21 | `wheel_vel` | rad/s | `[-dq_l_wheel, dq_r_wheel] * 0.05` | 训练观测中左轮速度取负，右轮不取负 |
| 21:27 | `last_action` | 1 | 上一帧送入 VMC 的 processed action | 顺序 `[theta_L, L0_L, wheel_L, theta_R, L0_R, wheel_R]` |

`base_ang_vel_b`、`projected_gravity` 必须在 `base_link` 坐标系下。常见 IMU 坐标系如果不是 `base_link`，必须先做固定外参旋转。

### 3.2 速度指令

VMC policy 没有使用横向速度 `vy`。训练和键盘 play 中给 policy 的 command 是：

```text
[vx_cmd * 2.0, yaw_rate_cmd * 0.25, height_cmd * 5.0]
```

建议实机限幅从小开始：

```text
vx_cmd:       [-0.3, 0.3] m/s 起步，确认稳定后再增大
yaw_rate_cmd: [-0.4, 0.4] rad/s 起步
height_cmd:   0.23 m 默认
```

平地配置训练范围更大：

```text
vx_cmd:       [-2.0, 2.0] m/s
yaw_rate_cmd: [-1.5, 1.5] rad/s
height_cmd:   0.23 m
```

不要一开始直接开放完整训练范围。

## 4. 腿部任务空间量定义

VMC 使用左右镜像后的任务空间坐标。部署端可以直接复用以下公式，保证和训练一致。

### 4.1 从 URDF joint 角得到 VMC 内部角

实机读取的关节角必须先对齐到 URDF joint position 正方向。记：

```text
q_lf0, q_lf1, q_rf0, q_rf1
dq_lf0, dq_lf1, dq_rf0, dq_rf1
```

训练代码使用：

```text
theta1_L =  q_lf0 + theta1_offset
theta2_L =  q_lf1 + theta2_offset
theta1_R = -q_rf0 + theta1_offset
theta2_R = -q_rf1 + theta2_offset

theta1_dot_L =  dq_lf0
theta2_dot_L =  dq_lf1
theta1_dot_R = -dq_rf0
theta2_dot_R = -dq_rf1
```

参数：

```text
l1 = 0.21665632675675972 m
l2 = 0.2540023491164531 m
offset = -0.007712217793726145 m
theta1_offset = 0.14299916248023697 rad
theta2_offset = 2.406020345452543 rad
```

### 4.2 正运动学

对每条腿分别计算：

```text
end_x = offset + l1*cos(theta1) + l2*cos(theta1 + theta2)
end_y =          l1*sin(theta1) + l2*sin(theta1 + theta2)

L0     = sqrt(end_x^2 + end_y^2)
theta0 = atan2(end_y, end_x) - pi/2
```

含义：

- `L0`：hip 到轮心的等效腿长，单位 m。
- `theta0`：任务空间腿角，0 表示竖直向下。
- `theta0 > 0` 表示在 VMC 内部镜像坐标里腿向一个方向摆动；左右腿已经通过右腿取负完成镜像，所以 policy 看到的左右腿语义一致。
- `L0_dot` 和 `theta0_dot` 训练中用小步前向差分得到，默认 `dt = 0.001 s`。实机可用雅可比或差分，但符号必须和上面镜像后的 `theta_dot` 一致。

## 5. Policy 输出和 VMC 转力矩

ONNX 输出：

```text
actions: shape = [batch, 6]
```

顺序：

```text
a = [a_theta_L, a_L0_L, a_wheel_L, a_theta_R, a_L0_R, a_wheel_R]
```

### 5.1 action 限幅

送入 VMC 前按训练配置限幅：

```text
a_theta_L, a_L0_L, a_theta_R, a_L0_R: clamp 到 [-3.0, 3.0]
a_wheel_L, a_wheel_R:                 clamp 到 [-1.3262599469496021, 1.3262599469496021]
```

### 5.2 action 转任务空间目标

平地 VMC 配置实际覆盖为：

```text
theta0_ref = a_theta * 0.5 + 0.0
L0_ref     = clamp(a_L0 * 0.1 + 0.17, 0.1219258562330587, 0.3006386827708927)
wheel_vel_ref = a_wheel * 52.0
```

单位：

- `theta0_ref`：rad
- `L0_ref`：m
- `wheel_vel_ref`：rad/s，按 URDF wheel joint velocity 正方向

### 5.3 VMC PD

```text
torque_leg = kp_theta * (theta0_ref - theta0) - kd_theta * theta0_dot
force_leg  = kp_l0    * (L0_ref     - L0)     - kd_l0    * L0_dot
```

参数：

```text
kp_theta = 50.0
kd_theta = 3.0
kp_l0 = 900.0
kd_l0 = 20.0
feedforward_force = 40.0 N
```

`force_leg + feedforward_force` 再通过 VMC 雅可比转成 hip/knee 力矩。

### 5.4 轮速 PD

训练中的轮速控制为：

```text
tau_wheel_L = wheel_damping * (wheel_vel_ref_L - dq_l_wheel)
tau_wheel_R = wheel_damping * (wheel_vel_ref_R - dq_r_wheel)
```

平地配置：

```text
wheel_damping = 0.08 Nm*s/rad
wheel torque limit = 4.0 Nm
```

注意这里使用的是 URDF joint velocity 原始方向，不是 observation 中左轮取负后的方向。因此有两个独立概念：

- observation 里：`wheel_vel_obs = [-dq_l_wheel, dq_r_wheel] * 0.05`
- VMC 轮速闭环里：`dq_l_wheel` 和 `dq_r_wheel` 都按 URDF joint velocity 原始方向使用

部署时必须通过架空轮测试确认：

```text
a_wheel_L > 0 时，l_wheel_Joint 的 measured dq_l_wheel 应该朝 URDF 正方向增大
a_wheel_R > 0 时，r_wheel_Joint 的 measured dq_r_wheel 应该朝 URDF 正方向增大
```

如果实机底层定义和 URDF 相反，只改电机到 URDF 的编码器/力矩映射，不要随意改 policy 输出顺序。

### 5.5 输出力矩顺序和右腿符号

VMC 内部算出左右腿镜像坐标下的 `T1, T2` 后，写回 URDF joint 力矩时：

```text
tau_lf0 =  T1_L
tau_lf1 =  T2_L
tau_rf0 = -T1_R
tau_rf1 = -T2_R
tau_l_wheel = tau_wheel_L
tau_r_wheel = tau_wheel_R
```

然后按完整电机顺序输出，例如：

```text
[lf0_Joint, rf0_Joint, lf1_Joint, rf1_Joint, l_wheel_Joint, r_wheel_Joint]
```

对应力矩为：

```text
[tau_lf0, tau_rf0, tau_lf1, tau_rf1, tau_l_wheel, tau_r_wheel]
```

力矩限幅：

```text
hip/knee: 30 Nm
wheel:    4 Nm
```

## 6. ONNX 导出

在项目目录：

```bash
cd /home/sf4/Workspace/rm/rl_wheel_legged/IRobot_wl/IRobot_wl
conda activate rl_wheel_legged
```

示例导出：

```bash
python scripts/rsl_rl/export_onnx.py \
  --run_dir logs/rsl_rl/wl_vmc_flat/2026-06-19_01-45-54 \
  --checkpoint model_4999.pt \
  --output exported/wl_vmc_flat_policy.onnx
```

导出脚本会自动从 checkpoint 推断：

- `num_obs = 27`
- `num_encoder_obs = 135`
- `num_actions = 6`
- `latent_dim = 3`

ONNX 推理时只需要 actor 输入输出：

```text
inputs:
  observations: [1, 27], float32
  observation_history: [1, 135], float32

outputs:
  actions: [1, 6], float32
```

## 7. 实机部署控制周期

训练环境中：

```text
sim dt = 1/200 s
decimation = 4
policy step dt = 0.02 s
policy frequency = 50 Hz
```

实机建议：

- policy / VMC 目标更新：50 Hz。
- 电机底层力矩环：越快越好，至少高于 policy 频率。
- 如果底层只能做速度环或位置环，需要重新设计接口，不建议直接拿这个 VMC policy 上机。

训练里有 `0~10 ms` action delay 随机化，实机总延迟应尽量低于这个范围或稳定可估计。

## 8. 上机前检查清单

### 8.1 坐标和传感器

- `base_link` 坐标：`+X` 前、`+Y` 左、`+Z` 上。
- IMU 角速度已经旋转到 `base_link`。
- `projected_gravity` 直立静止时约为 `[0, 0, -1]`。
- yaw 正方向：绕 base `+Z` 左转为正。

### 8.2 关节方向

逐个电机低力矩/低速度测试，确认：

- 编码器位置增大方向等于 URDF joint 正方向。
- 关节速度增大方向等于 URDF joint 正方向。
- 正力矩能推动关节朝 URDF 正方向加速。

右腿在 VMC 内部会取负镜像，所以部署端输入原始 URDF joint 角即可，不要提前把右腿再翻一次。

### 8.3 轮子方向

架空测试最重要：

- 输入小的 `a_wheel_L > 0`，确认左轮 URDF joint velocity 为正。
- 输入小的 `a_wheel_R > 0`，确认右轮 URDF joint velocity 为正。
- 同时确认 observation 仍然使用 `[-dq_l_wheel, dq_r_wheel] * 0.05`。

如果发现机器人前进/后退反了，先检查轮子 URDF 方向映射，不要直接把 command 或 policy action 取反。

### 8.4 初始姿态

仿真默认初始关节：

```text
lf0_Joint = -0.033355643155 rad
lf1_Joint =  0.012163245493 rad
rf0_Joint =  0.033355643155 rad
rf1_Joint = -0.012163245493 rad
l_wheel_Joint = 0
r_wheel_Joint = 0
```

base 初始高度约：

```text
0.220191 m
```

实机上电时尽量从接近这个站立姿态开始，先用 `vx_cmd = 0`、`yaw_rate_cmd = 0`、`height_cmd = 0.23` 测站稳，再逐步给速度。

## 9. 最小部署伪代码

```python
obs = build_current_observation(
    base_ang_vel_b=imu_gyro_in_base,
    projected_gravity_b=gravity_in_base,
    command=(vx_cmd, yaw_rate_cmd, 0.23),
    urdf_joint_pos=q,
    urdf_joint_vel=dq,
    last_action=last_processed_action,
)

history.push(obs)

actions = onnx_session.run(
    ["actions"],
    {
        "observations": obs[None, :].astype("float32"),
        "observation_history": history.flatten()[None, :].astype("float32"),
    },
)[0][0]

processed_action = clip_action(actions)
tau = compute_vmc_torque(processed_action, q, dq)
send_joint_torque_in_motor_order(tau)
last_processed_action = processed_action
```

`compute_vmc_torque` 应该逐行对齐 `mdp/vmc.py` 中的 `compute_vmc_action()`。sim2real 初期不建议重写一套“看起来等价”的数学，最好直接移植并写单元测试：同一组 `q, dq, action` 输入，Python/实机端输出力矩误差应接近浮点误差。

## 10. 常见问题

### 10.1 一给前进指令就后退

优先检查：

1. 速度指令是否使用 `vx_cmd * 2.0`。
2. base `+X` 是否定义为前方。
3. 轮子 encoder/velocity/torque 是否映射到 URDF 正方向。
4. observation 左轮是否按训练取负：`-dq_l_wheel * 0.05`。

### 10.2 左右转向相反

优先检查：

1. `yaw_rate_cmd` 是否绕 base `+Z` 左转为正。
2. IMU 到 `base_link` 的外参是否正确。
3. 左右轮和左右腿名称是否被交换。

### 10.3 上电后腿突然抽动

优先检查：

1. 关节零位是否和 URDF 一致。
2. `theta1_offset/theta2_offset` 是否重复补偿。
3. 右腿是否被重复镜像。
4. 力矩限幅是否生效：腿 30 Nm、轮 4 Nm。
5. 初始 history 是否合理，不要让 policy 第一帧看到全零姿态。

### 10.4 policy 输出很大

这是正常可能发生的情况，训练中 VMC action 会被限幅。部署端必须执行：

```text
leg action clamp:   [-3.0, 3.0]
wheel action clamp: [-1.3262599469496021, 1.3262599469496021]
```

并且必须做最终力矩限幅。
