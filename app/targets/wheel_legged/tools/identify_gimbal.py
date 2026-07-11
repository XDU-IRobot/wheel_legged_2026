"""
云台动力学参数分步辨识工具

按照文档推荐的顺序逐步辨识 9 个动力学参数：
  1. gravity       — 静态重力项 (theta[3], theta[4])
  2. friction      — 低速摩擦项 (theta[5], theta[6], theta[7], theta[8])
  3. pitch-inertia — Pitch 等效惯量 (theta[2])
  4. coupling      — Yaw-Pitch 耦合惯量 (theta[0], theta[1])
  5. verify        — 综合验证全部 9 参数

参数索引与 C++ Gimbal2DofDynamics 严格一致：
  theta[0] = I1zz_com        theta[3] = m2*l2x (水平偏心)
  theta[1] = I2xx_com        theta[4] = m2*l2z (垂直偏心)
  theta[2] = I2yy_com        theta[5] = fv1     (Yaw 粘性摩擦)
                             theta[6] = fc1     (Yaw 库仑摩擦)
                             theta[7] = fv2     (Pitch 粘性摩擦)
                             theta[8] = fc2     (Pitch 库仑摩擦)

Usage:
  python identify_gimbal.py gravity gravity_static.csv
  python identify_gimbal.py friction friction_sweep.csv --theta34 0.027,-0.026
  python identify_gimbal.py pitch-inertia dynamic_pitch.csv --theta34 0.027,-0.026 --theta78 0.3,0.04
  python identify_gimbal.py coupling q2_m40.csv q2_0.csv q2_p40.csv --theta56 0.2,0.03
  python identify_gimbal.py verify dynamic_harmonics.csv --theta 0.01,0.005,0.03,0.027,-0.026,0.2,0.03,0.3,0.04
"""

import argparse
import sys
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
from sklearn.linear_model import LinearRegression

G_DEFAULT = 9.81


# ---------------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------------

def load_csv(path):
    """读取 CSV 并校验列名。"""
    df = pd.read_csv(path)
    required = {'time', 'q1', 'q2', 'dq1', 'dq2', 'tau1', 'tau2'}
    missing = required - set(df.columns)
    if missing:
        raise ValueError(f"CSV 缺少列: {missing}")
    return df


def parse_theta(s, expected, name):
    """解析逗号分隔的浮点数列表。"""
    parts = [float(x) for x in s.split(',')]
    if len(parts) != expected:
        print(f"错误: --{name} 需要 {expected} 个逗号分隔的数值，实际收到 {len(parts)} 个")
        sys.exit(1)
    return parts


def add_ddq(df, window=11):
    """对 dq1/dq2 做 Savitzky-Golay 滤波后用梯度求 ddq。"""
    dt = float(np.median(np.diff(df['time'].values)))
    if dt <= 0:
        dt = 0.01
    half = window // 2
    if len(df) < window:
        df['ddq1'] = np.gradient(df['dq1'].values, dt)
        df['ddq2'] = np.gradient(df['dq2'].values, dt)
        return df, dt
    dq1_s = savgol_filter(df['dq1'].values, window, 2)
    dq2_s = savgol_filter(df['dq2'].values, window, 2)
    df = df.copy()
    df['ddq1'] = np.gradient(dq1_s, dt)
    df['ddq2'] = np.gradient(dq2_s, dt)
    return df, dt


def fmt_theta(theta):
    """格式化参数为可直接复制到 params.hpp 的 C++ 初始化列表。"""
    parts = ', '.join(f'{v:.7f}f' for v in theta)
    return f'{{{parts}}}'


# ---------------------------------------------------------------------------
# 数据分段检测
# ---------------------------------------------------------------------------

def find_static_segments(df, dq_threshold=0.1, min_duration_s=0.5, dt=None):
    """找到两轴速度都接近零且位置不变的静止段。"""
    if dt is None:
        dt = float(np.median(np.diff(df['time'].values)))
        if dt <= 0:
            dt = 0.01
    dq_mag = np.sqrt(df['dq1'].values ** 2 + df['dq2'].values ** 2)
    min_len = int(min_duration_s / dt)
    if min_len < 5:
        min_len = 5

    is_static = dq_mag < dq_threshold
    # 位置跳变处也切开：处理速度数据 stale 但 q2 确实在移动的情况
    q2 = df['q2'].values
    dq2_from_pos = np.abs(np.diff(q2, prepend=q2[0])) / dt
    is_pos_stable = dq2_from_pos < dq_threshold
    is_static = is_static & is_pos_stable

    segments = []
    start = None
    for i, s in enumerate(is_static):
        if s and start is None:
            start = i
        elif not s and start is not None:
            if i - start >= min_len:
                segments.append((start, i))
            start = None
    if start is not None and len(is_static) - start >= min_len:
        segments.append((start, len(is_static)))
    return segments


def find_const_vel_segments(dq, dq_stability=0.03, min_duration_s=0.5, dt=None):
    """找到速度近似常数（标准差小）且非零的匀速段。"""
    if dt is None:
        dt = 0.01
    min_len = int(min_duration_s / dt)
    if min_len < 5:
        min_len = 5

    window = max(min_len, 11)
    if window % 2 == 0:
        window += 1
    if len(dq) < window:
        return []

    dq_std = pd.Series(dq).rolling(window, center=True, min_periods=window // 2).std().values
    stable = (dq_std < dq_stability) & (np.abs(dq) > 0.02)

    segments = []
    start = None
    for i, s in enumerate(stable):
        if s and start is None:
            start = i
        elif not s and start is not None:
            if i - start >= min_len:
                segments.append((start, i))
            start = None
    if start is not None and len(stable) - start >= min_len:
        segments.append((start, len(stable)))
    return segments


# ---------------------------------------------------------------------------
# Step 1: 静态重力项辨识
# ---------------------------------------------------------------------------

def cmd_gravity(args):
    df = load_csv(args.csv)
    df, dt = add_ddq(df)

    segments = find_static_segments(df, args.dq_threshold, args.min_duration, dt)

    if len(segments) < 3:
        print(f"错误: 仅检测到 {len(segments)} 个静止段 (需要 >= 3 个)。")
        print("请确认数据包含多个不同角度的 Pitch 静止悬停段。")
        print(f"当前 dq 阈值 = {args.dq_threshold} rad/s, 最小时长 = {args.min_duration} s")
        sys.exit(1)

    q2_means, tau2_means = [], []
    for start, end in segments:
        q2_means.append(float(np.mean(df['q2'].values[start:end])))
        tau2_means.append(float(np.mean(df['tau2'].values[start:end])))

    q2_enc_arr = np.array(q2_means)
    tau2_arr = np.array(tau2_means)

    # q2 从编码器坐标转到模型坐标 (q2=0 为 pitch 水平)
    q2_model_arr = q2_enc_arr - args.pitch_offset

    # 模型: tau2 = A*cos(q2) + B*sin(q2) + C
    # 对应 C++: tau2_gravity = theta[3]*(-g*cos(q2)) + theta[4]*(-g*sin(q2))
    # 所以 A = -g * theta[3]  →  theta[3] = -A / g
    #     B = -g * theta[4]  →  theta[4] = -B / g
    X = np.column_stack([np.cos(q2_model_arr), np.sin(q2_model_arr), np.ones_like(q2_model_arr)])
    reg = LinearRegression(fit_intercept=False)
    reg.fit(X, tau2_arr)
    A, B, C = reg.coef_

    g = args.g
    theta3 = -A / g
    theta4 = -B / g

    tau2_pred = reg.predict(X)
    r2 = reg.score(X, tau2_arr)

    print(f"\n{'=' * 60}")
    print(f"Step 1: 静态重力项辨识  →  theta[3], theta[4]")
    print(f"{'=' * 60}")
    print(f"  pitch-offset = {args.pitch_offset:.4f} rad ({np.rad2deg(args.pitch_offset):.1f}°)")
    print(f"  使用静止段数: {len(segments)}")
    for i, (s, e) in enumerate(segments):
        q2_deg = np.rad2deg(q2_enc_arr[i])
        q2m_deg = np.rad2deg(q2_model_arr[i])
        print(f"    [{i}] q2={q2_deg:.1f}° (model:{q2m_deg:.1f}°)  tau2={tau2_arr[i]:.4f}  (行 {s}-{e})")

    print(f"\n  拟合: tau2 = A*cos(q2_model) + B*sin(q2_model) + C")
    print(f"    A  = {A:.6f}")
    print(f"    B  = {B:.6f}")
    print(f"    C  = {C:.6f}")
    print(f"    R² = {r2:.4f}")

    print(f"\n  辨识结果:")
    print(f"    theta[3] (m2*l2x, 水平偏心) = {theta3:.6f}")
    print(f"    theta[4] (m2*l2z, 垂直偏心) = {theta4:.6f}")

    # 偏置过大的提醒
    max_ab = max(abs(A), abs(B), 0.01)
    if abs(C) > 0.1 * max_ab:
        print(f"\n  *** 注意: 常值偏置 C={C:.4f} 偏大，建议检查 ***")
        print(f"      可能原因: 电流零偏 / 线缆拉力 / 编码器零位 / 力矩方向")

    print(f"\n  下一步 (friction) 请使用:")
    print(f"    --theta34 {theta3:.6f},{theta4:.6f}")

    return theta3, theta4


# ---------------------------------------------------------------------------
# Step 2: 摩擦项辨识
# ---------------------------------------------------------------------------

def cmd_friction(args):
    theta3, theta4 = parse_theta(args.theta34, 2, 'theta34')

    df = load_csv(args.csv)
    df, dt = add_ddq(df)

    g = args.g
    q2 = df['q2'].values
    dq1 = df['dq1'].values
    dq2 = df['dq2'].values
    tau1 = df['tau1'].values
    tau2 = df['tau2'].values

    # Pitch 先扣重力
    tau2_gravity = -g * (theta3 * np.cos(q2) + theta4 * np.sin(q2))
    tau2_no_g = tau2 - tau2_gravity

    # ── Yaw 摩擦 ──
    print(f"\n{'=' * 60}")
    print(f"Step 2a: Yaw 摩擦辨识  →  theta[5] (fv1), theta[6] (fc1)")
    print(f"{'=' * 60}")

    yaw_segs = find_const_vel_segments(dq1, args.dq_stability, args.min_duration, dt)

    if len(yaw_segs) < 3:
        print(f"  警告: 仅找到 {len(yaw_segs)} 个 Yaw 匀速段，theta[5]/theta[6] 置零")
        theta5, theta6 = 0.0, 0.0
    else:
        dq1_pts, tau1_pts = [], []
        for s, e in yaw_segs:
            dq1_pts.append(float(np.mean(dq1[s:e])))
            tau1_pts.append(float(np.mean(tau1[s:e])))

        dq1_arr = np.array(dq1_pts)
        tau1_arr = np.array(tau1_pts)

        X = np.column_stack([dq1_arr, np.tanh(dq1_arr)])
        reg = LinearRegression(fit_intercept=False)
        reg.fit(X, tau1_arr)
        theta5, theta6 = reg.coef_
        r2_yaw = reg.score(X, tau1_arr)

        print(f"  使用匀速段数: {len(yaw_segs)}")
        for i, (s, e) in enumerate(yaw_segs):
            print(f"    [{i}] dq1={dq1_pts[i]:.4f} rad/s  tau1={tau1_pts[i]:.4f}  (行 {s}-{e})")
        print(f"\n  拟合: tau1 = theta[5]*dq1 + theta[6]*tanh(dq1)")
        print(f"    theta[5] (fv1, 粘性) = {theta5:.6f}")
        print(f"    theta[6] (fc1, 库仑) = {theta6:.6f}")
        print(f"    R² = {r2_yaw:.4f}")

    # ── Pitch 摩擦 ──
    print(f"\n{'=' * 60}")
    print(f"Step 2b: Pitch 摩擦辨识  →  theta[7] (fv2), theta[8] (fc2)")
    print(f"{'=' * 60}")

    pitch_segs = find_const_vel_segments(dq2, args.dq_stability, args.min_duration, dt)

    if len(pitch_segs) < 3:
        print(f"  警告: 仅找到 {len(pitch_segs)} 个 Pitch 匀速段，theta[7]/theta[8] 置零")
        theta7, theta8 = 0.0, 0.0
    else:
        dq2_pts, tau2_pts = [], []
        for s, e in pitch_segs:
            dq2_pts.append(float(np.mean(dq2[s:e])))
            tau2_pts.append(float(np.mean(tau2_no_g[s:e])))

        dq2_arr = np.array(dq2_pts)
        tau2_arr = np.array(tau2_pts)

        X = np.column_stack([dq2_arr, np.tanh(dq2_arr)])
        reg = LinearRegression(fit_intercept=False)
        reg.fit(X, tau2_arr)
        theta7, theta8 = reg.coef_
        r2_pitch = reg.score(X, tau2_arr)

        print(f"  使用匀速段数: {len(pitch_segs)}")
        for i, (s, e) in enumerate(pitch_segs):
            print(f"    [{i}] dq2={dq2_pts[i]:.4f} rad/s  tau2_no_g={tau2_pts[i]:.4f}  (行 {s}-{e})")
        print(f"\n  拟合: tau2_no_g = theta[7]*dq2 + theta[8]*tanh(dq2)")
        print(f"    theta[7] (fv2, 粘性) = {theta7:.6f}")
        print(f"    theta[8] (fc2, 库仑) = {theta8:.6f}")
        print(f"    R² = {r2_pitch:.4f}")

    print(f"\n  下一步 (pitch-inertia) 请使用:")
    print(f"    --theta34 {theta3:.6f},{theta4:.6f} --theta78 {theta7:.6f},{theta8:.6f}")

    return theta5, theta6, theta7, theta8


# ---------------------------------------------------------------------------
# Step 3: Pitch 惯量辨识
# ---------------------------------------------------------------------------

def cmd_pitch_inertia(args):
    theta3, theta4 = parse_theta(args.theta34, 2, 'theta34')
    theta7, theta8 = parse_theta(args.theta78, 2, 'theta78')

    df = load_csv(args.csv)
    df, dt = add_ddq(df)

    g = args.g
    q2 = df['q2'].values
    dq1 = df['dq1'].values
    dq2 = df['dq2'].values
    ddq2 = df['ddq2'].values
    tau2 = df['tau2'].values

    # 扣除重力和摩擦
    tau2_gravity = -g * (theta3 * np.cos(q2) + theta4 * np.sin(q2))
    tau2_friction = theta7 * dq2 + theta8 * np.tanh(dq2)
    tau2_residual = tau2 - tau2_gravity - tau2_friction

    # 筛选: Yaw 基本不动 且 Pitch 有明显的加速度
    mask = (np.abs(dq1) < 0.15) & (np.abs(ddq2) > 0.05)

    if np.sum(mask) < 20:
        print(f"错误: 有效数据点不足 ({np.sum(mask)} 个)。")
        print("请确认: Yaw 固定不动，Pitch 做低频平滑加减速运动。")
        sys.exit(1)

    X = ddq2[mask].reshape(-1, 1)
    y = tau2_residual[mask]

    reg = LinearRegression(fit_intercept=False)
    reg.fit(X, y)
    theta2 = float(reg.coef_[0])
    r2 = reg.score(X, y)

    print(f"\n{'=' * 60}")
    print(f"Step 3: Pitch 惯量辨识  →  theta[2] (I2yy_com)")
    print(f"{'=' * 60}")
    print(f"  有效数据点: {np.sum(mask)} / {len(df)}")
    print(f"\n  拟合: tau2_residual = theta[2] * ddq2")
    print(f"    theta[2] (I2yy_com) = {theta2:.6f}")
    print(f"    R² = {r2:.4f}")

    if theta2 < 0:
        print(f"\n  *** 注意: 惯量为负值! 请检查 ***")
        print(f"      - ddq 信号质量 (是否由噪声差分得到)")
        print(f"      - 轨迹是否跟丢")
        print(f"      - 重力和摩擦是否已正确扣除")

    print(f"\n  下一步 (coupling) 请使用:")
    print(f"    --theta56 <theta5>,<theta6>  (来自 friction 步骤的 Yaw 摩擦)")

    return theta2


# ---------------------------------------------------------------------------
# Step 4: Yaw-Pitch 耦合惯量辨识
# ---------------------------------------------------------------------------

def cmd_coupling(args):
    theta5, theta6 = parse_theta(args.theta56, 2, 'theta56')

    dfs = [load_csv(p) for p in args.csv]
    for i, (df, p) in enumerate(zip(dfs, args.csv)):
        df, _ = add_ddq(df)
        df['_file'] = i
        dfs[i] = df
    df_all = pd.concat(dfs, ignore_index=True)

    q1 = df_all['q1'].values
    q2 = df_all['q2'].values
    dq1 = df_all['dq1'].values
    dq2 = df_all['dq2'].values
    ddq1 = df_all['ddq1'].values
    tau1 = df_all['tau1'].values

    # 扣除 Yaw 摩擦
    tau1_residual = tau1 - theta5 * dq1 - theta6 * np.tanh(dq1)

    sin_q2 = np.sin(q2)
    cos_q2 = np.cos(q2)
    sin2_q2 = sin_q2 ** 2
    cos2_q2 = cos_q2 ** 2
    sin_2q2 = 2.0 * sin_q2 * cos_q2

    # C++ 模型 Yaw 惯性项:
    # Y(0,0) = sin²(q2)*ddq1 + sin(2q2)*dq1*dq2   → theta[0]
    # Y(0,1) = cos²(q2)*ddq1 - sin(2q2)*dq1*dq2   → theta[1]
    col0 = sin2_q2 * ddq1 + sin_2q2 * dq1 * dq2
    col1 = cos2_q2 * ddq1 - sin_2q2 * dq1 * dq2

    # 筛选: 有明显 Yaw 加速度 且 力矩未饱和
    mask = (np.abs(ddq1) > 0.02) & (np.abs(tau1_residual) < 8.0)

    if np.sum(mask) < 20:
        print(f"错误: 有效数据点不足 ({np.sum(mask)} 个)。")
        print("请确认数据包含 Yaw 轴明显的加减速运动。")
        sys.exit(1)

    X = np.column_stack([col0[mask], col1[mask]])
    y = tau1_residual[mask]

    reg = LinearRegression(fit_intercept=False)
    reg.fit(X, y)
    theta0, theta1 = float(reg.coef_[0]), float(reg.coef_[1])
    r2 = reg.score(X, y)

    q2_range = float(np.max(q2[mask]) - np.min(q2[mask]))

    print(f"\n{'=' * 60}")
    print(f"Step 4: Yaw-Pitch 耦合惯量辨识  →  theta[0], theta[1]")
    print(f"{'=' * 60}")
    print(f"  数据文件: {args.csv}")
    print(f"  有效数据点: {np.sum(mask)} / {len(df_all)}")
    print(f"  q2 范围: {np.rad2deg(np.min(q2[mask])):.1f}° ~ {np.rad2deg(np.max(q2[mask])):.1f}°")
    print(f"\n  拟合: tau1_residual = theta[0]*Y(0,0) + theta[1]*Y(0,1)")
    print(f"    theta[0] (I1zz_com) = {theta0:.6f}")
    print(f"    theta[1] (I2xx_com) = {theta1:.6f}")
    print(f"    R² = {r2:.4f}")

    if q2_range < 0.25:
        print(f"\n  *** 注意: q2 范围仅 {np.rad2deg(q2_range):.1f}°, theta[0] 可能辨识不准 ***")
        print(f"      建议在更大 pitch 角度范围采集数据 (如 -40° ~ +40°)")

    # 汇总所有已知参数
    print(f"\n  {'─' * 50}")
    print(f"  汇总 (需手动将前几步结果填入下面命令):")
    print(f"  python identify_gimbal.py verify <data.csv> \\")
    print(f"      --theta {theta0:.6f},{theta1:.6f},<theta2>,<theta3>,<theta4>,{theta5:.6f},{theta6:.6f},<theta7>,<theta8>")

    return theta0, theta1


# ---------------------------------------------------------------------------
# Step 5: 综合验证
# ---------------------------------------------------------------------------

def cmd_verify(args):
    theta = parse_theta(args.theta, 9, 'theta')

    df = load_csv(args.csv)
    df, _ = add_ddq(df)

    g = args.g
    gx, gy, gz = 0.0, 0.0, -g  # 静止基座

    q1 = df['q1'].values
    q2 = df['q2'].values
    dq1 = df['dq1'].values
    dq2 = df['dq2'].values
    ddq1 = df['ddq1'].values
    ddq2 = df['ddq2'].values
    tau1_meas = df['tau1'].values
    tau2_meas = df['tau2'].values

    # ── 前向计算: tau = Y * theta (与 C++ Gimbal2DofDynamics::ComputeY 一致) ──
    sin_q1 = np.sin(q1)
    cos_q1 = np.cos(q1)
    sin_q2 = np.sin(q2)
    cos_q2 = np.cos(q2)
    sin_2q2 = 2.0 * sin_q2 * cos_q2
    sin2_q2 = sin_q2 * sin_q2
    cos2_q2 = cos_q2 * cos_q2
    dq1_sq = dq1 * dq1
    dq1_dq2 = dq1 * dq2

    gx_sin_gy_cos_q1 = gx * sin_q1 - gy * cos_q1
    gx_cos_gy_sin_q1 = gx * cos_q1 + gy * sin_q1

    # Yaw (与 C++ gimbal_dynamics.hpp:94-99 完全一致)
    tau1_pred = (
        theta[0] * (sin2_q2 * ddq1 + sin_2q2 * dq1_dq2) +
        theta[1] * (cos2_q2 * ddq1 - sin_2q2 * dq1_dq2) +
        theta[3] * gx_sin_gy_cos_q1 * cos_q2 +
        theta[4] * gx_sin_gy_cos_q1 * sin_q2 +
        theta[5] * dq1 +
        theta[6] * np.tanh(dq1)
    )

    # Pitch (与 C++ gimbal_dynamics.hpp:102-108 完全一致)
    tau2_pred = (
        theta[0] * (-0.5 * sin_2q2 * dq1_sq) +
        theta[1] * (0.5 * sin_2q2 * dq1_sq) +
        theta[2] * ddq2 +
        theta[3] * (gx_cos_gy_sin_q1 * sin_q2 + gz * cos_q2) +
        theta[4] * (-gx_cos_gy_sin_q1 * cos_q2 + gz * sin_q2) +
        theta[7] * dq2 +
        theta[8] * np.tanh(dq2)
    )

    # ── 评估指标 ──
    def r2_score(y_true, y_pred):
        ss_res = np.sum((y_true - y_pred) ** 2)
        ss_tot = np.sum((y_true - np.mean(y_true)) ** 2)
        return float(1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else 0.0

    r2_yaw = r2_score(tau1_meas, tau1_pred)
    r2_pitch = r2_score(tau2_meas, tau2_pred)
    rmse_yaw = float(np.sqrt(np.mean((tau1_meas - tau1_pred) ** 2)))
    rmse_pitch = float(np.sqrt(np.mean((tau2_meas - tau2_pred) ** 2)))
    mae_yaw = float(np.mean(np.abs(tau1_meas - tau1_pred)))
    mae_pitch = float(np.mean(np.abs(tau2_meas - tau2_pred)))

    print(f"\n{'=' * 60}")
    print(f"Step 5: 综合验证")
    print(f"{'=' * 60}")

    print(f"\n  当前参数:")
    print(f"    theta[0] (I1zz_com) = {theta[0]:.7f}")
    print(f"    theta[1] (I2xx_com) = {theta[1]:.7f}")
    print(f"    theta[2] (I2yy_com) = {theta[2]:.7f}")
    print(f"    theta[3] (m2*l2x)   = {theta[3]:.7f}")
    print(f"    theta[4] (m2*l2z)   = {theta[4]:.7f}")
    print(f"    theta[5] (fv1)       = {theta[5]:.7f}")
    print(f"    theta[6] (fc1)       = {theta[6]:.7f}")
    print(f"    theta[7] (fv2)       = {theta[7]:.7f}")
    print(f"    theta[8] (fc2)       = {theta[8]:.7f}")

    print(f"\n  Yaw 轴:")
    print(f"    R²   = {r2_yaw:.4f}")
    print(f"    RMSE = {rmse_yaw:.4f}")
    print(f"    MAE  = {mae_yaw:.4f}")

    print(f"\n  Pitch 轴:")
    print(f"    R²   = {r2_pitch:.4f}")
    print(f"    RMSE = {rmse_pitch:.4f}")
    print(f"    MAE  = {mae_pitch:.4f}")

    # ── 诊断 ──
    issues = []
    if r2_yaw < 0.6:
        issues.append("Yaw R² 偏低 → 检查 theta[0]/theta[1] (耦合惯量) 和 theta[5]/theta[6] (Yaw 摩擦)")
    if r2_pitch < 0.6:
        issues.append("Pitch R² 偏低 → 检查 theta[3]/theta[4] (重力), theta[2] (惯量), theta[7]/theta[8] (摩擦)")
    if np.max(np.abs(tau1_meas)) > 9.5 or np.max(np.abs(tau2_meas)) > 9.5:
        issues.append("检测到力矩可能饱和 → 部分数据不适用于线性辨识")

    if issues:
        print(f"\n  诊断建议:")
        for issue in issues:
            print(f"    - {issue}")

    print(f"\n  C++ 初始化列表 (可直接复制到 params.hpp):")
    print(f"    {fmt_theta(theta)}")


# ---------------------------------------------------------------------------
# 命令行入口
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='云台动力学参数分步辨识工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # Step 1: 静态重力项
  python identify_gimbal.py gravity gravity_static.csv

  # Step 2: 低速摩擦项 (需要 θ3,θ4)
  python identify_gimbal.py friction friction_sweep.csv --theta34 0.027,-0.026

  # Step 3: Pitch 惯量 (需要 θ3,θ4 和 θ7,θ8)
  python identify_gimbal.py pitch-inertia dynamic_pitch.csv --theta34 0.027,-0.026 --theta78 0.3,0.04

  # Step 4: 耦合惯量 (需要 θ5,θ6)
  python identify_gimbal.py coupling data_q2_m40.csv data_q2_0.csv data_q2_p40.csv --theta56 0.2,0.03

  # Step 5: 综合验证
  python identify_gimbal.py verify dynamic_harmonics.csv --theta 0.01,0.005,0.03,0.027,-0.026,0.2,0.03,0.3,0.04
        """,
    )

    sub = parser.add_subparsers(dest='cmd', required=True)

    # --- gravity ---
    p = sub.add_parser('gravity', help='Step 1: 静态重力项辨识 (theta[3], theta[4])')
    p.add_argument('csv', help='Pitch 静态多角度悬停数据 CSV')
    p.add_argument('--pitch-offset', type=float, default=0.0,
                   help='编码器零位偏移，即 kIdentPitchCenter (rad). q2_model = q2_enc - offset')
    p.add_argument('--g', type=float, default=G_DEFAULT, help='重力加速度 (m/s²)')
    p.add_argument('--dq-threshold', type=float, default=0.05, help='静止判定的 |dq| 阈值 (rad/s)')
    p.add_argument('--min-duration', type=float, default=0.5, help='静止段最短持续时间 (s)')

    # --- friction ---
    p = sub.add_parser('friction', help='Step 2: 低速摩擦项辨识 (theta[5..8])')
    p.add_argument('csv', help='Yaw/Pitch 低速匀速扫角数据 CSV')
    p.add_argument('--theta34', required=True, help='theta[3],theta[4] (来自 gravity 步骤)')
    p.add_argument('--g', type=float, default=G_DEFAULT)
    p.add_argument('--dq-stability', type=float, default=0.03, help='匀速判定的 dq 标准差阈值 (rad/s)')
    p.add_argument('--min-duration', type=float, default=0.5, help='匀速段最短持续时间 (s)')

    # --- pitch-inertia ---
    p = sub.add_parser('pitch-inertia', help='Step 3: Pitch 惯量辨识 (theta[2])')
    p.add_argument('csv', help='Yaw 固定、Pitch 低频加减速数据 CSV')
    p.add_argument('--theta34', required=True, help='theta[3],theta[4] (来自 gravity 步骤)')
    p.add_argument('--theta78', required=True, help='theta[7],theta[8] (来自 friction 步骤的 Pitch 摩擦)')
    p.add_argument('--g', type=float, default=G_DEFAULT)

    # --- coupling ---
    p = sub.add_parser('coupling', help='Step 4: 耦合惯量辨识 (theta[0], theta[1])')
    p.add_argument('csv', nargs='+', help='多个 q2 姿态下的 Yaw 加减速数据 CSV (可指定多个文件)')
    p.add_argument('--theta56', required=True, help='theta[5],theta[6] (来自 friction 步骤的 Yaw 摩擦)')

    # --- verify ---
    p = sub.add_parser('verify', help='Step 5: 综合验证全部 9 参数')
    p.add_argument('csv', help='五次谐波综合轨迹数据 CSV')
    p.add_argument('--theta', required=True, help='全部 9 参数: theta0,theta1,...,theta8')
    p.add_argument('--g', type=float, default=G_DEFAULT)

    args = parser.parse_args()

    if args.cmd == 'gravity':
        cmd_gravity(args)
    elif args.cmd == 'friction':
        cmd_friction(args)
    elif args.cmd == 'pitch-inertia':
        cmd_pitch_inertia(args)
    elif args.cmd == 'coupling':
        cmd_coupling(args)
    elif args.cmd == 'verify':
        cmd_verify(args)


if __name__ == '__main__':
    main()
