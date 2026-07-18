"""
云台动力学参数分步辨识工具

按照文档推荐的顺序逐步辨识 9 个动力学参数：
  1. gravity       — 静态重力项检查（可选）
  2. friction      — Pitch 重力/摩擦联合辨识 + Yaw 摩擦辨识
  3. pitch-inertia — Pitch 等效惯量 theta[2] (多频率正弦, 谐波拟合)
  4. coupling      — Yaw-Pitch 耦合惯量 theta[0], theta[1] (两步法)
  5. verify        — 综合验证全部 9 参数

参数索引与 C++ Gimbal2DofDynamics 严格一致：
  theta[0] = I1zz_com        theta[3] = m2*l2x (水平偏心)
  theta[1] = I2xx_com        theta[4] = m2*l2z (垂直偏心)
  theta[2] = I2yy_com        theta[5] = fv1     (Yaw 粘性摩擦)
                             theta[6] = fc1     (Yaw 库仑摩擦)
                             theta[7] = fv2     (Pitch 粘性摩擦)
                             theta[8] = fc2     (Pitch 库仑摩擦)

耦合惯量辨识 (coupling) 采用两步法:
  Step A — 每 pitch 角度独立拟合有效 yaw 惯量:
    I_eff(q2) = tau1_residual / ddq1  (单参数, 无共线性问题)
  Step B — 跨角度拟合 I1zz / I2xx:
    I_eff = I2xx + (I1zz - I2xx) * sin²(q2)
  要求: 多个固定 pitch 角度下 yaw 做正弦激励 (≥1 Hz, 大振幅接近 360°)

Usage:
  python identify_gimbal.py gravity gravity_static.csv
  python identify_gimbal.py friction friction_sweep.csv
  python identify_gimbal.py pitch-inertia dynamic_pitch.csv --theta34 0.027,-0.026 --theta78 0.3,0.04 --pitch-bias 0.1
  python identify_gimbal.py coupling q2_m40.csv --theta56 0.2,0.03
  python identify_gimbal.py verify dynamic_harmonics.csv --theta 0.01,0.005,0.03,0.027,-0.026,0.2,0.03,0.3,0.04
"""

import argparse
import sys
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter
from scipy.optimize import lsq_linear
from sklearn.linear_model import LinearRegression

sys.stdout.reconfigure(encoding='utf-8')

G_DEFAULT = 9.81
FRICTION_VELOCITY_SCALE_DEFAULT = 0.02


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


def add_ddq(df, window=21):
    """用 Savitzky-Golay 直接从位置求二阶导 (deriv=2)，避免滤波-差分级联放大噪声。

    默认窗口 21 (~0.42s at 50Hz)，适合 0.3–5 Hz 辨识轨迹。
    窗口越大滤波越强，但过高频信号会被衰减。可用 --window 调整。
    """
    dt = float(np.median(np.diff(df['time'].values)))
    if dt <= 0:
        dt = 0.01
    w = min(window, len(df) // 4 * 2 + 1)  # 奇数窗口
    if w < 7:
        w = 7
    poly = 3

    df = df.copy()
    df['ddq1'] = savgol_filter(df['q1'].values, w, poly, deriv=2, delta=dt)
    df['ddq2'] = savgol_filter(df['q2'].values, w, poly, deriv=2, delta=dt)
    return df, dt


def add_ddq_from_ref(df):
    """从参考轨迹 (q1_ref, q2_ref) 计算干净 ddq，避免数值微分噪声。

    固件参考轨迹是平滑的正弦叠加信号，两次求导几乎无噪声放大。
    在 Harmonic/Coupling 等模式下，电机跟踪良好（误差 < 1°），
    ddq_ref ≈ ddq_actual，替代数值 ddq 用于模型验证。
    """
    t = df['time'].values
    dt = float(np.median(np.diff(t)))
    if dt <= 0:
        dt = 0.01

    # Yaw 需先解缠绕，避免 ±π 跳变导致导数尖峰
    q1_ref = np.unwrap(df['q1_ref'].values)
    # Pitch 直接求导（不存在缠绕问题）
    q2_ref = df['q2_ref'].values

    from scipy.signal import savgol_filter
    w = min(11, len(df) // 4 * 2 + 1)  # 奇数窗口
    if w >= 5:
        q1_s = savgol_filter(q1_ref, w, 2)
        q2_s = savgol_filter(q2_ref, w, 2)
    else:
        q1_s, q2_s = q1_ref, q2_ref

    df = df.copy()
    df['ddq1_ref_clean'] = np.gradient(np.gradient(q1_s, dt), dt)
    df['ddq2_ref_clean'] = np.gradient(np.gradient(q2_s, dt), dt)
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


def find_motion_periods(dq, vel_threshold=0.005, min_duration_s=0.3, merge_gap_s=0.5, dt=None, time=None):
    """找到所有速度超过阈值的连续运动段。

    与 find_const_vel_segments 不同，此函数不要求速度恒定，
    适用于 pitch 轴等受重力/摩擦影响、速度难以保持恒定的关节。

    当提供 time 参数时，先在时间间隙 > merge_gap_s 处切分数据块，
    每个时间连续块直接作为一个运动段（固件仅在运动期间输出数据，
    stick-slip 零速穿越点不会导致错误切分）。
    """
    if dt is None:
        dt = 0.01
    min_len = int(min_duration_s / dt)
    if min_len < 5:
        min_len = 5

    if time is not None and len(time) > 1:
        time_diffs = np.diff(time)
        split_points = [0] + (np.where(time_diffs > merge_gap_s)[0] + 1).tolist() + [len(dq)]

        segments = []
        for a, b in zip(split_points[:-1], split_points[1:]):
            if b - a < min_len:
                continue
            chunk_dq = dq[a:b]
            moving = np.where(np.abs(chunk_dq) > vel_threshold)[0]
            if len(moving) < min_len:
                continue
            start = a + moving[0]
            end = a + moving[-1] + 1
            if end - start >= min_len:
                segments.append((start, end))
        return segments

    # 无 time 参数的兼容路径：纯速度阈值检测
    merge_gap = int(merge_gap_s / dt)

    moving = np.abs(dq) > vel_threshold
    segments = []
    start = None
    for i, s in enumerate(moving):
        if s and start is None:
            start = i
        elif not s and start is not None:
            if i - start >= min_len:
                segments.append((start, i))
            start = None
    if start is not None and len(moving) - start >= min_len:
        segments.append((start, len(moving)))

    if merge_gap > 0 and len(segments) > 1:
        merged = [segments[0]]
        for seg in segments[1:]:
            if seg[0] - merged[-1][1] <= merge_gap:
                merged[-1] = (merged[-1][0], seg[1])
            else:
                merged.append(seg)
        segments = merged

    return segments


def average_bidirectional_static_points(q2_points, tau2_points):
    """检测 0→N-1→0 三角扫描，并将同一目标角的正反向静止点配对平均。"""
    q2_points = np.asarray(q2_points, dtype=float)
    tau2_points = np.asarray(tau2_points, dtype=float)
    count = len(q2_points)

    # 标准往返序列包含 2N-1 个静止段，转折点位于序列中央。
    if count < 5 or count % 2 == 0:
        return q2_points, tau2_points, None

    turn = count // 2
    span = q2_points[turn] - q2_points[0]
    if abs(span) < 0.1:
        return q2_points, tau2_points, None

    direction = np.sign(span)
    forward_steps = direction * np.diff(q2_points[:turn + 1])
    reverse_steps = direction * np.diff(q2_points[turn:])
    if np.any(forward_steps <= 1e-3) or np.any(reverse_steps >= -1e-3):
        return q2_points, tau2_points, None

    # 返回端应回到起始角附近；容许位置环静差和静摩擦造成少量角度差。
    endpoint_tolerance = max(0.15, 0.25 * abs(span))
    if abs(q2_points[-1] - q2_points[0]) > endpoint_tolerance:
        return q2_points, tau2_points, None

    q2_averaged, tau2_averaged = [], []
    pair_tau_differences = []
    pair_q_differences = []
    for i in range(turn):
        j = count - 1 - i
        q2_averaged.append(0.5 * (q2_points[i] + q2_points[j]))
        tau2_averaged.append(0.5 * (tau2_points[i] + tau2_points[j]))
        pair_q_differences.append(q2_points[i] - q2_points[j])
        pair_tau_differences.append(tau2_points[i] - tau2_points[j])

    # 转折端只测一次，直接作为该目标角的代表点。
    q2_averaged.append(q2_points[turn])
    tau2_averaged.append(tau2_points[turn])

    info = {
        'raw_count': count,
        'averaged_count': len(q2_averaged),
        'pair_q_differences': np.asarray(pair_q_differences),
        'pair_tau_differences': np.asarray(pair_tau_differences),
    }
    return np.asarray(q2_averaged), np.asarray(tau2_averaged), info


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

    q2_raw_arr = np.array(q2_means)
    tau2_raw_arr = np.array(tau2_means)
    q2_enc_arr, tau2_arr, bidirectional_info = average_bidirectional_static_points(q2_raw_arr, tau2_raw_arr)

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
    print(f"  检测到静止段数: {len(segments)}")
    for i, (s, e) in enumerate(segments):
        q2_deg = np.rad2deg(q2_raw_arr[i])
        q2m_deg = q2_deg - np.rad2deg(args.pitch_offset)
        print(f"    [{i}] q2={q2_deg:.1f}° (model:{q2m_deg:.1f}°)  tau2={tau2_raw_arr[i]:.4f}  (行 {s}-{e})")

    if bidirectional_info is not None:
        tau_diff = np.abs(bidirectional_info['pair_tau_differences'])
        q_diff_deg = np.abs(np.rad2deg(bidirectional_info['pair_q_differences']))
        print(f"\n  检测到正向/反向往返扫描，已按同一目标角配对平均:")
        print(f"    原始静止段: {bidirectional_info['raw_count']}  →  拟合点: {bidirectional_info['averaged_count']}")
        print(f"    正反向力矩差: mean={np.mean(tau_diff):.4f} Nm, max={np.max(tau_diff):.4f} Nm")
        print(f"    正反向角度差: mean={np.mean(q_diff_deg):.2f}°, max={np.max(q_diff_deg):.2f}°")

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

    print(f"\n  本结果仅用于静态交叉验证。正式参数请运行联合辨识:")
    print("    python identify_gimbal.py friction friction_sweep.csv")

    return theta3, theta4


# ---------------------------------------------------------------------------
# Step 2: 摩擦项辨识
# ---------------------------------------------------------------------------

def cmd_friction(args):
    if args.friction_velocity_scale <= 0.0:
        raise ValueError("--friction-velocity-scale 必须大于 0")

    df = load_csv(args.csv)
    df, dt = add_ddq(df)

    g = args.g
    q2 = df['q2'].values
    dq1 = df['dq1'].values
    dq2 = df['dq2'].values
    tau1 = df['tau1'].values
    tau2 = df['tau2'].values

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

        X = np.column_stack([dq1_arr, np.tanh(dq1_arr / args.friction_velocity_scale)])
        reg = LinearRegression(fit_intercept=False)
        reg.fit(X, tau1_arr)
        theta5, theta6 = reg.coef_
        r2_yaw = reg.score(X, tau1_arr)

        print(f"  使用匀速段数: {len(yaw_segs)}")
        for i, (s, e) in enumerate(yaw_segs):
            print(f"    [{i}] dq1={dq1_pts[i]:.4f} rad/s  tau1={tau1_pts[i]:.4f}  (行 {s}-{e})")
        print(f"\n  拟合: tau1 = theta[5]*dq1 + theta[6]*tanh(dq1/{args.friction_velocity_scale:g})")
        print(f"    theta[5] (fv1, 粘性) = {theta5:.6f}")
        print(f"    theta[6] (fc1, 库仑) = {theta6:.6f}")
        print(f"    R² = {r2_yaw:.4f}")

    # ── Pitch 重力 + 偏置 + 摩擦联合辨识 ──
    use_known_gravity = hasattr(args, 'theta34') and args.theta34 is not None

    if use_known_gravity:
        print(f"\n{'=' * 60}")
        print(f"Step 2b: Pitch 摩擦辨识 (使用已知重力项)")
        print(f"  → theta[7], theta[8]")
        print(f"{'=' * 60}")
        theta3_known, theta4_known = parse_theta(args.theta34, 2, 'theta34')
        pitch_bias_known = args.pitch_bias if args.pitch_bias is not None else 0.0
        A_known = -theta3_known * g
        B_known = -theta4_known * g
        print(f"  已知 theta[3]={theta3_known:.6f}, theta[4]={theta4_known:.6f}, C={pitch_bias_known:.6f}")
        print(f"  → A={A_known:.6f}, B={B_known:.6f}")
    else:
        print(f"\n{'=' * 60}")
        print(f"Step 1+2b: Pitch 重力与摩擦联合辨识")
        print(f"  → theta[3], theta[4], pitch-bias, theta[7], theta[8]")
        print(f"{'=' * 60}")

    # Pitch 使用宽松运动检测 (不要求匀速)，适应 stick-slip 振荡
    # 传入 time 以利用固件暂停期间的 CSV 时间间隙 (>0.5s) 自动切分 sweep
    pitch_segs = find_motion_periods(dq2, vel_threshold=0.005, min_duration_s=0.3, merge_gap_s=0.5, dt=dt,
                                     time=df['time'].values)

    if len(pitch_segs) < 2:
        print(f"错误: 仅找到 {len(pitch_segs)} 个 Pitch 运动段，联合辨识至少需要 2 段。")
        print("请确认每个速度档都完成了正向和反向全行程扫描。")
        sys.exit(1)

    sample_indices = []
    sample_weights = []
    segment_stats = []
    for s, e in pitch_segs:
        idx = np.arange(s, e)
        # Yaw 应基本静止；排除力矩接近饱和的数据。
        keep = (np.abs(dq1[idx]) < args.pitch_yaw_velocity_max) & (np.abs(tau2[idx]) < args.torque_limit)
        idx = idx[keep]
        q2_span = float(np.ptp(q2[idx]))
        if len(idx) < 10 or q2_span < 0.087:  # 至少 5° 行程，排除非 pitch sweep 段
            continue
        sample_indices.append(idx)
        # 每个运动段总权重相同，避免低速段因样本更多而主导回归。
        sample_weights.append(np.full(len(idx), 1.0 / len(idx)))
        segment_stats.append((s, e, float(np.mean(dq2[idx])), float(np.ptp(q2[idx])), len(idx)))

    if len(sample_indices) < 4:
        print(f"错误: 筛选后仅剩 {len(sample_indices)} 个有效 Pitch 匀速段。")
        sys.exit(1)

    idx_all = np.concatenate(sample_indices)
    weights = np.concatenate(sample_weights)

    if use_known_gravity:
        # 两步法: 减去已知重力矩，仅拟合摩擦
        tau2_gravity = A_known * np.cos(q2[idx_all]) + B_known * np.sin(q2[idx_all]) + pitch_bias_known
        y = tau2[idx_all] - tau2_gravity
        X = np.column_stack([
            dq2[idx_all],
            np.tanh(dq2[idx_all] / args.friction_velocity_scale),
        ])

        reg = LinearRegression(fit_intercept=False)
        reg.fit(X, y, sample_weight=weights)
        theta7, theta8 = reg.coef_
        pred = reg.predict(X)
        residual = y - pred
        r2_pitch = reg.score(X, y, sample_weight=weights)
        rmse_pitch = float(np.sqrt(np.average(residual ** 2, weights=weights)))
        mae_pitch = float(np.average(np.abs(residual), weights=weights))

        col_norm = np.linalg.norm(X, axis=0)
        condition = float(np.linalg.cond(X / np.maximum(col_norm, 1e-12)))

        print(f"  使用匀速段数: {len(segment_stats)}, 有效样本: {len(idx_all)} / {len(df)}")
        for i, (s, e, mean_dq, q_span, count) in enumerate(segment_stats):
            print(f"    [{i}] dq2={mean_dq:+.4f} rad/s  q跨度={np.rad2deg(q_span):.1f}°  "
                  f"样本={count}  (行 {s}-{e})")

        print(f"\n  模型 (重力已减除):")
        print(f"    tau2 - (A*cos(q2) + B*sin(q2) + C) = theta[7]*dq2 "
              f"+ theta[8]*tanh(dq2/{args.friction_velocity_scale:g})")
        print(f"    theta[7] (fv2)     = {theta7:.6f}")
        print(f"    theta[8] (fc2)     = {theta8:.6f}")
        print(f"    R²={r2_pitch:.4f}, RMSE={rmse_pitch:.4f} Nm, MAE={mae_pitch:.4f} Nm")
        print(f"    归一化条件数: {condition:.1f}")

        if condition > 30.0:
            print("\n  *** 警告: theta[7]/theta[8] 仍高度耦合，请增加速度档差异。***")

        theta3, theta4 = theta3_known, theta4_known
        pitch_bias = pitch_bias_known
    else:
        # 联合辨识: 同时拟合重力与摩擦
        X = np.column_stack([
            np.cos(q2[idx_all]),
            np.sin(q2[idx_all]),
            np.ones(len(idx_all)),
            dq2[idx_all],
            np.tanh(dq2[idx_all] / args.friction_velocity_scale),
        ])
        y = tau2[idx_all]

        if np.linalg.matrix_rank(X) < X.shape[1]:
            print("错误: 联合回归矩阵秩不足；需要多个速度幅值，并且每档同时包含正、反方向。")
            sys.exit(1)

        reg = LinearRegression(fit_intercept=False)
        reg.fit(X, y, sample_weight=weights)
        A, B, pitch_bias, theta7, theta8 = reg.coef_
        theta3 = -A / g
        theta4 = -B / g
        pred = reg.predict(X)
        residual = y - pred
        r2_pitch = reg.score(X, y, sample_weight=weights)
        rmse_pitch = float(np.sqrt(np.average(residual ** 2, weights=weights)))
        mae_pitch = float(np.average(np.abs(residual), weights=weights))

        col_norm = np.linalg.norm(X, axis=0)
        condition = float(np.linalg.cond(X / np.maximum(col_norm, 1e-12)))
        gravity_condition = float(np.linalg.cond(X[:, :3] / np.maximum(col_norm[:3], 1e-12)))
        friction_condition = float(np.linalg.cond(X[:, 3:] / np.maximum(col_norm[3:], 1e-12)))

        print(f"  使用匀速段数: {len(segment_stats)}, 有效样本: {len(idx_all)} / {len(df)}")
        for i, (s, e, mean_dq, q_span, count) in enumerate(segment_stats):
            print(f"    [{i}] dq2={mean_dq:+.4f} rad/s  q跨度={np.rad2deg(q_span):.1f}°  "
                  f"样本={count}  (行 {s}-{e})")

        print(f"\n  联合模型:")
        print(f"    tau2 = A*cos(q2) + B*sin(q2) + C + theta[7]*dq2 "
              f"+ theta[8]*tanh(dq2/{args.friction_velocity_scale:g})")
        print(f"    A = {A:.6f}, B = {B:.6f}, C = {pitch_bias:.6f}")
        print(f"    theta[3] (m2*l2x) = {theta3:.6f}")
        print(f"    theta[4] (m2*l2z) = {theta4:.6f}")
        print(f"    theta[7] (fv2)     = {theta7:.6f}")
        print(f"    theta[8] (fc2)     = {theta8:.6f}")
        print(f"    R²={r2_pitch:.4f}, RMSE={rmse_pitch:.4f} Nm, MAE={mae_pitch:.4f} Nm")
        print(f"    归一化条件数: full={condition:.1f}, gravity={gravity_condition:.1f}, "
              f"friction={friction_condition:.1f}")

        if gravity_condition > 100.0:
            print("\n  *** 警告: 重力 A/B/C 仍高度耦合，请扩大安全扫角范围或独立标定 C。***")
        if friction_condition > 30.0:
            print("\n  *** 警告: theta[7]/theta[8] 仍高度耦合，请增加速度档差异。***")

    if abs(pitch_bias) > 0.1 * max(abs(A_known if use_known_gravity else A),
                                    abs(B_known if use_known_gravity else B), 0.01):
        print(f"\n  注意: C={pitch_bias:.4f} Nm 较大；它是执行器/线缆等效偏置，不属于理想重力项。")

    print(f"\n  下一步 (pitch-inertia) 请使用:")
    print(f"    --theta34 {theta3:.6f},{theta4:.6f} --theta78 {theta7:.6f},{theta8:.6f} \\")
    print(f"    --pitch-bias {pitch_bias:.6f}")

    # ── 配对差分发: 利用正反向 sweep 消除重力，纯摩擦拟合 ──
    print(f"\n{'─' * 60}")
    print(f"配对差分发验证: 正反向 sweep 均值差分消除重力")
    paired_dq, paired_tanh, paired_dtau = [], [], []
    for i in range(0, len(segment_stats) - 1, 2):
        s_fwd = segment_stats[i]
        s_bwd = segment_stats[i + 1]
        # 只配对方向相反的 sweep
        if s_fwd[2] * s_bwd[2] >= 0:
            continue
        idx_fwd = sample_indices[i]
        idx_bwd = sample_indices[i + 1]
        mean_tau_fwd = float(np.mean(tau2[idx_fwd]))
        mean_tau_bwd = float(np.mean(tau2[idx_bwd]))
        dtau_val = mean_tau_fwd - mean_tau_bwd

        mean_dq_fwd = s_fwd[2]
        mean_dq_bwd = s_bwd[2]
        mean_tanh_fwd = float(np.mean(np.tanh(dq2[idx_fwd] / args.friction_velocity_scale)))
        mean_tanh_bwd = float(np.mean(np.tanh(dq2[idx_bwd] / args.friction_velocity_scale)))

        paired_dq.append(mean_dq_fwd - mean_dq_bwd)
        paired_tanh.append(mean_tanh_fwd - mean_tanh_bwd)
        paired_dtau.append(dtau_val)

    if len(paired_dtau) >= 2:
        Xp = np.column_stack([paired_dq, paired_tanh])
        yp = np.array(paired_dtau)
        reg_p = LinearRegression(fit_intercept=False)
        reg_p.fit(Xp, yp)
        fv2_paired, fc2_paired = reg_p.coef_
        r2_paired = reg_p.score(Xp, yp)

        # 纯库仑模型: Δtau ≈ fc2 * Δtanh (假设 fv2 ≈ 0)
        fc2_per_pair = [paired_dtau[i] / paired_tanh[i] for i in range(len(paired_dtau))]
        fc2_coulomb = float(np.mean(fc2_per_pair))
        r2_coulomb = float(1 - np.sum((yp - fc2_coulomb * np.array(paired_tanh))**2)
                           / np.sum((yp - np.mean(yp))**2))

        print(f"  速度对数量: {len(paired_dtau)}")
        for i in range(len(paired_dtau)):
            print(f"    [{i}] Δdq={paired_dq[i]:.4f}  Δtanh={paired_tanh[i]:.4f}  "
                  f"Δtau={paired_dtau[i]:.4f}  fc2_per_pair={fc2_per_pair[i]:.4f}")
        print(f"  联合拟合: Δtau = fv2*Δdq + fc2*Δtanh")
        print(f"    fv2 = {fv2_paired:.6f}, fc2 = {fc2_paired:.6f}, R² = {r2_paired:.4f}")
        print(f"  纯库仑拟合 (fv2≡0):")
        print(f"    fc2 = {fc2_coulomb:.6f} (mean of per-pair estimates)")
        print(f"    R² = {r2_coulomb:.4f}")
        if fv2_paired < 0:
            print(f"  注意: 联合 fv2 为负，且 Δtau 基本不随速度变化 → 建议 fv2≈0, fc2≈{fc2_coulomb:.4f}")

    return theta3, theta4, pitch_bias, theta5, theta6, theta7, theta8


# ---------------------------------------------------------------------------
# Step 3: Pitch 惯量辨识
# ---------------------------------------------------------------------------

def cmd_pitch_inertia_multifrequency(args):
    """Identify pitch inertia from extended, real-time, multi-frequency CSV data."""
    required = {
        'dt', 'q2_ref', 'dq2_ref', 'ddq2_ref', 'frequency_hz',
        'frequency_index', 'cycle_index', 'valid_for_fit',
    }
    df = load_csv(args.csv)
    if not required.issubset(df.columns) or df['frequency_hz'].fillna(0).max() <= 0:
        print("\n警告: 检测到旧版 CSV，回退到原始速度差分算法；该结果仅供诊断。")
        return cmd_pitch_inertia(args)

    if args.friction_velocity_scale <= 0.0:
        raise ValueError("--friction-velocity-scale 必须大于 0")
    theta3, theta4 = parse_theta(args.theta34, 2, 'theta34')
    theta7, theta8 = parse_theta(args.theta78, 2, 'theta78')
    pitch_bias = args.pitch_bias
    g = args.g

    valid = df['valid_for_fit'].fillna(0).astype(int).to_numpy() > 0
    valid &= np.isfinite(df['time'].to_numpy()) & np.isfinite(df['q2'].to_numpy())
    valid &= np.abs(df['dq1'].to_numpy()) < 0.15
    valid &= np.abs(df['tau2'].to_numpy()) < 9.5

    q = df['q2'].to_numpy(dtype=float)
    t = df['time'].to_numpy(dtype=float)
    dq_motor = df['dq2'].to_numpy(dtype=float)
    tau = df['tau2'].to_numpy(dtype=float)
    freq_index = df['frequency_index'].fillna(-1).astype(int).to_numpy()
    frequency = df['frequency_hz'].fillna(0).to_numpy(dtype=float)
    dq_hat = np.full(len(df), np.nan)
    ddq_hat = np.full(len(df), np.nan)

    block_diagnostics = []
    active_indices = sorted(set(freq_index[valid]))
    for index in active_indices:
        mask = valid & (freq_index == index)
        if np.sum(mask) < 30:
            continue
        f = float(np.median(frequency[mask]))
        w = 2.0 * np.pi * f
        tb = t[mask]
        basis = np.column_stack([np.sin(w * tb), np.cos(w * tb), np.ones(np.sum(mask))])
        coef, *_ = np.linalg.lstsq(basis, q[mask], rcond=None)
        q_fit = basis @ coef
        dq_fit = w * (coef[0] * np.cos(w * tb) - coef[1] * np.sin(w * tb))
        ddq_fit = -(w ** 2) * (coef[0] * np.sin(w * tb) + coef[1] * np.cos(w * tb))
        dq_hat[mask] = dq_fit
        ddq_hat[mask] = ddq_fit
        ss_res = float(np.sum((q[mask] - q_fit) ** 2))
        ss_tot = float(np.sum((q[mask] - np.mean(q[mask])) ** 2))
        q_r2 = 1.0 - ss_res / max(ss_tot, 1e-12)
        dq_scale = float(np.dot(dq_motor[mask], dq_fit) /
                         max(np.dot(dq_motor[mask], dq_motor[mask]), 1e-12))
        block_diagnostics.append((index, f, np.sum(mask), np.hypot(coef[0], coef[1]), q_r2, dq_scale))

    fit_mask = valid & np.isfinite(dq_hat) & np.isfinite(ddq_hat) & (np.abs(ddq_hat) > 0.2)
    if np.sum(fit_mask) < 100 or len(block_diagnostics) < 2:
        print("错误: 有效多频数据不足；至少需要两个完整频率段和 100 个有效样本。")
        return None

    qf = q[fit_mask]
    vf = dq_hat[fit_mask]
    af = ddq_hat[fit_mask]
    yf = tau[fit_mask]
    group = freq_index[fit_mask]
    vs = args.friction_velocity_scale

    # Subtract known gravity and friction (from Step 2), fit only inertia + residual bias.
    # tau_grav = -g*theta3*cos(q) - g*theta4*sin(q)
    tau_grav = -g * theta3 * np.cos(qf) - g * theta4 * np.sin(qf)
    tau_fric = theta7 * vf + theta8 * np.tanh(vf / vs)
    yf_corrected = yf - tau_grav - tau_fric - pitch_bias

    # Weights: equal total weight per frequency, then Huber IRLS.
    weights = np.zeros(len(yf_corrected))
    for index in np.unique(group):
        n = np.sum(group == index)
        weights[group == index] = 1.0 / max(n, 1)
    robust = np.ones(len(yf_corrected))
    X_inertia = np.column_stack([af, np.ones(len(af))])
    lower = np.array([0.0, -np.inf])  # I >= 0
    upper = np.array([np.inf, np.inf])
    beta = np.zeros(2)
    for _ in range(6):
        total_weight = weights * robust
        sw = np.sqrt(total_weight)
        beta = lsq_linear(X_inertia * sw[:, None], yf_corrected * sw, bounds=(lower, upper)).x
        residual = yf_corrected - X_inertia @ beta
        scale = 1.4826 * np.median(np.abs(residual - np.median(residual))) + 1e-9
        robust = np.minimum(1.0, 1.5 * scale / np.maximum(np.abs(residual), 1e-12))

    inertia, residual_bias = beta
    A = -g * theta3  # from Step 2
    B = -g * theta4  # from Step 2
    bias = pitch_bias + residual_bias
    fv, fc = theta7, theta8
    full_pred = tau_grav + tau_fric + pitch_bias + X_inertia @ beta
    final_weight = weights * robust
    y_mean = np.average(yf, weights=final_weight)
    r2 = 1.0 - np.sum(final_weight * (yf - full_pred) ** 2) / max(
        np.sum(final_weight * (yf - y_mean) ** 2), 1e-12)
    rmse = float(np.sqrt(np.average((yf - full_pred) ** 2, weights=final_weight)))
    condition = 0.0
    inertia_condition = 0.0

    def inertia_diagnostic(mask):
        # Subtract non-inertia components using known gravity/friction
        residual_i = yf[mask] - (
            A * np.cos(qf[mask]) + B * np.sin(qf[mask]) + pitch_bias +
            fv * vf[mask] + fc * np.tanh(vf[mask] / vs)
        )
        design = np.column_stack([af[mask], np.ones(np.sum(mask))])
        coef, *_ = np.linalg.lstsq(design, residual_i, rcond=None)
        return float(coef[0]), float(coef[1])

    i_pos, c_pos = inertia_diagnostic(af > 0.2)
    i_neg, c_neg = inertia_diagnostic(af < -0.2)
    asymmetry = abs(i_pos - i_neg) / max(abs(inertia), 1e-6)
    per_frequency = []
    for index in np.unique(group):
        i_f, c_f = inertia_diagnostic(group == index)
        per_frequency.append((int(index), float(np.median(frequency[fit_mask][group == index])), i_f, c_f))
    inertia_values = np.array([item[2] for item in per_frequency])
    frequency_variation = float(np.std(inertia_values) / max(abs(np.mean(inertia_values)), 1e-6))

    print(f"\n{'=' * 60}")
    print("Step 3: Pitch 惯量辨识 (基于已知重力/摩擦)")
    print("  tau2 = A*cos(q2)+B*sin(q2)+C+I*ddq2+fv*dq2+fc*tanh(dq2/vs)")
    print("  其中 A,B,fv,fc,C 使用 Step 2 结果，仅拟合 I + residual")
    print(f"{'=' * 60}")
    if 'dt' in df:
        dt_values = df.loc[df['dt'].notna(), 'dt'].to_numpy(dtype=float)
        if len(dt_values):
            print(f"  实测控制周期: mean={np.mean(dt_values)*1e3:.3f} ms, "
                  f"std={np.std(dt_values)*1e3:.3f} ms, "
                  f"min={np.min(dt_values)*1e3:.3f} ms, max={np.max(dt_values)*1e3:.3f} ms")
    print("\n  各频率位置谐波拟合:")
    for index, f, count, amplitude, q_r2, dq_scale in block_diagnostics:
        print(f"    [{index}] f={f:.3f} Hz  n={count}  A={amplitude:.4f} rad  "
              f"q-R²={q_r2:.4f}  dq位置/电机比例={dq_scale:.3f}")

    print(f"\n  有效样本: {np.sum(fit_mask)} / {len(df)}")
    print(f"    theta[2] I2yy        = {inertia:.6f} kg*m²  (拟合)")
    print(f"    残差偏置              = {residual_bias:.6f} Nm")
    print(f"\n  (来自 Step 2 的已知参数):")
    print(f"    theta[3] m2*l2x      = {theta3:.6f}")
    print(f"    theta[4] m2*l2z      = {theta4:.6f}")
    print(f"    pitch-bias C         = {pitch_bias:.6f} Nm")
    print(f"    theta[7] fv2         = {fv:.6f}")
    print(f"    theta[8] fc2         = {fc:.6f}")
    print(f"\n  R²={r2:.4f}, RMSE={rmse:.4f} Nm")
    print("\n  分频率惯量检查:")
    for index, f, i_f, c_f in per_frequency:
        print(f"    [{index}] f={f:.3f} Hz: I={i_f:.6f}, residual bias={c_f:.6f} Nm")
    print(f"    分频率相对标准差={frequency_variation:.1%}")
    print(f"\n  正负加速度检查:")
    print(f"    ddq>0: I={i_pos:.6f}, residual bias={c_pos:.6f} Nm")
    print(f"    ddq<0: I={i_neg:.6f}, residual bias={c_neg:.6f} Nm")
    print(f"    相对不对称度={asymmetry:.1%}")

    passed = (r2 > 0.3 and inertia > 0 and asymmetry < 0.20 and
              frequency_variation < 0.20)
    if passed:
        print("\n  结果通过基础质量检查，可继续结合机械量级复核。")
    else:
        print("\n  *** 结果未通过质量检查，暂不要写入 theta[2] 或继续 coupling。***")
    return inertia




def cmd_coupling(args):
    """Step 4: Yaw-Pitch coupling inertia identification (two-step method)."""
    if args.friction_velocity_scale <= 0.0:
        raise ValueError("--friction-velocity-scale must be > 0")
    theta5, theta6 = parse_theta(args.theta56, 2, 'theta56')
    vs = args.friction_velocity_scale

    dfs = [load_csv(p) for p in args.csv]
    for i, (df, p) in enumerate(zip(dfs, args.csv)):
        df['_file'] = i
        dfs[i] = df
    df_all = pd.concat(dfs, ignore_index=True)

    # Require extended CSV format
    extended_cols = {'valid_for_fit', 'frequency_index'}
    if not extended_cols.issubset(df_all.columns):
        print("Error: CSV missing columns 'valid_for_fit' and/or 'frequency_index'.")
        print('Please use firmware with gimbal_ident kCoupling mode (extended CSV format).')
        sys.exit(1)
    if df_all['valid_for_fit'].sum() == 0:
        print('Error: No valid_for_fit rows in CSV. Check firmware data collection.')
        sys.exit(1)

    return _cmd_coupling_twostep(df_all, theta5, theta6, vs)


def _cmd_coupling_twostep(df_all, theta5, theta6, vs):
    """Two-step coupling identification:
    Step A: Per-angle harmonic fit + single-parameter I_eff regression.
    Step B: Cross-angle fit I_eff = I2xx + (I1zz - I2xx) * sin^2(q2).
    """
    valid_mask = df_all['valid_for_fit'].fillna(0).astype(int).to_numpy() > 0
    valid_mask &= np.isfinite(df_all['time'].to_numpy()) & np.isfinite(df_all['q1'].to_numpy())
    valid_mask &= np.abs(df_all['tau1'].to_numpy()) < 9.5

    q1 = df_all['q1'].to_numpy(dtype=float)
    q2 = df_all['q2'].to_numpy(dtype=float)
    tau1 = df_all['tau1'].to_numpy(dtype=float)
    t = df_all['time'].to_numpy(dtype=float)
    freq_index = df_all['frequency_index'].fillna(-1).astype(int).to_numpy()
    frequency = df_all['frequency_hz'].fillna(0.2).to_numpy(dtype=float)

    # Check for firmware-provided ddq1_ref
    has_ddq1_ref = 'ddq1_ref' in df_all.columns and np.any(np.abs(df_all['ddq1_ref'].to_numpy(dtype=float)) > 1e-6)
    ddq1_ref = df_all['ddq1_ref'].to_numpy(dtype=float) if 'ddq1_ref' in df_all.columns else np.zeros(len(df_all))

    active_indices = sorted(set(freq_index[valid_mask]))
    if len(active_indices) < 3:
        print(f'Error: Only {len(active_indices)} angle blocks found; need at least 3.')
        sys.exit(1)

    print(f'\n{"=" * 60}')
    print('Step 4: Yaw-Pitch 耦合惯量辨识 (两步法)')
    print(f'{"=" * 60}')

    # ---- Step A: Per-angle harmonic fitting + I_eff regression ----
    block_results = []

    print(f'\n  --- 步骤 A: 每角度谐波拟合 & 有效惯量回归 ---')
    if has_ddq1_ref:
        print('  使用固件 ddq1_ref 进行回归.')
    else:
        print('  ddq1_ref 不可用; 使用谐波拟合求导得到的 ddq1.')

    for index in active_indices:
        mask = valid_mask & (freq_index == index)
        n = np.sum(mask)
        if n < 30:
            continue

        f = float(np.median(frequency[mask]))
        if f <= 0:
            f = 0.2
        w = 2.0 * np.pi * f

        # Unwrap q1 for large-amplitude excitation (handles +/-pi wraps)
        q1_block = q1[mask]
        q1_unwrapped = np.unwrap(q1_block)
        tb = t[mask] - t[mask][0]

        # Harmonic fit: q1 = A*sin(wt) + B*cos(wt) + C
        basis = np.column_stack([np.sin(w * tb), np.cos(w * tb), np.ones(n)])
        coef, *_ = np.linalg.lstsq(basis, q1_unwrapped, rcond=None)
        q1_fit = basis @ coef
        amplitude = np.hypot(coef[0], coef[1])
        ss_res = float(np.sum((q1_unwrapped - q1_fit) ** 2))
        ss_tot = float(np.sum((q1_unwrapped - np.mean(q1_unwrapped)) ** 2))
        q_r2 = 1.0 - ss_res / max(ss_tot, 1e-12)

        # Analytic derivatives from harmonic fit
        dq1_fit = w * (coef[0] * np.cos(w * tb) - coef[1] * np.sin(w * tb))
        ddq1_fit = -(w ** 2) * (coef[0] * np.sin(w * tb) + coef[1] * np.cos(w * tb))

        # Use firmware ddq1_ref if available, otherwise harmonic-fit ddq1
        ddq1_use = ddq1_ref[mask] if has_ddq1_ref else ddq1_fit
        dq1_use = dq1_fit  # always use harmonic-fit velocity (smoother)

        # Subtract yaw friction
        tau1_residual = tau1[mask] - theta5 * dq1_use - theta6 * np.tanh(dq1_use / vs)

        # Single-parameter regression: tau1_residual = I_eff * ddq1
        fit_sel = np.abs(ddq1_use) > 0.05
        if np.sum(fit_sel) < 20:
            continue

        Xa = ddq1_use[fit_sel].reshape(-1, 1)
        ya = tau1_residual[fit_sel]
        reg_a = LinearRegression(fit_intercept=False)
        reg_a.fit(Xa, ya)
        i_eff = float(reg_a.coef_[0])
        r2_a = reg_a.score(Xa, ya)

        q2_mean = float(np.mean(q2[mask]))

        # Standard error estimate
        residual_std = float(np.std(ya - reg_a.predict(Xa)))
        ddq1_rms = float(np.sqrt(np.mean(ddq1_use[fit_sel] ** 2)))
        i_eff_se = residual_std / (ddq1_rms * np.sqrt(np.sum(fit_sel))) if ddq1_rms > 1e-9 else np.inf

        block_results.append({
            'index': index, 'q2_mean': q2_mean,
            'i_eff': i_eff, 'r2_a': r2_a, 'n': n, 'f': f,
            'amplitude': amplitude, 'q_r2': q_r2,
            'i_eff_se': i_eff_se,
        })

    if len(block_results) < 3:
        print('错误: 有效角度块少于 3 个, 无法继续.')
        sys.exit(1)

    # Print Step A table
    print(f'\n  {"序号":>5s}  {"q2[度]":>8s}  {"I_eff":>12s}  {"R²(A)":>8s}  {"q-R²":>8s}  {"振幅[rad]":>8s}  {"样本数":>6s}')
    print(f'  {"-"*5}  {"-"*8}  {"-"*12}  {"-"*8}  {"-"*8}  {"-"*8}  {"-"*6}')
    for r in block_results:
        print(f'  {r["index"]:5d}  {np.rad2deg(r["q2_mean"]):8.1f}  {r["i_eff"]:12.6f}  '
              f'{r["r2_a"]:8.4f}  {r["q_r2"]:8.4f}  {r["amplitude"]:8.3f}  {r["n"]:6d}')

    # ---- Step B: Cross-angle I_eff vs sin^2(q2) ----
    print(f'\n  --- 步骤 B: 跨角度拟合 I_eff = I2xx + (I1zz - I2xx) * sin²(q2) ---')

    q2_means = np.array([r['q2_mean'] for r in block_results])
    i_effs = np.array([r['i_eff'] for r in block_results])
    i_eff_ses = np.array([r['i_eff_se'] for r in block_results])

    sin2_q2 = np.sin(q2_means) ** 2

    # Weighted regression
    weights = 1.0 / (i_eff_ses ** 2 + np.median(i_eff_ses) ** 2)
    weights = np.clip(weights, 0.0, np.percentile(weights, 90))

    Xb = np.column_stack([np.ones(len(block_results)), sin2_q2])
    reg_b = LinearRegression(fit_intercept=False)
    reg_b.fit(Xb, i_effs, sample_weight=weights)
    i2xx = float(reg_b.coef_[0])
    i1zz_minus_i2xx = float(reg_b.coef_[1])
    i1zz = i2xx + i1zz_minus_i2xx
    r2_b = reg_b.score(Xb, i_effs, sample_weight=weights)

    i_eff_pred = Xb @ reg_b.coef_

    q2_range = float(np.max(q2_means) - np.min(q2_means))

    print(f'\n    模型: I_eff = I2xx + (I1zz - I2xx) * sin²(q2)')
    print(f'    I2xx (截距)          = {i2xx:.6f}  kg·m²  -> theta[1]')
    print(f'    I1zz - I2xx (斜率)   = {i1zz_minus_i2xx:.6f}  kg·m²')
    print(f'    I1zz                 = {i1zz:.6f}  kg·m²  -> theta[0]')
    print(f'    步骤 B R²            = {r2_b:.4f}')
    print(f'    sin²(q2) 范围        = [{np.min(sin2_q2):.4f}, {np.max(sin2_q2):.4f}]')
    print(f'    q2 范围              = {np.rad2deg(q2_range):.1f} 度')

    print(f'\n    各角度与步骤 B 拟合值的残差:')
    print(f'    {"q2[度]":>8s}  {"I_eff":>12s}  {"预测值":>12s}  {"残差":>10s}  {"残差%":>8s}')
    print(f'    {"-"*8}  {"-"*12}  {"-"*12}  {"-"*10}  {"-"*8}')
    for i, r in enumerate(block_results):
        resid = r['i_eff'] - i_eff_pred[i]
        res_pct = 100.0 * resid / max(abs(r['i_eff']), 1e-12)
        print(f'    {np.rad2deg(r["q2_mean"]):8.1f}  {r["i_eff"]:12.6f}  '
              f'{i_eff_pred[i]:12.6f}  {resid:10.6f}  {res_pct:7.1f}%')

    # ---- Quality checks ----
    print()
    warnings = []
    for r in block_results:
        if r['q_r2'] < 0.95:
            warnings.append(f'角度 {r["index"]}: q-R²={r["q_r2"]:.3f} < 0.95, yaw 跟踪可能不良.')
        if r['r2_a'] < 0.5:
            warnings.append(f'角度 {r["index"]}: I_eff R²={r["r2_a"]:.3f} < 0.5, 信噪比不足.')
    if q2_range < 0.25:
        warnings.append(f'q2 范围仅 {np.rad2deg(q2_range):.1f} 度; theta[0] 可能不可靠.')
    if r2_b < 0.5:
        warnings.append(f'步骤 B R²={r2_b:.3f} < 0.5; I1zz/I2xx 的分离可能不可靠.')

    if warnings:
        print('  *** 警告:')
        for w in warnings:
            print(f'      - {w}')
    else:
        print('  所有质量检查通过.')

    print(f'\n  下一步 (verify):')
    print(f'    python identify_gimbal.py verify <data.csv> \\')
    print(f'        --theta {i1zz:.6f},{i2xx:.6f},<theta2>,<theta3>,<theta4>,{theta5:.6f},{theta6:.6f},<theta7>,<theta8>')

    return i1zz, i2xx


def cmd_verify(args):
    if args.friction_velocity_scale <= 0.0:
        raise ValueError("--friction-velocity-scale 必须大于 0")
    theta = parse_theta(args.theta, 9, 'theta')

    df = load_csv(args.csv)
    df, dt = add_ddq(df, window=args.window)

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
        theta[6] * np.tanh(dq1 / args.friction_velocity_scale)
    )

    # Pitch (与 C++ gimbal_dynamics.hpp:102-108 完全一致)
    tau2_pred = (
        theta[0] * (-0.5 * sin_2q2 * dq1_sq) +
        theta[1] * (0.5 * sin_2q2 * dq1_sq) +
        theta[2] * ddq2 +
        theta[3] * (gx_cos_gy_sin_q1 * sin_q2 + gz * cos_q2) +
        theta[4] * (-gx_cos_gy_sin_q1 * cos_q2 + gz * sin_q2) +
        theta[7] * dq2 +
        theta[8] * np.tanh(dq2 / args.friction_velocity_scale)
    ) + args.pitch_bias

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

    print(f"\n  数据: {len(df)} 样本, dt={dt:.4f}s, SG window={args.window} ({args.window*dt:.3f}s)")
    print(f"  ddq1 std={float(np.std(ddq1)):.4f}  ddq2 std={float(np.std(ddq2)):.4f}")

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
    print(f"    pitch bias C         = {args.pitch_bias:.7f} Nm")

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

  # Step 2: Pitch 重力/摩擦联合辨识 + Yaw 摩擦辨识
  python identify_gimbal.py friction friction_sweep.csv

  # Step 3: Pitch 惯量 (需要 θ3,θ4 和 θ7,θ8)
  python identify_gimbal.py pitch-inertia dynamic_pitch.csv --theta34 0.027,-0.026 --theta78 0.3,0.04

  # Step 4: 耦合惯量 - 两步法 (每角度 I_eff -> 跨角度 I1zz/I2xx)
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
    p = sub.add_parser('friction', help='Pitch 重力/摩擦联合辨识 + Yaw 摩擦辨识')
    p.add_argument('csv', help='Pitch 多速度正反扫角 + Yaw 匀速数据 CSV')
    p.add_argument('--g', type=float, default=G_DEFAULT)
    p.add_argument('--dq-stability', type=float, default=0.03, help='Yaw 匀速判定的 dq 标准差阈值 (rad/s)')
    p.add_argument('--min-duration', type=float, default=0.5, help='Yaw 匀速段最短持续时间 (s)')
    p.add_argument('--pitch-dq-stability', type=float, default=0.06, help='Pitch 匀速判定的 dq 标准差阈值 (rad/s)')
    p.add_argument('--pitch-min-duration', type=float, default=0.3, help='Pitch 匀速段最短持续时间 (s)')
    p.add_argument('--pitch-yaw-velocity-max', type=float, default=0.10,
                   help='Pitch 联合辨识时允许的最大 |dq1| (rad/s)')
    p.add_argument('--torque-limit', type=float, default=9.5,
                   help='排除 |tau2| 超过此值的饱和样本 (Nm)')
    p.add_argument('--friction-velocity-scale', type=float, default=FRICTION_VELOCITY_SCALE_DEFAULT,
                   help='tanh 摩擦过渡速度 (rad/s)，必须与 C++ 动力学模型一致')
    p.add_argument('--theta34', help='已知的 theta[3],theta[4]（gravity 步骤结果），提供后仅拟合摩擦项')
    p.add_argument('--pitch-bias', type=float, default=None,
                   help='已知的 Pitch 常值偏置 C (Nm)，与 --theta34 配合使用')

    # --- pitch-inertia ---
    p = sub.add_parser('pitch-inertia', help='Step 3: Pitch 惯量辨识 (theta[2])')
    p.add_argument('csv', help='Yaw 固定、Pitch 低频加减速数据 CSV')
    p.add_argument('--theta34', required=True, help='theta[3],theta[4] (来自 gravity 步骤)')
    p.add_argument('--theta78', required=True, help='theta[7],theta[8] (来自 friction 步骤的 Pitch 摩擦)')
    p.add_argument('--pitch-bias', type=float, default=0.0, help='联合辨识得到的 Pitch 常值力矩偏置 C (Nm)')
    p.add_argument('--friction-velocity-scale', type=float, default=FRICTION_VELOCITY_SCALE_DEFAULT,
                   help='tanh 摩擦过渡速度 (rad/s)')
    p.add_argument('--g', type=float, default=G_DEFAULT)

    # --- coupling ---
    p = sub.add_parser('coupling', help='Step 4: Yaw-Pitch coupling inertia (two-step: per-angle I_eff, then cross-angle fit)')
    p.add_argument('csv', nargs='+', help='多个 q2 姿态下的 Yaw 加减速数据 CSV (可指定多个文件)')
    p.add_argument('--theta56', required=True, help='theta[5],theta[6] (来自 friction 步骤的 Yaw 摩擦)')
    p.add_argument('--friction-velocity-scale', type=float, default=FRICTION_VELOCITY_SCALE_DEFAULT,
                   help='tanh 摩擦过渡速度 (rad/s)')

    # --- verify ---
    p = sub.add_parser('verify', help='Step 5: 综合验证全部 9 参数')
    p.add_argument('csv', help='五次谐波综合轨迹数据 CSV')
    p.add_argument('--theta', required=True, help='全部 9 参数: theta0,theta1,...,theta8')
    p.add_argument('--pitch-bias', type=float, default=0.0, help='Pitch 常值力矩偏置 C (Nm)')
    p.add_argument('--friction-velocity-scale', type=float, default=FRICTION_VELOCITY_SCALE_DEFAULT,
                   help='tanh 摩擦过渡速度 (rad/s)')
    p.add_argument('--g', type=float, default=G_DEFAULT)
    p.add_argument('--window', type=int, default=21,
                   help='SG 滤波器窗口大小 (奇数, 默认 21 = ~0.42s@50Hz). 高频数据减小窗口以保留信号.')

    args = parser.parse_args()

    if args.cmd == 'gravity':
        cmd_gravity(args)
    elif args.cmd == 'friction':
        cmd_friction(args)
    elif args.cmd == 'pitch-inertia':
        cmd_pitch_inertia_multifrequency(args)
    elif args.cmd == 'coupling':
        cmd_coupling(args)
    elif args.cmd == 'verify':
        cmd_verify(args)


if __name__ == '__main__':
    main()
