#!/usr/bin/env python3
"""
Generate wheel-legged LQR kCtrlP coefficients from leg-length scheduled Q/R.

Workflow:
  1. Fill HERO_QR_POINTS / INFANTRY3_QR_POINTS / INFANTRY4_QR_POINTS with
     measured/tuned equal-leg-length Q/R matrices.
  2. The script interpolates Q/R by mean leg length.
  3. For a 2-D (left length, right length) grid, it computes A/B and LQR K.
  4. It fits each K element to:
       Kij = p00 + p10*l_l + p01*l_r + p20*l_l^2 + p11*l_l*l_r + p02*l_r^2
  5. It prints a C++ constexpr array compatible with kCtrlPLow.

This mirrors HerKules_VOCAL_SJ_LQR_v4_with_data.m, but keeps the online
controller's current 2-D kCtrlP structure while letting you tune Q/R on the
mostly equal-leg-length real robot.

---------------------------------------------------------------------
Usage examples (run from this script's directory):

  # 1) Dry run: print on-screen C++ header & fit statistics (default params)
  python generate_scheduled_kctrlp.py

  # 2) Generate the C++ header file (overwrites kctrlp_generated.hpp)
  python generate_scheduled_kctrlp.py --output kctrlp_generated.hpp

  # 3) Custom leg-length range & finer grid step (header only)
  python generate_scheduled_kctrlp.py \\
      --leg-min 0.14 --leg-max 0.34 --grid-step 0.005 \\
      --output kctrlp_generated.hpp

  # 4) Generate HTML fit-visualisation reports for all three robot variants
  python generate_scheduled_kctrlp.py \\
      --html-report kctrlp_fit_reports/ --report-variant all

  # 5) Generate only the infantry3 HTML report (single file)
  python generate_scheduled_kctrlp.py \\
      --html-report kctrlp_fit_report_infantry3.html --report-variant infantry3

  # 6) ★ Final one-shot: C++ header + all HTML reports, full leg range, fine grid
  python generate_scheduled_kctrlp.py \\
      --leg-min 0.14 --leg-max 0.35 --grid-step 0.005 \\
      --output kctrlp_generated.hpp \\
      --html-report kctrlp_fit_reports/ --report-variant all

      python generate_scheduled_kctrlp.py --leg-min 0.14 --leg-max 0.35 --grid-step 0.005 --output kctrlp_generated.hpp --html-report kctrlp_fit_reports/ --report-variant all

Arguments summary:
  --leg-min FLOAT       Min leg length for 2-D fit grid        [default: 0.15]
  --leg-max FLOAT       Max leg length for 2-D fit grid        [default: 0.35]
  --grid-step FLOAT     Leg length grid step                   [default: 0.01]
  --output PATH         Write C++ constexpr header to PATH
  --html-report PATH    Write HTML/SVG fit report(s):
                        - a directory  → one .html per variant inside it
                        - a .html file → single variant report (see next arg)
  --report-variant STR  Variant(s) for --html-report:
                        hero | infantry3 | infantry4 | all     [default: infantry3]
---------------------------------------------------------------------
"""

from __future__ import annotations

import argparse
import html
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np

try:
    from scipy.linalg import solve_continuous_are
except ImportError:  # pragma: no cover - user environment dependent
    solve_continuous_are = None


# ---------------------------------------------------------------------------
# 1) EDIT THIS TABLE
# ---------------------------------------------------------------------------
#
# Put your measured/tuned equal-leg Q/R points here. You may use np.diag(...)
# for diagonal matrices, or paste a full 10x10 Q / 4x4 R np.array.
#
# Keep left/right symmetric terms equal unless you intentionally want otherwise:
#   Q[4,4] == Q[6,6], Q[5,5] == Q[7,7], R[0,0] == R[1,1], R[2,2] == R[3,3]
#
# The default values below are placeholders based on the current MATLAB file's
# low/mid/high examples. Replace them with your real tuned values.


@dataclass(frozen=True)
class QrPoint:
    leg_length_m: float
    Q: np.ndarray
    R: np.ndarray


HERO_QR_POINTS: list[QrPoint] = [
    # QrPoint(
    #     0.14,
    #     np.diag([200.0, 120.0, 210.0, 1.0, 1400.0, 1.0, 1400.0, 1.0, 3200.0, 1.0]),
    #     np.diag([2.4, 2.4, 0.32, 0.32]),
    # ),
    # QrPoint(
    #     0.18,
    #     np.diag([200.0, 120.0, 210.0, 1.0, 1400.0, 1.0, 1400.0, 1.0, 3200.0, 1.0]),
    #     np.diag([2.4, 2.4, 0.32, 0.32]),
    # ),
    QrPoint(
        0.14,
        np.diag([200.0, 110.0, 210.0, 1.0, 1450.0, 1.0, 1450.0, 1.0, 3200.0, 1.0]),
        np.diag([2.6, 2.6, 0.35, 0.35]),
    ),
    QrPoint(
        0.155,
        np.diag([200.0, 100.0, 210.0, 1.0, 1450.0, 1.0, 1450.0, 1.0, 3200.0, 1.0]),
        np.diag([2.8, 2.8, 0.4, 0.4]),
    ),
    # QrPoint(
    #     0.185,
    #     np.diag([200.0, 100.0, 200.0, 1.0, 1300.0, 1.0, 1300.0, 1.0, 3400.0, 1.0]),
    #     np.diag([3.6, 3.6, 0.5, 0.5]),
    # ),
    QrPoint(
        0.195,
        np.diag([200.0, 100.0, 210.0, 1.0, 1450.0, 1.0, 1450.0, 1.0, 3800.0, 1.0]),
        np.diag([3.6, 3.6, 0.45, 0.45]),
    ),
    QrPoint(
        0.245,
        np.diag([160.0, 90.0, 210.0, 1.0, 1450.0, 1.0, 1450.0, 1.0, 3800.0, 1.0]),
        np.diag([3.8, 3.8, 0.45, 0.45]),
    ),
    QrPoint(
        0.33,
        np.diag([150.0, 90.0, 210.0, 1.0, 1500.0, 1.0, 1500.0, 1.0, 4000.0, 1.0]),
        np.diag([3.8, 3.8, 0.45, 0.45]),
    ),
]

INFANTRY3_QR_POINTS: list[QrPoint] = [
    QrPoint(
        0.14,
        np.diag([200.0, 140.0, 250.0, 1.0, 1500.0, 1.0, 1500.0, 1.0, 4000.0, 1.0]),
        np.diag([2.1, 2.1, 0.4, 0.4]),
    ),
    QrPoint(
        0.155,
        np.diag([200.0, 120.0, 250.0, 1.0, 1500.0, 1.0, 1500.0, 1.0, 4000.0, 1.0]),
        np.diag([2.1, 2.1, 0.4, 0.4]),
    ),
    # QrPoint(
    #     0.185,
    #     np.diag([200.0, 100.0, 200.0, 1.0, 1300.0, 1.0, 1300.0, 1.0, 3400.0, 1.0]),
    #     np.diag([3.6, 3.6, 0.5, 0.5]),
    # ),
    QrPoint(
        0.195,
        np.diag([200.0, 120.0, 250.0, 1.0, 1500.0, 1.0, 1500.0, 1.0, 4200.0, 1.0]),
        np.diag([2.2, 2.2, 0.5, 0.5]),
   ),
    QrPoint(
        0.245,
        np.diag([160.0, 100.0, 250.0, 1.0, 1550.0, 1.0, 1550.0, 1.0, 4200.0, 1.0]),
        np.diag([2.2, 2.2, 0.53, 0.53]),
    ),
    QrPoint(
        0.33,
        np.diag([150.0, 50.0, 250.0, 1.0, 1600.0, 1.0, 1600.0, 1.0, 4500.0, 1.0]),
        np.diag([2.3, 2.3, 0.55, 0.55]),
    ),

]

INFANTRY4_QR_POINTS: list[QrPoint] = [
    QrPoint(
        0.16,
        np.diag([200.0, 100.0, 200.0, 1.0, 800.0, 1.0, 800.0, 1.0, 3200.0, 1.0]),
        np.diag([2.4, 2.4, 1.0, 1.0]),
    ),
    QrPoint(
        0.23,
        np.diag([200.0, 120.0, 200.0, 1.0, 1200.0, 4.0, 1200.0, 4.0, 3200.0, 1.0]),
        np.diag([3.5, 3.5, 0.5, 0.5]),
    ),
    QrPoint(
        0.33,
        np.diag([150.0, 60.0, 200.0, 1.0, 1200.0, 1.0, 1200.0, 1.0, 3200.0, 1.0]),
        np.diag([4.0, 4.0, 0.5, 0.5]),
    ),
]

VARIANT_QR_POINTS: dict[str, list[QrPoint]] = {
    "hero": HERO_QR_POINTS,
    "infantry3": INFANTRY3_QR_POINTS,
    "infantry4": INFANTRY4_QR_POINTS,
}

VARIANT_CPP_NAMES: dict[str, str] = {
    "hero": "kCtrlPHero",
    "infantry3": "kCtrlPInfantry3",
    "infantry4": "kCtrlPInfantry4",
}


CONTROL_NAMES = ["T_wl", "T_wr", "T_bl", "T_br"]
STATE_NAMES = [
    "s",
    "s_dot",
    "phi",
    "phi_dot",
    "theta_ll",
    "theta_ll_dot",
    "theta_lr",
    "theta_lr_dot",
    "theta_b",
    "theta_b_dot",
]
CONTROL_DESCRIPTIONS = [
    "left wheel torque",
    "right wheel torque",
    "left hip equivalent torque",
    "right hip equivalent torque",
]
STATE_DESCRIPTIONS = [
    "forward displacement error",
    "forward velocity error",
    "yaw angle error",
    "yaw rate error",
    "left leg swing angle error",
    "left leg swing angular velocity error",
    "right leg swing angle error",
    "right leg swing angular velocity error",
    "body pitch angle error",
    "body pitch angular velocity error",
]


# ---------------------------------------------------------------------------
# 2) Physical parameters and leg data copied from the MATLAB script
# ---------------------------------------------------------------------------

G = 9.81

R_W = 0.058
R_L = 0.2025
L_C = 0.024
M_W = 0.3
M_L = 2.3
M_B = 20.0
I_W = 0.001009
I_B = 0.3
I_Z = 0.53302282

# Columns: leg_length, l_w, l_b, I_l
LEG_DATA = np.array(
    [
        [0.110000, 0.061990, 0.048010, 0.045494],
        [0.120000, 0.067466, 0.052534, 0.044995],
        [0.130000, 0.072986, 0.057014, 0.044639],
        [0.140000, 0.078550, 0.061450, 0.044425],
        [0.150000, 0.084158, 0.065842, 0.044354],
        [0.160000, 0.089810, 0.070190, 0.044425],
        [0.170000, 0.095506, 0.074494, 0.044639],
        [0.180000, 0.101246, 0.078754, 0.044995],
        [0.190000, 0.107030, 0.082970, 0.045494],
        [0.200000, 0.112858, 0.087142, 0.046135],
        [0.210000, 0.118730, 0.091270, 0.046919],
        [0.220000, 0.124646, 0.095354, 0.047845],
        [0.230000, 0.130606, 0.099394, 0.048914],
        [0.240000, 0.136610, 0.103390, 0.050125],
        [0.250000, 0.142658, 0.107342, 0.051479],
        [0.260000, 0.148750, 0.111250, 0.052975],
        [0.270000, 0.154886, 0.115114, 0.054614],
        [0.280000, 0.161066, 0.118934, 0.056396],
        [0.290000, 0.167290, 0.122710, 0.058320],
        [0.300000, 0.173558, 0.126442, 0.060387],
        [0.310000, 0.179870, 0.130130, 0.062597],
        [0.320000, 0.186226, 0.133774, 0.064949],
        [0.330000, 0.192626, 0.137374, 0.067444],
        [0.340000, 0.199070, 0.140930, 0.070081],
    ],
    dtype=float,
)


def interp_or_extrapolate(x: float, xp: np.ndarray, fp: np.ndarray) -> float:
    """Linear interpolation with linear extrapolation at the two ends."""
    if x <= xp[0]:
        slope = (fp[1] - fp[0]) / (xp[1] - xp[0])
        return float(fp[0] + slope * (x - xp[0]))
    if x >= xp[-1]:
        slope = (fp[-1] - fp[-2]) / (xp[-1] - xp[-2])
        return float(fp[-1] + slope * (x - xp[-1]))
    return float(np.interp(x, xp, fp))


def leg_params(leg_length_m: float) -> tuple[float, float, float]:
    lengths = LEG_DATA[:, 0]
    l_w = interp_or_extrapolate(leg_length_m, lengths, LEG_DATA[:, 1])
    l_b = interp_or_extrapolate(leg_length_m, lengths, LEG_DATA[:, 2])
    i_l = interp_or_extrapolate(leg_length_m, lengths, LEG_DATA[:, 3])
    return l_w, l_b, i_l


def validate_qr_points(points: Iterable[QrPoint]) -> list[QrPoint]:
    sorted_points = sorted(points, key=lambda p: p.leg_length_m)
    if len(sorted_points) < 2:
        raise ValueError("QR_POINTS must contain at least two leg-length points.")

    seen: set[float] = set()
    for point in sorted_points:
        if point.leg_length_m in seen:
            raise ValueError(f"Duplicate QR point at leg length {point.leg_length_m}.")
        seen.add(point.leg_length_m)

        if point.Q.shape != (10, 10):
            raise ValueError(f"Q at {point.leg_length_m} must be 10x10.")
        if point.R.shape != (4, 4):
            raise ValueError(f"R at {point.leg_length_m} must be 4x4.")
        if not np.allclose(point.Q, point.Q.T, atol=1e-8):
            raise ValueError(f"Q at {point.leg_length_m} is not symmetric.")
        if not np.allclose(point.R, point.R.T, atol=1e-8):
            raise ValueError(f"R at {point.leg_length_m} is not symmetric.")

    return sorted_points


def scheduled_qr(mean_leg_length_m: float, points: list[QrPoint]) -> tuple[np.ndarray, np.ndarray]:
    lengths = np.array([p.leg_length_m for p in points])
    alpha_index = np.searchsorted(lengths, mean_leg_length_m)

    if alpha_index <= 0:
        return points[0].Q.copy(), points[0].R.copy()
    if alpha_index >= len(points):
        return points[-1].Q.copy(), points[-1].R.copy()

    lo = points[alpha_index - 1]
    hi = points[alpha_index]
    alpha = (mean_leg_length_m - lo.leg_length_m) / (hi.leg_length_m - lo.leg_length_m)
    Q = (1.0 - alpha) * lo.Q + alpha * hi.Q
    R = (1.0 - alpha) * lo.R + alpha * hi.R
    return Q, R


def build_ab(l_l: float, l_r: float) -> tuple[np.ndarray, np.ndarray]:
    l_wl, l_bl, i_ll = leg_params(l_l)
    l_wr, l_br, i_lr = leg_params(l_r)

    m = np.zeros((5, 5), dtype=float)
    gravity_theta = np.zeros((5, 3), dtype=float)
    torque = np.zeros((5, 4), dtype=float)

    m[0, 0] = I_W * l_l / R_W + M_W * R_W * l_l + M_L * R_W * l_bl
    m[0, 2] = M_L * l_wl * l_bl - i_ll
    gravity_theta[0, 0] = (M_L * l_wl + M_B * l_l / 2.0) * G
    torque[0, :] = [-(1.0 + l_l / R_W), 0.0, 1.0, 0.0]

    m[1, 1] = I_W * l_r / R_W + M_W * R_W * l_r + M_L * R_W * l_br
    m[1, 3] = M_L * l_wr * l_br - i_lr
    gravity_theta[1, 1] = (M_L * l_wr + M_B * l_r / 2.0) * G
    torque[1, :] = [0.0, -(1.0 + l_r / R_W), 0.0, 1.0]

    wheel_inertia = M_W * R_W * R_W + I_W + M_L * R_W * R_W + M_B * R_W * R_W / 2.0
    m[2, 0] = -wheel_inertia
    m[2, 1] = -wheel_inertia
    m[2, 2] = -(M_L * R_W * l_wl + M_B * R_W * l_l / 2.0)
    m[2, 3] = -(M_L * R_W * l_wr + M_B * R_W * l_r / 2.0)
    torque[2, :] = [1.0, 1.0, 0.0, 0.0]

    wheel_body = M_W * R_W * L_C + I_W * L_C / R_W + M_L * R_W * L_C
    m[3, 0] = wheel_body
    m[3, 1] = wheel_body
    m[3, 2] = M_L * l_wl * L_C
    m[3, 3] = M_L * l_wr * L_C
    m[3, 4] = -I_B
    gravity_theta[3, 2] = M_B * G * L_C
    torque[3, :] = [-L_C / R_W, -L_C / R_W, -1.0, -1.0]

    yaw_wheel = I_Z * R_W / (2.0 * R_L) + I_W * R_L / R_W
    m[4, 0] = yaw_wheel
    m[4, 1] = -yaw_wheel
    m[4, 2] = I_Z * l_l / (2.0 * R_L)
    m[4, 3] = -I_Z * l_r / (2.0 * R_L)
    torque[4, :] = [-R_L / R_W, R_L / R_W, 0.0, 0.0]

    j_a = -np.linalg.solve(m, gravity_theta)
    j_b = -np.linalg.solve(m, torque)

    A = np.zeros((10, 10), dtype=float)
    B = np.zeros((10, 4), dtype=float)

    for row in range(0, 10, 2):
        A[row, row + 1] = 1.0

    for p in (4, 6, 8):
        a_idx = (p - 4) // 2
        A[1, p] = R_W * (j_a[0, a_idx] + j_a[1, a_idx]) / 2.0
        A[3, p] = (
            R_W * (-j_a[0, a_idx] + j_a[1, a_idx]) / (2.0 * R_L)
            - l_l * j_a[2, a_idx] / (2.0 * R_L)
            + l_r * j_a[3, a_idx] / (2.0 * R_L)
        )
        A[5, p] = j_a[2, a_idx]
        A[7, p] = j_a[3, a_idx]
        A[9, p] = j_a[4, a_idx]

    for h in range(4):
        B[1, h] = R_W * (j_b[0, h] + j_b[1, h]) / 2.0
        B[3, h] = (
            R_W * (-j_b[0, h] + j_b[1, h]) / (2.0 * R_L)
            - l_l * j_b[2, h] / (2.0 * R_L)
            + l_r * j_b[3, h] / (2.0 * R_L)
        )
        B[5, h] = j_b[2, h]
        B[7, h] = j_b[3, h]
        B[9, h] = j_b[4, h]

    return A, B


def lqr_gain(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    if solve_continuous_are is not None:
        P = solve_continuous_are(A, B, Q, R)
    else:
        P = solve_care_hamiltonian(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)


def solve_care_hamiltonian(A: np.ndarray, B: np.ndarray, Q: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Solve continuous ARE using the stable Hamiltonian invariant subspace.

    SciPy's solve_continuous_are is preferred when available. This fallback keeps
    the generator usable on lightweight Python installs that have numpy only.
    """
    n = A.shape[0]
    r_inv_bt = np.linalg.solve(R, B.T)
    hamiltonian = np.block(
        [
            [A, -B @ r_inv_bt],
            [-Q, -A.T],
        ]
    )
    eigvals, eigvecs = np.linalg.eig(hamiltonian)
    stable_indices = np.where(np.real(eigvals) < 0.0)[0]
    if len(stable_indices) != n:
        raise np.linalg.LinAlgError(
            f"CARE fallback expected {n} stable eigenvalues, got {len(stable_indices)}."
        )
    stable_vecs = eigvecs[:, stable_indices]
    u1 = stable_vecs[:n, :]
    u2 = stable_vecs[n:, :]
    P_complex = u2 @ np.linalg.inv(u1)
    max_imag = float(np.max(np.abs(np.imag(P_complex))))
    if max_imag > 1e-7:
        raise np.linalg.LinAlgError(f"CARE fallback produced a complex P, max imaginary part {max_imag:g}.")
    P = np.real(P_complex)
    return 0.5 * (P + P.T)


def poly22_design(samples: np.ndarray) -> np.ndarray:
    l_l = samples[:, 0]
    l_r = samples[:, 1]
    return np.column_stack([np.ones_like(l_l), l_l, l_r, l_l * l_l, l_l * l_r, l_r * l_r])


def fit_kctrlp(samples: np.ndarray, gains: np.ndarray) -> np.ndarray:
    design = poly22_design(samples)
    coeffs = np.zeros((40, 6), dtype=float)
    for idx in range(40):
        coeffs[idx, :] = np.linalg.lstsq(design, gains[:, idx], rcond=None)[0]
    return coeffs


def eval_kctrlp(coeffs: np.ndarray, l_l: float, l_r: float) -> np.ndarray:
    basis = np.array([1.0, l_l, l_r, l_l * l_l, l_l * l_r, l_r * l_r])
    return (coeffs @ basis).reshape(4, 10)


def format_cpp_array(coeffs: np.ndarray, name: str) -> str:
    flat = coeffs.reshape(-1)
    lines = [
        f"constexpr std::array<float, 240> {name}{{",
    ]
    for start in range(0, len(flat), 6):
        values = ", ".join(f"{v:.8g}f" for v in flat[start : start + 6])
        lines.append(f"    {values},")
    lines.append("};")
    return "\n".join(lines)


def format_cpp_header(variant_coeffs: dict[str, np.ndarray]) -> str:
    blocks = [
        "#pragma once",
        "",
        "#include <array>",
        "",
        "#ifndef WHEEL_LEGGED_ROBOT_VARIANT",
        "#define WHEEL_LEGGED_ROBOT_VARIANT 2",
        "#endif",
        "",
        "namespace wheel_legged::params::generated {",
        "",
    ]
    for variant in ("hero", "infantry3", "infantry4"):
        blocks.append(format_cpp_array(variant_coeffs[variant], VARIANT_CPP_NAMES[variant]))
        blocks.append("")

    blocks.extend(
        [
            "#if WHEEL_LEGGED_ROBOT_VARIANT == 1",
            "static constexpr const auto &kCtrlP = kCtrlPHero;",
            "#elif WHEEL_LEGGED_ROBOT_VARIANT == 2",
            "static constexpr const auto &kCtrlP = kCtrlPInfantry3;",
            "#elif WHEEL_LEGGED_ROBOT_VARIANT == 3",
            "static constexpr const auto &kCtrlP = kCtrlPInfantry4;",
            "#else",
            '#error "WHEEL_LEGGED_ROBOT_VARIANT must be 1, 2, or 3"',
            "#endif",
            "",
            "}  // namespace wheel_legged::params::generated",
        ]
    )
    return "\n".join(blocks) + "\n"


def svg_polyline(points: list[tuple[float, float]], stroke: str, width: float = 1.5) -> str:
    coords = " ".join(f"{x:.3f},{y:.3f}" for x, y in points)
    return f'<polyline points="{coords}" fill="none" stroke="{stroke}" stroke-width="{width}" />'


def svg_circle(x: float, y: float, radius: float, fill: str) -> str:
    return f'<circle cx="{x:.3f}" cy="{y:.3f}" r="{radius:.3f}" fill="{fill}" />'


def make_k_element_svg(
    x_values: np.ndarray,
    direct_values: np.ndarray,
    fitted_values: np.ndarray,
    title: str,
    subtitle: str,
    width: int = 240,
    height: int = 150,
) -> str:
    pad_l, pad_r, pad_t, pad_b = 38.0, 8.0, 20.0, 24.0
    plot_w = width - pad_l - pad_r
    plot_h = height - pad_t - pad_b

    y_all = np.concatenate([direct_values, fitted_values])
    y_min = float(np.min(y_all))
    y_max = float(np.max(y_all))
    if abs(y_max - y_min) < 1e-9:
        y_min -= 1.0
        y_max += 1.0
    y_margin = 0.08 * (y_max - y_min)
    y_min -= y_margin
    y_max += y_margin

    x_min = float(np.min(x_values))
    x_max = float(np.max(x_values))

    def sx(x: float) -> float:
        return pad_l + (x - x_min) / (x_max - x_min) * plot_w

    def sy(y: float) -> float:
        return pad_t + (y_max - y) / (y_max - y_min) * plot_h

    fitted_points = [(sx(float(x)), sy(float(y))) for x, y in zip(x_values, fitted_values)]
    direct_points = [(sx(float(x)), sy(float(y))) for x, y in zip(x_values, direct_values)]

    zero_line = ""
    if y_min <= 0.0 <= y_max:
        y0 = sy(0.0)
        zero_line = f'<line x1="{pad_l}" y1="{y0:.3f}" x2="{pad_l + plot_w}" y2="{y0:.3f}" stroke="#d5dce6" />'

    direct_marks = "\n".join(svg_circle(x, y, 2.3, "#c2410c") for x, y in direct_points)
    return f"""
<svg viewBox="0 0 {width} {height}" role="img" aria-label="{html.escape(title)}">
  <rect x="0" y="0" width="{width}" height="{height}" fill="#ffffff" />
  <text x="{pad_l}" y="13" font-size="11" font-family="Consolas, monospace" fill="#172033">{html.escape(title)}</text>
  <title>{html.escape(subtitle)}</title>
  <rect x="{pad_l}" y="{pad_t}" width="{plot_w}" height="{plot_h}" fill="#f8fafc" stroke="#d9e0ea" />
  {zero_line}
  {svg_polyline(fitted_points, "#2563eb", 1.8)}
  {direct_marks}
  <text x="4" y="{pad_t + 9}" font-size="9" font-family="Consolas, monospace" fill="#64748b">{y_max:.3g}</text>
  <text x="4" y="{pad_t + plot_h}" font-size="9" font-family="Consolas, monospace" fill="#64748b">{y_min:.3g}</text>
  <text x="{pad_l}" y="{height - 6}" font-size="9" font-family="Consolas, monospace" fill="#64748b">{x_min:.2f}m</text>
  <text x="{width - 42}" y="{height - 6}" font-size="9" font-family="Consolas, monospace" fill="#64748b">{x_max:.2f}m</text>
</svg>""".strip()


def build_html_report(
    coeffs: np.ndarray, args: argparse.Namespace, report: str, variant: str, points: list[QrPoint]
) -> str:
    x_values = make_grid(args.leg_min, args.leg_max, args.grid_step)

    direct = np.zeros((len(x_values), 40), dtype=float)
    fitted = np.zeros((len(x_values), 40), dtype=float)
    abs_err = np.zeros((len(x_values), 40), dtype=float)

    for idx, leg_length in enumerate(x_values):
        leg = float(leg_length)
        Q, R = scheduled_qr(leg, points)
        A, B = build_ab(leg, leg)
        direct_k = lqr_gain(A, B, Q, R).reshape(-1)
        fitted_k = eval_kctrlp(coeffs, leg, leg).reshape(-1)
        direct[idx, :] = direct_k
        fitted[idx, :] = fitted_k
        abs_err[idx, :] = np.abs(fitted_k - direct_k)

    cards: list[str] = []
    for row in range(4):
        for col in range(10):
            flat_idx = row * 10 + col
            title = f"K[{row + 1},{col + 1}] {CONTROL_NAMES[row]}<-{STATE_NAMES[col]}"
            meaning = (
                f"{CONTROL_NAMES[row]} ({CONTROL_DESCRIPTIONS[row]}) gain on "
                f"{STATE_NAMES[col]} ({STATE_DESCRIPTIONS[col]})"
            )
            max_err = float(np.max(abs_err[:, flat_idx]))
            svg = make_k_element_svg(x_values, direct[:, flat_idx], fitted[:, flat_idx], title, meaning)
            cards.append(
                f"""
<section class="card">
  {svg}
  <div class="meaning">{html.escape(meaning)}</div>
  <div class="metric">max |fit-direct| = {max_err:.6g}</div>
</section>""".strip()
            )

    escaped_report = html.escape(report)
    generated_from = html.escape(str(Path(__file__).as_posix()))
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <title>Wheel-Legged kCtrlP Fit Report</title>
  <style>
    body {{
      margin: 24px;
      background: #eef2f7;
      color: #172033;
      font-family: Arial, "Microsoft YaHei", sans-serif;
    }}
    h1 {{ margin: 0 0 8px; font-size: 22px; }}
    .subtle {{ color: #64748b; margin-bottom: 16px; }}
    pre {{
      background: #172033;
      color: #e5edf7;
      padding: 14px;
      border-radius: 6px;
      overflow-x: auto;
      line-height: 1.45;
    }}
    .legend {{
      display: flex;
      gap: 18px;
      margin: 14px 0 18px;
      font-size: 14px;
      align-items: center;
    }}
    .swatch {{
      display: inline-block;
      width: 26px;
      height: 3px;
      vertical-align: middle;
      margin-right: 6px;
      background: #2563eb;
    }}
    .dot {{
      display: inline-block;
      width: 8px;
      height: 8px;
      border-radius: 50%;
      vertical-align: middle;
      margin-right: 6px;
      background: #c2410c;
    }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(5, minmax(220px, 1fr));
      gap: 12px;
    }}
    .card {{
      background: #ffffff;
      border: 1px solid #d9e0ea;
      border-radius: 6px;
      padding: 8px;
    }}
    .card svg {{ display: block; width: 100%; height: auto; }}
    .meaning {{
      color: #1f2937;
      font-size: 12px;
      line-height: 1.35;
      min-height: 32px;
      margin-top: 4px;
    }}
    .metric {{
      color: #475569;
      font-family: Consolas, monospace;
      font-size: 11px;
      margin-top: 4px;
    }}
    @media (max-width: 1400px) {{ .grid {{ grid-template-columns: repeat(4, minmax(220px, 1fr)); }} }}
    @media (max-width: 1100px) {{ .grid {{ grid-template-columns: repeat(3, minmax(220px, 1fr)); }} }}
    @media (max-width: 760px) {{ .grid {{ grid-template-columns: 1fr; }} }}
  </style>
</head>
<body>
  <h1>Wheel-Legged kCtrlP Fit Report - {html.escape(variant)}</h1>
  <div class="subtle">Generated by {generated_from}</div>
  <pre>{escaped_report}</pre>
  <div class="legend">
    <span><span class="swatch"></span>fitted kCtrlP on equal-leg line</span>
    <span><span class="dot"></span>direct LQR sample</span>
  </div>
  <div class="grid">
    {''.join(cards)}
  </div>
</body>
</html>
"""


def make_grid(start: float, stop: float, step: float) -> np.ndarray:
    count = int(round((stop - start) / step))
    values = start + step * np.arange(count + 1)
    values[-1] = stop
    return values


def generate(args: argparse.Namespace, variant: str, qr_points: list[QrPoint]) -> tuple[np.ndarray, str]:
    points = validate_qr_points(qr_points)
    grid = make_grid(args.leg_min, args.leg_max, args.grid_step)

    if args.leg_min < LEG_DATA[0, 0] or args.leg_max > LEG_DATA[-1, 0]:
        print(
            f"warning: leg data range is {LEG_DATA[0, 0]:.3f}..{LEG_DATA[-1, 0]:.3f}; "
            "values outside it use linear extrapolation."
        )

    samples: list[tuple[float, float]] = []
    gains: list[np.ndarray] = []
    max_real_part = -math.inf

    for l_l in grid:
        for l_r in grid:
            mean_l = 0.5 * (float(l_l) + float(l_r))
            Q, R = scheduled_qr(mean_l, points)
            A, B = build_ab(float(l_l), float(l_r))
            K = lqr_gain(A, B, Q, R)
            max_real_part = max(max_real_part, float(np.max(np.real(np.linalg.eigvals(A - B @ K)))))
            samples.append((float(l_l), float(l_r)))
            gains.append(K.reshape(-1))

    sample_array = np.array(samples, dtype=float)
    gain_array = np.array(gains, dtype=float)
    coeffs = fit_kctrlp(sample_array, gain_array)

    fitted_gain_array = np.array([eval_kctrlp(coeffs, l_l, l_r).reshape(-1) for l_l, l_r in sample_array])
    abs_error = np.abs(fitted_gain_array - gain_array)
    rel_error = abs_error / np.maximum(np.abs(gain_array), 1e-3)

    equal_errors = []
    for l in grid:
        Q, R = scheduled_qr(float(l), points)
        A, B = build_ab(float(l), float(l))
        direct = lqr_gain(A, B, Q, R)
        fitted = eval_kctrlp(coeffs, float(l), float(l))
        equal_errors.append(np.abs(fitted - direct))

    equal_abs_error = np.max(np.array(equal_errors))
    report = "\n".join(
        [
            f"grid points       : {len(grid)} x {len(grid)} = {len(sample_array)}",
            f"variant           : {variant}",
            f"leg range         : {args.leg_min:.3f} .. {args.leg_max:.3f} m, step {args.grid_step:.3f} m",
            f"QR schedule points: {', '.join(f'{p.leg_length_m:.3f}' for p in points)}",
            f"max fit abs error : {float(np.max(abs_error)):.6g}",
            f"max fit rel error : {float(np.max(rel_error)):.6g}",
            f"equal-line max abs: {float(equal_abs_error):.6g}",
            f"direct LQR max Re(lambda(A-BK)): {max_real_part:.6g}",
        ]
    )
    return coeffs, report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--leg-min", type=float, default=0.15, help="minimum leg length for the 2-D fit grid")
    parser.add_argument("--leg-max", type=float, default=0.35, help="maximum leg length for the 2-D fit grid")
    parser.add_argument("--grid-step", type=float, default=0.01, help="leg length grid step")
    parser.add_argument("--output", type=Path, help="optional file path for the generated conditional C++ header")
    parser.add_argument("--html-report", type=Path, help="optional HTML/SVG report path or directory")
    parser.add_argument(
        "--report-variant",
        choices=["hero", "infantry3", "infantry4", "all"],
        default="infantry3",
        help="which variant(s) to write when --html-report is set",
    )
    args = parser.parse_args()

    variant_coeffs: dict[str, np.ndarray] = {}
    variant_reports: dict[str, str] = {}
    validated_points: dict[str, list[QrPoint]] = {}
    for variant, qr_points in VARIANT_QR_POINTS.items():
        coeffs, report = generate(args, variant, qr_points)
        variant_coeffs[variant] = coeffs
        variant_reports[variant] = report
        validated_points[variant] = validate_qr_points(qr_points)

    cpp = format_cpp_header(variant_coeffs)

    print("\n\n".join(variant_reports[variant] for variant in ("hero", "infantry3", "infantry4")))
    print()
    print(cpp)

    if args.output:
        args.output.write_text(cpp + "\n", encoding="utf-8")
        print()
        print(f"wrote: {args.output}")

    if args.html_report:
        selected_variants = ("hero", "infantry3", "infantry4") if args.report_variant == "all" else (args.report_variant,)
        report_path = args.html_report
        for variant in selected_variants:
            if len(selected_variants) == 1 and report_path.suffix:
                out_path = report_path
            else:
                report_path.mkdir(parents=True, exist_ok=True)
                out_path = report_path / f"kctrlp_fit_report_{variant}.html"
            out_path.write_text(
                build_html_report(
                    variant_coeffs[variant], args, variant_reports[variant], variant, validated_points[variant]
                ),
                encoding="utf-8",
            )
            print(f"wrote: {out_path}")


if __name__ == "__main__":
    main()
