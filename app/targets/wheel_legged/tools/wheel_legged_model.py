#!/usr/bin/env python3
"""Shared five-DOF wheel-legged model used by the LQR and LESO generators.

Model-coordinate convention
---------------------------
q = [theta_wl, theta_wr, theta_ll, theta_lr, theta_b]
u = [T_wl, T_wr, T_bl, T_br]

The descriptor-form dynamics are

    M q_ddot + G q = B_tau u + d

and the controller state is

    x = [s, s_dot, phi, phi_dot,
         theta_ll, theta_ll_dot,
         theta_lr, theta_lr_dot,
         theta_b, theta_b_dot].

Each robot variant deliberately owns an independent parameter object and leg
table, even while several values are currently identical. This makes later
replacement with measured/CAD values explicit and local.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RobotModelParams:
    wheel_radius_m: float
    half_track_m: float
    body_com_offset_m: float
    wheel_mass_kg: float
    leg_mass_kg: float
    body_mass_kg: float
    wheel_inertia_kg_m2: float
    body_pitch_inertia_kg_m2: float
    body_yaw_inertia_kg_m2: float
    gravity_mps2: float = 9.81


@dataclass(frozen=True)
class FiveDofModel:
    mass: np.ndarray
    gravity: np.ndarray
    input_map: np.ndarray


# Columns: leg length, wheel-side COM distance, body-side COM distance,
# leg pitch inertia.
_COMMON_LEG_DATA = np.array(
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


# TODO(model-identification): replace each profile with measured/CAD values.
# Current values intentionally match the latest shared LQR model, except for
# the confirmed wheel radius and Hero half-track.
ROBOT_MODEL_PARAMS: dict[str, RobotModelParams] = {
    "hero": RobotModelParams(
        wheel_radius_m=0.0575,
        half_track_m=0.24,
        body_com_offset_m=0.024,
        wheel_mass_kg=0.3,
        leg_mass_kg=2.3,
        body_mass_kg=20.0,
        wheel_inertia_kg_m2=0.001009,
        body_pitch_inertia_kg_m2=0.3,
        body_yaw_inertia_kg_m2=0.53302282,
    ),
    "infantry3": RobotModelParams(
        wheel_radius_m=0.0575,
        half_track_m=0.2025,
        body_com_offset_m=0.024,
        wheel_mass_kg=0.3,
        leg_mass_kg=2.3,
        body_mass_kg=20.0,
        wheel_inertia_kg_m2=0.001009,
        body_pitch_inertia_kg_m2=0.3,
        body_yaw_inertia_kg_m2=0.53302282,
    ),
    "infantry4": RobotModelParams(
        wheel_radius_m=0.0575,
        half_track_m=0.2025,
        body_com_offset_m=0.024,
        wheel_mass_kg=0.3,
        leg_mass_kg=2.3,
        body_mass_kg=20.0,
        wheel_inertia_kg_m2=0.001009,
        body_pitch_inertia_kg_m2=0.3,
        body_yaw_inertia_kg_m2=0.53302282,
    ),
}


# These are intentionally separate arrays, not aliases. Later measurements for
# one robot can be pasted without silently changing another variant.
ROBOT_LEG_DATA: dict[str, np.ndarray] = {
    "hero": _COMMON_LEG_DATA.copy(),
    "infantry3": _COMMON_LEG_DATA.copy(),
    "infantry4": _COMMON_LEG_DATA.copy(),
}


def base_variant_name(variant: str) -> str:
    """Map controller variants such as infantry3_spin to a physical profile."""
    return variant.removesuffix("_spin")


def model_params(variant: str) -> RobotModelParams:
    return ROBOT_MODEL_PARAMS[base_variant_name(variant)]


def leg_data(variant: str) -> np.ndarray:
    return ROBOT_LEG_DATA[base_variant_name(variant)]


def interp_or_extrapolate(x: float, xp: np.ndarray, fp: np.ndarray) -> float:
    """Linear interpolation with linear extrapolation at both ends."""
    if x <= xp[0]:
        slope = (fp[1] - fp[0]) / (xp[1] - xp[0])
        return float(fp[0] + slope * (x - xp[0]))
    if x >= xp[-1]:
        slope = (fp[-1] - fp[-2]) / (xp[-1] - xp[-2])
        return float(fp[-1] + slope * (x - xp[-1]))
    return float(np.interp(x, xp, fp))


def leg_params(variant: str, leg_length_m: float) -> tuple[float, float, float]:
    data = leg_data(variant)
    lengths = data[:, 0]
    l_w = interp_or_extrapolate(leg_length_m, lengths, data[:, 1])
    l_b = interp_or_extrapolate(leg_length_m, lengths, data[:, 2])
    i_l = interp_or_extrapolate(leg_length_m, lengths, data[:, 3])
    return l_w, l_b, i_l


def build_five_dof_model(variant: str, l_l: float, l_r: float) -> FiveDofModel:
    """Build M, G and B_tau for M*q_ddot + G*q = B_tau*u + d."""
    p = model_params(variant)
    l_wl, l_bl, i_ll = leg_params(variant, l_l)
    l_wr, l_br, i_lr = leg_params(variant, l_r)

    r_w = p.wheel_radius_m
    r_l = p.half_track_m
    l_c = p.body_com_offset_m
    m_w = p.wheel_mass_kg
    m_l = p.leg_mass_kg
    m_b = p.body_mass_kg
    i_w = p.wheel_inertia_kg_m2
    i_b = p.body_pitch_inertia_kg_m2
    i_z = p.body_yaw_inertia_kg_m2
    gravity_accel = p.gravity_mps2

    mass = np.zeros((5, 5), dtype=float)
    gravity = np.zeros((5, 5), dtype=float)
    torque_lhs = np.zeros((5, 4), dtype=float)

    mass[0, 0] = i_w * l_l / r_w + m_w * r_w * l_l + m_l * r_w * l_bl
    mass[0, 2] = m_l * l_wl * l_bl - i_ll
    gravity[0, 2] = (m_l * l_wl + m_b * l_l / 2.0) * gravity_accel
    torque_lhs[0, :] = [-(1.0 + l_l / r_w), 0.0, 1.0, 0.0]

    mass[1, 1] = i_w * l_r / r_w + m_w * r_w * l_r + m_l * r_w * l_br
    mass[1, 3] = m_l * l_wr * l_br - i_lr
    gravity[1, 3] = (m_l * l_wr + m_b * l_r / 2.0) * gravity_accel
    torque_lhs[1, :] = [0.0, -(1.0 + l_r / r_w), 0.0, 1.0]

    wheel_inertia = m_w * r_w * r_w + i_w + m_l * r_w * r_w + m_b * r_w * r_w / 2.0
    mass[2, 0] = -wheel_inertia
    mass[2, 1] = -wheel_inertia
    mass[2, 2] = -(m_l * r_w * l_wl + m_b * r_w * l_l / 2.0)
    mass[2, 3] = -(m_l * r_w * l_wr + m_b * r_w * l_r / 2.0)
    torque_lhs[2, :] = [1.0, 1.0, 0.0, 0.0]

    wheel_body = m_w * r_w * l_c + i_w * l_c / r_w + m_l * r_w * l_c
    mass[3, 0] = wheel_body
    mass[3, 1] = wheel_body
    mass[3, 2] = m_l * l_wl * l_c
    mass[3, 3] = m_l * l_wr * l_c
    mass[3, 4] = -i_b
    gravity[3, 4] = m_b * gravity_accel * l_c
    torque_lhs[3, :] = [-l_c / r_w, -l_c / r_w, -1.0, -1.0]

    yaw_wheel = i_z * r_w / (2.0 * r_l) + i_w * r_l / r_w
    mass[4, 0] = yaw_wheel
    mass[4, 1] = -yaw_wheel
    mass[4, 2] = i_z * l_l / (2.0 * r_l)
    mass[4, 3] = -i_z * l_r / (2.0 * r_l)
    torque_lhs[4, :] = [-r_l / r_w, r_l / r_w, 0.0, 0.0]

    return FiveDofModel(mass=mass, gravity=gravity, input_map=-torque_lhs)


def build_ab(variant: str, l_l: float, l_r: float) -> tuple[np.ndarray, np.ndarray]:
    """Transform the five-DOF descriptor model into the 10-state LQR model."""
    p = model_params(variant)
    model = build_five_dof_model(variant, l_l, l_r)
    gravity_theta = model.gravity[:, 2:5]
    j_a = -np.linalg.solve(model.mass, gravity_theta)
    j_b = np.linalg.solve(model.mass, model.input_map)

    a = np.zeros((10, 10), dtype=float)
    b = np.zeros((10, 4), dtype=float)

    for row in range(0, 10, 2):
        a[row, row + 1] = 1.0

    for state_col in (4, 6, 8):
        angle_index = (state_col - 4) // 2
        a[1, state_col] = p.wheel_radius_m * (j_a[0, angle_index] + j_a[1, angle_index]) / 2.0
        a[3, state_col] = (
            p.wheel_radius_m * (-j_a[0, angle_index] + j_a[1, angle_index]) / (2.0 * p.half_track_m)
            - l_l * j_a[2, angle_index] / (2.0 * p.half_track_m)
            + l_r * j_a[3, angle_index] / (2.0 * p.half_track_m)
        )
        a[5, state_col] = j_a[2, angle_index]
        a[7, state_col] = j_a[3, angle_index]
        a[9, state_col] = j_a[4, angle_index]

    for control_col in range(4):
        b[1, control_col] = p.wheel_radius_m * (j_b[0, control_col] + j_b[1, control_col]) / 2.0
        b[3, control_col] = (
            p.wheel_radius_m * (-j_b[0, control_col] + j_b[1, control_col]) / (2.0 * p.half_track_m)
            - l_l * j_b[2, control_col] / (2.0 * p.half_track_m)
            + l_r * j_b[3, control_col] / (2.0 * p.half_track_m)
        )
        b[5, control_col] = j_b[2, control_col]
        b[7, control_col] = j_b[3, control_col]
        b[9, control_col] = j_b[4, control_col]

    return a, b


def state_to_generalized(
    variant: str,
    state: np.ndarray,
    left_leg_rate_mps: float = 0.0,
    right_leg_rate_mps: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply the complete inverse transform used by the LQR derivation.

    state order:
      [s, s_dot, phi, phi_dot, theta_ll, theta_ll_dot,
       theta_lr, theta_lr_dot, theta_b, theta_b_dot, l_l, l_r]
    """
    if state.shape != (12,):
        raise ValueError(f"state must have shape (12,), got {state.shape}")

    p = model_params(variant)
    s, s_dot, phi, phi_dot, theta_ll, theta_ll_dot, theta_lr, theta_lr_dot, theta_b, theta_b_dot, l_l, l_r = state

    left_coupling = 0.5 * l_l * theta_ll
    right_coupling = 0.5 * l_r * theta_lr
    theta_wl = (s - p.half_track_m * phi - left_coupling + right_coupling) / p.wheel_radius_m
    theta_wr = (s + p.half_track_m * phi + left_coupling - right_coupling) / p.wheel_radius_m

    left_coupling_dot = 0.5 * (left_leg_rate_mps * theta_ll + l_l * theta_ll_dot)
    right_coupling_dot = 0.5 * (right_leg_rate_mps * theta_lr + l_r * theta_lr_dot)
    theta_wl_dot = (
        s_dot - p.half_track_m * phi_dot - left_coupling_dot + right_coupling_dot
    ) / p.wheel_radius_m
    theta_wr_dot = (
        s_dot + p.half_track_m * phi_dot + left_coupling_dot - right_coupling_dot
    ) / p.wheel_radius_m

    q = np.array([theta_wl, theta_wr, theta_ll, theta_lr, theta_b], dtype=float)
    dq = np.array([theta_wl_dot, theta_wr_dot, theta_ll_dot, theta_lr_dot, theta_b_dot], dtype=float)
    return q, dq
