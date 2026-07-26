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
class LegMassProperties:
    """Equivalent single-leg properties in the sagittal virtual-leg frame.

    ``com_parallel_from_upper_m`` is positive from the virtual-leg upper pivot
    towards the wheel axle. ``com_normal_m`` is positive along the in-plane
    normal obtained by rotating that direction counter-clockwise. The CAD
    coordinate convention used for the current table therefore produces a
    negative normal offset.
    """

    com_parallel_from_upper_m: float
    com_normal_m: float
    pitch_inertia_com_kg_m2: float


@dataclass(frozen=True)
class FiveDofModel:
    mass: np.ndarray
    gravity: np.ndarray
    input_map: np.ndarray


# CAD source:
#   outputs/solidworks_leg_measurement_20260724/leg_mass_properties_raw.csv
#
# Columns:
#   virtual-leg length,
#   COM projection from the upper pivot along upper-pivot -> wheel,
#   signed in-plane COM normal offset,
#   pitch inertia about the COM (global CAD X / wheel-axle direction).
#
# The old table forced the COM onto the virtual-leg line. This table retains
# the measured two-dimensional COM vector. See leg_params() and
# build_five_dof_model() for the exact linearized reduction used by LQR/LESO.
_COMMON_LEG_DATA = np.array(
    [
        [0.110000, -0.002223478, -0.086718091, 0.0146738336],
        [0.120000, 0.003731177, -0.087073613, 0.0147945296],
        [0.130000, 0.009212570, -0.087155441, 0.0149124178],
        [0.140000, 0.014321405, -0.087027371, 0.0150303403],
        [0.150000, 0.019131471, -0.086731423, 0.0151502099],
        [0.160000, 0.023698014, -0.086296060, 0.0152733568],
        [0.170000, 0.028063223, -0.085740970, 0.0154007342],
        [0.180000, 0.032259877, -0.085079952, 0.0155330443],
        [0.190000, 0.036313893, -0.084322717, 0.0156708193],
        [0.200000, 0.040245854, -0.083476092, 0.0158144659],
        [0.210000, 0.044072538, -0.082544730, 0.0159643094],
        [0.220000, 0.047807637, -0.081531657, 0.0161206112],
        [0.230000, 0.051462465, -0.080438607, 0.0162835867],
        [0.240000, 0.055046451, -0.079266254, 0.0164534165],
        [0.250000, 0.058567518, -0.078014363, 0.0166302550],
        [0.260000, 0.062032369, -0.076681880, 0.0168142368],
        [0.270000, 0.065446714, -0.075266984, 0.0170054807],
        [0.280000, 0.068815442, -0.073767091, 0.0172040935],
        [0.290000, 0.072142762, -0.072178828, 0.0174101730],
        [0.300000, 0.075432312, -0.070497968, 0.0176238097],
        [0.310000, 0.078687244, -0.068719336, 0.0178450891],
        [0.320000, 0.081910299, -0.066836653, 0.0180740929],
        [0.330000, 0.085103856, -0.064842333, 0.0183109005],
        [0.336337, 0.087113357, -0.063516607, 0.0184650458],
    ],
    dtype=float,
)


# TODO(model-identification): replace each profile with measured/CAD values.
# The three variants intentionally share the current CAD single-leg mass until
# independent CAD/measurement results are available.
ROBOT_MODEL_PARAMS: dict[str, RobotModelParams] = {
    "hero": RobotModelParams(
        wheel_radius_m=0.0575,
        half_track_m=0.24,
        body_com_offset_m=0.024,
        wheel_mass_kg=0.3,
        leg_mass_kg=1.640907,
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
        leg_mass_kg=1.640907,
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
        leg_mass_kg=1.640907,
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


def leg_params(variant: str, leg_length_m: float) -> LegMassProperties:
    data = leg_data(variant)
    lengths = data[:, 0]
    return LegMassProperties(
        com_parallel_from_upper_m=interp_or_extrapolate(leg_length_m, lengths, data[:, 1]),
        com_normal_m=interp_or_extrapolate(leg_length_m, lengths, data[:, 2]),
        pitch_inertia_com_kg_m2=interp_or_extrapolate(leg_length_m, lengths, data[:, 3]),
    )


def build_five_dof_model(variant: str, l_l: float, l_r: float) -> FiveDofModel:
    """Build M, G and B_tau for M*q_ddot + G*q = B_tau*u + d."""
    p = model_params(variant)
    left_leg = leg_params(variant, l_l)
    right_leg = leg_params(variant, l_r)

    # The legacy collinear model used l_b from the upper pivot to the COM and
    # l_w = l_0 - l_b from the wheel to the COM. In the upright linearization,
    # the translational and gravity couplings use the tangent projection
    # c_parallel. The normal offset contributes through the parallel-axis term
    # m*c_normal^2. This reduction is exactly the legacy model when
    # c_normal == 0.
    l_bl = left_leg.com_parallel_from_upper_m
    l_br = right_leg.com_parallel_from_upper_m
    l_wl = l_l - l_bl
    l_wr = l_r - l_br

    r_w = p.wheel_radius_m
    r_l = p.half_track_m
    l_c = p.body_com_offset_m
    m_w = p.wheel_mass_kg
    m_l = p.leg_mass_kg
    m_b = p.body_mass_kg
    i_w = p.wheel_inertia_kg_m2
    i_ll = left_leg.pitch_inertia_com_kg_m2 + m_l * left_leg.com_normal_m**2
    i_lr = right_leg.pitch_inertia_com_kg_m2 + m_l * right_leg.com_normal_m**2
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
