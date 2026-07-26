#!/usr/bin/env python3
"""Generate scheduled model data for the momentum-based LESO.

The generated data follows the model-coordinate equation

    M q_ddot + G q = B_tau u + d

with

    q = [theta_wl, theta_wr, theta_ll, theta_lr, theta_b]
    u = [T_wl, T_wr, T_bl, T_br].

Runtime contract
----------------
The observer must use the complete LQR-coordinate inverse transform implemented
by wheel_legged_model.state_to_generalized(). Its input u must be the previous
cycle's final four-dimensional virtual command after mode overrides and virtual
actuator saturation, not the newly computed nominal LQR command.

The disturbance map is constructed from the Moore-Penrose pseudoinverse of
B_tau, followed by the requested engineering correction that makes the Pitch
disturbance column common-mode on the left/right hip virtual torques.
"""

from __future__ import annotations

import argparse
import html
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

TOOLS_DIR = Path(__file__).resolve().parents[1]
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

from wheel_legged_model import (
    ROBOT_MODEL_PARAMS,
    build_five_dof_model,
    leg_data,
    model_params,
    state_to_generalized,
)

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_OUTPUT = SCRIPT_DIR / "generated" / "leso_model_generated.hpp"
DEFAULT_HTML_REPORT = SCRIPT_DIR / "reports" / "leso_model_fit_report.html"


VARIANTS = ("hero", "infantry3", "infantry4")
BASIS_SIZE = 6
REGION_COUNT = 4
FIELD_SIZES = {
    "mass": 25,
    "gravity": 25,
    "input_map": 20,
    "disturbance_map": 20,
}
CPP_FIELD_NAMES = {
    "mass": "M",
    "gravity": "G",
    "input_map": "BTau",
    "disturbance_map": "DisturbanceMap",
}
CPP_VARIANT_NAMES = {
    "hero": "Hero",
    "infantry3": "Infantry3",
    "infantry4": "Infantry4",
}


@dataclass(frozen=True)
class DirectModelData:
    mass: np.ndarray
    gravity: np.ndarray
    input_map: np.ndarray
    strict_pinv: np.ndarray
    disturbance_map: np.ndarray


@dataclass(frozen=True)
class FieldMetrics:
    max_abs: float
    rms_abs: float
    max_fro_rel: float
    max_float32_abs: float
    worst_left_m: float
    worst_right_m: float
    boundary_jump: float


@dataclass(frozen=True)
class VariantResult:
    coefficients: dict[str, np.ndarray]
    field_metrics: dict[str, FieldMetrics]
    max_mass_condition: float
    max_btau_condition: float
    max_strict_pinv_identity_residual: float
    max_pitch_common_mode_error: float
    synthetic_generalized_error: float
    synthetic_motor_error: float


def make_grid(start: float, stop: float, step: float) -> np.ndarray:
    if step <= 0.0:
        raise ValueError("grid step must be positive")
    if stop <= start:
        raise ValueError("leg-max must be greater than leg-min")
    count = int(round((stop - start) / step))
    values = start + step * np.arange(count + 1, dtype=float)
    values[-1] = stop
    return values


def poly22_basis(l_l: np.ndarray | float, l_r: np.ndarray | float) -> np.ndarray:
    left = np.asarray(l_l)
    right = np.asarray(l_r)
    return np.stack(
        [
            np.ones_like(left),
            left,
            right,
            left * left,
            left * right,
            right * right,
        ],
        axis=-1,
    )


def region_index(l_l: float, l_r: float, split: float) -> int:
    return (1 if l_l > split else 0) + (2 if l_r > split else 0)


def pitch_common_mode_map(strict_pinv: np.ndarray) -> np.ndarray:
    result = strict_pinv.copy()
    common = 0.5 * (result[2, 4] + result[3, 4])
    result[2, 4] = common
    result[3, 4] = common
    return result


def direct_model_data(variant: str, l_l: float, l_r: float) -> DirectModelData:
    model = build_five_dof_model(variant, l_l, l_r)
    strict_pinv = np.linalg.pinv(model.input_map)
    return DirectModelData(
        mass=model.mass,
        gravity=model.gravity,
        input_map=model.input_map,
        strict_pinv=strict_pinv,
        disturbance_map=pitch_common_mode_map(strict_pinv),
    )


def flatten_fields(data: DirectModelData) -> dict[str, np.ndarray]:
    return {
        "mass": data.mass.reshape(-1),
        "gravity": data.gravity.reshape(-1),
        "input_map": data.input_map.reshape(-1),
        "disturbance_map": data.disturbance_map.reshape(-1),
    }


def collect_samples(variant: str, grid: np.ndarray) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    samples: list[tuple[float, float]] = []
    fields: dict[str, list[np.ndarray]] = {name: [] for name in FIELD_SIZES}
    for l_l in grid:
        for l_r in grid:
            samples.append((float(l_l), float(l_r)))
            flattened = flatten_fields(direct_model_data(variant, float(l_l), float(l_r)))
            for name in fields:
                fields[name].append(flattened[name])
    return np.asarray(samples, dtype=float), {name: np.asarray(values) for name, values in fields.items()}


def fit_fields(
    samples: np.ndarray,
    fields: dict[str, np.ndarray],
    split: float,
    fit_mode: str,
) -> dict[str, np.ndarray]:
    coefficients: dict[str, np.ndarray] = {}
    if fit_mode == "global":
        region_masks = [np.ones(len(samples), dtype=bool)]
    else:
        indices = np.array([region_index(float(l_l), float(l_r), split) for l_l, l_r in samples])
        region_masks = [indices == region for region in range(REGION_COUNT)]

    for field_name, values in fields.items():
        field_coeffs = np.zeros((len(region_masks), values.shape[1], BASIS_SIZE), dtype=float)
        for region, mask in enumerate(region_masks):
            if int(np.count_nonzero(mask)) < BASIS_SIZE:
                raise ValueError(f"{field_name} region {region} has fewer than {BASIS_SIZE} fit samples")
            design = poly22_basis(samples[mask, 0], samples[mask, 1])
            field_coeffs[region, :, :] = np.linalg.lstsq(design, values[mask, :], rcond=None)[0].T
        coefficients[field_name] = field_coeffs
    return coefficients


def evaluate_field(
    coefficients: np.ndarray,
    l_l: float,
    l_r: float,
    split: float,
) -> np.ndarray:
    region = 0 if coefficients.shape[0] == 1 else region_index(l_l, l_r, split)
    return coefficients[region] @ poly22_basis(l_l, l_r)


def fitted_model_data(
    coefficients: dict[str, np.ndarray],
    l_l: float,
    l_r: float,
    split: float,
) -> DirectModelData:
    mass = evaluate_field(coefficients["mass"], l_l, l_r, split).reshape(5, 5)
    gravity = evaluate_field(coefficients["gravity"], l_l, l_r, split).reshape(5, 5)
    input_map = evaluate_field(coefficients["input_map"], l_l, l_r, split).reshape(5, 4)
    disturbance_map = evaluate_field(coefficients["disturbance_map"], l_l, l_r, split).reshape(4, 5)
    return DirectModelData(
        mass=mass,
        gravity=gravity,
        input_map=input_map,
        strict_pinv=np.linalg.pinv(input_map),
        disturbance_map=disturbance_map,
    )


def boundary_jump(coefficients: np.ndarray, leg_min: float, leg_max: float, split: float) -> float:
    if coefficients.shape[0] == 1:
        return 0.0
    epsilon = max(1e-7, (leg_max - leg_min) * 1e-6)
    scan = np.linspace(leg_min, leg_max, 101)
    worst = 0.0
    for other in scan:
        left_below = evaluate_field(coefficients, split - epsilon, float(other), split)
        left_above = evaluate_field(coefficients, split + epsilon, float(other), split)
        right_below = evaluate_field(coefficients, float(other), split - epsilon, split)
        right_above = evaluate_field(coefficients, float(other), split + epsilon, split)
        worst = max(
            worst,
            float(np.max(np.abs(left_above - left_below))),
            float(np.max(np.abs(right_above - right_below))),
        )
    return worst


def validate_coordinate_transform(variant: str) -> None:
    """Check that the complete inverse transform reconstructs s and phi."""
    p = model_params(variant)
    state = np.array(
        [0.37, -0.42, 0.28, 0.63, -0.19, 0.54, 0.23, -0.47, 0.08, -0.31, 0.18, 0.27],
        dtype=float,
    )
    left_rate = 0.021
    right_rate = -0.017
    q, dq = state_to_generalized(variant, state, left_rate, right_rate)
    s_reconstructed = p.wheel_radius_m * (q[0] + q[1]) / 2.0
    phi_reconstructed = (
        -p.wheel_radius_m * q[0]
        + p.wheel_radius_m * q[1]
        - state[10] * q[2]
        + state[11] * q[3]
    ) / (2.0 * p.half_track_m)
    s_dot_reconstructed = p.wheel_radius_m * (dq[0] + dq[1]) / 2.0
    phi_dot_reconstructed = (
        -p.wheel_radius_m * dq[0]
        + p.wheel_radius_m * dq[1]
        - left_rate * q[2]
        - state[10] * dq[2]
        + right_rate * q[3]
        + state[11] * dq[3]
    ) / (2.0 * p.half_track_m)
    reconstructed = np.array([s_reconstructed, s_dot_reconstructed, phi_reconstructed, phi_dot_reconstructed])
    expected = state[[0, 1, 2, 3]]
    if not np.allclose(reconstructed, expected, atol=1e-11, rtol=1e-11):
        raise AssertionError(f"{variant} complete coordinate transform self-check failed: {reconstructed} != {expected}")


def synthetic_leso_test(
    direct: DirectModelData,
    dt_s: float = 0.002,
    duration_s: float = 4.0,
) -> tuple[float, float]:
    """Noise-free constant-disturbance test of the reference second-order LESO."""
    omega = np.array([5.0, 5.0, 5.0, 5.0, 10.0])
    beta1 = 2.0 * omega
    beta2 = omega * omega
    injected_motor_disturbance = np.array([0.55, -0.35, 0.72, 0.72])
    injected_generalized_disturbance = direct.input_map @ injected_motor_disturbance

    q = np.array([0.1, -0.15, 0.08, -0.04, 0.03])
    previous_final_virtual_command = np.array([0.2, -0.1, 0.15, 0.12])
    nominal_rate = direct.input_map @ previous_final_virtual_command - direct.gravity @ q
    p_measured = np.zeros(5)
    p_hat = p_measured.copy()
    d_hat = np.zeros(5)

    steps = int(round(duration_s / dt_s))
    for _ in range(steps):
        p_measured += (nominal_rate + injected_generalized_disturbance) * dt_s
        error = p_measured - p_hat
        d_hat += beta2 * error * dt_s
        p_hat += (nominal_rate + d_hat + beta1 * error) * dt_s

    estimated_motor_disturbance = direct.disturbance_map @ d_hat
    expected_motor_disturbance = direct.disturbance_map @ injected_generalized_disturbance
    generalized_error = float(np.max(np.abs(d_hat - injected_generalized_disturbance)))
    motor_error = float(np.max(np.abs(estimated_motor_disturbance - expected_motor_disturbance)))
    return generalized_error, motor_error


def validate_variant(
    variant: str,
    coefficients: dict[str, np.ndarray],
    leg_min: float,
    leg_max: float,
    validation_step: float,
    split: float,
) -> VariantResult:
    validate_coordinate_transform(variant)
    grid = make_grid(leg_min, leg_max, validation_step)
    error_samples: dict[str, list[np.ndarray]] = {name: [] for name in FIELD_SIZES}
    direct_samples: dict[str, list[np.ndarray]] = {name: [] for name in FIELD_SIZES}
    float32_errors: dict[str, list[np.ndarray]] = {name: [] for name in FIELD_SIZES}
    locations: list[tuple[float, float]] = []

    max_mass_condition = 0.0
    max_btau_condition = 0.0
    max_pinv_residual = 0.0
    max_pitch_common_error = 0.0

    coefficients_f32 = {name: values.astype(np.float32) for name, values in coefficients.items()}

    for l_l in grid:
        for l_r in grid:
            left = float(l_l)
            right = float(l_r)
            direct = direct_model_data(variant, left, right)
            fitted = fitted_model_data(coefficients, left, right, split)
            fitted_f32 = fitted_model_data(coefficients_f32, left, right, split)
            locations.append((left, right))

            direct_fields = flatten_fields(direct)
            fitted_fields = flatten_fields(fitted)
            fitted_f32_fields = flatten_fields(fitted_f32)
            for name in FIELD_SIZES:
                direct_samples[name].append(direct_fields[name])
                error_samples[name].append(fitted_fields[name] - direct_fields[name])
                float32_errors[name].append(fitted_f32_fields[name] - fitted_fields[name])

            max_mass_condition = max(max_mass_condition, float(np.linalg.cond(direct.mass)))
            max_btau_condition = max(max_btau_condition, float(np.linalg.cond(direct.input_map)))
            pinv_residual = direct.input_map @ direct.strict_pinv @ direct.input_map - direct.input_map
            max_pinv_residual = max(max_pinv_residual, float(np.max(np.abs(pinv_residual))))
            max_pitch_common_error = max(
                max_pitch_common_error,
                abs(float(direct.disturbance_map[2, 4] - direct.disturbance_map[3, 4])),
            )
            if np.linalg.matrix_rank(direct.input_map) != 4:
                raise AssertionError(f"{variant} B_tau loses rank at ({left}, {right})")

    metrics: dict[str, FieldMetrics] = {}
    location_array = np.asarray(locations)
    for name in FIELD_SIZES:
        direct_array = np.asarray(direct_samples[name])
        error_array = np.asarray(error_samples[name])
        f32_array = np.asarray(float32_errors[name])
        per_point_max = np.max(np.abs(error_array), axis=1)
        worst_index = int(np.argmax(per_point_max))
        direct_norm = np.linalg.norm(direct_array, axis=1)
        error_norm = np.linalg.norm(error_array, axis=1)
        relative_norm = error_norm / np.maximum(direct_norm, 1e-9)
        metrics[name] = FieldMetrics(
            max_abs=float(np.max(np.abs(error_array))),
            rms_abs=float(np.sqrt(np.mean(error_array * error_array))),
            max_fro_rel=float(np.max(relative_norm)),
            max_float32_abs=float(np.max(np.abs(f32_array))),
            worst_left_m=float(location_array[worst_index, 0]),
            worst_right_m=float(location_array[worst_index, 1]),
            boundary_jump=boundary_jump(coefficients[name], leg_min, leg_max, split),
        )

    mid = 0.5 * (leg_min + leg_max)
    synthetic_direct = fitted_model_data(coefficients, mid, mid, split)
    generalized_error, motor_error = synthetic_leso_test(synthetic_direct)
    return VariantResult(
        coefficients=coefficients,
        field_metrics=metrics,
        max_mass_condition=max_mass_condition,
        max_btau_condition=max_btau_condition,
        max_strict_pinv_identity_residual=max_pinv_residual,
        max_pitch_common_mode_error=max_pitch_common_error,
        synthetic_generalized_error=generalized_error,
        synthetic_motor_error=motor_error,
    )


def generate_variant(
    variant: str,
    leg_min: float,
    leg_max: float,
    grid_step: float,
    validation_step: float,
    split: float,
    fit_mode: str,
) -> VariantResult:
    data = leg_data(variant)
    if leg_min < data[0, 0] or leg_max > data[-1, 0]:
        print(
            f"warning: {variant} leg data range is {data[0, 0]:.3f}..{data[-1, 0]:.3f}; "
            "values outside it use linear extrapolation."
        )
    training_grid = make_grid(leg_min, leg_max, grid_step)
    samples, fields = collect_samples(variant, training_grid)
    coefficients = fit_fields(samples, fields, split, fit_mode)
    return validate_variant(variant, coefficients, leg_min, leg_max, validation_step, split)


def report_text(
    variant: str,
    result: VariantResult,
    leg_min: float,
    leg_max: float,
    grid_step: float,
    validation_step: float,
    split: float,
    fit_mode: str,
) -> str:
    lines = [
        f"variant                         : {variant}",
        f"fit mode                       : {fit_mode}",
        f"leg range                      : {leg_min:.4f} .. {leg_max:.4f} m",
        f"training / validation step     : {grid_step:.4f} / {validation_step:.4f} m",
        f"region split                   : {split:.4f} m",
        f"max cond(M)                    : {result.max_mass_condition:.6g}",
        f"max cond(B_tau)                : {result.max_btau_condition:.6g}",
        f"max pinv identity residual     : {result.max_strict_pinv_identity_residual:.6g}",
        f"max Pitch common-mode mismatch : {result.max_pitch_common_mode_error:.6g}",
        f"synthetic LESO generalized err : {result.synthetic_generalized_error:.6g}",
        f"synthetic LESO motor-map err   : {result.synthetic_motor_error:.6g}",
    ]
    for name, metrics in result.field_metrics.items():
        lines.append(
            f"{name:16s}: max={metrics.max_abs:.6g}, rms={metrics.rms_abs:.6g}, "
            f"max_rel={metrics.max_fro_rel:.6g}, f32={metrics.max_float32_abs:.6g}, "
            f"boundary={metrics.boundary_jump:.6g}, "
            f"worst=({metrics.worst_left_m:.4f},{metrics.worst_right_m:.4f})"
        )
    return "\n".join(lines)


def format_cpp_array(values: np.ndarray, name: str) -> str:
    def cpp_float(value: float) -> str:
        text = f"{float(value):.9g}"
        if "." not in text and "e" not in text.lower():
            text += ".0"
        return text + "f"

    flat = values.reshape(-1)
    lines = [f"inline constexpr std::array<float, {len(flat)}> {name}{{"]
    for start in range(0, len(flat), BASIS_SIZE):
        chunk = ", ".join(cpp_float(value) for value in flat[start : start + BASIS_SIZE])
        lines.append(f"    {chunk},")
    lines.append("};")
    return "\n".join(lines)


def format_cpp_header(
    results: dict[str, VariantResult],
    leg_min: float,
    leg_max: float,
    split: float,
    fit_mode: str,
    command: str,
) -> str:
    region_count = REGION_COUNT if fit_mode == "quadrant" else 1
    blocks = [
        "#pragma once",
        "",
        "#include <array>",
        "",
        "#ifndef WHEEL_LEGGED_ROBOT_VARIANT",
        "#define WHEEL_LEGGED_ROBOT_VARIANT 2",
        "#endif",
        "",
        "// Generated by generate_scheduled_leso.py.",
        f"// Command: {command}",
        "// Model: M*q_ddot + G*q = B_tau*u + d",
        "// q: theta_wl, theta_wr, theta_ll, theta_lr, theta_b",
        "// u: T_wl, T_wr, T_bl, T_br (model coordinates)",
        "// Runtime observer input u is the previous cycle's final virtual command.",
        "// Matrix elements and polynomial coefficients are row-major.",
        "// Quadrant regions: 0=low/low, 1=high/low, 2=low/high, 3=high/high.",
        "// Flat coefficient index: ((region * element_count) + element) * 6 + coefficient.",
        "// DisturbanceMap starts from pinv(B_tau), then makes the Pitch column",
        "// common-mode on T_bl/T_br.",
        "",
        "namespace wheel_legged::params::generated {",
        "",
        f"inline constexpr float kLesoModelLegMinM = {leg_min:.9g}f;",
        f"inline constexpr float kLesoModelLegMaxM = {leg_max:.9g}f;",
        f"inline constexpr float kLesoModelRegionSplitM = {split:.9g}f;",
        f"inline constexpr int kLesoModelRegionCount = {region_count};",
        "inline constexpr int kLesoModelPolynomialCoefficientCount = 6;",
        "inline constexpr int kLesoMElementCount = 25;",
        "inline constexpr int kLesoGElementCount = 25;",
        "inline constexpr int kLesoBTauElementCount = 20;",
        "inline constexpr int kLesoDisturbanceMapElementCount = 20;",
        "",
    ]

    for variant in VARIANTS:
        suffix = CPP_VARIANT_NAMES[variant]
        params = model_params(variant)
        blocks.append(f"// {variant} model parameters: {asdict(params)}")
        for field_name in FIELD_SIZES:
            cpp_name = f"kLeso{CPP_FIELD_NAMES[field_name]}{suffix}"
            blocks.append(format_cpp_array(results[variant].coefficients[field_name], cpp_name))
            blocks.append("")

    blocks.extend(
        [
            "#if WHEEL_LEGGED_ROBOT_VARIANT == 1",
            "inline constexpr const auto &kLesoM = kLesoMHero;",
            "inline constexpr const auto &kLesoG = kLesoGHero;",
            "inline constexpr const auto &kLesoBTau = kLesoBTauHero;",
            "inline constexpr const auto &kLesoDisturbanceMap = kLesoDisturbanceMapHero;",
            "#elif WHEEL_LEGGED_ROBOT_VARIANT == 2",
            "inline constexpr const auto &kLesoM = kLesoMInfantry3;",
            "inline constexpr const auto &kLesoG = kLesoGInfantry3;",
            "inline constexpr const auto &kLesoBTau = kLesoBTauInfantry3;",
            "inline constexpr const auto &kLesoDisturbanceMap = kLesoDisturbanceMapInfantry3;",
            "#elif WHEEL_LEGGED_ROBOT_VARIANT == 3",
            "inline constexpr const auto &kLesoM = kLesoMInfantry4;",
            "inline constexpr const auto &kLesoG = kLesoGInfantry4;",
            "inline constexpr const auto &kLesoBTau = kLesoBTauInfantry4;",
            "inline constexpr const auto &kLesoDisturbanceMap = kLesoDisturbanceMapInfantry4;",
            "#else",
            '#error "WHEEL_LEGGED_ROBOT_VARIANT must be 1, 2, or 3"',
            "#endif",
            "",
            "}  // namespace wheel_legged::params::generated",
            "",
        ]
    )
    return "\n".join(blocks)


def format_html_report(reports: dict[str, str], results: dict[str, VariantResult]) -> str:
    sections: list[str] = []
    for variant in VARIANTS:
        rows = []
        for field, metrics in results[variant].field_metrics.items():
            rows.append(
                "<tr>"
                f"<td>{html.escape(field)}</td>"
                f"<td>{metrics.max_abs:.6g}</td>"
                f"<td>{metrics.rms_abs:.6g}</td>"
                f"<td>{metrics.max_fro_rel:.6g}</td>"
                f"<td>{metrics.max_float32_abs:.6g}</td>"
                f"<td>{metrics.boundary_jump:.6g}</td>"
                f"<td>{metrics.worst_left_m:.4f}, {metrics.worst_right_m:.4f}</td>"
                "</tr>"
            )
        sections.append(
            f"<section><h2>{html.escape(variant)}</h2>"
            f"<pre>{html.escape(reports[variant])}</pre>"
            "<table><thead><tr><th>field</th><th>max abs</th><th>RMS</th>"
            "<th>max Fro rel</th><th>float32</th><th>boundary jump</th><th>worst L/R</th>"
            f"</tr></thead><tbody>{''.join(rows)}</tbody></table></section>"
        )
    return f"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>Momentum LESO scheduled-model report</title>
  <style>
    body {{ margin: 24px; font-family: Arial, "Microsoft YaHei", sans-serif; color: #172033; }}
    section {{ margin-bottom: 30px; }}
    pre {{ background: #172033; color: #e5edf7; padding: 14px; overflow-x: auto; }}
    table {{ border-collapse: collapse; width: 100%; }}
    th, td {{ border: 1px solid #d9e0ea; padding: 7px; text-align: right; }}
    th:first-child, td:first-child {{ text-align: left; }}
  </style>
</head>
<body>
  <h1>Momentum LESO scheduled-model report</h1>
  <p>Validation uses a denser grid than fitting and includes float32 replay,
     pseudoinverse checks, complete-coordinate-transform checks, and a
     noise-free constant-disturbance LESO convergence test.</p>
  {''.join(sections)}
</body>
</html>
"""


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--leg-min", type=float, default=0.11)
    parser.add_argument("--leg-max", type=float, default=0.34)
    parser.add_argument("--grid-step", type=float, default=0.01)
    parser.add_argument("--validation-step", type=float, default=0.005)
    parser.add_argument("--region-split", type=float)
    parser.add_argument("--fit-mode", choices=["global", "quadrant"], default="quadrant")
    parser.add_argument("--variant", choices=[*VARIANTS, "all"], default="all")
    parser.add_argument(
        "--output",
        nargs="?",
        const=DEFAULT_OUTPUT,
        type=Path,
        help="write the C++ header; omit PATH to use leso/generated/",
    )
    parser.add_argument(
        "--html-report",
        nargs="?",
        const=DEFAULT_HTML_REPORT,
        type=Path,
        help="write the HTML report; omit PATH to use leso/reports/",
    )
    args = parser.parse_args()

    split = args.region_split if args.region_split is not None else 0.5 * (args.leg_min + args.leg_max)
    if not args.leg_min < split < args.leg_max:
        raise ValueError("region split must lie strictly inside the fit range")

    selected = VARIANTS if args.variant == "all" else (args.variant,)
    results: dict[str, VariantResult] = {}
    reports: dict[str, str] = {}
    for variant in selected:
        result = generate_variant(
            variant,
            args.leg_min,
            args.leg_max,
            args.grid_step,
            args.validation_step,
            split,
            args.fit_mode,
        )
        results[variant] = result
        reports[variant] = report_text(
            variant,
            result,
            args.leg_min,
            args.leg_max,
            args.grid_step,
            args.validation_step,
            split,
            args.fit_mode,
        )
        print(reports[variant])
        print()

    if args.output:
        if selected != VARIANTS:
            raise ValueError("--output requires --variant all so the conditional C++ header is complete")
        command = (
            f"python generate_scheduled_leso.py --leg-min {args.leg_min:g} --leg-max {args.leg_max:g} "
            f"--grid-step {args.grid_step:g} --validation-step {args.validation_step:g} "
            f"--fit-mode {args.fit_mode} --output {args.output.as_posix()}"
        )
        cpp = format_cpp_header(results, args.leg_min, args.leg_max, split, args.fit_mode, command)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(cpp, encoding="utf-8")
        print(f"wrote: {args.output}")

    if args.html_report:
        if selected != VARIANTS:
            raise ValueError("--html-report currently requires --variant all")
        args.html_report.parent.mkdir(parents=True, exist_ok=True)
        args.html_report.write_text(format_html_report(reports, results), encoding="utf-8")
        print(f"wrote: {args.html_report}")


if __name__ == "__main__":
    main()
