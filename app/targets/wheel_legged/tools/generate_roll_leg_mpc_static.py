#!/usr/bin/env python3
"""Generate fixed-size data for the roll-leg MPC static solver.

The generated header intentionally mirrors the current RollLegMpc model and
TinyMPC cache precomputation path, so the static solver can be compared against
the dynamic TinyMPC implementation with the same problem data.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path


NX = 7
NU = 2

K_E_L = 0
K_D_L = 1
K_D = 2
K_D_D = 3
K_E_ROLL = 4
K_D_ROLL = 5
K_AY = 6

K_DF_LEFT = 0
K_DF_RIGHT = 1


@dataclass
class Config:
    dt_s: float = 0.01
    horizon: int = 15
    rho: float = 1.0

    body_mass_kg: float = 22.0
    leg_mass_kg: float = 2.3
    gravity_mps2: float = 9.81
    support_half_width_m: float = 0.2025
    com_height_m: float = 0.28
    roll_inertia_kg_m2: float = 0.0

    a_dL: float = -8.0
    b_sum: float = 0.0
    a_dD: float = -10.0
    b_D: float = 0.0
    a_Drho: float = 0.0
    a_rho: float = 0.0
    a_drho: float = -4.0
    b_lrho: float = 0.0
    b_rrho: float = 0.0
    b_ay: float = 0.0

    q_L: float = 200000.0
    q_dL: float = 50.0
    q_D: float = 300.0
    q_dD: float = 30.0
    q_roll: float = 5000.0
    q_droll: float = 50.0
    q_ay: float = 0.0
    r_left: float = 0.01
    r_right: float = 0.01


def zeros(rows: int, cols: int) -> list[list[float]]:
    return [[0.0 for _ in range(cols)] for _ in range(rows)]


def eye(n: int) -> list[list[float]]:
    out = zeros(n, n)
    for i in range(n):
        out[i][i] = 1.0
    return out


def transpose(a: list[list[float]]) -> list[list[float]]:
    return [list(row) for row in zip(*a)]


def matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    rows = len(a)
    mid = len(b)
    cols = len(b[0])
    out = zeros(rows, cols)
    for i in range(rows):
        for k in range(mid):
            aik = a[i][k]
            if aik == 0.0:
                continue
            for j in range(cols):
                out[i][j] += aik * b[k][j]
    return out


def matadd(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[a[i][j] + b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def matsub(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [[a[i][j] - b[i][j] for j in range(len(a[0]))] for i in range(len(a))]


def matscale(s: float, a: list[list[float]]) -> list[list[float]]:
    return [[s * v for v in row] for row in a]


def diag(values: list[float]) -> list[list[float]]:
    out = zeros(len(values), len(values))
    for i, value in enumerate(values):
        out[i][i] = value
    return out


def inv2(a: list[list[float]]) -> list[list[float]]:
    det = a[0][0] * a[1][1] - a[0][1] * a[1][0]
    if abs(det) < 1.0e-12:
        raise ValueError("2x2 matrix is singular")
    inv_det = 1.0 / det
    return [
        [a[1][1] * inv_det, -a[0][1] * inv_det],
        [-a[1][0] * inv_det, a[0][0] * inv_det],
    ]


def max_abs_diff(a: list[list[float]], b: list[list[float]]) -> float:
    out = 0.0
    for i in range(len(a)):
        for j in range(len(a[0])):
            out = max(out, abs(a[i][j] - b[i][j]))
    return out


def matvec(a: list[list[float]], x: list[float]) -> list[float]:
    return [sum(aij * xj for aij, xj in zip(row, x)) for row in a]


def apply_physical_model(config: Config) -> None:
    body_mass_kg = max(config.body_mass_kg, 1.0e-5)
    leg_mass_kg = max(config.leg_mass_kg, 1.0e-5)
    total_mass_kg = body_mass_kg + 2.0 * leg_mass_kg
    half_width_m = max(config.support_half_width_m, 0.05)
    model_height_m = max(config.com_height_m, 0.05)
    default_roll_inertia = total_mass_kg * half_width_m * half_width_m
    roll_inertia = config.roll_inertia_kg_m2 if config.roll_inertia_kg_m2 > 1.0e-5 else default_roll_inertia
    roll_inertia = max(roll_inertia, 0.1)

    config.com_height_m = model_height_m
    config.roll_inertia_kg_m2 = roll_inertia
    config.b_sum = 1.0 / total_mass_kg
    config.b_D = 1.0 / max(leg_mass_kg, 1.0)
    config.a_rho = total_mass_kg * config.gravity_mps2 * model_height_m / roll_inertia
    config.a_Drho = -config.a_rho / half_width_m
    config.b_lrho = -half_width_m / roll_inertia
    config.b_rrho = half_width_m / roll_inertia
    config.b_ay = total_mass_kg * model_height_m / roll_inertia


def build_continuous_a(config: Config) -> list[list[float]]:
    a = zeros(NX, NX)
    a[K_E_L][K_D_L] = 1.0
    a[K_D_L][K_D_L] = config.a_dL
    a[K_D][K_D_D] = 1.0
    a[K_D_D][K_D_D] = config.a_dD
    a[K_E_ROLL][K_D_ROLL] = 1.0
    a[K_D_ROLL][K_D] = config.a_Drho
    a[K_D_ROLL][K_E_ROLL] = config.a_rho
    a[K_D_ROLL][K_D_ROLL] = config.a_drho
    a[K_D_ROLL][K_AY] = config.b_ay
    return a


def build_continuous_b(config: Config) -> list[list[float]]:
    b = zeros(NX, NU)
    b[K_D_L][K_DF_LEFT] = config.b_sum
    b[K_D_L][K_DF_RIGHT] = config.b_sum
    b[K_D_D][K_DF_LEFT] = -config.b_D
    b[K_D_D][K_DF_RIGHT] = config.b_D
    b[K_D_ROLL][K_DF_LEFT] = config.b_lrho
    b[K_D_ROLL][K_DF_RIGHT] = config.b_rrho
    return b


def precompute(config: Config) -> dict[str, object]:
    apply_physical_model(config)
    a_cont = build_continuous_a(config)
    b_cont = build_continuous_b(config)
    a_dyn = matadd(eye(NX), matscale(config.dt_s, a_cont))
    b_dyn = matscale(config.dt_s, b_cont)
    f_dyn = [0.0] * NX

    q_diag = [config.q_L, config.q_dL, config.q_D, config.q_dD, config.q_roll, config.q_droll, config.q_ay]
    r_diag = [config.r_left, config.r_right]

    work_q = [q + config.rho for q in q_diag]
    work_r = [r + config.rho for r in r_diag]
    riccati_q = diag([q + config.rho for q in work_q])
    riccati_r = diag([r + config.rho for r in work_r])

    b_t = transpose(b_dyn)
    a_t = transpose(a_dyn)
    k_prev = zeros(NU, NX)
    p_prev = diag([config.rho for _ in range(NX)])
    k_inf = zeros(NU, NX)
    p_inf = zeros(NX, NX)
    riccati_iterations = 1000

    for i in range(1000):
        rbpb = matadd(riccati_r, matmul(matmul(b_t, p_prev), b_dyn))
        k_inf = matmul(matmul(inv2(rbpb), b_t), matmul(p_prev, a_dyn))
        p_inf = matadd(riccati_q, matmul(matmul(a_t, p_prev), matsub(a_dyn, matmul(b_dyn, k_inf))))
        riccati_iterations = i + 1
        if max_abs_diff(k_inf, k_prev) < 1.0e-5:
            break
        k_prev = k_inf
        p_prev = p_inf

    quu_inv = inv2(matadd(riccati_r, matmul(matmul(b_t, p_inf), b_dyn)))
    am_bk_t = transpose(matsub(a_dyn, matmul(b_dyn, k_inf)))
    p_f = matvec(p_inf, f_dyn)
    apf = matvec(matmul(am_bk_t, p_inf), f_dyn)
    bpf = matvec(b_t, p_f)

    return {
        "config": config,
        "nominal_leg_length_m": getattr(config, "nominal_leg_length_m", config.com_height_m),
        "a_dyn": a_dyn,
        "b_dyn": b_dyn,
        "f_dyn": f_dyn,
        "work_q": work_q,
        "work_r": work_r,
        "k_inf": k_inf,
        "p_inf": p_inf,
        "quu_inv": quu_inv,
        "am_bk_t": am_bk_t,
        "apf": apf,
        "bpf": bpf,
        "riccati_iterations": riccati_iterations,
    }


def format_float(value: float) -> str:
    if value == 0.0:
        value = 0.0
    text = f"{value:.9g}"
    if "e" not in text and "E" not in text and "." not in text:
        text += ".0"
    return f"{text}f"


def format_vector(values: list[float], indent: str = "    ") -> str:
    body = ", ".join(format_float(v) for v in values)
    return f"{indent}{{{body}}}"


def format_matrix(values: list[list[float]], indent: str = "    ") -> str:
    rows = [format_vector(row, indent + "  ") for row in values]
    return "{\n" + ",\n".join(rows) + "\n" + indent + "}"


def format_model(data: dict[str, object], indent: str = "    ") -> str:
    config = data["config"]
    assert isinstance(config, Config)
    inner = indent + "  "
    return f"""{indent}{{
{inner}.nominal_leg_length_m = {format_float(data["nominal_leg_length_m"])},
{inner}.model_com_height_m = {format_float(config.com_height_m)},
{inner}.model_roll_inertia_kg_m2 = {format_float(config.roll_inertia_kg_m2)},
{inner}.riccati_iterations = {data["riccati_iterations"]},
{inner}.adyn = {format_matrix(data["a_dyn"], inner)},
{inner}.bdyn = {format_matrix(data["b_dyn"], inner)},
{inner}.fd = {format_vector(data["f_dyn"], inner)},
{inner}.work_q = {format_vector(data["work_q"], inner)},
{inner}.work_r = {format_vector(data["work_r"], inner)},
{inner}.kinf = {format_matrix(data["k_inf"], inner)},
{inner}.pinf = {format_matrix(data["p_inf"], inner)},
{inner}.quu_inv = {format_matrix(data["quu_inv"], inner)},
{inner}.am_bk_t = {format_matrix(data["am_bk_t"], inner)},
{inner}.apf = {format_vector(data["apf"], inner)},
{inner}.bpf = {format_vector(data["bpf"], inner)},
{indent}}}"""


def write_header(models: list[dict[str, object]], output: Path) -> None:
    first_config = models[0]["config"]
    assert isinstance(first_config, Config)
    model_rows = ",\n".join(format_model(model, "    ") for model in models)
    text = f"""#pragma once

// Generated by app/targets/wheel_legged/tools/generate_roll_leg_mpc_static.py.
// Do not edit this file by hand.

namespace chassis::roll_leg_mpc_static_data {{

inline constexpr int kNumStates = {NX};
inline constexpr int kNumInputs = {NU};
inline constexpr int kHorizon = {first_config.horizon};
inline constexpr int kNumModels = {len(models)};
inline constexpr float kDtS = {format_float(first_config.dt_s)};
inline constexpr float kRho = {format_float(first_config.rho)};
inline constexpr float kLegLengthMinM = {format_float(models[0]["nominal_leg_length_m"])};
inline constexpr float kLegLengthMaxM = {format_float(models[-1]["nominal_leg_length_m"])};

struct ModelData {{
  float nominal_leg_length_m;
  float model_com_height_m;
  float model_roll_inertia_kg_m2;
  int riccati_iterations;
  float adyn[kNumStates][kNumStates];
  float bdyn[kNumStates][kNumInputs];
  float fd[kNumStates];
  float work_q[kNumStates];
  float work_r[kNumInputs];
  float kinf[kNumInputs][kNumStates];
  float pinf[kNumStates][kNumStates];
  float quu_inv[kNumInputs][kNumInputs];
  float am_bk_t[kNumStates][kNumStates];
  float apf[kNumStates];
  float bpf[kNumInputs];
}};

inline constexpr ModelData kModels[kNumModels] = {{
{model_rows}
}};

}}  // namespace chassis::roll_leg_mpc_static_data
"""
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8", newline="\n")


def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[4]
    default_output = repo_root / "app" / "targets" / "wheel_legged" / "include" / "chassis" / "roll_leg_mpc_static_data.hpp"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=default_output)
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--horizon", type=int, default=15)
    parser.add_argument("--rho", type=float, default=1.0)
    parser.add_argument("--body-mass", type=float, default=22.0)
    parser.add_argument("--leg-mass", type=float, default=2.3)
    parser.add_argument("--gravity", type=float, default=9.81)
    parser.add_argument("--support-half-width", type=float, default=0.2025)
    parser.add_argument("--leg-min", type=float, default=0.14)
    parser.add_argument("--leg-max", type=float, default=0.35)
    parser.add_argument("--leg-step", type=float, default=0.03)
    parser.add_argument("--wheel-radius", type=float, default=0.0575)
    parser.add_argument("--body-com-height-offset", type=float, default=0.0)
    parser.add_argument("--roll-inertia", type=float, default=0.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.horizon < 2:
        raise SystemExit("--horizon must be at least 2")
    if args.leg_step <= 0.0:
        raise SystemExit("--leg-step must be positive")
    if args.leg_max < args.leg_min:
        raise SystemExit("--leg-max must be greater than or equal to --leg-min")

    leg_points: list[float] = []
    leg = args.leg_min
    while leg < args.leg_max + 1.0e-6:
        leg_points.append(round(leg, 6))
        leg += args.leg_step
    if abs(leg_points[-1] - args.leg_max) > 1.0e-5:
        leg_points.append(args.leg_max)

    models: list[dict[str, object]] = []
    for leg in leg_points:
        com_height = leg + args.wheel_radius + args.body_com_height_offset
        config = Config(
            dt_s=args.dt,
            horizon=args.horizon,
            rho=args.rho,
            body_mass_kg=args.body_mass,
            leg_mass_kg=args.leg_mass,
            gravity_mps2=args.gravity,
            support_half_width_m=args.support_half_width,
            com_height_m=com_height,
            roll_inertia_kg_m2=args.roll_inertia,
        )
        setattr(config, "nominal_leg_length_m", leg)
        models.append(precompute(config))

    write_header(models, args.output)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
