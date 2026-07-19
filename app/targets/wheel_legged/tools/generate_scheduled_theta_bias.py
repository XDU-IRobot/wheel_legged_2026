#!/usr/bin/env python3
"""
Generate leg swing angle bias scheduled by leg length.

Left leg bias uses left leg length, right leg bias uses right leg length.
Each variant has separate key points for Ll and Lr.

Usage:
  python generate_scheduled_theta_bias.py --output theta_bias_generated.hpp
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# EDIT THIS TABLE
# ---------------------------------------------------------------------------
# Key points: (leg_length_m, theta_bias_rad)

HERO_KEY_POINTS_LL = [
    (0.16, - 0.011),
    (0.18, - 0.013),
    (0.2, - 0.015),
    (0.22, - 0.018),
    (0.24, - 0.024),
    (0.26, - 0.03),
    (0.28, - 0.033),
    (0.3, - 0.038),
]
HERO_KEY_POINTS_LR = [
    (0.16, - 0.011),
    (0.18, - 0.013),
    (0.2, - 0.015),
    (0.22, - 0.018),
    (0.24, - 0.024),
    (0.26, - 0.03),
    (0.28, - 0.033),
    (0.3, - 0.038),
]

INFANTRY3_KEY_POINTS_LL = [
    (0.16, - 0.011),
    (0.18, - 0.013),
    (0.2, - 0.015),
    (0.22, - 0.018),
    (0.24, - 0.023),
    (0.26, - 0.028),
    (0.28, - 0.033),
    (0.3, - 0.038),
    (0.32, - 0.04),
    (0.34, - 0.045),
]
INFANTRY3_KEY_POINTS_LR = [
    (0.16, - 0.011),
    (0.18, - 0.013),
    (0.2, - 0.015),
    (0.22, - 0.018),
    (0.24, - 0.023),
    (0.26, - 0.028),
    (0.28, - 0.033),
    (0.3, - 0.038),
    (0.32, - 0.04),
    (0.34, - 0.045),
]

INFANTRY4_KEY_POINTS_LL = [
    (0.17, -0.0),
    (0.23, -0.1),
    (0.33, -0.15),
]
INFANTRY4_KEY_POINTS_LR = [
    (0.17, -0.0),
    (0.23, -0.1),
    (0.33, -0.15),
]

VARIANT_KEY_POINTS_LL = {
    "hero": HERO_KEY_POINTS_LL,
    "infantry3": INFANTRY3_KEY_POINTS_LL,
    "infantry4": INFANTRY4_KEY_POINTS_LL,
}
VARIANT_KEY_POINTS_LR = {
    "hero": HERO_KEY_POINTS_LR,
    "infantry3": INFANTRY3_KEY_POINTS_LR,
    "infantry4": INFANTRY4_KEY_POINTS_LR,
}

VARIANT_CPP_NAMES_LL = {
    "hero": "kThetaBiasLlHero",
    "infantry3": "kThetaBiasLlInfantry3",
    "infantry4": "kThetaBiasLlInfantry4",
}
VARIANT_CPP_NAMES_LR = {
    "hero": "kThetaBiasLrHero",
    "infantry3": "kThetaBiasLrInfantry3",
    "infantry4": "kThetaBiasLrInfantry4",
}


def interpolate_table(key_points: list[tuple[float, float]], num_points: int = 50) -> np.ndarray:
    """Linearly interpolate key points into a dense table."""
    lengths = np.array([p[0] for p in key_points])
    theta_bias = np.array([p[1] for p in key_points])

    t_min, t_max = lengths[0], lengths[-1]
    grid = np.linspace(t_min, t_max, num_points)

    bias_interp = np.interp(grid, lengths, theta_bias)

    result = np.column_stack([grid, bias_interp])
    return result


def format_cpp_array(table: np.ndarray, name: str) -> str:
    """Format as C++ constexpr array: {leg_length, theta_bias} per entry."""
    lines = [f"constexpr float {name}[{len(table)}][2] = {{"]
    for row in table:
        lines.append(f"    {{{row[0]:.6f}f, {row[1]:.8f}f}},")
    lines.append("};")
    return "\n".join(lines)


def format_cpp_header(variant_tables_ll: dict[str, np.ndarray], variant_tables_lr: dict[str, np.ndarray]) -> str:
    blocks = [
        "#pragma once",
        "",
        "#ifndef WHEEL_LEGGED_ROBOT_VARIANT",
        "#define WHEEL_LEGGED_ROBOT_VARIANT 2",
        "#endif",
        "",
        "namespace wheel_legged::params::generated {",
        "",
    ]
    for variant in ("hero", "infantry3", "infantry4"):
        blocks.append(format_cpp_array(variant_tables_ll[variant], VARIANT_CPP_NAMES_LL[variant]))
        blocks.append("")
        blocks.append(format_cpp_array(variant_tables_lr[variant], VARIANT_CPP_NAMES_LR[variant]))
        blocks.append("")

    blocks.extend([
        "#if WHEEL_LEGGED_ROBOT_VARIANT == 1",
        "static constexpr const auto &kThetaBiasLl = kThetaBiasLlHero;",
        "static constexpr const auto &kThetaBiasLr = kThetaBiasLrHero;",
        "#elif WHEEL_LEGGED_ROBOT_VARIANT == 2",
        "static constexpr const auto &kThetaBiasLl = kThetaBiasLlInfantry3;",
        "static constexpr const auto &kThetaBiasLr = kThetaBiasLrInfantry3;",
        "#elif WHEEL_LEGGED_ROBOT_VARIANT == 3",
        "static constexpr const auto &kThetaBiasLl = kThetaBiasLlInfantry4;",
        "static constexpr const auto &kThetaBiasLr = kThetaBiasLrInfantry4;",
        "#else",
        '#error "WHEEL_LEGGED_ROBOT_VARIANT must be 1, 2, or 3"',
        "#endif",
        "",
        "}  // namespace wheel_legged::params::generated",
    ])
    return "\n".join(blocks) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--num-points", type=int, default=50, help="number of interpolation points")
    parser.add_argument("--output", type=Path, help="output C++ header file path")
    args = parser.parse_args()

    variant_tables_ll: dict[str, np.ndarray] = {}
    variant_tables_lr: dict[str, np.ndarray] = {}
    for variant in VARIANT_KEY_POINTS_LL:
        variant_tables_ll[variant] = interpolate_table(VARIANT_KEY_POINTS_LL[variant], args.num_points)
        variant_tables_lr[variant] = interpolate_table(VARIANT_KEY_POINTS_LR[variant], args.num_points)

    cpp = format_cpp_header(variant_tables_ll, variant_tables_lr)
    print(cpp)

    if args.output:
        args.output.write_text(cpp + "\n", encoding="utf-8")
        print(f"\nwrote: {args.output}")


if __name__ == "__main__":
    main()
