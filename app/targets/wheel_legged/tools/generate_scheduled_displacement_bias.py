#!/usr/bin/env python3
"""
Generate expected displacement bias scheduled by leg length.

Unlike theta_bias, the displacement bias is a single scalar per robot
(passed to LQR in chassis.cc), so each variant has one key-point table
scheduled by leg length.

Usage:
  python generate_scheduled_displacement_bias.py --output displacement_bias_generated.hpp
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# EDIT THIS TABLE
# ---------------------------------------------------------------------------
# Key points: (leg_length_m, displacement_bias_m)

HERO_KEY_POINTS = [
    (0.16, 0.2),
    (0.3, 0.2),
]

INFANTRY3_KEY_POINTS = [
    (0.16, -0.3),
    (0.34, -0.3),
]

INFANTRY4_KEY_POINTS = [
    (0.17, -0.9),
    (0.23, -1.5),
    (0.32, -2.8),
]

VARIANT_KEY_POINTS = {
    "hero": HERO_KEY_POINTS,
    "infantry3": INFANTRY3_KEY_POINTS,
    "infantry4": INFANTRY4_KEY_POINTS,
}

VARIANT_CPP_NAMES = {
    "hero": "kDisplacementBiasHero",
    "infantry3": "kDisplacementBiasInfantry3",
    "infantry4": "kDisplacementBiasInfantry4",
}


def interpolate_table(key_points: list[tuple[float, float]], num_points: int = 50) -> np.ndarray:
    """Linearly interpolate key points into a dense table."""
    lengths = np.array([p[0] for p in key_points])
    bias = np.array([p[1] for p in key_points])

    t_min, t_max = lengths[0], lengths[-1]
    grid = np.linspace(t_min, t_max, num_points)

    bias_interp = np.interp(grid, lengths, bias)

    result = np.column_stack([grid, bias_interp])
    return result


def format_cpp_array(table: np.ndarray, name: str) -> str:
    """Format as C++ constexpr array: {leg_length, displacement_bias} per entry."""
    lines = [f"constexpr float {name}[{len(table)}][2] = {{"]
    for row in table:
        lines.append(f"    {{{row[0]:.6f}f, {row[1]:.8f}f}},")
    lines.append("};")
    return "\n".join(lines)


def format_cpp_header(variant_tables: dict[str, np.ndarray]) -> str:
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
        blocks.append(format_cpp_array(variant_tables[variant], VARIANT_CPP_NAMES[variant]))
        blocks.append("")

    blocks.extend([
        "#if WHEEL_LEGGED_ROBOT_VARIANT == 1",
        "static constexpr const auto &kDisplacementBias = kDisplacementBiasHero;",
        "#elif WHEEL_LEGGED_ROBOT_VARIANT == 2",
        "static constexpr const auto &kDisplacementBias = kDisplacementBiasInfantry3;",
        "#elif WHEEL_LEGGED_ROBOT_VARIANT == 3",
        "static constexpr const auto &kDisplacementBias = kDisplacementBiasInfantry4;",
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

    variant_tables: dict[str, np.ndarray] = {}
    for variant in VARIANT_KEY_POINTS:
        variant_tables[variant] = interpolate_table(VARIANT_KEY_POINTS[variant], args.num_points)

    cpp = format_cpp_header(variant_tables)
    print(cpp)

    if args.output:
        args.output.write_text(cpp + "\n", encoding="utf-8")
        print(f"\nwrote: {args.output}")


if __name__ == "__main__":
    main()