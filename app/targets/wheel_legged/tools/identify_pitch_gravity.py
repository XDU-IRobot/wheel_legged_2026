#!/usr/bin/env python3
"""
Identify pitch gravity parameters theta_3 (m2*l2x) and theta_4 (m2*l2z)
from static identification CSV data.

CSV format: time_ms, yaw_tau, yaw_pos, yaw_vel, pitch_tau, pitch_pos, pitch_vel, pitch_static

At each static point (pitch_static=1), the pitch dynamics simplifies to:
  tau_pitch = theta_3 * gz * cos(q2) + theta_4 * gz * sin(q2)

Usage:
  python identify_pitch_gravity.py data.csv
  python identify_pitch_gravity.py data.csv --plot
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np


GZ = -9.81


def load_csv(path: Path) -> list[dict]:
    rows = []
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                "time_ms": float(row["time_ms"]),
                "yaw_tau": float(row["yaw_tau"]),
                "yaw_pos": float(row["yaw_pos"]),
                "yaw_vel": float(row["yaw_vel"]),
                "pitch_tau": float(row["pitch_tau"]),
                "pitch_pos": float(row["pitch_pos"]),
                "pitch_vel": float(row["pitch_vel"]),
                "pitch_static": int(row["pitch_static"]),
            })
    return rows


def extract_static_segments(rows: list[dict]) -> list[tuple[float, float]]:
    """Extract (q2_mean, tau_mean) pairs from static segments."""
    segments = []
    current_q2 = []
    current_tau = []

    for row in rows:
        if row["pitch_static"] == 1:
            current_q2.append(row["pitch_pos"])
            current_tau.append(row["pitch_tau"])
        else:
            if len(current_q2) > 10:
                q2_mean = np.mean(current_q2)
                tau_mean = np.mean(current_tau)
                segments.append((q2_mean, tau_mean))
            current_q2 = []
            current_tau = []

    if len(current_q2) > 10:
        q2_mean = np.mean(current_q2)
        tau_mean = np.mean(current_tau)
        segments.append((q2_mean, tau_mean))

    return segments


def identify(segments: list[tuple[float, float]]) -> tuple[float, float, float]:
    """Least squares: tau = theta_3 * gz * cos(q2) + theta_4 * gz * sin(q2)."""
    n = len(segments)
    Y = np.zeros((n, 2))
    tau = np.zeros(n)

    for i, (q2, t) in enumerate(segments):
        Y[i, 0] = GZ * np.cos(q2)
        Y[i, 1] = GZ * np.sin(q2)
        tau[i] = t

    result, residuals, rank, sv = np.linalg.lstsq(Y, tau, rcond=None)
    theta3, theta4 = result

    tau_pred = Y @ result
    max_error = float(np.max(np.abs(tau_pred - tau)))
    rmse = float(np.sqrt(np.mean((tau_pred - tau) ** 2)))

    return theta3, theta4, rmse


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", type=Path, help="CSV data file from serial capture")
    parser.add_argument("--plot", action="store_true", help="show fit plot")
    args = parser.parse_args()

    rows = load_csv(args.csv_file)
    print(f"loaded {len(rows)} rows")

    segments = extract_static_segments(rows)
    print(f"extracted {len(segments)} static segments:")
    for i, (q2, tau) in enumerate(segments):
        print(f"  point {i}: q2={np.degrees(q2):.1f}°, tau={tau:.4f} Nm")

    if len(segments) < 2:
        print("error: need at least 2 static points")
        sys.exit(1)

    theta3, theta4, rmse = identify(segments)
    print(f"\nidentified parameters:")
    print(f"  theta_3 (m2*l2x) = {theta3:.6f}")
    print(f"  theta_4 (m2*l2z) = {theta4:.6f}")
    print(f"  fit RMSE = {rmse:.6f} Nm")

    if args.plot:
        try:
            import matplotlib.pyplot as plt

            q2_range = np.linspace(min(s[0] for s in segments) - 0.05,
                                   max(s[0] for s in segments) + 0.05, 100)
            tau_fit = theta3 * GZ * np.cos(q2_range) + theta4 * GZ * np.sin(q2_range)

            plt.figure(figsize=(8, 5))
            plt.plot(np.degrees(q2_range), tau_fit, "b-", label="fitted curve")
            plt.plot([np.degrees(s[0]) for s in segments],
                     [s[1] for s in segments], "ro", markersize=8, label="static data points")
            plt.xlabel("pitch angle [deg]")
            plt.ylabel("torque [Nm]")
            plt.title(f"Pitch Gravity Identification (RMSE={rmse:.4f} Nm)")
            plt.legend()
            plt.grid(True)
            plt.tight_layout()
            plt.show()
        except ImportError:
            print("matplotlib not available, skipping plot")


if __name__ == "__main__":
    main()
