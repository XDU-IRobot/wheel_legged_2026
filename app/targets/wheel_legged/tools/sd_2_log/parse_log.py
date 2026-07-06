#!/usr/bin/env python3
"""
parse_log.py — 解析 SdLogger 二进制日志，导出为 CSV

用法:
  python parse_log.py sd_out/20260706_133712/sd_raw_xxx.bin

  # -o 指定输出路径（默认自动生成在同目录下）
  python parse_log.py sd_out/.../sd_raw_xxx.bin -o result.csv
"""

import argparse
import csv
import struct
import sys
import time
from pathlib import Path
from typing import Any, Optional

# ── Constants ──────────────────────────────────────────────────────────────

MAGIC = 0x53444C47  # "SDLG"
SECTOR_SIZE = 512
HEADER_FIELD_DESC_OFFSET = 32
FIELD_DESC_SIZE = 18
FIELD_NAME_LEN = 16

FIELD_TYPE_MAP = {
    0: ("float32", "f"),
    1: ("uint32", "I"),
}

# ── Header parsing ─────────────────────────────────────────────────────────


def parse_header(data: bytes) -> dict[str, Any]:
    """Parse the 512-byte log header sector. Returns a dict with schema info."""
    if len(data) < SECTOR_SIZE:
        raise ValueError(f"File too small for header: {len(data)} bytes (need {SECTOR_SIZE})")

    magic = struct.unpack_from("<I", data, 0)[0]
    if magic != MAGIC:
        raise ValueError(f"Bad magic: 0x{magic:08X}, expected 0x{MAGIC:08X}")

    version = struct.unpack_from("<H", data, 4)[0]
    flags = struct.unpack_from("<H", data, 6)[0]
    record_size = struct.unpack_from("<I", data, 8)[0]
    total_records = struct.unpack_from("<I", data, 12)[0]
    max_records = struct.unpack_from("<I", data, 16)[0]
    decimation = struct.unpack_from("<I", data, 20)[0]
    num_fields = struct.unpack_from("<H", data, 24)[0]
    sample_rate_hz = struct.unpack_from("<H", data, 26)[0]

    graceful_stop = bool(flags & 0x01)

    # Parse field descriptors
    field_names = []
    field_types = []  # human-readable
    field_fmts = []   # struct.unpack chars
    for i in range(num_fields):
        off = HEADER_FIELD_DESC_OFFSET + i * FIELD_DESC_SIZE
        name_bytes = data[off : off + FIELD_NAME_LEN]
        name = name_bytes.split(b"\x00", 1)[0].decode("utf-8", errors="replace")
        field_names.append(name)

        type_byte = data[off + FIELD_NAME_LEN]
        type_name, type_fmt = FIELD_TYPE_MAP.get(type_byte, (f"unknown({type_byte})", "I"))
        field_types.append(type_name)
        field_fmts.append(type_fmt)

    return {
        "version": version,
        "flags": flags,
        "graceful_stop": graceful_stop,
        "record_size": record_size,
        "total_records": total_records,
        "max_records": max_records,
        "decimation": decimation,
        "sample_rate_hz": sample_rate_hz,
        "num_fields": num_fields,
        "field_names": field_names,
        "field_types": field_types,
        "field_fmts": field_fmts,
    }


# ── Record parsing ─────────────────────────────────────────────────────────


def parse_records(data: bytes, header: dict[str, Any]) -> list[dict[str, Any]]:
    """
    Parse data records from the payload (everything after the header sector).
    Records are packed at `record_size` intervals and never cross sector boundaries.
    """
    total = header["total_records"]
    record_size = header["record_size"]
    num_fields = header["num_fields"]
    field_names = header["field_names"]
    field_fmts = header["field_fmts"]

    # Data starts after header sector
    data_start = SECTOR_SIZE
    records = []

    byte_idx = data_start
    for _ in range(total):
        # Determine which sector we're in
        sector_start = (byte_idx // SECTOR_SIZE) * SECTOR_SIZE
        sector_end = sector_start + SECTOR_SIZE

        # If record would span a sector boundary, skip padding to next sector
        if byte_idx + record_size > sector_end:
            byte_idx = sector_end  # skip zeros to next sector

        if byte_idx + record_size > len(data):
            print(f"  [warn] truncated at record {len(records)}/{total} (file too short)")
            break

        # Parse timestamp (uint32 LE)
        tick_ms = struct.unpack_from("<I", data, byte_idx)[0]

        # Parse field values
        vals = {}
        off = byte_idx + 4
        for i in range(num_fields):
            val = struct.unpack_from(f"<{field_fmts[i]}", data, off)[0]
            vals[field_names[i]] = val
            off += 4

        records.append({"tick_ms": tick_ms, **vals})
        byte_idx += record_size

    return records


# ── CSV output ─────────────────────────────────────────────────────────────


def write_csv(records: list[dict[str, Any]], output_path: Path) -> None:
    """Write parsed records to a CSV file."""
    if not records:
        print("No records to write.")
        return

    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)

    print(f"CSV written: {output_path}  ({len(records)} rows)")


# ── Plotting ───────────────────────────────────────────────────────────────


def plot_records(
    records: list[dict[str, Any]],
    field_names: list[str],
    output_dir: Path,
    input_stem: str,
) -> None:
    """Generate PNG time-series plots using matplotlib (optional dependency)."""
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[warn] matplotlib not installed. Install with: pip install matplotlib")
        return

    if not records:
        print("No records to plot.")
        return

    time_s = [r["tick_ms"] / 1000.0 for r in records]

    # Auto-group fields by unit suffix for grouped subplots
    groups: dict[str, list[str]] = {}
    for name in field_names:
        # Group by suffix pattern: _m, _rad, _mps, _rad_s, _nm, _n, etc.
        suffix = name.rsplit("_", 1)[-1] if "_" in name else "other"
        if suffix in ("ll", "lr", "l", "r", "lf", "lb", "rf", "rb", "yaw", "b"):
            # side/position suffix — use parent group
            parent = name.rsplit("_", 1)[0] if "_" in name else name
            groups.setdefault(parent, []).append(name)
        else:
            groups.setdefault(suffix, []).append(name)

    n_groups = len(groups)
    fig, axes = plt.subplots(n_groups, 1, figsize=(12, 3 * n_groups), sharex=True, squeeze=False)
    fig.suptitle(f"SD Log: {input_stem}", fontsize=13)

    for ax_row, (group_name, group_fields) in zip(axes[:, 0], groups.items()):
        ax = ax_row
        for fn in group_fields:
            values = [r[fn] for r in records]
            ax.plot(time_s, values, label=fn, linewidth=0.8, alpha=0.85)
        ax.set_ylabel(group_name)
        ax.legend(loc="upper right", fontsize=7, ncol=2)
        ax.grid(True, alpha=0.3)

    axes[-1, 0].set_xlabel("Time (s)")

    plot_path = output_dir / f"{input_stem}_plot.png"
    fig.tight_layout()
    fig.savefig(plot_path, dpi=150)
    plt.close(fig)
    print(f"Plot saved: {plot_path}")


# ── Main ───────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Parse SdLogger binary log and export CSV / plots.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("input", type=str, help="Input .bin file path (raw sector dump)")
    parser.add_argument("-o", "--output", type=str, default=None, help="Output .csv file path (default: <input>_log.csv)")
    parser.add_argument("--plot", action="store_true", help="Generate matplotlib time-series plot")
    parser.add_argument("--plot-dir", type=str, default=None, help="Directory for plot output (default: same as output)")

    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.is_file():
        print(f"Error: file not found: {input_path}")
        sys.exit(1)

    print(f"Reading: {input_path}  ({input_path.stat().st_size:,} bytes)")

    with open(input_path, "rb") as f:
        data = f.read()

    # ── Parse header ──
    try:
        header = parse_header(data)
    except ValueError as e:
        print(f"Error parsing header: {e}")
        sys.exit(1)

    print(f"  Magic:      0x{0x53444C47:08X} ✓")
    print(f"  Version:    {header['version']}")
    print(f"  Fields:     {header['num_fields']} ({', '.join(header['field_names'])})")
    print(f"  Record sz:  {header['record_size']} B")
    print(f"  Records:    {header['total_records']} / {header['max_records']}")
    print(f"  Decimation: {header['decimation']} (→ {500 // header['decimation']} Hz)")
    print(f"  Stop:       {'graceful' if header['graceful_stop'] else 'abrupt (may have missing tail)'}")

    # ── Parse records ──
    records = parse_records(data, header)
    print(f"Parsed {len(records)} records.")

    if not records:
        print("No records found — empty log.")
        sys.exit(0)

    # Show preview
    print(f"\nPreview (first 3 records):")
    for r in records[:3]:
        ts = r["tick_ms"]
        fields_str = " ".join(f"{k}={r[k]:.4f}" for k in header["field_names"])
        print(f"  [{ts:8d} ms] {fields_str}")
    if len(records) > 3:
        print(f"  ... ({len(records) - 3} more)")

    # ── Output CSV ──
    output_csv = Path(args.output) if args.output else input_path.parent / f"{input_path.stem}_log.csv"
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    write_csv(records, output_csv)

    # ── Optional plot ──
    if args.plot:
        plot_dir = Path(args.plot_dir) if args.plot_dir else output_csv.parent
        plot_dir.mkdir(parents=True, exist_ok=True)
        plot_records(records, header["field_names"], plot_dir, input_path.stem)


if __name__ == "__main__":
    main()
