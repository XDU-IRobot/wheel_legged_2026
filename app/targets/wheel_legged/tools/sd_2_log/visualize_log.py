#!/usr/bin/env python3
"""
visualize_log.py — SdLogger 交互式可视化（浏览器打开）

用法:
  python visualize_log.py sd_out/20260706_133712/sd_raw_xxx.bin --open

  # 只输出统计，不要图表
  python visualize_log.py sd_out/.../sd_raw_xxx.bin --no-ts --no-scatter

  # 只要时间序列
  python visualize_log.py sd_out/.../sd_raw_xxx.bin --no-scatter --no-stats

首次使用需安装依赖:
  pip install plotly pandas
"""

import argparse
import sys
from pathlib import Path

# ── Import parse_log's parsing functions ────────────────────────────────────
# Ensure parse_log.py is in the same directory or PYTHONPATH
try:
    import parse_log  # type: ignore
except ImportError:
    print("Error: parse_log.py must be in the same directory or PYTHONPATH")
    sys.exit(1)

MAGIC = parse_log.MAGIC  # 0x53444C47

# ── Plotly check ────────────────────────────────────────────────────────────
try:
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots
except ImportError:
    print("Error: plotly not installed. Run: pip install plotly")
    sys.exit(1)

try:
    import pandas as pd
except ImportError:
    print("Error: pandas not installed. Run: pip install pandas")
    sys.exit(1)


# ── Statistics ──────────────────────────────────────────────────────────────


def compute_stats(df: pd.DataFrame, field_names: list[str]) -> pd.DataFrame:
    """Compute per-field statistics."""
    rows = []
    for name in field_names:
        col = df[name]
        rows.append(
            {
                "Field": name,
                "Min": col.min(),
                "Max": col.max(),
                "Mean": col.mean(),
                "Std": col.std(),
                "Count": col.count(),
            }
        )
    return pd.DataFrame(rows)


def format_stats_table(stats_df: pd.DataFrame) -> list[str]:
    """Format statistics as a list of strings for display."""
    lines = []
    header = f"{'Field':<20} {'Min':>12} {'Max':>12} {'Mean':>12} {'Std':>12}"
    lines.append(header)
    lines.append("-" * len(header))
    for _, row in stats_df.iterrows():
        lines.append(
            f"{row['Field']:<20} {row['Min']:12.4f} {row['Max']:12.4f} "
            f"{row['Mean']:12.4f} {row['Std']:12.4f}"
        )
    return lines


# ── Data Cleaning ────────────────────────────────────────────────────────────


def clean_records(records: list[dict], header: dict) -> tuple[list[dict], int]:
    """Remove stale/garbage records by detecting tick_ms non-monotonicity.

    Old log data from previous runs may remain on the SD card after the
    current log's end.  These carry much larger tick_ms values (or wrap
    around), causing the time axis to jump to thousands of seconds.

    We scan tick_ms and trim everything after the first non-monotonic
    jump that exceeds 5× the expected record interval.
    """
    if len(records) < 2:
        return records, 0

    expected_interval_ms = header["decimation"] * 2  # 500 Hz → 2ms per cycle, ×decimation
    max_jump_ms = expected_interval_ms * 5  # 5× tolerance for scheduling jitter

    cut_idx = len(records)
    prev_tick = records[0]["tick_ms"]

    for i in range(1, len(records)):
        cur_tick = records[i]["tick_ms"]
        if cur_tick < prev_tick:
            # Time went backwards — stale data from a newer boot
            cut_idx = i
            break
        if cur_tick - prev_tick > max_jump_ms:
            # Too-large forward jump — gap between old and new sessions
            cut_idx = i
            break
        prev_tick = cur_tick

    trimmed = len(records) - cut_idx
    if trimmed > 0:
        last_good_tick = records[cut_idx - 1]["tick_ms"]
        first_bad_tick = records[cut_idx]["tick_ms"]
        print(f"\n  ⚠️  Trimmed {trimmed} stale records at index {cut_idx}")
        print(f"      Last good: tick_ms={last_good_tick}  →  First bad: tick_ms={first_bad_tick}")
        return records[:cut_idx], trimmed

    return records, 0


# ── Time Series Plot ────────────────────────────────────────────────────────


def build_timeseries(df: pd.DataFrame, field_names: list[str], title: str) -> go.Figure:
    """Build interactive time series subplots (one row per field, shared X)."""
    n = len(field_names)
    t0_ms = df["tick_ms"].iloc[0]
    time_s = (df["tick_ms"] - t0_ms) / 1000.0  # relative time from first record

    fig = make_subplots(
        rows=n,
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.03,
        subplot_titles=[f"{name} vs Time" for name in field_names],
    )

    colors = [
        "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
        "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf",
    ]

    for i, name in enumerate(field_names):
        color = colors[i % len(colors)]
        fig.add_trace(
            go.Scattergl(
                x=time_s,
                y=df[name],
                mode="lines",
                name=name,
                line=dict(color=color, width=1.2),
                hovertemplate=(f"{name}: %{{y:.6f}}<br>"
                               f"t+%{{x:.3f}}s<extra></extra>"),
            ),
            row=i + 1,
            col=1,
        )

    fig.update_xaxes(title_text="Time (s)", row=n, col=1, showgrid=True, gridwidth=0.5)
    fig.update_yaxes(showgrid=True, gridwidth=0.5)

    fig.update_layout(
        title=dict(text=title, x=0.5, font=dict(size=16)),
        height=220 * n,
        hovermode="x unified",
        margin=dict(l=80, r=30, t=60, b=40),
    )

    return fig


# ── Scatter Matrix ──────────────────────────────────────────────────────────


def build_scatter_matrix(df: pd.DataFrame, field_names: list[str]) -> go.Figure | None:
    """Build a scatter plot matrix (SPLOM) for pairwise variable relationships."""
    if len(field_names) < 2:
        return None

    import plotly.express as px

    # Normalize time for color mapping
    time_s = df["tick_ms"] / 1000.0
    df_plot = df[field_names].copy()
    df_plot["_time_s"] = time_s

    fig = px.scatter_matrix(
        df_plot,
        dimensions=field_names,
        color="_time_s",
        color_continuous_scale="Viridis",
        labels={name: name for name in field_names},
        title="Scatter Matrix (colored by time)",
    )

    fig.update_traces(diagonal_visible=False, marker=dict(size=3, opacity=0.5))
    fig.update_layout(
        height=280 * len(field_names),
        margin=dict(l=40, r=40, t=60, b=40),
    )

    return fig


# ── HTML Assembly ───────────────────────────────────────────────────────────


def build_html(
    ts_fig: go.Figure,
    scatter_fig: go.Figure | None,
    stats_lines: list[str],
    header_info: dict,
    include_ts: bool,
    include_scatter: bool,
    include_stats: bool,
    title: str,
) -> str:
    """Assemble all components into one standalone HTML page."""
    parts = [
        "<!DOCTYPE html>",
        '<html lang="zh-CN">',
        "<head>",
        '<meta charset="UTF-8">',
        '<meta name="viewport" content="width=device-width, initial-scale=1.0">',
        f"<title>{title}</title>",
        "<style>",
        "  body { font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif; "
        "max-width: 1400px; margin: 0 auto; padding: 20px; background: #f5f5f5; }",
        "  .header { background: white; border-radius: 8px; padding: 16px 24px; margin-bottom: 16px; "
        "box-shadow: 0 1px 3px rgba(0,0,0,0.1); }",
        "  .header h1 { margin: 0 0 8px 0; font-size: 20px; }",
        "  .header .meta { color: #666; font-size: 13px; display: flex; flex-wrap: wrap; gap: 16px; }",
        "  .header .meta span { background: #e8f0fe; padding: 2px 8px; border-radius: 4px; }",
        "  .section { background: white; border-radius: 8px; padding: 16px 24px; margin-bottom: 16px; "
        "box-shadow: 0 1px 3px rgba(0,0,0,0.1); }",
        "  .section h2 { margin: 0 0 12px 0; font-size: 16px; color: #333; border-bottom: 2px solid #1f77b4; "
        "padding-bottom: 6px; }",
        "  .stats-table { border-collapse: collapse; font-size: 13px; width: 100%; }",
        "  .stats-table th { background: #f0f0f0; padding: 6px 12px; text-align: right; "
        "border-bottom: 2px solid #ccc; }",
        "  .stats-table th:first-child { text-align: left; }",
        "  .stats-table td { padding: 4px 12px; text-align: right; border-bottom: 1px solid #eee; }",
        "  .stats-table td:first-child { text-align: left; font-weight: 500; }",
        "</style>",
        "</head>",
        "<body>",
        # ── Header block ──
        '<div class="header">',
        f"<h1>{title}</h1>",
        '<div class="meta">',
        f"<span>📄 Records: {header_info['total_records']:,}</span>",
        f"<span>📡 Rate: {500 // header_info['decimation']} Hz</span>",
        f"<span>⏱️ Duration: {header_info['total_records'] * header_info['decimation'] / 500:.1f}s</span>",
        f"<span>{'🟢 Graceful stop' if header_info['graceful_stop'] else '🔴 Abrupt stop'}</span>",
        "</div>",
        "</div>",
    ]

    # ── Stats section ──
    if include_stats:
        parts.append('<div class="section">')
        parts.append("<h2>📊 Statistics</h2>")
        parts.append('<table class="stats-table">')
        # Header row
        header_parts = stats_lines[0].split()
        parts.append(
            "<tr>" + "".join(f"<th>{h}</th>" for h in header_parts) + "</tr>"
        )
        # Data rows (skip divider line)
        for line in stats_lines[2:]:
            cols = [c.strip() for c in line.split() if c.strip()]
            parts.append(
                "<tr>" + "".join(f"<td>{c}</td>" for c in cols) + "</tr>"
            )
        parts.append("</table>")
        parts.append("</div>")

    # ── Time series section ──
    if include_ts:
        parts.append('<div class="section">')
        parts.append("<h2>📈 Time Series</h2>")
        # Write Plotly div without the full HTML wrapper
        ts_html = ts_fig.to_html(full_html=False, include_plotlyjs="cdn")
        parts.append(ts_html)
        parts.append("</div>")

    # ── Scatter matrix section ──
    if include_scatter and scatter_fig is not None:
        parts.append('<div class="section">')
        parts.append("<h2>🔵 Scatter Matrix</h2>")
        sc_html = scatter_fig.to_html(full_html=False, include_plotlyjs=False)
        parts.append(sc_html)
        parts.append("</div>")

    parts.append("</body></html>")

    return "\n".join(parts)


# ── Main ───────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate interactive HTML visualization for SdLogger binary logs.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("input", type=str, help="Input .bin file (raw sector dump)")
    parser.add_argument(
        "-o", "--output", type=str, default=None,
        help="Output .html file (default: <input>_viz.html)",
    )
    parser.add_argument(
        "--no-ts", action="store_true",
        help="Skip time series plots",
    )
    parser.add_argument(
        "--no-scatter", action="store_true",
        help="Skip scatter matrix",
    )
    parser.add_argument(
        "--no-stats", action="store_true",
        help="Skip statistics table",
    )
    parser.add_argument(
        "--open", action="store_true",
        help="Automatically open the HTML file in browser",
    )
    parser.add_argument(
        "--title", type=str, default=None,
        help="Custom title for the report page",
    )

    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.is_file():
        print(f"Error: file not found: {input_path}")
        sys.exit(1)

    print(f"Reading: {input_path}  ({input_path.stat().st_size:,} bytes)")

    with open(input_path, "rb") as f:
        data = f.read()

    # Parse
    try:
        header = parse_log.parse_header(data)
    except ValueError as e:
        print(f"Error parsing header: {e}")
        sys.exit(1)

    print(f"  Magic: 0x{MAGIC:08X} ✓")
    print(f"  Fields: {header['num_fields']} ({', '.join(header['field_names'])})")
    print(f"  Records: {header['total_records']:,}  |  "
          f"Duration: {header['total_records'] * header['decimation'] / 500:.1f}s  |  "
          f"Stop: {'graceful' if header['graceful_stop'] else 'abrupt'}")

    records = parse_log.parse_records(data, header)
    print(f"Parsed {len(records)} records.")

    # Clean stale records from previous sessions on the SD card
    records, trimmed = clean_records(records, header)

    if not records:
        print("No records to visualize.")
        sys.exit(0)

    # Build DataFrame
    df = pd.DataFrame(records)
    field_names = header["field_names"]
    title = args.title or f"SD Log: {input_path.stem}"

    include_ts = not args.no_ts
    include_scatter = not args.no_scatter
    include_stats = not args.no_stats

    # Build components
    stats_df = compute_stats(df, field_names)
    stats_lines = format_stats_table(stats_df)

    print(f"\n📊 Statistics:")
    for line in stats_lines:
        print(f"  {line}")

    ts_fig = build_timeseries(df, field_names, title) if include_ts else None
    scatter_fig = (
        build_scatter_matrix(df, field_names) if include_scatter else None
    )

    # Assemble HTML
    html = build_html(
        ts_fig=ts_fig,
        scatter_fig=scatter_fig,
        stats_lines=stats_lines,
        header_info=header,
        include_ts=include_ts and ts_fig is not None,
        include_scatter=include_scatter and scatter_fig is not None,
        include_stats=include_stats,
        title=title,
    )

    # Write output
    output_path = Path(args.output) if args.output else input_path.parent / f"{input_path.stem}_viz.html"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(html)

    print(f"\n✅ Report saved: {output_path}")

    # Open in browser
    if args.open:
        import webbrowser
        webbrowser.open(str(output_path.resolve()))


if __name__ == "__main__":
    main()
