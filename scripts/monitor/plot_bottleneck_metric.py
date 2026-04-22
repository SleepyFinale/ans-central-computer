#!/usr/bin/env python3
"""Plot a selected bottleneck metric over time from monitor CSV."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Tuple


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot one bottleneck metric over time")
    parser.add_argument("log_csv", type=Path, help="Path to monitor CSV log")
    parser.add_argument("--metric", help="Metric column to plot (default: cpu_usage_pct)")
    parser.add_argument("--output", type=Path, help="Output image path (default: scripts/monitor/logs/plot_<metric>_<timestamp>.png)")
    parser.add_argument("--list-metrics", action="store_true", help="List numeric metric columns and exit")
    parser.add_argument("--rolling-window", type=int, default=1, help="Rolling average window in samples (default: 1)")
    return parser.parse_args()


def resolve_log_path(path: Path) -> Path:
    if path.exists():
        return path
    if not path.is_absolute():
        script_path = Path(__file__).resolve()
        repo_root = script_path.parents[2]
        alt = repo_root.parent / "ans-turtlebot3/scripts/monitor/logs" / path.name
        if alt.exists():
            return alt
    raise FileNotFoundError(
        f"Log file not found: {path}. Try full path or "
        "../ans-turtlebot3/scripts/monitor/logs/<your_log>.csv"
    )


def read_rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        raise ValueError("CSV has no data rows.")
    return rows


def detect_numeric_columns(rows: List[Dict[str, str]]) -> List[str]:
    cols = list(rows[0].keys())
    numeric = []
    for c in cols:
        if c in {"timestamp_iso", "iface", "ssid", "ip4"}:
            continue
        ok = True
        for row in rows[:20]:
            raw = (row.get(c) or "").strip()
            if not raw:
                continue
            if raw.startswith("0x"):
                raw = str(int(raw, 16))
            try:
                float(raw)
            except ValueError:
                ok = False
                break
        if ok:
            numeric.append(c)
    return numeric


def parse_series(rows: List[Dict[str, str]], metric: str) -> Tuple[List[datetime], List[float]]:
    xs: List[datetime] = []
    ys: List[float] = []
    for row in rows:
        ts = row.get("timestamp_iso", "").strip()
        if not ts:
            continue
        try:
            x = datetime.fromisoformat(ts)
        except ValueError:
            continue
        raw = (row.get(metric) or "").strip()
        if not raw:
            continue
        if raw.startswith("0x"):
            y = float(int(raw, 16))
        else:
            try:
                y = float(raw)
            except ValueError:
                continue
        xs.append(x)
        ys.append(y)
    if not xs:
        raise ValueError(f"No valid numeric samples found for metric: {metric}")
    return xs, ys


def rolling_average(values: List[float], window: int) -> List[float]:
    if window <= 1:
        return values
    out: List[float] = []
    run = 0.0
    for idx, val in enumerate(values):
        run += val
        if idx >= window:
            run -= values[idx - window]
        denom = min(idx + 1, window)
        out.append(run / float(denom))
    return out


def main() -> None:
    args = parse_args()
    log_csv = resolve_log_path(args.log_csv)
    rows = read_rows(log_csv)
    numeric_cols = detect_numeric_columns(rows)

    if args.list_metrics:
        print("Numeric metrics:")
        for col in numeric_cols:
            print(f"- {col}")
        return

    metric = args.metric or "cpu_usage_pct"
    if metric not in numeric_cols:
        if args.metric is None and numeric_cols:
            metric = numeric_cols[0]
            print(f"--metric not provided; using: {metric}")
        else:
            raise SystemExit(
                f"Metric '{metric}' is not numeric/available. "
                f"Use --list-metrics to inspect valid metrics."
            )

    try:
        import matplotlib.dates as mdates
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit(
            "matplotlib is required for plotting. Install with: sudo apt install python3-matplotlib"
        ) from exc

    xs, ys = parse_series(rows, metric)
    ys_plot = rolling_average(ys, max(args.rolling_window, 1))

    if args.output:
        out_path = args.output
    else:
        out_path = Path("scripts/monitor/logs") / f"plot_{metric}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
    out_path.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(xs, ys_plot, linewidth=1.5)
    ax.set_title(f"{metric} over time")
    ax.set_xlabel("Time")
    ax.set_ylabel(metric)
    ax.grid(True, alpha=0.3)
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
    fig.autofmt_xdate()
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)

    print(f"Plot written to: {out_path}")


if __name__ == "__main__":
    main()
