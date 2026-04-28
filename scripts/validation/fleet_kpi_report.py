#!/usr/bin/env python3
"""
Compute fleet navigation stability KPIs from ROS console logs.

Usage:
  python3 scripts/validation/fleet_kpi_report.py /path/to/nav.log
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path


PATTERNS = {
    "transform_too_old": re.compile(r"Transform data too old"),
    "message_filter_drop": re.compile(r"Message Filter dropping message"),
    "control_loop_miss": re.compile(r"Control loop missed its desired rate"),
    "follow_path_abort": re.compile(r"\[follow_path\].*Aborting handle"),
    "goal_canceled": re.compile(r"Goal canceled"),
    "goal_succeeded": re.compile(r"Goal succeeded"),
}


def compute_counts(log_file: Path) -> tuple[int, dict[str, int]]:
    if not log_file.exists():
        raise SystemExit(f"log file not found: {log_file}")
    text = log_file.read_text(encoding="utf-8", errors="replace")
    lines = text.splitlines()
    total_lines = len(lines)
    counts: dict[str, int] = {}
    for key, pattern in PATTERNS.items():
        counts[key] = len(pattern.findall(text))
    return total_lines, counts


def print_report(title: str, log_file: Path, total_lines: int, counts: dict[str, int]) -> None:
    print(title)
    print(f"log_file: {log_file}")
    print(f"total_lines: {total_lines}")
    print("")
    for key, count in counts.items():
        per_1k = (count * 1000.0 / total_lines) if total_lines else 0.0
        print(f"{key}: {count} ({per_1k:.2f} / 1k lines)")
    print("")


def main() -> int:
    parser = argparse.ArgumentParser(description="Report fleet TF/Nav2 KPI counts from logs.")
    parser.add_argument("log_file", type=Path, help="Primary log file")
    parser.add_argument(
        "--compare-other",
        type=Path,
        default=None,
        help="Optional second log file to compare against primary",
    )
    args = parser.parse_args()

    total_lines, counts = compute_counts(args.log_file)
    print_report("Fleet KPI Report", args.log_file, total_lines, counts)

    if args.compare_other is None:
        return 0

    other_lines, other_counts = compute_counts(args.compare_other)
    print_report("Fleet KPI Comparison (other)", args.compare_other, other_lines, other_counts)
    print("Delta (other - primary)")
    for key in PATTERNS:
        delta = other_counts.get(key, 0) - counts.get(key, 0)
        print(f"{key}: {delta:+d}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
