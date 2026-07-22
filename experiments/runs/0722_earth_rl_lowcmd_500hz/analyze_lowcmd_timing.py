#!/usr/bin/env python3
"""Summarize generated/published/applied LowCmd timing diagnostics."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from pathlib import Path


def read_rows(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def int_field(row: dict[str, str], name: str, default: int = 0) -> int:
    value = row.get(name, "")
    if value is None or value == "":
        return default
    try:
        return int(float(value))
    except ValueError:
        return default


def timing_summary(times_us: list[int]) -> dict[str, float | int | bool]:
    times = sorted(t for t in times_us if t > 0)
    if len(times) < 2:
        return {
            "count": len(times),
            "duration_us": 0,
            "median_dt_us": 0,
            "median_rate_hz": 0.0,
            "duration_rate_hz": 0.0,
            "duplicate_timestamp_ratio": 0.0,
            "max_gap_us": 0,
            "has_sustained_gt_10ms_gap": False,
        }

    deltas = [b - a for a, b in zip(times, times[1:])]
    positive = [d for d in deltas if d > 0]
    median_dt = statistics.median(positive) if positive else 0
    duration = times[-1] - times[0]
    duplicate_count = sum(1 for d in deltas if d == 0)
    max_gap = max(positive) if positive else 0
    return {
        "count": len(times),
        "duration_us": duration,
        "median_dt_us": median_dt,
        "median_rate_hz": 1000000.0 / median_dt if median_dt else 0.0,
        "duration_rate_hz": ((len(times) - 1) * 1000000.0 / duration) if duration else 0.0,
        "duplicate_timestamp_ratio": duplicate_count / max(1, len(deltas)),
        "max_gap_us": max_gap,
        "has_sustained_gt_10ms_gap": max_gap > 10000,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--publisher-csv", type=Path, required=True)
    parser.add_argument("--apply-csv", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    args = parser.parse_args()

    publisher_rows = [r for r in read_rows(args.publisher_csv) if r.get("event") == "LOWCMD"]
    apply_rows = [r for r in read_rows(args.apply_csv) if r.get("event") == "CMD_APPLY"]
    receive_rows = [r for r in read_rows(args.apply_csv) if r.get("event") == "CMD_RECEIVE"]

    published_times = [int_field(r, "lowcmd_sim_time_us") for r in publisher_rows]
    new_apply_rows = [
        r for r in apply_rows
        if int_field(r, "new_command") == 1 and int_field(r, "command_sequence") > 0
    ]
    applied_times = [int_field(r, "sim_time_us") for r in new_apply_rows]
    received_times = [int_field(r, "receive_sim_time_us") for r in receive_rows]

    applied_sequences = [int_field(r, "command_sequence") for r in new_apply_rows]
    sequence_jumps = [
        b - a for a, b in zip(applied_sequences, applied_sequences[1:]) if b - a > 1
    ]

    published = timing_summary(published_times)
    applied = timing_summary(applied_times)
    received = timing_summary(received_times)
    published_rate = float(published["median_rate_hz"])
    applied_rate = float(applied["median_rate_hz"])
    rate_diff_ratio = (
        abs(published_rate - applied_rate) / published_rate if published_rate > 0 else 0.0
    )

    metrics = {
        "publisher_csv": str(args.publisher_csv),
        "apply_csv": str(args.apply_csv),
        "published": published,
        "received": received,
        "applied_new": applied,
        "published_applied_rate_diff_ratio": rate_diff_ratio,
        "callback_burst_apply_sequence_jump_count": len(sequence_jumps),
        "callback_burst_max_apply_sequence_jump": max(sequence_jumps) if sequence_jumps else 0,
        "apply_rows_total": len(apply_rows),
        "receive_rows_total": len(receive_rows),
    }

    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(metrics, indent=2, sort_keys=True) + "\n")
    print(json.dumps(metrics, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
