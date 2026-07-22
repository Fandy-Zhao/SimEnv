#!/usr/bin/env python3
"""Analyze a Unitree TimingDiagnostics CSV (logs/unitree_timing.csv).

Outputs JSON with per-event counts, period statistics, jitter estimates,
duplicate rates, deadline-miss summaries, inference latency, and optional
RTF-vs-control-period correlation.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
import sys
from typing import Any, Dict, List, Optional, Sequence


EVENTS_OF_INTEREST = ("FSM", "ACTION", "POLICY_WAIT", "LOWCMD")
FSM_EVENT = "FSM"
ACTION_EVENT = "ACTION"
POLICY_WAIT_EVENT = "POLICY_WAIT"
LOWCMD_EVENT = "LOWCMD"


def _safe_float(raw: Optional[str], default: float = 0.0) -> float:
    if raw is None:
        return default
    try:
        return float(raw)
    except (TypeError, ValueError):
        return default


def _safe_int(raw: Optional[str], default: int = 0) -> int:
    if raw is None:
        return default
    try:
        return int(float(raw))
    except (TypeError, ValueError):
        return default


def load_rows(path: str) -> List[Dict[str, str]]:
    with open(path, newline="") as fh:
        reader = csv.DictReader(fh)
        return list(reader)


def p10(values: Sequence[float]) -> float:
    if not values:
        return float("nan")
    s = sorted(values)
    idx = max(0, min(len(s) - 1, int(math.ceil(len(s) * 0.1)) - 1))
    return s[idx]


def pearson_r(xs: Sequence[float], ys: Sequence[float]) -> Optional[float]:
    n = min(len(xs), len(ys))
    if n < 3:
        return None
    x = xs[:n]
    y = ys[:n]
    mx = statistics.mean(x)
    my = statistics.mean(y)
    num = sum((xi - mx) * (yi - my) for xi, yi in zip(x, y))
    den = math.sqrt(sum((xi - mx) ** 2 for xi in x) * sum((yi - my) ** 2 for yi in y))
    return num / den if den > 1e-30 else None


def _wall_periods_ns(wall_ns: Sequence[int]) -> List[float]:
    return [float(wall_ns[i] - wall_ns[i - 1]) for i in range(1, len(wall_ns))]


def _sim_periods_us(sim_us: Sequence[int]) -> List[float]:
    return [float(sim_us[i] - sim_us[i - 1]) for i in range(1, len(sim_us))]


def analyze(path: str) -> Dict[str, Any]:
    rows = load_rows(path)
    if not rows:
        return {"error": "empty_or_unreadable_csv", "path": path}

    # Partition by event
    by_event: Dict[str, List[Dict[str, str]]] = {e: [] for e in EVENTS_OF_INTEREST}
    for r in rows:
        event = (r.get("event") or "").strip()
        if event in by_event:
            by_event[event].append(r)

    counts = {e: len(by_event[e]) for e in EVENTS_OF_INTEREST}
    total = len(rows)

    def event_hz(event_rows: Sequence[Dict[str, str]], time_key: str, scale: float) -> Optional[float]:
        if len(event_rows) < 2:
            return None
        times = [_safe_float(r.get(time_key)) / scale for r in event_rows]
        duration = max(times[-1] - times[0], 0.0)
        if duration <= 0.0:
            return None
        return (len(times) - 1) / duration

    frequencies: Dict[str, Dict[str, Optional[float]]] = {}
    for event in EVENTS_OF_INTEREST:
        event_rows = by_event[event]
        frequencies[event] = {
            "sim_hz": event_hz(event_rows, "sim_time_us", 1000000.0),
            "wall_hz": event_hz(event_rows, "wall_time_ns", 1000000000.0),
        }

    # --- FSM accepted updates -------------------------------------------------
    fsm = by_event[FSM_EVENT]
    fsm_accepted = [r for r in fsm if _safe_int(r.get("control_update_accepted")) == 1]
    fsm_all_wall = [_safe_int(r.get("wall_time_ns")) for r in fsm]
    fsm_acc_wall = [_safe_int(r.get("wall_time_ns")) for r in fsm_accepted]
    fsm_acc_sim = [_safe_int(r.get("sim_time_us")) for r in fsm_accepted]
    fsm_all_sim = [_safe_int(r.get("sim_time_us")) for r in fsm]

    fsm_all_wp = _wall_periods_ns(fsm_all_wall) if len(fsm_all_wall) > 1 else []
    fsm_acc_wp = _wall_periods_ns(fsm_acc_wall) if len(fsm_acc_wall) > 1 else []
    fsm_acc_sp = _sim_periods_us(fsm_acc_sim) if len(fsm_acc_sim) > 1 else []

    target_control_hz = _safe_float(fsm_accepted[0].get("target_control_hz")) if fsm_accepted else None

    sim_period_us = {
        "median": statistics.median(fsm_acc_sp) if fsm_acc_sp else None,
        "mean": statistics.mean(fsm_acc_sp) if fsm_acc_sp else None,
        "p10": p10(fsm_acc_sp) if fsm_acc_sp else None,
        "min": min(fsm_acc_sp) if fsm_acc_sp else None,
    }
    wall_period_ns = {
        "median": statistics.median(fsm_acc_wp) if fsm_acc_wp else None,
        "mean": statistics.mean(fsm_acc_wp) if fsm_acc_wp else None,
    }
    # Jitter: stdev of wall period (ns)
    jitter_ns = statistics.stdev(fsm_acc_wp) if len(fsm_acc_wp) > 1 else None

    # --- Duplicate / repeated-state counts -----------------------------------
    repeated_consumed = sum(1 for r in fsm if _safe_int(r.get("repeated_state_consumed")) == 1)
    new_consumed = sum(1 for r in fsm if _safe_int(r.get("new_state_consumed")) == 1)
    rejected = sum(_safe_int(r.get("repeated_state_rejected_count")) for r in fsm)
    dup_total = repeated_consumed + rejected

    # History duplicate ratio (from policy wait rows, where these fields are populated)
    policy_waits = by_event[POLICY_WAIT_EVENT]
    latest_dup = 0
    policy_samples = 0
    for r in reversed(policy_waits):
        span = _safe_int(r.get("history_span_us"))
        if span > 0:
            latest_dup = _safe_int(r.get("history_duplicate_count"))
            policy_samples = _safe_int(r.get("policy_sequence"))
            break
    history_duplicate_ratio = latest_dup / max(policy_samples, 1)

    # --- Deadline misses ------------------------------------------------------
    missed_all = [_safe_int(r.get("missed_periods")) for r in fsm]
    deadline_misses = {
        "sum": sum(missed_all),
        "max": max(missed_all) if missed_all else 0,
    }

    # --- Inference latency: ACTION wall_time_ns - policy_wall_time_ns ---------
    actions = by_event[ACTION_EVENT]
    action_latencies: List[float] = []
    if actions:
        for a in actions:
            lat = _safe_int(a.get("wall_time_ns")) - _safe_int(a.get("policy_wall_time_ns"))
            if lat > 0:
                action_latencies.append(float(lat))
    inference_latency_ns: Dict[str, Optional[float]] = {
        "samples": len(action_latencies),
        "median": statistics.median(action_latencies) if action_latencies else None,
        "mean": statistics.mean(action_latencies) if action_latencies else None,
    }

    # --- RTF / control-period correlation -------------------------------------
    rtf_values: List[float] = []
    control_periods: List[float] = []
    for prev, cur in zip(fsm_accepted, fsm_accepted[1:]):
        rtf = _safe_float(cur.get("estimated_rtf"))
        period = _safe_float(cur.get("sim_time_us")) - _safe_float(prev.get("sim_time_us"))
        if rtf > 0 and period > 0:
            rtf_values.append(rtf)
            control_periods.append(period)
    rtf_correlation: Optional[Dict[str, Optional[float]]] = None
    if len(rtf_values) >= 3 and len(control_periods) >= 3:
        r = pearson_r(rtf_values, control_periods)
        rtf_correlation = {
            "pearson_r": r,
            "rtf_samples": len(rtf_values),
            "note": "estimated_rtf vs accepted FSM simulation-time period",
        }

    return {
        "source": os.path.basename(path),
        "total_rows": total,
        "counts": counts,
        "frequencies": frequencies,
        "fsm_accepted_count": len(fsm_accepted),
        "target_control_hz": target_control_hz,
        "sim_period_us": sim_period_us,
        "wall_period_ns": wall_period_ns,
        "jitter_wall_period_stdev_ns": jitter_ns,
        "repeated_state_consumed": repeated_consumed,
        "new_state_consumed": new_consumed,
        "repeated_state_rejected_total": rejected,
        "duplicate_total": dup_total,
        "history_duplicate_ratio": round(history_duplicate_ratio, 6),
        "deadline_misses": deadline_misses,
        "inference_latency_ns": inference_latency_ns,
        "rtf_control_period_correlation": rtf_correlation,
    }


def main() -> None:
    p = argparse.ArgumentParser(description="Analyze a Unitree TimingDiagnostics CSV")
    p.add_argument("input", help="Path to unitree_timing.csv")
    p.add_argument("--output", "-o", default="-", help="JSON output path (default stdout)")
    args = p.parse_args()

    if not os.path.isfile(args.input):
        print(f"ERROR: input file not found: {args.input}", file=sys.stderr)
        sys.exit(1)

    result = analyze(args.input)
    text = json.dumps(result, indent=2, default=str)

    if args.output == "-":
        print(text)
    else:
        os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
        with open(args.output, "w") as fh:
            fh.write(text)
        print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
