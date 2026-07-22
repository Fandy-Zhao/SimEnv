#!/usr/bin/env python3
"""Combo timing analysis: FSM + LOWCMD + LOWCMD_TRACE stages T1–T4 across two windows."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from pathlib import Path


def _int(row: dict[str, str], name: str, default: int = 0) -> int:
    v = row.get(name, "")
    if v is None or v == "":
        return default
    try:
        return int(float(v))
    except (ValueError, TypeError):
        return default


def _summary(times_us: list[int]) -> dict:
    times = sorted(t for t in times_us if t > 0)
    if len(times) < 2:
        return {"count": len(times), "duration_us": 0, "median_dt_us": 0,
                "median_rate_hz": 0.0, "duration_rate_hz": 0.0, "max_gap_us": 0}
    deltas = [b - a for a, b in zip(times, times[1:])]
    pos = [d for d in deltas if d > 0]
    med = statistics.median(pos) if pos else 0
    dur = times[-1] - times[0]
    return {"count": len(times), "duration_us": dur, "median_dt_us": med,
            "median_rate_hz": 1_000_000.0 / med if med else 0.0,
            "duration_rate_hz": ((len(times) - 1) * 1_000_000.0 / dur) if dur else 0.0,
            "max_gap_us": max(pos) if pos else 0}


def _seq_jumps(seqs: list[int]) -> dict:
    jumps = [b - a for a, b in zip(seqs, seqs[1:]) if b - a > 1]
    return {"jump_count": len(jumps), "max_jump": max(jumps) if jumps else 0}


def _rate_diff(a: dict, b: dict) -> float:
    ar = float(a["median_rate_hz"])
    return abs(ar - float(b["median_rate_hz"])) / ar if ar > 0 else 0.0


def _stage_metrics(rows: list[dict]) -> dict:
    """Compute per-stage metrics from a list of row dicts.

    Each row dict must have: sim_time_us, command_sequence, new_command, effective_application.
    """
    if not rows:
        return {
            "count": 0, "unique_sequence_count": 0, "duplicate_sequence_count": 0,
            "skipped_sequence_count": 0, "out_of_order_count": 0,
            "median_rate_hz": 0.0, "mean_rate_hz": 0.0, "p10_rate_hz": 0.0,
            "period_median_us": 0.0, "period_p90_us": 0.0, "period_p99_us": 0.0,
            "max_gap_us": 0, "duration_rate_hz": 0.0,
        }

    sorted_rows = sorted(rows, key=lambda r: r["sim_time_us"])
    times = [r["sim_time_us"] for r in sorted_rows]
    seqs = [r["command_sequence"] for r in sorted_rows]

    count = len(sorted_rows)
    positive_seqs = [s for s in seqs if s > 0]
    unique_seqs = len(set(positive_seqs))
    duplicate_count = len(positive_seqs) - unique_seqs

    skipped = 0
    out_of_order = 0
    prev_seq = None
    for s in seqs:
        if s <= 0:
            continue
        if prev_seq is not None:
            if s < prev_seq:
                out_of_order += 1
            elif s - prev_seq > 1:
                skipped += s - prev_seq - 1
        prev_seq = s

    deltas = [b - a for a, b in zip(times, times[1:])]
    pos_deltas = [d for d in deltas if d > 0]

    if pos_deltas:
        rates = [1_000_000.0 / d for d in pos_deltas]
        s_rates = sorted(rates)
        s_deltas = sorted(pos_deltas)
        n = len(s_rates)
        median_rate = statistics.median(s_rates)
        mean_rate = statistics.mean(s_rates)
        p10_rate = s_rates[max(0, int(n * 0.10))]
        period_median = statistics.median(s_deltas)
        period_p90 = s_deltas[min(n - 1, int(n * 0.90))]
        period_p99 = s_deltas[min(n - 1, int(n * 0.99))]
        max_gap = max(pos_deltas)
        dur = times[-1] - times[0]
        duration_rate = (count - 1) * 1_000_000.0 / dur if dur > 0 else 0.0
    else:
        median_rate = mean_rate = p10_rate = 0.0
        period_median = period_p90 = period_p99 = 0.0
        max_gap = 0
        duration_rate = 0.0

    return {
        "count": count,
        "unique_sequence_count": unique_seqs,
        "duplicate_sequence_count": duplicate_count,
        "skipped_sequence_count": skipped,
        "out_of_order_count": out_of_order,
        "median_rate_hz": round(median_rate, 2),
        "mean_rate_hz": round(mean_rate, 2),
        "p10_rate_hz": round(p10_rate, 2),
        "period_median_us": round(period_median, 1),
        "period_p90_us": round(period_p90, 1),
        "period_p99_us": round(period_p99, 1),
        "max_gap_us": max_gap,
        "duration_rate_hz": round(duration_rate, 2),
    }


_STAGE_MAP = {
    "T1_CALLBACK_ENTRY": "lowcmd_receive_500hz",
    "T2_BUFFER_WRITE": "buffer",
    "T3_CONTROLLER_READ": "gazebo_controller",
    "T4_JOINT_APPLY": "joint_application",
}


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--timing-csv", type=Path, required=True)
    p.add_argument("--apply-csv", type=Path, required=True)
    p.add_argument("--transition-summary", type=Path, required=True)
    p.add_argument("--output-json", type=Path, required=True)
    args = p.parse_args()

    ts = json.loads(args.transition_summary.read_text())
    switch_us = int(ts["switch_sim_time"] * 1_000_000)

    windows = {
        "fixed_last5": (switch_us - 5_000_000, switch_us),
        "rl_zero_8p5": (switch_us, switch_us + 8_500_000),
    }

    # Collect from timing CSV (streaming) — FSM and T0 LOWCMD
    win_data = {w: {"fsm": [], "lowcmd": []} for w in windows}
    if args.timing_csv.exists():
        with args.timing_csv.open(newline="") as f:
            for row in csv.DictReader(f):
                ev = row.get("event", "")
                sim_us = _int(row, "sim_time_us")
                for wname, (lo, hi) in windows.items():
                    if not (lo <= sim_us < hi):
                        continue
                    if ev == "FSM" and _int(row, "control_update_accepted") == 1:
                        win_data[wname]["fsm"].append(sim_us)
                    elif ev == "LOWCMD":
                        lsu = _int(row, "lowcmd_sim_time_us")
                        if lsu > 0:
                            win_data[wname]["lowcmd"].append(lsu)

    # Collect from apply CSV (streaming) — old CMD_RECEIVE/CMD_APPLY + new LOWCMD_TRACE
    win_apply = {w: {"recv": [], "apply": [], "seqs": []} for w in windows}
    win_trace = {w: {sn: [] for sn in _STAGE_MAP} for w in windows}
    if args.apply_csv.exists():
        with args.apply_csv.open(newline="") as f:
            for row in csv.DictReader(f):
                ev = row.get("event", "")
                sim_us = _int(row, "sim_time_us")
                for wname, (lo, hi) in windows.items():
                    if not (lo <= sim_us < hi):
                        continue
                    if ev == "CMD_RECEIVE":
                        rsu = _int(row, "receive_sim_time_us")
                        if rsu > 0:
                            win_apply[wname]["recv"].append(rsu)
                    elif ev == "CMD_APPLY" and _int(row, "new_command") == 1:
                        cs = _int(row, "command_sequence")
                        if cs > 0:
                            win_apply[wname]["apply"].append(sim_us)
                            win_apply[wname]["seqs"].append(cs)
                    elif ev == "LOWCMD_TRACE":
                        stage = row.get("stage", "")
                        if stage in _STAGE_MAP:
                            win_trace[wname][stage].append({
                                "sim_time_us": sim_us,
                                "command_sequence": _int(row, "command_sequence"),
                                "new_command": _int(row, "new_command"),
                                "effective_application": _int(row, "effective_application"),
                            })

    out = {"switch_sim_time_s": ts["switch_sim_time"]}
    for wname in windows:
        d = win_data[wname]
        a = win_apply[wname]
        fsm_s = _summary(d["fsm"])
        low_s = _summary(d["lowcmd"])
        recv_s = _summary(a["recv"])
        appl_s = _summary(a["apply"])
        jumps = _seq_jumps(a["seqs"])

        window_out: dict = {
            "window_us": list(windows[wname]),
            "fsm_accepted": fsm_s,
            "lowcmd_published": low_s,
        }

        # Old-style apply metrics (backward compat, present only if data exists)
        if recv_s["count"] > 0:
            window_out["cmd_receive"] = recv_s
        if appl_s["count"] > 0:
            window_out["cmd_apply_new"] = appl_s
            window_out["cmd_apply_sequence_jumps"] = jumps

        # New LOWCMD_TRACE stage metrics
        for stage_key, out_key in _STAGE_MAP.items():
            rows = win_trace[wname][stage_key]
            if not rows:
                continue
            if stage_key == "T4_JOINT_APPLY":
                eff_rows = [r for r in rows if r["effective_application"] == 1]
                sm = _stage_metrics(eff_rows)
                sm["new_payload_count"] = sum(1 for r in eff_rows if r["new_command"] == 1)
            else:
                sm = _stage_metrics(rows)
                if stage_key == "T3_CONTROLLER_READ":
                    sm["new_payload_count"] = sum(1 for r in rows if r["new_command"] == 1)
            window_out[out_key] = sm

        # Rate diff: T0 LOWCMD vs T4 joint_application
        t4 = window_out.get("joint_application", {})
        window_out["lowcmd_vs_apply_rate_diff_ratio"] = _rate_diff(low_s, t4) if t4 else 0.0

        out[wname] = window_out

    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    txt = json.dumps(out, indent=2, sort_keys=True)
    args.output_json.write_text(txt + "\n")
    print(txt)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
