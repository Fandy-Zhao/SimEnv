#!/usr/bin/env python3
"""Combo timing analysis: FSM + LOWCMD + CMD_RECEIVE + CMD_APPLY across two windows."""

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

    # Collect from timing CSV (streaming)
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

    # Collect from apply CSV (streaming)
    win_apply = {w: {"recv": [], "apply": [], "seqs": []} for w in windows}
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

    out = {"switch_sim_time_s": ts["switch_sim_time"]}
    for wname in windows:
        d = win_data[wname]
        a = win_apply[wname]
        fsm_s = _summary(d["fsm"])
        low_s = _summary(d["lowcmd"])
        recv_s = _summary(a["recv"])
        appl_s = _summary(a["apply"])
        jumps = _seq_jumps(a["seqs"])
        out[wname] = {
            "window_us": list(windows[wname]),
            "fsm_accepted": fsm_s,
            "lowcmd_published": low_s,
            "cmd_receive": recv_s,
            "cmd_apply_new": appl_s,
            "cmd_apply_sequence_jumps": jumps,
            "lowcmd_vs_apply_rate_diff_ratio": _rate_diff(low_s, appl_s),
        }

    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    txt = json.dumps(out, indent=2, sort_keys=True)
    args.output_json.write_text(txt + "\n")
    print(txt)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
