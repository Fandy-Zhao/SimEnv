#!/usr/bin/env python3
"""Analyze speed-sweep CSV, output compact JSON per capture_<vx>."""

import argparse, csv, json, math, statistics
from typing import Any, Dict, List, Optional, Tuple


def _float(v: Any) -> Optional[float]:
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def body_rotate(vx: float, vy: float, yaw: float) -> Tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return vx * c + vy * s, -vx * s + vy * c


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def percentile(sorted_vals: List[float], p: float) -> Optional[float]:
    if not sorted_vals:
        return None
    n = len(sorted_vals)
    k = (n - 1) * p / 100.0
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    if lo == hi:
        return sorted_vals[lo]
    return sorted_vals[lo] * (hi - k) + sorted_vals[hi] * (k - lo)


def median(values: List[float]) -> Optional[float]:
    vals = sorted(v for v in values if math.isfinite(v))
    return statistics.median(vals) if vals else None


def rms(values: List[float]) -> Optional[float]:
    vals = [v for v in values if math.isfinite(v)]
    return math.sqrt(sum(v * v for v in vals) / len(vals)) if vals else None


def analyze_phase(rows: List[Dict], diag_rows: Optional[List[Dict[str, float]]] = None
                  ) -> Dict[str, Any]:
    if not rows:
        return {}
    heights = [z for r in rows if (z := _float(r.get("base_z"))) is not None]
    tilts = [t for r in rows if (t := _float(r.get("tilt_deg"))) is not None]
    rtfs = sorted([t for r in rows if (t := _float(r.get("rtf"))) is not None])

    yaws: List[float] = []
    bf_vx: List[float] = []
    bf_vy: List[float] = []
    yaw_rates: List[float] = []
    for r in rows:
        qx = _float(r.get("base_qx")); qy = _float(r.get("base_qy"))
        qz = _float(r.get("base_qz")); qw = _float(r.get("base_qw"))
        if None in (qx, qy, qz, qw):
            continue
        yaw = yaw_from_quat(qx, qy, qz, qw)  # type: ignore[arg-type]
        yaws.append(yaw)
        wvx = _float(r.get("base_vx")) or 0.0
        wvy = _float(r.get("base_vy")) or 0.0
        fwd, lat = body_rotate(wvx, wvy, yaw)
        bf_vx.append(fwd)
        bf_vy.append(lat)
        wz = _float(r.get("base_wz"))
        if wz is not None:
            yaw_rates.append(wz)

    xs = [x for r in rows if (x := _float(r.get("base_x"))) is not None]
    ys = [y for r in rows if (y := _float(r.get("base_y"))) is not None]
    bf_disp_fwd: Optional[float] = None
    bf_disp_lat: Optional[float] = None
    planar_disp: Optional[float] = None
    if xs and ys and yaws:
        dx = xs[-1] - xs[0]
        dy = ys[-1] - ys[0]
        planar_disp = math.hypot(dx, dy)
        bf_disp_fwd, bf_disp_lat = body_rotate(dx, dy, yaws[0])

    yaw_drift = angle_diff(yaws[-1], yaws[0]) if yaws else None
    cmd_values = [v for r in rows if (v := _float(r.get("cmd_vx"))) is not None]
    cmd_vx = median(cmd_values)
    errors = [v - cmd_vx for v in bf_vx] if cmd_vx not in (None, 0.0) else []

    result: Dict[str, Any] = {
        "commanded_vx": cmd_vx,
        "body_forward_displacement": bf_disp_fwd,
        "body_lateral_displacement": bf_disp_lat,
        "planar_displacement": planar_disp,
        "body_forward_vx_mean": statistics.mean(bf_vx) if bf_vx else None,
        "body_forward_vx_median": median(bf_vx),
        "body_forward_vx_p10": percentile(sorted(bf_vx), 10.0),
        "body_forward_vx_p90": percentile(sorted(bf_vx), 90.0),
        "body_lateral_vx_mean": statistics.mean(bf_vy) if bf_vy else None,
        "body_lateral_vx_median": median(bf_vy),
        "tracking_gain": (median(bf_vx) / cmd_vx) if cmd_vx not in (None, 0.0) and median(bf_vx) is not None else None,
        "mae": statistics.mean([abs(e) for e in errors]) if errors else None,
        "rmse": rms(errors),
        "yaw_drift_deg": math.degrees(yaw_drift) if yaw_drift is not None else None,
        "median_yaw_rate": median(yaw_rates),
        "min_height": min(heights) if heights else None,
        "max_tilt_deg": max(tilts) if tilts else None,
        "fell": bool(heights and min(heights) < 0.18),
        "rtf_mean": statistics.mean(rtfs) if rtfs else None,
        "rtf_median": statistics.median(rtfs) if rtfs else None,
        "rtf_p10": percentile(rtfs, 10.0),
        "sample_count": len(rows),
    }

    targets = [value for r in rows for k, raw in r.items()
               if k.startswith("target_") and (value := _float(raw)) is not None]
    result["joint_target_rms"] = rms(targets)

    if diag_rows is not None:
        sims = [t for r in rows if (t := _float(r.get("sim_time"))) is not None]
        if sims:
            t0, t1 = min(sims), max(sims)
            scoped = [r for r in diag_rows if t0 <= r["sim_time"] <= t1]
            result["policy_observation_command_vx"] = median([r["policy_cmd_vx"] for r in scoped])
            result["raw_action_rms"] = rms([v for r in scoped for v in r["raw_action"]])
            result["scaled_action_rms"] = rms([v for r in scoped for v in r["scaled_action"]])
            result["diag_sample_count"] = len(scoped)

    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument("--diag-csv", default=None)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    with open(args.csv, newline="") as f:
        rows = list(csv.DictReader(f))

    diag_rows: Optional[List[Dict[str, float]]] = None
    if args.diag_csv:
        diag_rows = []
        with open(args.diag_csv, newline="") as f:
            for r in csv.DictReader(f):
                source_us = _float(r.get("source_sim_time_us"))
                policy_vx = _float(r.get("commands_scaled_0"))
                raw_action = [_float(r.get(f"raw_action_model_{i}")) for i in range(12)]
                scaled_action = [_float(r.get(f"scaled_action_policy_{i}")) for i in range(12)]
                if source_us is None or policy_vx is None:
                    continue
                if any(v is None for v in raw_action + scaled_action):
                    continue
                diag_rows.append({
                    "sim_time": source_us / 1_000_000.0,
                    "policy_cmd_vx": policy_vx,
                    "raw_action": [v for v in raw_action if v is not None],
                    "scaled_action": [v for v in scaled_action if v is not None],
                })

    phases: Dict[str, List[Dict]] = {}
    for r in rows:
        ph = r.get("phase", "")
        if ph.startswith("capture_"):
            phases.setdefault(ph, []).append(r)

    summary = {ph: analyze_phase(phases[ph], diag_rows)
               for ph in sorted(phases, key=lambda p: float(p.split("_", 1)[1]))}

    zero = summary.get("capture_0.0") or summary.get("capture_0")
    if zero and zero.get("raw_action_rms") is not None:
        for ph, data in summary.items():
            if data.get("raw_action_rms") is not None:
                data["action_rms_delta_vs_zero"] = abs(data["raw_action_rms"] - zero["raw_action_rms"])

    for ph, data in summary.items():
        cmd = data.get("commanded_vx")
        med = data.get("body_forward_vx_median")
        disp = data.get("body_forward_displacement")
        stable = not data.get("fell")
        motion = cmd in (None, 0.0) or (
            (disp is not None and disp >= 0.10) or
            (med is not None and med >= 0.015)
        )
        gain = data.get("tracking_gain")
        error = abs((med or 0.0) - cmd) if cmd not in (None, 0.0) and med is not None else None
        tracking = cmd in (None, 0.0) or (
            gain is not None and 0.50 <= gain <= 1.30 and
            error is not None and error <= 0.10
        )
        recommended = cmd in (None, 0.0) or (
            gain is not None and 0.70 <= gain <= 1.20 and
            error is not None and error <= 0.08
        )
        data["motion_pass"] = bool(motion)
        data["tracking_pass"] = bool(tracking)
        data["recommended_tracking_pass"] = bool(recommended)
        data["verdict"] = "FAIL" if not stable else ("TRACKING_PASS" if tracking else ("MOTION_PASS" if motion else "NO_EFFECTIVE_MOTION"))

    with open(args.output, "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
