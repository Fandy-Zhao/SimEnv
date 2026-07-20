#!/usr/bin/env python3
"""Metric helpers for the G2 trotting baseline.

The functions in this file are ROS-free so they can be unit-tested without a
running simulator.
"""

from __future__ import annotations

import math
import statistics
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


EPSILON = 1e-9


def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def wrap_angle_rad(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def world_to_body_velocity(vx: float, vy: float, yaw: float) -> Tuple[float, float]:
    """Rotate planar world velocity into the body frame using yaw only."""
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * vx + s * vy, -s * vx + c * vy


def rows_in_window(rows: Sequence[Dict[str, float]], start: float, end: float) -> List[Dict[str, float]]:
    return [row for row in rows if start <= float(row["sim_time"]) <= end]


def finite_values(values: Iterable[float]) -> List[float]:
    return [float(value) for value in values if math.isfinite(float(value))]


def sample_stats(values: Iterable[float]) -> Dict[str, Optional[float]]:
    vals = finite_values(values)
    if not vals:
        return {"mean": None, "median": None, "std": None, "min": None, "max": None}
    return {
        "mean": statistics.fmean(vals),
        "median": statistics.median(vals),
        "std": statistics.pstdev(vals) if len(vals) > 1 else 0.0,
        "min": min(vals),
        "max": max(vals),
    }


def lateral_drift_ratio(forward_displacement: float, lateral_displacement: float) -> float:
    return abs(lateral_displacement) / max(abs(forward_displacement), EPSILON)


def displacement_body_frame(rows: Sequence[Dict[str, float]]) -> Tuple[float, float]:
    if len(rows) < 2:
        return 0.0, 0.0
    start = rows[0]
    end = rows[-1]
    yaw0 = float(start["yaw"])
    dx = float(end["x"]) - float(start["x"])
    dy = float(end["y"]) - float(start["y"])
    return world_to_body_velocity(dx, dy, yaw0)


def stop_time_to_threshold(
    rows: Sequence[Dict[str, float]],
    command_zero_time: float,
    threshold: float,
) -> Optional[float]:
    for row in rows:
        sim_time = float(row["sim_time"])
        if sim_time < command_zero_time:
            continue
        speed = math.hypot(float(row["body_vx"]), float(row["body_vy"]))
        if speed <= threshold:
            return sim_time - command_zero_time
    return None


def aggregate_valid_trials(trials: Sequence[Dict[str, object]], key: str) -> Dict[str, Optional[float]]:
    values = [
        float(trial[key])
        for trial in trials
        if trial.get("trial_result") in ("PASS", "FAIL") and trial.get(key) is not None
    ]
    return sample_stats(values)


def classify_trial_status(reasons: Sequence[str]) -> str:
    return "PASS" if not reasons else "INVALID"


def compute_truth_metrics(
    rows: Sequence[Dict[str, float]],
    command_vx: float,
    command_start: float,
    command_end: float,
    zero_start: float,
    zero_end: float,
) -> Dict[str, object]:
    steady = rows_in_window(rows, command_start + 2.0, command_end)
    stop = rows_in_window(rows, zero_start, zero_end)
    if len(steady) < 2:
        raise ValueError("insufficient steady-window truth samples")

    forward, lateral = displacement_body_frame(steady)
    vx_stats = sample_stats(row["body_vx"] for row in steady)
    vy_stats = sample_stats(row["body_vy"] for row in steady)
    wz_stats = sample_stats(row["body_wz"] for row in steady)
    roll_stats = sample_stats(abs(row["roll"]) for row in steady)
    pitch_stats = sample_stats(abs(row["pitch"]) for row in steady)
    height_stats = sample_stats(row["z"] for row in steady)
    yaw_change = wrap_angle_rad(float(steady[-1]["yaw"]) - float(steady[0]["yaw"]))
    tail = stop[-max(1, len(stop) // 3):] if stop else []
    tail_speed = [math.hypot(float(row["body_vx"]), float(row["body_vy"])) for row in tail]
    mean_vx = vx_stats["mean"]
    tracking_ratio = None
    tracking_error = None
    if mean_vx is not None:
        tracking_error = abs(float(command_vx) - mean_vx)
        if abs(command_vx) > EPSILON:
            tracking_ratio = mean_vx / float(command_vx)

    return {
        "steady_sample_count": len(steady),
        "stop_sample_count": len(stop),
        "steady_mean_vx": mean_vx,
        "steady_median_vx": vx_stats["median"],
        "steady_std_vx": vx_stats["std"],
        "steady_min_vx": vx_stats["min"],
        "steady_max_vx": vx_stats["max"],
        "mean_vy": vy_stats["mean"],
        "yaw_rate_rms": wz_stats["std"],
        "absolute_tracking_error": tracking_error,
        "tracking_ratio": tracking_ratio,
        "forward_displacement": forward,
        "lateral_displacement": lateral,
        "lateral_drift_ratio": lateral_drift_ratio(forward, lateral),
        "yaw_drift_deg": math.degrees(yaw_change),
        "roll_peak_deg": math.degrees(roll_stats["max"] or 0.0),
        "pitch_peak_deg": math.degrees(pitch_stats["max"] or 0.0),
        "base_height_mean": height_stats["mean"],
        "base_height_min": height_stats["min"],
        "stop_time_to_0_10": stop_time_to_threshold(stop, zero_start, 0.10),
        "stop_time_to_0_05": stop_time_to_threshold(stop, zero_start, 0.05),
        "tail_speed_mean": statistics.fmean(tail_speed) if tail_speed else None,
        "tail_speed_max": max(tail_speed) if tail_speed else None,
    }
