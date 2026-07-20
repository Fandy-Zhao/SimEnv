#!/usr/bin/env python3
"""ROS-free metric helpers for Earth flat-ground runtime validation."""

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
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def quaternion_tilt_deg(qx: float, qy: float, qz: float, qw: float) -> float:
    # Body z-axis expressed in world; tilt is the angle from world +Z.
    z_axis_z = 1.0 - 2.0 * (qx * qx + qy * qy)
    z_axis_z = max(-1.0, min(1.0, z_axis_z))
    return math.degrees(math.acos(z_axis_z))


def wrap_angle_rad(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def world_to_initial_body(dx: float, dy: float, initial_yaw: float) -> Tuple[float, float]:
    c = math.cos(initial_yaw)
    s = math.sin(initial_yaw)
    return c * dx + s * dy, -s * dx + c * dy


def finite(values: Iterable[float]) -> List[float]:
    return [float(value) for value in values if math.isfinite(float(value))]


def mean(values: Iterable[float]) -> Optional[float]:
    vals = finite(values)
    return statistics.fmean(vals) if vals else None


def median(values: Iterable[float]) -> Optional[float]:
    vals = finite(values)
    return statistics.median(vals) if vals else None


def p95(values: Iterable[float]) -> Optional[float]:
    vals = sorted(finite(values))
    if not vals:
        return None
    return vals[min(len(vals) - 1, int(math.ceil(0.95 * len(vals))) - 1)]


def stdev(values: Iterable[float]) -> Optional[float]:
    vals = finite(values)
    return statistics.stdev(vals) if len(vals) > 1 else 0.0 if vals else None


def summarize_pose(rows: Sequence[Dict[str, float]]) -> Dict[str, object]:
    if len(rows) < 2:
        return {"valid": False, "sample_count": len(rows), "reason": "INSUFFICIENT_POSE_SAMPLES"}
    start = rows[0]
    end = rows[-1]
    duration = max(float(end["sim_time"]) - float(start["sim_time"]), EPSILON)
    forward, lateral = world_to_initial_body(
        float(end["x"]) - float(start["x"]),
        float(end["y"]) - float(start["y"]),
        float(start["yaw"]),
    )
    tilts = [float(row["tilt_deg"]) for row in rows]
    wz = [abs(float(row["angular_z"])) for row in rows]
    linear_speed = [
        math.hypot(float(row["linear_x"]), float(row["linear_y"]))
        for row in rows
    ]
    return {
        "valid": True,
        "sample_count": len(rows),
        "sim_duration": duration,
        "initial_x": float(start["x"]),
        "initial_y": float(start["y"]),
        "initial_z": float(start["z"]),
        "final_x": float(end["x"]),
        "final_y": float(end["y"]),
        "final_z": float(end["z"]),
        "min_base_height": min(float(row["z"]) for row in rows),
        "final_base_height": float(end["z"]),
        "forward_displacement": forward,
        "lateral_displacement": lateral,
        "yaw_change_deg": math.degrees(wrap_angle_rad(float(end["yaw"]) - float(start["yaw"]))),
        "max_tilt_deg": max(tilts),
        "p95_tilt_deg": p95(tilts),
        "duration_tilt_over_20_deg": duration_over(rows, "tilt_deg", 20.0),
        "duration_tilt_over_30_deg": duration_over(rows, "tilt_deg", 30.0),
        "peak_angular_velocity": max(wz) if wz else None,
        "mean_planar_speed": mean(linear_speed),
    }


def duration_over(rows: Sequence[Dict[str, float]], key: str, threshold: float) -> float:
    total = 0.0
    for prev, cur in zip(rows, rows[1:]):
        if float(prev[key]) > threshold or float(cur[key]) > threshold:
            total += max(0.0, float(cur["sim_time"]) - float(prev["sim_time"]))
    return total


def summarize_rtf(clock_rows: Sequence[Dict[str, float]]) -> Dict[str, object]:
    rtfs = [float(row["rtf"]) for row in clock_rows if "rtf" in row and math.isfinite(float(row["rtf"]))]
    return {
        "rtf_sample_count": len(rtfs),
        "rtf_mean": mean(rtfs),
        "rtf_median": median(rtfs),
        "rtf_min": min(rtfs) if rtfs else None,
        "rtf_max": max(rtfs) if rtfs else None,
    }


def classify_fixedstand(summary: Dict[str, object]) -> str:
    if not summary.get("valid"):
        return "FAIL_RUNTIME"
    if float(summary["min_base_height"]) < 0.12:
        return "FAIL_ATTITUDE"
    if float(summary["max_tilt_deg"]) > 45.0 or float(summary["duration_tilt_over_30_deg"]) > 0.25:
        return "FAIL_ATTITUDE"
    return "PASS"


def aggregate(values: Sequence[float]) -> Dict[str, Optional[float]]:
    vals = finite(values)
    return {
        "mean": mean(vals),
        "stdev": stdev(vals),
        "min": min(vals) if vals else None,
        "max": max(vals) if vals else None,
    }
