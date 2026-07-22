#!/usr/bin/env python3
"""Short Earth RL speed probe for an already running auto.sh epoch."""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import statistics
import time
from typing import Any, Dict, List, Optional

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Int8


FSM_FIXEDSTAND = 2
FSM_RL = 6


def median(values: List[float]) -> Optional[float]:
    return statistics.median(values) if values else None


def mean(values: List[float]) -> Optional[float]:
    return sum(values) / len(values) if values else None


def percentile(values: List[float], q: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    idx = max(0, min(len(ordered) - 1, int(round((len(ordered) - 1) * q))))
    return ordered[idx]


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class Capture:
    def __init__(self, model_name: str):
        self.model_name = model_name
        self.model_names: List[str] = []
        self.rows: List[Dict[str, Any]] = []
        self.clock_rows: List[Dict[str, float]] = []
        self.last_clock_wall: Optional[float] = None
        self.last_clock_sim: Optional[float] = None
        self.latest_rtf: Optional[float] = None
        rospy.Subscriber("/clock", Clock, self._clock, queue_size=200)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._models, queue_size=100)

    def _clock(self, msg: Clock) -> None:
        wall = time.monotonic()
        sim = msg.clock.to_sec()
        row = {"wall_time": wall, "sim_time": sim}
        if self.last_clock_wall is not None and self.last_clock_sim is not None:
            wall_dt = max(wall - self.last_clock_wall, 1e-9)
            sim_dt = max(sim - self.last_clock_sim, 0.0)
            self.latest_rtf = sim_dt / wall_dt
            row["rtf"] = self.latest_rtf
        self.last_clock_wall = wall
        self.last_clock_sim = sim
        self.clock_rows.append(row)

    def _models(self, msg: ModelStates) -> None:
        self.model_names = list(msg.name)
        if self.model_name not in msg.name:
            return
        idx = msg.name.index(self.model_name)
        pose = msg.pose[idx]
        twist = msg.twist[idx]
        q = pose.orientation
        self.rows.append({
            "sim_time": rospy.Time.now().to_sec(),
            "wall_time": time.monotonic(),
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z,
            "qx": q.x,
            "qy": q.y,
            "qz": q.z,
            "qw": q.w,
            "yaw": yaw_from_quat(q.x, q.y, q.z, q.w),
            "vx": twist.linear.x,
            "vy": twist.linear.y,
            "vz": twist.linear.z,
            "wx": twist.angular.x,
            "wy": twist.angular.y,
            "wz": twist.angular.z,
            "rtf": self.latest_rtf,
        })


def wait_until(predicate, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


def publish_state(pub: rospy.Publisher, state: int, duration: float) -> None:
    msg = Int8(data=state)
    rate = rospy.Rate(10)
    end = time.monotonic() + duration
    while not rospy.is_shutdown() and time.monotonic() < end:
        pub.publish(msg)
        rate.sleep()


def publish_twist(pub: rospy.Publisher, vx: float, duration_sim: float, wall_timeout: float) -> None:
    msg = Twist()
    msg.linear.x = vx
    start = rospy.Time.now().to_sec()
    deadline = time.monotonic() + wall_timeout
    rate = rospy.Rate(30)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        pub.publish(msg)
        if rospy.Time.now().to_sec() - start >= duration_sim:
            return
        rate.sleep()


def write_csv(path: str, rows: List[Dict[str, Any]]) -> None:
    fields = ["sim_time", "wall_time", "x", "y", "z", "yaw", "vx", "vy", "wz", "rtf"]
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def summarize(cmd_vx: float, rows: List[Dict[str, Any]], grace: float) -> Dict[str, Any]:
    if not rows:
        return {"cmd_vx": cmd_vx, "stable": False, "verdict": "NO_SAMPLES"}
    start = rows[0]
    window = [r for r in rows if r["sim_time"] >= start["sim_time"] + grace]
    if len(window) < 2:
        window = rows
    vx = [float(r["vx"]) for r in window if r.get("vx") is not None]
    vy = [float(r["vy"]) for r in window if r.get("vy") is not None]
    wz = [float(r["wz"]) for r in window if r.get("wz") is not None]
    rtf = [float(r["rtf"]) for r in window if r.get("rtf") is not None]
    z = [float(r["z"]) for r in window if r.get("z") is not None]
    measured = median(vx) or 0.0
    gain = measured / cmd_vx if abs(cmd_vx) > 1e-9 else None
    mae = mean([abs(v - cmd_vx) for v in vx])
    lateral_drift = window[-1]["y"] - window[0]["y"]
    yaw_drift = wrap_angle(window[-1]["yaw"] - window[0]["yaw"])
    stable = bool(
        min(z) > 0.18 and
        measured > 0.02 and
        abs(median(vy) or 0.0) <= 0.08 and
        abs(median(wz) or 0.0) <= 0.15
    )
    tracking = bool(gain is not None and (0.70 <= gain <= 1.30 or (mae is not None and mae <= 0.10)))
    verdict = "PASS" if stable and tracking else ("STABLE_TRACKING_WEAK" if stable else "UNSTABLE")
    return {
        "cmd_vx": cmd_vx,
        "measured_vx_median": measured,
        "measured_vx_mean": mean(vx),
        "measured_vx_p10": percentile(vx, 0.10),
        "measured_vx_p90": percentile(vx, 0.90),
        "tracking_gain": gain,
        "mae": mae,
        "median_vy": median(vy),
        "median_yaw_rate": median(wz),
        "lateral_drift": lateral_drift,
        "yaw_drift": yaw_drift,
        "min_base_height": min(z) if z else None,
        "rtf_median": median(rtf),
        "rtf_mean": mean(rtf),
        "stable": stable,
        "verdict": verdict,
        "sample_count": len(window),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--model-name", default="a1_gazebo")
    parser.add_argument("--speeds", default="0.10,0.20,0.30,0.40,0.50")
    parser.add_argument("--settle-sim", type=float, default=3.0)
    parser.add_argument("--sample-sim", type=float, default=8.0)
    parser.add_argument("--recover-sim", type=float, default=2.0)
    parser.add_argument("--grace-sim", type=float, default=1.0)
    parser.add_argument("--wall-timeout", type=float, default=90.0)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    rospy.init_node("earth_rl_speed_probe", anonymous=True)
    cap = Capture(args.model_name)
    state_pub = rospy.Publisher("/fsm/state_cmd", Int8, queue_size=4)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    if not wait_until(lambda: cap.clock_rows and rospy.Time.now().to_sec() > 0.0, 60.0):
        raise RuntimeError("/clock did not publish advancing simulation time")
    if not wait_until(lambda: cap.rows, 60.0):
        raise RuntimeError("robot model state was not observed")

    publish_state(state_pub, FSM_FIXEDSTAND, 3.0)
    publish_twist(cmd_pub, 0.0, args.settle_sim, args.wall_timeout)
    publish_state(state_pub, FSM_RL, 3.0)
    publish_twist(cmd_pub, 0.0, 3.0, args.wall_timeout)

    results: List[Dict[str, Any]] = []
    for raw in args.speeds.split(","):
        speed = float(raw.strip())
        publish_twist(cmd_pub, 0.0, args.settle_sim, args.wall_timeout)
        start_idx = len(cap.rows)
        publish_twist(cmd_pub, speed, args.sample_sim, args.wall_timeout)
        rows = cap.rows[start_idx:]
        write_csv(os.path.join(args.output_dir, f"vx_{speed:.2f}.csv"), rows)
        result = summarize(speed, rows, args.grace_sim)
        results.append(result)
        publish_twist(cmd_pub, 0.0, args.recover_sim, args.wall_timeout)
        if not result.get("stable", False):
            break

    with open(os.path.join(args.output_dir, "speed_summary.json"), "w") as handle:
        json.dump({"speeds": results}, handle, indent=2, sort_keys=True)
    print(json.dumps({"speeds": results}, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
