#!/usr/bin/env python3
"""Live ROS capture for the RL fast validation FixedStand gate."""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime, timezone
from typing import Any, Dict, List, Optional

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
import rl_fast_metrics as metrics  # noqa: E402


FSM_FIXEDSTAND = 2
FSM_RL = 6


class TimingTail:
    def __init__(self, path: str):
        self.path = path
        self.rows: List[Dict[str, str]] = []
        self.latest: Optional[Dict[str, str]] = None

    def refresh(self) -> None:
        if not self.path or not os.path.exists(self.path) or os.path.getsize(self.path) == 0:
            return
        try:
            with open(self.path, newline="") as handle:
                rows = list(csv.DictReader(handle))
        except (OSError, csv.Error):
            return
        if rows:
            self.rows = rows
            self.latest = rows[-1]

    def latest_int(self, key: str, default: int = 0) -> int:
        self.refresh()
        for row in reversed(self.rows):
            try:
                raw = row.get(key, "")
                if raw != "":
                    return int(float(raw))
            except (TypeError, ValueError):
                continue
        return default


class LiveCapture:
    def __init__(self, model_name: str):
        self.model_name = model_name
        self.model_names: List[str] = []
        self.rows: List[Dict[str, Any]] = []
        self.clock_rows: List[Dict[str, float]] = []
        self.joint_rows: List[Dict[str, Any]] = []
        self.last_clock_wall: Optional[float] = None
        self.last_clock_sim: Optional[float] = None
        self.latest_rtf: Optional[float] = None
        rospy.Subscriber("/clock", Clock, self._on_clock, queue_size=200)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_models, queue_size=100)
        rospy.Subscriber("/a1_gazebo/joint_states", JointState, self._on_joints, queue_size=50)

    def _on_clock(self, message: Clock) -> None:
        now_wall = time.monotonic()
        now_sim = float(message.clock.to_sec())
        row = {"wall_time": now_wall, "sim_time": now_sim}
        if self.last_clock_wall is not None and self.last_clock_sim is not None:
            wall_dt = max(now_wall - self.last_clock_wall, 1e-9)
            sim_dt = max(now_sim - self.last_clock_sim, 0.0)
            self.latest_rtf = sim_dt / wall_dt
            row["rtf"] = self.latest_rtf
        self.last_clock_wall = now_wall
        self.last_clock_sim = now_sim
        self.clock_rows.append(row)

    def _on_models(self, message: ModelStates) -> None:
        self.model_names = list(message.name)
        if self.model_name not in message.name:
            return
        idx = message.name.index(self.model_name)
        pose = message.pose[idx]
        twist = message.twist[idx]
        q = pose.orientation
        self.rows.append(
            {
                "sim_time": float(rospy.Time.now().to_sec()),
                "base_x": float(pose.position.x),
                "base_y": float(pose.position.y),
                "base_z": float(pose.position.z),
                "base_qx": float(q.x),
                "base_qy": float(q.y),
                "base_qz": float(q.z),
                "base_qw": float(q.w),
                "base_vx": float(twist.linear.x),
                "base_vy": float(twist.linear.y),
                "base_vz": float(twist.linear.z),
                "base_wx": float(twist.angular.x),
                "base_wy": float(twist.angular.y),
                "base_wz": float(twist.angular.z),
                "tilt_deg": metrics.quaternion_tilt_deg(q.x, q.y, q.z, q.w),
                "rtf": self.latest_rtf if self.latest_rtf is not None else "",
                "fsm_state": "",
            }
        )

    def _on_joints(self, message: JointState) -> None:
        row: Dict[str, Any] = {"sim_time": float(rospy.Time.now().to_sec())}
        for name, pos in zip(message.name, message.position):
            row["pos_" + name] = float(pos)
        for name, vel in zip(message.name, message.velocity):
            row["vel_" + name] = float(vel)
        self.joint_rows.append(row)


def wait_until(predicate, timeout_wall: float, poll: float = 0.05) -> bool:
    deadline = time.monotonic() + timeout_wall
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(poll)
    return False


def wait_sim_duration(duration: float, wall_timeout: float, cmd_pub, cmd: Twist) -> bool:
    start = float(rospy.Time.now().to_sec())
    deadline = time.monotonic() + wall_timeout
    rate = rospy.Rate(30)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        cmd_pub.publish(cmd)
        if float(rospy.Time.now().to_sec()) - start >= duration:
            return True
        rate.sleep()
    return False


def publish_state_until(pub, timing: TimingTail, state_id: int, timeout_wall: float) -> bool:
    msg = Int8(data=state_id)
    deadline = time.monotonic() + timeout_wall
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        pub.publish(msg)
        if timing.latest_int("fsm_state") == state_id:
            return True
        rate.sleep()
    return False


def twist(vx: float, vy: float, yaw_rate: float) -> Twist:
    msg = Twist()
    msg.linear.x = vx
    msg.linear.y = vy
    msg.angular.z = yaw_rate
    return msg


def write_csv(path: str, rows: List[Dict[str, Any]], preferred: Optional[List[str]] = None) -> None:
    fields = preferred or sorted({key for row in rows for key in row})
    if not fields:
        fields = ["sim_time"]
    extras = sorted({key for row in rows for key in row if key not in fields})
    fieldnames = list(fields) + extras
    with open(path, "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def attach_fsm_state(rows: List[Dict[str, Any]], timing: TimingTail) -> None:
    timing.refresh()
    latest_state = timing.latest_int("fsm_state", default=0)
    for row in rows:
        if row.get("fsm_state") in ("", None):
            row["fsm_state"] = latest_state


def summarize_rtf(clock_rows: List[Dict[str, float]]) -> Dict[str, Any]:
    rtfs = [float(row["rtf"]) for row in clock_rows if "rtf" in row]
    if not rtfs:
        return {"rtf_sample_count": 0, "rtf_median": None, "rtf_mean": None, "rtf_min": None, "rtf_max": None}
    return {
        "rtf_sample_count": len(rtfs),
        "rtf_median": metrics.median(rtfs),
        "rtf_mean": sum(rtfs) / len(rtfs),
        "rtf_min": min(rtfs),
        "rtf_max": max(rtfs),
    }


def write_outputs(output_dir: str, result: Dict[str, Any], rows: List[Dict[str, Any]], clock_rows: List[Dict[str, float]], joint_rows: List[Dict[str, Any]]) -> None:
    write_csv(os.path.join(output_dir, "timeseries.csv"), rows, metrics.TSV_COLUMNS + ["tilt_deg", "fsm_state"])
    write_csv(os.path.join(output_dir, "clock_rtf.csv"), clock_rows, ["wall_time", "sim_time", "rtf"])
    write_csv(os.path.join(output_dir, "joint_states.csv"), joint_rows)
    with open(os.path.join(output_dir, "metrics.json"), "w") as handle:
        json.dump(result, handle, indent=2, sort_keys=True)
    verdict = {
        "schema_version": result.get("schema_version", 1),
        "case_id": result.get("case_id"),
        "verdict": result.get("verdict", "UNKNOWN"),
        "reason": result.get("reason", ""),
        "invalid_reasons": result.get("invalid_reasons", []),
        "secondary": result.get("secondary", []),
    }
    with open(os.path.join(output_dir, "verdict.json"), "w") as handle:
        json.dump(verdict, handle, indent=2, sort_keys=True)
    with open(os.path.join(output_dir, "summary.md"), "w") as handle:
        handle.write(f"# {result.get('case_id', 'unknown')} Summary\n\n")
        handle.write(f"**Verdict:** `{result.get('verdict', 'UNKNOWN')}`\n\n")
        handle.write(f"- Reason: {result.get('reason', '')}\n")
        handle.write(f"- World mode: {result.get('world_mode', '')}\n")
        handle.write(f"- Case kind: {result.get('case_kind', '')}\n")
        handle.write(f"- Samples: {result.get('sample_count', 0)}\n")
        handle.write(f"- Evaluation min base height: {result.get('evaluation_min_base_height')}\n")
        handle.write(f"- Max evaluation tilt deg: {result.get('max_tilt_evaluation')}\n")
        handle.write(f"- RTF median: {result.get('rtf_median')}\n")


def failure_result(args: argparse.Namespace, verdict: str, reason: str, invalid: List[str], cap: LiveCapture, timing: TimingTail) -> Dict[str, Any]:
    attach_fsm_state(cap.rows, timing)
    rtf = summarize_rtf(cap.clock_rows)
    return {
        "schema_version": 1,
        "case_id": args.case_id,
        "case_kind": args.case_kind,
        "world_mode": args.world_mode,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "verdict": verdict,
        "reason": reason,
        "invalid_reasons": sorted(set(invalid)),
        "secondary": [],
        "sample_count": len(cap.rows),
        "clock_sample_count": len(cap.clock_rows),
        "timing_rows": len(timing.rows),
        "final_fsm_state": timing.latest_int("fsm_state", default=0),
        "model_names": sorted(cap.model_names),
        **rtf,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--case-id", required=True)
    parser.add_argument("--case-kind", choices=("fixedstand", "rl"), default="fixedstand")
    parser.add_argument("--world-mode", choices=("earth", "competition"), default="earth")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--timing-csv", default="")
    parser.add_argument("--model-name", default="a1_gazebo")
    parser.add_argument("--transition-grace", type=float, default=1.0)
    parser.add_argument("--evaluation-duration", type=float, default=4.0)
    parser.add_argument("--wall-timeout", type=float, default=240.0)
    parser.add_argument("--policy-path", default="")
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    rospy.init_node("rl_fast_capture_" + args.case_id.replace("-", "_"), anonymous=True)
    cap = LiveCapture(args.model_name)
    timing = TimingTail(args.timing_csv)
    state_pub = rospy.Publisher("/fsm/state_cmd", Int8, queue_size=4)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    invalid: List[str] = []

    if not wait_until(lambda: cap.clock_rows and float(rospy.Time.now().to_sec()) > 0.0, 60.0):
        invalid.append("NO_CLOCK")
        result = failure_result(args, metrics.classify_clock_master_failure(cap.clock_rows), "ROS clock did not publish advancing sim time", invalid, cap, timing)
        write_outputs(args.output_dir, result, cap.rows, cap.clock_rows, cap.joint_rows)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 2

    if not wait_until(lambda: cap.model_names, 60.0):
        invalid.append("NO_MODEL_STATES")
        result = failure_result(args, "FAIL_GAZEBO_SIM_STALL", "/gazebo/model_states did not publish", invalid, cap, timing)
        write_outputs(args.output_dir, result, cap.rows, cap.clock_rows, cap.joint_rows)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 2

    if not wait_until(lambda: cap.rows, 60.0):
        invalid.append("NO_ROBOT_POSE")
        result = failure_result(args, "FAIL_GAZEBO_SIM_STALL", "robot pose did not appear in model_states", invalid, cap, timing)
        write_outputs(args.output_dir, result, cap.rows, cap.clock_rows, cap.joint_rows)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 2

    if args.world_mode == "earth" and ("platform_1" in cap.model_names or "platform_2" in cap.model_names):
        invalid.append("PLATFORM_STILL_PRESENT")

    entry_start_wall = time.monotonic()
    if not publish_state_until(state_pub, timing, FSM_FIXEDSTAND, 45.0):
        invalid.append("FIXEDSTAND_NOT_ENTERED")
        result = failure_result(args, "FAIL_FSM_ENTRY", "controller did not enter FixedStand", invalid, cap, timing)
        write_outputs(args.output_dir, result, cap.rows, cap.clock_rows, cap.joint_rows)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 3

    entry_latency_wall = time.monotonic() - entry_start_wall
    start_index = len(cap.rows)
    duration = args.transition_grace + args.evaluation_duration
    ok_duration = wait_sim_duration(duration, args.wall_timeout, cmd_pub, twist(0.0, 0.0, 0.0))
    for _ in range(10):
        cmd_pub.publish(twist(0.0, 0.0, 0.0))
        time.sleep(0.02)

    timing.refresh()
    rows = cap.rows[start_index:]
    attach_fsm_state(rows, timing)
    if not ok_duration:
        invalid.append("SIM_DURATION_INCOMPLETE")
    if timing.latest_int("fsm_state", default=0) != FSM_FIXEDSTAND:
        invalid.append("FIXEDSTAND_NOT_HELD")

    window_summary = metrics.summarize_fixedstand_windows(rows, transition_grace=args.transition_grace)
    fixedstand_verdict, fixedstand_flags = metrics.classify_fixedstand_evaluation(window_summary)
    rtf = summarize_rtf(cap.clock_rows)
    rtf_verdict, rtf_reason = metrics.rtf_gate(rtf.get("rtf_median"))
    verdicts = [fixedstand_verdict, rtf_verdict]
    secondary = list(fixedstand_flags)
    if rtf_reason == "limited_smoke_rtf_risk":
        secondary.append("LOW_RTF_RISK")
    if invalid:
        verdicts.append("FAIL_STATE_TRANSITION")
    final_verdict = metrics.worst_verdict(verdicts)
    reason = "FixedStand evaluation completed"
    if final_verdict != "PASS":
        reason = ",".join(sorted(set(invalid + secondary))) or final_verdict

    policy_sha = None
    if args.policy_path and os.path.exists(args.policy_path):
        policy_sha = metrics.compute_sha256(args.policy_path)

    result: Dict[str, Any] = {
        "schema_version": 1,
        "case_id": args.case_id,
        "case_kind": args.case_kind,
        "world_mode": args.world_mode,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "verdict": final_verdict,
        "reason": reason,
        "invalid_reasons": sorted(set(invalid)),
        "secondary": secondary,
        "sample_count": len(rows),
        "clock_sample_count": len(cap.clock_rows),
        "timing_rows": len(timing.rows),
        "final_fsm_state": timing.latest_int("fsm_state", default=0),
        "model_names": sorted(cap.model_names),
        "policy_path": args.policy_path,
        "policy_sha256": policy_sha,
        "entry_latency_wall": entry_latency_wall,
        "transition_grace": args.transition_grace,
        "evaluation_duration": args.evaluation_duration,
        **window_summary,
        **rtf,
    }
    write_outputs(args.output_dir, result, rows, cap.clock_rows, cap.joint_rows)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if final_verdict == "PASS" else 3


if __name__ == "__main__":
    raise SystemExit(main())
