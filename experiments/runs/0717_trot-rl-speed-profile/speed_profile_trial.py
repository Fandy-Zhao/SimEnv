#!/usr/bin/env python3
"""Capture one bounded, straight-line A1 speed trial from Gazebo truth."""

import argparse
import csv
import json
import math
import os
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8


class TrialCapture:
    def __init__(self):
        self.samples = []
        self.start_wall = time.monotonic()
        self._last_sample_time = None
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_models, queue_size=100)

    def _on_models(self, message):
        if "a1_gazebo" not in message.name:
            return
        stamp = rospy.Time.now().to_sec()
        # 50 Hz preserves a smooth planar trace without retaining every physics tick.
        if self._last_sample_time is not None and stamp - self._last_sample_time < 0.02:
            return
        self._last_sample_time = stamp
        index = message.name.index("a1_gazebo")
        pose, twist = message.pose[index], message.twist[index]
        self.samples.append({
            "ros_time": stamp,
            "wall_elapsed": time.monotonic() - self.start_wall,
            "x": pose.position.x, "y": pose.position.y, "z": pose.position.z,
            "qx": pose.orientation.x, "qy": pose.orientation.y,
            "qz": pose.orientation.z, "qw": pose.orientation.w,
            "vx": twist.linear.x, "vy": twist.linear.y, "vz": twist.linear.z,
            "wz": twist.angular.z,
        })

    def wait_for_truth(self, wall_timeout):
        deadline = time.monotonic() + wall_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.samples:
                return
            time.sleep(0.05)
        raise RuntimeError("wall-clock timeout waiting for /gazebo/model_states")


def publish_for_sim_time(publisher, velocity, sim_duration, wall_timeout):
    message = Twist()
    message.linear.x = velocity
    start_sim = rospy.Time.now().to_sec()
    start_wall = time.monotonic()
    # Wall-clock throttling prevents a low RTF from causing an unbounded publish loop.
    while not rospy.is_shutdown():
        publisher.publish(message)
        now_sim = rospy.Time.now().to_sec()
        if now_sim - start_sim >= sim_duration:
            return {"completed": True, "sim_start": start_sim, "sim_end": now_sim,
                    "wall_start": start_wall, "wall_end": time.monotonic()}
        if time.monotonic() - start_wall >= wall_timeout:
            publisher.publish(Twist())
            return {"completed": False, "sim_start": start_sim, "sim_end": now_sim,
                    "wall_start": start_wall, "wall_end": time.monotonic()}
        time.sleep(0.02)
    return {"completed": False, "sim_start": start_sim, "sim_end": rospy.Time.now().to_sec(),
            "wall_start": start_wall, "wall_end": time.monotonic()}


def finite(sample):
    return all(math.isfinite(value) for value in sample.values() if isinstance(value, float))


def yaw_deg(sample):
    siny = 2.0 * (sample["qw"] * sample["qz"] + sample["qx"] * sample["qy"])
    cosy = 1.0 - 2.0 * (sample["qy"] ** 2 + sample["qz"] ** 2)
    return math.degrees(math.atan2(siny, cosy))


def metrics_for(capture, active, stopped, command):
    active_samples = [s for s in capture.samples
                      if active["sim_start"] <= s["ros_time"] <= active["sim_end"]]
    stop_samples = [s for s in capture.samples
                    if stopped["sim_start"] <= s["ros_time"] <= stopped["sim_end"]]
    if len(active_samples) < 2:
        raise RuntimeError("insufficient active truth samples")
    path_length = sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"])
                      for a, b in zip(active_samples, active_samples[1:]))
    sim_elapsed = active["sim_end"] - active["sim_start"]
    wall_elapsed = active["wall_end"] - active["wall_start"]
    displacement = math.hypot(active_samples[-1]["x"] - active_samples[0]["x"],
                              active_samples[-1]["y"] - active_samples[0]["y"])
    tail = stop_samples[-max(1, len(stop_samples) // 3):]
    stop_speed = (sum(math.hypot(s["vx"], s["vy"]) for s in tail) / len(tail)) if tail else None
    halfway = active["sim_start"] + sim_elapsed / 2.0
    latter = [s for s in active_samples if s["ros_time"] >= halfway]
    latter_path = sum(math.hypot(b["x"] - a["x"], b["y"] - a["y"])
                      for a, b in zip(latter, latter[1:]))
    latter_elapsed = latter[-1]["ros_time"] - latter[0]["ros_time"] if len(latter) >= 2 else 0.0
    return {
        "truth_samples_total": len(capture.samples),
        "truth_samples_active": len(active_samples),
        "truth_samples_stop": len(stop_samples),
        "truth_finite": all(finite(s) for s in capture.samples),
        "active_sim_elapsed_s": sim_elapsed,
        "active_wall_elapsed_s": wall_elapsed,
        "real_time_factor": sim_elapsed / wall_elapsed if wall_elapsed > 0 else None,
        "path_length_m": path_length,
        "net_displacement_m": displacement,
        "actual_mean_horizontal_speed_mps": path_length / sim_elapsed if sim_elapsed > 0 else None,
        "latter_half_mean_horizontal_speed_mps": latter_path / latter_elapsed if latter_elapsed > 0 else None,
        "tracking_ratio": (path_length / sim_elapsed / command) if sim_elapsed > 0 and command else None,
        "final_stop_mean_speed_mps": stop_speed,
        "min_base_z_m": min(s["z"] for s in capture.samples),
        "max_abs_yaw_deg": max(abs(yaw_deg(s)) for s in active_samples),
        "active_start_xy_m": [active_samples[0]["x"], active_samples[0]["y"]],
        "active_end_xy_m": [active_samples[-1]["x"], active_samples[-1]["y"]],
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("trotting", "rl"), required=True)
    parser.add_argument("--speed", type=float, required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--active-sim-seconds", type=float, default=0.5)
    parser.add_argument("--stop-sim-seconds", type=float, default=0.25)
    parser.add_argument("--wall-timeout", type=float, default=90.0)
    args = parser.parse_args()
    if args.speed <= 0.0:
        parser.error("--speed must be positive")
    os.makedirs(args.output_dir, exist_ok=True)
    rospy.init_node("speed_profile_%s_%03d" % (args.mode, round(args.speed * 100)), anonymous=True)
    capture = TrialCapture()
    command_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    state_publisher = rospy.Publisher("/fsm/state_cmd", Int8, queue_size=2)
    capture.wait_for_truth(90.0)
    # Repeat requests over wall time so a slow /clock cannot hide the command.
    for _ in range(10):
        state_publisher.publish(Int8(data=2))
        time.sleep(0.1)
    time.sleep(5.0)
    for _ in range(10):
        state_publisher.publish(Int8(data=4 if args.mode == "trotting" else 6))
        time.sleep(0.1)
    time.sleep(4.0)
    active = publish_for_sim_time(command_publisher, args.speed, args.active_sim_seconds, args.wall_timeout)
    stopped = publish_for_sim_time(command_publisher, 0.0, args.stop_sim_seconds, args.wall_timeout)
    for _ in range(10):
        command_publisher.publish(Twist())
        time.sleep(0.03)
    result = {"schema_version": 1, "mode": args.mode, "command_speed_mps": args.speed,
              "active": active, "stop": stopped,
              "status": "completed" if active["completed"] and stopped["completed"] else "timeout"}
    try:
        result.update(metrics_for(capture, active, stopped, args.speed))
    except RuntimeError as error:
        result["status"] = "insufficient_data"
        result["error"] = str(error)
    with open(os.path.join(args.output_dir, "ground_truth.csv"), "w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(capture.samples[0]) if capture.samples else ["ros_time"])
        writer.writeheader()
        writer.writerows(capture.samples)
    with open(os.path.join(args.output_dir, "trial_metrics.json"), "w") as handle:
        json.dump(result, handle, indent=2, sort_keys=True)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
