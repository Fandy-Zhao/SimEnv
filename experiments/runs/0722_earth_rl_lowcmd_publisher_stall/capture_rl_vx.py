#!/usr/bin/env python3
"""Capture a FixedStand-to-RL transition while commanding constant vx."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

HELPER_DIR = Path(__file__).resolve().parents[1] / "0722_earth_rl_deployment_semantics"
sys.path.insert(0, str(HELPER_DIR))

from transition_capture import (  # noqa: E402
    Capture,
    FSM_FIXEDSTAND,
    FSM_RL,
    publish_state,
    summarize,
    write_csv,
)


def publish_cmd(cmd_pub: rospy.Publisher, vx: float) -> None:
    msg = Twist()
    msg.linear.x = vx
    cmd_pub.publish(msg)


def publish_state_sampled(
    state_pub: rospy.Publisher,
    cmd_pub: rospy.Publisher,
    cap: Capture,
    state: int,
    phase: str,
    switch_sim: float,
    duration_wall: float,
    vx: float,
) -> None:
    msg = Int8(data=state)
    rate = rospy.Rate(50)
    deadline = time.monotonic() + duration_wall
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        state_pub.publish(msg)
        publish_cmd(cmd_pub, vx)
        cap.sample(phase, switch_sim)
        rate.sleep()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--model-name", default="a1_gazebo")
    parser.add_argument("--fixedstand-hold-sim", type=float, default=5.0)
    parser.add_argument("--post-rl-sim", type=float, default=8.0)
    parser.add_argument("--label", default="rl_vx_0p10")
    parser.add_argument("--vx", type=float, default=0.10)
    parser.add_argument("--wall-timeout", type=float, default=180.0)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    rospy.init_node("rl_vx_capture", anonymous=True)
    cap = Capture(args.model_name)
    state_pub = rospy.Publisher("/fsm/state_cmd", Int8, queue_size=4)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=20)

    deadline = time.monotonic() + args.wall_timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if cap.clock_rows and cap.latest_model and cap.latest_joint and cap.latest_imu:
            break
        time.sleep(0.05)
    if not (cap.clock_rows and cap.latest_model and cap.latest_joint and cap.latest_imu):
        raise RuntimeError("required ROS streams did not become ready")

    publish_state(state_pub, FSM_FIXEDSTAND, 2.0)
    start_fixed = rospy.Time.now().to_sec()
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() - start_fixed < args.fixedstand_hold_sim:
        publish_cmd(cmd_pub, 0.0)
        cap.sample("fixedstand_hold", None)
        rate.sleep()

    pre_rows = list(cap.rows)
    pre_cutoff = rospy.Time.now().to_sec() - 1.0
    cap.rows = [r for r in pre_rows if float(r["sim_time"]) >= pre_cutoff]

    switch_sim = rospy.Time.now().to_sec()
    publish_state_sampled(state_pub, cmd_pub, cap, FSM_RL, "rl_vx", switch_sim, 1.0, args.vx)
    start_post = rospy.Time.now().to_sec()
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() - start_post < args.post_rl_sim:
        publish_cmd(cmd_pub, args.vx)
        cap.sample("rl_vx", switch_sim)
        rate.sleep()

    csv_path = os.path.join(args.output_dir, f"{args.label}.csv")
    json_path = os.path.join(args.output_dir, f"{args.label}_summary.json")
    write_csv(csv_path, cap.rows)
    summary = summarize(cap.rows, switch_sim)
    post = [r for r in cap.rows if isinstance(r.get("rel_to_rl_switch"), float) and r["rel_to_rl_switch"] >= 0.0]
    if len(post) >= 2:
        summary["post_delta_x"] = float(post[-1]["base_x"]) - float(post[0]["base_x"])
        summary["post_mean_base_vx"] = sum(float(r["base_vx"]) for r in post) / len(post)
    summary["commanded_vx"] = args.vx
    with open(json_path, "w") as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
