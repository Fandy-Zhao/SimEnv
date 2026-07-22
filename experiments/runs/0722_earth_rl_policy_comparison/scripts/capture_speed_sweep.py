#!/usr/bin/env python3
"""Capture speed-sweep evidence for an already-running RL epoch."""

import argparse, json, os, sys, time
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../0722_earth_rl_deployment_semantics"))
from transition_capture import Capture, write_csv, FSM_FIXEDSTAND, FSM_RL  # noqa: E402


def run_phase(state_pub, cmd_pub, cap, state, phase, switch_sim, duration_sim, vx=0.0):
    msg = Int8(data=state)
    cmd = Twist()
    cmd.linear.x = vx
    rate = rospy.Rate(50)
    deadline = rospy.Time.now().to_sec() + duration_sim
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
        state_pub.publish(msg)
        cmd_pub.publish(cmd)
        cap.sample(phase, switch_sim)
        rate.sleep()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--model-name", default="a1_gazebo")
    parser.add_argument("--speeds", type=float, nargs="+", default=[0.0, 0.10, 0.20, 0.30, 0.40])
    parser.add_argument("--policy-name", default="earth_rl")
    parser.add_argument("--policy-sha", default="unknown")
    parser.add_argument("--fixedstand-sim", type=float, default=3.0)
    parser.add_argument("--rl-zero-sim", type=float, default=3.0)
    parser.add_argument("--settle-sim", type=float, default=2.0)
    parser.add_argument("--capture-sim", type=float, default=6.0)
    parser.add_argument("--recovery-sim", type=float, default=2.0)
    parser.add_argument("--wall-timeout", type=float, default=300.0)
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    rospy.init_node("speed_sweep_capture", anonymous=True)
    cap = Capture(args.model_name)
    state_pub = rospy.Publisher("/fsm/state_cmd", Int8, queue_size=4)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=20)

    deadline = time.monotonic() + args.wall_timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if cap.clock_rows and cap.latest_model and cap.latest_joint and cap.latest_imu:
            break
        time.sleep(0.05)
    if not (cap.clock_rows and cap.latest_model and cap.latest_joint and cap.latest_imu):
        raise RuntimeError("ROS streams did not become ready")

    run_phase(state_pub, cmd_pub, cap, FSM_FIXEDSTAND, "fixedstand", None, args.fixedstand_sim, vx=0.0)

    rl_switch_sim = rospy.Time.now().to_sec()
    run_phase(state_pub, cmd_pub, cap, FSM_RL, "rl_zero", rl_switch_sim, args.rl_zero_sim, vx=0.0)

    for vx in args.speeds:
        run_phase(state_pub, cmd_pub, cap, FSM_RL, f"settle_{vx}", rl_switch_sim, args.settle_sim, vx=vx)
        run_phase(state_pub, cmd_pub, cap, FSM_RL, f"capture_{vx}", rl_switch_sim, args.capture_sim, vx=vx)
        run_phase(state_pub, cmd_pub, cap, FSM_RL, f"recovery_{vx}", rl_switch_sim, args.recovery_sim, vx=0.0)

    csv_path = os.path.join(args.output_dir, "speed_sweep.csv")
    json_path = os.path.join(args.output_dir, "speed_sweep_meta.json")
    write_csv(csv_path, cap.rows)
    meta = {"policy_name": args.policy_name, "policy_sha": args.policy_sha,
            "speeds_attempted": args.speeds}
    with open(json_path, "w") as f:
        json.dump(meta, f, indent=2)
    print(json.dumps(meta, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
