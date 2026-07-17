#!/usr/bin/env python3
"""Publish a fixed route and capture truth/odometry/map evidence for one FSM mode."""

import argparse
import csv
import json
import math
import os
import time
import itertools

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int8


class Capture:
    def __init__(self, output_dir):
        self.output_dir = output_dir
        self.truth = []
        self.odom = []
        self.registered_count = 0
        self.map_count = 0
        self.last_cloud = None
        self.start_wall = time.monotonic()
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.on_models, queue_size=10)
        rospy.Subscriber("/Odometry", Odometry, self.on_odom, queue_size=50)
        rospy.Subscriber("/cloud_registered", PointCloud2, self.on_registered, queue_size=2)
        rospy.Subscriber("/Laser_map", PointCloud2, self.on_map, queue_size=1)

    def on_models(self, msg):
        if "a1_gazebo" not in msg.name:
            return
        i = msg.name.index("a1_gazebo")
        p, q, v = msg.pose[i].position, msg.pose[i].orientation, msg.twist[i]
        self.truth.append((rospy.Time.now().to_sec(), time.monotonic() - self.start_wall,
                           p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                           v.linear.x, v.linear.y, v.linear.z, v.angular.z))

    def on_odom(self, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        self.odom.append((msg.header.stamp.to_sec(), time.monotonic() - self.start_wall,
                          p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                          v.linear.x, v.linear.y, v.linear.z, v.angular.z))

    def on_registered(self, msg):
        self.registered_count += 1
        if self.last_cloud is None:
            self.last_cloud = msg

    def on_map(self, msg):
        self.map_count += 1
        self.last_cloud = msg

    def write_csv(self, name, rows):
        header = ["ros_time", "wall_elapsed", "x", "y", "z", "qx", "qy", "qz", "qw",
                  "vx", "vy", "vz", "wz"]
        with open(os.path.join(self.output_dir, name), "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(rows)

    def save_cloud(self, max_raw_points=200000, max_saved_points=50000, wall_timeout=8.0):
        if self.last_cloud is None:
            return {"points": 0, "pcd": None, "image": None}
        points = []
        deadline = time.monotonic() + wall_timeout
        raw_points = point_cloud2.read_points(
            self.last_cloud, field_names=("x", "y", "z"), skip_nans=True)
        stride = max(1, max_raw_points // max_saved_points)
        for i, p in enumerate(itertools.islice(raw_points, max_raw_points)):
            if time.monotonic() >= deadline:
                break
            if i % stride == 0 and all(math.isfinite(x) for x in p):
                points.append(p)
                if len(points) >= max_saved_points:
                    break
        pcd_path = os.path.join(self.output_dir, "map_ascii.pcd")
        with open(pcd_path, "w") as f:
            f.write("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n")
            f.write("COUNT 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n" % len(points))
            f.write("POINTS %d\nDATA ascii\n" % len(points))
            for p in points:
                f.write("%.5f %.5f %.5f\n" % p)
        image_path = None
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            if points:
                image_path = os.path.join(self.output_dir, "map_topdown.png")
                fig, ax = plt.subplots(figsize=(8, 8), dpi=150)
                ax.scatter([p[0] for p in points], [p[1] for p in points], s=0.15, c=[p[2] for p in points], cmap="viridis")
                if self.truth:
                    ax.plot([r[2] for r in self.truth], [r[3] for r in self.truth], "r-", linewidth=1.5, label="Gazebo truth")
                    ax.legend()
                ax.set_aspect("equal", adjustable="box")
                ax.set_xlabel("x [m]")
                ax.set_ylabel("y [m]")
                ax.set_title("FAST-LIO2 map, top-down")
                ax.grid(True, linewidth=0.2)
                fig.tight_layout()
                fig.savefig(image_path)
                plt.close(fig)
        except Exception as exc:
            rospy.logwarn("Could not render map projection: %s", exc)
        return {"points": len(points), "raw_point_limit": max_raw_points,
                "saved_point_limit": max_saved_points, "wall_timeout_s": wall_timeout,
                "pcd": pcd_path, "image": image_path,
                "frame_id": self.last_cloud.header.frame_id}


def publish_segment(pub, vx, wz, sim_duration, wall_timeout=360.0):
    start_sim = rospy.Time.now().to_sec()
    start_wall = time.monotonic()
    rate = rospy.Rate(10)
    msg = Twist()
    msg.linear.x, msg.angular.z = vx, wz
    while not rospy.is_shutdown():
        pub.publish(msg)
        if rospy.Time.now().to_sec() - start_sim >= sim_duration:
            return True
        if time.monotonic() - start_wall >= wall_timeout:
            return False
        rate.sleep()
    return False


def wait_for(predicate, description, wall_timeout):
    start = time.monotonic()
    while not rospy.is_shutdown() and time.monotonic() - start < wall_timeout:
        if predicate():
            return
        time.sleep(0.1)
    raise RuntimeError("wall-clock timeout waiting for %s" % description)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("trotting", "rl"), required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    os.makedirs(args.output_dir, exist_ok=True)
    rospy.init_node("mapping_trial_%s" % args.mode, anonymous=False)
    capture = Capture(args.output_dir)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    state_pub = rospy.Publisher("/fsm/state_cmd", Int8, queue_size=2)
    # rospy.wait_for_message(timeout=...) uses ROS time when /use_sim_time is
    # enabled. Explicit monotonic deadlines keep failures bounded at low RTF.
    wait_for(lambda: bool(capture.truth), "/gazebo/model_states", 90.0)
    wait_for(lambda: bool(capture.odom), "/Odometry", 180.0)
    wait_for(lambda: capture.registered_count > 0, "/cloud_registered", 180.0)
    time.sleep(2)
    state_pub.publish(Int8(data=2))
    time.sleep(8)
    state_pub.publish(Int8(data=4 if args.mode == "trotting" else 6))
    time.sleep(5)
    completed = []
    for name, vx, wz, duration in (("forward_1", 0.15, 0.0, 0.5),
                                   ("left_turn", 0.0, 0.25, 0.5),
                                   ("forward_2", 0.15, 0.0, 0.5),
                                   ("stop", 0.0, 0.0, 0.25)):
        completed.append({"name": name, "sim_duration": duration,
                          "completed": publish_segment(cmd_pub, vx, wz, duration)})
    for _ in range(10):
        cmd_pub.publish(Twist())
        time.sleep(0.1)
    time.sleep(3)
    capture.write_csv("ground_truth.csv", capture.truth)
    capture.write_csv("odometry.csv", capture.odom)
    cloud = capture.save_cloud()
    truth_finite = all(all(math.isfinite(v) for v in r[2:]) for r in capture.truth)
    odom_finite = all(all(math.isfinite(v) for v in r[2:]) for r in capture.odom)
    displacement = None
    min_z = None
    max_tilt_deg = None
    stop_speed = None
    if capture.truth:
        a, b = capture.truth[0], capture.truth[-1]
        displacement = math.hypot(b[2] - a[2], b[3] - a[3])
        min_z = min(r[4] for r in capture.truth)
        max_tilt_deg = max(math.degrees(2 * math.acos(min(1.0, math.sqrt(r[8] ** 2 + r[7] ** 2)))) for r in capture.truth)
        tail = capture.truth[-min(20, len(capture.truth)):]
        stop_speed = sum(math.hypot(r[9], r[10]) for r in tail) / len(tail)
    result = {
        "mode": args.mode, "status": "completed" if all(x["completed"] for x in completed) else "timeout",
        "segments": completed, "truth_samples": len(capture.truth), "odom_samples": len(capture.odom),
        "registered_messages": capture.registered_count, "laser_map_messages": capture.map_count,
        "truth_finite": truth_finite, "odom_finite": odom_finite,
        "ground_truth_displacement_m": displacement, "minimum_base_z_m": min_z,
        "maximum_tilt_deg": max_tilt_deg, "final_stop_mean_speed_mps": stop_speed,
        "map": cloud, "wall_duration_s": time.monotonic() - capture.start_wall
    }
    with open(os.path.join(args.output_dir, "trial_metrics.json"), "w") as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
