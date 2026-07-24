#!/usr/bin/env python3
import csv
import math
import statistics
import time

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Latest:
    def __init__(self):
        self.odom = None
        self.cloud = None
        self.path = None
        self.cmd = None
        self.cmd_vel = None
        self.odom_count = 0
        self.cloud_count = 0
        self.path_count = 0
        self.cmd_count = 0
        self.cmd_vel_count = 0

    def on_odom(self, msg):
        self.odom = msg
        self.odom_count += 1

    def on_cloud(self, msg):
        self.cloud = msg
        self.cloud_count += 1

    def on_path(self, msg):
        self.path = msg
        self.path_count += 1

    def on_cmd(self, msg):
        self.cmd = msg
        self.cmd_count += 1

    def on_cmd_vel(self, msg):
        self.cmd_vel = msg
        self.cmd_vel_count += 1


def wait_sim_duration(duration):
    start = rospy.Time.now()
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        if now.to_sec() > 0 and (now - start).to_sec() >= duration:
            return start, now
        rospy.sleep(0.05)
    raise RuntimeError("shutdown")


def wait_for_inputs(state, min_sim_duration=10.0):
    while not rospy.is_shutdown() and (state.odom is None or state.cloud is None):
        rospy.sleep(0.1)
    c0 = (state.odom_count, state.cloud_count)
    t0, t1 = wait_sim_duration(min_sim_duration)
    c1 = (state.odom_count, state.cloud_count)
    return t0, t1, c0, c1


def cloud_stats(cloud, odom, max_samples=20000):
    px = odom.pose.pose.position.x
    py = odom.pose.pose.position.y
    pz = odom.pose.pose.position.z
    distances = []
    heights = []
    adjacent = 0
    total = 0
    for point in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
        x, y, z = point
        dx = x - px
        dy = y - py
        dz = z - pz
        dist = math.hypot(dx, dy)
        distances.append(dist)
        heights.append(dz)
        total += 1
        if dist < 3.5:
            adjacent += 1
        if total >= max_samples:
            break
    distances.sort()
    def pct(values, frac):
        if not values:
            return float("nan")
        idx = min(len(values) - 1, max(0, int(round(frac * (len(values) - 1)))))
        return values[idx]
    return {
        "raw_cloud_point_count": cloud.width * cloud.height,
        "sampled_finite_point_count": total,
        "sample_limit": max_samples,
        "adjacentRange_point_count": adjacent,
        "distance_min": pct(distances, 0.0),
        "distance_median": statistics.median(distances) if distances else float("nan"),
        "distance_p90": pct(distances, 0.9),
        "height_min": min(heights) if heights else float("nan"),
        "height_max": max(heights) if heights else float("nan"),
    }


def publish_waypoint(state, waypoint_pub):
    odom = state.odom
    pose = odom.pose.pose
    yaw = yaw_from_quat(pose.orientation)
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = odom.header.frame_id
    goal.pose.position.x = pose.position.x + 0.8 * math.cos(yaw)
    goal.pose.position.y = pose.position.y + 0.8 * math.sin(yaw)
    goal.pose.position.z = pose.position.z
    goal.pose.orientation.w = 1.0
    waypoint_pub.publish(goal)
    return goal, yaw


def twist_norm(msg):
    if msg is None:
        return 0.0
    return abs(msg.twist.linear.x) + abs(msg.twist.linear.y) + abs(msg.twist.angular.z)


def cmd_vel_norm(msg):
    if msg is None:
        return 0.0
    return abs(msg.linear.x) + abs(msg.linear.y) + abs(msg.angular.z)


def path_summary(path):
    if path is None:
        return {"path_count": 0}
    out = {"path_count": len(path.poses)}
    if path.poses:
        first = path.poses[0].pose.position
        last = path.poses[-1].pose.position
        out.update({
            "path_first": (first.x, first.y, first.z),
            "path_last": (last.x, last.y, last.z),
        })
    return out


def run_case(name, check_obstacle, state, check_pub, waypoint_pub, log_path):
    check_pub.publish(Bool(data=check_obstacle))
    rospy.sleep(0.5)
    goal, yaw = publish_waypoint(state, waypoint_pub)
    wall = time.time()
    sim = rospy.Time.now().to_sec()
    path0 = state.path_count
    cmd0 = state.cmd_count
    t0, t1 = wait_sim_duration(10.0)
    summary = {
        "case": name,
        "checkObstacle": check_obstacle,
        "wall_time": wall,
        "waypoint_sim_time": sim,
        "observe_start_sim": t0.to_sec(),
        "observe_end_sim": t1.to_sec(),
        "path_updates": state.path_count - path0,
        "cmd_updates": state.cmd_count - cmd0,
        "odom_frame": state.odom.header.frame_id if state.odom else "",
        "odom_x": state.odom.pose.pose.position.x,
        "odom_y": state.odom.pose.pose.position.y,
        "odom_yaw": yaw,
        "goal_x": goal.pose.position.x,
        "goal_y": goal.pose.position.y,
        "goal_frame": goal.header.frame_id,
        "cmd_norm": twist_norm(state.cmd),
        "cmd_linear_x": state.cmd.twist.linear.x if state.cmd else float("nan"),
        "cmd_linear_y": state.cmd.twist.linear.y if state.cmd else float("nan"),
        "cmd_angular_z": state.cmd.twist.angular.z if state.cmd else float("nan"),
        "cmd_vel_norm": cmd_vel_norm(state.cmd_vel),
    }
    summary.update(path_summary(state.path))
    with open(log_path, "w", encoding="utf-8") as handle:
        for key in sorted(summary):
            handle.write(f"{key}: {summary[key]}\n")
    return summary


def main():
    rospy.init_node("falco_r3_case_collector", anonymous=True)
    state = Latest()
    rospy.Subscriber("/navigation/state_estimation", Odometry, state.on_odom, queue_size=10)
    rospy.Subscriber("/navigation/registered_scan", PointCloud2, state.on_cloud, queue_size=2)
    rospy.Subscriber("/navigation/path", Path, state.on_path, queue_size=10)
    rospy.Subscriber("/navigation/falco/cmd_vel_stamped", TwistStamped, state.on_cmd, queue_size=10)
    rospy.Subscriber("/cmd_vel", Twist, state.on_cmd_vel, queue_size=10)
    waypoint_pub = rospy.Publisher("/navigation/way_point", PoseStamped, queue_size=1, latch=True)
    check_pub = rospy.Publisher("/navigation/check_obstacle", Bool, queue_size=1, latch=True)
    enabled_pub = rospy.Publisher("/navigation/enabled", Bool, queue_size=1, latch=True)
    enabled_pub.publish(Bool(data=False))

    base = "experiments/runs/0724_falco_r3_diagnosis"
    t0, t1, c0, c1 = wait_for_inputs(state, 10.0)
    with open(f"{base}/sim_time_timeline.csv", "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["event", "sim_time", "odom_count", "cloud_count", "path_count", "cmd_count"])
        writer.writerow(["fast_lio_window_start", t0.to_sec(), c0[0], c0[1], state.path_count, state.cmd_count])
        writer.writerow(["fast_lio_window_end", t1.to_sec(), c1[0], c1[1], state.path_count, state.cmd_count])

    stats = cloud_stats(state.cloud, state.odom)
    with open(f"{base}/cloud_statistics.txt", "w", encoding="utf-8") as handle:
        for key in sorted(stats):
            handle.write(f"{key}: {stats[key]}\n")

    summaries = []
    summaries.append(run_case("A_real_cloud_obstacle_on", True, state, check_pub, waypoint_pub,
                              f"{base}/case_a_real_cloud_obstacle_on.log"))
    summaries.append(run_case("B_real_cloud_obstacle_off", False, state, check_pub, waypoint_pub,
                              f"{base}/case_b_real_cloud_obstacle_off.log"))
    check_pub.publish(Bool(data=True))

    with open(f"{base}/falco_path_diagnostics.txt", "w", encoding="utf-8") as handle:
        for summary in summaries:
            handle.write(f"[{summary['case']}]\n")
            for key in sorted(summary):
                handle.write(f"{key}: {summary[key]}\n")
            handle.write("\n")


if __name__ == "__main__":
    main()
