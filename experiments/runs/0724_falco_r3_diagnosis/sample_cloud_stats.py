#!/usr/bin/env python3
import math
import statistics
import sys

import rospy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


def main():
    out_path = sys.argv[1]
    rospy.init_node("sample_falco_cloud_stats_once", anonymous=True, disable_signals=True)
    odom = rospy.wait_for_message("/navigation/state_estimation", Odometry, timeout=30.0)
    cloud = rospy.wait_for_message("/navigation/registered_scan", PointCloud2, timeout=30.0)
    px = odom.pose.pose.position.x
    py = odom.pose.pose.position.y
    pz = odom.pose.pose.position.z
    distances = []
    heights = []
    adjacent = 0
    sampled = 0
    for x, y, z in pc2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True):
        dx = x - px
        dy = y - py
        dz = z - pz
        dist = math.hypot(dx, dy)
        distances.append(dist)
        heights.append(dz)
        if dist < 3.5:
            adjacent += 1
        sampled += 1
        if sampled >= 50000:
            break
    distances.sort()
    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write(f"odom_frame: {odom.header.frame_id}\n")
        handle.write(f"cloud_frame: {cloud.header.frame_id}\n")
        handle.write(f"cloud_width: {cloud.width}\n")
        handle.write(f"cloud_height: {cloud.height}\n")
        handle.write(f"raw_cloud_point_count: {cloud.width * cloud.height}\n")
        handle.write(f"sampled_finite_point_count: {sampled}\n")
        handle.write(f"adjacentRange_point_count_sampled: {adjacent}\n")
        if distances:
            handle.write(f"distance_min: {distances[0]}\n")
            handle.write(f"distance_median: {statistics.median(distances)}\n")
            handle.write(f"distance_p90: {distances[int(0.9 * (len(distances) - 1))]}\n")
        if heights:
            handle.write(f"height_min: {min(heights)}\n")
            handle.write(f"height_max: {max(heights)}\n")


if __name__ == "__main__":
    main()
