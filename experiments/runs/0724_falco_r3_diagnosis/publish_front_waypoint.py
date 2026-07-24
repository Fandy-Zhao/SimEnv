#!/usr/bin/env python3
import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def main():
    out_path = sys.argv[1]
    rospy.init_node("publish_front_waypoint_once", anonymous=True)
    odom = rospy.wait_for_message("/navigation/state_estimation", Odometry, timeout=30.0)
    yaw = yaw_from_quat(odom.pose.pose.orientation)
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = odom.header.frame_id
    goal.pose.position.x = odom.pose.pose.position.x + 0.8 * math.cos(yaw)
    goal.pose.position.y = odom.pose.pose.position.y + 0.8 * math.sin(yaw)
    goal.pose.position.z = odom.pose.pose.position.z
    goal.pose.orientation.w = 1.0
    pub = rospy.Publisher("/navigation/way_point", PoseStamped, queue_size=1, latch=True)
    deadline = rospy.Time.now() + rospy.Duration(2.0)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        pub.publish(goal)
        rospy.sleep(0.1)
    with open(out_path, "w", encoding="utf-8") as handle:
        handle.write(f"wall_time: {rospy.get_time()}\n")
        handle.write(f"sim_time: {goal.header.stamp.to_sec()}\n")
        handle.write(f"frame_id: {goal.header.frame_id}\n")
        handle.write(f"odom_x: {odom.pose.pose.position.x}\n")
        handle.write(f"odom_y: {odom.pose.pose.position.y}\n")
        handle.write(f"odom_z: {odom.pose.pose.position.z}\n")
        handle.write(f"odom_yaw: {yaw}\n")
        handle.write(f"goal_x: {goal.pose.position.x}\n")
        handle.write(f"goal_y: {goal.pose.position.y}\n")
        handle.write(f"goal_z: {goal.pose.position.z}\n")


if __name__ == "__main__":
    main()
