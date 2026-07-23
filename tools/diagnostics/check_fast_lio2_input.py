#!/usr/bin/env python3
"""Check FAST-LIO2 simulated input continuity from ROS topics."""

import argparse
import math
import struct
import sys
import time
from dataclasses import dataclass

try:
    import numpy as np
except Exception:
    np = None


FLOAT32 = 7


@dataclass
class CloudFrameStats:
    stamp: float
    frame_id: str
    width: int
    height: int
    point_step: int
    row_step: int
    total_points: int
    finite_xyz_count: int
    nonzero_xyz_count: int
    range_above_blind_count: int
    min_range: float
    max_range: float
    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float
    max_z: float
    inter_frame_ros_time: float
    inter_frame_wall_time: float


def _stamp_to_float(stamp):
    if hasattr(stamp, "to_sec"):
        return float(stamp.to_sec())
    secs = getattr(stamp, "secs", 0)
    nsecs = getattr(stamp, "nsecs", 0)
    return float(secs) + float(nsecs) * 1e-9


def _float_field_offsets(cloud):
    offsets = {}
    for field in cloud.fields:
        if field.name in ("x", "y", "z") and field.datatype == FLOAT32:
            offsets[field.name] = field.offset
    missing = sorted(set(("x", "y", "z")) - set(offsets))
    if missing:
        raise ValueError("missing FLOAT32 field(s): %s" % ", ".join(missing))
    return offsets


def _iter_xyz(cloud):
    offsets = _float_field_offsets(cloud)
    data = cloud.data
    point_step = int(cloud.point_step)
    total = int(cloud.width) * int(cloud.height)
    endian = ">" if getattr(cloud, "is_bigendian", False) else "<"
    for index in range(total):
        base = index * point_step
        if base + point_step > len(data):
            break
        yield (
            struct.unpack_from(endian + "f", data, base + offsets["x"])[0],
            struct.unpack_from(endian + "f", data, base + offsets["y"])[0],
            struct.unpack_from(endian + "f", data, base + offsets["z"])[0],
        )


def _xyz_arrays(cloud):
    if np is None:
        return None
    offsets = _float_field_offsets(cloud)
    total = int(cloud.width) * int(cloud.height)
    if total <= 0:
        empty = np.array([], dtype=np.float32)
        return empty, empty, empty
    endian = ">" if getattr(cloud, "is_bigendian", False) else "<"
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [endian + "f4", endian + "f4", endian + "f4"],
            "offsets": [offsets["x"], offsets["y"], offsets["z"]],
            "itemsize": int(cloud.point_step),
        }
    )
    available = len(cloud.data) // int(cloud.point_step)
    count = min(total, available)
    arr = np.frombuffer(cloud.data, dtype=dtype, count=count)
    return arr["x"], arr["y"], arr["z"]


def summarize_cloud(cloud, blind, previous_stamp=None, previous_wall_time=None, wall_time=None):
    stamp = _stamp_to_float(cloud.header.stamp)
    frame_id = getattr(cloud.header, "frame_id", "")
    wall_time = time.time() if wall_time is None else float(wall_time)
    total_points = int(cloud.width) * int(cloud.height)

    arrays = _xyz_arrays(cloud)
    if arrays is not None:
        x_arr, y_arr, z_arr = arrays
        finite_mask = np.isfinite(x_arr) & np.isfinite(y_arr) & np.isfinite(z_arr)
        finite_count = int(np.count_nonzero(finite_mask))
        finite_x = x_arr[finite_mask]
        finite_y = y_arr[finite_mask]
        finite_z = z_arr[finite_mask]
        ranges = np.sqrt(finite_x * finite_x + finite_y * finite_y + finite_z * finite_z)
        nonzero_count = int(np.count_nonzero((finite_x != 0.0) | (finite_y != 0.0) | (finite_z != 0.0)))
        above_blind_count = int(np.count_nonzero(ranges > blind))
        nan = float("nan")
        min_range = float(np.min(ranges)) if ranges.size else nan
        max_range = float(np.max(ranges)) if ranges.size else nan
        min_x = float(np.min(finite_x)) if finite_x.size else nan
        max_x = float(np.max(finite_x)) if finite_x.size else nan
        min_y = float(np.min(finite_y)) if finite_y.size else nan
        max_y = float(np.max(finite_y)) if finite_y.size else nan
        min_z = float(np.min(finite_z)) if finite_z.size else nan
        max_z = float(np.max(finite_z)) if finite_z.size else nan
    else:
        finite_count = 0
        nonzero_count = 0
        above_blind_count = 0
        ranges = []
        xs = []
        ys = []
        zs = []
        for x, y, z in _iter_xyz(cloud):
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            finite_count += 1
            xs.append(x)
            ys.append(y)
            zs.append(z)
            rng = math.sqrt(x * x + y * y + z * z)
            ranges.append(rng)
            if x != 0.0 or y != 0.0 or z != 0.0:
                nonzero_count += 1
            if rng > blind:
                above_blind_count += 1
        nan = float("nan")
        min_range = min(ranges) if ranges else nan
        max_range = max(ranges) if ranges else nan
        min_x = min(xs) if xs else nan
        max_x = max(xs) if xs else nan
        min_y = min(ys) if ys else nan
        max_y = max(ys) if ys else nan
        min_z = min(zs) if zs else nan
        max_z = max(zs) if zs else nan

    return CloudFrameStats(
        stamp=stamp,
        frame_id=frame_id,
        width=int(cloud.width),
        height=int(cloud.height),
        point_step=int(cloud.point_step),
        row_step=int(cloud.row_step),
        total_points=total_points,
        finite_xyz_count=finite_count,
        nonzero_xyz_count=nonzero_count,
        range_above_blind_count=above_blind_count,
        min_range=min_range,
        max_range=max_range,
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
        min_z=min_z,
        max_z=max_z,
        inter_frame_ros_time=(stamp - previous_stamp) if previous_stamp is not None else nan,
        inter_frame_wall_time=(wall_time - previous_wall_time) if previous_wall_time is not None else nan,
    )


class FastLio2InputMonitor:
    def __init__(self, args, rospy, PointCloud2, Imu, Odometry):
        self.args = args
        self.rospy = rospy
        self.end_monotonic = time.monotonic() + args.duration
        self.frames = []
        self.imu_stamps = []
        self.odom_count = 0
        self.cloud_registered_count = 0
        self.last_wall = None
        self.max_ros_gap = 0.0
        self.max_wall_gap = 0.0
        self.timestamp_regressions = 0

        self.pc_sub = rospy.Subscriber(args.pointcloud_topic, PointCloud2, self.cloud_cb, queue_size=1)
        self.imu_sub = rospy.Subscriber(args.imu_topic, Imu, self.imu_cb, queue_size=200)
        self.odom_sub = rospy.Subscriber(args.odometry_topic, Odometry, self.odom_cb, queue_size=20)
        self.reg_sub = rospy.Subscriber(args.cloud_registered_topic, PointCloud2, self.registered_cb, queue_size=20)

    def cloud_cb(self, msg):
        if time.monotonic() > self.end_monotonic:
            return
        if len(self.frames) >= self.args.max_stored_frames:
            return
        prev = self.frames[-1] if self.frames else None
        now = time.time()
        stats = summarize_cloud(
            msg,
            self.args.blind,
            previous_stamp=prev.stamp if prev else None,
            previous_wall_time=self.last_wall,
            wall_time=now,
        )
        if prev:
            if stats.inter_frame_ros_time < 0:
                self.timestamp_regressions += 1
            self.max_ros_gap = max(self.max_ros_gap, stats.inter_frame_ros_time)
            self.max_wall_gap = max(self.max_wall_gap, stats.inter_frame_wall_time)
        self.last_wall = now
        self.frames.append(stats)

    def imu_cb(self, msg):
        self.imu_stamps.append(_stamp_to_float(msg.header.stamp))

    def odom_cb(self, _msg):
        self.odom_count += 1

    def registered_cb(self, _msg):
        self.cloud_registered_count += 1

    def wait(self):
        while time.monotonic() < self.end_monotonic and not self.rospy.is_shutdown():
            time.sleep(0.1)

    def close(self):
        for sub in (self.pc_sub, self.imu_sub, self.odom_sub, self.reg_sub):
            try:
                sub.unregister()
            except Exception:
                pass
        try:
            self.rospy.signal_shutdown("check complete")
        except Exception:
            pass

    def verdict(self):
        frames = self.frames
        if not frames:
            return 1, "pointcloud absent"
        if len(frames) < self.args.min_frames:
            return 1, "insufficient pointcloud frames"
        if self.timestamp_regressions:
            return 2, "pointcloud timestamp regression"
        duplicate_stamps = len(frames) - len({frame.stamp for frame in frames})
        if duplicate_stamps:
            return 2, "pointcloud duplicate timestamps"
        if self.max_ros_gap > self.args.max_ros_gap or self.max_wall_gap > self.args.max_wall_gap:
            return 1, "pointcloud interrupted"
        if any(frame.finite_xyz_count == 0 for frame in frames):
            return 3, "pointcloud all-nonfinite frame"
        if any(frame.range_above_blind_count == 0 for frame in frames):
            return 3, "pointcloud all-inside-blind frame"
        if not self.imu_stamps:
            return 4, "IMU absent"
        if len(set(self.imu_stamps)) < self.args.min_imu_stamps:
            return 4, "IMU insufficient unique stamps"
        if self.odom_count <= 0 or self.cloud_registered_count <= 0:
            return 5, "FAST-LIO2 odometry/cloud_registered absent"
        return 0, "continuity pass"

    def print_report(self):
        frames = self.frames
        duplicate_stamps = len(frames) - len({frame.stamp for frame in frames})
        empty = sum(1 for frame in frames if frame.total_points == 0)
        all_nonfinite = sum(1 for frame in frames if frame.finite_xyz_count == 0)
        all_inside_blind = sum(1 for frame in frames if frame.range_above_blind_count == 0)

        for index, frame in enumerate(frames, start=1):
            print(
                "frame=%d stamp=%.9f frame_id=%s width=%d height=%d point_step=%d "
                "row_step=%d total_points=%d finite_xyz=%d nonzero_xyz=%d "
                "above_blind=%d min_range=%.6f max_range=%.6f "
                "x=[%.6f,%.6f] y=[%.6f,%.6f] z=[%.6f,%.6f] "
                "dt_ros=%.6f dt_wall=%.6f"
                % (
                    index, frame.stamp, frame.frame_id, frame.width, frame.height,
                    frame.point_step, frame.row_step, frame.total_points,
                    frame.finite_xyz_count, frame.nonzero_xyz_count,
                    frame.range_above_blind_count, frame.min_range, frame.max_range,
                    frame.min_x, frame.max_x, frame.min_y, frame.max_y,
                    frame.min_z, frame.max_z, frame.inter_frame_ros_time,
                    frame.inter_frame_wall_time,
                )
            )

        print("summary received_frames=%d" % len(frames))
        print("summary unique_stamps=%d" % len({frame.stamp for frame in frames}))
        print("summary duplicate_stamps=%d" % duplicate_stamps)
        print("summary timestamp_regressions=%d" % self.timestamp_regressions)
        print("summary max_ros_time_gap=%.6f" % self.max_ros_gap)
        print("summary max_wall_time_gap=%.6f" % self.max_wall_gap)
        print("summary empty_frames=%d" % empty)
        print("summary all_nonfinite_frames=%d" % all_nonfinite)
        print("summary all_inside_blind_frames=%d" % all_inside_blind)
        print("summary imu_messages=%d" % len(self.imu_stamps))
        print("summary imu_unique_stamps=%d" % len(set(self.imu_stamps)))
        print("summary odometry_messages=%d" % self.odom_count)
        print("summary cloud_registered_messages=%d" % self.cloud_registered_count)


def parse_args(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pointcloud-topic", default="/scan_pointcloud2")
    parser.add_argument("--imu-topic", default="/trunk_imu")
    parser.add_argument("--odometry-topic", default="/Odometry")
    parser.add_argument("--cloud-registered-topic", default="/cloud_registered")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--min-frames", type=int, default=20)
    parser.add_argument("--min-imu-stamps", type=int, default=20)
    parser.add_argument("--blind", type=float, default=0.5)
    parser.add_argument("--max-ros-gap", type=float, default=2.0)
    parser.add_argument("--max-wall-gap", type=float, default=5.0)
    parser.add_argument("--max-stored-frames", type=int, default=400)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, PointCloud2

    rospy.init_node("check_fast_lio2_input", anonymous=True)
    monitor = FastLio2InputMonitor(args, rospy, PointCloud2, Imu, Odometry)
    try:
        monitor.wait()
        monitor.print_report()
        code, reason = monitor.verdict()
        print("verdict code=%d reason=%s" % (code, reason))
        return code
    finally:
        monitor.close()


if __name__ == "__main__":
    sys.exit(main())
