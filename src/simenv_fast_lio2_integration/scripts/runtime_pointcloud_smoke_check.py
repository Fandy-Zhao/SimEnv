#!/usr/bin/env python3
"""Runtime smoke checks for the SimEnv FAST-LIO2 pointcloud chain."""

import argparse
import math
import os
import re
import subprocess
import sys
import time

import rospy
import sensor_msgs.point_cloud2 as pc2
import tf
from sensor_msgs.msg import PointCloud, PointCloud2


EXPECTED_FIX_MERGE = "69ff34e7"
POINT_TOLERANCE = 1e-6


def _run(command, cwd=None):
    return subprocess.check_output(command, cwd=cwd, text=True).strip()


def _package_root():
    return _run(["rospack", "find", "simenv_fast_lio2_integration"])


def _repo_root(package_root):
    return os.path.abspath(os.path.join(package_root, "..", ".."))


def _check_git_contains_fix(repo):
    head = _run(["git", "rev-parse", "--short", "HEAD"], cwd=repo)
    code = subprocess.call(
        ["git", "merge-base", "--is-ancestor", EXPECTED_FIX_MERGE, "HEAD"],
        cwd=repo,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if code != 0:
        raise RuntimeError("HEAD %s does not contain %s" %
                           (head, EXPECTED_FIX_MERGE))
    print("git_head=%s contains_%s=true" % (head, EXPECTED_FIX_MERGE))


def _adapter_node_and_pid():
    nodes = _run(["rosnode", "list"]).splitlines()
    adapter_nodes = [node for node in nodes if "scan_to_pointcloud2" in node]
    if len(adapter_nodes) != 1:
        raise RuntimeError("expected one scan_to_pointcloud2 node, got %r" %
                           adapter_nodes)
    node = adapter_nodes[0]
    info = _run(["rosnode", "info", node])
    match = re.search(r"Pid:\s*(\d+)", info)
    if not match:
        raise RuntimeError("could not parse adapter PID from rosnode info")
    pid = int(match.group(1))
    return node, pid, info


def _read_proc(pid):
    with open("/proc/%d/cmdline" % pid, "rb") as stream:
        cmdline = stream.read().replace(b"\0", b" ").decode().strip()
    cwd = os.readlink("/proc/%d/cwd" % pid)
    return cmdline, cwd


def _check_adapter_runtime(package_root):
    node, pid, info = _adapter_node_and_pid()
    cmdline, cwd = _read_proc(pid)
    expected_script = os.path.join(package_root, "scripts",
                                   "scan_to_pointcloud2.py")
    if expected_script not in cmdline:
        raise RuntimeError("adapter cmdline does not use expected script: %s" %
                           cmdline)
    params = _run(["rosparam", "list"]).splitlines()
    private_params = [param for param in params if param.startswith(node + "/")]
    values = {}
    for name in private_params:
        values[name] = _run(["rosparam", "get", name])
    for suffix in ("rotation_y_deg", "rotation_x_deg"):
        matches = [name for name in values if name.endswith("/" + suffix)]
        if matches and abs(float(values[matches[0]])) > POINT_TOLERANCE:
            raise RuntimeError("%s is active: %s" %
                               (matches[0], values[matches[0]]))
    print("adapter_node=%s" % node)
    print("adapter_pid=%d" % pid)
    print("adapter_cmdline=%s" % cmdline)
    print("adapter_cwd=%s" % cwd)
    print("adapter_private_params=%s" % values)
    print("adapter_rosnode_info_begin\n%s\nadapter_rosnode_info_end" % info)


def _stamp_key(stamp):
    return stamp.secs, stamp.nsecs


def _compare_clouds(timeout_sec):
    scans = {}
    result = {}

    def remember_scan(msg):
        scans[_stamp_key(msg.header.stamp)] = msg
        if len(scans) > 100:
            for key in sorted(scans)[:-100]:
                scans.pop(key, None)

    def compare(msg):
        key = _stamp_key(msg.header.stamp)
        scan = scans.get(key)
        if scan is None or result:
            return
        points2 = list(pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=False))
        count = min(len(scan.points), len(points2))
        max_error = 0.0
        first_mismatch = None
        samples = []
        for index in range(count):
            left = scan.points[index]
            right = points2[index]
            error = max(abs(left.x - right[0]),
                        abs(left.y - right[1]),
                        abs(left.z - right[2]))
            if index < 3:
                samples.append((index, left.x, left.y, left.z,
                                right[0], right[1], right[2]))
            max_error = max(max_error, error)
            if error > POINT_TOLERANCE and first_mismatch is None:
                first_mismatch = (index, left.x, left.y, left.z,
                                  right[0], right[1], right[2], error)
        result.update({
            "stamp": key,
            "scan_frame": scan.header.frame_id,
            "pc2_frame": msg.header.frame_id,
            "scan_points": len(scan.points),
            "pc2_points": len(points2),
            "compared": count,
            "max_error": max_error,
            "first_mismatch": first_mismatch,
            "samples": samples,
            "fields": [(field.name, field.offset, field.datatype, field.count)
                       for field in msg.fields],
        })

    rospy.Subscriber("/scan", PointCloud, remember_scan, queue_size=50)
    rospy.Subscriber("/scan_pointcloud2", PointCloud2, compare, queue_size=50)
    deadline = time.time() + timeout_sec
    while not rospy.is_shutdown() and not result and time.time() < deadline:
        time.sleep(0.01)
    if not result:
        raise RuntimeError("no matching /scan and /scan_pointcloud2 stamp "
                           "within %.1fs; cached scan stamps=%d" %
                           (timeout_sec, len(scans)))
    if result["scan_frame"] != "laser_livox" or result["pc2_frame"] != "laser_livox":
        raise RuntimeError("unexpected frames: scan=%s pc2=%s" %
                           (result["scan_frame"], result["pc2_frame"]))
    if result["scan_points"] != result["pc2_points"]:
        raise RuntimeError("point count mismatch: scan=%d pc2=%d" %
                           (result["scan_points"], result["pc2_points"]))
    if result["max_error"] > POINT_TOLERANCE:
        raise RuntimeError("point mismatch: %r" % (result["first_mismatch"],))
    print("matched_stamp=%d.%09d" % result["stamp"])
    print("scan_frame=%s pc2_frame=%s" %
          (result["scan_frame"], result["pc2_frame"]))
    print("scan_points=%d pc2_points=%d compared=%d max_abs_err=%.9g" %
          (result["scan_points"], result["pc2_points"],
           result["compared"], result["max_error"]))
    print("sample_pairs=%r" % (result["samples"],))
    print("pc2_fields=%r" % (result["fields"],))


def _check_lidar_tf(timeout_sec):
    listener = tf.TransformListener()
    deadline = time.time() + timeout_sec
    last_error = None
    while time.time() < deadline and not rospy.is_shutdown():
        try:
            trans, quat = listener.lookupTransform(
                "base", "laser_livox", rospy.Time(0))
            break
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as error:
            last_error = error
            time.sleep(0.05)
    else:
        raise RuntimeError("base->laser_livox TF unavailable: %s" % last_error)
    matrix = tf.transformations.quaternion_matrix(quat)
    ex_base = [matrix[row][0] for row in range(3)]
    if abs(trans[0] - 0.2) > 2e-3 or abs(trans[1]) > 2e-3 or abs(trans[2] - 0.08) > 2e-3:
        raise RuntimeError("unexpected base->laser_livox translation: %r" %
                           (trans,))
    if ex_base[0] <= 0.6 or ex_base[2] >= -0.5:
        raise RuntimeError("LiDAR +X is not forward/down in base: %r" %
                           (ex_base,))
    print("tf_base_laser_livox_translation=%r" % (trans,))
    print("tf_lidar_plus_x_in_base=%r" % (ex_base,))


def _check_registered_cloud(timeout_sec, max_angle_deg):
    if timeout_sec <= 0:
        print("cloud_registered_check=skipped")
        return
    try:
        import numpy as np
    except ImportError:
        raise RuntimeError("numpy is required for /cloud_registered plane check")
    try:
        msg = rospy.wait_for_message(
            "/cloud_registered", PointCloud2, timeout=timeout_sec)
    except rospy.ROSException as error:
        raise RuntimeError("/cloud_registered unavailable: %s" % error)
    points = []
    for point in pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True):
        if all(abs(value) < 1e6 for value in point):
            points.append(point)
        if len(points) >= 30000:
            break
    if len(points) < 100:
        raise RuntimeError("not enough /cloud_registered points: %d" %
                           len(points))
    array = np.asarray(points, dtype=float)
    z_threshold = np.quantile(array[:, 2], 0.25)
    ground = array[array[:, 2] <= z_threshold]
    if ground.shape[0] > 8000:
        ground = ground[:8000]
    system = np.column_stack(
        [ground[:, 0], ground[:, 1], np.ones(ground.shape[0])])
    coefficients, _, _, _ = np.linalg.lstsq(
        system, ground[:, 2], rcond=None)
    slope_x, slope_y, intercept = coefficients
    normal = np.array([-slope_x, -slope_y, 1.0])
    normal = normal / np.linalg.norm(normal)
    if normal[2] < 0:
        normal = -normal
    angle = math.degrees(math.acos(max(-1.0, min(1.0, normal[2]))))
    if angle > max_angle_deg:
        raise RuntimeError("/cloud_registered ground normal angle %.3f > %.3f" %
                           (angle, max_angle_deg))
    print("cloud_registered_frame=%s stamp=%d.%09d points=%d ground_points=%d" %
          (msg.header.frame_id, msg.header.stamp.secs, msg.header.stamp.nsecs,
           array.shape[0], ground.shape[0]))
    print("cloud_registered_ground_plane_z=%.6f*x + %.6f*y + %.6f" %
          (slope_x, slope_y, intercept))
    print("cloud_registered_ground_normal=%r angle_from_+Z_deg=%.3f" %
          (normal.tolist(), angle))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--cloud-registered-timeout", type=float, default=0.0,
                        help="Also validate /cloud_registered ground plane "
                             "when > 0 seconds.")
    parser.add_argument("--max-ground-normal-angle-deg", type=float,
                        default=5.0)
    args = parser.parse_args()
    rospy.init_node("runtime_pointcloud_smoke_check", anonymous=True)
    package_root = _package_root()
    repo = _repo_root(package_root)
    _check_git_contains_fix(repo)
    _check_adapter_runtime(package_root)
    _compare_clouds(args.timeout)
    _check_lidar_tf(args.timeout)
    _check_registered_cloud(args.cloud_registered_timeout,
                            args.max_ground_normal_angle_deg)
    print("PASS: runtime pointcloud adapter preserves laser_livox coordinates")


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("FAIL: %s" % error, file=sys.stderr)
        sys.exit(1)
