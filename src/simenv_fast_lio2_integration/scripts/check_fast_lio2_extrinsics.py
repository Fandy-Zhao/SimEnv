#!/usr/bin/env python3
"""Verify SimEnv's configured FAST-LIO2 LiDAR-to-IMU extrinsic.

FAST-LIO2 applies a point transform as ``p_imu = R_L_I * p_lidar + T_L_I``.
This checker derives its expected values from the URDF LiDAR mounting pose and
the body-aligned trunk IMU convention, so a reversed rotation or translation
is caught before mapping starts.
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET

import yaml


TOLERANCE = 2e-3


def _rotation_y(angle):
    cosine, sine = math.cos(angle), math.sin(angle)
    return [[cosine, 0.0, sine], [0.0, 1.0, 0.0], [-sine, 0.0, cosine]]


def _find_lidar_origin(robot_xacro):
    root = ET.parse(robot_xacro).getroot()
    for joint in root.iter("joint"):
        if joint.get("name") != "laser_livox_joint":
            continue
        origin = joint.find("origin")
        if origin is None:
            raise ValueError("laser_livox_joint has no origin")
        return ([float(value) for value in origin.get("xyz").split()],
                [float(value) for value in origin.get("rpy").split()])
    raise ValueError("laser_livox_joint not found")


def _assert_close(name, actual, expected):
    if len(actual) != len(expected):
        raise ValueError("%s size is invalid" % name)
    errors = [abs(left - right) for left, right in zip(actual, expected)]
    if max(errors) > TOLERANCE:
        raise ValueError("%s mismatch: got %s, expected %s" %
                         (name, actual, expected))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, help="FAST-LIO2 YAML config")
    parser.add_argument("--robot-xacro", required=True, help="SimEnv robot.xacro")
    args = parser.parse_args()

    with open(args.config, encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    translation_base_lidar, rpy_base_lidar = _find_lidar_origin(args.robot_xacro)
    if abs(rpy_base_lidar[0]) > TOLERANCE or abs(rpy_base_lidar[2]) > TOLERANCE:
        raise ValueError("only a Y-axis LiDAR mount is supported by this checker")

    # TF's imu_link -> laser_livox pose is the coordinate transform
    # p_imu = R_imu_lidar * p_lidar + T_imu_lidar.  FAST-LIO2 consumes this
    # exact LiDAR->IMU point transform; it must not be inverted.
    expected_rotation = _rotation_y(rpy_base_lidar[1])
    expected_translation = translation_base_lidar
    mapping = config["mapping"]
    configured_rotation = mapping["extrinsic_R"]
    configured_translation = mapping["extrinsic_T"]
    _assert_close("extrinsic_R", configured_rotation,
                  [value for row in expected_rotation for value in row])
    _assert_close("extrinsic_T", configured_translation, expected_translation)
    if config["common"]["imu_topic"] != "/trunk_imu":
        raise ValueError("common.imu_topic must be /trunk_imu (body-aligned)")

    print("PASS: FAST-LIO2 LiDAR->IMU extrinsic matches robot.xacro")
    print("  R_L_I = Ry({:.6f} rad), T_L_I = [{:.6f}, {:.6f}, {:.6f}]".format(
        rpy_base_lidar[1], *expected_translation))


if __name__ == "__main__":
    try:
        main()
    except (KeyError, TypeError, ValueError, ET.ParseError) as error:
        print("FAIL: {}".format(error), file=sys.stderr)
        sys.exit(1)
