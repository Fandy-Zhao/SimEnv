#!/usr/bin/env python3
"""Static regression tests for the FAST-LIO2 Stage 2 topic contract."""

import os
import unittest
import xml.etree.ElementTree as ET


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_FILE = os.path.join(
    PACKAGE_ROOT, "launch", "simenv_fast_lio2_mapping.launch")


class Stage2TopicContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.root = ET.parse(LAUNCH_FILE).getroot()

    def test_navigation_topic_defaults(self):
        args = {item.attrib["name"]: item.attrib.get("default")
                for item in self.root.findall("arg")}
        self.assertEqual(args["state_estimation_topic"], "/state_estimation")
        self.assertEqual(args["registered_scan_topic"], "/registered_scan")

    def test_transparent_relays_preserve_legacy_topics(self):
        nodes = {item.attrib.get("name"): item
                 for item in self.root.findall("node")}
        self.assertEqual(nodes["state_estimation_relay"].attrib["pkg"],
                         "topic_tools")
        self.assertEqual(nodes["state_estimation_relay"].attrib["args"],
                         "/Odometry $(arg state_estimation_topic)")
        self.assertEqual(nodes["registered_scan_relay"].attrib["args"],
                         "/cloud_registered $(arg registered_scan_topic)")
        self.assertEqual(nodes["laserMapping"].findall("remap"), [])

    def test_tf_bridge_keeps_native_odometry_input(self):
        node = next(item for item in self.root.findall("node")
                    if item.attrib.get("name") ==
                    "map_to_camera_init_bridge")
        self.assertEqual(node.findall("param"), [])


if __name__ == "__main__":
    unittest.main()
