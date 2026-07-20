#!/usr/bin/env python3
"""Static regression checks for auto.sh world-mode wiring."""

from pathlib import Path
import unittest


WORKSPACE = Path(__file__).resolve().parents[3]
AUTO_SH = WORKSPACE / "auto.sh"


class AutoWorldModeStaticTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.text = AUTO_SH.read_text()

    def test_default_world_mode_is_competition(self):
        self.assertIn('WORLD_MODE="${WORLD_MODE:-competition}"', self.text)

    def test_earth_world_uses_repository_relative_resolution(self):
        self.assertIn(
            'EARTH_WORLD_FILE="$WORKSPACE_DIR/src/unitree_guide/unitree_ros/unitree_gazebo/worlds/earth.world"',
            self.text,
        )
        self.assertNotIn("/home/zzf", self.text)

    def test_earth_mode_skips_competition_generator(self):
        generator_index = self.text.index('echo "Generating competition scene..."')
        branch_index = self.text.rfind('if [ "$WORLD_MODE" = "competition" ]; then', 0, generator_index)
        self.assertGreater(branch_index, -1)
        self.assertIn('echo "Skipping competition scene generation for WORLD_MODE=earth."', self.text)

    def test_earth_defaults_remain_env_overridable(self):
        expected = [
            'ENABLE_FAST_LIO2="${ENABLE_FAST_LIO2:-0}"',
            'ENABLE_RVIZ="${ENABLE_RVIZ:-0}"',
            'ENABLE_POINTCLOUD_CONVERTER="${ENABLE_POINTCLOUD_CONVERTER:-0}"',
            'START_BUILDING_CONTROL="${START_BUILDING_CONTROL:-0}"',
        ]
        for needle in expected:
            self.assertIn(needle, self.text)


if __name__ == "__main__":
    unittest.main()
