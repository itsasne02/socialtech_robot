"""Offline ROS-message tests; creates no ROS node or publishers."""
import json
import math
from types import SimpleNamespace
import unittest

from sensor_msgs.msg import LaserScan
from depth_lidar_probe import ComparisonProbe, finite_json


class LaserDiagnosticTest(unittest.TestCase):
    def plane_scan(self, clockwise):
        msg = LaserScan()
        msg.angle_min = math.pi if clockwise else -math.pi
        msg.angle_increment = math.radians(-1 if clockwise else 1)
        msg.range_min, msg.range_max = 0.1, 10.0
        values = []
        for i in range(360):
            a = msg.angle_min + i * msg.angle_increment
            values.append(1.5 / math.cos(a) if math.cos(a) > 0.5 else math.inf)
        msg.ranges = values
        return msg

    def measure(self, msg):
        probe = SimpleNamespace(scan_count=0, scan_times=[], last_saved_scan=-math.inf, scans=[])
        ComparisonProbe.on_scan(probe, msg)
        return probe.scans[0]

    def test_plane_both_scan_directions(self):
        for clockwise in (False, True):
            result = self.measure(self.plane_scan(clockwise))
            self.assertGreater(result['sector_valid'], 10)
            self.assertAlmostEqual(result['sector_x_m']['median'], 1.5, places=6)
            self.assertGreater(result['sector_range_m']['median'], 1.5)

    def test_invalid_and_out_of_range_do_not_become_obstacles(self):
        msg = self.plane_scan(True)
        ranges = list(msg.ranges)
        ranges[175:182] = [math.nan, math.inf, -math.inf, 0.0, -1.0, 0.05, 11.0]
        msg.ranges = ranges
        result = self.measure(msg)
        self.assertAlmostEqual(result['sector_x_m']['median'], 1.5, places=6)
        serialized = json.dumps(finite_json(result), allow_nan=False)
        self.assertIn('"nan"', serialized)
        self.assertIn('"inf"', serialized)

    def test_empty_sector_is_not_free_space(self):
        msg = self.plane_scan(False)
        msg.ranges = [math.inf] * 360
        result = self.measure(msg)
        self.assertEqual(result['sector_valid'], 0)
        self.assertIsNone(result['sector_x_m'])


if __name__ == '__main__':
    unittest.main()
