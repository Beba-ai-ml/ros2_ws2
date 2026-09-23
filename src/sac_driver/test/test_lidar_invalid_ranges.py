"""Regression checks for missing lidar returns; no ROS or hardware required."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sac_driver.lidar_converter import LidarConverter


class InvalidLidarRanges(unittest.TestCase):
    def scan(self, ranges):
        return SimpleNamespace(
            ranges=ranges, angle_min=0.0, angle_increment=math.radians(1.0),
            range_min=0.05, range_max=40.0,
        )

    def converter(self, angle=0.5, interpolation=True):
        return LidarConverter(
            [angle], max_range_m=20.0, angle_offset_deg=0.0,
            angle_direction=1.0, use_interpolation=interpolation,
        )

    def test_two_valid_samples_keep_linear_interpolation(self):
        out = self.converter().convert(self.scan([0.4, 0.6]))
        self.assertAlmostEqual(float(out[0]) * 20.0, 0.5, places=6)

    def test_invalid_neighbor_does_not_erase_nearby_obstacle(self):
        for bad in [math.inf, -math.inf, math.nan, 0.0, -1.0, 0.01, 45.0]:
            for ranges in ([0.5, bad], [bad, 0.5]):
                with self.subTest(ranges=ranges):
                    out = self.converter().convert(self.scan(ranges))
                    self.assertAlmostEqual(float(out[0]) * 20.0, 0.5, places=6)

    def test_zero_weight_invalid_neighbor_preserves_exact_sample(self):
        out = self.converter(angle=0.0).convert(self.scan([0.5, math.inf]))
        self.assertAlmostEqual(float(out[0]) * 20.0, 0.5, places=6)

    def test_no_extension_across_two_invalid_endpoints(self):
        out = self.converter(angle=1.5).convert(self.scan([0.5, math.inf, math.inf, 0.5]))
        self.assertEqual(float(out[0]), 1.0)

    def test_short_bounded_gap_uses_closer_return(self):
        scan = self.scan([0.5, math.inf, math.inf, 0.8])
        scan.angle_increment = math.radians(0.5)
        out = self.converter(angle=0.75).convert(scan)
        self.assertAlmostEqual(float(out[0]) * 20.0, 0.5, places=6)

    def test_gap_fill_can_be_disabled(self):
        scan = self.scan([0.5, math.inf, math.inf, 0.8])
        scan.angle_increment = math.radians(0.5)
        converter = self.converter(angle=0.75)
        converter.max_invalid_gap_deg = 0.0
        self.assertEqual(float(converter.convert(scan)[0]), 1.0)

    def test_unbounded_gap_is_not_filled(self):
        scan = self.scan([math.inf, math.inf, 0.5])
        scan.angle_increment = math.radians(0.5)
        out = self.converter(angle=0.25).convert(scan)
        self.assertEqual(float(out[0]), 1.0)

    def test_invalid_gap_width_is_rejected(self):
        for gap in [-1.0, math.nan, math.inf]:
            with self.subTest(gap=gap):
                with self.assertRaises(ValueError):
                    LidarConverter([0.0], 20.0, max_invalid_gap_deg=gap)

    def test_no_history_delays_appearing_or_disappearing_obstacle(self):
        converter = self.converter()
        for distance in [30.0, 0.5, 30.0, 0.4]:
            out = converter.convert(self.scan([distance, distance]))
            self.assertAlmostEqual(float(out[0]) * 20.0, min(distance, 20.0), places=6)

    def test_unusable_scan_raises_instead_of_claiming_clear_space(self):
        for ranges in [[], [math.inf, math.nan], [0.0, -1.0], [0.01, 45.0]]:
            with self.subTest(ranges=ranges):
                with self.assertRaises(ValueError):
                    self.converter().convert(self.scan(ranges))

    def test_nearest_mode_keeps_missing_ray_at_max_range(self):
        out = self.converter(angle=0.0, interpolation=False).convert(self.scan([math.inf, 0.5]))
        self.assertEqual(float(out[0]), 1.0)

    def test_sensor_valid_distance_is_clipped_to_model_range(self):
        out = self.converter().convert(self.scan([30.0, 30.0]))
        self.assertEqual(float(out[0]), 1.0)

    def test_caller_ranges_are_unchanged(self):
        ranges = np.asarray([0.5, np.nan], dtype=np.float32)
        original = ranges.copy()
        self.converter().convert(self.scan(ranges))
        np.testing.assert_equal(ranges, original)

    def test_invalid_angle_increment_is_rejected(self):
        for increment in [0.0, -1.0, math.inf, math.nan]:
            scan = self.scan([1.0, 1.0])
            scan.angle_increment = increment
            with self.subTest(increment=increment):
                with self.assertRaises(ValueError):
                    self.converter().convert(scan)


if __name__ == '__main__':
    unittest.main()
