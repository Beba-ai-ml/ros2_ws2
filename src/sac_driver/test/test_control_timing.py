"""Control callback regressions; source ROS, but never create a ROS node/publisher."""

import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import numpy as np
from rclpy.time import Time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from sac_driver.control_mapper import ControlMapper
from sac_driver.sac_driver_node import SACDriverNode


class ControlTiming(unittest.TestCase):
    def harness(self):
        mapper = ControlMapper(
            max_steering_angle_deg=20.0, max_speed_mps=2.5,
            max_accel_mps2=2.0, speed_limit_mps=0.5,
            steer_rate_limit_deg_s=120.0, accel_rate_limit_mps2=18.0,
            safe_mode=True, safe_speed_limit_mps=0.5,
            safe_steer_scale=1.0, safe_accel_scale=1.0,
            speed_sign=-1.0, default_dt=1.0 / 60.0,
        )
        return SimpleNamespace(
            get_clock=Mock(return_value=Mock(now=Mock(return_value=Time(seconds=180)))),
            get_logger=Mock(), _emergency_stop=False, enabled=True,
            engine=Mock(get_action=Mock(return_value=(-1.0, 2.0))),
            _data_ready=Mock(return_value=True), _needs_reset=True,
            _last_control_time=Time(seconds=10),
            converter=Mock(convert=Mock(return_value=np.ones(450))),
            latest_scan=object(), latest_speed_mps=0.0, _speed_default=0.0,
            latest_linear_accel=0.0, latest_yaw_rate=0.0,
            state_builder=Mock(), control_mapper=mapper,
            _held_action=None, _ticks_since_decision=0, _decision_every_n=8,
            cmd_pub=Mock(), _last_stop_sent=False, _throttled_log=Mock(),
        )

    def test_reenable_excludes_time_with_rb_released(self):
        node = self.harness()
        SACDriverNode._on_timer(node)
        command = node.cmd_pub.publish.call_args[0][0].drive
        self.assertAlmostEqual(command.speed, -0.005, places=6)
        self.assertAlmostEqual(command.steering_angle, -math.radians(2.0), places=6)
        self.assertFalse(node._needs_reset)

    def test_active_tick_uses_elapsed_control_time(self):
        node = self.harness()
        node._needs_reset = False
        node._last_control_time = Time(seconds=179, nanoseconds=980000000)
        node.control_mapper.reset(0.0)
        SACDriverNode._on_timer(node)
        command = node.cmd_pub.publish.call_args[0][0].drive
        self.assertAlmostEqual(command.speed, -0.0072, places=6)
        self.assertAlmostEqual(command.steering_angle, -math.radians(2.4), places=6)

    def test_stop_clears_clock_before_resume(self):
        node = self.harness()
        SACDriverNode._publish_stop(node, Time(seconds=180), 'data_missing')
        self.assertIsNone(node._last_control_time)
        self.assertEqual(node.cmd_pub.publish.call_args[0][0].drive.speed, 0.0)

    def test_deduplicated_stop_also_clears_clock(self):
        node = self.harness()
        node._last_stop_sent = True
        SACDriverNode._publish_stop(node, Time(seconds=180), 'disabled')
        self.assertIsNone(node._last_control_time)
        node.cmd_pub.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
