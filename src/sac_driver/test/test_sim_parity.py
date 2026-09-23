"""Offline parity tests: the car-side pipeline must match the training simulator.

Runs without ROS (plain python + numpy; torch only for the policy test):

    cd src/sac_driver && python3 -m unittest test.test_sim_parity -v

Reference = occupancy-racer-sac2 `racer_env.py` (LIDAR_CENTER_DEG = 90,
observation [lidar, collision, speed, servo(0..1), accel, yaw]) and
`vehicle.py` (speed-dependent steering limit).
"""

from __future__ import annotations

import math
import os
import sys
import unittest

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)
sys.path.insert(0, _PKG)

from sac_driver.control_mapper import ControlMapper  # noqa: E402
from sac_driver.lidar_converter import LidarConverter, build_lidar_angles  # noqa: E402
from sac_driver.state_builder import StateBuilder  # noqa: E402

_PARAMS = os.path.join(_PKG, "config", "driver_params.yaml")
_WEIGHTS_DIR = os.path.join(_PKG, "weights")


def _load_params() -> dict:
    """Tiny flat parser for driver_params.yaml (key: value lines only)."""
    out = {}
    with open(_PARAMS, encoding="utf-8") as fh:
        for line in fh:
            line = line.split("#", 1)[0].strip()
            if ":" not in line or line.endswith(":"):
                continue
            key, val = line.split(":", 1)
            val = val.strip().strip('"')
            try:
                out[key.strip()] = float(val)
            except ValueError:
                out[key.strip()] = val
    return out


class _Scan:
    """Raw LaserScan stand-in: front 0deg, left positive (measured 2026-09-23)."""

    def __init__(self, n: int = 1440, obstacle_deg: float | None = None, dist: float = 0.5):
        self.angle_min = -math.pi
        self.angle_increment = 2.0 * math.pi / n
        self.ranges = [20.0] * n
        if obstacle_deg is not None:
            rad = math.radians(obstacle_deg)
            rad = ((rad + math.pi) % (2.0 * math.pi)) - math.pi
            idx = int(round((rad - self.angle_min) / self.angle_increment)) % n
            for k in range(-3, 4):
                self.ranges[(idx + k) % n] = dist


def _converter_from_params() -> LidarConverter:
    p = _load_params()
    return LidarConverter(
        target_angles_deg=build_lidar_angles(p["lidar.front_step_deg"], p["lidar.rear_step_deg"]),
        max_range_m=p["lidar.max_range_m"],
        angle_offset_deg=p["lidar.angle_offset_deg"],
        angle_direction=p["lidar.angle_direction"],
        use_interpolation=True,
    )


def _assert_min_at(tc: unittest.TestCase, out: np.ndarray, idx: int, msg: str) -> None:
    """The obstacle (a few equal-distance rays) must contain index `idx`; far rays stay at max."""
    tc.assertAlmostEqual(float(out[idx]), float(out.min()), places=5, msg=msg)
    tc.assertLess(float(out[idx]), 0.1, msg)
    n = out.size
    for far in (idx + n // 4, idx + n // 2, idx - n // 4):
        tc.assertGreater(float(out[far % n]), 0.9, f"{msg} (ray {far % n} should be clear)")


class LidarFrameParity(unittest.TestCase):
    """Sim front is 90deg; measured raw front is 0deg, left is positive."""

    def test_450_rays(self):
        self.assertEqual(len(build_lidar_angles(0.5, 2.0)), 450)

    def test_front_obstacle_lands_on_sim_forward_index(self):
        out = _converter_from_params().convert(_Scan(obstacle_deg=0.0))
        _assert_min_at(self, out, 180, "sim angle 90deg (index 180) must see the front")

    def test_left_obstacle_lands_on_sim_angle_0(self):
        out = _converter_from_params().convert(_Scan(obstacle_deg=+90.0))
        _assert_min_at(self, out, 0, "raw +90deg (measured car left) must be sim angle 0deg")

    def test_right_obstacle_lands_on_sim_angle_180(self):
        out = _converter_from_params().convert(_Scan(obstacle_deg=-90.0))
        _assert_min_at(self, out, 360, "raw -90deg (measured car right) must be sim angle 180deg")

    def test_rear_obstacle_lands_in_rear_hemisphere(self):
        out = _converter_from_params().convert(_Scan(obstacle_deg=180.0))
        idx = int(np.argmin(out))
        angles = build_lidar_angles(0.5, 2.0)
        self.assertAlmostEqual(angles[idx], 270.0, delta=2.0)

    def test_default_converter_preserves_measured_sides(self):
        converter = LidarConverter(build_lidar_angles(), max_range_m=20.0)
        for raw_angle, expected_index in ((0.0, 180), (90.0, 0), (-90.0, 360)):
            with self.subTest(raw_angle=raw_angle):
                out = converter.convert(_Scan(obstacle_deg=raw_angle))
                _assert_min_at(self, out, expected_index,
                               "omitting YAML must preserve the measured raw frame")
                np.testing.assert_allclose(out, _converter_from_params().convert(
                    _Scan(obstacle_deg=raw_angle)))

    def test_positive_steer_turns_left_in_ros(self):
        p = _load_params()
        self.assertGreater(p["control.steer_sign"], 0.0,
                           "positive policy steer must keep the sim angle-0 side mapping")


class ObservationLayoutParity(unittest.TestCase):
    def test_channel_order_and_ranges(self):
        sb = StateBuilder(stack_frames=4, lidar_dim=450, max_speed_mps=2.5)
        lidar = np.full(450, 0.3, dtype=np.float32)
        obs = sb.build_observation(lidar, speed_mps=1.25, steer_cmd_norm=0.0,
                                   linear_accel_mps2=2.0, yaw_rate_rad_s=-1.5)
        self.assertEqual(obs.shape, (455,))
        self.assertEqual(obs[450], 0.0, "collision flag is channel +0 and 0 while driving")
        self.assertAlmostEqual(obs[451], 0.5, places=6, msg="speed_norm = |v|/max_speed")
        self.assertAlmostEqual(obs[452], 0.5, places=6, msg="servo_norm 0.5 = straight")
        self.assertAlmostEqual(obs[453], 0.5, places=6, msg="accel / 4.0")
        self.assertAlmostEqual(obs[454], -0.5, places=6, msg="yaw / 3.0")

    def test_servo_norm_is_0_to_1_of_policy_steer(self):
        sb = StateBuilder(stack_frames=1, lidar_dim=4, max_speed_mps=2.5)
        full_left = sb.build_observation(np.ones(4), 0.0, +1.0)
        full_right = sb.build_observation(np.ones(4), 0.0, -1.0)
        self.assertAlmostEqual(full_left[4 + 2], 1.0)
        self.assertAlmostEqual(full_right[4 + 2], 0.0)

    def test_speed_divisor_matches_training_physics(self):
        p = _load_params()
        self.assertAlmostEqual(p["state.max_speed_mps"], 2.5,
                               msg="car_1_x policies were trained with physics max_speed 2.5")

    def test_stack_is_oldest_first_and_1820(self):
        sb = StateBuilder(stack_frames=4, lidar_dim=450, max_speed_mps=2.5)
        state = sb.reset(sb.build_observation(np.zeros(450), 0.0, 0.0))
        self.assertEqual(state.shape, (1820,))
        newer = sb.update(np.ones(450), 0.0, 0.0)
        self.assertEqual(newer[0], 0.0, "oldest frame first")
        self.assertEqual(newer[3 * 455], 1.0, "newest frame last")


class ControlParity(unittest.TestCase):
    def _mapper(self, **kw) -> ControlMapper:
        base = dict(max_steering_angle_deg=20.0, max_speed_mps=2.5, max_accel_mps2=2.0,
                    speed_limit_mps=2.0, steer_rate_limit_deg_s=None, accel_rate_limit_mps2=None,
                    safe_mode=False, min_steering_angle_deg=5.0, steer_speed_ref_mps=8.0,
                    speed_sign=-1.0, steer_sign=1.0, default_dt=1 / 60)
        base.update(kw)
        return ControlMapper(**base)

    def test_steer_curve_matches_vehicle_py(self):
        m = self._mapper()
        for v in (0.0, 2.0, 4.0, 8.0, 12.0):
            ratio = min(abs(v) / 8.0, 1.0)
            expected = math.radians(20.0) + (math.radians(5.0) - math.radians(20.0)) * ratio ** 2
            self.assertAlmostEqual(m.steer_limit_rad(v), expected, places=9)

    def test_steer_feedback_is_policy_space(self):
        m = self._mapper()
        m.reset(0.0)
        m.map_to_ackermann(+1.0, 0.0, current_speed=0.0)
        self.assertAlmostEqual(m.last_steer_norm, 1.0, places=6)
        m.map_to_ackermann(-0.5, 0.0, current_speed=0.0)
        self.assertAlmostEqual(m.last_steer_norm, -0.5, places=6)

    def test_forward_accel_gives_forward_command_on_this_car(self):
        m = self._mapper()
        m.reset(0.0)
        cmd = m.map_to_ackermann(0.0, 2.0, current_speed=0.0)
        self.assertLess(cmd["speed"], 0.0, "positive drive.speed = REVERSE on this car")

    def test_timing_params_match_sim(self):
        p = _load_params()
        self.assertAlmostEqual(p["control.rate_hz"], 60.0)
        self.assertAlmostEqual(p["control.decision_every_n"], 8.0)


class PolicyLoads(unittest.TestCase):
    def test_default_model_exists_is_1820_and_gives_finite_actions(self):
        try:
            import torch  # noqa: F401
        except Exception:  # pragma: no cover
            self.skipTest("torch not installed")
        from sac_driver.policy_loader import load_policy

        p = _load_params()
        path = os.path.join(_PKG, str(p["model.path"]))
        self.assertTrue(os.path.exists(path), path)
        policy = load_policy(path, device="cpu")
        self.assertEqual(int(policy.backbone[0].in_features), 1820)
        import torch
        state = torch.zeros(1, 1820)
        act = policy.get_mean_action(state).detach().numpy().reshape(-1)
        self.assertEqual(act.size, 2)
        self.assertTrue(np.all(np.isfinite(act)))
        self.assertGreaterEqual(float(act[1]), 0.0, "accel action range is [0, 2]")
        self.assertLessEqual(float(act[1]), 2.0)


if __name__ == "__main__":
    unittest.main()
