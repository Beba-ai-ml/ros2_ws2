"""Map policy actions to vehicle command structures (offline, ROS-agnostic)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


@dataclass
class ControlMapper:
    max_steering_angle_deg: float
    max_speed_mps: float
    max_accel_mps2: float
    speed_limit_mps: Optional[float] = None
    steer_rate_limit_deg_s: Optional[float] = None
    accel_rate_limit_mps2: Optional[float] = None
    safe_mode: bool = False
    safe_speed_limit_mps: Optional[float] = None
    safe_steer_scale: float = 0.5
    safe_accel_scale: float = 0.5
    wheelbase_m: Optional[float] = None
    max_yaw_rate_rad_s: Optional[float] = None
    default_dt: float = 0.1
    speed_sign: float = 1.0
    steer_sign: float = 1.0
    # Speed-dependent steering limit, same formula as the training simulator
    # (occupancy-racer-sac2 vehicle.py): limit = max + (min - max) * (|v| / ref)^2.
    # None disables the curve (constant max_steering_angle_deg).
    min_steering_angle_deg: Optional[float] = None
    steer_speed_ref_mps: Optional[float] = None

    def __post_init__(self) -> None:
        self._max_steer_rad = math.radians(float(self.max_steering_angle_deg))
        self._last_steer_rad = 0.0
        self._last_accel = 0.0
        self._last_speed_cmd = None
        self._speed_sign = 1.0 if self.speed_sign >= 0 else -1.0
        self._steer_sign = 1.0 if self.steer_sign >= 0 else -1.0
        # Last steering command expressed in POLICY space ([-1, 1], policy sign
        # convention, after rate limiting). This is what the simulator feeds back
        # to the network as `servo_norm = (steer + 1) / 2`.
        self.last_steer_norm: float = 0.0

    def reset(self, current_speed: float = 0.0) -> None:
        """Reset internal integrators to avoid speed jumps after re-enable."""
        self._last_steer_rad = 0.0
        self._last_accel = 0.0
        self._last_speed_cmd = float(current_speed) * self._speed_sign
        self.last_steer_norm = 0.0

    def steer_limit_rad(self, current_speed: float) -> float:
        """Max steering angle at the given speed (training-simulator curve)."""
        limit = self._max_steer_rad
        ref = self.steer_speed_ref_mps
        if self.min_steering_angle_deg is None or ref is None or float(ref) <= 0.0:
            return limit
        min_rad = min(math.radians(float(self.min_steering_angle_deg)), limit)
        ratio = min(abs(float(current_speed)) / float(ref), 1.0)
        return limit + (min_rad - limit) * ratio * ratio

    def _effective_speed_limit(self) -> float:
        base = float(self.speed_limit_mps) if self.speed_limit_mps is not None else float(self.max_speed_mps)
        if self.safe_mode and self.safe_speed_limit_mps is not None:
            return min(base, float(self.safe_speed_limit_mps))
        return base

    def _apply_rate_limit(self, value: float, last: float, rate_limit: Optional[float], dt: float) -> float:
        if rate_limit is None or rate_limit <= 0.0:
            return value
        delta = rate_limit * dt
        return _clamp(value, last - delta, last + delta)

    def map_to_ackermann(
        self,
        steering_raw: float,
        accel_raw: float,
        current_speed: float,
        dt: Optional[float] = None,
    ) -> Dict[str, float]:
        if dt is None:
            dt = self.default_dt
        dt = max(float(dt), 1e-4)

        steer_norm = _clamp(float(steering_raw), -1.0, 1.0)
        steer_norm *= self._steer_sign
        steer_limit = self.steer_limit_rad(current_speed)
        steer_rad = steer_norm * steer_limit
        accel = _clamp(float(accel_raw), -self.max_accel_mps2, self.max_accel_mps2)
        accel *= self._speed_sign

        steer_scale = 1.0
        if self.safe_mode:
            steer_scale = float(self.safe_steer_scale)
            steer_rad *= steer_scale
            accel *= float(self.safe_accel_scale)

        steer_rate = None
        if self.steer_rate_limit_deg_s is not None:
            steer_rate = math.radians(float(self.steer_rate_limit_deg_s))
        steer_rad = self._apply_rate_limit(steer_rad, self._last_steer_rad, steer_rate, dt)
        accel = self._apply_rate_limit(accel, self._last_accel, self.accel_rate_limit_mps2, dt)

        # Feedback for the observation: the steering actually commanded, mapped
        # back to policy space (undo sign flip, safe scale and speed curve).
        denom = steer_limit * steer_scale
        if denom > 1e-9:
            self.last_steer_norm = _clamp(steer_rad / denom * self._steer_sign, -1.0, 1.0)
        else:
            self.last_steer_norm = 0.0

        speed_limit = self._effective_speed_limit()
        if self._last_speed_cmd is None:
            self._last_speed_cmd = float(current_speed)
        speed_cmd = self._last_speed_cmd + accel * dt
        speed_cmd = _clamp(speed_cmd, -speed_limit, speed_limit)

        self._last_steer_rad = steer_rad
        self._last_accel = accel
        self._last_speed_cmd = speed_cmd

        return {
            "steering_angle": steer_rad,
            "speed": speed_cmd,
            "acceleration": accel,
        }

    def map_to_twist(
        self,
        steering_raw: float,
        accel_raw: float,
        current_speed: float,
        dt: Optional[float] = None,
    ) -> Dict[str, float]:
        ack = self.map_to_ackermann(steering_raw, accel_raw, current_speed, dt=dt)
        speed_cmd = ack["speed"]
        steer_rad = ack["steering_angle"]

        if self.wheelbase_m and abs(steer_rad) > 1e-6:
            yaw_rate = (speed_cmd / float(self.wheelbase_m)) * math.tan(steer_rad)
        elif self.max_yaw_rate_rad_s is not None:
            steer_norm = _clamp(float(steering_raw), -1.0, 1.0)
            yaw_rate = steer_norm * float(self.max_yaw_rate_rad_s)
        else:
            yaw_rate = 0.0

        return {
            "linear_x": speed_cmd,
            "angular_z": yaw_rate,
        }


__all__ = ["ControlMapper"]
