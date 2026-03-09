"""Build stacked state vectors matching training layout."""

from __future__ import annotations

from collections import deque
from typing import Deque, Iterable, Optional, Sequence

import numpy as np


class StateBuilder:
    def __init__(
        self,
        stack_frames: int,
        lidar_dim: int,
        max_speed_mps: float,
        max_accel_mps2: float = 4.0,
        max_yaw_rate_rad_s: float = 3.0,
    ) -> None:
        self.stack_frames = max(1, int(stack_frames))
        self.lidar_dim = int(lidar_dim)
        self.max_speed_mps = float(max_speed_mps) if max_speed_mps > 0 else 1.0
        self.max_accel_mps2 = float(max_accel_mps2) if max_accel_mps2 > 0 else 1.0
        self.max_yaw_rate_rad_s = float(max_yaw_rate_rad_s) if max_yaw_rate_rad_s > 0 else 1.0
        self._stack: Deque[np.ndarray] = deque(maxlen=self.stack_frames)

    @property
    def single_obs_dim(self) -> int:
        return self.lidar_dim + 5

    @property
    def state_dim(self) -> int:
        return self.single_obs_dim * self.stack_frames

    def reset(self, first_obs: Optional[np.ndarray] = None) -> np.ndarray:
        if first_obs is None:
            first_obs = np.zeros(self.single_obs_dim, dtype=np.float32)
        first_obs = np.asarray(first_obs, dtype=np.float32).reshape(-1)
        if first_obs.size != self.single_obs_dim:
            raise ValueError(f"Expected obs dim {self.single_obs_dim}, got {first_obs.size}")

        self._stack.clear()
        if self.stack_frames <= 1:
            self._stack.append(first_obs)
            return first_obs
        for _ in range(self.stack_frames):
            self._stack.append(first_obs)
        return np.concatenate(list(self._stack))

    def _build_observation(
        self,
        lidar_normalized: Sequence[float],
        speed_mps: float,
        servo_normalized: float,
        collision_flag: float,
        linear_accel_mps2: float = 0.0,
        yaw_rate_rad_s: float = 0.0,
    ) -> np.ndarray:
        lidar_arr = np.asarray(lidar_normalized, dtype=np.float32).reshape(-1)
        if lidar_arr.size != self.lidar_dim:
            raise ValueError(f"Expected lidar dim {self.lidar_dim}, got {lidar_arr.size}")
        lidar_arr = np.clip(lidar_arr, 0.0, 1.0)

        speed_norm = min(abs(float(speed_mps)) / self.max_speed_mps, 1.0)
        servo_norm = min(max(float(servo_normalized), 0.0), 1.0)
        collision = 1.0 if float(collision_flag) > 0.0 else 0.0
        accel_norm = max(-1.0, min(1.0, float(linear_accel_mps2) / self.max_accel_mps2))
        yaw_norm = max(-1.0, min(1.0, float(yaw_rate_rad_s) / self.max_yaw_rate_rad_s))

        obs = np.array(
            list(lidar_arr) + [collision, speed_norm, servo_norm, accel_norm, yaw_norm],
            dtype=np.float32,
        )
        return obs

    def update(
        self,
        lidar_normalized: Sequence[float],
        speed_mps: float,
        servo_normalized: float,
        collision_flag: float = 0.0,
        linear_accel_mps2: float = 0.0,
        yaw_rate_rad_s: float = 0.0,
    ) -> np.ndarray:
        obs = self._build_observation(
            lidar_normalized, speed_mps, servo_normalized, collision_flag,
            linear_accel_mps2, yaw_rate_rad_s,
        )
        if self.stack_frames <= 1:
            self._stack.clear()
            self._stack.append(obs)
            return obs
        if not self._stack:
            return self.reset(obs)
        self._stack.append(obs)
        return np.concatenate(list(self._stack))


__all__ = ["StateBuilder"]
