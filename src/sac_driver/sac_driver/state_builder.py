"""Build stacked state vectors matching the training layout.

Per-frame observation (occupancy-racer-sac2 `racer_env.py::_build_observation`):

    [lidar_0 .. lidar_{n-1},   # distance / max_range, clipped to [0, 1]
     collision,                # 0/1 in training; always 0.0 on the car
     speed_norm,               # |v| / max_speed, [0, 1]
     servo_norm,               # (steer_cmd + 1) / 2, [0, 1], 0.5 = straight
     linear_accel_norm,        # a / max_accel, [-1, 1]
     angular_vel_norm]         # yaw_rate / max_yaw_rate, [-1, 1]

`stack_frames` consecutive frames are concatenated oldest-first.
"""

from __future__ import annotations

from collections import deque
from typing import Deque, Optional, Sequence

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

    def build_observation(
        self,
        lidar_normalized: Sequence[float],
        speed_mps: float,
        steer_cmd_norm: float,
        linear_accel_mps2: float = 0.0,
        yaw_rate_rad_s: float = 0.0,
        collision: bool = False,
    ) -> np.ndarray:
        lidar_arr = np.asarray(lidar_normalized, dtype=np.float32).reshape(-1)
        if lidar_arr.size != self.lidar_dim:
            raise ValueError(f"Expected lidar dim {self.lidar_dim}, got {lidar_arr.size}")
        lidar_arr = np.clip(lidar_arr, 0.0, 1.0)

        speed_norm = min(abs(float(speed_mps)) / self.max_speed_mps, 1.0)
        steer_cmd = max(-1.0, min(1.0, float(steer_cmd_norm)))
        servo_norm = (steer_cmd + 1.0) * 0.5
        accel_norm = max(-1.0, min(1.0, float(linear_accel_mps2) / self.max_accel_mps2))
        yaw_norm = max(-1.0, min(1.0, float(yaw_rate_rad_s) / self.max_yaw_rate_rad_s))

        obs = np.empty(self.single_obs_dim, dtype=np.float32)
        obs[: self.lidar_dim] = lidar_arr
        obs[self.lidar_dim] = 1.0 if collision else 0.0
        obs[self.lidar_dim + 1] = speed_norm
        obs[self.lidar_dim + 2] = servo_norm
        obs[self.lidar_dim + 3] = accel_norm
        obs[self.lidar_dim + 4] = yaw_norm
        return obs

    def update(
        self,
        lidar_normalized: Sequence[float],
        speed_mps: float,
        steer_cmd_norm: float,
        linear_accel_mps2: float = 0.0,
        yaw_rate_rad_s: float = 0.0,
    ) -> np.ndarray:
        obs = self.build_observation(
            lidar_normalized, speed_mps, steer_cmd_norm, linear_accel_mps2, yaw_rate_rad_s,
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
