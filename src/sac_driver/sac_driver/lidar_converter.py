"""Convert ROS2 LaserScan into training-aligned lidar rays."""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Iterable, Sequence

import numpy as np

_LOGGER = logging.getLogger(__name__)


def _get_attr(obj: object, name: str):
    if hasattr(obj, name):
        return getattr(obj, name)
    if isinstance(obj, dict) and name in obj:
        return obj[name]
    raise AttributeError(f"scan_msg missing required field: {name}")


def _get_optional_attr(obj: object, name: str, default):
    if isinstance(obj, dict):
        return obj.get(name, default)
    return getattr(obj, name, default)


@dataclass
class LidarConverter:
    target_angles_deg: Sequence[float]
    max_range_m: float
    # Cardboard measurements confirm raw front=0, left=+90, right=-90.
    # Match the active YAML: sim angle a maps to raw 90-a. Legacy 27-ray
    # callers must pass their different historical mapping explicitly.
    angle_offset_deg: float = -90.0
    angle_direction: float = -1.0
    use_interpolation: bool = True
    max_invalid_gap_deg: float = 1.5

    def __post_init__(self) -> None:
        if not math.isfinite(self.max_invalid_gap_deg) or self.max_invalid_gap_deg < 0.0:
            raise ValueError("max_invalid_gap_deg must be finite and nonnegative.")

    def _to_scan_angle_rad(self, target_angle_deg: float) -> float:
        # Convert target angle into scan frame with optional offset and direction,
        # then wrap to [-pi, pi) so rear-hemisphere angles map into RPLiDAR range.
        angle_deg = self.angle_direction * (float(target_angle_deg) + float(self.angle_offset_deg))
        angle_rad = math.radians(angle_deg)
        angle_rad = ((angle_rad + math.pi) % (2.0 * math.pi)) - math.pi
        return angle_rad

    def convert(self, scan_msg: object) -> np.ndarray:
        angle_min = float(_get_attr(scan_msg, "angle_min"))
        angle_increment = float(_get_attr(scan_msg, "angle_increment"))
        ranges = _get_attr(scan_msg, "ranges")
        ranges_np = np.asarray(ranges, dtype=np.float32)

        if ranges_np.size == 0:
            raise ValueError("LaserScan ranges is empty.")
        if not math.isfinite(angle_increment) or angle_increment <= 0:
            raise ValueError("LaserScan angle_increment must be finite and > 0.")

        # Discard invalid source samples before interpolation. In particular,
        # finite + inf (and even 0 * inf at an exact ray) used to erase a nearby
        # obstacle by producing inf/NaN and then replacing it with max range.
        range_min = float(_get_optional_attr(scan_msg, "range_min", 0.0))
        range_max = float(_get_optional_attr(scan_msg, "range_max", math.inf))
        valid = (
            np.isfinite(ranges_np) & (ranges_np > 0.0)
            & (ranges_np >= range_min) & (ranges_np <= range_max)
        )
        if not np.any(valid):
            # The control callback catches this error and publishes a stop.
            raise ValueError("LaserScan contains no valid range measurements.")

        if self.max_invalid_gap_deg > 0.0:
            # Fill only short runs bounded by valid samples in THIS scan.
            # The closer edge is conservative at an obstacle boundary. Longer
            # gaps and unbounded scan edges remain unknown; no history is used.
            ranges_np = ranges_np.copy()
            valid_indices = np.flatnonzero(valid)
            step_deg = math.degrees(angle_increment)
            for left, right in zip(valid_indices[:-1], valid_indices[1:]):
                missing = right - left - 1
                if missing > 0 and missing * step_deg <= self.max_invalid_gap_deg:
                    ranges_np[left + 1:right] = min(ranges_np[left], ranges_np[right])
                    valid[left + 1:right] = True

        max_range = float(self.max_range_m)
        out = np.empty(len(self.target_angles_deg), dtype=np.float32)

        for i, target_angle_deg in enumerate(self.target_angles_deg):
            target_rad = self._to_scan_angle_rad(target_angle_deg)
            idx_float = (target_rad - angle_min) / angle_increment

            if idx_float < 0 or idx_float > (ranges_np.size - 1):
                _LOGGER.warning(
                    "Target angle %.2f deg outside scan range; using max range.",
                    target_angle_deg,
                )
                dist = max_range
            else:
                if self.use_interpolation:
                    idx0 = int(math.floor(idx_float))
                    idx1 = min(idx0 + 1, ranges_np.size - 1)
                    w = idx_float - idx0
                    v0 = float(ranges_np[idx0])
                    v1 = float(ranges_np[idx1])
                    if valid[idx0] and valid[idx1]:
                        dist = (1.0 - w) * v0 + w * v1
                    elif valid[idx0]:
                        dist = v0
                    elif valid[idx1]:
                        dist = v1
                    else:
                        dist = max_range
                else:
                    idx = int(round(idx_float))
                    dist = float(ranges_np[idx]) if valid[idx] else max_range

                if not math.isfinite(dist) or dist <= 0.0:
                    dist = max_range
                elif dist > max_range:
                    dist = max_range

            out[i] = dist / max_range

        return np.clip(out, 0.0, 1.0)


def build_lidar_angles(front_step_deg: float = 0.5, rear_step_deg: float = 2.0) -> list:
    """Generate variable-resolution lidar angles matching training env.

    Front hemisphere (0-180 deg): front_step resolution.
    Rear hemisphere (180-360 deg): rear_step resolution.
    """
    angles = []
    a = 0.0
    while a <= 180.0 + 1e-9:
        angles.append(round(a, 4))
        a += front_step_deg
    a = 180.0 + rear_step_deg
    while a < 360.0 - 1e-9:
        angles.append(round(a, 4))
        a += rear_step_deg
    return angles


__all__ = ["LidarConverter", "build_lidar_angles"]
