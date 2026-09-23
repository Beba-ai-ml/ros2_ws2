#!/usr/bin/env python3
"""Read-only monitor for LiDAR, odometry, AI drive commands, and servo output.

Run from the workspace root after sourcing ROS 2 and the workspace setup:
    python3 tools/ros2_input_diagnostic.py
    python3 tools/ros2_input_diagnostic.py --angle-offset 90 --angle-direction -1
    python3 tools/ros2_input_diagnostic.py --capture --angle-offset 90 --angle-direction -1

This node only subscribes. It does not publish commands or start other nodes.
Pass the active lidar parameters from `ros2 param get /sac_driver ...` to match the
running AI node. Without overrides, the checked-in driver_params.yaml is used.
"""

from __future__ import annotations

import argparse
import datetime
import json
import math
import sys
import threading
import time
from collections import deque
from pathlib import Path
from statistics import mean
from typing import Optional, Tuple

import rclpy
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64


WORKSPACE_ROOT = Path(__file__).resolve().parents[1]
SAC_SOURCE = WORKSPACE_ROOT / "src" / "sac_driver"
PARAMS_PATH = SAC_SOURCE / "config" / "driver_params.yaml"
sys.path.insert(0, str(SAC_SOURCE))

from sac_driver.lidar_converter import LidarConverter, build_lidar_angles  # noqa: E402


def _load_lidar_config() -> dict:
    with PARAMS_PATH.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    return document.get("sac_driver", {}).get("ros__parameters", {})


def _wrap_degrees(angle_deg: float) -> float:
    return (float(angle_deg) + 180.0) % 360.0 - 180.0


def _assumed_sector(angle_deg: float) -> str:
    """Describe a ROS scan angle assuming laser yaw is aligned with base_link."""
    angle = _wrap_degrees(angle_deg)
    if -45.0 <= angle <= 45.0:
        return "front"
    if 45.0 < angle < 135.0:
        return "left"
    if -135.0 < angle < -45.0:
        return "right"
    return "rear"


def _range_summary(scan: LaserScan) -> Tuple[Optional[int], Optional[float], Optional[float]]:
    best_index = None
    best_range = None
    for index, value in enumerate(scan.ranges):
        distance = float(value)
        if not math.isfinite(distance) or distance < max(0.05, float(scan.range_min)):
            continue
        if distance > float(scan.range_max):
            continue
        if best_range is None or distance < best_range:
            best_index = index
            best_range = distance

    if best_index is None or best_range is None:
        return None, None, None
    angle_deg = math.degrees(float(scan.angle_min) + best_index * float(scan.angle_increment))
    return best_index, _wrap_degrees(angle_deg), best_range


class InputDiagnostic(Node):
    def __init__(
        self,
        report_interval_sec: float = 1.0,
        angle_offset_override: Optional[float] = None,
        angle_direction_override: Optional[float] = None,
        capture_mode: bool = False,
    ) -> None:
        super().__init__("ros2_input_diagnostic")
        params = _load_lidar_config()

        self.scan_topic = str(params.get("topics.scan", "/scan"))
        self.odom_topic = str(params.get("topics.odom", "/odom"))
        self.drive_topic = str(params.get("topics.cmd", "/drive"))
        self.servo_topic = "/commands/servo/position"

        front_step = float(params.get("lidar.front_step_deg", 0.5))
        rear_step = float(params.get("lidar.rear_step_deg", 2.0))
        self.max_range_m = float(params.get("lidar.max_range_m", 20.0))
        source_offset = float(params.get("lidar.angle_offset_deg", -90.0))
        source_direction = float(params.get("lidar.angle_direction", -1.0))
        self.angle_offset_deg = (
            source_offset if angle_offset_override is None else float(angle_offset_override)
        )
        self.angle_direction = (
            source_direction if angle_direction_override is None else float(angle_direction_override)
        )
        if angle_offset_override is None and angle_direction_override is None:
            self.converter_config_source = "checked-in driver_params.yaml"
        else:
            self.converter_config_source = "CLI override(s) over driver_params.yaml"
        self.converter = LidarConverter(
            target_angles_deg=build_lidar_angles(front_step, rear_step),
            max_range_m=self.max_range_m,
            angle_offset_deg=self.angle_offset_deg,
            angle_direction=self.angle_direction,
            use_interpolation=bool(params.get("lidar.use_interpolation", True)),
        )

        self.scan: Optional[LaserScan] = None
        self.scan_received_monotonic: Optional[float] = None
        self.scan_receive_intervals = deque(maxlen=12)
        self._last_scan_received_monotonic: Optional[float] = None
        self._condition = threading.Condition()
        self._scan_sequence = 0
        self.odom: Optional[Odometry] = None
        self.odom_received_monotonic: Optional[float] = None
        self.drive: Optional[AckermannDriveStamped] = None
        self.drive_received_monotonic: Optional[float] = None
        self.servo: Optional[Float64] = None
        self.servo_received_monotonic: Optional[float] = None

        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)
        self.drive_sub = self.create_subscription(AckermannDriveStamped, self.drive_topic, self._on_drive, 10)
        self.servo_sub = self.create_subscription(Float64, self.servo_topic, self._on_servo, 10)
        self.report_timer = None
        if not capture_mode:
            self.report_timer = self.create_timer(max(0.25, float(report_interval_sec)), self.report)

        self.get_logger().info(
            "Read-only subscriptions active: scan=%s odom=%s drive=%s servo=%s"
            % (self.scan_topic, self.odom_topic, self.drive_topic, self.servo_topic)
        )
        self.get_logger().info(
            "Converter config (%s): rays=%d offset=%+.1f direction=%+.1f max_range=%.1fm"
            % (
                self.converter_config_source,
                len(self.converter.target_angles_deg),
                self.angle_offset_deg,
                self.angle_direction,
                self.max_range_m,
            )
        )
    def _on_scan(self, msg: LaserScan) -> None:
        now = time.monotonic()
        with self._condition:
            if self._last_scan_received_monotonic is not None:
                interval = now - self._last_scan_received_monotonic
                if interval > 0.0:
                    self.scan_receive_intervals.append(interval)
            self._last_scan_received_monotonic = now
            self.scan_received_monotonic = now
            self.scan = msg
            self._scan_sequence += 1
            self._condition.notify_all()

    def _on_odom(self, msg: Odometry) -> None:
        with self._condition:
            self.odom = msg
            self.odom_received_monotonic = time.monotonic()

    def _on_drive(self, msg: AckermannDriveStamped) -> None:
        with self._condition:
            self.drive = msg
            self.drive_received_monotonic = time.monotonic()

    def _on_servo(self, msg: Float64) -> None:
        with self._condition:
            self.servo = msg
            self.servo_received_monotonic = time.monotonic()

    def capture_after(
        self,
        after_sequence: int,
        timeout_sec: float,
        label: str,
        fresh_scan_count: int = 2,
    ) -> Optional[dict]:
        """Capture after two post-prompt scans, avoiding a sweep already in progress."""
        deadline = time.monotonic() + float(timeout_sec)
        target_sequence = after_sequence + max(1, int(fresh_scan_count))
        with self._condition:
            while self._scan_sequence < target_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0 or not rclpy.ok():
                    return None
                self._condition.wait(remaining)

            scan = self.scan
            scan_sequence = self._scan_sequence
            scan_received = self.scan_received_monotonic
            scan_intervals = list(self.scan_receive_intervals)
            odom = self.odom
            odom_received = self.odom_received_monotonic
            drive = self.drive
            drive_received = self.drive_received_monotonic
            servo = self.servo
            servo_received = self.servo_received_monotonic

        if scan is None:
            return None

        captured_monotonic = time.monotonic()

        def age_ms(received: Optional[float]) -> Optional[float]:
            if received is None:
                return None
            return round((captured_monotonic - received) * 1000.0, 1)

        raw_index, raw_angle, raw_distance = _range_summary(scan)
        model_ranges = self.converter.convert(scan)
        model_index = int(model_ranges.argmin())
        model_distance = float(model_ranges[model_index]) * self.max_range_m
        model_angle = float(self.converter.target_angles_deg[model_index])
        expected_ros_angle = _wrap_degrees(
            self.angle_direction * (model_angle + self.angle_offset_deg)
        )

        stamp_ns = int(scan.header.stamp.sec) * 1000000000 + int(scan.header.stamp.nanosec)
        now_ns = self.get_clock().now().nanoseconds
        header_age_ms = None
        if stamp_ns > 0:
            delta_ms = (now_ns - stamp_ns) / 1000000.0
            if 0.0 <= delta_ms < 60000.0:
                header_age_ms = round(delta_ms, 1)

        def finite_ranges(values):
            result = []
            for value in values:
                number = float(value)
                result.append(number if math.isfinite(number) else None)
            return result

        raw_nearest = None
        if raw_index is not None:
            raw_nearest = {
                "index": raw_index,
                "angle_deg": raw_angle,
                "sector_if_laser_yaw_is_zero": _assumed_sector(raw_angle),
                "distance_m": raw_distance,
            }

        ai_nearest = {
            "index": model_index,
            "sim_angle_deg": model_angle,
            "sampled_ros_angle_deg": expected_ros_angle,
            "sector_if_laser_yaw_is_zero": _assumed_sector(expected_ros_angle),
            "distance_m": model_distance,
            "clear_to_max_range": model_distance >= self.max_range_m * 0.99,
        }

        scan_rate_hz = None
        if scan_intervals:
            interval_mean = mean(scan_intervals)
            if interval_mean > 0.0:
                scan_rate_hz = round(1.0 / interval_mean, 2)

        record = {
            "label": label,
            "captured_at_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "scan_sequence": scan_sequence,
            "topics": {
                "scan": self.scan_topic,
                "odom": self.odom_topic,
                "drive": self.drive_topic,
                "servo": self.servo_topic,
            },
            "converter": {
                "source": self.converter_config_source,
                "angle_offset_deg": self.angle_offset_deg,
                "angle_direction": self.angle_direction,
                "max_range_m": self.max_range_m,
                "target_ray_count": len(self.converter.target_angles_deg),
            },
            "scan": {
                "frame_id": scan.header.frame_id,
                "stamp_sec": int(scan.header.stamp.sec),
                "stamp_nanosec": int(scan.header.stamp.nanosec),
                "header_age_ms": header_age_ms,
                "receive_age_ms": age_ms(scan_received),
                "receive_rate_hz": scan_rate_hz,
                "angle_min_rad": float(scan.angle_min),
                "angle_max_rad": float(scan.angle_max),
                "angle_increment_rad": float(scan.angle_increment),
                "scan_time_sec": float(scan.scan_time),
                "time_increment_sec": float(scan.time_increment),
                "range_min_m": float(scan.range_min),
                "range_max_m": float(scan.range_max),
                "raw_nearest": raw_nearest,
                "ranges_m": finite_ranges(scan.ranges),
            },
            "ai_lidar": {
                "nearest": ai_nearest,
                "normalized_ranges": [float(value) for value in model_ranges],
            },
            "odom": None,
            "drive": None,
            "servo_position": None,
        }

        if odom is not None:
            record["odom"] = {
                "frame_id": odom.header.frame_id,
                "child_frame_id": odom.child_frame_id,
                "receive_age_ms": age_ms(odom_received),
                "linear_x_mps": float(odom.twist.twist.linear.x),
                "angular_z_rad_s": float(odom.twist.twist.angular.z),
            }
        if drive is not None:
            record["drive"] = {
                "receive_age_ms": age_ms(drive_received),
                "speed_mps": float(drive.drive.speed),
                "steering_angle_rad": float(drive.drive.steering_angle),
                "acceleration_mps2": float(drive.drive.acceleration),
            }
        if servo is not None:
            record["servo_position"] = {
                "receive_age_ms": age_ms(servo_received),
                "value": float(servo.data),
            }
        return record

    def scan_sequence(self) -> int:
        with self._condition:
            return self._scan_sequence

    @staticmethod
    def _age_ms(received_monotonic: Optional[float]) -> str:
        if received_monotonic is None:
            return "waiting"
        return "%.0f ms" % ((time.monotonic() - received_monotonic) * 1000.0)

    def _scan_report(self) -> str:
        scan = self.scan
        if scan is None:
            return "  /scan: waiting"

        interval_mean = mean(self.scan_receive_intervals) if self.scan_receive_intervals else None
        rate = "warming up" if not interval_mean else "%.2f Hz" % (1.0 / interval_mean)
        raw_index, raw_angle, raw_distance = _range_summary(scan)
        if raw_index is None:
            raw_hit = "no valid returns"
        else:
            raw_hit = "i=%d angle=%+.1f deg (%s if yaw=0) range=%.2f m" % (
                raw_index,
                raw_angle,
                _assumed_sector(raw_angle),
                raw_distance,
            )

        try:
            model_scan = self.converter.convert(scan)
            model_index = int(model_scan.argmin())
            model_distance = float(model_scan[model_index]) * self.max_range_m
            model_angle = float(self.converter.target_angles_deg[model_index])
            expected_ros_angle = _wrap_degrees(
                self.angle_direction * (model_angle + self.angle_offset_deg)
            )
            if model_distance >= self.max_range_m * 0.99:
                model_hit = "clear to max range"
            else:
                model_hit = "ray=%d sim_angle=%.1f deg -> ROS=%+.1f deg (%s if yaw=0), %.2f m" % (
                    model_index,
                    model_angle,
                    expected_ros_angle,
                    _assumed_sector(expected_ros_angle),
                    model_distance,
                )
        except Exception as exc:  # keep monitor alive if a malformed scan arrives
            model_hit = "conversion error: %s" % exc

        header = scan.header
        stamp_ns = int(header.stamp.sec) * 1000000000 + int(header.stamp.nanosec)
        now_ns = self.get_clock().now().nanoseconds
        header_age = "n/a"
        if stamp_ns > 0:
            delta_ms = (now_ns - stamp_ns) / 1000000.0
            if 0.0 <= delta_ms < 60000.0:
                header_age = "%.0f ms" % delta_ms

        return (
            "  /scan: age=%s header_age=%s rate=%s frame=%s rays=%d "
            "amin=%+.1f deg inc=%.3f deg scan_time=%.3f s\n"
            "    raw nearest: %s\n"
            "    AI nearest:  %s"
            % (
                self._age_ms(self.scan_received_monotonic),
                header_age,
                rate,
                header.frame_id or "(empty)",
                len(scan.ranges),
                math.degrees(float(scan.angle_min)),
                math.degrees(float(scan.angle_increment)),
                float(scan.scan_time),
                raw_hit,
                model_hit,
            )
        )

    def report(self) -> None:
        print("\n--- ROS2 input diagnostic (read-only) ---")
        print(self._scan_report())

        if self.odom is None:
            print("  /odom: waiting")
        else:
            msg = self.odom
            print(
                "  /odom: age=%s frame=%s child=%s linear.x=%+.3f m/s angular.z=%+.3f rad/s"
                % (
                    self._age_ms(self.odom_received_monotonic),
                    msg.header.frame_id or "(empty)",
                    msg.child_frame_id or "(empty)",
                    float(msg.twist.twist.linear.x),
                    float(msg.twist.twist.angular.z),
                )
            )

        if self.drive is None:
            print("  /drive: waiting")
        else:
            drive = self.drive.drive
            print(
                "  /drive: age=%s speed=%+.3f m/s steering=%+.3f rad accel=%+.3f m/s^2"
                % (
                    self._age_ms(self.drive_received_monotonic),
                    float(drive.speed),
                    float(drive.steering_angle),
                    float(drive.acceleration),
                )
            )

        if self.servo is None:
            print("  /commands/servo/position: waiting")
        else:
            print(
                "  /commands/servo/position: age=%s value=%.4f"
                % (self._age_ms(self.servo_received_monotonic), float(self.servo.data))
            )
        print("  Note: side labels assume the raw laser frame has yaw=0 relative to base_link.")


def main() -> None:
    parser = argparse.ArgumentParser(description="Read-only ROS2 motion input diagnostic")
    parser.add_argument(
        "--angle-offset",
        type=float,
        default=None,
        help="active lidar.angle_offset_deg from /sac_driver (default: source YAML)",
    )
    parser.add_argument(
        "--angle-direction",
        type=float,
        default=None,
        help="active lidar.angle_direction from /sac_driver (default: source YAML)",
    )
    parser.add_argument("--report-interval-sec", type=float, default=1.0)
    parser.add_argument(
        "--capture",
        action="store_true",
        help="press Enter to save one fresh scan/odometry snapshot per cardboard position",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="JSONL output path in capture mode (default: workspace log/ with a timestamp)",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = InputDiagnostic(
        report_interval_sec=args.report_interval_sec,
        angle_offset_override=args.angle_offset,
        angle_direction_override=args.angle_direction,
        capture_mode=args.capture,
    )
    if not args.capture:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    if args.output:
        output_path = Path(args.output).expanduser()
        if not output_path.is_absolute():
            output_path = WORKSPACE_ROOT / output_path
    else:
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = WORKSPACE_ROOT / "log" / ("ros2_input_diag_%s.jsonl" % stamp)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    capture_number = 0
    print(
        "\nCapture mode is read-only. Keep the car stationary and autonomy locked (RB released)."
    )
    print("Position one cardboard target at a time.")
    print("Type an optional label (front/left/right/rear), then press Enter. Type q to finish.")
    print("Each capture waits for two fresh /scan messages, then appends one JSONL record to:")
    print("  %s\n" % output_path)

    try:
        while rclpy.ok():
            try:
                label = input("Target position [Enter captures, q quits]: ").strip()
            except EOFError:
                break
            if label.lower() in ("q", "quit", "exit"):
                break

            after_sequence = node.scan_sequence()
            print("Waiting for two fresh /scan messages...")
            record = node.capture_after(after_sequence, timeout_sec=3.0, label=label)
            if record is None:
                print("No fresh /scan arrived within 3 seconds; nothing saved.\n")
                continue

            capture_number += 1
            record["capture_number"] = capture_number
            with output_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
                stream.flush()

            raw = record["scan"]["raw_nearest"]
            ai = record["ai_lidar"]["nearest"]
            raw_text = "none" if raw is None else "%.1f deg / %.2f m" % (
                raw["angle_deg"], raw["distance_m"]
            )
            ai_text = "ray %d / %.2f m" % (ai["index"], ai["distance_m"])
            print(
                "Saved #%d (%s): raw %s; AI %s\n"
                % (capture_number, label or "unlabeled", raw_text, ai_text)
            )
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
