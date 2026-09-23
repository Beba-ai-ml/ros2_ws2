"""ROS2 node that wires SAC policy to vehicle controls."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
import rclpy.logging
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from .control_mapper import ControlMapper
from .lidar_converter import LidarConverter, build_lidar_angles
from .state_builder import StateBuilder

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - optional in offline usage
    get_package_share_directory = None


_PACKAGE_URI_PREFIX = "package://"

# Layout of an installed ament package:
#   <ws>/install/<pkg>/share/<pkg>          <- package share dir
#   <ws>/src/<pkg>                          <- source tree
# so the workspace root is 4 levels above the share dir.
_SHARE_TO_WS_DEPTH = 4


def _log_info(message: str) -> None:
    """Log at INFO level without needing a Node instance."""
    try:
        rclpy.logging.get_logger("sac_driver").info(message)
    except Exception:  # pragma: no cover - logging must never break resolution
        print(f"[sac_driver] {message}")


def _package_share(package: str) -> Optional[Path]:
    if get_package_share_directory is None:
        return None
    try:
        return Path(get_package_share_directory(package))
    except Exception:
        return None


def _resolve_path(path_str: str) -> Path:
    """Resolve a model/config path in a machine-independent way.

    Supported forms:
      * ``package://<pkg>/<rel>``  -> ``<share dir of pkg>/<rel>``
      * ``~/x.pth``               -> expanded against the current user's home
      * ``/abs/x.pth``            -> used as is
      * ``weights/x.pth``         -> relative to the ``sac_driver`` share dir,
        falling back to the source tree ``<ws>/src/sac_driver/<rel>`` (so an
        un-rebuilt workspace still works), then to the current directory.
    """
    if path_str.startswith(_PACKAGE_URI_PREFIX):
        remainder = path_str[len(_PACKAGE_URI_PREFIX):]
        package, _, relative = remainder.partition("/")
        if not package or not relative:
            raise ValueError(f"Malformed package URI: {path_str!r}")
        base = _package_share(package)
        if base is None:
            raise FileNotFoundError(
                f"Cannot resolve {path_str!r}: package '{package}' share directory not found"
            )
        resolved = (base / relative).resolve()
        _log_info(f"Resolved {path_str} -> {resolved} (package share)")
        return resolved

    path = Path(path_str).expanduser()
    if path.is_absolute():
        return path

    base = _package_share("sac_driver")
    if base is not None:
        share_candidate = (base / path).resolve()
        if share_candidate.exists():
            _log_info(f"Resolved {path_str} -> {share_candidate} (package share)")
            return share_candidate

        # Not installed (or not rebuilt yet): try the source tree next to it.
        ws_root = base.parents[_SHARE_TO_WS_DEPTH - 1]
        src_candidate = (ws_root / "src" / "sac_driver" / path).resolve()
        if src_candidate.exists():
            _log_info(f"Resolved {path_str} -> {src_candidate} (source tree)")
            return src_candidate

        # Nothing exists; report the share path so the error message is the
        # canonical location the file is expected in.
        _log_info(
            f"Resolved {path_str} -> {share_candidate} (package share, file missing)"
        )
        return share_candidate

    cwd_candidate = (Path.cwd() / path).resolve()
    _log_info(f"Resolved {path_str} -> {cwd_candidate} (cwd)")
    return cwd_candidate


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class SACDriverNode(Node):
    def __init__(self) -> None:
        super().__init__("sac_driver")

        # Parameters
        model_path = self._param("model.path", "")
        model_device = self._param("model.device", "cpu")
        model_weights_only = bool(self._param("model.weights_only", False))
        model_cpu_threads = int(self._param("model.cpu_threads", 1))

        lidar_angles = self._param(
            "lidar.angles_deg",
            [-15.0, 0.0, 15.0, 30.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0, 90.0,
             95.0, 100.0, 105.0, 110.0, 115.0, 120.0, 125.0, 130.0, 135.0, 150.0, 165.0, 180.0, 195.0],
        )
        lidar_max_range_m = float(self._param("lidar.max_range_m", 20.0))
        # Match YAML and the measured raw frame: sim front (90deg) maps to
        # raw front (0deg). See the 2026-09-23 cardboard captures.
        lidar_angle_offset_deg = float(self._param("lidar.angle_offset_deg", -90.0))
        lidar_angle_direction = float(self._param("lidar.angle_direction", -1.0))
        lidar_use_interpolation = bool(self._param("lidar.use_interpolation", True))

        lidar_front_step = float(self._param("lidar.front_step_deg", 0.0))
        lidar_rear_step = float(self._param("lidar.rear_step_deg", 0.0))
        if lidar_front_step > 0.0 and lidar_rear_step > 0.0:
            lidar_angles = build_lidar_angles(lidar_front_step, lidar_rear_step)
            self.get_logger().info(
                f"Lidar: {len(lidar_angles)} angles (front_step={lidar_front_step}, rear_step={lidar_rear_step})"
            )

        stack_frames = int(self._param("state.stack_frames", 4))
        max_speed_mps = float(self._param("state.max_speed_mps", 2.5))
        speed_default = float(self._param("state.speed_default", 0.0))
        max_accel_mps2 = float(self._param("state.max_accel_mps2", 4.0))
        max_yaw_rate_rad_s = float(self._param("state.max_yaw_rate_rad_s", 3.0))

        # Timing matches the simulator: the observation stack is pushed every
        # tick (sim frame, 60 Hz) and the policy is queried every
        # `decision_every_n` ticks (sim action_repeat = 8 -> 7.5 Hz); the last
        # action is held in between.
        control_rate_hz = float(self._param("control.rate_hz", 60.0))
        decision_every_n = max(1, int(self._param("control.decision_every_n", 8)))
        control_max_steer_deg = float(self._param("control.max_steering_angle_deg", 20.0))
        control_min_steer_deg = self._param("control.min_steering_angle_deg", 5.0)
        control_steer_speed_ref = self._param("control.steer_speed_ref_mps", 8.0)
        control_max_speed_mps = float(self._param("control.max_speed_mps", 8.0))
        control_max_accel = float(self._param("control.max_accel_mps2", 2.0))
        control_speed_limit = self._param("control.speed_limit_mps", 1.0)
        control_steer_rate = self._param("control.steer_rate_limit_deg_s", 90.0)
        control_accel_rate = self._param("control.accel_rate_limit_mps2", 3.0)
        control_safe_mode = bool(self._param("control.safe_mode", True))
        control_safe_speed = self._param("control.safe_speed_limit_mps", 0.5)
        control_safe_steer_scale = float(self._param("control.safe_steer_scale", 0.5))
        control_safe_accel_scale = float(self._param("control.safe_accel_scale", 0.5))
        control_wheelbase = self._param("control.wheelbase_m", 0.33)
        control_max_yaw_rate = self._param("control.max_yaw_rate_rad_s", 2.5)
        control_default_dt = float(self._param("control.default_dt", 0.033))
        control_speed_sign = float(self._param("control.speed_sign", 1.0))
        control_steer_sign = float(self._param("control.steer_sign", 1.0))
        enable_on_start = bool(self._param("control.enable_on_start", False))

        scan_topic = self._param("topics.scan", "/scan")
        odom_topic = self._param("topics.odom", "/odom")
        cmd_topic = self._param("topics.cmd", "/drive")
        estop_topic = self._param("topics.emergency_stop", "/autonomy_lock")
        enable_service = self._param("topics.enable_service", "/sac_driver/enable")

        watchdog_timeout = float(self._param("safety.watchdog_timeout_sec", 0.5))
        log_throttle_sec = float(self._param("debug.log_throttle_sec", 1.0))

        self._log_throttle_sec = max(0.1, log_throttle_sec)
        self._last_log_time = self.get_clock().now()

        self._speed_default = speed_default
        self._watchdog_timeout = max(0.0, watchdog_timeout)
        self._decision_every_n = decision_every_n

        if not model_path:
            self.get_logger().warning("model.path is empty; node will not run inference until set.")
        self._model_path = _resolve_path(model_path) if model_path else None

        self.converter = LidarConverter(
            target_angles_deg=[float(x) for x in lidar_angles],
            max_range_m=lidar_max_range_m,
            angle_offset_deg=lidar_angle_offset_deg,
            angle_direction=lidar_angle_direction,
            use_interpolation=lidar_use_interpolation,
        )
        self.state_builder = StateBuilder(
            stack_frames=stack_frames,
            lidar_dim=len(self.converter.target_angles_deg),
            max_speed_mps=max_speed_mps,
            max_accel_mps2=max_accel_mps2,
            max_yaw_rate_rad_s=max_yaw_rate_rad_s,
        )
        self.engine = None
        if self._model_path is not None:
            try:
                from .inference_engine import InferenceEngine  # local import to allow missing torch
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().error(f"Inference engine unavailable: {exc}")
                self.engine = None
            else:
                try:
                    self.engine = InferenceEngine(
                        self._model_path,
                        device=model_device,
                        weights_only=model_weights_only,
                        cpu_threads=model_cpu_threads,
                    )
                except Exception as exc:  # pylint: disable=broad-except
                    self.get_logger().error(f"Failed to load model from {self._model_path}: {exc}")
                    self.engine = None

        self.control_mapper = ControlMapper(
            max_steering_angle_deg=control_max_steer_deg,
            max_speed_mps=control_max_speed_mps,
            max_accel_mps2=control_max_accel,
            speed_limit_mps=control_speed_limit,
            steer_rate_limit_deg_s=control_steer_rate,
            accel_rate_limit_mps2=control_accel_rate,
            safe_mode=control_safe_mode,
            safe_speed_limit_mps=control_safe_speed,
            safe_steer_scale=control_safe_steer_scale,
            safe_accel_scale=control_safe_accel_scale,
            wheelbase_m=control_wheelbase,
            max_yaw_rate_rad_s=control_max_yaw_rate,
            default_dt=control_default_dt,
            speed_sign=control_speed_sign,
            steer_sign=control_steer_sign,
            min_steering_angle_deg=(
                float(control_min_steer_deg) if control_min_steer_deg is not None else None
            ),
            steer_speed_ref_mps=(
                float(control_steer_speed_ref) if control_steer_speed_ref is not None else None
            ),
        )

        self.enabled = enable_on_start
        self._needs_reset = True
        self._emergency_stop = False

        self.latest_scan: Optional[LaserScan] = None
        self.latest_speed_mps: Optional[float] = None
        self.latest_yaw_rate: float = 0.0
        self.latest_linear_accel: float = 0.0
        self._prev_speed_mps: float = 0.0
        self._prev_odom_time = None

        # Action-repeat bookkeeping (policy output held between decisions).
        self._ticks_since_decision: int = 0
        self._held_action: Optional[tuple] = None

        self._last_scan_time = None
        self._last_odom_time = None
        self._last_control_time = None
        self._last_stop_sent = False

        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.estop_sub = None
        if estop_topic:
            self.estop_sub = self.create_subscription(Bool, estop_topic, self._on_estop, 10)

        self.cmd_pub = self.create_publisher(AckermannDriveStamped, cmd_topic, 10)
        self.enable_srv = self.create_service(SetBool, enable_service, self._on_enable)
        self.timer = self.create_timer(1.0 / max(1.0, control_rate_hz), self._on_timer)

        self.get_logger().info(f"SACDriverNode initialized (enabled={self.enabled}).")

    def _param(self, name: str, default):
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self._last_scan_time = self.get_clock().now()

    def _on_odom(self, msg: Odometry) -> None:
        current_speed = float(msg.twist.twist.linear.x)
        now = self.get_clock().now()

        if self._prev_odom_time is not None:
            dt = (now - self._prev_odom_time).nanoseconds * 1e-9
            if dt > 0.0:
                self.latest_linear_accel = (current_speed - self._prev_speed_mps) / dt

        self._prev_speed_mps = current_speed
        self._prev_odom_time = now

        self.latest_yaw_rate = float(msg.twist.twist.angular.z)
        self.latest_speed_mps = current_speed
        self._last_odom_time = now

    def _on_estop(self, msg: Bool) -> None:
        was_enabled = self.enabled
        self._emergency_stop = bool(msg.data)
        if self._emergency_stop:
            # Deadman pressed/released -> lock active, stop immediately.
            self.enabled = False
            self._needs_reset = True
            self._last_stop_sent = False
        else:
            # Auto-resume when lock is released (no service call needed).
            if not self.enabled:
                self.enabled = True
                self._needs_reset = True
                self._last_stop_sent = False
        if was_enabled != self.enabled:
            self.get_logger().info(f"Autonomy {'ENABLED' if self.enabled else 'DISABLED'} (lock={msg.data})")

    def _on_enable(self, request: SetBool.Request, response: SetBool.Response):
        if self._emergency_stop and request.data:
            response.success = False
            response.message = "Emergency stop active"
            return response

        self.enabled = bool(request.data)
        self._needs_reset = True
        response.success = True
        response.message = "Enabled" if self.enabled else "Disabled"
        return response

    def _data_ready(self, now) -> bool:
        if self.latest_scan is None or self.latest_speed_mps is None:
            return False
        if self._watchdog_timeout <= 0.0:
            return True

        def _fresh(stamp):
            if stamp is None:
                return False
            age = (now - stamp).nanoseconds * 1e-9
            return age <= self._watchdog_timeout

        if not _fresh(self._last_scan_time) or not _fresh(self._last_odom_time):
            return False
        return True

    def _publish_stop(self, now, reason: str) -> None:
        # Time spent stopped must not become an acceleration/steering step on
        # resume, including stops deduplicated by _last_stop_sent.
        self._last_control_time = None
        if self._last_stop_sent:
            return
        msg = AckermannDriveStamped()
        msg.header.stamp = now.to_msg()
        msg.drive.steering_angle = 0.0
        msg.drive.speed = 0.0
        msg.drive.acceleration = 0.0
        msg.drive.jerk = 0.0
        self.cmd_pub.publish(msg)
        self._last_stop_sent = True
        self._throttled_log(f"Published stop command ({reason}).")

    def _throttled_log(self, message: str, *args) -> None:
        now = self.get_clock().now()
        if (now - self._last_log_time).nanoseconds * 1e-9 >= self._log_throttle_sec:
            if args:
                message = message % args
            self.get_logger().info(message)
            self._last_log_time = now

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        if self._emergency_stop:
            self._publish_stop(now, "emergency_stop")
            return
        if not self.enabled:
            self._publish_stop(now, "disabled")
            return
        if self.engine is None:
            self._publish_stop(now, "no_model")
            return
        if not self._data_ready(now):
            self._throttled_log(
                "Waiting for data: scan=%s odom=%s",
                self.latest_scan is not None,
                self.latest_speed_mps is not None,
            )
            self._publish_stop(now, "data_missing")
            return

        if self._needs_reset or self._last_control_time is None:
            dt = None
        else:
            dt = (now - self._last_control_time).nanoseconds * 1e-9
            if dt <= 0.0:
                dt = None
        self._last_control_time = now

        try:
            lidar_norm = self.converter.convert(self.latest_scan)
            speed_mps = float(self.latest_speed_mps) if self.latest_speed_mps is not None else self._speed_default
            sensor_accel = self.latest_linear_accel
            sensor_yaw = self.latest_yaw_rate

            if self._needs_reset:
                # Fresh episode: like the simulator, the stack is filled with the
                # first frame and the steering feedback starts at "straight".
                self.control_mapper.reset(speed_mps)
                first_obs = self.state_builder.build_observation(
                    lidar_norm, speed_mps, 0.0, sensor_accel, sensor_yaw,
                )
                state = self.state_builder.reset(first_obs)
                self._held_action = None
                self._ticks_since_decision = 0
                self._needs_reset = False
            else:
                # Steering feedback = the command we sent last tick, in policy
                # space (sim: servo_norm = (steer_action + 1) / 2).
                state = self.state_builder.update(
                    lidar_norm, speed_mps, self.control_mapper.last_steer_norm,
                    sensor_accel, sensor_yaw,
                )

            if self._held_action is None or self._ticks_since_decision >= self._decision_every_n:
                steer, accel = self.engine.get_action(state)
                if not math.isfinite(steer) or not math.isfinite(accel):
                    raise ValueError("Policy returned non-finite action")
                self._held_action = (steer, accel)
                self._ticks_since_decision = 0
            else:
                steer, accel = self._held_action
            self._ticks_since_decision += 1

            cmd = self.control_mapper.map_to_ackermann(steer, accel, speed_mps, dt=dt)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"Control loop error: {exc}")
            self._publish_stop(now, "exception")
            return

        msg = AckermannDriveStamped()
        msg.header.stamp = now.to_msg()
        msg.drive.steering_angle = float(cmd.get("steering_angle", 0.0))
        msg.drive.speed = float(cmd.get("speed", 0.0))
        msg.drive.acceleration = float(cmd.get("acceleration", 0.0))
        msg.drive.jerk = 0.0
        self.cmd_pub.publish(msg)
        self._last_stop_sent = False


def main() -> None:
    rclpy.init()
    node = SACDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
