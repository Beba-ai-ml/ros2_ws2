#!/usr/bin/env python3
"""Read SAC parameters directly, without the ros2cli daemon or control calls."""

import argparse
import json
import os
from pathlib import Path
import time


PARAMETERS = [
    "lidar.angle_offset_deg", "lidar.angle_direction", "model.path", "model.cpu_threads",
    "lidar.max_invalid_gap_deg",
    "control.enable_on_start", "control.rate_hz", "control.decision_every_n",
    "control.speed_limit_mps", "control.safe_speed_limit_mps", "control.safe_mode",
    "control.speed_sign", "control.steer_sign", "safety.watchdog_timeout_sec",
    "topics.scan", "topics.odom", "topics.emergency_stop",
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    parser.add_argument(
        "--udp-local", action="store_true",
        help="use the isolated domain-0 UDP diagnostic profile for this client only",
    )
    args = parser.parse_args()
    if args.timeout_sec <= 0:
        parser.error("--timeout-sec must be positive")
    if args.udp_local:
        if os.environ.get("ROS_DOMAIN_ID", "0") != "0":
            parser.error("the bundled UDP profile is for ROS_DOMAIN_ID=0")
        profile = Path(__file__).resolve().parent / "diagnostics" / "fastdds_local_readonly.xml"
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(profile)

    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.srv import GetParameters

    rclpy.init(args=[])
    node = Node("read_sac_parameters_" + str(os.getpid()))
    client = node.create_client(GetParameters, "/sac_driver/get_parameters")
    deadline = time.monotonic() + args.timeout_sec
    future = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if future is None and client.service_is_ready():
                request = GetParameters.Request()
                request.names = PARAMETERS
                future = client.call_async(request)
            rclpy.spin_once(node, timeout_sec=0.05)
            if future is not None and future.done():
                fields = {
                    1: "bool_value", 2: "integer_value", 3: "double_value",
                    4: "string_value",
                }
                values = future.result().values
                result = {
                    name: getattr(value, fields[value.type]) if value.type in fields else None
                    for name, value in zip(PARAMETERS, values)
                }
                print(json.dumps(result, indent=2, ensure_ascii=False))
                return 0
        parser.exit(1, "Timed out reading /sac_driver/get_parameters; no settings changed.\n")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
