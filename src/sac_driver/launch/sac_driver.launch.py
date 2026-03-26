from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - optional in offline usage
    get_package_share_directory = None


def _default_paths():
    if get_package_share_directory is None:
        return "", ""
    pkg_share = get_package_share_directory("sac_driver")
    params = os.path.join(pkg_share, "config", "driver_params.yaml")
    model = "/home/laptop/shared/różne/session_car_1_3.pth"
    return params, model


def generate_launch_description() -> LaunchDescription:
    default_params, default_model = _default_paths()

    params_file = LaunchConfiguration("params_file")
    model_path = LaunchConfiguration("model_path")
    namespace = LaunchConfiguration("namespace")

    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    servo_topic = LaunchConfiguration("servo_topic")
    cmd_topic = LaunchConfiguration("cmd_topic")
    estop_topic = LaunchConfiguration("estop_topic")
    enable_service = LaunchConfiguration("enable_service")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("model_path", default_value=default_model),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("servo_topic", default_value="/commands/servo/position"),
            DeclareLaunchArgument("cmd_topic", default_value="/drive"),
            DeclareLaunchArgument("estop_topic", default_value="/autonomy_lock"),
            DeclareLaunchArgument("enable_service", default_value="/sac_driver/enable"),
            Node(
                package="sac_driver",
                executable="sac_driver_node",
                name="sac_driver",
                namespace=namespace,
                output="screen",
                parameters=[
                    params_file,
                    {
                        "model.path": model_path,
                        "topics.scan": scan_topic,
                        "topics.odom": odom_topic,
                        "topics.servo": servo_topic,
                        "topics.cmd": cmd_topic,
                        "topics.emergency_stop": estop_topic,
                        "topics.enable_service": enable_service,
                    },
                ],
            ),
        ]
    )
