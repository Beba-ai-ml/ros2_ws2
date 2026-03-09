#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanYawRotator(Node):
    """Republishes a LaserScan with a fixed yaw offset applied to all angles."""

    def __init__(self) -> None:
        super().__init__("scan_yaw_rotator")

        self.input_topic = self.declare_parameter("input_topic", "/scan").value
        self.output_topic = self.declare_parameter("output_topic", "/scan_2").value
        self.output_frame_id = str(self.declare_parameter("output_frame_id", "").value)
        yaw_offset_deg = float(self.declare_parameter("yaw_offset_deg", -220.0).value)
        self.yaw_offset_rad = math.radians(yaw_offset_deg)

        self.create_subscription(LaserScan, self.input_topic, self.scan_callback, 10)
        self.publisher = self.create_publisher(LaserScan, self.output_topic, 10)

        self.get_logger().info(
            f"Republishing {self.input_topic} to {self.output_topic} with "
            f"{yaw_offset_deg} deg yaw offset"
        )

    def scan_callback(self, msg: LaserScan) -> None:
        rotated_scan = LaserScan()
        rotated_scan.header = msg.header
        if self.output_frame_id:
            rotated_scan.header.frame_id = self.output_frame_id
        rotated_scan.angle_min = msg.angle_min + self.yaw_offset_rad
        rotated_scan.angle_max = msg.angle_max + self.yaw_offset_rad
        rotated_scan.angle_increment = msg.angle_increment
        rotated_scan.time_increment = msg.time_increment
        rotated_scan.scan_time = msg.scan_time
        rotated_scan.range_min = msg.range_min
        rotated_scan.range_max = msg.range_max

        # Copy ranges/intensities to avoid mutating the incoming message.
        rotated_scan.ranges = list(msg.ranges)
        rotated_scan.intensities = list(msg.intensities)

        self.publisher.publish(rotated_scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanYawRotator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
