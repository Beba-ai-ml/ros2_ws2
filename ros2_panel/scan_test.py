#!/usr/bin/env python3
"""Scan test - shows which lidar indices detect a close object."""
import rclpy
from sensor_msgs.msg import LaserScan
import math
import numpy as np

def main():
    rclpy.init()
    node = rclpy.create_node('scan_test')
    done = False

    def cb(msg):
        nonlocal done
        if done:
            return
        done = True

        ranges = np.array(msg.ranges)
        angle_min_deg = math.degrees(msg.angle_min)
        inc_deg = math.degrees(msg.angle_increment)

        # Find closest points (likely the test object)
        valid = np.where(np.isfinite(ranges) & (ranges > 0.01))[0]
        if len(valid) == 0:
            print("No valid readings!")
            return

        sorted_idx = valid[np.argsort(ranges[valid])]

        print("=== 20 CLOSEST POINTS ===")
        print(f"{'Index':>6} {'Angle':>10} {'Distance':>10}")
        print("-" * 30)
        for i in sorted_idx[:20]:
            angle = angle_min_deg + i * inc_deg
            print(f"{i:6d} {angle:9.2f}° {ranges[i]:9.3f} m")

        print(f"\n=== SUMMARY ===")
        closest = sorted_idx[0]
        closest_angle = angle_min_deg + closest * inc_deg
        print(f"Closest point: index {closest}, angle {closest_angle:.2f}°, dist {ranges[closest]:.3f} m")
        print(f"\nIf this object is DIRECTLY IN FRONT of the vehicle,")
        print(f"then FRONT = {closest_angle:.1f}° in the raw lidar frame.")
        print(f"This car's lidar is mounted backwards: expected front ≈ ±180°;")
        print(f"AI config uses angle_offset=+90° and static TF yaw=180°.")
        if abs(abs(closest_angle) - 180.0) < 10:
            print("=> Backwards lidar mount looks CORRECT (front ≈ ±180°)")
        else:
            print("=> This does not match the configured 180° mount; check physical orientation.")

    sub = node.create_subscription(LaserScan, '/scan', cb, 10)
    print("Put an object DIRECTLY IN FRONT of the vehicle, close (~20cm).")
    print("Waiting for /scan...")

    import time
    t = time.time()
    while not done and (time.time() - t) < 30:
        rclpy.spin_once(node, timeout_sec=1.0)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
