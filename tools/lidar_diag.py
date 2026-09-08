#!/usr/bin/env python3
"""Diagnose lidar-to-NN mapping. Tests different offset and channel configs."""
import math, os, time, sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# This script lives in <ws>/tools/, so the workspace root is one level up.
WS_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS_ROOT, 'src', 'sac_driver'))
from sac_driver.lidar_converter import LidarConverter, build_lidar_angles
from sac_driver.state_builder import StateBuilder
from sac_driver.inference_engine import InferenceEngine

MODEL = os.path.join(WS_ROOT, 'src', 'sac_driver', 'weights', 'session_car_1_3.pth')

angles_450 = build_lidar_angles(0.5, 2.0)

# Test 3 offset configurations
CONFIGS = {
    'offset=-90': LidarConverter(angles_450, max_range_m=20.0, angle_offset_deg=-90.0),
    'offset=  0': LidarConverter(angles_450, max_range_m=20.0, angle_offset_deg=0.0),
    'offset=+90': LidarConverter(angles_450, max_range_m=20.0, angle_offset_deg=90.0),
}

class Diag(Node):
    def __init__(self):
        super().__init__('lidar_diag')
        self.scan = None
        self.speed = 0.0
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self.engine = InferenceEngine(MODEL, device='cpu', weights_only=False)
        self.get_logger().info('Model loaded. Waiting for scan...')

    def _on_scan(self, msg):
        self.scan = msg

    def _on_odom(self, msg):
        self.speed = float(msg.twist.twist.linear.x)

    def run_once(self):
        if self.scan is None:
            return

        # Find where obstacle is in raw lidar
        ranges = list(self.scan.ranges)
        amin = self.scan.angle_min
        ainc = self.scan.angle_increment
        close_angles = []
        for i, r in enumerate(ranges):
            if 0.01 < r < 0.5:
                a = math.degrees(amin + i * ainc)
                close_angles.append(a)

        if close_angles:
            center = sum(close_angles) / len(close_angles)
            obstacle_str = f'obstacle @ {center:+.0f}deg (lidar frame)'
        else:
            obstacle_str = 'no close obstacle'

        print(f'\n{"="*70}')
        print(f'  {obstacle_str}  |  speed={self.speed:.2f} m/s')
        print(f'{"="*70}')
        print(f'  {"Config":<14s}  {"Steer":>7s}  {"Accel":>7s}  {"Direction":<20s}  {"Front ray":>10s}')
        print(f'  {"-"*64}')

        for name, converter in CONFIGS.items():
            lidar = converter.convert(self.scan)

            # Find which observation index has the obstacle (lowest value)
            front_idx = len(angles_450) // 4  # ~index at 90deg training angle
            front_val = float(lidar[front_idx])

            # Build state with OLD format: [collision=0, speed, servo=0.5, accel, yaw]
            # AND new format: [speed, steer, accel_fb, accel, yaw]
            # Test both channel orders

            for ch_name, channels in [
                ('new', [0.0, 0.0, 0.0, 0.0, 0.0]),   # [speed, steer, accel_fb, accel, yaw]
                ('old', [0.0, 0.0, 0.5, 0.0, 0.0]),    # [collision=0, speed, servo=0.5, accel, yaw]
            ]:
                obs = np.array(list(lidar) + channels, dtype=np.float32)
                sb = StateBuilder(stack_frames=4, lidar_dim=450, max_speed_mps=6.0)
                state = sb.reset(obs)
                steer, accel = self.engine.get_action(state)

                if abs(steer) < 0.05:
                    direction = 'STRAIGHT'
                elif steer > 0:
                    direction = f'LEFT  ({steer:+.3f})'
                else:
                    direction = f'RIGHT ({steer:+.3f})'

                label = f'{name}/{ch_name}'
                print(f'  {label:<14s}  {steer:+7.3f}  {accel:7.3f}  {direction:<20s}  {front_val:10.3f}')

def main():
    rclpy.init()
    node = Diag()

    # Wait for first scan
    while node.scan is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    print('\nHold cardboard to RIGHT, LEFT, or FRONT of car.')
    print('Press Enter for each reading. Ctrl+C to quit.\n')

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.05)
            input('>>> Press Enter to take reading...')
            # Get fresh scan
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.05)
            node.run_once()
    except (KeyboardInterrupt, EOFError):
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
