#!/usr/bin/env python3
"""Record lidar scans for 20s and show where the closest object is."""
import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

DURATION = 20.0
CLOSE_THRESHOLD = 0.5  # meters — cardboard should be well below this

class LidarTest(Node):
    def __init__(self):
        super().__init__('lidar_test')
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.start = time.time()
        self.scans = []
        self.get_logger().info(f'Recording for {DURATION}s — move the cardboard now!')

    def cb(self, msg):
        elapsed = time.time() - self.start
        if elapsed > DURATION:
            return

        ranges = list(msg.ranges)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        n = len(ranges)

        # Find all close readings
        close = []
        for i, r in enumerate(ranges):
            if 0.01 < r < CLOSE_THRESHOLD:
                angle_rad = angle_min + i * angle_inc
                angle_deg = math.degrees(angle_rad)
                close.append((angle_deg, r))

        if close:
            angles = [c[0] for c in close]
            dists = [c[1] for c in close]
            min_d = min(dists)
            min_a = angles[dists.index(min_d)]
            self.get_logger().info(
                f't={elapsed:5.1f}s | closest={min_d:.3f}m @ {min_a:+7.1f}deg | '
                f'sector=[{min(angles):+.0f} to {max(angles):+.0f}]deg | {len(close)} rays'
            )

        # Store raw for post-analysis
        self.scans.append({
            't': elapsed,
            'angle_min': angle_min,
            'angle_inc': angle_inc,
            'ranges': ranges,
        })

def main():
    rclpy.init()
    node = LidarTest()
    t0 = time.time()
    while time.time() - t0 < DURATION + 1.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    # Post-analysis: summarize angle sectors over time
    print('\n' + '='*70)
    print('SUMMARY — Closest object angle over time')
    print('='*70)
    print(f'{"Time":>6s}  {"Angle(deg)":>10s}  {"Dist(m)":>8s}  {"Sector":>20s}')
    print('-'*70)

    for s in node.scans:
        ranges = s['ranges']
        close = []
        for i, r in enumerate(ranges):
            if 0.01 < r < CLOSE_THRESHOLD:
                a = math.degrees(s['angle_min'] + i * s['angle_inc'])
                close.append((a, r))
        if close:
            dists = [c[1] for c in close]
            angles = [c[0] for c in close]
            idx = dists.index(min(dists))
            print(f"{s['t']:6.1f}  {angles[idx]:+10.1f}  {dists[idx]:8.3f}  "
                  f"[{min(angles):+.0f} to {max(angles):+.0f}]")

    print('='*70)
    print('Lidar frame: 0deg=front, +90=left, -90=right, +/-180=rear')
    print(f'Total scans: {len(node.scans)}')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
