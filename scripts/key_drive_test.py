#!/usr/bin/env python3
"""Automated drive test (car must be OFF THE GROUND).

Publishes forward / reverse / left / right on /teleop_gated for a couple of
seconds each and checks feedback: motor ERPM from /sensors/core and the servo
command on /commands/servo/position.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from vesc_msgs.msg import VescStateStamped
from std_msgs.msg import Float64


class Tester(Node):
    def __init__(self):
        super().__init__('key_drive_test')
        self.pub = self.create_publisher(AckermannDriveStamped, '/teleop_gated', 10)
        self.erpm = None
        self.servo = None
        self.cmd_speed = None
        self.create_subscription(VescStateStamped, '/sensors/core', self._core, 10)
        self.create_subscription(Float64, '/commands/servo/position', self._servo, 10)
        self.create_subscription(Float64, '/commands/motor/speed', self._mspeed, 10)

    def _core(self, m):
        self.erpm = m.state.speed

    def _servo(self, m):
        self.servo = m.data

    def _mspeed(self, m):
        self.cmd_speed = m.data

    def drive(self, speed, steer, secs):
        t_end = time.time() + secs
        samples = []
        while time.time() < t_end:
            msg = AckermannDriveStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.drive.speed = float(speed)
            msg.drive.steering_angle = float(steer)
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() > t_end - secs / 2:
                samples.append((self.erpm, self.servo, self.cmd_speed))
        return samples


def avg(vals):
    vals = [v for v in vals if v is not None]
    return sum(vals) / len(vals) if vals else None


def main():
    rclpy.init()
    t = Tester()
    speed = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    steer = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
    print('waiting for /sensors/core ...')
    t0 = time.time()
    while t.erpm is None and time.time() - t0 < 10:
        rclpy.spin_once(t, timeout_sec=0.1)
    if t.erpm is None:
        print('FAIL: no VESC telemetry on /sensors/core')
        rclpy.shutdown()
        sys.exit(1)

    SPEED_SIGN = -1.0  # same convention as key_drive.py: negative cmd = car forward
    cases = [
        ('FORWARD', SPEED_SIGN * speed, 0.0),
        ('STOP', 0.0, 0.0),
        ('REVERSE', -SPEED_SIGN * speed, 0.0),
        ('STOP', 0.0, 0.0),
        ('LEFT', 0.0, steer),
        ('RIGHT', 0.0, -steer),
        ('CENTER', 0.0, 0.0),
    ]
    results = []
    for name, sp, st in cases:
        s = t.drive(sp, st, 2.5)
        e, sv, cs = avg([x[0] for x in s]), avg([x[1] for x in s]), avg([x[2] for x in s])
        results.append((name, sp, st, e, sv, cs))
        print('%-8s cmd speed %+5.2f steer %+5.2f -> erpm %8s  servo %s  motor_cmd %s' % (
            name, sp, st, '%.0f' % e if e is not None else 'n/a',
            '%.3f' % sv if sv is not None else 'n/a', '%.0f' % cs if cs is not None else 'n/a'))
    # final zero
    t.drive(0.0, 0.0, 0.5)

    ok = True
    by = {r[0]: r for r in results}
    if by['FORWARD'][3] is None or SPEED_SIGN * by['FORWARD'][3] < 200:
        print('FAIL: forward - motor did not spin (erpm %s)' % by['FORWARD'][3]); ok = False
    if by['REVERSE'][3] is None or SPEED_SIGN * by['REVERSE'][3] > -200:
        print('FAIL: reverse - motor did not spin backwards (erpm %s)' % by['REVERSE'][3]); ok = False
    if by['STOP'][3] is not None and abs(by['STOP'][3]) > 300:
        print('FAIL: stop - motor still spinning (erpm %s)' % by['STOP'][3]); ok = False
    l, r, c = by['LEFT'][4], by['RIGHT'][4], by['CENTER'][4]
    if None in (l, r, c) or abs(l - c) < 0.05 or abs(r - c) < 0.05 or (l - c) * (r - c) > 0:
        print('FAIL: steering - servo cmd left %s right %s center %s' % (l, r, c)); ok = False
    print('RESULT:', 'PASS' if ok else 'FAIL')
    t.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 2)


if __name__ == '__main__':
    main()
