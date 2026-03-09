#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class JoyModeManager(Node):
    """
    Gates teleop commands with a manual deadman and drives the autonomy lock
    so navigation is only enabled while the autonomy button is held.
    """

    def __init__(self):
        super().__init__('joy_mode_manager')

        # Parameters to adapt to different controllers/topics without touching joy config.
        self.manual_button = self.declare_parameter('manual_button', 4).value
        self.autonomy_button = self.declare_parameter('autonomy_button', 5).value
        self.teleop_in_topic = self.declare_parameter('teleop_in', 'teleop').value
        self.teleop_out_topic = self.declare_parameter('teleop_out', 'teleop_gated').value
        self.autonomy_lock_topic = self.declare_parameter('autonomy_lock_topic', 'autonomy_lock').value
        self.lock_publish_rate = self.declare_parameter('lock_publish_rate', 50.0).value

        self._manual_active = False
        self._autonomy_active = False
        self._last_lock_state = True  # start locked

        self._teleop_pub = self.create_publisher(AckermannDriveStamped, self.teleop_out_topic, 10)
        self._autonomy_lock_pub = self.create_publisher(Bool, self.autonomy_lock_topic, 10)

        self.create_subscription(Joy, 'joy', self._joy_callback, 10)
        self.create_subscription(AckermannDriveStamped, self.teleop_in_topic, self._teleop_callback, 10)

        # Keep the lock topic alive even if no joy messages arrive for a moment.
        self.create_timer(1.0 / self.lock_publish_rate, self._publish_lock)

    def _joy_callback(self, msg: Joy):
        # Buttons array is zero-based; default maps to LB=4, RB=5 on most pads.
        self._manual_active = self._button_pressed(msg.buttons, self.manual_button)
        self._autonomy_active = self._button_pressed(msg.buttons, self.autonomy_button)

    def _teleop_callback(self, msg: AckermannDriveStamped):
        if self._manual_active:
            self._teleop_pub.publish(msg)

    def _publish_lock(self):
        # Lock navigation unless autonomy button is held and manual is not.
        lock_msg = Bool()
        lock_msg.data = not self._autonomy_active or self._manual_active
        self._autonomy_lock_pub.publish(lock_msg)

        # On transition to locked, send a single zero command to stop motion quickly.
        if lock_msg.data and not self._last_lock_state:
            self._teleop_pub.publish(AckermannDriveStamped())

        self._last_lock_state = lock_msg.data

    @staticmethod
    def _button_pressed(buttons, index):
        return 0 <= index < len(buttons) and bool(buttons[index])


def main(args=None):
    rclpy.init(args=args)
    node = JoyModeManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
