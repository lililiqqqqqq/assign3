"""Manually run this executable once to send p, home, o, home, o, home, l.

Requires measured real keyboard coordinates and a separately started IK node.
This helper sends only EE targets; it neither starts hardware nor enables motors.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from arm_control.real_keyboard_positions import HOME_POSITION, REAL_KEY_POSITIONS

SEQUENCE = ("p", "home", "o", "home", "o", "home", "l")
MOVE_WAIT_SEC = 5.0  # Increase for the real robot if it needs more settling time.
INITIAL_WAIT_SEC = 1.0  # Allow ROS discovery before sending the first target.


class PoolTyper(Node):
    def __init__(self):
        super().__init__("pool_typer")
        self.done = False
        self.index = 0
        self.positions = {key: REAL_KEY_POSITIONS.get(key) for key in ("p", "o", "l")}
        self.positions["home"] = HOME_POSITION
        missing = [key for key, value in self.positions.items() if value is None]
        if missing:
            self.get_logger().warn(
                f"Missing calibration values: {', '.join(missing)}. Refusing to move.")
            self.done = True
            return
        for key, value in self.positions.items():
            if len(value) != 3 or not all(math.isfinite(v) for v in value):
                self.get_logger().warn(f"Invalid calibration for {key}. Refusing to move.")
                self.done = True
                return
        self.publisher = self.create_publisher(PoseStamped, "/ik_target", 10)
        self.timer = self.create_timer(INITIAL_WAIT_SEC, self.on_timer)
        self.get_logger().info("Calibrated sequence ready; waiting for an IK subscriber.")

    def on_timer(self):
        if self.publisher.get_subscription_count() == 0:
            return
        key = SEQUENCE[self.index]
        target = PoseStamped()
        target.header.frame_id = "base"
        target.header.stamp = self.get_clock().now().to_msg()
        x, y, z = self.positions[key]
        target.pose.position.x = float(x)
        target.pose.position.y = float(y)
        target.pose.position.z = float(z)
        target.pose.orientation.w = 1.0
        self.publisher.publish(target)
        self.get_logger().info(f"Sent {key}")
        self.index += 1
        if self.index == len(SEQUENCE):
            self.timer.cancel()
            self.done = True
        elif self.index == 1:
            # After the first target, use the full movement delay between targets.
            self.destroy_timer(self.timer)
            self.timer = self.create_timer(MOVE_WAIT_SEC, self.on_timer)


def main():
    rclpy.init()
    node = PoolTyper()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
