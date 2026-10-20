"""KeyboardTyper node: presses a key when it is told a letter.

Subscribes:
    key           std_msgs/String: one letter out of KEY_POSITIONS, for example "a"
    arm_state     std_msgs/Float64MultiArray: where the arm is now
Publishes:
    arm_commands  std_msgs/Float64MultiArray, 6 values: 5 joint angles (deg) and the gripper (%)

For each letter the fingertip moves to a point above the key, down onto it, and back up.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from arm_control.kinematics import inverse_kinematics

# Where the top of each key is, in the robot's base frame (metres). These values are for the
# SIMULATED keyboard. Real measurements belong in real_keyboard_positions.py.
KEY_POSITIONS = {
    "a": [0.300, 0.070, -0.012],
    "s": [0.300, 0.000, -0.012],
    "d": [0.300, -0.070, -0.012],
}
HOVER_HEIGHT = 0.04   # metres above the key before and after pressing
PRESS_DEPTH = 0.008   # metres below the key's top to aim for when pressing
MOVE_TIME = 1.5       # seconds for each of the three moves
RATE_HZ = 20.0
FINGER_DOWN = [0.0, 0.0, -1.0]
GRIPPER_CLOSED = 0.0


class KeyboardTyper(Node):
    def __init__(self):
        super().__init__("keyboard_typer")
        self.arm_state = None
        self.waypoints = []      # joint-angle targets still to visit, each a list of 6 numbers
        self.start = None        # where the current move started
        self.steps_done = 0
        self.publisher = self.create_publisher(Float64MultiArray, "arm_commands", 10)
        self.create_subscription(String, "key", self.on_key, 10)
        self.create_subscription(Float64MultiArray, "arm_state", self.on_arm_state, 10)
        self.create_timer(1.0 / RATE_HZ, self.on_timer)
        self.get_logger().info(f"KeyboardTyper started. Keys: {sorted(KEY_POSITIONS)}")

    def on_arm_state(self, msg):
        self.arm_state = list(msg.data)

    def on_key(self, msg):
        letter = msg.data.strip().lower()
        if letter not in KEY_POSITIONS:
            self.get_logger().warn(f"Unknown key {letter!r}. Known keys: {sorted(KEY_POSITIONS)}")
            return
        if self.waypoints:
            self.get_logger().warn("Still busy with the previous key. Ignored.")
            return
        if self.arm_state is None:
            self.get_logger().warn("No arm_state received yet.")
            return
        hover = np.array(KEY_POSITIONS[letter]) + [0.0, 0.0, HOVER_HEIGHT]
        press = np.array(KEY_POSITIONS[letter]) - [0.0, 0.0, PRESS_DEPTH]
        angles = self.arm_state
        waypoints = []
        for target in (hover, press, hover):
            angles, error = inverse_kinematics(target, FINGER_DOWN, angles)
            if not np.isfinite(error) or error > 0.005:
                self.get_logger().warn(
                    f"Key {letter!r} is out of reach (error {error * 1000:.1f} mm). Not moving.")
                return
            waypoints.append(angles.tolist() + [GRIPPER_CLOSED])
        # Install the plan only after all three waypoints have passed the reachability check.
        self.waypoints = waypoints
        self.start = self.arm_state.copy()
        self.steps_done = 0
        self.get_logger().info(f"Typing {letter!r}")

    def on_timer(self):
        if not self.waypoints:
            return
        self.steps_done += 1
        fraction = min(self.steps_done / (MOVE_TIME * RATE_HZ), 1.0)
        start = np.asarray(self.start)
        target = np.asarray(self.waypoints[0])
        command = start + (target - start) * fraction
        self.publisher.publish(Float64MultiArray(data=command.tolist()))
        if fraction >= 1.0:
            self.start = self.waypoints.pop(0)
            self.steps_done = 0


def main():
    rclpy.init()
    node = KeyboardTyper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
