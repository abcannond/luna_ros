#!/usr/bin/env python3
"""
Subscribes to /joy and prints human-readable feedback when axes/buttons change.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


AXIS_LABELS = {
    0: "Left stick X (strafe)",
    1: "Left stick Y (forward/back)",
    2: "Left trigger",
    3: "Right stick X (rotate)",
    4: "Right stick Y",
    5: "Right trigger",
}

BUTTON_LABELS = {
    0: "A (enable)",
    1: "B (turbo)",
    2: "X (stop)",
    3: "Y",
    4: "Left bumper",
    5: "Right bumper",
    6: "Back",
    7: "Start",
    8: "Mode",
    9: "Left stick click",
    10: "Right stick click",
}


class JoyEcho(Node):
    def __init__(self):
        super().__init__("joy_echo")
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("threshold", 0.05)
        topic = self.get_parameter("joy_topic").value
        self.threshold = float(self.get_parameter("threshold").value)
        self.prev_axes = []
        self.prev_buttons = []
        self._first = True
        self.sub = self.create_subscription(Joy, topic, self.callback, 10)
        self._warn_timer = self.create_timer(3.0, self._warn_if_no_joy)
        self.get_logger().info(
            f"JoyEcho: listening on {topic} (threshold={self.threshold}). "
            "Hold A (enable) + move sticks to drive."
        )

    def _warn_if_no_joy(self):
        if self._first:
            self.get_logger().warning(
                "Still no /joy messages — check: joy_node running? Controller plugged in? "
                "Try: ls -la /dev/input/js*   ros2 topic echo /joy"
            )
        self._warn_timer.cancel()

    def callback(self, msg: Joy):
        if self._first:
            self._first = False
            try:
                self._warn_timer.cancel()
            except Exception:
                pass
            print(
                "\n[JoyEcho] *** /joy is live *** "
                f"{len(msg.axes)} axes, {len(msg.buttons)} buttons. "
                "Move sticks / press buttons — you should see lines below. "
                "Robot moves only while A (enable) is held.\n"
            )

        for i, v in enumerate(msg.axes):
            prev = self.prev_axes[i] if i < len(self.prev_axes) else 0.0
            if abs(v - prev) > self.threshold:
                label = AXIS_LABELS.get(i, f"axis_{i}")
                if "Y" in label or "forward" in label:
                    action = "Forward" if v > 0.1 else "Backward" if v < -0.1 else "Stop (forward axis)"
                elif "X" in label and "stick" in label.lower():
                    action = "Strafe right" if v > 0.1 else "Strafe left" if v < -0.1 else "Stop (strafe axis)"
                elif "rotate" in label.lower():
                    action = "Rotate right" if v > 0.1 else "Rotate left" if v < -0.1 else "Stop (rotate axis)"
                else:
                    action = f"{label}: {v:.2f}"
                print(f"[JoyEcho] {action}")

        for i, v in enumerate(msg.buttons):
            prev = self.prev_buttons[i] if i < len(self.prev_buttons) else 0
            if v != prev:
                if i == 0 and v:
                    print("[JoyEcho] Enable (A) pressed — you can drive with sticks")
                elif i == 0 and not v:
                    print("[JoyEcho] Enable (A) released — robot should stop")
                elif i == 2 and v:
                    print("[JoyEcho] Stop (X) pressed")
                else:
                    label = BUTTON_LABELS.get(i, f"button_{i}")
                    state = "pressed" if v else "released"
                    print(f"[JoyEcho] {label} {state}")

        self.prev_axes = list(msg.axes)
        self.prev_buttons = list(msg.buttons)


def main(args=None):
    rclpy.init(args=args)
    node = JoyEcho()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
