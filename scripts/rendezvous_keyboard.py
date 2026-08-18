#!/usr/bin/env python3
"""Publish manual rendezvous commands from a terminal keypress."""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class RendezvousKeyboard(Node):
    def __init__(self):
        super().__init__("rendezvous_keyboard")
        topic = str(self.declare_parameter("topic", "/rendezvous_now").value)
        self.publisher = self.create_publisher(Bool, topic, 10)

    def publish_command(self, enabled):
        msg = Bool()
        msg.data = enabled
        self.publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.1)
        state = "requested" if enabled else "cancelled"
        self.get_logger().info(f"manual rendezvous {state}")


def main():
    if not sys.stdin.isatty():
        raise RuntimeError("rendezvous_keyboard requires an interactive terminal")

    rclpy.init()
    node = RendezvousKeyboard()
    original_settings = termios.tcgetattr(sys.stdin)
    print("r: rendezvous  c: cancel  q: quit", flush=True)

    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not ready:
                rclpy.spin_once(node, timeout_sec=0.0)
                continue
            key = sys.stdin.read(1).lower()
            if key == "r":
                node.publish_command(True)
            elif key == "c":
                node.publish_command(False)
            elif key == "q":
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
