#!/usr/bin/env python3
"""GateNode variant that assigns global goals only after UWB merge validation."""

import rclpy
from std_msgs.msg import Bool

from merge_map.gate_node import GateNode


class GateNodeUwb(GateNode):
    def __init__(self):
        self.merge_valid = False
        super().__init__()
        self._valid_sub = self.create_subscription(
            Bool, "/merge_map_uwb_valid", self._on_valid, 10
        )

    def _on_valid(self, msg):
        self.merge_valid = bool(msg.data)

    def on_timer(self):
        if not self.merge_valid:
            return
        super().on_timer()


def main(args=None):
    rclpy.init(args=args)
    node = GateNodeUwb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
