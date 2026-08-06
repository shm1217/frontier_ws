#!/usr/bin/env python3
"""Read one front or back DW3000 tag and publish its anchor range."""

import re

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

try:
    import serial
except ImportError:
    serial = None


LINE_RE = re.compile(
    r"T(?P<tag_id>\d+),mask:(?P<mask>[0-9A-Fa-f]+),seq:(?P<seq>\d+),"
    r"beacon:(?P<beacon>\d+),range:\((?P<ranges>[-\d,]+)\),"
    r"ancid:\((?P<ancid>[-\d,]+)\)"
)


class UwbRangeNode(Node):
    def __init__(self):
        super().__init__("uwb_range_node_uwb")
        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("anchor_index", 0)
        self.declare_parameter("frame_id", "uwb_anchor")
        self.declare_parameter("tag_name", "front")
        self.declare_parameter("min_range", 0.05)
        self.declare_parameter("max_range", 50.0)

        port = self.get_parameter("serial_port").value
        baud = int(self.get_parameter("baud_rate").value)
        self.anchor_index = int(self.get_parameter("anchor_index").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.tag_name = str(self.get_parameter("tag_name").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        if self.tag_name not in ("front", "back"):
            raise ValueError("tag_name must be 'front' or 'back'")
        self.pub = self.create_publisher(Range, f"uwb/{self.tag_name}/range", 10)

        self.ser = None
        if serial is None:
            self.get_logger().error("pyserial is not installed")
        else:
            try:
                self.ser = serial.Serial(port, baud, timeout=0.05)
                self.get_logger().info(f"serial connected: {port} @ {baud}")
            except serial.SerialException as exc:
                self.get_logger().error(f"serial open failed: {exc}")
        self.timer = self.create_timer(0.02, self.read_serial)

    def read_serial(self):
        if self.ser is None or self.ser.in_waiting == 0:
            return
        try:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
        except Exception as exc:
            self.get_logger().warn(f"serial read failed: {exc}")
            return
        match = LINE_RE.search(line)
        if match is None:
            return

        ranges = [int(v) / 100.0 for v in match.group("ranges").split(",")]
        mask = int(match.group("mask"), 16)
        i = self.anchor_index
        if i < 0 or i >= len(ranges) or not (mask & (1 << i)):
            return
        value = ranges[i]
        if not self.min_range <= value <= self.max_range:
            return

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 2.0 * 3.141592653589793
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        msg.range = value
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UwbRangeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
