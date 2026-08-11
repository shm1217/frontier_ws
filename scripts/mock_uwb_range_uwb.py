#!/usr/bin/env python3
"""Publish simulated UWB ranges from Gazebo Classic ground-truth poses.

The configured anchor is one physical point in Gazebo's world frame.  Two
virtual tags are mounted symmetrically on each robot's forward axis.  For
robot ``tb3_0``, for example:

    front/back range = distance(anchor, base +/- rotated tag offset)

The result is published with the same sensor_msgs/Range interface as hardware.
"""

import math
import random

from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class MockUwbRangeNode(Node):
    def __init__(self):
        super().__init__("mock_uwb_range_uwb")

        self.robots = list(
            self.declare_parameter("robot_namespaces", ["tb3_0", "tb3_1"]).value
        )
        self.anchor_x = float(self.declare_parameter("anchor_x", -3.0).value)
        self.anchor_y = float(self.declare_parameter("anchor_y", 5.0).value)
        self.model_states_topic = str(
            self.declare_parameter("model_states_topic", "/gazebo/model_states").value
        )
        initial_x = list(
            self.declare_parameter("initial_world_x", [-0.978, 0.978]).value
        )
        initial_y = list(self.declare_parameter("initial_world_y", [1.92, 1.92]).value)
        initial_yaw = list(
            self.declare_parameter("initial_world_yaw", [0.0, 0.0]).value
        )
        self.tag_offset = float(
            self.declare_parameter("tag_offset_from_base_m", 0.15).value
        )
        self.publish_rate = float(self.declare_parameter("publish_rate_hz", 10.0).value)
        self.noise_stddev = float(self.declare_parameter("noise_stddev_m", 0.0).value)
        self.min_range = float(self.declare_parameter("min_range_m", 0.05).value)
        self.max_range = float(self.declare_parameter("max_range_m", 50.0).value)
        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate_hz must be greater than zero")
        if self.noise_stddev < 0.0:
            raise ValueError("noise_stddev_m must not be negative")
        if not (
            len(initial_x) == len(self.robots)
            and len(initial_y) == len(self.robots)
            and len(initial_yaw) == len(self.robots)
        ):
            raise ValueError(
                "initial_world_x/y/yaw lengths must match robot_namespaces"
            )

        self.world_poses = {}
        self.pose_source = {}
        self.initial_world_pose = {
            robot: (float(initial_x[i]), float(initial_y[i]), float(initial_yaw[i]))
            for i, robot in enumerate(self.robots)
        }
        self.model_states_sub = self.create_subscription(
            ModelStates,
            self.model_states_topic,
            self.on_model_states,
            10,
        )
        self.odom_subs = [
            self.create_subscription(
                Odometry,
                f"/{robot}/odom",
                lambda msg, name=robot: self.on_odom(msg, name),
                20,
            )
            for robot in self.robots
        ]
        self.range_publishers = {
            robot: {
                "front": self.create_publisher(Range, f"/{robot}/uwb/front/range", 10),
                "back": self.create_publisher(Range, f"/{robot}/uwb/back/range", 10),
            }
            for robot in self.robots
        }
        self.last_pose_warning = {robot: 0.0 for robot in self.robots}
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_ranges)

        self.get_logger().info(
            f"mock UWB world anchor=({self.anchor_x:.2f}, {self.anchor_y:.2f}), "
            f"robots={self.robots}, rate={self.publish_rate:.1f}Hz, "
            f"noise_stddev={self.noise_stddev:.3f}m, "
            f"primary_source={self.model_states_topic}, fallback=/<robot>/odom"
        )

    def on_model_states(self, msg):
        poses = {}
        for name, pose in zip(msg.name, msg.pose):
            if name in self.range_publishers:
                q = pose.orientation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )
                poses[name] = (
                    float(pose.position.x),
                    float(pose.position.y),
                    yaw,
                )
                self.pose_source[name] = "gazebo_model_states"
        self.world_poses.update(poses)

    def on_odom(self, msg, robot):
        # Never overwrite direct Gazebo ground truth if it becomes available.
        if self.pose_source.get(robot) == "gazebo_model_states":
            return
        origin_x, origin_y, origin_yaw = self.initial_world_pose[robot]
        local_x = float(msg.pose.pose.position.x)
        local_y = float(msg.pose.pose.position.y)
        c = math.cos(origin_yaw)
        s = math.sin(origin_yaw)
        self.world_poses[robot] = (
            origin_x + c * local_x - s * local_y,
            origin_y + s * local_x + c * local_y,
            origin_yaw
            + math.atan2(
                2.0
                * (
                    msg.pose.pose.orientation.w * msg.pose.pose.orientation.z
                    + msg.pose.pose.orientation.x * msg.pose.pose.orientation.y
                ),
                1.0
                - 2.0
                * (msg.pose.pose.orientation.y**2 + msg.pose.pose.orientation.z**2),
            ),
        )
        self.pose_source[robot] = "odom_plus_spawn_pose"

    def publish_ranges(self):
        now = self.get_clock().now()
        for robot in self.robots:
            pose = self.world_poses.get(robot)
            if pose is None:
                now_sec = now.nanoseconds / 1e9
                if now_sec - self.last_pose_warning[robot] >= 2.0:
                    self.get_logger().warn(
                        f"[{robot}] model pose not found on {self.model_states_topic}"
                    )
                    self.last_pose_warning[robot] = now_sec
                continue

            robot_x, robot_y, robot_yaw = pose
            for tag, sign in (("front", 1.0), ("back", -1.0)):
                tag_x = robot_x + sign * self.tag_offset * math.cos(robot_yaw)
                tag_y = robot_y + sign * self.tag_offset * math.sin(robot_yaw)
                distance = math.hypot(tag_x - self.anchor_x, tag_y - self.anchor_y)
                if self.noise_stddev > 0.0:
                    distance += random.gauss(0.0, self.noise_stddev)
                distance = min(self.max_range, max(self.min_range, distance))

                msg = Range()
                msg.header.stamp = now.to_msg()
                msg.header.frame_id = f"{robot}/uwb_{tag}_tag"
                msg.radiation_type = Range.ULTRASOUND
                msg.field_of_view = 2.0 * math.pi
                msg.min_range = self.min_range
                msg.max_range = self.max_range
                msg.range = float(distance)
                self.range_publishers[robot][tag].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MockUwbRangeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
