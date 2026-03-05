#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped

from tf2_ros import Buffer, TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

from sensor_msgs_py import point_cloud2
from rclpy.time import Time
from rclpy.duration import Duration


class CloudMergeToOctomap(Node):
    """
    Subscribes multiple PointCloud2 topics, transforms each to global_frame,
    merges points, publishes a single PointCloud2 for octomap_server (/cloud_in).
    """

    def __init__(self):
        super().__init__('cloud_merge_to_octomap')

        self.global_frame = self.declare_parameter('global_frame', 'tb3_0/map').get_parameter_value().string_value
        self.input_topics = self.declare_parameter(
            'input_topics',
            ['/tb3_0/scan_matched_points2', '/tb3_1/scan_matched_points2']
        ).get_parameter_value().string_array_value
        self.output_topic = self.declare_parameter('output_topic', '/cloud_in').get_parameter_value().string_value

        self.publish_rate_hz = float(self.declare_parameter('publish_rate_hz', 5.0).value)
        self.max_points = int(self.declare_parameter('max_points', 60000).value)  # 너무 커지면 RViz/Octomap이 버벅임
        self.tf_timeout_sec = float(self.declare_parameter('tf_timeout_sec', 0.15).value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_cloud = {}  # topic -> PointCloud2

        self.subs = []
        for t in self.input_topics:
            self.subs.append(
                self.create_subscription(PointCloud2, t, lambda msg, tt=t: self._cb(msg, tt), qos_profile_sensor_data)
            )

        
        self.pub = self.create_publisher(PointCloud2, self.output_topic, qos_profile_sensor_data)

        period = 1.0 / max(self.publish_rate_hz, 0.1)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"cloud_merge_to_octomap: global_frame={self.global_frame}, "
            f"inputs={list(self.input_topics)}, output={self.output_topic}, rate={self.publish_rate_hz}Hz"
        )

    def _cb(self, msg: PointCloud2, topic: str):
        self.last_cloud[topic] = msg

    def _transform_to_global(self, cloud: PointCloud2) -> PointCloud2 | None:
        try:
            t = Time()
            tf = self.tf_buffer.lookup_transform(
                self.global_frame,
                cloud.header.frame_id,
                t,
                timeout=Duration(seconds=self.tf_timeout_sec)
            )
            return do_transform_cloud(cloud, tf)
        except Exception as e:
            self.get_logger().warn(
                f"TF transform failed {cloud.header.frame_id} -> {self.global_frame}: {e}"
            )
            return None

    def _on_timer(self):
        # 아직 어떤 cloud도 없으면 패스
        if not self.last_cloud:
            return

        merged_points = []
        stamp = self.get_clock().now().to_msg()

        for t in self.input_topics:
            cloud = self.last_cloud.get(t)
            if cloud is None:
                continue

            tc = self._transform_to_global(cloud)
            if tc is None:
                continue

            # xyz만 뽑아서 병합 (필요하면 rgb/intensity도 확장 가능)
            for p in point_cloud2.read_points(tc, field_names=('x', 'y', 'z'), skip_nans=True):
                merged_points.append((float(p[0]), float(p[1]), float(p[2])))
                if len(merged_points) >= self.max_points:
                    break
            if len(merged_points) >= self.max_points:
                break

        if not merged_points:
            return

        out = point_cloud2.create_cloud_xyz32(
            header=PointCloud2().header,  # 임시
            points=merged_points
        )
        out.header.stamp = stamp
        out.header.frame_id = self.global_frame

        self.pub.publish(out)


def main():
    rclpy.init()
    node = CloudMergeToOctomap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
