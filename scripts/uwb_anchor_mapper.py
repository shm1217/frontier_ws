#!/usr/bin/env python3
"""
간단한 UWB range -> map-frame 정렬 브로드캐스터

작동 원리 (거리만 제공되는 UWB의 간단한 처리):
- 각 로봇 네임스페이스별로 `/NAMESPACE/uwb/range` (또는 Float32) 토픽을 구독
- tf에서 `NAMESPACE/map` -> `NAMESPACE/base_link` 변환을 조회하여 로컬 맵에서의 베이스 위치를 구함
- UWB 거리 r과 베이스의 yaw를 이용해 글로벌(앵커) 좌표계에서 베이스의 예상 위치 p = [r*cos(yaw), r*sin(yaw)] 계산
- `anchor` 프레임을 기준으로 `NAMESPACE/map` 프레임의 변환을 계산해 TF로 브로드캐스트

주의: 단일 앵커 + 거리만 상황에서는 방위 불명확성을 가지므로, 로봇의 로컬 맵/오도메트리를 기준으로 방향을 추정하는 휴리스틱을 사용합니다.

사용법 예:
python3 scripts/uwb_anchor_mapper.py --namespaces tb3_0 tb3_1 --anchor-frame anchor --rate 5
"""

import argparse
import math
from collections import defaultdict

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Range
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster, Buffer, TransformListener


def quaternion_from_yaw(yaw: float):
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


def yaw_from_quat(q):
    # q: geometry_msgs.msg.Quaternion like object
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class UwbAnchorMapper(Node):
    def __init__(
        self, namespaces, anchor_frame="anchor", rate=5.0, range_topic="uwb/range"
    ):
        super().__init__("uwb_anchor_mapper")
        self.namespaces = namespaces
        self.anchor_frame = anchor_frame
        self.rate = rate
        self.range_topic = range_topic

        self.latest_range = defaultdict(lambda: None)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = TransformBroadcaster(self)

        # subscribers
        for ns in namespaces:
            # support multiple publish formats used in project:
            # - /{ns}/uwb/position (geometry_msgs/PointStamped)
            # - /{ns}/uwb/ranges (std_msgs/Float32MultiArray)
            # - /{ns}/uwb/range (sensor_msgs/Range) or Float32
            pos_topic = f"/{ns}/uwb/position"
            ranges_topic = f"/{ns}/uwb/ranges"
            topic = f"/{ns}/{range_topic}"

            self.create_subscription(
                PointStamped, pos_topic, self._make_position_cb(ns), 10
            )
            self.create_subscription(
                Float32MultiArray, ranges_topic, self._make_ranges_cb(ns), 10
            )
            self.create_subscription(Range, topic, self._make_range_cb(ns), 10)
            # also support Float32 if node publishes raw float
            self.create_subscription(Float32, topic, self._make_range_cb_float(ns), 10)

        self.timer = self.create_timer(1.0 / float(self.rate), self.timer_cb)

    def _make_range_cb(self, ns):
        def cb(msg: Range):
            self.latest_range[ns] = float(msg.range)

        return cb

    def _make_ranges_cb(self, ns):
        def cb(msg: Float32MultiArray):
            # store list of floats
            try:
                self.latest_ranges[ns] = [float(x) for x in msg.data]
            except Exception:
                self.latest_ranges[ns] = None

        return cb

    def _make_position_cb(self, ns):
        def cb(msg: PointStamped):
            if not hasattr(self, "latest_position"):
                self.latest_position = defaultdict(lambda: None)
            # assume msg.point is in anchor/world frame
            self.latest_position[ns] = msg.point

        return cb

    def _make_range_cb_float(self, ns):
        def cb(msg: Float32):
            self.latest_range[ns] = float(msg.data)

        return cb

    def timer_cb(self):
        now = self.get_clock().now()
        for ns in self.namespaces:
            # priority: explicit PointStamped position > ranges array > single range
            pos = (
                getattr(self, "latest_position", {}).get(ns)
                if hasattr(self, "latest_position")
                else None
            )
            ranges = (
                getattr(self, "latest_ranges", {}).get(ns)
                if hasattr(self, "latest_ranges")
                else None
            )
            r = self.latest_range.get(ns)
            if r is None:
                if pos is None and (ranges is None):
                    continue
            # lookup map -> base_link in this namespace
            target_map = f"/{ns}/map"
            target_base = f"/{ns}/base_link"
            try:
                tf = self.tf_buffer.lookup_transform(
                    target_map, target_base, rclpy.time.Time()
                )
            except Exception as e:
                self.get_logger().debug(f"TF lookup failed for {ns}: {e}")
                continue

            px = tf.transform.translation.x
            py = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = yaw_from_quat(q)

            # determine predicted robot base position in anchor/world frame
            if pos is not None:
                # use PointStamped (assumed reported in anchor/world frame)
                gx = float(pos.x)
                gy = float(pos.y)
            elif ranges is not None and len(ranges) > 0:
                # use first range value as distance to anchor
                try:
                    rr = float(ranges[0])
                except Exception:
                    rr = None
                if rr is None:
                    continue
                gx = rr * math.cos(yaw)
                gy = rr * math.sin(yaw)
            else:
                # fallback to prior single-range value
                gx = r * math.cos(yaw)
                gy = r * math.sin(yaw)

            # rotation matrix Rg from yaw
            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)

            # Rg * p_mb
            rpx = cos_y * px - sin_y * py
            rpy = sin_y * px + cos_y * py

            # map origin in anchor frame
            map_tx = gx - rpx
            map_ty = gy - rpy

            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.anchor_frame
            t.child_frame_id = f"/{ns}/map"
            t.transform.translation.x = float(map_tx)
            t.transform.translation.y = float(map_ty)
            t.transform.translation.z = 0.0
            qy = quaternion_from_yaw(yaw)
            t.transform.rotation.x = qy[0]
            t.transform.rotation.y = qy[1]
            t.transform.rotation.z = qy[2]
            t.transform.rotation.w = qy[3]

            self.broadcaster.sendTransform(t)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--namespaces", nargs="+", required=True)
    parser.add_argument("--anchor-frame", default="world")
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--range-topic", default="uwb/range")
    args = parser.parse_args()

    rclpy.init()
    node = UwbAnchorMapper(
        args.namespaces,
        anchor_frame=args.anchor_frame,
        rate=args.rate,
        range_topic=args.range_topic,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
