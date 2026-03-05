#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile

def yaw_from_quat(q):
    # q = geometry_msgs.msg.Quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

class MapMergeFixed(Node):
    def __init__(self):
        super().__init__('map_merge_fixed')

        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_maps', ['/tb3_0/map', '/tb3_1/map'])
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('size_x', 20.0)   # meters
        self.declare_parameter('size_y', 20.0)   # meters
        self.declare_parameter('origin_x', -10.0)
        self.declare_parameter('origin_y', -10.0)
        self.declare_parameter('publish_topic', '/map')
        self.declare_parameter('publish_rate_hz', 2.0)

        self.global_frame = self.get_parameter('global_frame').value
        self.robot_maps = self.get_parameter('robot_maps').value
        self.res = float(self.get_parameter('resolution').value)
        self.size_x = float(self.get_parameter('size_x').value)
        self.size_y = float(self.get_parameter('size_y').value)
        self.origin_x = float(self.get_parameter('origin_x').value)
        self.origin_y = float(self.get_parameter('origin_y').value)
        self.pub_topic = self.get_parameter('publish_topic').value
        rate = float(self.get_parameter('publish_rate_hz').value)

        self.W = int(round(self.size_x / self.res))
        self.H = int(round(self.size_y / self.res))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest = {t: None for t in self.robot_maps}
        self.subs = []
        for t in self.robot_maps:
            self.subs.append(
                self.create_subscription(
                    OccupancyGrid, t, lambda msg, topic=t: self._cb(msg, topic), 10
                )
            )

        qos_map = QoSProfile(depth=1)   # = VOLATILE + RELIABLE
        self.pub = self.create_publisher(OccupancyGrid, self.pub_topic, qos_map)

        period = 1.0 / max(rate, 0.1)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"map_merge_fixed: global_frame={self.global_frame}, topics={self.robot_maps}, "
            f"global {self.W}x{self.H} res={self.res} origin=({self.origin_x},{self.origin_y})"
        )

    def _cb(self, msg: OccupancyGrid, topic: str):
        self.latest[topic] = msg

    def _world_to_global_idx(self, x, y):
        gx = int(math.floor((x - self.origin_x) / self.res))
        gy = int(math.floor((y - self.origin_y) / self.res))
        if gx < 0 or gy < 0 or gx >= self.W or gy >= self.H:
            return None
        return gy * self.W + gx

    def _on_timer(self):
        # merged init: unknown
        merged = [-1] * (self.W * self.H)

        any_map = False

        for topic, m in self.latest.items():
            if m is None:
                continue

            any_map = True
            src_frame = m.header.frame_id  # expected: tb3_i/map

            # tf: global_frame <- src_frame
            try:
                tf = self.tf_buffer.lookup_transform(self.global_frame, src_frame, rclpy.time.Time())
            except Exception as e:
                self.get_logger().warn(f"TF missing: {self.global_frame} <- {src_frame} : {e}")
                continue

            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            yaw = yaw_from_quat(tf.transform.rotation)
            cy = math.cos(yaw)
            sy = math.sin(yaw)

            info = m.info
            mw = int(info.width)
            mh = int(info.height)
            res = float(info.resolution)

            # iterate all known cells (v != -1) and place into global
            # NOTE: cartographer maps can be a bit large; this is the "simple reliable" version.
            for j in range(mh):
                for i in range(mw):
                    v = m.data[j * mw + i]
                    if v < 0:
                        continue  # unknown skip

                    # cell center in src_frame (map frame of that robot)
                    lx = info.origin.position.x + (i + 0.5) * res
                    ly = info.origin.position.y + (j + 0.5) * res

                    # transform to global_frame: [x;y] = R*[lx;ly] + t
                    gx = cy * lx - sy * ly + tx
                    gy = sy * lx + cy * ly + ty

                    idx = self._world_to_global_idx(gx, gy)
                    if idx is None:
                        continue

                    # merge rule: max works well for {unknown=-1, free~0, occ~100}
                    if merged[idx] < v:
                        merged[idx] = v

        if not any_map:
            return

        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.global_frame
        out.info.resolution = self.res
        out.info.width = self.W
        out.info.height = self.H
        out.info.origin.position.x = self.origin_x
        out.info.origin.position.y = self.origin_y
        out.info.origin.orientation.w = 1.0
        out.data = merged

        self.pub.publish(out)

def main():
    rclpy.init()
    node = MapMergeFixed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
