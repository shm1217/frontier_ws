#!/usr/bin/env python3
"""Feature/UWB assisted multi-robot occupancy-grid registration and merging.

The reference robot map becomes ``world``.  ORB map features produce SE(2)
candidates.  Per-robot range-only observations estimate the common anchor in
each local map.  Candidate selection combines occupancy agreement and anchor
alignment.  TF and /merge_map are published only after validation succeeds.
"""

import math
from collections import defaultdict, deque

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener


def yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class MergeMapUwb(Node):
    def __init__(self):
        super().__init__("merge_map_uwb")
        self.robots = list(self.declare_parameter(
            "robot_namespaces", ["tb3_0", "tb3_1"]).value)
        self.reference_robot = str(self.declare_parameter(
            "reference_robot", self.robots[0]).value)
        self.global_frame = str(self.declare_parameter("global_frame", "world").value)
        self.base_suffix = str(self.declare_parameter(
            "base_frame_suffix", "base_footprint").value)
        self.range_topic_suffix = str(self.declare_parameter(
            "range_topic_suffix", "uwb/range").value)
        self.min_samples = int(self.declare_parameter("min_range_samples", 20).value)
        self.max_samples = int(self.declare_parameter("max_range_samples", 1000).value)
        self.min_motion = float(self.declare_parameter("min_sample_motion_m", 0.08).value)
        self.max_anchor_rmse = float(self.declare_parameter(
            "max_anchor_rmse_m", 0.35).value)
        self.max_anchor_match = float(self.declare_parameter(
            "max_anchor_match_error_m", 0.60).value)
        self.min_feature_matches = int(self.declare_parameter(
            "min_feature_matches", 5).value)
        self.min_overlap_score = float(self.declare_parameter(
            "min_overlap_score", 0.25).value)
        self.feature_weight = float(self.declare_parameter(
            "feature_weight", 1.0).value)
        self.anchor_weight = float(self.declare_parameter(
            "anchor_weight", 2.0).value)
        self.output_resolution = float(self.declare_parameter(
            "output_resolution", 0.05).value)
        self.map_padding = float(self.declare_parameter("map_padding_m", 1.0).value)

        self.maps = {}
        self.samples = {ns: deque(maxlen=self.max_samples) for ns in self.robots}
        self.anchors = {}
        self.transforms = {}
        self.locked = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static = StaticTransformBroadcaster(self)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.map_subs = []
        self.range_subs = []
        for ns in self.robots:
            self.map_subs.append(self.create_subscription(
                OccupancyGrid, f"/{ns}/map",
                lambda msg, robot=ns: self.on_map(msg, robot), qos))
            self.range_subs.append(self.create_subscription(
                Range, f"/{ns}/{self.range_topic_suffix}",
                lambda msg, robot=ns: self.on_range(msg, robot), 30))

        self.map_pub = self.create_publisher(OccupancyGrid, "/merge_map", qos)
        self.valid_pub = self.create_publisher(Bool, "/merge_map_uwb_valid", qos)
        self.timer = self.create_timer(1.0, self.tick)
        self.publish_valid(False)

    def publish_valid(self, value):
        msg = Bool()
        msg.data = bool(value)
        self.valid_pub.publish(msg)

    def on_map(self, msg, robot):
        self.maps[robot] = msg

    def on_range(self, msg, robot):
        if not math.isfinite(msg.range) or msg.range <= 0.0:
            return
        map_frame = f"{robot}/map"
        base_frame = f"{robot}/{self.base_suffix}"
        try:
            tf = self.tf_buffer.lookup_transform(
                map_frame, base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except Exception:
            return
        sample = (tf.transform.translation.x, tf.transform.translation.y, float(msg.range))
        q = self.samples[robot]
        if q and math.hypot(sample[0] - q[-1][0], sample[1] - q[-1][1]) < self.min_motion:
            return
        q.append(sample)

    @staticmethod
    def estimate_anchor(samples):
        data = np.asarray(samples, dtype=np.float64)
        if len(data) < 3:
            return None
        p0, r0 = data[0, :2], data[0, 2]
        points, ranges = data[1:, :2], data[1:, 2]
        A = 2.0 * (points - p0)
        b = (points * points).sum(axis=1) - p0.dot(p0) - ranges**2 + r0**2
        if np.linalg.matrix_rank(A) < 2:
            return None
        anchor = np.linalg.lstsq(A, b, rcond=None)[0]

        # Robust Gauss-Newton refinement.
        all_points, all_ranges = data[:, :2], data[:, 2]
        for _ in range(15):
            delta = anchor - all_points
            predicted = np.linalg.norm(delta, axis=1)
            good = predicted > 1e-6
            residual = predicted[good] - all_ranges[good]
            J = delta[good] / predicted[good, None]
            scale = max(0.05, 1.4826 * np.median(np.abs(residual - np.median(residual))))
            weights = np.minimum(1.0, (1.5 * scale) / np.maximum(np.abs(residual), 1e-9))
            H = J.T @ (weights[:, None] * J)
            g = J.T @ (weights * residual)
            if abs(np.linalg.det(H)) < 1e-10:
                return None
            step = np.linalg.solve(H, g)
            anchor -= step
            if np.linalg.norm(step) < 1e-5:
                break
        residual = np.linalg.norm(all_points - anchor, axis=1) - all_ranges
        rmse = float(np.sqrt(np.mean(np.minimum(residual**2, 1.0))))
        return anchor, rmse

    @staticmethod
    def map_image(msg):
        data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
        image = np.full(data.shape, 127, dtype=np.uint8)
        image[data <= 40] = 255
        image[data >= 60] = 0
        return image

    @staticmethod
    def pixel_to_local(msg, xy):
        xy = np.asarray(xy, dtype=np.float64)
        out = np.empty_like(xy)
        out[..., 0] = msg.info.origin.position.x + (xy[..., 0] + 0.5) * msg.info.resolution
        out[..., 1] = msg.info.origin.position.y + (xy[..., 1] + 0.5) * msg.info.resolution
        return out

    def feature_candidates(self, ref, mov):
        orb = cv2.ORB_create(nfeatures=1800, fastThreshold=8)
        kp1, des1 = orb.detectAndCompute(self.map_image(ref), None)
        kp2, des2 = orb.detectAndCompute(self.map_image(mov), None)
        if des1 is None or des2 is None:
            return []
        pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(des2, des1, k=2)
        # knnMatch(k=2)도 비교 대상 descriptor가 부족하면 한 개만
        # 반환할 수 있다. 두 이웃이 있는 항목에만 ratio test를 적용한다.
        matches = []
        for pair in pairs:
            if len(pair) < 2:
                continue
            first, second = pair[0], pair[1]
            if first.distance < 0.78 * second.distance:
                matches.append(first)
        if len(matches) < self.min_feature_matches:
            return []
        src_px = np.float32([kp2[m.queryIdx].pt for m in matches])
        dst_px = np.float32([kp1[m.trainIdx].pt for m in matches])
        src = self.pixel_to_local(mov, src_px)
        dst = self.pixel_to_local(ref, dst_px)

        candidates = []
        rng = np.random.default_rng(7)
        batches = [np.arange(len(matches))]
        for _ in range(30):
            size = min(len(matches), max(6, len(matches) // 2))
            batches.append(rng.choice(len(matches), size=size, replace=False))
        for idx in batches:
            M, inliers = cv2.estimateAffinePartial2D(
                src[idx], dst[idx], method=cv2.RANSAC,
                ransacReprojThreshold=0.20, maxIters=2000, confidence=0.995)
            if M is None:
                continue
            scale = math.hypot(M[0, 0], M[1, 0])
            if not 0.95 <= scale <= 1.05:
                continue
            yaw = math.atan2(M[1, 0], M[0, 0])
            candidate = (float(M[0, 2]), float(M[1, 2]), yaw)
            if all(abs(math.atan2(math.sin(yaw-c[2]), math.cos(yaw-c[2]))) > math.radians(2)
                   or math.hypot(candidate[0]-c[0], candidate[1]-c[1]) > 0.20
                   for c in candidates):
                candidates.append(candidate)
        return candidates

    @staticmethod
    def transform_point(transform, point):
        tx, ty, yaw = transform
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array([c * point[0] - s * point[1] + tx,
                         s * point[0] + c * point[1] + ty])

    def overlap_score(self, ref, mov, transform):
        mov_data = np.asarray(mov.data, dtype=np.int16).reshape(mov.info.height, mov.info.width)
        ys, xs = np.where(mov_data >= 60)
        if len(xs) == 0:
            return 0.0
        stride = max(1, len(xs) // 5000)
        xy = np.column_stack((xs[::stride], ys[::stride]))
        local = self.pixel_to_local(mov, xy)
        tx, ty, yaw = transform
        c, s = math.cos(yaw), math.sin(yaw)
        rx = c * local[:, 0] - s * local[:, 1] + tx
        ry = s * local[:, 0] + c * local[:, 1] + ty
        gx = np.floor((rx - ref.info.origin.position.x) / ref.info.resolution).astype(int)
        gy = np.floor((ry - ref.info.origin.position.y) / ref.info.resolution).astype(int)
        inside = (gx >= 0) & (gy >= 0) & (gx < ref.info.width) & (gy < ref.info.height)
        if inside.sum() < 20:
            return 0.0
        ref_data = np.asarray(ref.data, dtype=np.int16).reshape(ref.info.height, ref.info.width)
        values = ref_data[gy[inside], gx[inside]]
        known = values >= 0
        if known.sum() < 20:
            return 0.0
        occupied_agree = np.count_nonzero(values[known] >= 60)
        free_conflict = np.count_nonzero(values[known] <= 40)
        return float((occupied_agree - free_conflict) / known.sum())

    def select_transform(self, ref_ns, mov_ns):
        ref, mov = self.maps[ref_ns], self.maps[mov_ns]
        candidates = self.feature_candidates(ref, mov)
        if not candidates:
            return None
        a_ref, a_mov = self.anchors[ref_ns][0], self.anchors[mov_ns][0]
        best = None
        for transform in candidates:
            overlap = self.overlap_score(ref, mov, transform)
            anchor_error = float(np.linalg.norm(
                self.transform_point(transform, a_mov) - a_ref))
            total = self.feature_weight * overlap - self.anchor_weight * anchor_error
            item = (total, overlap, anchor_error, transform)
            # item = (overlap, transform)
            if best is None or item[0] > best[0]:
                best = item
        if best[1] < self.min_overlap_score or best[2] > self.max_anchor_match:
            self.get_logger().warn(
                f"registration rejected: overlap={best[1]:.3f}, anchor_error={best[2]:.3f}m")
            return None
        self.get_logger().info(
            f"registration accepted: overlap={best[1]:.3f}, anchor_error={best[2]:.3f}m")
        return best[3]
        # if best[0] < self.min_overlap_score:
        #     self.get_logger().warn(
        #         f"registration rejected: overlap={best[0]:.3f}")
        #     return None
        # self.get_logger().info(
        #     f"registration accepted: overlap={best[0]:.3f}")
        # return best[1]

    def broadcast_transforms(self):
        messages = []
        for ns, (tx, ty, yaw) in self.transforms.items():
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.global_frame
            msg.child_frame_id = f"{ns}/map"
            msg.transform.translation.x = tx
            msg.transform.translation.y = ty
            q = yaw_quaternion(yaw)
            msg.transform.rotation.x, msg.transform.rotation.y = q[0], q[1]
            msg.transform.rotation.z, msg.transform.rotation.w = q[2], q[3]
            messages.append(msg)
        self.tf_static.sendTransform(messages)

    def merge_and_publish(self):
        res = self.output_resolution
        bounds = []
        for ns, msg in self.maps.items():
            t = self.transforms[ns]
            corners = np.array([
                [msg.info.origin.position.x, msg.info.origin.position.y],
                [msg.info.origin.position.x + msg.info.width * msg.info.resolution,
                 msg.info.origin.position.y],
                [msg.info.origin.position.x,
                 msg.info.origin.position.y + msg.info.height * msg.info.resolution],
                [msg.info.origin.position.x + msg.info.width * msg.info.resolution,
                 msg.info.origin.position.y + msg.info.height * msg.info.resolution]])
            bounds.append(np.array([self.transform_point(t, p) for p in corners]))
        all_bounds = np.vstack(bounds)
        min_xy = all_bounds.min(axis=0) - self.map_padding
        max_xy = all_bounds.max(axis=0) + self.map_padding
        width, height = np.ceil((max_xy - min_xy) / res).astype(int)
        sums = np.zeros((height, width), dtype=np.float32)
        counts = np.zeros((height, width), dtype=np.int16)
        for ns, msg in self.maps.items():
            data = np.asarray(msg.data, dtype=np.int16).reshape(msg.info.height, msg.info.width)
            ys, xs = np.where(data >= 0)
            xy = self.pixel_to_local(msg, np.column_stack((xs, ys)))
            t = self.transforms[ns]
            c, s = math.cos(t[2]), math.sin(t[2])
            wx = c * xy[:, 0] - s * xy[:, 1] + t[0]
            wy = s * xy[:, 0] + c * xy[:, 1] + t[1]
            gx = ((wx - min_xy[0]) / res).astype(int)
            gy = ((wy - min_xy[1]) / res).astype(int)
            valid = (gx >= 0) & (gy >= 0) & (gx < width) & (gy < height)
            np.add.at(sums, (gy[valid], gx[valid]), data[ys[valid], xs[valid]])
            np.add.at(counts, (gy[valid], gx[valid]), 1)
        out = np.full((height, width), -1, dtype=np.int8)
        seen = counts > 0
        out[seen] = np.clip(np.rint(sums[seen] / counts[seen]), 0, 100).astype(np.int8)
        merged = OccupancyGrid()
        merged.header.stamp = self.get_clock().now().to_msg()
        merged.header.frame_id = self.global_frame
        merged.info.resolution = res
        merged.info.width, merged.info.height = int(width), int(height)
        merged.info.origin.position.x, merged.info.origin.position.y = map(float, min_xy)
        merged.info.origin.orientation.w = 1.0
        merged.data = out.ravel().tolist()
        self.map_pub.publish(merged)

    def tick(self):
        if any(ns not in self.maps for ns in self.robots):
            return
        for ns in self.robots:
            estimate = self.estimate_anchor(self.samples[ns])
            if estimate is not None and len(self.samples[ns]) >= self.min_samples:
                self.anchors[ns] = estimate
        if any(ns not in self.anchors or self.anchors[ns][1] > self.max_anchor_rmse
               for ns in self.robots):
            return

        if not self.locked:
            self.transforms = {self.reference_robot: (0.0, 0.0, 0.0)}
            for ns in self.robots:
                if ns == self.reference_robot:
                    continue
                transform = self.select_transform(self.reference_robot, ns)
                if transform is None:
                    self.transforms = {}
                    self.publish_valid(False)
                    return
                self.transforms[ns] = transform
            self.locked = True
            self.broadcast_transforms()
            self.publish_valid(True)
        self.merge_and_publish()


def main(args=None):
    rclpy.init(args=args)
    node = MergeMapUwb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
