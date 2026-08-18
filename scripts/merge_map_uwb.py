#!/usr/bin/env python3
"""Feature/UWB assisted multi-robot occupancy-grid registration and merging.

All map pairs are evaluated and the strongest valid overlaps form a transform
tree.  The tree is rooted at the reference robot, whose map becomes ``world``;
the original grids are then merged once to avoid repeated resampling.  ORB map
features produce SE(2) yaw candidates.  Front/back tags mounted at +/-
``tag_offset_from_base_m`` observe one common anchor while the robot moves;
both range histories jointly estimate that anchor in each local map.  Candidate
selection combines feature support, occupancy agreement, and the two-tag
anchor constraint.  A single anchor does not independently make absolute yaw
observable, so UWB validates and stabilizes feature yaw rather than replacing
it.
"""

import math
from collections import defaultdict, deque

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
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
        self.robots = list(
            self.declare_parameter("robot_namespaces", ["tb3_1", "tb3_2"]).value
        )
        self.reference_robot = str(
            self.declare_parameter("reference_robot", self.robots[0]).value
        )
        self.global_frame = str(self.declare_parameter("global_frame", "world").value)
        self.base_suffix = str(
            self.declare_parameter("base_frame_suffix", "base_footprint").value
        )
        self.front_range_topic_suffix = str(
            self.declare_parameter("front_range_topic_suffix", "uwb/front/range").value
        )
        self.back_range_topic_suffix = str(
            self.declare_parameter("back_range_topic_suffix", "uwb/back/range").value
        )
        self.tag_offset = float(
            self.declare_parameter("tag_offset_from_base_m", 0.15).value
        )
        self.min_samples = int(self.declare_parameter("min_range_samples", 20).value)
        self.max_samples = int(self.declare_parameter("max_range_samples", 1000).value)
        self.min_motion = float(
            self.declare_parameter("min_sample_motion_m", 0.1).value
        )
        self.max_anchor_rmse = float(
            self.declare_parameter("max_anchor_rmse_m", 0.5).value ## 0.35
        )  
        self.max_anchor_match = float(
            self.declare_parameter("max_anchor_match_error_m", 0.6).value
        )
        self.min_feature_matches = int(
            self.declare_parameter("min_feature_matches", 8).value
        )  
        self.max_features = int(self.declare_parameter("max_features", 2500).value)
        self.orb_fast_threshold = int(
            self.declare_parameter("orb_fast_threshold", 8).value
        )
        self.orb_edge_threshold = int(
            self.declare_parameter("orb_edge_threshold", 8).value
        )
        self.orb_patch_size = int(
            self.declare_parameter("orb_patch_size", 31).value ## 10
        ) 
        self.feature_ratio = float(
            self.declare_parameter("feature_ratio", 0.78).value
        ) 
        self.ransac_batches = int(
            self.declare_parameter("ransac_batches", 100).value ## 60
        ) 
        self.ransac_threshold = float(
            self.declare_parameter("ransac_threshold_m", 0.20).value
        ) 
        self.min_ransac_inliers = int(
            self.declare_parameter("min_ransac_inliers", 4).value
        ) 
        self.min_ransac_inlier_ratio = float(
            self.declare_parameter("min_ransac_inlier_ratio", 0.30).value
        )  
        self.min_scale = float(self.declare_parameter("min_scale", 0.95).value)
        self.max_scale = float(self.declare_parameter("max_scale", 1.05).value)
        self.yaw_cluster_deg = float(
            self.declare_parameter("yaw_cluster_deg", 5.0).value
        )
        self.max_yaw_hypotheses = int(
            self.declare_parameter("max_yaw_hypotheses", 12).value
        )
        self.max_pairwise_matches = int(
            self.declare_parameter("max_pairwise_matches", 80).value
        )
        self.min_pair_baseline = float(
            self.declare_parameter("min_pair_baseline_m", 0.50).value
        )
        self.min_overlap_score = float(
            self.declare_parameter("min_overlap_score", 0.5).value
        )
        self.min_overlap_coverage = float(
            self.declare_parameter("min_overlap_coverage", 0.1).value ## 0.1
        ) 
        self.wall_tolerance = float(
            self.declare_parameter("wall_tolerance_m", 0.10).value
        )
        self.refine_yaw = float(self.declare_parameter("refine_yaw_deg", 3.0).value)
        self.refine_yaw_step = float(
            self.declare_parameter("refine_yaw_step_deg", 1.0).value
        )
        self.refine_fine_yaw_step = float(
            self.declare_parameter("refine_fine_yaw_step_deg", 0.25).value
        )
        self.feature_refine_xy = float(
            self.declare_parameter("feature_refine_xy_m", 0.60).value
        )
        self.global_min_known_cells = int(
            self.declare_parameter("global_min_known_cells", 300).value
        )
        self.global_min_coverage = float(
            self.declare_parameter("global_min_coverage", 0.30).value
        )
        self.global_min_overlap = float(
            self.declare_parameter("global_min_overlap", 0.60).value
        )
        self.feature_weight = float(self.declare_parameter("feature_weight", 1.0).value)
        self.overlap_weight = float(self.declare_parameter("overlap_weight", 1.0).value)
        self.coverage_weight = float(
            self.declare_parameter("coverage_weight", 0.5).value
        )
        self.anchor_weight = float(self.declare_parameter("anchor_weight", 2.0).value)
        self.global_yaw_step = float(
            self.declare_parameter("global_yaw_step_deg", 5.0).value
        )
        self.global_yaw_top_k = int(self.declare_parameter("global_yaw_top_k", 5).value)
        self.global_yaw_nms = math.radians(
            float(self.declare_parameter("global_yaw_nms_deg", 10.0).value)
        )
        self.output_resolution = float(
            self.declare_parameter("output_resolution", 0.05).value
        )
        self.map_padding = float(self.declare_parameter("map_padding_m", 1.0).value)
        self.rendezvous_enabled = bool(
            self.declare_parameter("rendezvous_enabled", True).value
        )
        self.rendezvous_trigger_timeout = float(
            self.declare_parameter("rendezvous_trigger_timeout_s", 60.0).value
        )
        self.rendezvous_trigger_distance = float(
            self.declare_parameter("rendezvous_trigger_distance_m", 6.0).value
        )
        self.rendezvous_arrival_radius = float(
            self.declare_parameter("rendezvous_arrival_radius_m", 2.0).value
        )
        self.rendezvous_manual_topic = str(
            self.declare_parameter("rendezvous_manual_topic", "/rendezvous_now").value
        )

        self.maps = {}
        self.samples = {ns: deque(maxlen=self.max_samples) for ns in self.robots}
        self.last_sample_pose = {
            ns: {"front": None, "back": None} for ns in self.robots
        }
        self.latest_base_pose = {ns: None for ns in self.robots}
        self.rendezvous_active = {ns: False for ns in self.robots}
        self.rendezvous_forced = {ns: False for ns in self.robots}
        self.rendezvous_cycle_start = {
            ns: self.get_clock().now() for ns in self.robots
        }
        self.anchors = {}
        self.transforms = {}
        # Successful pair registrations survive later timer cycles.  This lets
        # robots form the global map through overlaps observed at different
        # times (for example, 0<->1 first and 1<->2 later).
        self.edge_cache = {}
        self.locked = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_static = StaticTransformBroadcaster(self)
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        map_input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_subs = []
        self.range_subs = []
        for ns in self.robots:
            self.map_subs.append(
                self.create_subscription(
                    OccupancyGrid,
                    f"/{ns}/map",
                    lambda msg, robot=ns: self.on_map(msg, robot),
                    map_input_qos,
                )
            )
            self.range_subs.append(
                self.create_subscription(
                    Range,
                    f"/{ns}/{self.front_range_topic_suffix}",
                    lambda msg, robot=ns: self.on_range(msg, robot, "front"),
                    30,
                )
            )
            self.range_subs.append(
                self.create_subscription(
                    Range,
                    f"/{ns}/{self.back_range_topic_suffix}",
                    lambda msg, robot=ns: self.on_range(msg, robot, "back"),
                    30,
                )
            )

        self.map_pub = self.create_publisher(
            OccupancyGrid, "/merge_map", output_qos
        )
        self.valid_pub = self.create_publisher(
            Bool, "/merge_map_uwb_valid", output_qos
        )
        self.rendezvous_pubs = {
            ns: self.create_publisher(
                PoseStamped, f"/{ns}/rendezvous_anchor", 10
            )
            for ns in self.robots
        }
        self.rendezvous_manual_sub = self.create_subscription(
            Bool, self.rendezvous_manual_topic, self.on_manual_rendezvous, 10
        )
        self.timer = self.create_timer(1.0, self.tick)
        self.publish_valid(False)

    def on_manual_rendezvous(self, msg):
        now = self.get_clock().now()
        if msg.data and self.locked:
            self.get_logger().info(
                "manual rendezvous ignored: map registration is already locked"
            )
            return

        for ns in self.robots:
            self.rendezvous_forced[ns] = bool(msg.data)
            if not msg.data:
                self.rendezvous_cycle_start[ns] = now

        if msg.data:
            self.get_logger().warn("manual rendezvous requested for all robots")
        else:
            self.get_logger().info("manual rendezvous cancelled for all robots")

    def publish_valid(self, value):
        msg = Bool()
        msg.data = bool(value)
        self.valid_pub.publish(msg)

    def on_map(self, msg, robot):
        self.maps[robot] = msg

    @staticmethod
    def quaternion_yaw(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def on_range(self, msg, robot, tag):
        if not math.isfinite(msg.range) or msg.range <= 0.0:
            return
        map_frame = f"{robot}/map"
        base_frame = f"{robot}/{self.base_suffix}"
        try:
            tf = self.tf_buffer.lookup_transform(
                map_frame, base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
        except Exception:
            return
        base_x = float(tf.transform.translation.x)
        base_y = float(tf.transform.translation.y)
        base_yaw = self.quaternion_yaw(tf.transform.rotation)
        self.latest_base_pose[robot] = (base_x, base_y, base_yaw)
        signed_offset = self.tag_offset if tag == "front" else -self.tag_offset
        tag_x = base_x + signed_offset * math.cos(base_yaw)
        tag_y = base_y + signed_offset * math.sin(base_yaw)
        sample = (tag_x, tag_y, float(msg.range), tag)
        q = self.samples[robot]
        previous = self.last_sample_pose[robot][tag]
        if (
            previous is not None
            and math.hypot(sample[0] - previous[0], sample[1] - previous[1])
            < self.min_motion
        ):
            return
        q.append(sample)
        self.last_sample_pose[robot][tag] = sample

    def publish_rendezvous_commands(self):
        if not self.rendezvous_enabled or self.locked:
            return
        now = self.get_clock().now()
        for ns in self.robots:
            elapsed = (now - self.rendezvous_cycle_start[ns]).nanoseconds * 1e-9
            timed_out = elapsed >= self.rendezvous_trigger_timeout
            should_rendezvous = self.rendezvous_forced[ns] or timed_out
            estimate = self.anchors.get(ns)
            pose = self.latest_base_pose.get(ns)
            if estimate is None or pose is None:
                self.rendezvous_active[ns] = False
                continue
            anchor, rmse = estimate
            if rmse > self.max_anchor_rmse:
                self.rendezvous_active[ns] = False
                continue
            distance = math.hypot(anchor[0] - pose[0], anchor[1] - pose[1])
            if distance <= self.rendezvous_arrival_radius:
                if self.rendezvous_active[ns] or should_rendezvous:
                    self.rendezvous_cycle_start[ns] = now
                    self.get_logger().info(
                        f"[{ns}] rendezvous arrived; restarting "
                        f"{self.rendezvous_trigger_timeout:.1f}s exploration window"
                    )
                self.rendezvous_forced[ns] = False
                self.rendezvous_active[ns] = False
                continue
            if not should_rendezvous:
                self.rendezvous_active[ns] = False
                continue

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"{ns}/map"
            msg.pose.position.x = float(anchor[0])
            msg.pose.position.y = float(anchor[1])
            msg.pose.orientation.w = 1.0
            self.rendezvous_pubs[ns].publish(msg)
            if not self.rendezvous_active[ns]:
                self.get_logger().info(
                    f"[{ns}] rendezvous active: anchor_distance={distance:.2f}m, "
                    f"elapsed={elapsed:.1f}s, manual={self.rendezvous_forced[ns]}"
                )
            self.rendezvous_active[ns] = True

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

        all_points, all_ranges = data[:, :2], data[:, 2]
        for _ in range(15):
            delta = anchor - all_points
            predicted = np.linalg.norm(delta, axis=1)
            good = predicted > 1e-6
            residual = predicted[good] - all_ranges[good]
            J = delta[good] / predicted[good, None]
            scale = max(
                0.05, 1.4826 * np.median(np.abs(residual - np.median(residual)))
            )
            weights = np.minimum(
                1.0, (1.5 * scale) / np.maximum(np.abs(residual), 1e-9)
            )
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
        data = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width
        )
        image = np.full(data.shape, 255, dtype=np.uint8)
        image[data >= 60] = 0
        occupied = (image == 0).astype(np.uint8)
        occupied = cv2.dilate(occupied, np.ones((3, 3), np.uint8), iterations=1)
        return np.where(occupied > 0, 0, 255).astype(np.uint8)

    @staticmethod
    def pixel_to_local(msg, xy):
        xy = np.asarray(xy, dtype=np.float64)
        out = np.empty_like(xy)
        out[..., 0] = (
            msg.info.origin.position.x + (xy[..., 0] + 0.5) * msg.info.resolution
        )
        out[..., 1] = (
            msg.info.origin.position.y + (xy[..., 1] + 0.5) * msg.info.resolution
        )
        return out

    def feature_candidates(self, ref, mov):
        orb = cv2.ORB_create(
            nfeatures=self.max_features,
            fastThreshold=self.orb_fast_threshold,
            edgeThreshold=self.orb_edge_threshold,
            patchSize=self.orb_patch_size,
        )
        kp1, des1 = orb.detectAndCompute(self.map_image(ref), None)
        kp2, des2 = orb.detectAndCompute(self.map_image(mov), None)
        stats = {
            "ref_keypoints": len(kp1),
            "mov_keypoints": len(kp2),
            "ratio_matches": 0,
            "ransac_votes": 0,
            "yaw_hypotheses": 0,
        }
        if des1 is None or des2 is None:
            return [], stats
        pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(des2, des1, k=2)
        matches = []
        for pair in pairs:
            if len(pair) < 2:
                continue
            first, second = pair[0], pair[1]
            if first.distance < self.feature_ratio * second.distance:
                matches.append(first)
        matches.sort(key=lambda match: match.distance)
        stats["ratio_matches"] = len(matches)
        if len(matches) < self.min_feature_matches:
            return [], stats
        src_px = np.float32([kp2[m.queryIdx].pt for m in matches])
        dst_px = np.float32([kp1[m.trainIdx].pt for m in matches])
        src = self.pixel_to_local(mov, src_px)
        dst = self.pixel_to_local(ref, dst_px)

        yaw_votes = []
        affine_candidates = []
        rng = np.random.default_rng(7)
        batches = [np.arange(len(matches))]
        for _ in range(self.ransac_batches):
            size = min(len(matches), max(6, len(matches) // 2))
            batches.append(rng.choice(len(matches), size=size, replace=False))
        for idx in batches:
            M, inliers = cv2.estimateAffinePartial2D(
                src[idx],
                dst[idx],
                method=cv2.RANSAC,
                ransacReprojThreshold=self.ransac_threshold,
                maxIters=3000,
                confidence=0.995,
            )
            if M is None or inliers is None:
                continue
            inlier_count = int(inliers.sum())
            inlier_ratio = inlier_count / max(1, len(idx))
            if inlier_count < self.min_ransac_inliers:
                continue
            if inlier_ratio < self.min_ransac_inlier_ratio:
                continue
            scale = math.hypot(M[0, 0], M[1, 0])
            if not self.min_scale <= scale <= self.max_scale:
                continue
            yaw = math.atan2(M[1, 0], M[0, 0])
            weight = float(3 * inlier_count)
            yaw_votes.append((yaw, weight))
            affine_candidates.append(
                ((float(M[0, 2]), float(M[1, 2]), yaw), weight)
            )
            stats["ransac_votes"] += 1

        pair_count = min(len(matches), self.max_pairwise_matches)
        for i in range(pair_count):
            for j in range(i + 1, pair_count):
                src_delta = src[j] - src[i]
                dst_delta = dst[j] - dst[i]
                src_length = float(np.linalg.norm(src_delta))
                dst_length = float(np.linalg.norm(dst_delta))
                if (
                    src_length < self.min_pair_baseline
                    or dst_length < self.min_pair_baseline
                ):
                    continue
                scale = dst_length / src_length
                if not self.min_scale <= scale <= self.max_scale:
                    continue
                yaw = math.atan2(dst_delta[1], dst_delta[0]) - math.atan2(
                    src_delta[1], src_delta[0]
                )
                yaw = math.atan2(math.sin(yaw), math.cos(yaw))
                yaw_votes.append((yaw, 1.0))

        yaw_hypotheses = self.cluster_yaw_votes(yaw_votes)
        hypotheses = []
        for yaw, support in yaw_hypotheses:
            nearby = [
                item for item in affine_candidates
                if abs(math.atan2(
                    math.sin(item[0][2] - yaw), math.cos(item[0][2] - yaw)
                )) <= math.radians(self.yaw_cluster_deg * 1.5)
            ]
            if not nearby:
                continue
            transform, _ = max(nearby, key=lambda item: item[1])
            hypotheses.append((transform, support))
        stats["yaw_hypotheses"] = len(hypotheses)
        return hypotheses, stats

    def cluster_yaw_votes(self, yaw_votes):
        """Cluster circular yaw votes and return (yaw, normalized support)."""
        if not yaw_votes:
            return []
        bin_count = max(12, int(math.ceil(360.0 / self.yaw_cluster_deg)))
        weights = np.zeros(bin_count, dtype=np.float64)
        sin_sum = np.zeros(bin_count, dtype=np.float64)
        cos_sum = np.zeros(bin_count, dtype=np.float64)
        for yaw, weight in yaw_votes:
            normalized = (yaw + math.pi) % (2.0 * math.pi)
            index = min(bin_count - 1, int(normalized / (2.0 * math.pi) * bin_count))
            weights[index] += weight
            sin_sum[index] += weight * math.sin(yaw)
            cos_sum[index] += weight * math.cos(yaw)

        smoothed = np.array(
            [
                weights[(i - 1) % bin_count] + weights[i] + weights[(i + 1) % bin_count]
                for i in range(bin_count)
            ]
        )
        order = np.argsort(smoothed)[::-1]
        hypotheses = []
        for index in order:
            support = float(smoothed[index])
            if support <= 0.0:
                break
            indices = ((index - 1) % bin_count, index, (index + 1) % bin_count)
            yaw = math.atan2(
                float(sum(sin_sum[i] for i in indices)),
                float(sum(cos_sum[i] for i in indices)),
            )
            if any(
                abs(math.atan2(math.sin(yaw - old[0]), math.cos(yaw - old[0])))
                <= math.radians(self.yaw_cluster_deg)
                for old in hypotheses
            ):
                continue
            hypotheses.append((yaw, support))
            if len(hypotheses) >= self.max_yaw_hypotheses:
                break
        max_support = max(item[1] for item in hypotheses)
        return [(yaw, support / max_support) for yaw, support in hypotheses]

    @staticmethod
    def transform_point(transform, point):
        tx, ty, yaw = transform
        c, s = math.cos(yaw), math.sin(yaw)
        return np.array(
            [c * point[0] - s * point[1] + tx, s * point[0] + c * point[1] + ty]
        )

    @staticmethod
    def compose_transform(outer, inner):
        """Compose SE(2) transforms: result(point) = outer(inner(point))."""
        outer_tx, outer_ty, outer_yaw = outer
        inner_tx, inner_ty, inner_yaw = inner
        c, s = math.cos(outer_yaw), math.sin(outer_yaw)
        tx = outer_tx + c * inner_tx - s * inner_ty
        ty = outer_ty + s * inner_tx + c * inner_ty
        yaw = math.atan2(
            math.sin(outer_yaw + inner_yaw),
            math.cos(outer_yaw + inner_yaw),
        )
        return float(tx), float(ty), float(yaw)

    @staticmethod
    def inverse_transform(transform):
        """Invert an SE(2) transform."""
        tx, ty, yaw = transform
        c, s = math.cos(yaw), math.sin(yaw)
        return (
            float(-c * tx - s * ty),
            float(s * tx - c * ty),
            float(-yaw),
        )

    def make_overlap_context(self, ref):
        ref_data = np.asarray(ref.data, dtype=np.int16).reshape(
            ref.info.height, ref.info.width
        )
        non_wall = (ref_data < 60).astype(np.uint8)
        return {
            "data": ref_data,
            "wall_distance_px": cv2.distanceTransform(non_wall, cv2.DIST_L2, 5),
            "wall_tolerance_px": self.wall_tolerance / ref.info.resolution,
        }

    def overlap_score(self, ref, mov, transform, context=None):
        if context is None:
            context = self.make_overlap_context(ref)
        mov_data = np.asarray(mov.data, dtype=np.int16).reshape(
            mov.info.height, mov.info.width
        )
        ys, xs = np.where(mov_data >= 60)
        if len(xs) == 0:
            return -1.0, 0, 0, 0, 0.0
        stride = max(1, len(xs) // 5000)
        xy = np.column_stack((xs[::stride], ys[::stride]))
        local = self.pixel_to_local(mov, xy)
        tx, ty, yaw = transform
        c, s = math.cos(yaw), math.sin(yaw)
        rx = c * local[:, 0] - s * local[:, 1] + tx
        ry = s * local[:, 0] + c * local[:, 1] + ty
        gx = np.floor((rx - ref.info.origin.position.x) / ref.info.resolution).astype(
            int
        )
        gy = np.floor((ry - ref.info.origin.position.y) / ref.info.resolution).astype(
            int
        )
        inside = (gx >= 0) & (gy >= 0) & (gx < ref.info.width) & (gy < ref.info.height)
        if inside.sum() < 40:
            return -1.0, 0, 0, 0, 0.0
        values = context["data"][gy[inside], gx[inside]]
        distances = context["wall_distance_px"][gy[inside], gx[inside]]
        agree = distances <= context["wall_tolerance_px"]
        conflict = (values >= 0) & (values <= 40) & ~agree
        agree_count = int(np.count_nonzero(agree))
        conflict_count = int(np.count_nonzero(conflict))
        known_count = agree_count + conflict_count
        if known_count < 40:
            return -1.0, agree_count, conflict_count, known_count, 0.0
        score = float((agree_count - conflict_count) / known_count)
        coverage = float(known_count / len(xy))
        return score, agree_count, conflict_count, known_count, coverage

    @staticmethod
    def anchor_constrained_transform(yaw, anchor_ref, anchor_mov):
        """Return map1->map0 SE(2) whose common-anchor positions coincide."""
        c, s = math.cos(yaw), math.sin(yaw)
        rotated_x = c * anchor_mov[0] - s * anchor_mov[1]
        rotated_y = s * anchor_mov[0] + c * anchor_mov[1]
        tx = float(anchor_ref[0] - rotated_x)
        ty = float(anchor_ref[1] - rotated_y)
        normalized_yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        return tx, ty, normalized_yaw

    def refine_anchor_constrained_candidate(
        self, ref, mov, yaw, anchor_ref, anchor_mov, context
    ):
        """Refine feature yaw while recomputing translation from the anchor."""
        best_transform = self.anchor_constrained_transform(yaw, anchor_ref, anchor_mov)
        best_metrics = self.overlap_score(ref, mov, best_transform, context)
        stages = (
            (self.refine_yaw, self.refine_yaw_step),
            (self.refine_yaw_step, self.refine_fine_yaw_step),
        )
        center_yaw = yaw
        for yaw_radius, yaw_step in stages:
            yaw_offsets = np.deg2rad(
                np.arange(-yaw_radius, yaw_radius + 0.5 * yaw_step, yaw_step)
            )
            stage_metrics, stage_transform = best_metrics, best_transform
            for dyaw in yaw_offsets:
                candidate = self.anchor_constrained_transform(
                    center_yaw + float(dyaw), anchor_ref, anchor_mov
                )
                metrics = self.overlap_score(ref, mov, candidate, context)
                if (metrics[0], metrics[4]) > (stage_metrics[0], stage_metrics[4]):
                    stage_metrics, stage_transform = metrics, candidate
            best_metrics, best_transform = stage_metrics, stage_transform
            center_yaw = best_transform[2]
        return best_transform, best_metrics

    def refine_feature_candidate(
        self, ref, mov, initial, anchor_ref, anchor_mov, context
    ):
        """Refine the full ORB SE(2) estimate without forcing anchor coincidence."""
        best_transform = initial
        best_metrics = self.overlap_score(ref, mov, best_transform, context)

        def quality(transform, metrics):
            anchor_error = float(np.linalg.norm(
                self.transform_point(transform, anchor_mov) - anchor_ref
            ))
            return (
                metrics[0] + self.coverage_weight * metrics[4]
                - self.anchor_weight * anchor_error
            )

        stages = (
            (self.feature_refine_xy, math.radians(self.refine_yaw)),
            (max(0.10, self.feature_refine_xy / 3.0),
             math.radians(self.refine_yaw_step)),
        )
        for xy_radius, yaw_radius in stages:
            center = best_transform
            stage_transform, stage_metrics = best_transform, best_metrics
            for dx in (-xy_radius, 0.0, xy_radius):
                for dy in (-xy_radius, 0.0, xy_radius):
                    for dyaw in (-yaw_radius, 0.0, yaw_radius):
                        candidate = (
                            center[0] + dx,
                            center[1] + dy,
                            math.atan2(
                                math.sin(center[2] + dyaw),
                                math.cos(center[2] + dyaw),
                            ),
                        )
                        metrics = self.overlap_score(ref, mov, candidate, context)
                        if quality(candidate, metrics) > quality(
                            stage_transform, stage_metrics
                        ):
                            stage_transform, stage_metrics = candidate, metrics
            best_transform, best_metrics = stage_transform, stage_metrics
        return best_transform, best_metrics

    def select_transform(self, ref_ns, mov_ns):
        ref, mov = self.maps[ref_ns], self.maps[mov_ns]
        yaw_hypotheses, feature_stats = self.feature_candidates(ref, mov)
        a_ref, a_mov = self.anchors[ref_ns][0], self.anchors[mov_ns][0]
        overlap_context = self.make_overlap_context(ref)

        def feature_support_at(yaw):
            if not yaw_hypotheses:
                return 0.0
            sigma = max(math.radians(self.yaw_cluster_deg), 1e-6)
            return max(
                support
                * math.exp(
                    -0.5
                    * (
                        math.atan2(
                            math.sin(yaw - feature_yaw), math.cos(yaw - feature_yaw)
                        )
                        / sigma
                    )
                    ** 2
                )
                for feature_transform, support in yaw_hypotheses
                for feature_yaw in (feature_transform[2],)
            )

        def make_item(transform, metrics, support, mode):
            overlap, agree, conflict, known, coverage = metrics
            anchor_error = float(
                np.linalg.norm(self.transform_point(transform, a_mov) - a_ref)
            )
            total = (
                self.overlap_weight * overlap
                + self.coverage_weight * coverage
                + self.feature_weight * support
                - self.anchor_weight * anchor_error
            )
            return {
                "total": total,
                "overlap": overlap,
                "coverage": coverage,
                "anchor_error": anchor_error,
                "support": support,
                "transform": transform,
                "agree": agree,
                "conflict": conflict,
                "known": known,
                "mode": mode,
            }

        def is_valid(item):
            valid = (
                item["overlap"] >= self.min_overlap_score
                and item["coverage"] >= self.min_overlap_coverage
                and item["anchor_error"] <= self.max_anchor_match
            )
            if not valid:
                return False
            if item["mode"] == "global" and item["support"] < 0.20:
                return (
                    item["overlap"] >= self.global_min_overlap
                    and item["coverage"] >= self.global_min_coverage
                    and item["known"] >= self.global_min_known_cells
                )
            return True

        feature_items = []
        for feature_transform, support in yaw_hypotheses:
            transform, metrics = self.refine_feature_candidate(
                ref, mov, feature_transform, a_ref, a_mov, overlap_context
            )
            feature_items.append(
                make_item(
                    transform, metrics, feature_support_at(transform[2]), "feature"
                )
            )

        valid_items = [item for item in feature_items if is_valid(item)]
        evaluated_items = list(feature_items)

        if not valid_items:
            coarse_items = []
            for yaw_deg in np.arange(-180.0, 180.0, self.global_yaw_step):
                yaw = math.radians(float(yaw_deg))
                transform = self.anchor_constrained_transform(yaw, a_ref, a_mov)
                metrics = self.overlap_score(ref, mov, transform, overlap_context)
                coarse_items.append(
                    make_item(
                        transform, metrics, feature_support_at(yaw), "global_coarse"
                    )
                )

            coarse_items.sort(
                key=lambda item: (item["overlap"], item["coverage"]), reverse=True
            )

            coarse_peaks = []
            for item in coarse_items:
                yaw = item["transform"][2]
                if any(
                    abs(
                        math.atan2(
                            math.sin(yaw - old["transform"][2]),
                            math.cos(yaw - old["transform"][2]),
                        )
                    )
                    < self.global_yaw_nms
                    for old in coarse_peaks
                ):
                    continue
                coarse_peaks.append(item)
                if len(coarse_peaks) >= self.global_yaw_top_k:
                    break

            global_items = []
            for peak in coarse_peaks:
                transform, metrics = self.refine_anchor_constrained_candidate(
                    ref, mov, peak["transform"][2], a_ref, a_mov, overlap_context
                )
                global_items.append(
                    make_item(
                        transform, metrics, feature_support_at(transform[2]), "global"
                    )
                )
            evaluated_items.extend(global_items)
            valid_items = [item for item in global_items if is_valid(item)]

        pool = valid_items if valid_items else evaluated_items
        if not pool:
            self.get_logger().warn("registration rejected: no yaw hypothesis")
            return None
        best = max(pool, key=lambda item: item["total"])
        if not is_valid(best):
            self.get_logger().warn(
                f"registration rejected: mode={best['mode']}, "
                f"overlap={best['overlap']:.3f}, coverage={best['coverage']:.3f}, "
                f"known={best['known']}, feature_support={best['support']:.3f}, "
                f"anchor_error={best['anchor_error']:.3f}m, "
                f"features=({feature_stats['ref_keypoints']}/"
                f"{feature_stats['mov_keypoints']} kp, "
                f"{feature_stats['ratio_matches']} matches, "
                f"{feature_stats['ransac_votes']} ransac, "
                f"{feature_stats['yaw_hypotheses']} yaw), "
                f"tx={best['transform'][0]:.3f}m, ty={best['transform'][1]:.3f}m, "
                f"yaw={math.degrees(best['transform'][2]):.2f}deg"
            )
            return None
        self.get_logger().info(
            f"registration accepted: mode={best['mode']}, "
            f"overlap={best['overlap']:.3f}, coverage={best['coverage']:.3f}, "
            f"known={best['known']}, feature_support={best['support']:.3f}, "
            f"anchor_error={best['anchor_error']:.3f}m, "
            f"features=({feature_stats['ref_keypoints']}/"
            f"{feature_stats['mov_keypoints']} kp, "
            f"{feature_stats['ratio_matches']} matches, "
            f"{feature_stats['ransac_votes']} ransac, "
            f"{feature_stats['yaw_hypotheses']} yaw), "
            f"tx={best['transform'][0]:.3f}m, ty={best['transform'][1]:.3f}m, "
            f"yaw={math.degrees(best['transform'][2]):.2f}deg"
        )
        return best

    def build_overlap_transform_graph(self):
        """Register every map pair and keep the best connected edge set.

        Each accepted edge stores a transform from ``mov`` into ``ref``.  A
        maximum-spanning-tree selection prefers strongly overlapping pairs, so
        a robot need not overlap the configured reference map directly.
        """
        for ref_index, ref_ns in enumerate(self.robots):
            for mov_ns in self.robots[ref_index + 1 :]:
                pair_key = (ref_ns, mov_ns)
                self.get_logger().info(f"evaluating map pair: {mov_ns} -> {ref_ns}")
                result = self.select_transform(ref_ns, mov_ns)
                if result is None:
                    if pair_key in self.edge_cache:
                        cached = self.edge_cache[pair_key]
                        self.get_logger().warn(
                            f"current map pair unavailable: {mov_ns} <-> {ref_ns}; "
                            f"retaining cached edge score={cached['score']:.3f}"
                        )
                    else:
                        self.get_logger().warn(
                            f"map pair unavailable and not cached: "
                            f"{mov_ns} <-> {ref_ns}"
                        )
                    continue

                edge = {
                    "ref": ref_ns,
                    "mov": mov_ns,
                    "transform": result["transform"],
                    "score": result["total"],
                    "overlap": result["overlap"],
                    "coverage": result["coverage"],
                    "support": result["support"],
                    "anchor_error": result["anchor_error"],
                    "stamp_ns": self.get_clock().now().nanoseconds,
                }
                cached = self.edge_cache.get(pair_key)
                if cached is None:
                    self.edge_cache[pair_key] = edge
                    self.get_logger().info(
                        f"cached new map edge: {mov_ns} -> {ref_ns}, "
                        f"score={edge['score']:.3f}"
                    )
                elif (edge["score"], edge["coverage"]) > (
                    cached["score"],
                    cached["coverage"],
                ):
                    self.edge_cache[pair_key] = edge
                    self.get_logger().info(
                        f"updated cached map edge: {mov_ns} -> {ref_ns}, "
                        f"score={cached['score']:.3f}->{edge['score']:.3f}"
                    )
                else:
                    self.get_logger().info(
                        f"retaining better cached map edge: {mov_ns} -> {ref_ns}, "
                        f"cached_score={cached['score']:.3f}, "
                        f"current_score={edge['score']:.3f}"
                    )

        candidates = list(self.edge_cache.values())
        self.get_logger().info(
            f"building transform graph from {len(candidates)} cached map edges"
        )

        candidates.sort(key=lambda edge: edge["score"], reverse=True)

        parent = {ns: ns for ns in self.robots}

        def find(ns):
            while parent[ns] != ns:
                parent[ns] = parent[parent[ns]]
                ns = parent[ns]
            return ns

        selected = []
        for edge in candidates:
            ref_root = find(edge["ref"])
            mov_root = find(edge["mov"])
            if ref_root == mov_root:
                continue
            parent[mov_root] = ref_root
            selected.append(edge)
            self.get_logger().info(
                f"selected map edge: {edge['mov']} -> {edge['ref']}, "
                f"score={edge['score']:.3f}, overlap={edge['overlap']:.3f}, "
                f"coverage={edge['coverage']:.3f}"
            )
            if len(selected) == len(self.robots) - 1:
                break

        if len(selected) != len(self.robots) - 1:
            connected = sorted(
                ns for ns in self.robots if find(ns) == find(self.reference_robot)
            )
            self.get_logger().error(
                "map registration graph is disconnected: "
                f"connected_to_{self.reference_robot}={connected}"
            )
            return None

        adjacency = defaultdict(list)
        for edge in selected:
            ref_ns = edge["ref"]
            mov_ns = edge["mov"]
            mov_to_ref = edge["transform"]
            adjacency[ref_ns].append((mov_ns, mov_to_ref))
            adjacency[mov_ns].append(
                (ref_ns, self.inverse_transform(mov_to_ref))
            )

        transforms = {self.reference_robot: (0.0, 0.0, 0.0)}
        queue = deque([self.reference_robot])
        while queue:
            current = queue.popleft()
            for neighbor, neighbor_to_current in adjacency[current]:
                if neighbor in transforms:
                    continue
                transforms[neighbor] = self.compose_transform(
                    transforms[current], neighbor_to_current
                )
                queue.append(neighbor)

        return transforms

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
            corners = np.array(
                [
                    [msg.info.origin.position.x, msg.info.origin.position.y],
                    [
                        msg.info.origin.position.x
                        + msg.info.width * msg.info.resolution,
                        msg.info.origin.position.y,
                    ],
                    [
                        msg.info.origin.position.x,
                        msg.info.origin.position.y
                        + msg.info.height * msg.info.resolution,
                    ],
                    [
                        msg.info.origin.position.x
                        + msg.info.width * msg.info.resolution,
                        msg.info.origin.position.y
                        + msg.info.height * msg.info.resolution,
                    ],
                ]
            )
            bounds.append(np.array([self.transform_point(t, p) for p in corners]))
        all_bounds = np.vstack(bounds)
        min_xy = all_bounds.min(axis=0) - self.map_padding
        max_xy = all_bounds.max(axis=0) + self.map_padding
        width, height = np.ceil((max_xy - min_xy) / res).astype(int)
        sums = np.zeros((height, width), dtype=np.float32)
        counts = np.zeros((height, width), dtype=np.int16)
        for ns, msg in self.maps.items():
            data = np.asarray(msg.data, dtype=np.int16).reshape(
                msg.info.height, msg.info.width
            )
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
        merged.info.origin.position.x, merged.info.origin.position.y = map(
            float, min_xy
        )
        merged.info.origin.orientation.w = 1.0
        merged.data = out.ravel().tolist()
        self.map_pub.publish(merged)

    def tick(self):

        # 이미 한 번 registration 성공했으면 기존 transform을 계속 사용해서 map만 갱신
        if self.locked:
            if any(ns not in self.maps for ns in self.robots):
                self.get_logger().warn(f"waiting maps: have={list(self.maps.keys())}")
                return
            self.get_logger().info(
                "registration already locked -> using existing transforms"
            )

            for ns, transform in self.transforms.items():
                self.get_logger().info(
                    f"[{ns}] fixed transform: "
                    f"tx={transform[0]:.3f}, "
                    f"ty={transform[1]:.3f}, "
                    f"yaw={math.degrees(transform[2]):.2f} deg"
                )

            self.merge_and_publish()
            return

        for ns in self.robots:
            numeric_samples = [sample[:3] for sample in self.samples[ns]]
            front_count = sum(sample[3] == "front" for sample in self.samples[ns])
            back_count = sum(sample[3] == "back" for sample in self.samples[ns])
            min_per_tag = max(3, self.min_samples // 2)

            self.get_logger().info(
                f"[{ns}] samples: total={len(numeric_samples)}, "
                f"front={front_count}, back={back_count}, "
                f"required_total={self.min_samples}, "
                f"required_per_tag={min_per_tag}"
            )

            estimate = self.estimate_anchor(numeric_samples)

            if estimate is None:
                self.get_logger().warn(
                    f"[{ns}] anchor estimation failed "
                    f"(samples={len(numeric_samples)})"
                )
            else:
                anchor, rmse = estimate

                self.get_logger().info(
                    f"[{ns}] anchor estimate: "
                    f"x={anchor[0]:.3f}, y={anchor[1]:.3f}, "
                    f"rmse={rmse:.3f} m "
                    f"(max={self.max_anchor_rmse:.3f} m)"
                )

            if (
                estimate is not None
                and len(numeric_samples) >= self.min_samples
                and front_count >= min_per_tag
                and back_count >= min_per_tag
            ):
                self.anchors[ns] = estimate

        # 병합 로직과 독립적으로, 준비된 로봇부터 자기 local-map의 앵커
        # 방향으로 유도한다. 숫자 좌표는 달라도 동일한 물리 앵커를 뜻한다.
        self.publish_rendezvous_commands()

        if any(ns not in self.maps for ns in self.robots):
            self.get_logger().warn(f"waiting maps: have={list(self.maps.keys())}")
            return

        anchor_not_ready = False

        for ns in self.robots:
            if ns not in self.anchors:
                self.get_logger().warn(
                    f"[{ns}] waiting anchor: anchor does not exist yet"
                )
                anchor_not_ready = True
                continue

            anchor, rmse = self.anchors[ns]

            if rmse > self.max_anchor_rmse:
                self.get_logger().warn(
                    f"[{ns}] anchor rejected by RMSE: "
                    f"{rmse:.3f} > {self.max_anchor_rmse:.3f}"
                )
                anchor_not_ready = True

        if anchor_not_ready:
            self.get_logger().warn("map merging waiting: anchors are not ready")
            return

        self.get_logger().info(
            "starting pairwise map registration: "
            f"world_reference={self.reference_robot}"
        )

        transforms = self.build_overlap_transform_graph()
        if transforms is None:
            self.transforms = {}
            self.publish_valid(False)
            return
        self.transforms = transforms

        for ns, transform in self.transforms.items():
            self.get_logger().info(
                f"registration transform LOCKED: {ns}/map -> world, "
                f"tx={transform[0]:.3f}, ty={transform[1]:.3f}, "
                f"yaw={math.degrees(transform[2]):.2f} deg"
            )

        self.locked = True
        self.get_logger().info("map registration complete -> transforms are now LOCKED")
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
