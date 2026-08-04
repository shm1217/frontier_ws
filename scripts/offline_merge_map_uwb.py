#!/usr/bin/env python3
"""Merge two saved ROS occupancy maps using map features only.

This is an offline test utility: it does not use ROS topics, TF, odometry, or
UWB.  It loads two map_server YAML/PGM pairs, extracts ORB features, estimates
map-1 -> map-0 SE(2) candidates with RANSAC, validates them with occupancy
overlap, and writes a merged PGM/YAML plus diagnostic files.
"""

import argparse
import math
import os
from dataclasses import dataclass

import cv2
import numpy as np
import yaml


@dataclass
class SavedMap:
    yaml_path: str
    image: np.ndarray
    occupancy: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def height(self):
        return int(self.occupancy.shape[0])

    @property
    def width(self):
        return int(self.occupancy.shape[1])

    def pixels_to_local(self, xy):
        """OpenCV image pixels (x right, y down) -> ROS map metric points."""
        xy = np.asarray(xy, dtype=np.float64)
        out = np.empty_like(xy)
        out[..., 0] = self.origin_x + (xy[..., 0] + 0.5) * self.resolution
        out[..., 1] = (
            self.origin_y + (self.height - xy[..., 1] - 0.5) * self.resolution
        )
        return out

    def local_to_pixels(self, xy):
        xy = np.asarray(xy, dtype=np.float64)
        out = np.empty_like(xy)
        out[..., 0] = (xy[..., 0] - self.origin_x) / self.resolution - 0.5
        grid_y = (xy[..., 1] - self.origin_y) / self.resolution - 0.5
        out[..., 1] = self.height - 1.0 - grid_y
        return out


def load_saved_map(yaml_path):
    yaml_path = os.path.abspath(yaml_path)
    with open(yaml_path, "r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)

    image_path = str(metadata["image"])
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)
    raw = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if raw is None:
        raise RuntimeError(f"failed to read map image: {image_path}")

    resolution = float(metadata["resolution"])
    origin = metadata.get("origin", [0.0, 0.0, 0.0])
    negate = int(metadata.get("negate", 0))
    occupied_thresh = float(metadata.get("occupied_thresh", 0.65))
    free_thresh = float(metadata.get("free_thresh", 0.196))

    pixel_probability = raw.astype(np.float32) / 255.0
    occupied_probability = pixel_probability if negate else 1.0 - pixel_probability
    # map_saver uses gray 205 for unknown.  A permissive free_thresh (for
    # example 0.25) would otherwise misclassify it as free because
    # 1 - 205/255 is about 0.196.  Preserve the map_saver sentinel explicitly.
    unknown_mask = (raw >= 204) & (raw <= 206)
    occupancy = np.full(raw.shape, -1, dtype=np.int16)
    occupancy[occupied_probability >= occupied_thresh] = 100
    occupancy[occupied_probability <= free_thresh] = 0
    occupancy[unknown_mask] = -1

    # Extract features from occupied structure only.  Giving unknown and free
    # cells different gray levels creates strong, but meaningless, features on
    # the changing frontier/PGM crop boundary.
    feature_image = np.full(raw.shape, 255, dtype=np.uint8)
    feature_image[occupancy == 100] = 0
    return SavedMap(
        yaml_path=yaml_path,
        image=feature_image,
        occupancy=occupancy,
        resolution=resolution,
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
    )


def transform_points(transform, points):
    tx, ty, yaw = transform
    points = np.asarray(points, dtype=np.float64)
    c, s = math.cos(yaw), math.sin(yaw)
    out = np.empty_like(points)
    out[..., 0] = c * points[..., 0] - s * points[..., 1] + tx
    out[..., 1] = s * points[..., 0] + c * points[..., 1] + ty
    return out


def extract_matches(reference, moving, args):
    orb = cv2.ORB_create(
        nfeatures=args.max_features,
        fastThreshold=args.orb_fast_threshold,
        edgeThreshold=args.orb_edge_threshold,
        patchSize=args.orb_patch_size,
    )
    kp_ref, des_ref = orb.detectAndCompute(reference.image, None)
    kp_mov, des_mov = orb.detectAndCompute(moving.image, None)
    if des_ref is None or des_mov is None:
        return kp_ref, kp_mov, []

    pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(des_mov, des_ref, k=2)
    matches = []
    for pair in pairs:
        if len(pair) < 2:
            continue
        first, second = pair[0], pair[1]
        if first.distance < args.ratio * second.distance:
            matches.append(first)
    matches.sort(key=lambda item: item.distance)
    return kp_ref, kp_mov, matches


def feature_candidates(reference, moving, kp_ref, kp_mov, matches, args):
    if len(matches) < args.min_feature_matches:
        return []
    src_px = np.float32([kp_mov[m.queryIdx].pt for m in matches])
    dst_px = np.float32([kp_ref[m.trainIdx].pt for m in matches])
    src = moving.pixels_to_local(src_px)
    dst = reference.pixels_to_local(dst_px)

    rng = np.random.default_rng(args.random_seed)
    batches = [np.arange(len(matches))]
    for _ in range(args.ransac_batches):
        size = min(len(matches), max(6, len(matches) // 2))
        batches.append(rng.choice(len(matches), size=size, replace=False))

    candidates = []
    for idx in batches:
        matrix, inliers = cv2.estimateAffinePartial2D(
            src[idx], dst[idx], method=cv2.RANSAC,
            ransacReprojThreshold=args.ransac_threshold_m,
            maxIters=3000, confidence=0.995,
        )
        if matrix is None or inliers is None:
            continue
        inlier_count = int(inliers.sum())
        inlier_ratio = inlier_count / max(1, len(idx))
        if inlier_count < args.min_ransac_inliers:
            continue
        if inlier_ratio < args.min_ransac_inlier_ratio:
            continue
        scale = math.hypot(matrix[0, 0], matrix[1, 0])
        if not args.min_scale <= scale <= args.max_scale:
            continue
        yaw = math.atan2(matrix[1, 0], matrix[0, 0])
        candidate = (float(matrix[0, 2]), float(matrix[1, 2]), float(yaw))
        duplicate = any(
            abs(math.atan2(math.sin(yaw - old[2]), math.cos(yaw - old[2])))
            <= math.radians(args.dedup_yaw_deg)
            and math.hypot(candidate[0] - old[0], candidate[1] - old[1])
            <= args.dedup_translation_m
            for old in candidates
        )
        if not duplicate:
            candidates.append(candidate)
    return candidates


def make_overlap_context(reference, wall_tolerance_m):
    """Precompute each reference cell's distance to the nearest wall."""
    non_wall = (reference.occupancy != 100).astype(np.uint8)
    wall_distance_px = cv2.distanceTransform(non_wall, cv2.DIST_L2, 5)
    return {
        "wall_distance_px": wall_distance_px,
        "wall_tolerance_px": wall_tolerance_m / reference.resolution,
    }


def overlap_score(reference, moving, transform, context, max_points=20000):
    rows, cols = np.where(moving.occupancy == 100)
    if len(cols) == 0:
        return -1.0, 0, 0, 0
    stride = max(1, len(cols) // max_points)
    pixels = np.column_stack((cols[::stride], rows[::stride]))
    points = moving.pixels_to_local(pixels)
    transformed = transform_points(transform, points)
    ref_pixels = np.rint(reference.local_to_pixels(transformed)).astype(np.int64)
    x, y = ref_pixels[:, 0], ref_pixels[:, 1]
    inside = (x >= 0) & (y >= 0) & (x < reference.width) & (y < reference.height)
    if int(inside.sum()) < 20:
        return -1.0, 0, 0, 0
    values = reference.occupancy[y[inside], x[inside]]
    distances = context["wall_distance_px"][y[inside], x[inside]]

    # A transformed wall is an agreement when it falls near a reference wall,
    # rather than requiring the two SLAM rasters to hit the exact same pixel.
    agree_mask = distances <= context["wall_tolerance_px"]
    # Unknown cells do not vote.  A conflict requires confirmed free space and
    # no reference wall within the tolerance radius.
    conflict_mask = (values == 0) & ~agree_mask
    agree = int(np.count_nonzero(agree_mask))
    conflict = int(np.count_nonzero(conflict_mask))
    known_count = agree + conflict
    if known_count < 20:
        return -1.0, 0, 0, 0
    score = float((agree - conflict) / known_count)
    return score, agree, conflict, known_count


def refine_candidate(reference, moving, transform, context, args):
    """Locally refine a feature-based candidate using occupancy agreement."""
    best = (*overlap_score(reference, moving, transform, context), transform)
    stages = (
        (args.refine_translation_m, args.refine_translation_step_m,
         args.refine_yaw_deg, args.refine_yaw_step_deg),
        (args.refine_translation_step_m, args.refine_fine_translation_step_m,
         args.refine_yaw_step_deg, args.refine_fine_yaw_step_deg),
    )
    center = transform
    for translation_radius, translation_step, yaw_radius_deg, yaw_step_deg in stages:
        translation_offsets = np.arange(
            -translation_radius, translation_radius + 0.5 * translation_step,
            translation_step)
        yaw_offsets = np.deg2rad(np.arange(
            -yaw_radius_deg, yaw_radius_deg + 0.5 * yaw_step_deg,
            yaw_step_deg))
        stage_best = best
        for dx in translation_offsets:
            for dy in translation_offsets:
                for dyaw in yaw_offsets:
                    candidate = (center[0] + float(dx), center[1] + float(dy),
                                 center[2] + float(dyaw))
                    result = overlap_score(
                        reference, moving, candidate, context)
                    if result[0] > stage_best[0]:
                        stage_best = (*result, candidate)
        best = stage_best
        center = best[4]
    return best


def select_candidate(reference, moving, candidates, min_overlap, args):
    context = make_overlap_context(reference, args.wall_tolerance_m)
    ranked = []
    for transform in candidates:
        ranked.append(refine_candidate(
            reference, moving, transform, context, args))
    ranked.sort(key=lambda item: item[0], reverse=True)
    if not ranked or ranked[0][0] < min_overlap:
        return None, ranked
    return ranked[0][4], ranked


def metric_cell_centers(saved_map):
    rows, cols = np.where(saved_map.occupancy >= 0)
    pixels = np.column_stack((cols, rows))
    return saved_map.pixels_to_local(pixels), saved_map.occupancy[rows, cols]


def merge_maps(reference, moving, transform, output_resolution, padding):
    corners = []
    for saved_map, map_transform in (
        (reference, (0.0, 0.0, 0.0)), (moving, transform)
    ):
        local = np.array([
            [saved_map.origin_x, saved_map.origin_y],
            [saved_map.origin_x + saved_map.width * saved_map.resolution,
             saved_map.origin_y],
            [saved_map.origin_x,
             saved_map.origin_y + saved_map.height * saved_map.resolution],
            [saved_map.origin_x + saved_map.width * saved_map.resolution,
             saved_map.origin_y + saved_map.height * saved_map.resolution],
        ])
        corners.append(transform_points(map_transform, local))
    bounds = np.vstack(corners)
    minimum = bounds.min(axis=0) - padding
    maximum = bounds.max(axis=0) + padding
    width, height = np.ceil((maximum - minimum) / output_resolution).astype(int)
    probability_sum = np.zeros((height, width), dtype=np.float64)
    count = np.zeros((height, width), dtype=np.int32)

    for saved_map, map_transform in (
        (reference, (0.0, 0.0, 0.0)), (moving, transform)
    ):
        points, occupancy = metric_cell_centers(saved_map)
        points = transform_points(map_transform, points)
        gx = np.floor((points[:, 0] - minimum[0]) / output_resolution).astype(int)
        gy = np.floor((points[:, 1] - minimum[1]) / output_resolution).astype(int)
        valid = (gx >= 0) & (gy >= 0) & (gx < width) & (gy < height)
        # Array rows are image-style (top down), while gy is ROS grid-style.
        rows = height - 1 - gy[valid]
        cols = gx[valid]
        np.add.at(probability_sum, (rows, cols), occupancy[valid] / 100.0)
        np.add.at(count, (rows, cols), 1)

    merged = np.full((height, width), -1, dtype=np.int16)
    seen = count > 0
    merged[seen] = np.rint(
        100.0 * probability_sum[seen] / count[seen]
    ).astype(np.int16)
    return merged, minimum


def save_merged_map(occupancy, origin, resolution, output_prefix):
    output_prefix = os.path.abspath(output_prefix)
    directory = os.path.dirname(output_prefix)
    if directory:
        os.makedirs(directory, exist_ok=True)
    pgm_path = output_prefix + ".pgm"
    yaml_path = output_prefix + ".yaml"
    image = np.full(occupancy.shape, 205, dtype=np.uint8)
    known = occupancy >= 0
    image[known] = np.clip(
        np.rint(255.0 * (1.0 - occupancy[known] / 100.0)), 0, 255
    ).astype(np.uint8)
    if not cv2.imwrite(pgm_path, image):
        raise RuntimeError(f"failed to write {pgm_path}")
    metadata = {
        "image": os.path.basename(pgm_path),
        "mode": "trinary",
        "resolution": float(resolution),
        "origin": [float(origin[0]), float(origin[1]), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    with open(yaml_path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(metadata, stream, sort_keys=False)
    return pgm_path, yaml_path


def save_diagnostics(reference, moving, kp_ref, kp_mov, matches, ranked, prefix):
    match_image = cv2.drawMatches(
        moving.image, kp_mov, reference.image, kp_ref,
        matches[:100], None,
        flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS,
    )
    cv2.imwrite(prefix + "_matches.png", match_image)
    with open(prefix + "_result.yaml", "w", encoding="utf-8") as stream:
        rows = []
        for score, agree, conflict, known, transform in ranked:
            rows.append({
                "tx": float(transform[0]),
                "ty": float(transform[1]),
                "yaw_rad": float(transform[2]),
                "yaw_deg": float(math.degrees(transform[2])),
                "overlap_score": float(score),
                "occupied_agree": int(agree),
                "free_conflict": int(conflict),
                "known_compared": int(known),
            })
        yaml.safe_dump({"candidates": rows}, stream, sort_keys=False)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("map0_yaml", help="reference map YAML, e.g. tb3_0_map.yaml")
    parser.add_argument("map1_yaml", help="moving map YAML, e.g. tb3_1_map.yaml")
    parser.add_argument("--output-prefix", default="merged_map_uwb")
    parser.add_argument("--max-features", type=int, default=2500)
    parser.add_argument("--orb-fast-threshold", type=int, default=5)
    parser.add_argument("--orb-edge-threshold", type=int, default=8,
                        help="allow features near cropped map borders")
    parser.add_argument("--orb-patch-size", type=int, default=31)
    parser.add_argument("--ratio", type=float, default=0.8) # 0.78 (높을수록 더 많은 매칭이 통과됨)
    parser.add_argument("--min-feature-matches", type=int, default=4) # 12
    parser.add_argument("--ransac-batches", type=int, default=200) # 60
    parser.add_argument("--ransac-threshold-m", type=float, default=0.50) # 0.2 (대응점 threshold)
    parser.add_argument("--min-ransac-inliers", type=int, default=4) # 6
    parser.add_argument("--min-ransac-inlier-ratio", type=float, default=0.30)
    parser.add_argument("--min-scale", type=float, default=0.95)
    parser.add_argument("--max-scale", type=float, default=1.05)
    parser.add_argument("--dedup-yaw-deg", type=float, default=2.0)
    parser.add_argument("--dedup-translation-m", type=float, default=0.20)
    parser.add_argument("--min-overlap-score", type=float, default=0.15)
    parser.add_argument("--wall-tolerance-m", type=float, default=0.1, # 0.1
                        help="maximum distance from a transformed wall to a reference wall")
    parser.add_argument("--refine-translation-m", type=float, default=0.30,
                        help="coarse local-search radius around each RANSAC candidate")
    parser.add_argument("--refine-translation-step-m", type=float, default=0.10)
    parser.add_argument("--refine-yaw-deg", type=float, default=3.0)
    parser.add_argument("--refine-yaw-step-deg", type=float, default=1.0)
    parser.add_argument("--refine-fine-translation-step-m", type=float, default=0.025)
    parser.add_argument("--refine-fine-yaw-step-deg", type=float, default=0.25)
    parser.add_argument("--output-resolution", type=float, default=0.0,
                        help="0 uses map0 resolution")
    parser.add_argument("--padding-m", type=float, default=1.0)
    parser.add_argument("--random-seed", type=int, default=7)
    return parser.parse_args()


def main():
    args = parse_args()
    reference = load_saved_map(args.map0_yaml)
    moving = load_saved_map(args.map1_yaml)
    kp_ref, kp_mov, matches = extract_matches(reference, moving, args)
    print(f"ORB keypoints: map0={len(kp_ref)}, map1={len(kp_mov)}")
    print(f"ratio-test matches: {len(matches)}")

    candidates = feature_candidates(reference, moving, kp_ref, kp_mov, matches, args)
    print(f"RANSAC candidates: {len(candidates)}")
    selected, ranked = select_candidate(
        reference, moving, candidates, args.min_overlap_score, args)
    prefix = os.path.abspath(args.output_prefix)
    save_diagnostics(reference, moving, kp_ref, kp_mov, matches, ranked, prefix)

    for index, item in enumerate(ranked[:10]):
        score, agree, conflict, known, transform = item
        print(
            f"candidate[{index}] tx={transform[0]:.3f}m "
            f"ty={transform[1]:.3f}m yaw={math.degrees(transform[2]):.2f}deg "
            f"overlap={score:.3f} agree={agree} conflict={conflict} known={known}"
        )
    if selected is None:
        raise SystemExit(
            "registration failed: no candidate passed the overlap threshold; "
            f"see {prefix}_matches.png and {prefix}_result.yaml"
        )

    resolution = args.output_resolution or reference.resolution
    merged, origin = merge_maps(
        reference, moving, selected, resolution, args.padding_m)
    pgm_path, yaml_path = save_merged_map(merged, origin, resolution, prefix)
    print(
        f"selected transform map1->map0: tx={selected[0]:.4f}m, "
        f"ty={selected[1]:.4f}m, yaw={math.degrees(selected[2]):.3f}deg"
    )
    print(f"merged PGM:  {pgm_path}")
    print(f"merged YAML: {yaml_path}")
    print(f"matches PNG: {prefix}_matches.png")
    print(f"results YAML: {prefix}_result.yaml")


if __name__ == "__main__":
    main()
