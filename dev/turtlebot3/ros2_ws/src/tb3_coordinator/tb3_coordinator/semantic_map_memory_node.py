#!/usr/bin/env python3
"""
semantic_map_memory_node.py — Persistent semantic landmark memory with
occupancy-grid validation, wall-island rejection, and obstacle-island refinement.

Pipeline per observation:
  1. TF transform base_link -> map
  2. Prefer occupancy-grid validation and obstacle-island refinement
  3. Optionally keep repeated LiDAR-grounded observations when the map is not
     available or has not expanded to the object yet
  4. Match refined/provisional point against existing landmarks/candidates
  5. Candidates promoted to persistent landmarks after min_observations
  6. Publish all persistent landmarks as MarkerArray and SemanticMapState
"""

from __future__ import annotations

import math
import time as _time
from collections import deque
from dataclasses import dataclass
from typing import Optional

import numpy as np

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration

from visualization_msgs.msg import Marker, MarkerArray
from vision_msgs.msg import Detection3DArray
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from std_msgs.msg import ColorRGBA

import tf2_ros
import tf2_geometry_msgs  # noqa: F401

from ament_index_python.packages import get_package_share_directory
from tb3_query.query_core import load_target_mapping
from tb3_semantic_map_msgs.msg import SemanticEntity, SemanticMapState


@dataclass
class Candidate:
    semantic_class: str
    x: float
    y: float
    obs_count: int = 1
    last_seen: float = 0.0
    validation: str = "occupancy_grid_validated"


@dataclass
class Landmark:
    landmark_id: str
    semantic_class: str
    x: float
    y: float
    observation_count: int = 1
    last_seen: float = 0.0
    validation: str = "occupancy_grid_validated"


CLASS_COLORS = {
    "person":    ColorRGBA(r=0.2, g=0.8, b=0.2, a=0.9),
    "bench":     ColorRGBA(r=0.8, g=0.6, b=0.2, a=0.9),
    "stop sign": ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.9),
}
DEFAULT_COLOR = ColorRGBA(r=0.6, g=0.6, b=0.6, a=0.9)


def world_to_grid(wx, wy, ox, oy, res):
    return int((wx - ox) / res), int((wy - oy) / res)


def grid_to_world(gx, gy, ox, oy, res):
    return ox + (gx + 0.5) * res, oy + (gy + 0.5) * res


def find_nearest_valid_island(
    grid, width, height, cx, cy, search_radius,
    origin_x, origin_y, resolution,
    occupied_thresh=50, min_island_pixels=2, wall_margin_cells=4,
):
    """BFS flood-fill for obstacle islands, rejecting wall-like islands."""
    x_lo = max(0, cx - search_radius)
    x_hi = min(width, cx + search_radius + 1)
    y_lo = max(0, cy - search_radius)
    y_hi = min(height, cy + search_radius + 1)

    visited = set()
    islands = []

    for iy in range(y_lo, y_hi):
        for ix in range(x_lo, x_hi):
            if (ix, iy) in visited:
                continue
            val = grid[iy * width + ix]
            if val < occupied_thresh:
                visited.add((ix, iy))
                continue
            island = []
            queue = deque()
            queue.append((ix, iy))
            visited.add((ix, iy))
            touches_border = False
            while queue:
                px, py = queue.popleft()
                island.append((px, py))
                if px <= 0 or px >= width - 1 or py <= 0 or py >= height - 1:
                    touches_border = True
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = px + dx, py + dy
                    if x_lo <= nx < x_hi and y_lo <= ny < y_hi and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        if grid[ny * width + nx] >= occupied_thresh:
                            queue.append((nx, ny))
            if len(island) < min_island_pixels:
                continue
            if touches_border:
                continue
            n = len(island)
            avg_gx = sum(p[0] for p in island) / n
            avg_gy = sum(p[1] for p in island) / n
            if (avg_gx < wall_margin_cells or avg_gx >= width - wall_margin_cells or
                    avg_gy < wall_margin_cells or avg_gy >= height - wall_margin_cells):
                continue
            islands.append((avg_gx, avg_gy))

    if not islands:
        return None
    best = min(islands, key=lambda t: math.hypot(t[0] - cx, t[1] - cy))
    return grid_to_world(int(best[0]), int(best[1]), origin_x, origin_y, resolution)



def find_bench_cluster_centroid(
    grid, width, height, cx, cy, search_radius,
    origin_x, origin_y, resolution,
    occupied_thresh=50, min_island_pixels=2, wall_margin_cells=4,
    cluster_radius_cells=8,
    logger=None,
):
    """For bench-like objects: cluster nearby valid islands and return
    the size-weighted centroid of the cluster nearest to the observation."""
    x_lo = max(0, cx - search_radius)
    x_hi = min(width, cx + search_radius + 1)
    y_lo = max(0, cy - search_radius)
    y_hi = min(height, cy + search_radius + 1)

    visited = set()
    valid_islands = []  # (avg_gx, avg_gy, pixel_count)

    for iy in range(y_lo, y_hi):
        for ix in range(x_lo, x_hi):
            if (ix, iy) in visited:
                continue
            val = grid[iy * width + ix]
            if val < occupied_thresh:
                visited.add((ix, iy))
                continue
            island = []
            queue = deque()
            queue.append((ix, iy))
            visited.add((ix, iy))
            touches_border = False
            while queue:
                px, py = queue.popleft()
                island.append((px, py))
                if px <= 0 or px >= width - 1 or py <= 0 or py >= height - 1:
                    touches_border = True
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = px + dx, py + dy
                    if x_lo <= nx < x_hi and y_lo <= ny < y_hi and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        if grid[ny * width + nx] >= occupied_thresh:
                            queue.append((nx, ny))
            if len(island) < min_island_pixels or touches_border:
                continue
            n = len(island)
            agx = sum(p[0] for p in island) / n
            agy = sum(p[1] for p in island) / n
            if (agx < wall_margin_cells or agx >= width - wall_margin_cells or
                    agy < wall_margin_cells or agy >= height - wall_margin_cells):
                continue
            valid_islands.append((agx, agy, n))

    if not valid_islands:
        return None

    # Find islands close to the observation point
    nearby = [(gx, gy, n) for gx, gy, n in valid_islands
              if math.hypot(gx - cx, gy - cy) <= cluster_radius_cells]

    if not nearby:
        # Fallback: nearest single island
        best = min(valid_islands, key=lambda t: math.hypot(t[0] - cx, t[1] - cy))
        pt = grid_to_world(int(best[0]), int(best[1]), origin_x, origin_y, resolution)
        if logger:
            logger.info("[bench] fallback nearest island at (%.2f, %.2f)" % pt)
        return pt

    if len(nearby) == 1:
        pt = grid_to_world(int(nearby[0][0]), int(nearby[0][1]),
                           origin_x, origin_y, resolution)
        if logger:
            logger.info("[bench] single island at (%.2f, %.2f)  %d px" % (pt[0], pt[1], nearby[0][2]))
        return pt

    # Multi-island cluster: size-weighted centroid
    total_px = sum(t[2] for t in nearby)
    wgx = sum(t[0] * t[2] for t in nearby) / total_px
    wgy = sum(t[1] * t[2] for t in nearby) / total_px
    pt = grid_to_world(int(wgx), int(wgy), origin_x, origin_y, resolution)
    if logger:
        logger.info("[bench] cluster %d islands  %d total_px  centroid (%.2f, %.2f)"
                    % (len(nearby), total_px, pt[0], pt[1]))
    return pt



def classify_local_geometry(
    grid, width, height, cx, cy, radius_cells,
    occupied_thresh=50, min_island_pixels=2, wall_margin_cells=4,
):
    """Classify the local obstacle geometry around (cx, cy) in grid coords.

    Returns (n_islands, total_pixels, max_island_pixels, aspect_ratio).
    - n_islands: number of valid (non-wall) islands within radius
    - total_pixels: sum of all valid island pixels
    - max_island_pixels: size of the largest island
    - aspect_ratio: bounding-box width/height of the largest island (>1 = wide)
    """
    x_lo = max(0, cx - radius_cells)
    x_hi = min(width, cx + radius_cells + 1)
    y_lo = max(0, cy - radius_cells)
    y_hi = min(height, cy + radius_cells + 1)

    visited = set()
    islands = []

    for iy in range(y_lo, y_hi):
        for ix in range(x_lo, x_hi):
            if (ix, iy) in visited:
                continue
            val = grid[iy * width + ix]
            if val < occupied_thresh:
                visited.add((ix, iy))
                continue
            island_cells = []
            queue = deque()
            queue.append((ix, iy))
            visited.add((ix, iy))
            touches_border = False
            while queue:
                px, py = queue.popleft()
                island_cells.append((px, py))
                if px <= 0 or px >= width - 1 or py <= 0 or py >= height - 1:
                    touches_border = True
                for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                    nx, ny = px + dx, py + dy
                    if x_lo <= nx < x_hi and y_lo <= ny < y_hi and (nx, ny) not in visited:
                        visited.add((nx, ny))
                        if grid[ny * width + nx] >= occupied_thresh:
                            queue.append((nx, ny))
            if len(island_cells) < min_island_pixels or touches_border:
                continue
            n = len(island_cells)
            agx = sum(p[0] for p in island_cells) / n
            agy = sum(p[1] for p in island_cells) / n
            if (agx < wall_margin_cells or agx >= width - wall_margin_cells or
                    agy < wall_margin_cells or agy >= height - wall_margin_cells):
                continue
            xs = [p[0] for p in island_cells]
            ys = [p[1] for p in island_cells]
            bbox_w = max(xs) - min(xs) + 1
            bbox_h = max(ys) - min(ys) + 1
            aspect = max(bbox_w, bbox_h) / max(min(bbox_w, bbox_h), 1)
            islands.append((n, aspect))

    if not islands:
        return 0, 0, 0, 1.0

    n_islands = len(islands)
    total_px = sum(i[0] for i in islands)
    max_px = max(i[0] for i in islands)
    max_aspect = max(i[1] for i in islands)
    return n_islands, total_px, max_px, max_aspect


def check_geometry_consistency(
    label, n_islands, total_pixels, max_island_px, aspect_ratio,
    person_max_islands=2, person_max_total_px=20,
    bench_min_total_px=8, bench_min_islands_or_wide=True,
):
    """Return (compatible, reason) for the detector label vs local geometry.

    compatible=True  -> observation is allowed
    compatible=False -> observation should be rejected
    """
    if label == "person":
        if n_islands > person_max_islands and total_pixels > person_max_total_px:
            return False, (
                "person obs but %d islands, %d px (bench-like geometry)"
                % (n_islands, total_pixels))
        if max_island_px > person_max_total_px and aspect_ratio > 2.5:
            return False, (
                "person obs but largest island %d px, aspect %.1f (wide/bench-like)"
                % (max_island_px, aspect_ratio))
        return True, "person-compatible (%d islands, %d px)" % (n_islands, total_pixels)

    if label == "bench":
        if n_islands == 1 and total_pixels < bench_min_total_px and aspect_ratio < 1.5:
            return False, (
                "bench obs but only 1 small compact island (%d px, aspect %.1f, person-like)"
                % (total_pixels, aspect_ratio))
        return True, "bench-compatible (%d islands, %d px, aspect %.1f)" % (
            n_islands, total_pixels, aspect_ratio)

    return True, "no geometry check for class '%s'" % label


class SemanticMapMemoryNode(Node):

    def __init__(self):
        super().__init__("semantic_map_memory_node")

        self.declare_parameter("input_topic", "/semantic_memory_node/objects")
        self.declare_parameter("output_topic", "/semantic_memory_markers")
        self.declare_parameter("landmark_objects_topic", "/semantic_map_memory_node/landmark_objects")
        self.declare_parameter("state_topic", "")
        self.declare_parameter("semantic_targets_file", "")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("tf_timeout", 0.3)
        self.declare_parameter("merge_distance", 0.8)
        self.declare_parameter("candidate_merge_distance", 0.8)
        self.declare_parameter("min_observations", 3)
        self.declare_parameter("search_radius_cells", 15)
        self.declare_parameter("sphere_radius", 0.18)
        self.declare_parameter("text_offset_z", 0.45)
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("candidate_timeout", 45.0)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("min_island_pixels", 2)
        self.declare_parameter("wall_boundary_margin_m", 0.20)
        self.declare_parameter("bench_cluster_radius_cells", 10)
        self.declare_parameter("max_observation_range_m", 4.0)
        self.declare_parameter("person_max_range_m", 2.5)
        self.declare_parameter("bench_max_range_m", 4.0)
        self.declare_parameter("stop_sign_max_range_m", 1.8)
        self.declare_parameter("person_min_observations", 3)
        self.declare_parameter("bench_min_observations", 4)
        self.declare_parameter("stop_sign_min_observations", 5)
        self.declare_parameter("geometry_check_enabled", True)
        self.declare_parameter("geometry_check_radius_cells", 12)
        self.declare_parameter("person_max_islands", 2)
        self.declare_parameter("person_max_total_px", 20)
        self.declare_parameter("bench_min_total_px", 8)
        self.declare_parameter("cross_class_mutex_enabled", True)
        self.declare_parameter("cross_class_mutex_distance_m", 0.6)
        self.declare_parameter("mutex_min_observation_count", 3)
        self.declare_parameter("excluded_classes", ["person"])
        self.declare_parameter("allow_lidar_only_landmarks", False)

        in_topic        = self.get_parameter("input_topic").value
        out_topic       = self.get_parameter("output_topic").value
        state_topic     = self.get_parameter("state_topic").value
        targets_file    = self.get_parameter("semantic_targets_file").value
        map_topic       = self.get_parameter("map_topic").value
        self._frame     = self.get_parameter("target_frame").value
        self._tf_tout   = self.get_parameter("tf_timeout").value
        self._merge_d   = self.get_parameter("merge_distance").value
        self._cand_d    = self.get_parameter("candidate_merge_distance").value
        self._min_obs   = self.get_parameter("min_observations").value
        self._search_r  = self.get_parameter("search_radius_cells").value
        self._sphere_r  = self.get_parameter("sphere_radius").value
        self._text_z    = self.get_parameter("text_offset_z").value
        pub_rate        = self.get_parameter("publish_rate").value
        self._cand_tout = self.get_parameter("candidate_timeout").value
        self._occ_thresh = self.get_parameter("occupied_threshold").value
        self._min_island = self.get_parameter("min_island_pixels").value
        self._wall_margin_m = self.get_parameter("wall_boundary_margin_m").value
        self._bench_cluster_r = self.get_parameter("bench_cluster_radius_cells").value
        self._max_obs_range = self.get_parameter("max_observation_range_m").value

        self._class_max_range = {
            "person": self.get_parameter("person_max_range_m").value,
            "bench": self.get_parameter("bench_max_range_m").value,
            "stop sign": self.get_parameter("stop_sign_max_range_m").value,
        }
        self._class_min_obs = {
            "person": self.get_parameter("person_min_observations").value,
            "bench": self.get_parameter("bench_min_observations").value,
            "stop sign": self.get_parameter("stop_sign_min_observations").value,
        }

        self._geom_enabled = self.get_parameter("geometry_check_enabled").value
        self._geom_radius = self.get_parameter("geometry_check_radius_cells").value
        self._person_max_islands = self.get_parameter("person_max_islands").value
        self._person_max_px = self.get_parameter("person_max_total_px").value
        self._bench_min_px = self.get_parameter("bench_min_total_px").value

        self._mutex_enabled = self.get_parameter("cross_class_mutex_enabled").value
        self._mutex_dist = self.get_parameter("cross_class_mutex_distance_m").value
        self._mutex_min_obs = self.get_parameter("mutex_min_observation_count").value
        self._allow_lidar_only = bool(
            self.get_parameter("allow_lidar_only_landmarks").value)
        try:
            excluded = self.get_parameter("excluded_classes").get_parameter_value().string_array_value
        except Exception:
            excluded = []
        self._excluded_classes = {str(item) for item in excluded if str(item)}
        self._sem2det, self._det2sem = self._load_target_mapping(targets_file)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        qos_rel = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        qos_map = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(Detection3DArray, in_topic, self._obs_cb, qos_rel)
        self.create_subscription(OccupancyGrid, map_topic, self._map_cb, qos_map)
        self._pub = self.create_publisher(MarkerArray, out_topic, qos_rel)

        lm_obj_topic = self.get_parameter("landmark_objects_topic").value
        self._lm_obj_pub = self.create_publisher(Detection3DArray, lm_obj_topic, qos_rel)
        self._state_pub = (
            self.create_publisher(SemanticMapState, state_topic, qos_rel)
            if state_topic else None
        )

        self._map_data = None
        self._map_width = 0
        self._map_height = 0
        self._map_res = 0.05
        self._map_ox = 0.0
        self._map_oy = 0.0

        self._candidates = []
        self._landmarks = {}
        self._next_seq = {}

        self.create_timer(1.0 / max(pub_rate, 0.1), self._publish_markers)
        self.create_timer(5.0, self._cleanup_candidates)

        self.get_logger().info(
            "SemanticMapMemoryNode ready  merge=%.2fm  cand=%.2fm  min_obs=%d  "
            "search_r=%d  wall_margin=%.2fm  min_island=%d  excluded=%s  "
            "lidar_only=%s  state_topic=%s"
            % (self._merge_d, self._cand_d, self._min_obs,
               self._search_r, self._wall_margin_m, self._min_island,
               ",".join(sorted(self._excluded_classes)) if self._excluded_classes else "none",
               self._allow_lidar_only,
               state_topic or "disabled"))

    def _load_target_mapping(self, targets_file):
        if not targets_file:
            try:
                targets_file = (
                    get_package_share_directory("tb3_frontier_exploration")
                    + "/config/semantic_targets.yaml"
                )
            except Exception:
                targets_file = ""
        if not targets_file:
            return {}, {}
        try:
            return load_target_mapping(targets_file)
        except Exception as exc:
            self.get_logger().warn("Failed to load semantic target mapping: %s" % exc)
            return {}, {}

    def _map_cb(self, msg):
        self._map_width = msg.info.width
        self._map_height = msg.info.height
        self._map_res = msg.info.resolution
        self._map_ox = msg.info.origin.position.x
        self._map_oy = msg.info.origin.position.y
        self._map_data = np.array(msg.data, dtype=np.int8)

    @staticmethod
    def _validation_rank(validation):
        if validation == "occupancy_grid_validated":
            return 3
        if validation == "occupancy_grid_pending":
            return 2
        if validation == "map_bounds_pending":
            return 1
        return 0

    def _stronger_validation(self, current, new):
        if self._validation_rank(new) > self._validation_rank(current):
            return new
        return current

    def _obs_cb(self, msg):
        if self._map_data is None and not self._allow_lidar_only:
            return

        wall_margin_cells = max(1, int(self._wall_margin_m / self._map_res))

        for det in msg.detections:
            if not det.results:
                continue
            label = det.results[0].hypothesis.class_id
            if label in self._excluded_classes:
                continue
            src = det.header.frame_id or msg.header.frame_id or "base_link"

            raw_x = det.bbox.center.position.x
            raw_y = det.bbox.center.position.y
            obs_range = math.hypot(raw_x, raw_y)

            class_max = self._class_max_range.get(label, self._max_obs_range)
            if obs_range > class_max:
                continue

            pt_in = PointStamped()
            pt_in.header.stamp = rclpy.time.Time(seconds=0).to_msg()
            pt_in.header.frame_id = src
            pt_in.point.x = raw_x
            pt_in.point.y = raw_y

            try:
                pt_out = self._tf_buffer.transform(
                    pt_in, self._frame,
                    timeout=Duration(seconds=self._tf_tout))
            except Exception:
                continue

            mx, my = pt_out.point.x, pt_out.point.y
            rx, ry = mx, my
            validation = "map_unavailable"

            if self._map_data is not None:
                gx, gy = world_to_grid(mx, my, self._map_ox, self._map_oy, self._map_res)

                if not (0 <= gx < self._map_width and 0 <= gy < self._map_height):
                    if not self._allow_lidar_only:
                        continue
                    validation = "map_bounds_pending"
                    self.get_logger().debug(
                        "[provisional] %s outside map bounds at (%.2f, %.2f)"
                        % (label, mx, my))
                elif label == "bench":
                    refined = find_bench_cluster_centroid(
                        self._map_data, self._map_width, self._map_height,
                        gx, gy, self._search_r,
                        self._map_ox, self._map_oy, self._map_res,
                        occupied_thresh=self._occ_thresh,
                        min_island_pixels=self._min_island,
                        wall_margin_cells=wall_margin_cells,
                        cluster_radius_cells=self._bench_cluster_r,
                        logger=self.get_logger())
                    if refined is None:
                        if not self._allow_lidar_only:
                            continue
                        validation = "occupancy_grid_pending"
                    else:
                        rx, ry = refined
                        validation = "occupancy_grid_validated"
                else:
                    refined = find_nearest_valid_island(
                        self._map_data, self._map_width, self._map_height,
                        gx, gy, self._search_r,
                        self._map_ox, self._map_oy, self._map_res,
                        occupied_thresh=self._occ_thresh,
                        min_island_pixels=self._min_island,
                        wall_margin_cells=wall_margin_cells)
                    if refined is None:
                        if not self._allow_lidar_only:
                            continue
                        validation = "occupancy_grid_pending"
                    else:
                        rx, ry = refined
                        validation = "occupancy_grid_validated"
            else:
                if not self._allow_lidar_only:
                    continue
                validation = "map_unavailable"

            if (
                validation == "occupancy_grid_validated"
                and self._geom_enabled
                and label in ("person", "bench")
            ):
                n_isl, tot_px, max_px, aspect = classify_local_geometry(
                    self._map_data, self._map_width, self._map_height,
                    gx, gy, self._geom_radius,
                    occupied_thresh=self._occ_thresh,
                    min_island_pixels=self._min_island,
                    wall_margin_cells=wall_margin_cells)
                compat, reason = check_geometry_consistency(
                    label, n_isl, tot_px, max_px, aspect,
                    person_max_islands=self._person_max_islands,
                    person_max_total_px=self._person_max_px,
                    bench_min_total_px=self._bench_min_px)
                if not compat:
                    self.get_logger().info("[reject] %s" % reason)
                    continue
                self.get_logger().debug("[geometry] %s" % reason)

            blocked, mutex_reason = self._check_cross_class_mutex(label, rx, ry)
            if blocked:
                self.get_logger().info("[mutex] %s" % mutex_reason)
                continue

            now = _time.time()

            matched_lm = self._find_landmark(label, rx, ry)
            if matched_lm is not None:
                self._update_landmark(matched_lm, rx, ry, now, validation)
                continue

            matched_cand = self._find_candidate(label, rx, ry)
            if matched_cand is not None:
                self._update_candidate(matched_cand, rx, ry, now, validation)
                class_min_obs = self._class_min_obs.get(label, self._min_obs)
                if matched_cand.obs_count >= class_min_obs:
                    self._promote(matched_cand)
                continue

            self._candidates.append(Candidate(
                semantic_class=label, x=rx, y=ry, obs_count=1,
                last_seen=now, validation=validation))

    def _check_cross_class_mutex(self, label, x, y):
        """Check if a different-class landmark or strong candidate is at the same location.

        Returns (blocked, reason) where blocked=True means the observation
        should be rejected because a stronger different-class entry exists nearby.
        """
        if not self._mutex_enabled:
            return False, ""
        if label not in ("person", "bench"):
            return False, ""

        best_blocker = None
        best_obs = 0

        for lm in self._landmarks.values():
            if lm.semantic_class == label:
                continue
            d = math.hypot(lm.x - x, lm.y - y)
            if d < self._mutex_dist and lm.observation_count > best_obs:
                best_blocker = lm
                best_obs = lm.observation_count

        for c in self._candidates:
            if c.semantic_class == label:
                continue
            d = math.hypot(c.x - x, c.y - y)
            if d < self._mutex_dist and c.obs_count >= self._mutex_min_obs and c.obs_count > best_obs:
                best_blocker = c
                best_obs = c.obs_count

        if best_blocker is not None:
            if isinstance(best_blocker, Landmark):
                reason = (
                    "%s obs at (%.2f,%.2f) blocked by %s landmark %s (n=%d, d=%.2fm)"
                    % (label, x, y, best_blocker.semantic_class,
                       best_blocker.landmark_id, best_blocker.observation_count,
                       math.hypot(best_blocker.x - x, best_blocker.y - y)))
            else:
                reason = (
                    "%s obs at (%.2f,%.2f) blocked by %s candidate (n=%d, d=%.2fm)"
                    % (label, x, y, best_blocker.semantic_class,
                       best_blocker.obs_count,
                       math.hypot(best_blocker.x - x, best_blocker.y - y)))
            return True, reason

        return False, ""

    def _find_landmark(self, label, x, y):
        best = None
        best_score = float("inf")
        for lm in self._landmarks.values():
            if lm.semantic_class != label:
                continue
            d = math.hypot(lm.x - x, lm.y - y)
            if d < self._merge_d:
                score = d / max(lm.observation_count, 1)
                if score < best_score:
                    best = lm
                    best_score = score
        return best

    def _find_candidate(self, label, x, y):
        best = None
        best_d = float("inf")
        for c in self._candidates:
            if c.semantic_class != label:
                continue
            d = math.hypot(c.x - x, c.y - y)
            if d < self._cand_d and d < best_d:
                best = c
                best_d = d
        return best

    def _update_landmark(self, lm, x, y, now, validation):
        n = lm.observation_count
        lm.x = (lm.x * n + x) / (n + 1)
        lm.y = (lm.y * n + y) / (n + 1)
        lm.observation_count = n + 1
        lm.last_seen = now
        lm.validation = self._stronger_validation(lm.validation, validation)
        if lm.observation_count % 50 == 0:
            self.get_logger().info(
                "[merge_landmark] %s  n=%d  pos=(%.2f, %.2f)  validation=%s"
                % (lm.landmark_id, lm.observation_count, lm.x, lm.y, lm.validation))

    def _update_candidate(self, c, x, y, now, validation):
        n = c.obs_count
        c.x = (c.x * n + x) / (n + 1)
        c.y = (c.y * n + y) / (n + 1)
        c.obs_count = n + 1
        c.last_seen = now
        c.validation = self._stronger_validation(c.validation, validation)

    def _promote(self, c):
        seq = self._next_seq.get(c.semantic_class, 0)
        self._next_seq[c.semantic_class] = seq + 1
        lid = "%s_%d" % (c.semantic_class, seq)
        lm = Landmark(
            landmark_id=lid, semantic_class=c.semantic_class,
            x=c.x, y=c.y,
            observation_count=c.obs_count, last_seen=c.last_seen,
            validation=c.validation)
        self._landmarks[lid] = lm
        self._candidates.remove(c)
        self.get_logger().info(
            "[new_landmark] %s  pos=(%.2f, %.2f)  after %d obs  validation=%s  total=%d"
            % (lid, lm.x, lm.y, c.obs_count, lm.validation, len(self._landmarks)))

    def _cleanup_candidates(self):
        now = _time.time()
        before = len(self._candidates)
        self._candidates = [
            c for c in self._candidates if (now - c.last_seen) < self._cand_tout]
        removed = before - len(self._candidates)
        if removed > 0:
            self.get_logger().info("[cleanup] Removed %d stale candidates" % removed)

    def _publish_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        mid = 0

        delete_all = Marker()
        delete_all.header.stamp = stamp
        delete_all.header.frame_id = self._frame
        delete_all.action = Marker.DELETEALL
        ma.markers.append(delete_all)

        for lm in self._landmarks.values():
            color = CLASS_COLORS.get(lm.semantic_class, DEFAULT_COLOR)
            s = Marker()
            s.header.stamp = stamp
            s.header.frame_id = self._frame
            s.ns = "semantic_landmarks"
            s.id = mid
            s.type = Marker.SPHERE
            s.action = Marker.ADD
            s.pose.position.x = lm.x
            s.pose.position.y = lm.y
            s.pose.orientation.w = 1.0
            s.scale.x = s.scale.y = s.scale.z = self._sphere_r * 2
            s.color = color
            ma.markers.append(s)
            mid += 1
            t = Marker()
            t.header.stamp = stamp
            t.header.frame_id = self._frame
            t.ns = "semantic_landmarks_text"
            t.id = mid
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = lm.x
            t.pose.position.y = lm.y
            t.pose.position.z = self._text_z
            t.pose.orientation.w = 1.0
            t.scale.z = 0.15
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text = "%s (n=%d)" % (lm.landmark_id, lm.observation_count)
            ma.markers.append(t)
            mid += 1
        self._pub.publish(ma)
        self._publish_landmark_objects()
        self._publish_semantic_map_state(stamp)


    def _publish_landmark_objects(self):
        """Publish persistent landmarks as Detection3DArray for semantic_query_node."""
        from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose
        msg = Detection3DArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame
        for lm in self._landmarks.values():
            det = Detection3D()
            det.header = msg.header
            det.id = lm.landmark_id
            det.bbox.center.position.x = lm.x
            det.bbox.center.position.y = lm.y
            det.bbox.center.position.z = 0.0
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = lm.semantic_class
            hyp.hypothesis.score = float(lm.observation_count)
            det.results.append(hyp)
            msg.detections.append(det)
        self._lm_obj_pub.publish(msg)

    def _publish_semantic_map_state(self, stamp):
        """Publish validated persistent landmarks as the native semantic map."""
        if self._state_pub is None:
            return

        state = SemanticMapState()
        state.header.stamp = stamp
        state.header.frame_id = self._frame
        state.source = "tb3_coordinator/semantic_map_memory"
        state.refinement_active = True

        for lm in self._landmarks.values():
            entity = SemanticEntity()
            entity.header = state.header
            entity.entity_id = lm.landmark_id
            entity.detector_label = lm.semantic_class
            entity.semantic_name = self._det2sem.get(lm.semantic_class, lm.semantic_class)
            entity.state = "active"
            entity.aliases = [lm.semantic_class]
            if entity.semantic_name and entity.semantic_name != lm.semantic_class:
                entity.aliases.append(entity.semantic_name)
            entity.provenance = [
                "semantic_memory",
                "lidar_grounded",
                lm.validation,
            ]
            entity.pose.position.x = float(lm.x)
            entity.pose.position.y = float(lm.y)
            entity.pose.position.z = 0.0
            entity.pose.orientation.w = 1.0
            covariance = [0.0] * 36
            covariance[0] = 0.20
            covariance[7] = 0.20
            covariance[14] = 0.05
            covariance[35] = 0.25
            entity.pose_covariance = covariance
            entity.extent.x = self._sphere_r * 2.0
            entity.extent.y = self._sphere_r * 2.0
            entity.extent.z = self._sphere_r * 2.0
            min_obs = max(1, int(self._class_min_obs.get(lm.semantic_class, self._min_obs)))
            entity.confidence = float(min(1.0, lm.observation_count / float(min_obs)))
            if lm.validation != "occupancy_grid_validated":
                entity.confidence = float(min(entity.confidence, 0.85))
            entity.observation_count = int(lm.observation_count)
            entity.last_seen.sec = int(lm.last_seen)
            entity.last_seen.nanosec = int((lm.last_seen - int(lm.last_seen)) * 1e9)
            state.entities.append(entity)

        self._state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMapMemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
