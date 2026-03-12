#!/usr/bin/env python

import threading

import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32MultiArray

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np


class TreeXYPlotter(object):
    def __init__(self):
        self.pose_topic = rospy.get_param("~pose_topic", "/tree_detections_tracked")
        self.id_topic = rospy.get_param("~id_topic", "/tree_detection_ids")

        self.refresh_ms = rospy.get_param("~refresh_ms", 500)
        self.show_xy_in_label = rospy.get_param("~show_xy_in_label", True)
        self.fixed_axes = rospy.get_param("~fixed_axes", True)
        self.x_min = rospy.get_param("~x_min", -10.0)
        self.x_max = rospy.get_param("~x_max", 10.0)
        self.y_min = rospy.get_param("~y_min", -8.0)
        self.y_max = rospy.get_param("~y_max", 7.0)
        self.tick_step = rospy.get_param("~tick_step", 5.0)

        self.log_coords = rospy.get_param("~log_coords", True)
        self.log_throttle_sec = rospy.get_param("~log_throttle_sec", 2.0)
        self.log_only_on_new = rospy.get_param("~log_only_on_new", False)

        self.accumulate_unique = rospy.get_param("~accumulate_unique", True)
        self.cache_by_id = rospy.get_param("~cache_by_id", True)
        self.dedup_dist = rospy.get_param("~dedup_dist", 1.0)
        self.update_cached_position = rospy.get_param("~update_cached_position", False)

        self.enable_map_overlay = rospy.get_param("~enable_map_overlay", True)
        self.map_pose_topic = rospy.get_param("~map_pose_topic", "/tree_map_poses")
        self.map_id_topic = rospy.get_param("~map_id_topic", "/tree_map_ids")
        self.map_candidate_pose_topic = rospy.get_param("~map_candidate_pose_topic", "/tree_map_candidates")
        self.map_candidate_id_topic = rospy.get_param("~map_candidate_id_topic", "/tree_map_candidate_ids")

        self.show_live_labels = rospy.get_param("~show_live_labels", True)
        self.show_map_labels = rospy.get_param("~show_map_labels", True)
        self.show_metrics = rospy.get_param("~show_metrics", True)
        self.max_labels = rospy.get_param("~max_labels", 70)

        self.lock = threading.Lock()

        self.live_points = []
        self.live_ids = []

        # Backward-compatible cache mode.
        self.cached_trees = []  # list of {"id": int|None, "x": float, "y": float, "hits": int}
        self.id_to_cache_idx = {}  # tracker_id -> cached_trees index

        self.map_points = []
        self.map_ids = []
        self.map_candidate_points = []
        self.map_candidate_ids = []

        self.text_artists = []
        self.metrics_artist = None

        self.last_log_signature = None
        self.last_log_time = 0.0

        rospy.Subscriber(self.pose_topic, PoseArray, self.live_pose_cb, queue_size=1)
        rospy.Subscriber(self.id_topic, Int32MultiArray, self.live_id_cb, queue_size=1)

        if self.enable_map_overlay:
            rospy.Subscriber(self.map_pose_topic, PoseArray, self.map_pose_cb, queue_size=1)
            rospy.Subscriber(self.map_id_topic, Int32MultiArray, self.map_id_cb, queue_size=1)
            rospy.Subscriber(self.map_candidate_pose_topic, PoseArray, self.map_candidate_pose_cb, queue_size=1)
            rospy.Subscriber(self.map_candidate_id_topic, Int32MultiArray, self.map_candidate_id_cb, queue_size=1)

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_title("Tree Detections and Fused Map (X,Y)")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True)

        self.live_scatter = self.ax.scatter([], [], c="#2c7fb8", s=48, alpha=0.9, label="live")
        self.map_scatter = self.ax.scatter([], [], c="#1f8f3a", s=88, marker="x", linewidths=1.8, label="map")
        self.map_candidate_scatter = self.ax.scatter(
            [], [], c="#e67e22", s=62, marker="^", alpha=0.85, label="candidate"
        )

        if self.enable_map_overlay:
            self.ax.legend(loc="upper right")

        if self.show_metrics:
            self.metrics_artist = self.ax.text(
                0.01,
                0.99,
                "",
                transform=self.ax.transAxes,
                ha="left",
                va="top",
                fontsize=9,
                bbox=dict(boxstyle="round,pad=0.25", facecolor="white", alpha=0.65),
            )

        self._configure_axes()

        rospy.loginfo(
            "tree_xy_plotter listening live on %s and %s | map_overlay=%s",
            self.pose_topic,
            self.id_topic,
            str(self.enable_map_overlay),
        )
        if self.enable_map_overlay:
            rospy.loginfo(
                "tree_xy_plotter map topics: %s %s %s %s",
                self.map_pose_topic,
                self.map_id_topic,
                self.map_candidate_pose_topic,
                self.map_candidate_id_topic,
            )

        self.anim = FuncAnimation(self.fig, self.update_plot, interval=self.refresh_ms)

    def live_id_cb(self, msg):
        with self.lock:
            self.live_ids = list(msg.data)

    def live_pose_cb(self, msg):
        pts = []
        for p in msg.poses:
            pts.append((p.position.x, p.position.y))
        with self.lock:
            self.live_points = pts

    def map_id_cb(self, msg):
        with self.lock:
            self.map_ids = list(msg.data)

    def map_pose_cb(self, msg):
        pts = []
        for p in msg.poses:
            pts.append((p.position.x, p.position.y))
        with self.lock:
            self.map_points = pts

    def map_candidate_id_cb(self, msg):
        with self.lock:
            self.map_candidate_ids = list(msg.data)

    def map_candidate_pose_cb(self, msg):
        pts = []
        for p in msg.poses:
            pts.append((p.position.x, p.position.y))
        with self.lock:
            self.map_candidate_points = pts

    def _configure_axes(self):
        if self.fixed_axes:
            self.ax.set_xlim(self.x_min, self.x_max)
            self.ax.set_ylim(self.y_min, self.y_max)
            if self.tick_step > 0:
                xs = []
                ys = []
                x = self.x_min
                while x <= self.x_max + 1e-9:
                    xs.append(x)
                    x += self.tick_step
                y = self.y_min
                while y <= self.y_max + 1e-9:
                    ys.append(y)
                    y += self.tick_step
                self.ax.set_xticks(xs)
                self.ax.set_yticks(ys)

    @staticmethod
    def _dist_sq_xy(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return dx * dx + dy * dy

    @staticmethod
    def _set_offsets(scatter, pts):
        if pts:
            arr = np.asarray(pts, dtype=float)
            if arr.ndim == 1:
                if arr.size == 2:
                    arr = arr.reshape((1, 2))
                else:
                    arr = np.empty((0, 2), dtype=float)
            elif arr.ndim != 2 or arr.shape[1] != 2:
                arr = np.empty((0, 2), dtype=float)
        else:
            arr = np.empty((0, 2), dtype=float)
        scatter.set_offsets(arr)

    def _find_nearby_cache(self, x, y):
        max_d2 = self.dedup_dist * self.dedup_dist
        best_idx = None
        best_d2 = None
        for idx, tr in enumerate(self.cached_trees):
            d2 = self._dist_sq_xy((x, y), (tr["x"], tr["y"]))
            if d2 > max_d2:
                continue
            if best_d2 is None or d2 < best_d2:
                best_d2 = d2
                best_idx = idx
        return best_idx

    def _cache_label(self, idx, tree):
        if tree["id"] is not None:
            return "tree_%d" % tree["id"]
        return "tree_cached_%d" % (idx + 1)

    def _update_cache(self, pts, ids):
        if not self.accumulate_unique:
            return [], [], []

        has_ids = (len(ids) == len(pts))
        new_entries = []

        for i, (x, y) in enumerate(pts):
            tree_id = ids[i] if has_ids else None
            cache_idx = None

            if self.cache_by_id and tree_id is not None and tree_id in self.id_to_cache_idx:
                cache_idx = self.id_to_cache_idx[tree_id]

            if cache_idx is None:
                cache_idx = self._find_nearby_cache(x, y)

            if cache_idx is None:
                tree = {"id": tree_id, "x": x, "y": y, "hits": 1}
                self.cached_trees.append(tree)
                cache_idx = len(self.cached_trees) - 1
                if self.cache_by_id and tree_id is not None:
                    self.id_to_cache_idx[tree_id] = cache_idx
                new_entries.append((cache_idx, tree["x"], tree["y"]))
            else:
                tree = self.cached_trees[cache_idx]
                tree["hits"] += 1
                if self.cache_by_id and tree_id is not None:
                    if tree["id"] is None:
                        tree["id"] = tree_id
                    self.id_to_cache_idx[tree_id] = cache_idx

                if self.update_cached_position:
                    n = float(tree["hits"])
                    tree["x"] = tree["x"] + (x - tree["x"]) / n
                    tree["y"] = tree["y"] + (y - tree["y"]) / n

        display_pts = []
        display_labels = []
        for idx, tree in enumerate(self.cached_trees):
            display_pts.append((tree["x"], tree["y"]))
            display_labels.append(self._cache_label(idx, tree))

        return display_pts, display_labels, new_entries

    @staticmethod
    def _labels_from_ids(prefix, ids, n):
        labels = []
        has_ids = (len(ids) == n)
        for i in range(n):
            if has_ids:
                labels.append("%s_%d" % (prefix, ids[i]))
            else:
                labels.append("%s_%d" % (prefix, i + 1))
        return labels

    def _maybe_log(self, live_labels, live_pts, new_entries, n_map, n_candidates):
        if not self.log_coords:
            return

        if new_entries:
            msg = []
            for idx, x, y in new_entries:
                label = live_labels[idx] if idx < len(live_labels) else "tree_new"
                msg.append("%s:(%.2f,%.2f)" % (label, x, y))
            rospy.loginfo(
                "tree_xy_plotter new detections: %s | total_cached=%d",
                " | ".join(msg),
                len(self.cached_trees),
            )
            if self.log_only_on_new:
                return

        now = rospy.Time.now().to_sec()
        if (now - self.last_log_time) < self.log_throttle_sec:
            return

        entries = []
        for i, (x, y) in enumerate(live_pts):
            label = live_labels[i] if i < len(live_labels) else "p_%d" % i
            entries.append("%s:(%.2f,%.2f)" % (label, x, y))

        signature = tuple(entries + ["map=%d" % n_map, "cand=%d" % n_candidates])
        if signature == self.last_log_signature:
            return

        self.last_log_signature = signature
        self.last_log_time = now

        if entries:
            rospy.loginfo(
                "tree_xy_plotter live: %s | map_confirmed=%d | map_candidates=%d",
                " | ".join(entries),
                n_map,
                n_candidates,
            )
        else:
            rospy.loginfo(
                "tree_xy_plotter live: none | map_confirmed=%d | map_candidates=%d",
                n_map,
                n_candidates,
            )

    def _add_labels(self, pts, labels, color):
        if self.max_labels <= 0:
            return
        n = min(len(pts), len(labels), self.max_labels)
        for i in range(n):
            x, y = pts[i]
            label = labels[i]
            if self.show_xy_in_label:
                label = "%s (%.2f, %.2f)" % (label, x, y)
            txt = self.ax.text(x + 0.12, y + 0.12, label, fontsize=8.5, color=color)
            self.text_artists.append(txt)

    def _update_metrics(self, frame_count, live_display_count, map_count, candidate_count):
        if self.metrics_artist is None:
            return

        lines = [
            "frame=%d" % frame_count,
            "live_display=%d" % live_display_count,
            "map_confirmed=%d" % map_count,
            "map_candidates=%d" % candidate_count,
        ]
        if self.accumulate_unique:
            lines.append("cache_size=%d" % len(self.cached_trees))
        self.metrics_artist.set_text("\n".join(lines))

    def update_plot(self, _):
        with self.lock:
            live_pts = list(self.live_points)
            live_ids = list(self.live_ids)
            map_pts = list(self.map_points)
            map_ids = list(self.map_ids)
            map_candidate_pts = list(self.map_candidate_points)
            map_candidate_ids = list(self.map_candidate_ids)

        if self.accumulate_unique:
            display_live_pts, display_live_labels, new_entries = self._update_cache(live_pts, live_ids)
        else:
            has_ids = (len(live_ids) == len(live_pts))
            display_live_pts = list(live_pts)
            display_live_labels = []
            for i in range(len(display_live_pts)):
                if has_ids:
                    display_live_labels.append("tree_%d" % live_ids[i])
                else:
                    display_live_labels.append("p_%d" % i)
            new_entries = []

        map_labels = self._labels_from_ids("map", map_ids, len(map_pts))
        map_candidate_labels = self._labels_from_ids("cand", map_candidate_ids, len(map_candidate_pts))

        self._maybe_log(
            display_live_labels,
            display_live_pts,
            new_entries,
            len(map_pts),
            len(map_candidate_pts),
        )

        for txt in self.text_artists:
            txt.remove()
        self.text_artists = []

        self._set_offsets(self.live_scatter, display_live_pts)

        if self.enable_map_overlay:
            self._set_offsets(self.map_scatter, map_pts)
            self._set_offsets(self.map_candidate_scatter, map_candidate_pts)
        else:
            self._set_offsets(self.map_scatter, [])
            self._set_offsets(self.map_candidate_scatter, [])

        if self.show_live_labels:
            self._add_labels(display_live_pts, display_live_labels, "#0b3c5d")

        if self.enable_map_overlay and self.show_map_labels:
            self._add_labels(map_pts, map_labels, "#145a32")
            self._add_labels(map_candidate_pts, map_candidate_labels, "#9a4d00")

        self._update_metrics(
            frame_count=len(live_pts),
            live_display_count=len(display_live_pts),
            map_count=len(map_pts),
            candidate_count=len(map_candidate_pts),
        )

        if not self.fixed_axes:
            all_pts = list(display_live_pts)
            if self.enable_map_overlay:
                all_pts.extend(map_pts)
                all_pts.extend(map_candidate_pts)
            if all_pts:
                xs = [p[0] for p in all_pts]
                ys = [p[1] for p in all_pts]
                margin = 2.0
                self.ax.set_xlim(min(xs) - margin, max(xs) + margin)
                self.ax.set_ylim(min(ys) - margin, max(ys) + margin)
        else:
            self._configure_axes()


if __name__ == "__main__":
    rospy.init_node("tree_xy_plotter")
    node = TreeXYPlotter()
    plt.show()
