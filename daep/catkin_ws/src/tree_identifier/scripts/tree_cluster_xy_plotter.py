#!/usr/bin/env python

import os
import pickle
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Empty, Float32MultiArray, Int32MultiArray
from tree_identifier.msg import TreeDetectionArray

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle


class TreeClusterXYPlotter(object):
    def __init__(self):
        self.cluster_points_topic = rospy.get_param("~cluster_points_topic", "/tree_detector_cluster_points")
        self.cluster_labels_topic = rospy.get_param("~cluster_labels_topic", "/tree_detector_cluster_labels")
        self.use_raw_array_input = rospy.get_param("~use_raw_array_input", True)
        self.raw_array_topic = rospy.get_param("~raw_array_topic", "/tree_detections_full")
        self.raw_pose_topic = rospy.get_param("~raw_pose_topic", "/tree_detections")
        self.raw_radius_topic = rospy.get_param("~raw_radius_topic", "/tree_detection_radii")

        self.enable_map_overlay = rospy.get_param("~enable_map_overlay", True)
        self.map_pose_topic = rospy.get_param("~map_pose_topic", "/tree_map_poses")
        self.map_id_topic = rospy.get_param("~map_id_topic", "/tree_map_ids")
        self.map_candidate_pose_topic = rospy.get_param("~map_candidate_pose_topic", "/tree_map_candidates")
        self.map_candidate_id_topic = rospy.get_param("~map_candidate_id_topic", "/tree_map_candidate_ids")

        self.refresh_ms = rospy.get_param("~refresh_ms", 500)
        self.fixed_axes = rospy.get_param("~fixed_axes", True)
        self.x_min = rospy.get_param("~x_min", -10.0)
        self.x_max = rospy.get_param("~x_max", 10.0)
        self.y_min = rospy.get_param("~y_min", -8.0)
        self.y_max = rospy.get_param("~y_max", 7.0)
        self.tick_step = rospy.get_param("~tick_step", 5.0)

        self.cluster_point_size = rospy.get_param("~cluster_point_size", 12.0)
        self.cluster_alpha = rospy.get_param("~cluster_alpha", 0.62)
        self.show_tree_labels = rospy.get_param("~show_tree_labels", True)
        self.show_map_labels = rospy.get_param("~show_map_labels", True)
        self.show_metrics = rospy.get_param("~show_metrics", True)
        self.max_labels = rospy.get_param("~max_labels", 90)

        self.log_throttle_sec = rospy.get_param("~log_throttle_sec", 2.0)
        self.accumulate_history = rospy.get_param("~accumulate_history", True)
        self.plot_history = rospy.get_param("~plot_history", True)
        self.max_history_points = rospy.get_param("~max_history_points", 250000)
        self.history_label_stride = rospy.get_param("~history_label_stride", 1000)

        self.snapshot_topic = rospy.get_param("~snapshot_topic", "~save_snapshot")
        self.snapshot_dir = self._normalize_path(
            rospy.get_param("~snapshot_dir", "/tmp/tree_cluster_snapshots")
        )
        self.snapshot_prefix = rospy.get_param("~snapshot_prefix", "tree_cluster_state")
        self.auto_snapshot_period_sec = rospy.get_param("~auto_snapshot_period_sec", 0.0)
        self.save_png = rospy.get_param("~save_png", True)
        self.save_on_shutdown = rospy.get_param("~save_on_shutdown", True)

        self.lock = threading.Lock()

        self.cluster_points = []
        self.cluster_labels = []
        self.history_cluster_points = []
        self.history_cluster_labels = []
        self.history_frame_idx = 0
        self.last_history_signature = None
        self.raw_points = []
        self.raw_radii = []
        self.raw_confidences = []
        self.raw_fit_errors = []
        self.raw_cluster_labels = []

        self.map_points = []
        self.map_ids = []
        self.map_candidate_points = []
        self.map_candidate_ids = []

        self.last_frame_id = "world"
        self.last_log_time = 0.0
        self.pending_snapshot_reasons = []

        self.circle_artists = []
        self.text_artists = []
        self.metrics_artist = None

        self.palette = plt.cm.get_cmap("tab20", 20)

        rospy.Subscriber(self.cluster_points_topic, PoseArray, self.cluster_points_cb, queue_size=1)
        rospy.Subscriber(self.cluster_labels_topic, Int32MultiArray, self.cluster_labels_cb, queue_size=1)
        if self.use_raw_array_input:
            rospy.Subscriber(self.raw_array_topic, TreeDetectionArray, self.raw_array_cb, queue_size=1)
        else:
            rospy.Subscriber(self.raw_pose_topic, PoseArray, self.raw_pose_cb, queue_size=1)
            rospy.Subscriber(self.raw_radius_topic, Float32MultiArray, self.raw_radius_cb, queue_size=1)

        if self.enable_map_overlay:
            rospy.Subscriber(self.map_pose_topic, PoseArray, self.map_pose_cb, queue_size=1)
            rospy.Subscriber(self.map_id_topic, Int32MultiArray, self.map_id_cb, queue_size=1)
            rospy.Subscriber(self.map_candidate_pose_topic, PoseArray, self.map_candidate_pose_cb, queue_size=1)
            rospy.Subscriber(self.map_candidate_id_topic, Int32MultiArray, self.map_candidate_id_cb, queue_size=1)

        rospy.Subscriber(self.snapshot_topic, Empty, self.snapshot_cmd_cb, queue_size=1)

        if self.auto_snapshot_period_sec > 0.0:
            rospy.Timer(rospy.Duration(self.auto_snapshot_period_sec), self.auto_snapshot_timer_cb)
        rospy.on_shutdown(self.on_shutdown)

        self.fig, self.ax = plt.subplots(figsize=(9, 9))
        self.ax.set_title("Tree Clusters + Targets (Realtime)")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True, alpha=0.35)

        self.cluster_scatter = self.ax.scatter(
            [],
            [],
            s=self.cluster_point_size,
            c="#6d6d6d",
            alpha=self.cluster_alpha,
            linewidths=0,
            label="cluster_points",
        )
        self.raw_scatter = self.ax.scatter(
            [], [], c="#c11f1f", s=88, marker="x", linewidths=1.7, label="detected_target"
        )
        self.map_scatter = self.ax.scatter(
            [], [], c="#1f8f3a", s=88, marker="x", linewidths=1.8, label="map_confirmed"
        )
        self.map_candidate_scatter = self.ax.scatter(
            [], [], c="#e67e22", s=62, marker="^", alpha=0.85, label="map_candidate"
        )

        self.ax.legend(loc="upper right")
        self.ax.set_aspect("equal", "box")

        if self.show_metrics:
            self.metrics_artist = self.ax.text(
                0.01,
                0.99,
                "",
                transform=self.ax.transAxes,
                ha="left",
                va="top",
                fontsize=9,
                bbox=dict(boxstyle="round,pad=0.25", facecolor="white", alpha=0.68),
            )

        self._configure_axes()

        rospy.loginfo(
            "tree_cluster_xy_plotter: clusters=%s labels=%s use_raw_array=%s raw_array=%s raw=%s radii=%s",
            self.cluster_points_topic,
            self.cluster_labels_topic,
            str(self.use_raw_array_input),
            self.raw_array_topic,
            self.raw_pose_topic,
            self.raw_radius_topic,
        )
        rospy.loginfo("tree_cluster_xy_plotter snapshot topic: %s", self.snapshot_topic)
        rospy.loginfo(
            "tree_cluster_xy_plotter snapshot dir: %s (auto=%.2fs png=%s)",
            self.snapshot_dir,
            self.auto_snapshot_period_sec,
            str(self.save_png),
        )
        rospy.loginfo(
            "tree_cluster_xy_plotter history: accumulate=%s plot_history=%s max_history_points=%d",
            str(self.accumulate_history),
            str(self.plot_history),
            int(self.max_history_points),
        )

        self.anim = FuncAnimation(self.fig, self.update_plot, interval=self.refresh_ms)

    @staticmethod
    def _normalize_path(path):
        if not path:
            return ""
        return os.path.abspath(os.path.expanduser(path))

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

    def _configure_axes(self):
        if not self.fixed_axes:
            return
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
    def _align_labels(n_points, labels):
        out = [-1] * n_points
        m = min(n_points, len(labels))
        for i in range(m):
            out[i] = int(labels[i])
        return out

    def _cluster_colors(self, labels):
        colors = []
        for lb in labels:
            if lb < 0:
                rgba = [0.25, 0.25, 0.25, self.cluster_alpha * 0.65]
            else:
                rgba = list(self.palette(int(lb) % 20))
                rgba[3] = self.cluster_alpha
            colors.append(rgba)
        return np.asarray(colors, dtype=float)

    @staticmethod
    def _history_signature(cluster_points, aligned_labels):
        if not cluster_points:
            return None
        first = cluster_points[0]
        last = cluster_points[-1]
        lb0 = -1
        lbn = -1
        if aligned_labels:
            lb0 = int(aligned_labels[0])
            lbn = int(aligned_labels[-1])
        return (
            len(cluster_points),
            len(aligned_labels),
            round(float(first[0]), 3),
            round(float(first[1]), 3),
            round(float(last[0]), 3),
            round(float(last[1]), 3),
            lb0,
            lbn,
        )

    def _ingest_history(self, cluster_points, aligned_labels):
        if not self.accumulate_history:
            return
        if not cluster_points:
            return
        if len(cluster_points) != len(aligned_labels):
            return

        sig = self._history_signature(cluster_points, aligned_labels)
        if sig is None:
            return

        with self.lock:
            if sig == self.last_history_signature:
                return
            self.last_history_signature = sig

            base = self.history_frame_idx * self.history_label_stride
            mapped_labels = []
            for lb in aligned_labels:
                if lb < 0:
                    mapped_labels.append(-1)
                else:
                    mapped_labels.append(base + int(lb))

            self.history_cluster_points.extend(cluster_points)
            self.history_cluster_labels.extend(mapped_labels)
            self.history_frame_idx += 1

            if self.max_history_points > 0 and len(self.history_cluster_points) > self.max_history_points:
                overflow = len(self.history_cluster_points) - self.max_history_points
                self.history_cluster_points = self.history_cluster_points[overflow:]
                self.history_cluster_labels = self.history_cluster_labels[overflow:]

    def _cluster_view_for_plot(self, current_points, current_labels):
        if self.accumulate_history and self.plot_history:
            with self.lock:
                if self.history_cluster_points:
                    return (
                        list(self.history_cluster_points),
                        list(self.history_cluster_labels),
                        "history",
                    )
        return list(current_points), list(current_labels), "current"

    def cluster_points_cb(self, msg):
        pts = []
        for p in msg.poses:
            pts.append((p.position.x, p.position.y, p.position.z))

        with self.lock:
            self.cluster_points = pts
            if msg.header.frame_id:
                self.last_frame_id = msg.header.frame_id

    def cluster_labels_cb(self, msg):
        with self.lock:
            self.cluster_labels = list(msg.data)

    def raw_array_cb(self, msg):
        pts = []
        radii = []
        confidences = []
        fit_errors = []
        cluster_labels = []

        for det in msg.detections:
            pts.append((det.pose.position.x, det.pose.position.y, det.pose.position.z))
            r = float(det.radius)
            if r <= 0.0 and float(det.diameter) > 0.0:
                r = 0.5 * float(det.diameter)
            radii.append(max(r, 0.0))
            confidences.append(float(det.confidence))
            fit_errors.append(float(det.fit_error))
            cluster_labels.append(int(det.cluster_label))

        with self.lock:
            self.raw_points = pts
            self.raw_radii = radii
            self.raw_confidences = confidences
            self.raw_fit_errors = fit_errors
            self.raw_cluster_labels = cluster_labels
            if msg.header.frame_id:
                self.last_frame_id = msg.header.frame_id

    def raw_pose_cb(self, msg):
        pts = []
        for p in msg.poses:
            pts.append((p.position.x, p.position.y, p.position.z))

        with self.lock:
            self.raw_points = pts
            self.raw_confidences = [0.0] * len(pts)
            self.raw_fit_errors = [0.0] * len(pts)
            self.raw_cluster_labels = [-1] * len(pts)
            if msg.header.frame_id:
                self.last_frame_id = msg.header.frame_id

    def raw_radius_cb(self, msg):
        with self.lock:
            self.raw_radii = list(msg.data)

    def map_pose_cb(self, msg):
        pts = []
        for p in msg.poses:
            pts.append((p.position.x, p.position.y, p.position.z))
        with self.lock:
            self.map_points = pts

    def map_id_cb(self, msg):
        with self.lock:
            self.map_ids = list(msg.data)

    def map_candidate_pose_cb(self, msg):
        pts = []
        for p in msg.poses:
            pts.append((p.position.x, p.position.y, p.position.z))
        with self.lock:
            self.map_candidate_points = pts

    def map_candidate_id_cb(self, msg):
        with self.lock:
            self.map_candidate_ids = list(msg.data)

    def snapshot_cmd_cb(self, _):
        self._queue_snapshot("manual")

    def auto_snapshot_timer_cb(self, _):
        self._queue_snapshot("auto")

    def _queue_snapshot(self, reason):
        with self.lock:
            self.pending_snapshot_reasons.append(str(reason))

    @staticmethod
    def _ensure_parent_dir(path):
        parent = os.path.dirname(path)
        if parent and (not os.path.exists(parent)):
            os.makedirs(parent)

    def _timestamp_tag(self):
        now = rospy.Time.now()
        sec = int(now.to_sec())
        msec = int((now.to_nsec() % 1000000000) / 1000000)
        return "%s_%03d" % (time.strftime("%Y%m%d_%H%M%S", time.localtime(sec)), msec)

    def _build_snapshot_payload(self, reason):
        with self.lock:
            cluster_points = list(self.cluster_points)
            cluster_labels = list(self.cluster_labels)
            history_cluster_points = list(self.history_cluster_points)
            history_cluster_labels = list(self.history_cluster_labels)
            raw_points = list(self.raw_points)
            raw_radii = list(self.raw_radii)
            raw_confidences = list(self.raw_confidences)
            raw_fit_errors = list(self.raw_fit_errors)
            raw_cluster_labels = list(self.raw_cluster_labels)
            map_points = list(self.map_points)
            map_ids = list(self.map_ids)
            map_candidate_points = list(self.map_candidate_points)
            map_candidate_ids = list(self.map_candidate_ids)
            frame_id = self.last_frame_id

        current_aligned_labels = self._align_labels(len(cluster_points), cluster_labels)
        payload_cluster_points = list(cluster_points)
        payload_cluster_labels = list(current_aligned_labels)
        cluster_source = "current"

        if self.accumulate_history and self.plot_history and history_cluster_points:
            payload_cluster_points = history_cluster_points
            payload_cluster_labels = history_cluster_labels
            cluster_source = "history"

        if payload_cluster_points:
            points_xyz = np.asarray(payload_cluster_points, dtype=float)
            points_xy = points_xyz[:, :2]
        else:
            points_xyz = np.empty((0, 3), dtype=float)
            points_xy = np.empty((0, 2), dtype=float)

        if cluster_points:
            points_xyz_current = np.asarray(cluster_points, dtype=float)
            points_xy_current = points_xyz_current[:, :2]
        else:
            points_xyz_current = np.empty((0, 3), dtype=float)
            points_xy_current = np.empty((0, 2), dtype=float)

        detected_trees = []
        for i, p in enumerate(raw_points):
            r = 0.0
            if i < len(raw_radii):
                r = float(raw_radii[i])
            conf = 0.0
            if i < len(raw_confidences):
                conf = float(raw_confidences[i])
            fit_error = 0.0
            if i < len(raw_fit_errors):
                fit_error = float(raw_fit_errors[i])
            cluster_label = -1
            if i < len(raw_cluster_labels):
                cluster_label = int(raw_cluster_labels[i])
            detected_trees.append(
                {
                    "id": i + 1,
                    "position": (float(p[0]), float(p[1]), float(p[2])),
                    "diameter": max(2.0 * r, 0.0),
                    "confidence": conf,
                    "fit_error": fit_error,
                    "cluster_label": cluster_label,
                }
            )

        map_confirmed = []
        for i, p in enumerate(map_points):
            map_id = i + 1
            if i < len(map_ids):
                map_id = map_ids[i]
            map_confirmed.append(
                {
                    "map_id": int(map_id),
                    "position": (float(p[0]), float(p[1]), float(p[2])),
                }
            )

        map_candidates = []
        for i, p in enumerate(map_candidate_points):
            map_id = i + 1
            if i < len(map_candidate_ids):
                map_id = map_candidate_ids[i]
            map_candidates.append(
                {
                    "map_id": int(map_id),
                    "position": (float(p[0]), float(p[1]), float(p[2])),
                }
            )

        return {
            "points_xy": points_xy,
            "points_xyz": points_xyz,
            "labels": np.asarray(payload_cluster_labels, dtype=np.int32),
            "points_xy_current": points_xy_current,
            "points_xyz_current": points_xyz_current,
            "labels_current": np.asarray(current_aligned_labels, dtype=np.int32),
            "detected_trees": detected_trees,
            "map_confirmed": map_confirmed,
            "map_candidates": map_candidates,
            "meta": {
                "saved_at_sec": rospy.Time.now().to_sec(),
                "reason": str(reason),
                "frame_id": frame_id if frame_id else "world",
                "cluster_source": cluster_source,
                "cluster_count": len(set([lb for lb in payload_cluster_labels if lb >= 0])),
                "cluster_points": len(payload_cluster_points),
                "current_cluster_count": len(set([lb for lb in current_aligned_labels if lb >= 0])),
                "current_cluster_points": len(cluster_points),
                "history_cluster_points": len(history_cluster_points),
                "detected_target_count": len(raw_points),
                "detected_target_mean_confidence": (
                    float(np.mean(raw_confidences)) if raw_confidences else 0.0
                ),
                "map_confirmed_count": len(map_confirmed),
                "map_candidate_count": len(map_candidates),
            },
        }

    def _save_snapshot(self, reason):
        payload = self._build_snapshot_payload(reason)
        tag = self._timestamp_tag()
        base = "%s_%s" % (self.snapshot_prefix, tag)
        pkl_path = os.path.join(self.snapshot_dir, base + ".pkl")
        png_path = os.path.join(self.snapshot_dir, base + ".png")

        try:
            self._ensure_parent_dir(pkl_path)
            with open(pkl_path, "wb") as f:
                pickle.dump(payload, f, protocol=pickle.HIGHEST_PROTOCOL)

            png_saved = False
            if self.save_png:
                self._ensure_parent_dir(png_path)
                self.fig.savefig(png_path, dpi=180, bbox_inches="tight")
                png_saved = True

            rospy.loginfo(
                "tree_cluster_xy_plotter snapshot(%s): pkl=%s png=%s",
                reason,
                pkl_path,
                png_path if png_saved else "disabled",
            )
        except Exception as exc:
            rospy.logwarn("tree_cluster_xy_plotter snapshot failed: %s", exc)

    def _drain_snapshot_requests(self):
        with self.lock:
            reasons = list(self.pending_snapshot_reasons)
            self.pending_snapshot_reasons = []

        for reason in reasons:
            self._save_snapshot(reason)

    def _add_tree_labels(self, raw_points, raw_radii, raw_confidences):
        if (not self.show_tree_labels) or self.max_labels <= 0:
            return

        n = min(len(raw_points), self.max_labels)
        for i in range(n):
            x = raw_points[i][0]
            y = raw_points[i][1]
            diameter = 0.0
            if i < len(raw_radii):
                diameter = max(2.0 * float(raw_radii[i]), 0.0)
            conf = 0.0
            if i < len(raw_confidences):
                conf = max(float(raw_confidences[i]), 0.0)
            txt = self.ax.text(
                x + 0.12,
                y + 0.12,
                "det_%d d=%.2fm c=%.2f" % (i + 1, diameter, conf),
                fontsize=8.2,
                color="#8a1313",
            )
            self.text_artists.append(txt)

    def _add_map_labels(self, map_points, map_ids, map_candidate_points, map_candidate_ids):
        if (not self.show_map_labels) or self.max_labels <= 0:
            return

        n_map = min(len(map_points), self.max_labels)
        for i in range(n_map):
            x = map_points[i][0]
            y = map_points[i][1]
            label_id = map_ids[i] if i < len(map_ids) else (i + 1)
            txt = self.ax.text(x + 0.12, y + 0.12, "map_%d" % label_id, fontsize=8.0, color="#145a32")
            self.text_artists.append(txt)

        n_cand = min(len(map_candidate_points), self.max_labels)
        for i in range(n_cand):
            x = map_candidate_points[i][0]
            y = map_candidate_points[i][1]
            label_id = map_candidate_ids[i] if i < len(map_candidate_ids) else (i + 1)
            txt = self.ax.text(x + 0.12, y + 0.12, "cand_%d" % label_id, fontsize=8.0, color="#9a4d00")
            self.text_artists.append(txt)

    def _update_metrics(self, n_cluster_points, n_clusters, n_targets, n_map, n_cand):
        if self.metrics_artist is None:
            return

        lines = [
            "cluster_points=%d" % n_cluster_points,
            "clusters=%d" % n_clusters,
            "detected_targets=%d" % n_targets,
            "map_confirmed=%d" % n_map,
            "map_candidates=%d" % n_cand,
        ]
        if self.accumulate_history:
            with self.lock:
                current_pts = len(self.cluster_points)
                history_pts = len(self.history_cluster_points)
            lines.append("current_cluster_points=%d" % current_pts)
            lines.append("history_cluster_points=%d" % history_pts)
        self.metrics_artist.set_text("\n".join(lines))

    def _update_log(self, n_cluster_points, n_clusters, n_targets, n_map, n_cand, cluster_source):
        now = rospy.Time.now().to_sec()
        if (now - self.last_log_time) < self.log_throttle_sec:
            return
        self.last_log_time = now

        rospy.loginfo(
            "tree_cluster_xy_plotter live: cluster_points=%d clusters=%d targets=%d map=%d cand=%d source=%s",
            n_cluster_points,
            n_clusters,
            n_targets,
            n_map,
            n_cand,
            cluster_source,
        )

    def update_plot(self, _):
        with self.lock:
            cluster_points = list(self.cluster_points)
            cluster_labels = list(self.cluster_labels)
            raw_points = list(self.raw_points)
            raw_radii = list(self.raw_radii)
            raw_confidences = list(self.raw_confidences)
            map_points = list(self.map_points)
            map_ids = list(self.map_ids)
            map_candidate_points = list(self.map_candidate_points)
            map_candidate_ids = list(self.map_candidate_ids)

        current_aligned_labels = self._align_labels(len(cluster_points), cluster_labels)
        self._ingest_history(cluster_points, current_aligned_labels)
        plot_cluster_points, plot_cluster_labels, cluster_source = self._cluster_view_for_plot(
            cluster_points, current_aligned_labels
        )

        cluster_xy = [(p[0], p[1]) for p in plot_cluster_points]
        raw_xy = [(p[0], p[1]) for p in raw_points]
        map_xy = [(p[0], p[1]) for p in map_points]
        map_candidate_xy = [(p[0], p[1]) for p in map_candidate_points]

        self._set_offsets(self.cluster_scatter, cluster_xy)
        if cluster_xy:
            self.cluster_scatter.set_facecolors(self._cluster_colors(plot_cluster_labels))
        else:
            self.cluster_scatter.set_facecolors(np.empty((0, 4), dtype=float))

        self._set_offsets(self.raw_scatter, raw_xy)
        if self.enable_map_overlay:
            self._set_offsets(self.map_scatter, map_xy)
            self._set_offsets(self.map_candidate_scatter, map_candidate_xy)
        else:
            self._set_offsets(self.map_scatter, [])
            self._set_offsets(self.map_candidate_scatter, [])

        for c in self.circle_artists:
            c.remove()
        self.circle_artists = []

        for i, p in enumerate(raw_points):
            if i >= len(raw_radii):
                continue
            r = max(float(raw_radii[i]), 0.0)
            if r <= 0.0:
                continue
            circ = Circle((p[0], p[1]), r, fill=False, linewidth=1.5, linestyle="--", color="#b1173b", alpha=0.9)
            self.ax.add_patch(circ)
            self.circle_artists.append(circ)

        for txt in self.text_artists:
            txt.remove()
        self.text_artists = []

        self._add_tree_labels(raw_points, raw_radii, raw_confidences)
        if self.enable_map_overlay:
            self._add_map_labels(map_points, map_ids, map_candidate_points, map_candidate_ids)

        n_clusters = len(set([lb for lb in plot_cluster_labels if lb >= 0]))
        self._update_metrics(
            n_cluster_points=len(plot_cluster_points),
            n_clusters=n_clusters,
            n_targets=len(raw_points),
            n_map=len(map_points),
            n_cand=len(map_candidate_points),
        )
        self._update_log(
            n_cluster_points=len(plot_cluster_points),
            n_clusters=n_clusters,
            n_targets=len(raw_points),
            n_map=len(map_points),
            n_cand=len(map_candidate_points),
            cluster_source=cluster_source,
        )

        if not self.fixed_axes:
            all_pts = list(cluster_xy)
            all_pts.extend(raw_xy)
            if self.enable_map_overlay:
                all_pts.extend(map_xy)
                all_pts.extend(map_candidate_xy)
            if all_pts:
                xs = [p[0] for p in all_pts]
                ys = [p[1] for p in all_pts]
                margin = 2.0
                self.ax.set_xlim(min(xs) - margin, max(xs) + margin)
                self.ax.set_ylim(min(ys) - margin, max(ys) + margin)
        else:
            self._configure_axes()

        self._drain_snapshot_requests()

    def on_shutdown(self):
        if self.save_on_shutdown:
            self._save_snapshot("shutdown")


if __name__ == "__main__":
    rospy.init_node("tree_cluster_xy_plotter")
    node = TreeClusterXYPlotter()
    plt.show()
