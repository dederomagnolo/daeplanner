#!/usr/bin/env python

import threading

import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32MultiArray

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class TreeXYPlotter(object):
    def __init__(self):
        self.pose_topic = rospy.get_param("~pose_topic", "/tree_detections_tracked")
        self.id_topic = rospy.get_param("~id_topic", "/tree_detection_ids")
        self.refresh_ms = rospy.get_param("~refresh_ms", 500)
        self.show_xy_in_label = rospy.get_param("~show_xy_in_label", True)
        self.fixed_axes = rospy.get_param("~fixed_axes", True)
        self.x_min = rospy.get_param("~x_min", -25.0)
        self.x_max = rospy.get_param("~x_max", 25.0)
        self.y_min = rospy.get_param("~y_min", -28.0)
        self.y_max = rospy.get_param("~y_max", 20.0)
        self.tick_step = rospy.get_param("~tick_step", 5.0)
        self.log_coords = rospy.get_param("~log_coords", True)
        self.log_throttle_sec = rospy.get_param("~log_throttle_sec", 2.0)
        self.accumulate_unique = rospy.get_param("~accumulate_unique", True)
        self.cache_by_id = rospy.get_param("~cache_by_id", True)
        self.dedup_dist = rospy.get_param("~dedup_dist", 1.0)
        self.update_cached_position = rospy.get_param("~update_cached_position", False)
        self.log_only_on_new = rospy.get_param("~log_only_on_new", False)

        self.lock = threading.Lock()
        self.points = []  # list of (x, y)
        self.ids = []
        self.cached_trees = []  # list of {"id": int|None, "x": float, "y": float, "hits": int}
        self.id_to_cache_idx = {}  # track_id -> cached_trees index
        self.text_artists = []
        self.last_log_signature = None
        self.last_log_time = 0.0

        rospy.Subscriber(self.pose_topic, PoseArray, self.pose_cb, queue_size=1)
        rospy.Subscriber(self.id_topic, Int32MultiArray, self.id_cb, queue_size=1)

        self.fig, self.ax = plt.subplots(figsize=(7, 7))
        self.ax.set_title("Tree Detections (X,Y)")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.grid(True)
        self.scatter = self.ax.scatter([], [], c="green", s=60)
        self._configure_axes()

        rospy.loginfo("tree_xy_plotter listening on %s and %s", self.pose_topic, self.id_topic)

        self.anim = FuncAnimation(self.fig, self.update_plot, interval=self.refresh_ms)

    def id_cb(self, msg):
        with self.lock:
            self.ids = list(msg.data)

    def pose_cb(self, msg):
        pts = []
        for p in msg.poses:
            pts.append((p.position.x, p.position.y))
        with self.lock:
            self.points = pts

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
            return [], []

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

    def _maybe_log(self, labels, pts, new_entries):
        if not self.log_coords:
            return

        if new_entries:
            msg = []
            for idx, x, y in new_entries:
                label = labels[idx] if idx < len(labels) else "tree_new"
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
        for i, (x, y) in enumerate(pts):
            label = labels[i] if i < len(labels) else "p_%d" % i
            entries.append("%s:(%.2f,%.2f)" % (label, x, y))

        signature = tuple(entries)
        if signature == self.last_log_signature:
            return

        self.last_log_signature = signature
        self.last_log_time = now
        if entries:
            rospy.loginfo("tree_xy_plotter detections: %s", " | ".join(entries))
        else:
            rospy.loginfo("tree_xy_plotter detections: none")

    def update_plot(self, _):
        with self.lock:
            pts = list(self.points)
            ids = list(self.ids)

        if self.accumulate_unique:
            display_pts, display_labels, new_entries = self._update_cache(pts, ids)
        else:
            has_ids = (len(ids) == len(pts))
            display_pts = list(pts)
            display_labels = []
            for i in range(len(display_pts)):
                if has_ids:
                    display_labels.append("tree_%d" % ids[i])
                else:
                    display_labels.append("p_%d" % i)
            new_entries = []

        self._maybe_log(display_labels, display_pts, new_entries)

        for txt in self.text_artists:
            txt.remove()
        self.text_artists = []

        if not display_pts:
            self.scatter.set_offsets([])
            self._configure_axes()
            return

        self.scatter.set_offsets(display_pts)

        for i, (x, y) in enumerate(display_pts):
            label = display_labels[i] if i < len(display_labels) else "p_%d" % i
            if self.show_xy_in_label:
                label = "%s (%.2f, %.2f)" % (label, x, y)

            txt = self.ax.text(x + 0.12, y + 0.12, label, fontsize=9)
            self.text_artists.append(txt)

        if not self.fixed_axes:
            xs = [p[0] for p in display_pts]
            ys = [p[1] for p in display_pts]
            margin = 2.0
            self.ax.set_xlim(min(xs) - margin, max(xs) + margin)
            self.ax.set_ylim(min(ys) - margin, max(ys) + margin)
        else:
            self._configure_axes()


if __name__ == "__main__":
    rospy.init_node("tree_xy_plotter")
    node = TreeXYPlotter()
    plt.show()
