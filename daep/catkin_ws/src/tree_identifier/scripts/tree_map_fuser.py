#!/usr/bin/env python

import csv
import json
import math
import os
import threading

import rospy
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from tree_identifier.msg import TreeDetection, TreeDetectionArray


class TreeMapFuser(object):
    """
    Long-term tree map builder with diameter propagation.

    Input:
      - preferred: tracked TreeDetectionArray
      - legacy: PoseArray + aligned ID list

    Output:
      - confirmed/candidate PoseArray + IDs (legacy)
      - confirmed/candidate TreeDetectionArray (full contract)
      - MarkerArray for RViz
      - periodic CSV/JSON exports (position + diameter + confidence)
    """

    def __init__(self):
        self.use_array_input = rospy.get_param("~use_array_input", True)
        self.array_topic = rospy.get_param("~array_topic", "/tree_detections_tracked_full")
        self.pose_topic = rospy.get_param("~pose_topic", "/tree_detections_tracked")
        self.id_topic = rospy.get_param("~id_topic", "/tree_detection_ids")

        self.output_topic = rospy.get_param("~output_topic", "/tree_map_poses")
        self.output_id_topic = rospy.get_param("~output_id_topic", "/tree_map_ids")
        self.output_array_topic = rospy.get_param("~output_array_topic", "/tree_map_full")

        self.candidate_topic = rospy.get_param("~candidate_topic", "/tree_map_candidates")
        self.candidate_id_topic = rospy.get_param("~candidate_id_topic", "/tree_map_candidate_ids")
        self.candidate_array_topic = rospy.get_param("~candidate_array_topic", "/tree_map_candidate_full")
        self.marker_topic = rospy.get_param("~marker_topic", "/tree_map_markers")

        self.association_dist = rospy.get_param("~association_dist", 0.70)
        self.id_reuse_max_dist = rospy.get_param("~id_reuse_max_dist", 0.95)
        self.max_diameter_delta = rospy.get_param("~max_diameter_delta", 0.45)
        self.diameter_assoc_weight = rospy.get_param("~diameter_assoc_weight", 0.35)

        self.min_confirmations = rospy.get_param("~min_confirmations", 4)
        self.max_std_xy = rospy.get_param("~max_std_xy", 0.70)
        self.max_std_diameter = rospy.get_param("~max_std_diameter", 0.28)
        self.min_confirmed_confidence = rospy.get_param("~min_confirmed_confidence", 0.0)

        self.prune_unconfirmed_after_sec = rospy.get_param("~prune_unconfirmed_after_sec", 60.0)
        self.split_large_source_ids = rospy.get_param("~split_large_source_ids", 18)
        self.split_std_xy_threshold = rospy.get_param("~split_std_xy_threshold", 0.45)
        self.split_reject_dist = rospy.get_param("~split_reject_dist", 0.75)

        self.publish_rate_hz = rospy.get_param("~publish_rate_hz", 2.0)
        self.publish_candidates = rospy.get_param("~publish_candidates", True)
        self.log_throttle_sec = rospy.get_param("~log_throttle_sec", 2.0)

        self.default_height = rospy.get_param("~default_height", 2.2)
        self.default_radius = rospy.get_param("~default_radius", 0.35)
        self.ground_truth_diameter_m = float(rospy.get_param("~ground_truth_diameter_m", 0.30))

        self.auto_export_period_sec = rospy.get_param("~auto_export_period_sec", 5.0)
        self.csv_output_path = self._normalize_path(
            rospy.get_param("~csv_output_path", "/tmp/tree_map_final.csv")
        )
        self.json_output_path = self._normalize_path(
            rospy.get_param("~json_output_path", "/tmp/tree_map_final.json")
        )

        self.lock = threading.Lock()
        self.latest_ids = []
        self.next_map_id = 1
        self.entries = {}  # map_id -> state
        self.source_to_map = {}  # tracker_id -> map_id

        self.last_header = None
        self.last_log_time = 0.0
        self.published_marker_ids = set()

        self.pose_pub = rospy.Publisher(self.output_topic, PoseArray, queue_size=1)
        self.id_pub = rospy.Publisher(self.output_id_topic, Int32MultiArray, queue_size=1)
        self.array_pub = rospy.Publisher(self.output_array_topic, TreeDetectionArray, queue_size=1)

        self.candidate_pose_pub = rospy.Publisher(self.candidate_topic, PoseArray, queue_size=1)
        self.candidate_id_pub = rospy.Publisher(self.candidate_id_topic, Int32MultiArray, queue_size=1)
        self.candidate_array_pub = rospy.Publisher(self.candidate_array_topic, TreeDetectionArray, queue_size=1)

        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)

        if self.use_array_input:
            rospy.Subscriber(self.array_topic, TreeDetectionArray, self.array_cb, queue_size=1)
            rospy.loginfo("tree_map_fuser listening on %s (full mode)", self.array_topic)
        else:
            rospy.Subscriber(self.id_topic, Int32MultiArray, self.id_cb, queue_size=1)
            rospy.Subscriber(self.pose_topic, PoseArray, self.pose_cb, queue_size=1)
            rospy.loginfo("tree_map_fuser listening on %s + %s (legacy mode)", self.pose_topic, self.id_topic)

        publish_period = 0.5
        if self.publish_rate_hz > 0.0:
            publish_period = 1.0 / self.publish_rate_hz
        rospy.Timer(rospy.Duration(publish_period), self.publish_timer_cb)

        if self.auto_export_period_sec > 0.0:
            rospy.Timer(rospy.Duration(self.auto_export_period_sec), self.export_timer_cb)

        rospy.on_shutdown(self.on_shutdown)

    @staticmethod
    def _normalize_path(path):
        if not path:
            return ""
        return os.path.abspath(os.path.expanduser(path))

    @staticmethod
    def _dist_sq_xy(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return dx * dx + dy * dy

    @staticmethod
    def _linear_sum_assignment(cost):
        n = len(cost)
        if n == 0:
            return []
        m = len(cost[0])
        if m == 0:
            return [-1] * n

        transposed = False
        work = cost
        n_work = n
        m_work = m
        if n > m:
            transposed = True
            n_work = m
            m_work = n
            work = [[cost[j][i] for j in range(n)] for i in range(m)]

        inf = 1e18
        u = [0.0] * (n_work + 1)
        v = [0.0] * (m_work + 1)
        p = [0] * (m_work + 1)
        way = [0] * (m_work + 1)

        for i in range(1, n_work + 1):
            p[0] = i
            j0 = 0
            minv = [inf] * (m_work + 1)
            used = [False] * (m_work + 1)
            while True:
                used[j0] = True
                i0 = p[j0]
                delta = inf
                j1 = 0
                for j in range(1, m_work + 1):
                    if used[j]:
                        continue
                    cur = work[i0 - 1][j - 1] - u[i0] - v[j]
                    if cur < minv[j]:
                        minv[j] = cur
                        way[j] = j0
                    if minv[j] < delta:
                        delta = minv[j]
                        j1 = j
                for j in range(0, m_work + 1):
                    if used[j]:
                        u[p[j]] += delta
                        v[j] -= delta
                    else:
                        minv[j] -= delta
                j0 = j1
                if p[j0] == 0:
                    break
            while True:
                j1 = way[j0]
                p[j0] = p[j1]
                j0 = j1
                if j0 == 0:
                    break

        assignment_work = [-1] * n_work
        for j in range(1, m_work + 1):
            if p[j] != 0:
                assignment_work[p[j] - 1] = j - 1

        if not transposed:
            return assignment_work

        assignment = [-1] * n
        for col_idx in range(len(assignment_work)):
            row_idx = assignment_work[col_idx]
            if row_idx >= 0:
                assignment[row_idx] = col_idx
        return assignment

    def id_cb(self, msg):
        with self.lock:
            self.latest_ids = list(msg.data)

    def _entry_std_xy(self, entry):
        if entry["hits"] < 2:
            return 0.0
        var_x = entry["m2_x"] / float(entry["hits"] - 1)
        var_y = entry["m2_y"] / float(entry["hits"] - 1)
        var_x = max(var_x, 0.0)
        var_y = max(var_y, 0.0)
        return math.sqrt(var_x + var_y)

    def _entry_std_diameter(self, entry):
        if entry["hits"] < 2:
            return 0.0
        var_d = entry["m2_d"] / float(entry["hits"] - 1)
        var_d = max(var_d, 0.0)
        return math.sqrt(var_d)

    def _entry_confidence(self, entry):
        if entry["hits"] <= 0:
            return 0.0
        return float(entry["conf_sum"]) / float(entry["hits"])

    def _is_confirmed(self, entry):
        if entry["hits"] < self.min_confirmations:
            return False
        if self.max_std_xy > 0.0 and self._entry_std_xy(entry) > self.max_std_xy:
            return False
        if self.max_std_diameter > 0.0 and self._entry_std_diameter(entry) > self.max_std_diameter:
            return False
        if self.min_confirmed_confidence > 0.0 and self._entry_confidence(entry) < self.min_confirmed_confidence:
            return False
        return True

    def _entry_radius(self, entry):
        if entry["diameter"] > 0.0:
            return max(0.5 * entry["diameter"], 0.0)
        return max(self.default_radius, 0.0)

    def _make_detection(self, x, y, z, source_id, diameter, confidence):
        d = float(max(diameter, 0.0))
        c = float(max(confidence, 0.0))
        return {
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "source_id": source_id,
            "diameter": d,
            "confidence": c,
        }

    def _entry_cost(self, det, map_id, entry):
        d2 = self._dist_sq_xy((det["x"], det["y"]), (entry["x"], entry["y"]))
        dist = math.sqrt(d2)

        src = det.get("source_id")
        if src is not None and src in self.source_to_map and self.source_to_map[src] == map_id:
            if dist <= self.id_reuse_max_dist:
                return 0.1 * dist
            return None

        if dist > self.association_dist:
            return None

        det_d = max(float(det.get("diameter", 0.0)), 0.0)
        ent_d = max(float(entry.get("diameter", 0.0)), 0.0)
        if det_d > 0.0 and ent_d > 0.0:
            delta_d = abs(det_d - ent_d)
            if delta_d > self.max_diameter_delta:
                return None
        else:
            delta_d = 0.0

        if entry.get("suspect_merge", False) and dist > self.split_reject_dist:
            return None

        return dist + self.diameter_assoc_weight * delta_d

    def _create_entry(self, det, now):
        map_id = self.next_map_id
        self.next_map_id += 1

        entry = {
            "x": det["x"],
            "y": det["y"],
            "z": det["z"],
            "diameter": max(det["diameter"], 0.0),
            "hits": 1,
            "m2_x": 0.0,
            "m2_y": 0.0,
            "m2_z": 0.0,
            "m2_d": 0.0,
            "conf_sum": det["confidence"],
            "last_seen": now,
            "source_ids": set(),
            "suspect_merge": False,
        }

        source_id = det.get("source_id")
        if source_id is not None:
            entry["source_ids"].add(int(source_id))
            self.source_to_map[int(source_id)] = map_id

        self.entries[map_id] = entry
        return map_id

    def _update_entry(self, map_id, det, now):
        entry = self.entries[map_id]

        entry["hits"] += 1
        n = float(entry["hits"])

        dx = det["x"] - entry["x"]
        entry["x"] = entry["x"] + dx / n
        entry["m2_x"] = entry["m2_x"] + dx * (det["x"] - entry["x"])

        dy = det["y"] - entry["y"]
        entry["y"] = entry["y"] + dy / n
        entry["m2_y"] = entry["m2_y"] + dy * (det["y"] - entry["y"])

        dz = det["z"] - entry["z"]
        entry["z"] = entry["z"] + dz / n
        entry["m2_z"] = entry["m2_z"] + dz * (det["z"] - entry["z"])

        det_d = max(float(det.get("diameter", 0.0)), 0.0)
        if det_d > 0.0:
            dd = det_d - entry["diameter"]
            entry["diameter"] = entry["diameter"] + dd / n
            entry["m2_d"] = entry["m2_d"] + dd * (det_d - entry["diameter"])

        entry["conf_sum"] += float(max(det.get("confidence", 0.0), 0.0))
        entry["last_seen"] = now

        source_id = det.get("source_id")
        if source_id is not None:
            source_id = int(source_id)
            entry["source_ids"].add(source_id)
            self.source_to_map[source_id] = map_id

        if (
            self.split_large_source_ids > 0
            and len(entry["source_ids"]) >= self.split_large_source_ids
            and self._entry_std_xy(entry) >= self.split_std_xy_threshold
        ):
            entry["suspect_merge"] = True

    def _associate_batch(self, detections):
        map_ids = sorted(self.entries.keys())
        det_to_map = {}
        invalid_cost = 1e6

        if detections and map_ids:
            cost = []
            for det in detections:
                row = []
                for map_id in map_ids:
                    c = self._entry_cost(det, map_id, self.entries[map_id])
                    row.append(invalid_cost if c is None else float(c))
                cost.append(row)

            assign = self._linear_sum_assignment(cost)
            for det_idx, col in enumerate(assign):
                if col < 0:
                    continue
                if cost[det_idx][col] >= invalid_cost * 0.5:
                    continue
                det_to_map[det_idx] = map_ids[col]

        return det_to_map

    def _prune_unconfirmed(self, now):
        if self.prune_unconfirmed_after_sec <= 0.0:
            return

        to_remove = []
        for map_id, entry in self.entries.items():
            if self._is_confirmed(entry):
                continue
            age = now - entry["last_seen"]
            if age > self.prune_unconfirmed_after_sec:
                to_remove.append(map_id)

        for map_id in to_remove:
            source_ids = list(self.entries[map_id]["source_ids"])
            for source_id in source_ids:
                if self.source_to_map.get(source_id) == map_id:
                    del self.source_to_map[source_id]
            del self.entries[map_id]

    def _consume_detections(self, header, detections):
        now = rospy.Time.now().to_sec()

        with self.lock:
            self.last_header = header
            new_count = 0
            upd_count = 0

            det_to_map = self._associate_batch(detections)
            for i, det in enumerate(detections):
                map_id = det_to_map.get(i)
                if map_id is None:
                    self._create_entry(det, now)
                    new_count += 1
                else:
                    self._update_entry(map_id, det, now)
                    upd_count += 1

            self._prune_unconfirmed(now)

            now_log = rospy.Time.now().to_sec()
            if (now_log - self.last_log_time) >= self.log_throttle_sec:
                confirmed = 0
                candidates = 0
                suspects = 0
                for entry in self.entries.values():
                    if entry.get("suspect_merge", False):
                        suspects += 1
                    if self._is_confirmed(entry):
                        confirmed += 1
                    else:
                        candidates += 1
                rospy.loginfo(
                    "tree_map_fuser: frame=%d new=%d upd=%d map_confirmed=%d candidates=%d suspect=%d total=%d",
                    len(detections),
                    new_count,
                    upd_count,
                    confirmed,
                    candidates,
                    suspects,
                    len(self.entries),
                )
                self.last_log_time = now_log

    def array_cb(self, msg):
        detections = []
        for det in msg.detections:
            source_id = None
            if int(det.id) > 0:
                source_id = int(det.id)

            diameter = float(det.diameter)
            if diameter <= 0.0 and float(det.radius) > 0.0:
                diameter = 2.0 * float(det.radius)

            detections.append(
                self._make_detection(
                    det.pose.position.x,
                    det.pose.position.y,
                    det.pose.position.z,
                    source_id,
                    diameter,
                    float(det.confidence),
                )
            )

        self._consume_detections(msg.header, detections)

    def pose_cb(self, msg):
        detections = []
        with self.lock:
            ids = list(self.latest_ids)

        has_ids = len(ids) == len(msg.poses)
        for i, pose in enumerate(msg.poses):
            source_id = ids[i] if has_ids else None
            detections.append(
                self._make_detection(
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    source_id,
                    2.0 * self.default_radius,
                    0.0,
                )
            )

        self._consume_detections(msg.header, detections)

    def _build_marker(self, header, map_id, entry, confirmed):
        std_xy = self._entry_std_xy(entry)
        std_d = self._entry_std_diameter(entry)
        conf = self._entry_confidence(entry)
        radius = self._entry_radius(entry)

        cyl = Marker()
        cyl.header = header
        cyl.ns = "tree_map"
        cyl.id = map_id
        cyl.type = Marker.CYLINDER
        cyl.action = Marker.ADD
        cyl.pose.position.x = entry["x"]
        cyl.pose.position.y = entry["y"]
        cyl.pose.position.z = entry["z"] + self.default_height * 0.5
        cyl.pose.orientation.w = 1.0
        cyl.scale.z = self.default_height
        cyl.scale.x = max(2.0 * radius, 2.0 * (std_xy + 0.10))
        cyl.scale.y = max(2.0 * radius, 2.0 * (std_xy + 0.10))
        cyl.color.a = 0.5
        if confirmed:
            cyl.color.r = 0.1
            cyl.color.g = 0.75
            cyl.color.b = 0.2
        else:
            cyl.color.r = 0.95
            cyl.color.g = 0.65
            cyl.color.b = 0.12
        if entry.get("suspect_merge", False):
            cyl.color.r = 0.82
            cyl.color.g = 0.15
            cyl.color.b = 0.15
        cyl.lifetime = rospy.Duration(0.0)

        txt = Marker()
        txt.header = header
        txt.ns = "tree_map_text"
        txt.id = 100000 + map_id
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = entry["x"]
        txt.pose.position.y = entry["y"]
        txt.pose.position.z = entry["z"] + self.default_height + 0.35
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.30
        txt.color.r = 1.0
        txt.color.g = 1.0
        txt.color.b = 1.0
        txt.color.a = 0.95
        txt.text = "map_%d h=%d d=%.2f s=%.2f sd=%.2f c=%.2f" % (
            map_id,
            entry["hits"],
            max(entry["diameter"], 0.0),
            std_xy,
            std_d,
            conf,
        )
        txt.lifetime = rospy.Duration(0.0)

        return cyl, txt

    def _entry_to_detection(self, map_id, entry):
        det = TreeDetection()
        det.id = int(map_id)
        det.pose.position.x = entry["x"]
        det.pose.position.y = entry["y"]
        det.pose.position.z = entry["z"]
        det.pose.orientation.w = 1.0
        det.radius = float(self._entry_radius(entry))
        det.diameter = float(max(entry["diameter"], 0.0))
        det.cluster_label = -1
        det.cluster_points = int(entry["hits"])
        det.fit_error = float(self._entry_std_xy(entry))
        det.confidence = float(self._entry_confidence(entry))
        return det

    def publish_timer_cb(self, _):
        with self.lock:
            entries = sorted(self.entries.items(), key=lambda kv: kv[0])
            header = self.last_header

        if header is None:
            header = PoseArray().header
            header.frame_id = "world"
        header.stamp = rospy.Time.now()

        confirmed_pa = PoseArray()
        confirmed_pa.header = header
        confirmed_ids = Int32MultiArray()
        confirmed_full = TreeDetectionArray()
        confirmed_full.header = header

        candidate_pa = PoseArray()
        candidate_pa.header = header
        candidate_ids = Int32MultiArray()
        candidate_full = TreeDetectionArray()
        candidate_full.header = header

        markers = MarkerArray()
        current_marker_ids = set()

        for map_id, entry in entries:
            confirmed = self._is_confirmed(entry)

            pose = Pose()
            pose.position.x = entry["x"]
            pose.position.y = entry["y"]
            pose.position.z = entry["z"]
            pose.orientation.w = 1.0

            full_det = self._entry_to_detection(map_id, entry)

            if confirmed:
                confirmed_pa.poses.append(pose)
                confirmed_ids.data.append(map_id)
                confirmed_full.detections.append(full_det)
            elif self.publish_candidates:
                candidate_pa.poses.append(pose)
                candidate_ids.data.append(map_id)
                candidate_full.detections.append(full_det)

            cyl, txt = self._build_marker(header, map_id, entry, confirmed)
            markers.markers.append(cyl)
            markers.markers.append(txt)
            current_marker_ids.add(map_id)

        to_delete = self.published_marker_ids - current_marker_ids
        for map_id in to_delete:
            del_cyl = Marker()
            del_cyl.header = header
            del_cyl.ns = "tree_map"
            del_cyl.id = map_id
            del_cyl.action = Marker.DELETE
            markers.markers.append(del_cyl)

            del_txt = Marker()
            del_txt.header = header
            del_txt.ns = "tree_map_text"
            del_txt.id = 100000 + map_id
            del_txt.action = Marker.DELETE
            markers.markers.append(del_txt)

        self.published_marker_ids = current_marker_ids

        self.pose_pub.publish(confirmed_pa)
        self.id_pub.publish(confirmed_ids)
        self.array_pub.publish(confirmed_full)

        self.candidate_pose_pub.publish(candidate_pa)
        self.candidate_id_pub.publish(candidate_ids)
        self.candidate_array_pub.publish(candidate_full)

        self.marker_pub.publish(markers)

    @staticmethod
    def _ensure_parent_dir(path):
        if not path:
            return
        parent = os.path.dirname(path)
        if parent and not os.path.exists(parent):
            os.makedirs(parent)

    def _snapshot_rows(self):
        with self.lock:
            now = rospy.Time.now().to_sec()
            frame_id = "world"
            if self.last_header is not None and self.last_header.frame_id:
                frame_id = self.last_header.frame_id

            rows = []
            for map_id, entry in sorted(self.entries.items(), key=lambda kv: kv[0]):
                std_xy = self._entry_std_xy(entry)
                std_d = self._entry_std_diameter(entry)
                age_sec = now - entry["last_seen"]
                diameter_m = max(entry["diameter"], 0.0)
                gt_diameter_m = max(self.ground_truth_diameter_m, 0.0)
                diameter_error_m = abs(diameter_m - gt_diameter_m)
                diameter_sq_error_m2 = (diameter_m - gt_diameter_m) ** 2
                rows.append(
                    {
                        "map_id": map_id,
                        "x": entry["x"],
                        "y": entry["y"],
                        "z": entry["z"],
                        "diameter_m": diameter_m,
                        "gt_diameter_m": gt_diameter_m,
                        "diameter_error_m": diameter_error_m,
                        "diameter_sq_error_m2": diameter_sq_error_m2,
                        "hits": entry["hits"],
                        "std_xy": std_xy,
                        "std_diameter": std_d,
                        "confidence": self._entry_confidence(entry),
                        "confirmed": self._is_confirmed(entry),
                        "suspect_merge": bool(entry.get("suspect_merge", False)),
                        "age_sec": age_sec,
                        "last_seen_sec": entry["last_seen"],
                        "source_ids": sorted(list(entry["source_ids"])),
                    }
                )
        return frame_id, rows

    def _export_files(self, reason):
        frame_id, rows = self._snapshot_rows()
        exported = False

        if self.csv_output_path:
            try:
                self._ensure_parent_dir(self.csv_output_path)
                with open(self.csv_output_path, "w") as f:
                    writer = csv.writer(f)
                    writer.writerow(
                        [
                            "map_id",
                            "x",
                            "y",
                            "z",
                            "diameter_m",
                            "gt_diameter_m",
                            "diameter_error_m",
                            "diameter_sq_error_m2",
                            "hits",
                            "std_xy",
                            "std_diameter",
                            "confidence",
                            "confirmed",
                            "suspect_merge",
                            "age_sec",
                            "last_seen_sec",
                            "source_ids",
                        ]
                    )
                    for row in rows:
                        writer.writerow(
                            [
                                row["map_id"],
                                "%.6f" % row["x"],
                                "%.6f" % row["y"],
                                "%.6f" % row["z"],
                                "%.6f" % row["diameter_m"],
                                "%.6f" % row["gt_diameter_m"],
                                "%.6f" % row["diameter_error_m"],
                                "%.6f" % row["diameter_sq_error_m2"],
                                row["hits"],
                                "%.6f" % row["std_xy"],
                                "%.6f" % row["std_diameter"],
                                "%.6f" % row["confidence"],
                                int(row["confirmed"]),
                                int(row["suspect_merge"]),
                                "%.3f" % row["age_sec"],
                                "%.3f" % row["last_seen_sec"],
                                ";".join([str(v) for v in row["source_ids"]]),
                            ]
                        )
                exported = True
            except Exception as ex:
                rospy.logwarn("tree_map_fuser failed CSV export: %s", ex)

        if self.json_output_path:
            payload = {
                "frame_id": frame_id,
                "exported_at_sec": rospy.Time.now().to_sec(),
                "count_total": len(rows),
                "count_confirmed": len([r for r in rows if r["confirmed"]]),
                "trees": rows,
            }
            try:
                self._ensure_parent_dir(self.json_output_path)
                with open(self.json_output_path, "w") as f:
                    json.dump(payload, f, indent=2, sort_keys=True)
                exported = True
            except Exception as ex:
                rospy.logwarn("tree_map_fuser failed JSON export: %s", ex)

        if exported:
            rospy.loginfo(
                "tree_map_fuser export(%s): %d trees -> %s %s",
                reason,
                len(rows),
                self.csv_output_path if self.csv_output_path else "(csv:disabled)",
                self.json_output_path if self.json_output_path else "(json:disabled)",
            )

    def export_timer_cb(self, _):
        self._export_files("periodic")

    def on_shutdown(self):
        self._export_files("shutdown")


if __name__ == "__main__":
    rospy.init_node("tree_map_fuser")
    TreeMapFuser()
    rospy.spin()
