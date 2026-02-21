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


class TreeMapFuser(object):
    """
    Long-term tree map builder.

    Inputs:
      - PoseArray detections (usually tracked detections)
      - Int32MultiArray detection IDs aligned with PoseArray

    Outputs:
      - Confirmed tree map PoseArray + IDs
      - Candidate tree map PoseArray + IDs (debug)
      - MarkerArray for RViz
      - Periodic CSV/JSON export with final map estimates
    """

    def __init__(self):
        self.pose_topic = rospy.get_param("~pose_topic", "/tree_detections_tracked")
        self.id_topic = rospy.get_param("~id_topic", "/tree_detection_ids")

        self.output_topic = rospy.get_param("~output_topic", "/tree_map_poses")
        self.output_id_topic = rospy.get_param("~output_id_topic", "/tree_map_ids")
        self.candidate_topic = rospy.get_param("~candidate_topic", "/tree_map_candidates")
        self.candidate_id_topic = rospy.get_param("~candidate_id_topic", "/tree_map_candidate_ids")
        self.marker_topic = rospy.get_param("~marker_topic", "/tree_map_markers")

        self.association_dist = rospy.get_param("~association_dist", 1.5)
        self.id_reuse_max_dist = rospy.get_param("~id_reuse_max_dist", 2.2)
        self.min_confirmations = rospy.get_param("~min_confirmations", 4)
        self.max_std_xy = rospy.get_param("~max_std_xy", -1.0)
        self.prune_unconfirmed_after_sec = rospy.get_param("~prune_unconfirmed_after_sec", 60.0)

        self.publish_rate_hz = rospy.get_param("~publish_rate_hz", 2.0)
        self.publish_candidates = rospy.get_param("~publish_candidates", True)
        self.log_throttle_sec = rospy.get_param("~log_throttle_sec", 2.0)
        self.default_height = rospy.get_param("~default_height", 2.2)
        self.default_radius = rospy.get_param("~default_radius", 0.35)

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
        self.candidate_pose_pub = rospy.Publisher(self.candidate_topic, PoseArray, queue_size=1)
        self.candidate_id_pub = rospy.Publisher(self.candidate_id_topic, Int32MultiArray, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)

        rospy.Subscriber(self.id_topic, Int32MultiArray, self.id_cb, queue_size=1)
        rospy.Subscriber(self.pose_topic, PoseArray, self.pose_cb, queue_size=1)

        publish_period = 0.5
        if self.publish_rate_hz > 0.0:
            publish_period = 1.0 / self.publish_rate_hz
        rospy.Timer(rospy.Duration(publish_period), self.publish_timer_cb)

        if self.auto_export_period_sec > 0.0:
            rospy.Timer(rospy.Duration(self.auto_export_period_sec), self.export_timer_cb)

        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo(
            "tree_map_fuser listening on %s and %s | out=%s",
            self.pose_topic,
            self.id_topic,
            self.output_topic,
        )

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

    def _is_confirmed(self, entry):
        if entry["hits"] < self.min_confirmations:
            return False
        if self.max_std_xy > 0.0 and self._entry_std_xy(entry) > self.max_std_xy:
            return False
        return True

    def _find_association(self, det, source_id):
        if source_id is not None and source_id in self.source_to_map:
            map_id = self.source_to_map[source_id]
            if map_id in self.entries:
                e = self.entries[map_id]
                d2 = self._dist_sq_xy((det[0], det[1]), (e["x"], e["y"]))
                if d2 <= (self.id_reuse_max_dist * self.id_reuse_max_dist):
                    return map_id

        best_map_id = None
        best_d2 = None
        max_d2 = self.association_dist * self.association_dist
        for map_id, entry in self.entries.items():
            d2 = self._dist_sq_xy((det[0], det[1]), (entry["x"], entry["y"]))
            if d2 > max_d2:
                continue
            if best_d2 is None or d2 < best_d2:
                best_d2 = d2
                best_map_id = map_id
        return best_map_id

    def _create_entry(self, det, source_id, now):
        map_id = self.next_map_id
        self.next_map_id += 1
        entry = {
            "x": det[0],
            "y": det[1],
            "z": det[2],
            "hits": 1,
            "m2_x": 0.0,
            "m2_y": 0.0,
            "m2_z": 0.0,
            "last_seen": now,
            "source_ids": set(),
        }
        if source_id is not None:
            entry["source_ids"].add(source_id)
            self.source_to_map[source_id] = map_id
        self.entries[map_id] = entry
        return map_id

    def _update_entry(self, map_id, det, source_id, now):
        entry = self.entries[map_id]

        entry["hits"] += 1
        n = float(entry["hits"])

        dx = det[0] - entry["x"]
        entry["x"] = entry["x"] + dx / n
        entry["m2_x"] = entry["m2_x"] + dx * (det[0] - entry["x"])

        dy = det[1] - entry["y"]
        entry["y"] = entry["y"] + dy / n
        entry["m2_y"] = entry["m2_y"] + dy * (det[1] - entry["y"])

        dz = det[2] - entry["z"]
        entry["z"] = entry["z"] + dz / n
        entry["m2_z"] = entry["m2_z"] + dz * (det[2] - entry["z"])

        entry["last_seen"] = now
        if source_id is not None:
            entry["source_ids"].add(source_id)
            self.source_to_map[source_id] = map_id

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

    def pose_cb(self, msg):
        now = rospy.Time.now().to_sec()

        detections = []
        for pose in msg.poses:
            detections.append((pose.position.x, pose.position.y, pose.position.z))

        with self.lock:
            self.last_header = msg.header
            ids = list(self.latest_ids)

            new_count = 0
            update_count = 0

            has_ids = (len(ids) == len(detections))
            for i, det in enumerate(detections):
                source_id = ids[i] if has_ids else None
                map_id = self._find_association(det, source_id)
                if map_id is None:
                    self._create_entry(det, source_id, now)
                    new_count += 1
                else:
                    self._update_entry(map_id, det, source_id, now)
                    update_count += 1

            self._prune_unconfirmed(now)

            now_log = rospy.Time.now().to_sec()
            if (now_log - self.last_log_time) >= self.log_throttle_sec:
                confirmed = 0
                candidates = 0
                for entry in self.entries.values():
                    if self._is_confirmed(entry):
                        confirmed += 1
                    else:
                        candidates += 1
                rospy.loginfo(
                    "tree_map_fuser: frame=%d new=%d upd=%d map_confirmed=%d candidates=%d total=%d",
                    len(detections),
                    new_count,
                    update_count,
                    confirmed,
                    candidates,
                    len(self.entries),
                )
                self.last_log_time = now_log

    def _build_marker(self, header, map_id, entry, confirmed):
        std_xy = self._entry_std_xy(entry)

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
        cyl.scale.x = max(2.0 * self.default_radius, 2.0 * (std_xy + 0.12))
        cyl.scale.y = max(2.0 * self.default_radius, 2.0 * (std_xy + 0.12))
        cyl.color.a = 0.5
        if confirmed:
            cyl.color.r = 0.1
            cyl.color.g = 0.75
            cyl.color.b = 0.2
        else:
            cyl.color.r = 0.95
            cyl.color.g = 0.65
            cyl.color.b = 0.12
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
        txt.scale.z = 0.32
        txt.color.r = 1.0
        txt.color.g = 1.0
        txt.color.b = 1.0
        txt.color.a = 0.95
        txt.text = "map_%d h=%d s=%.2f" % (map_id, entry["hits"], std_xy)
        txt.lifetime = rospy.Duration(0.0)

        return cyl, txt

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

        candidate_pa = PoseArray()
        candidate_pa.header = header
        candidate_ids = Int32MultiArray()

        markers = MarkerArray()
        current_marker_ids = set()

        for map_id, entry in entries:
            confirmed = self._is_confirmed(entry)

            pose = Pose()
            pose.position.x = entry["x"]
            pose.position.y = entry["y"]
            pose.position.z = entry["z"]
            pose.orientation.w = 1.0

            if confirmed:
                confirmed_pa.poses.append(pose)
                confirmed_ids.data.append(map_id)
            elif self.publish_candidates:
                candidate_pa.poses.append(pose)
                candidate_ids.data.append(map_id)

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
        self.candidate_pose_pub.publish(candidate_pa)
        self.candidate_id_pub.publish(candidate_ids)
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
                age_sec = now - entry["last_seen"]
                rows.append(
                    {
                        "map_id": map_id,
                        "x": entry["x"],
                        "y": entry["y"],
                        "z": entry["z"],
                        "hits": entry["hits"],
                        "std_xy": std_xy,
                        "confirmed": self._is_confirmed(entry),
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
                            "hits",
                            "std_xy",
                            "confirmed",
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
                                row["hits"],
                                "%.6f" % row["std_xy"],
                                int(row["confirmed"]),
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
