#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

from tree_identifier.msg import TreeDetection, TreeDetectionArray


class TreeIdTracker(object):
    """
    Tree tracker with global assignment and diameter propagation.

    Input:
      - preferred: TreeDetectionArray (/tree_detections_full)
      - legacy: PoseArray (/tree_detections)

    Output:
      - TreeDetectionArray (/tree_detections_tracked_full)
      - legacy PoseArray + ID list + markers
    """

    def __init__(self):
        self.use_array_input = rospy.get_param("~use_array_input", True)
        self.input_topic = rospy.get_param("~input_topic", "/tree_detections")
        self.input_array_topic = rospy.get_param("~input_array_topic", "/tree_detections_full")
        self.output_topic = rospy.get_param("~output_topic", "/tree_detections_tracked")
        self.output_array_topic = rospy.get_param("~output_array_topic", "/tree_detections_tracked_full")
        self.id_topic = rospy.get_param("~id_topic", "/tree_detection_ids")
        self.marker_topic = rospy.get_param("~marker_topic", "/tree_detection_markers")

        self.max_match_dist = rospy.get_param("~max_match_dist", 0.60)
        self.max_diameter_delta = rospy.get_param("~max_diameter_delta", 0.45)
        self.diameter_weight = rospy.get_param("~diameter_weight", 0.40)
        self.max_missed_time = rospy.get_param("~max_missed_time", 5.0)
        self.position_alpha = rospy.get_param("~position_alpha", 0.45)
        self.diameter_alpha = rospy.get_param("~diameter_alpha", 0.38)
        self.default_radius = rospy.get_param("~default_radius", 0.3)
        self.default_height = rospy.get_param("~default_height", 2.0)
        self.marker_lifetime = rospy.get_param("~marker_lifetime", 0.8)
        self.min_detection_confidence = rospy.get_param("~min_detection_confidence", 0.0)

        self.next_id = 1
        # id -> track state
        self.tracks = {}

        self.pose_pub = rospy.Publisher(self.output_topic, PoseArray, queue_size=1)
        self.array_pub = rospy.Publisher(self.output_array_topic, TreeDetectionArray, queue_size=1)
        self.id_pub = rospy.Publisher(self.id_topic, Int32MultiArray, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)

        if self.use_array_input:
            rospy.Subscriber(self.input_array_topic, TreeDetectionArray, self.array_callback, queue_size=1)
            rospy.loginfo("tree_id_tracker listening on %s (full mode)", self.input_array_topic)
        else:
            rospy.Subscriber(self.input_topic, PoseArray, self.legacy_pose_callback, queue_size=1)
            rospy.loginfo("tree_id_tracker listening on %s (legacy mode)", self.input_topic)

    @staticmethod
    def _dist_sq_xyz(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return dx * dx + dy * dy + dz * dz

    @staticmethod
    def _linear_sum_assignment(cost):
        """
        Hungarian algorithm for rectangular cost matrix.
        Returns assignment array with column index for each row (or -1).
        """
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

    def _prune_old_tracks(self, now):
        to_remove = []
        for track_id, tr in self.tracks.items():
            if (now - tr["t_last"]) > self.max_missed_time:
                to_remove.append(track_id)
        for track_id in to_remove:
            del self.tracks[track_id]

    def _build_cost(self, det, tr):
        dx = det["x"] - tr["x"]
        dy = det["y"] - tr["y"]
        dist_xy = (dx * dx + dy * dy) ** 0.5
        if dist_xy > self.max_match_dist:
            return None

        det_diam = max(float(det.get("diameter", 0.0)), 0.0)
        tr_diam = max(float(tr.get("diameter", 0.0)), 0.0)
        if det_diam > 0.0 and tr_diam > 0.0:
            delta_d = abs(det_diam - tr_diam)
            if delta_d > self.max_diameter_delta:
                return None
        else:
            delta_d = 0.0

        return float(dist_xy + self.diameter_weight * delta_d)

    def _associate(self, detections, now):
        track_ids = sorted(self.tracks.keys())
        det_to_id = {}
        invalid_cost = 1e6

        if detections and track_ids:
            cost = []
            for det in detections:
                row = []
                for track_id in track_ids:
                    c = self._build_cost(det, self.tracks[track_id])
                    if c is None:
                        row.append(invalid_cost)
                    else:
                        row.append(c)
                cost.append(row)

            assign = self._linear_sum_assignment(cost)
            for det_idx, col in enumerate(assign):
                if col < 0:
                    continue
                if cost[det_idx][col] >= invalid_cost * 0.5:
                    continue
                det_to_id[det_idx] = track_ids[col]

        for det_idx, det in enumerate(detections):
            if det_idx in det_to_id:
                continue
            track_id = self.next_id
            self.next_id += 1
            self.tracks[track_id] = {
                "x": det["x"],
                "y": det["y"],
                "z": det["z"],
                "radius": det["radius"],
                "diameter": det["diameter"],
                "confidence": det["confidence"],
                "hits": 1,
                "t_last": now,
            }
            det_to_id[det_idx] = track_id

        for det_idx, track_id in det_to_id.items():
            det = detections[det_idx]
            tr = self.tracks[track_id]

            tr["x"] = (1.0 - self.position_alpha) * tr["x"] + self.position_alpha * det["x"]
            tr["y"] = (1.0 - self.position_alpha) * tr["y"] + self.position_alpha * det["y"]
            tr["z"] = (1.0 - self.position_alpha) * tr["z"] + self.position_alpha * det["z"]

            if det["diameter"] > 0.0:
                tr["diameter"] = (1.0 - self.diameter_alpha) * tr["diameter"] + self.diameter_alpha * det["diameter"]
                tr["radius"] = max(0.5 * tr["diameter"], 0.0)
            tr["confidence"] = max(float(tr["confidence"]), float(det.get("confidence", 0.0)))
            tr["hits"] += 1
            tr["t_last"] = now

        return det_to_id

    def _publish(self, header, detections, det_to_id):
        pose_array = PoseArray()
        pose_array.header = header

        ids_msg = Int32MultiArray()
        array_msg = TreeDetectionArray()
        array_msg.header = header
        markers = MarkerArray()

        for i, det in enumerate(detections):
            track_id = det_to_id[i]
            tr = self.tracks[track_id]

            pose = Pose()
            pose.position.x = tr["x"]
            pose.position.y = tr["y"]
            pose.position.z = tr["z"]
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
            ids_msg.data.append(track_id)

            out_det = TreeDetection()
            out_det.id = int(track_id)
            out_det.pose = pose
            out_det.radius = float(max(tr["radius"], 0.0))
            out_det.diameter = float(max(tr["diameter"], 0.0))
            out_det.cluster_label = int(det.get("cluster_label", -1))
            out_det.cluster_points = int(det.get("cluster_points", 0))
            out_det.fit_error = float(det.get("fit_error", 0.0))
            out_det.confidence = float(max(tr["confidence"], det.get("confidence", 0.0)))
            array_msg.detections.append(out_det)

            draw_radius = max(0.5 * float(out_det.diameter), self.default_radius)
            cyl = Marker()
            cyl.header = header
            cyl.ns = "trees"
            cyl.id = track_id
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.pose.position.x = tr["x"]
            cyl.pose.position.y = tr["y"]
            cyl.pose.position.z = tr["z"] + self.default_height * 0.5
            cyl.pose.orientation.w = 1.0
            cyl.scale.x = 2.0 * draw_radius
            cyl.scale.y = 2.0 * draw_radius
            cyl.scale.z = self.default_height
            cyl.color.r = 0.1
            cyl.color.g = 0.7
            cyl.color.b = 0.1
            cyl.color.a = 0.4
            cyl.lifetime = rospy.Duration(self.marker_lifetime)
            markers.markers.append(cyl)

            txt = Marker()
            txt.header = header
            txt.ns = "tree_ids"
            txt.id = 100000 + track_id
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = tr["x"]
            txt.pose.position.y = tr["y"]
            txt.pose.position.z = tr["z"] + self.default_height + 0.3
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.35
            txt.color.r = 1.0
            txt.color.g = 1.0
            txt.color.b = 1.0
            txt.color.a = 0.95
            txt.text = "tree_%d d=%.2f" % (track_id, max(float(out_det.diameter), 0.0))
            txt.lifetime = rospy.Duration(self.marker_lifetime)
            markers.markers.append(txt)

        self.pose_pub.publish(pose_array)
        self.array_pub.publish(array_msg)
        self.id_pub.publish(ids_msg)
        self.marker_pub.publish(markers)

    def _publish_empty(self, header):
        empty_pose_array = PoseArray()
        empty_pose_array.header = header
        self.pose_pub.publish(empty_pose_array)
        empty_array = TreeDetectionArray()
        empty_array.header = header
        self.array_pub.publish(empty_array)
        empty_ids = Int32MultiArray()
        empty_ids.data = []
        self.id_pub.publish(empty_ids)
        self.marker_pub.publish(MarkerArray())

    def _process(self, header, detections):
        now = rospy.Time.now().to_sec()
        self._prune_old_tracks(now)

        if not detections:
            self._publish_empty(header)
            return

        det_to_id = self._associate(detections, now)
        self._publish(header, detections, det_to_id)

    def array_callback(self, msg):
        detections = []
        for det in msg.detections:
            confidence = float(det.confidence)
            if confidence < self.min_detection_confidence:
                continue
            diameter = float(det.diameter)
            radius = float(det.radius)
            if diameter <= 0.0 and radius > 0.0:
                diameter = 2.0 * radius
            if radius <= 0.0 and diameter > 0.0:
                radius = 0.5 * diameter
            detections.append(
                {
                    "x": float(det.pose.position.x),
                    "y": float(det.pose.position.y),
                    "z": float(det.pose.position.z),
                    "radius": max(radius, 0.0),
                    "diameter": max(diameter, 0.0),
                    "cluster_label": int(det.cluster_label),
                    "cluster_points": int(det.cluster_points),
                    "fit_error": float(det.fit_error),
                    "confidence": confidence,
                }
            )
        self._process(msg.header, detections)

    def legacy_pose_callback(self, msg):
        detections = []
        for pose in msg.poses:
            detections.append(
                {
                    "x": float(pose.position.x),
                    "y": float(pose.position.y),
                    "z": float(pose.position.z),
                    "radius": float(self.default_radius),
                    "diameter": float(2.0 * self.default_radius),
                    "cluster_label": -1,
                    "cluster_points": 0,
                    "fit_error": 0.0,
                    "confidence": 0.0,
                }
            )
        self._process(msg.header, detections)


if __name__ == "__main__":
    rospy.init_node("tree_id_tracker")
    TreeIdTracker()
    rospy.spin()
