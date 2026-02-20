#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray


class TreeIdTracker(object):
    """
    Decoupled tree identifier/tracker.

    Input: PoseArray with tree detections (positions).
    Output:
      - PoseArray with tracked positions
      - Int32MultiArray with IDs aligned with poses
      - MarkerArray with cylinder + text(ID) for RViz
    """

    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/tree_detections")
        self.output_topic = rospy.get_param("~output_topic", "/tree_detections_tracked")
        self.id_topic = rospy.get_param("~id_topic", "/tree_detection_ids")
        self.marker_topic = rospy.get_param("~marker_topic", "/tree_detection_markers")

        self.max_match_dist = rospy.get_param("~max_match_dist", 1.0)
        self.max_missed_time = rospy.get_param("~max_missed_time", 5.0)
        self.default_radius = rospy.get_param("~default_radius", 0.3)
        self.default_height = rospy.get_param("~default_height", 2.0)
        self.marker_lifetime = rospy.get_param("~marker_lifetime", 0.8)

        self.next_id = 1
        # id -> {x, y, z, t_last}
        self.tracks = {}

        self.pose_pub = rospy.Publisher(self.output_topic, PoseArray, queue_size=1)
        self.id_pub = rospy.Publisher(self.id_topic, Int32MultiArray, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)

        rospy.Subscriber(self.input_topic, PoseArray, self.callback, queue_size=1)
        rospy.loginfo("tree_id_tracker listening on %s", self.input_topic)

    @staticmethod
    def _dist_sq(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return dx * dx + dy * dy + dz * dz

    def _prune_old_tracks(self, now):
        to_remove = []
        for track_id, tr in self.tracks.items():
            if (now - tr["t_last"]) > self.max_missed_time:
                to_remove.append(track_id)
        for track_id in to_remove:
            del self.tracks[track_id]

    def _associate(self, detections, now):
        max_d2 = self.max_match_dist * self.max_match_dist

        # build candidate pairs (distance, det_idx, track_id)
        pairs = []
        for det_idx, det in enumerate(detections):
            for track_id, tr in self.tracks.items():
                d2 = self._dist_sq(det, (tr["x"], tr["y"], tr["z"]))
                if d2 <= max_d2:
                    pairs.append((d2, det_idx, track_id))

        pairs.sort(key=lambda x: x[0])
        assigned_det = set()
        assigned_track = set()

        det_to_id = {}

        # greedy nearest-neighbor assignment
        for _, det_idx, track_id in pairs:
            if det_idx in assigned_det or track_id in assigned_track:
                continue
            assigned_det.add(det_idx)
            assigned_track.add(track_id)
            det_to_id[det_idx] = track_id

        # unmatched detections -> new IDs
        for det_idx, det in enumerate(detections):
            if det_idx in det_to_id:
                continue
            track_id = self.next_id
            self.next_id += 1
            self.tracks[track_id] = {
                "x": det[0],
                "y": det[1],
                "z": det[2],
                "t_last": now,
            }
            det_to_id[det_idx] = track_id

        # update matched tracks
        for det_idx, track_id in det_to_id.items():
            det = detections[det_idx]
            self.tracks[track_id]["x"] = det[0]
            self.tracks[track_id]["y"] = det[1]
            self.tracks[track_id]["z"] = det[2]
            self.tracks[track_id]["t_last"] = now

        return det_to_id

    def _publish(self, header, detections, det_to_id):
        pose_array = PoseArray()
        pose_array.header = header

        ids_msg = Int32MultiArray()

        markers = MarkerArray()

        for i, det in enumerate(detections):
            track_id = det_to_id[i]

            pose = Pose()
            pose.position.x = det[0]
            pose.position.y = det[1]
            pose.position.z = det[2]
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)
            ids_msg.data.append(track_id)

            cyl = Marker()
            cyl.header = header
            cyl.ns = "trees"
            cyl.id = track_id
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.pose.position.x = det[0]
            cyl.pose.position.y = det[1]
            cyl.pose.position.z = det[2] + self.default_height * 0.5
            cyl.pose.orientation.w = 1.0
            cyl.scale.x = 2.0 * self.default_radius
            cyl.scale.y = 2.0 * self.default_radius
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
            txt.pose.position.x = det[0]
            txt.pose.position.y = det[1]
            txt.pose.position.z = det[2] + self.default_height + 0.3
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.35
            txt.color.r = 1.0
            txt.color.g = 1.0
            txt.color.b = 1.0
            txt.color.a = 0.95
            txt.text = "tree_%d" % track_id
            txt.lifetime = rospy.Duration(self.marker_lifetime)
            markers.markers.append(txt)

        self.pose_pub.publish(pose_array)
        self.id_pub.publish(ids_msg)
        self.marker_pub.publish(markers)

    def callback(self, msg):
        now = rospy.Time.now().to_sec()
        self._prune_old_tracks(now)

        detections = []
        for pose in msg.poses:
            detections.append((pose.position.x, pose.position.y, pose.position.z))

        if not detections:
            # keep publishing empty arrays so downstream knows there are no trees
            empty_pose_array = PoseArray()
            empty_pose_array.header = msg.header
            self.pose_pub.publish(empty_pose_array)
            empty_ids = Int32MultiArray()
            empty_ids.data = []
            self.id_pub.publish(empty_ids)
            self.marker_pub.publish(MarkerArray())
            return

        det_to_id = self._associate(detections, now)
        self._publish(msg.header, detections, det_to_id)


if __name__ == "__main__":
    rospy.init_node("tree_id_tracker")
    TreeIdTracker()
    rospy.spin()
