#!/usr/bin/env python

import random
from collections import deque

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import PointCloud2
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from visualization_msgs.msg import Marker, MarkerArray


class TreeDetectorNode(object):
    """
    Decoupled tree detector:
    PointCloud2 -> PoseArray (/tree_detections)

    Method:
    1) trunk-height slice on Z
    2) grid occupancy + connected-components clustering
    3) circle RANSAC on each cluster footprint
    """

    def __init__(self):
        self.input_cloud_topic = rospy.get_param("~input_cloud_topic", "/camera/depth/points")
        self.output_topic = rospy.get_param("~output_topic", "/tree_detections")
        self.marker_topic = rospy.get_param("~marker_topic", "/tree_detections_raw_markers")
        self.target_frame = rospy.get_param("~target_frame", "world")

        self.slice_z_min = rospy.get_param("~slice_z_min", 1.0)
        self.slice_z_max = rospy.get_param("~slice_z_max", 1.6)

        self.cell_size = rospy.get_param("~cell_size", 0.20)
        self.min_points_per_cell = rospy.get_param("~min_points_per_cell", 3)
        self.min_cells_per_cluster = rospy.get_param("~min_cells_per_cluster", 3)
        self.max_cells_per_cluster = rospy.get_param("~max_cells_per_cluster", 200)

        self.ransac_iterations = rospy.get_param("~ransac_iterations", 60)
        self.ransac_distance_threshold = rospy.get_param("~ransac_distance_threshold", 0.03)
        self.min_diameter = rospy.get_param("~min_diameter", 0.10)
        self.max_diameter = rospy.get_param("~max_diameter", 1.20)

        self.max_points = rospy.get_param("~max_points", 120000)
        self.marker_lifetime = rospy.get_param("~marker_lifetime", 0.8)

        self.pose_pub = rospy.Publisher(self.output_topic, PoseArray, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber(self.input_cloud_topic, PointCloud2, self.cloud_callback, queue_size=1)
        rospy.loginfo(
            "tree_detector_node listening on %s (target_frame=%s)",
            self.input_cloud_topic,
            self.target_frame,
        )

    def _to_cell(self, x, y):
        return (int(np.floor(x / self.cell_size)), int(np.floor(y / self.cell_size)))

    def _fit_circle_ransac(self, points_xy):
        if points_xy.shape[0] < 3:
            return None, None, None

        best_inliers = 0
        best = None

        for _ in range(self.ransac_iterations):
            try:
                p1, p2, p3 = points_xy[random.sample(range(points_xy.shape[0]), 3)]

                a_mat = np.array([
                    [p2[0] - p1[0], p2[1] - p1[1]],
                    [p3[0] - p2[0], p3[1] - p2[1]],
                ]) * 2.0
                b_vec = np.array([
                    [p2[0] ** 2 - p1[0] ** 2 + p2[1] ** 2 - p1[1] ** 2],
                    [p3[0] ** 2 - p2[0] ** 2 + p3[1] ** 2 - p2[1] ** 2],
                ])

                if abs(np.linalg.det(a_mat)) < 1e-8:
                    continue

                center = np.linalg.solve(a_mat, b_vec).T[0]
                radius = np.linalg.norm(p1 - center)
                diameter = 2.0 * radius
                if diameter < self.min_diameter or diameter > self.max_diameter:
                    continue

                d = np.abs(np.linalg.norm(points_xy - center, axis=1) - radius)
                inliers = int(np.sum(d < self.ransac_distance_threshold))
                if inliers > best_inliers:
                    best_inliers = inliers
                    best = (center, radius)
            except Exception:
                continue

        if best is None:
            return None, None, None

        center, radius = best
        return float(center[0]), float(center[1]), float(radius)

    def _cluster_cells(self, occupied_cells):
        visited = set()
        components = []

        for start in occupied_cells:
            if start in visited:
                continue
            q = deque([start])
            visited.add(start)
            comp = []

            while q:
                c = q.popleft()
                comp.append(c)
                cx, cy = c
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        n = (cx + dx, cy + dy)
                        if n in occupied_cells and n not in visited:
                            visited.add(n)
                            q.append(n)
            components.append(comp)

        return components

    def _publish_empty(self, header):
        out = PoseArray()
        out.header = header
        self.pose_pub.publish(out)
        self.marker_pub.publish(MarkerArray())

    def _to_target_frame(self, msg):
        if not self.target_frame or msg.header.frame_id == self.target_frame:
            return msg

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                msg.header.stamp,
                rospy.Duration(0.1),
            )
            return do_transform_cloud(msg, transform)
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "tree_detector_node TF %s -> %s failed: %s",
                msg.header.frame_id,
                self.target_frame,
                exc,
            )
            return None

    def cloud_callback(self, msg):
        msg = self._to_target_frame(msg)
        if msg is None:
            return

        try:
            points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "tree_detector_node point read failed: %s", exc)
            return

        if points.shape[0] < 20:
            self._publish_empty(msg.header)
            return

        if points.shape[0] > self.max_points:
            idx = np.random.choice(points.shape[0], self.max_points, replace=False)
            points = points[idx]

        trunk = points[(points[:, 2] >= self.slice_z_min) & (points[:, 2] <= self.slice_z_max)]
        if trunk.shape[0] < 20:
            self._publish_empty(msg.header)
            return

        cells = {}
        for p in trunk:
            key = self._to_cell(p[0], p[1])
            if key not in cells:
                cells[key] = []
            cells[key].append((p[0], p[1], p[2]))

        occupied = set()
        for c, pts in cells.items():
            if len(pts) >= self.min_points_per_cell:
                occupied.add(c)

        components = self._cluster_cells(occupied)

        detections = []
        for comp in components:
            if len(comp) < self.min_cells_per_cluster or len(comp) > self.max_cells_per_cluster:
                continue

            comp_points = []
            for c in comp:
                comp_points.extend(cells[c])

            if len(comp_points) < 8:
                continue

            arr = np.array(comp_points)
            xy = arr[:, :2]
            cx, cy, r = self._fit_circle_ransac(xy)
            if r is None:
                continue

            z = float(np.mean(arr[:, 2]))
            detections.append((cx, cy, z, r))

        out = PoseArray()
        out.header = msg.header

        markers = MarkerArray()
        for i, d in enumerate(detections):
            pose = Pose()
            pose.position.x = d[0]
            pose.position.y = d[1]
            pose.position.z = d[2]
            pose.orientation.w = 1.0
            out.poses.append(pose)

            m = Marker()
            m.header = msg.header
            m.ns = "tree_detections_raw"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = d[0]
            m.pose.position.y = d[1]
            m.pose.position.z = d[2]
            m.pose.orientation.w = 1.0
            m.scale.x = max(2.0 * d[3], 0.1)
            m.scale.y = max(2.0 * d[3], 0.1)
            m.scale.z = 0.3
            m.color.r = 0.9
            m.color.g = 0.2
            m.color.b = 0.2
            m.color.a = 0.8
            m.lifetime = rospy.Duration(self.marker_lifetime)
            markers.markers.append(m)

        self.pose_pub.publish(out)
        self.marker_pub.publish(markers)
        rospy.loginfo_throttle(2.0, "tree_detector_node: %d detections", len(detections))


if __name__ == "__main__":
    rospy.init_node("tree_detector_node")
    TreeDetectorNode()
    rospy.spin()
