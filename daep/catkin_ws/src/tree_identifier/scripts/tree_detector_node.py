#!/usr/bin/env python

import math
import random
from collections import deque

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import tf2_ros
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from visualization_msgs.msg import Marker, MarkerArray

from tree_identifier.msg import TreeDetection, TreeDetectionArray

try:
    from sklearn.cluster import DBSCAN
    from sklearn.mixture import GaussianMixture

    SKLEARN_AVAILABLE = True
except Exception:
    SKLEARN_AVAILABLE = False


class TreeDetectorNode(object):
    """
    PointCloud2 -> tree detections (legacy + unified message contract)

    Main mode (default):
      slice by trunk height -> DBSCAN -> optional GMM split -> circle RANSAC

    Fallback mode:
      grid occupancy + connected components -> circle RANSAC
    """

    def __init__(self):
        self.input_cloud_topic = rospy.get_param("~input_cloud_topic", "/camera/depth/points")
        self.output_topic = rospy.get_param("~output_topic", "/tree_detections")
        self.output_array_topic = rospy.get_param("~output_array_topic", "/tree_detections_full")
        self.marker_topic = rospy.get_param("~marker_topic", "/tree_detections_raw_markers")
        self.target_frame = rospy.get_param("~target_frame", "world")
        self.publish_debug_clusters = rospy.get_param("~publish_debug_clusters", True)
        self.cluster_points_topic = rospy.get_param("~cluster_points_topic", "/tree_detector_cluster_points")
        self.cluster_labels_topic = rospy.get_param("~cluster_labels_topic", "/tree_detector_cluster_labels")
        self.detection_radius_topic = rospy.get_param("~detection_radius_topic", "/tree_detection_radii")

        self.slice_z_min = rospy.get_param("~slice_z_min", 1.0)
        self.slice_z_max = rospy.get_param("~slice_z_max", 1.6)
        self.max_points = rospy.get_param("~max_points", 120000)
        self.marker_lifetime = rospy.get_param("~marker_lifetime", 0.8)

        self.ransac_iterations = rospy.get_param("~ransac_iterations", 70)
        self.ransac_distance_threshold = rospy.get_param("~ransac_distance_threshold", 0.03)
        self.ransac_min_inlier_ratio = rospy.get_param("~ransac_min_inlier_ratio", 0.18)
        self.min_diameter = rospy.get_param("~min_diameter", 0.10)
        self.max_diameter = rospy.get_param("~max_diameter", 1.20)

        self.clustering_mode = rospy.get_param("~clustering_mode", "dbscan_gmm")

        self.dbscan_eps = rospy.get_param("~dbscan_eps", 0.30)
        self.dbscan_min_samples = rospy.get_param("~dbscan_min_samples", 10)
        self.min_cluster_points = rospy.get_param("~min_cluster_points", 8)

        self.enable_gmm_split = rospy.get_param("~enable_gmm_split", True)
        self.gmm_radius_threshold = rospy.get_param("~gmm_radius_threshold", 0.26)
        self.gmm_min_points = rospy.get_param("~gmm_min_points", 24)
        self.gmm_components = rospy.get_param("~gmm_components", 2)
        self.gmm_min_center_dist = rospy.get_param("~gmm_min_center_dist", 0.34)
        self.experiment_seed = int(rospy.get_param("~experiment_seed", -1))
        if rospy.has_param("~gmm_random_state"):
            self.gmm_random_state = int(rospy.get_param("~gmm_random_state"))
        elif self.experiment_seed >= 0:
            self.gmm_random_state = self.experiment_seed
        else:
            self.gmm_random_state = 42
        if self.experiment_seed >= 0:
            random.seed(self.experiment_seed)
            np.random.seed(self.experiment_seed % (2**32 - 1))

        self.cell_size = rospy.get_param("~cell_size", 0.20)
        self.min_points_per_cell = rospy.get_param("~min_points_per_cell", 3)
        self.min_cells_per_cluster = rospy.get_param("~min_cells_per_cluster", 3)
        self.max_cells_per_cluster = rospy.get_param("~max_cells_per_cluster", 200)

        self.pose_pub = rospy.Publisher(self.output_topic, PoseArray, queue_size=1)
        self.array_pub = rospy.Publisher(self.output_array_topic, TreeDetectionArray, queue_size=1)
        self.marker_pub = rospy.Publisher(self.marker_topic, MarkerArray, queue_size=1)
        self.cluster_points_pub = rospy.Publisher(self.cluster_points_topic, PoseArray, queue_size=1)
        self.cluster_labels_pub = rospy.Publisher(self.cluster_labels_topic, Int32MultiArray, queue_size=1)
        self.radius_pub = rospy.Publisher(self.detection_radius_topic, Float32MultiArray, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.Subscriber(self.input_cloud_topic, PointCloud2, self.cloud_callback, queue_size=1)

        if self.clustering_mode == "dbscan_gmm" and (not SKLEARN_AVAILABLE):
            rospy.logwarn("tree_detector_node: sklearn unavailable, forcing fallback to grid_cc")
            self.clustering_mode = "grid_cc"

        rospy.loginfo(
            "tree_detector_node listening on %s (target_frame=%s mode=%s array_out=%s seed=%d gmm_seed=%d)",
            self.input_cloud_topic,
            self.target_frame,
            self.clustering_mode,
            self.output_array_topic,
            self.experiment_seed,
            self.gmm_random_state,
        )

    def _to_cell(self, x, y):
        return (int(np.floor(x / self.cell_size)), int(np.floor(y / self.cell_size)))

    def _fit_circle_ransac(self, points_xy):
        if points_xy.shape[0] < 3:
            return None, None, None, None, None

        best = None
        best_inliers = 0
        best_fit_error = None

        for _ in range(self.ransac_iterations):
            try:
                p1, p2, p3 = points_xy[random.sample(range(points_xy.shape[0]), 3)]

                a_mat = np.array(
                    [
                        [p2[0] - p1[0], p2[1] - p1[1]],
                        [p3[0] - p2[0], p3[1] - p2[1]],
                    ]
                ) * 2.0
                b_vec = np.array(
                    [
                        [p2[0] ** 2 - p1[0] ** 2 + p2[1] ** 2 - p1[1] ** 2],
                        [p3[0] ** 2 - p2[0] ** 2 + p3[1] ** 2 - p2[1] ** 2],
                    ]
                )

                if abs(np.linalg.det(a_mat)) < 1e-8:
                    continue

                center = np.linalg.solve(a_mat, b_vec).T[0]
                radius = float(np.linalg.norm(p1 - center))
                diameter = 2.0 * radius
                if diameter < self.min_diameter or diameter > self.max_diameter:
                    continue

                residuals = np.abs(np.linalg.norm(points_xy - center, axis=1) - radius)
                inlier_mask = residuals < self.ransac_distance_threshold
                inliers = int(np.sum(inlier_mask))
                if inliers <= 0:
                    continue
                fit_error = float(np.mean(residuals[inlier_mask]))

                if inliers > best_inliers:
                    best_inliers = inliers
                    best_fit_error = fit_error
                    best = (center, radius)
            except Exception:
                continue

        if best is None:
            return None, None, None, None, None

        inlier_ratio = float(best_inliers) / float(max(points_xy.shape[0], 1))
        if inlier_ratio < self.ransac_min_inlier_ratio:
            return None, None, None, None, None

        center, radius = best
        return float(center[0]), float(center[1]), float(radius), float(best_fit_error), float(inlier_ratio)

    def _split_cluster_gmm(self, cluster_xy, global_indices):
        if (not self.enable_gmm_split) or (not SKLEARN_AVAILABLE):
            return [global_indices]
        if cluster_xy.shape[0] < self.gmm_min_points:
            return [global_indices]
        if self.gmm_components < 2:
            return [global_indices]

        _, _, radius, _, _ = self._fit_circle_ransac(cluster_xy)
        if radius is None or radius <= self.gmm_radius_threshold:
            return [global_indices]

        try:
            model = GaussianMixture(
                n_components=int(self.gmm_components),
                covariance_type="full",
                random_state=int(self.gmm_random_state),
            )
            labels = model.fit_predict(cluster_xy)
            means = model.means_
        except Exception:
            return [global_indices]

        if means.shape[0] >= 2:
            dist_centers = float(np.linalg.norm(means[0] - means[1]))
            if dist_centers < self.gmm_min_center_dist:
                return [global_indices]

        out = []
        for lb in sorted(set(labels.tolist())):
            idx_local = np.where(labels == lb)[0]
            if idx_local.shape[0] < self.min_cluster_points:
                continue
            out.append(global_indices[idx_local])

        if len(out) < 2:
            return [global_indices]
        return out

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

    def _confidence(self, inlier_ratio, fit_error):
        if inlier_ratio is None or fit_error is None:
            return 0.0
        err_term = math.exp(-float(fit_error) / max(2.0 * self.ransac_distance_threshold, 1e-6))
        conf = 0.62 * float(inlier_ratio) + 0.38 * float(err_term)
        return float(max(0.0, min(1.0, conf)))

    def _detect_dbscan_gmm(self, trunk):
        xy = trunk[:, :2]
        try:
            labels = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit_predict(xy)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "tree_detector_node DBSCAN failed: %s", exc)
            return [], [], []

        detections = []
        debug_cluster_points = []
        debug_cluster_labels = []
        debug_cluster_id = 0

        unique_labels = sorted(set(labels.tolist()))
        for lb in unique_labels:
            if lb < 0:
                continue

            cluster_idx = np.where(labels == lb)[0]
            if cluster_idx.shape[0] < self.min_cluster_points:
                continue

            split_groups = self._split_cluster_gmm(xy[cluster_idx], cluster_idx)
            for sub_idx in split_groups:
                if sub_idx.shape[0] < self.min_cluster_points:
                    continue

                cluster_points = trunk[sub_idx]
                if cluster_points.shape[0] < self.min_cluster_points:
                    continue

                for p in cluster_points:
                    debug_cluster_points.append((float(p[0]), float(p[1]), float(p[2])))
                    debug_cluster_labels.append(int(debug_cluster_id))

                cx, cy, r, fit_error, inlier_ratio = self._fit_circle_ransac(cluster_points[:, :2])
                if r is not None:
                    z = float(np.mean(cluster_points[:, 2]))
                    detections.append(
                        {
                            "x": cx,
                            "y": cy,
                            "z": z,
                            "radius": float(r),
                            "diameter": float(2.0 * r),
                            "cluster_label": int(debug_cluster_id),
                            "cluster_points": int(cluster_points.shape[0]),
                            "fit_error": float(fit_error),
                            "confidence": self._confidence(inlier_ratio, fit_error),
                        }
                    )

                debug_cluster_id += 1

        return detections, debug_cluster_points, debug_cluster_labels

    def _detect_grid_cc(self, trunk):
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
        debug_cluster_points = []
        debug_cluster_labels = []
        debug_cluster_id = 0

        for comp in components:
            if len(comp) < self.min_cells_per_cluster or len(comp) > self.max_cells_per_cluster:
                continue

            comp_points = []
            for c in comp:
                comp_points.extend(cells[c])
            if len(comp_points) < self.min_cluster_points:
                continue

            arr = np.array(comp_points, dtype=float)
            for p in comp_points:
                debug_cluster_points.append((float(p[0]), float(p[1]), float(p[2])))
                debug_cluster_labels.append(int(debug_cluster_id))

            cx, cy, r, fit_error, inlier_ratio = self._fit_circle_ransac(arr[:, :2])
            if r is not None:
                z = float(np.mean(arr[:, 2]))
                detections.append(
                    {
                        "x": cx,
                        "y": cy,
                        "z": z,
                        "radius": float(r),
                        "diameter": float(2.0 * r),
                        "cluster_label": int(debug_cluster_id),
                        "cluster_points": int(arr.shape[0]),
                        "fit_error": float(fit_error),
                        "confidence": self._confidence(inlier_ratio, fit_error),
                    }
                )
            debug_cluster_id += 1

        return detections, debug_cluster_points, debug_cluster_labels

    def _publish_debug_clusters(self, header, cluster_points, cluster_labels):
        cloud_msg = PoseArray()
        cloud_msg.header = header
        for p in cluster_points:
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            pose.position.z = p[2]
            pose.orientation.w = 1.0
            cloud_msg.poses.append(pose)
        labels_msg = Int32MultiArray()
        labels_msg.data = list(cluster_labels)
        self.cluster_points_pub.publish(cloud_msg)
        self.cluster_labels_pub.publish(labels_msg)

    def _publish_legacy(self, header, detections, cluster_points, cluster_labels):
        out = PoseArray()
        out.header = header
        markers = MarkerArray()
        radii = Float32MultiArray()
        radii.data = []

        for i, d in enumerate(detections):
            pose = Pose()
            pose.position.x = d["x"]
            pose.position.y = d["y"]
            pose.position.z = d["z"]
            pose.orientation.w = 1.0
            out.poses.append(pose)
            radii.data.append(float(d["radius"]))

            m = Marker()
            m.header = header
            m.ns = "tree_detections_raw"
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = d["x"]
            m.pose.position.y = d["y"]
            m.pose.position.z = d["z"]
            m.pose.orientation.w = 1.0
            m.scale.x = max(float(d["diameter"]), 0.1)
            m.scale.y = max(float(d["diameter"]), 0.1)
            m.scale.z = 0.3
            m.color.r = 0.9
            m.color.g = 0.2
            m.color.b = 0.2
            m.color.a = 0.8
            m.lifetime = rospy.Duration(self.marker_lifetime)
            markers.markers.append(m)

        self.pose_pub.publish(out)
        self.marker_pub.publish(markers)
        self.radius_pub.publish(radii)
        if self.publish_debug_clusters:
            self._publish_debug_clusters(header, cluster_points, cluster_labels)

    def _publish_unified(self, header, detections):
        msg = TreeDetectionArray()
        msg.header = header
        for i, d in enumerate(detections):
            det = TreeDetection()
            det.id = i + 1
            det.pose.position.x = d["x"]
            det.pose.position.y = d["y"]
            det.pose.position.z = d["z"]
            det.pose.orientation.w = 1.0
            det.radius = float(d["radius"])
            det.diameter = float(d["diameter"])
            det.cluster_label = int(d["cluster_label"])
            det.cluster_points = int(d["cluster_points"])
            det.fit_error = float(d["fit_error"])
            det.confidence = float(d["confidence"])
            msg.detections.append(det)
        self.array_pub.publish(msg)

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

    def _publish_empty(self, header):
        self._publish_legacy(header, [], [], [])
        empty_full = TreeDetectionArray()
        empty_full.header = header
        self.array_pub.publish(empty_full)

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

        if self.clustering_mode == "dbscan_gmm":
            detections, cluster_points, cluster_labels = self._detect_dbscan_gmm(trunk)
        else:
            detections, cluster_points, cluster_labels = self._detect_grid_cc(trunk)

        self._publish_legacy(msg.header, detections, cluster_points, cluster_labels)
        self._publish_unified(msg.header, detections)

        n_clusters = len(set([lb for lb in cluster_labels if lb >= 0]))
        rospy.loginfo_throttle(
            2.0,
            "tree_detector_node: mode=%s detections=%d clusters=%d cluster_points=%d",
            self.clustering_mode,
            len(detections),
            n_clusters,
            len(cluster_points),
        )


if __name__ == "__main__":
    rospy.init_node("tree_detector_node")
    TreeDetectorNode()
    rospy.spin()
