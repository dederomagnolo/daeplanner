#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.mixture import GaussianMixture
import random
import pickle

FILE = "dbscan_gmm_ransac"

class TreeDetector:
    def __init__(self):
        rospy.loginfo("Iniciando o nó de detecção de árvores...")
        self.detected_trees = []
        self.next_tree_id = 0
        self.has_processed_cloud = False
        self.final_slice_points = None
        self.final_labels = None
        cloud_topic = rospy.get_param('~cloud_topic', '/sdf_map/occupancy_all_1')
        self.cloud_sub = rospy.Subscriber(cloud_topic, PointCloud2, self.pointcloud_callback, queue_size=1)
        rospy.on_shutdown(self.shutdown_hook)
        rospy.loginfo(f"Nó pronto. Escutando PointCloud2 em: {cloud_topic}")

    def cluster_with_dbscan(self, points_2d, eps=0.3, min_samples=10):
        """
        Agrupa pontos usando DBSCAN.
        """
        db = DBSCAN(eps=eps, min_samples=min_samples)
        labels = db.fit_predict(points_2d)
        return labels

    def apply_gmm_if_needed(self, points_2d, radius_threshold=0.25):
        """
        Aplica GMM para tentar subdividir clusters muito grandes (possível mais de um tronco).
        """
        center_x, center_y, radius = self.fit_circle_ransac(points_2d)
        if radius is None or radius <= radius_threshold:
            # Não subdividir
            return [points_2d]
        
        # Tenta dividir em 2 subclusters com GMM
        gmm = GaussianMixture(n_components=2, covariance_type='full', random_state=42)
        labels = gmm.fit_predict(points_2d)
        subclusters = [points_2d[labels == i] for i in range(2)]
        return subclusters

    def pointcloud_callback(self, cloud_msg: PointCloud2):
        if self.has_processed_cloud:
            return
        
        rospy.loginfo("Recebida nova PointCloud2. Iniciando processamento...")
        z_min, z_max = 1.15, 1.45
        try:
            gen = pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
            all_points = np.array(list(gen))
        except Exception as e:
            rospy.logerr(f"Falha ao ler pontos da PointCloud2: {e}")
            return

        if all_points.shape[0] < 10:
            rospy.logwarn("Nuvem de pontos vazia ou com poucos pontos.")
            return

        trunk_slice_points = all_points[(all_points[:, 2] >= z_min) & (all_points[:, 2] <= z_max)]
        if trunk_slice_points.shape[0] < 10:
            rospy.loginfo("Não há pontos suficientes na fatia de altura do tronco para análise.")
            return
            
        rospy.loginfo(f"Encontrados {trunk_slice_points.shape[0]} pontos na fatia do tronco.")

        points_for_clustering = trunk_slice_points[:, :2]
        
        # --- AGRUPAMENTO COM DBSCAN ---
        labels = self.cluster_with_dbscan(points_for_clustering, eps=0.3, min_samples=10)
        
        self.final_slice_points = trunk_slice_points
        self.final_labels = labels
        
        num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
        rospy.loginfo(f"DBSCAN encontrou {num_clusters} cluster(s) potenciais.")

        self.process_clusters(labels, trunk_slice_points, points_for_clustering)
        
        self.has_processed_cloud = True
        rospy.loginfo("Processamento da PointCloud2 concluído.")
        rospy.signal_shutdown("Processamento concluído. A salvar dados.")

    def process_clusters(self, labels, trunk_slice_points, points_for_clustering):
        for label in set(labels):
            if label == -1:
                continue
            
            cluster_mask = (labels == label)
            points_in_cluster_2d = points_for_clustering[cluster_mask]

            # Verifica se cluster é grande e precisa de GMM
            subclusters = self.apply_gmm_if_needed(points_in_cluster_2d)

            for subc in subclusters:
                if subc.shape[0] < 3:
                    continue
                center_x, center_y, radius = self.fit_circle_ransac(subc)
                if radius is None or radius * 2 < 0.15:
                    continue
                diameter = radius * 2
                z_mean_cluster = np.mean(trunk_slice_points[cluster_mask, 2])
                position = Point(x=center_x, y=center_y, z=z_mean_cluster)
                self.update_tree_list(position, diameter)

    def update_tree_list(self, position, diameter):
        self.next_tree_id += 1
        new_tree = {'id': self.next_tree_id, 'position': position, 'diameter': diameter}
        self.detected_trees.append(new_tree)
        rospy.loginfo(f"NOVA Árvore ID {new_tree['id']} | Posição(X={position.x:.2f}, Y={position.y:.2f}), Diâmetro (RANSAC): {diameter:.2f}m")

    def fit_circle_ransac(self, points_xy):
        if points_xy.shape[0] < 3:
            return None, None, None
        best_inliers_count = 0
        best_circle_params = None
        n_iterations = 50
        distance_threshold = 0.01
        for _ in range(n_iterations):
            try:
                p1, p2, p3 = points_xy[random.sample(range(points_xy.shape[0]), 3)]
                A = np.array([[p2[0] - p1[0], p2[1] - p1[1]], [p3[0] - p2[0], p3[1] - p2[1]]]) * 2
                b = np.array([[p2[0]**2 - p1[0]**2 + p2[1]**2 - p1[1]**2],
                              [p3[0]**2 - p2[0]**2 + p3[1]**2 - p2[1]**2]])
                if abs(np.linalg.det(A)) < 1e-6:
                    continue
                center = np.linalg.solve(A, b).T[0]
                radius = np.linalg.norm(p1 - center)
                if radius > 0.5:
                    continue
                distances = np.abs(np.linalg.norm(points_xy - center, axis=1) - radius)
                inliers_count = np.sum(distances < distance_threshold)
                if inliers_count > best_inliers_count:
                    best_inliers_count = inliers_count
                    best_circle_params = (center, radius)
            except (np.linalg.LinAlgError, IndexError):
                continue
        if best_circle_params is None:
            return None, None, None
        final_center, final_radius = best_circle_params
        return final_center[0], final_center[1], final_radius

    def shutdown_hook(self):
        rospy.loginfo("Encerrando o nó de detecção de árvores...")
        if self.detected_trees:
            rospy.loginfo("--- Resumo Final das Árvores Detectadas ---")
            for tree in sorted(self.detected_trees, key=lambda t: t['id']):
                pos, d = tree['position'], tree['diameter']
                rospy.loginfo(f"Árvore ID {tree['id']}: Posição(X={pos.x:.2f}, Y={pos.y:.2f}), Diâmetro: {d:.2f}m")
        
        if self.final_slice_points is not None and self.final_labels is not None:
            dados_para_salvar = {
                'points_xy': self.final_slice_points[:, :2],
                'labels': self.final_labels,
                'detected_trees': self.detected_trees
            }
            try:
                filename = FILE + '.pkl'
                with open(filename, 'wb') as f:
                    pickle.dump(dados_para_salvar, f)
                rospy.loginfo(f"Dados do gráfico salvos com sucesso em '{filename}'")
            except Exception as e:
                rospy.logerr(f"Falha ao salvar os dados do gráfico: {e}")

if __name__ == '__main__':
    rospy.init_node('pointcloud_tree_detector', anonymous=True)
    try:
        detector = TreeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
