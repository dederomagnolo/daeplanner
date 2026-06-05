#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import Point
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.mixture import GaussianMixture
from scipy.spatial.distance import pdist
import pickle

FILE = "dbscan_gmm_rlts" # Nome do ficheiro alterado

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
        
        dbscan = DBSCAN(eps=0.3, min_samples=10).fit(points_for_clustering)
        labels = dbscan.labels_
        
        self.final_slice_points = trunk_slice_points
        self.final_labels = labels
        
        num_clusters = len(set(labels)) - (1 if -1 in labels else 0)
        rospy.loginfo(f"DBSCAN encontrou {num_clusters} cluster(s) potenciais.")

        self.process_clusters(labels, trunk_slice_points, points_for_clustering)
        
        self.has_processed_cloud = True
        rospy.loginfo("Processamento da PointCloud2 concluído.")
        rospy.signal_shutdown("Processamento concluído. A salvar dados.")

    def process_clusters(self, labels, trunk_slice_points, points_for_clustering):
        final_clusters = []
        for label in set(labels):
            if label == -1:
                continue
            
            cluster_pts = points_for_clustering[labels == label]
            
            # Usa uma medição simples (distância máxima) para decidir se divide
            if pdist(cluster_pts).max() > 0.45:
                gmm = GaussianMixture(n_components=2, covariance_type='full', random_state=42)
                sub_labels = gmm.fit_predict(cluster_pts)
                for sub_id in np.unique(sub_labels):
                    final_clusters.append(cluster_pts[sub_labels == sub_id])
            else:
                final_clusters.append(cluster_pts)
        
        # Mede cada cluster final
        for cluster_points in final_clusters:
            if cluster_points.shape[0] < 5:
                continue
            
            # --- ALTERAÇÃO AQUI: Usa o novo método de medição ---
            center_x, center_y, radius = self.fit_circle_lts(cluster_points)

            if radius is None or radius * 2 < 0.15:
                continue
            diameter = radius * 2
            
            # Encontra o Z médio correspondente
            # Esta é uma aproximação, mas geralmente eficaz
            z_mean_cluster = np.mean(trunk_slice_points[:, 2])
            position = Point(x=center_x, y=center_y, z=z_mean_cluster)
            self.update_tree_list(position, diameter)

    def update_tree_list(self, position, diameter):
        self.next_tree_id += 1
        new_tree = {'id': self.next_tree_id, 'position': position, 'diameter': diameter}
        self.detected_trees.append(new_tree)
        rospy.loginfo(f"NOVA Árvore ID {new_tree['id']} | Posição(X={position.x:.2f}, Y={position.y:.2f}), Diâmetro (LTS): {diameter:.2f}m")

    def fit_circle_lts(self, points_xy):
        """
        Encontra o melhor círculo em um conjunto de pontos 2D usando uma abordagem
        de Mínimos Quadrados Aparados (Least Trimmed Squares).
        """
        if points_xy.shape[0] < 5: # Precisa de pontos suficientes para aparar
            return None, None, None

        # 1. Encontra o centroide e apara os 25% de pontos mais distantes (outliers)
        centroide = np.mean(points_xy, axis=0)
        distancias = np.linalg.norm(points_xy - centroide, axis=1)
        limiar_distancia = np.percentile(distancias, 75) # Mantém os 75% mais próximos
        
        pontos_aparados = points_xy[distancias <= limiar_distancia]
        
        if pontos_aparados.shape[0] < 3:
            return None, None, None

        # 2. Executa um ajuste de mínimos quadrados nos pontos aparados
        x = pontos_aparados[:, 0]
        y = pontos_aparados[:, 1]
        
        A = np.array([x, y, np.ones(len(x))]).T
        b = -(x**2 + y**2)
        
        try:
            p, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        except np.linalg.LinAlgError:
            return None, None, None
            
        D, E, F = p
        centro_x = -D / 2
        centro_y = -E / 2
        
        raio_quadrado = centro_x**2 + centro_y**2 - F
        if raio_quadrado < 0:
            return None, None, None
        raio = np.sqrt(raio_quadrado)

        if np.isnan(raio) or raio > 0.5:
            return None, None, None

        return centro_x, centro_y, raio

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