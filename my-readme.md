# Rodar Experimento

- Terminal 1: Simulation
```
./dev_env.sh start daep
source .bashrc
./simulation.sh simple_forest dynamic "(0,0,0.5)" false true
```
- Terminal 2
Tree stack realtime (com plotters + export):

```

./run_tree_stack_realtime.sh \
  /camera/depth/points \
  world \
  /home/daep/data/tree_map_final.csv \
  /home/daep/data/tree_map_final.json \
  /home/daep/tree_snapshots

```

- Terminal 3: Start Exploration
```
./dev_env.sh bash daep
source .bashrc
./exploration.sh simple_forest_exploration.yaml
```

## Obter logs

- Salvar estado durante runtime (PKL/PNG + CSV/JSON + BT):

`./save_tree_runtime_state.sh`

ou com tag personalizada:
`./save_tree_runtime_state.sh /home/daep/meus-resultados/world-jean exp01_t120`

- Contar árvores atuais no mapa (sem usar rospy/python):

`./count_tree_map_runtime.sh`

- Gerar pacote completo do experimento (relatório + plots + artefatos):

`python3 /home/daep/export_experiment_report.py --name nome-do-teste`

saída em:
`/home/daep/experimentos/nome-do-teste`

# Visualizar pcd

- Terminal 1
```
pcd=$(ls -1t /home/daep/octomaps/pcd/*.pcd | head -n1)
rosrun pcl_ros pcd_to_pointcloud "$pcd" 0.1 _frame_id:=world
```
- Terminal 2: RVIz
`rosrun rviz rviz`

- No RViz: Fixed Frame = world e adiciona PointCloud2 no tópico /cloud_pcd
