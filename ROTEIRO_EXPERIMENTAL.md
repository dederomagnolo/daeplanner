exploraçao:

./simulation.sh world_jean
./exploration.sh world_jean_exploration.yaml k 2000

arvores:

./daep/run_tree_stack_realtime.sh /camera/depth/points world /home/daep/data/tree_map_final.csv /home/daep/data/tree_map_final.json /home/daep/tree_snapshots k

onde k é a seed e 2000 o tempo em segundos considerado na construção do baseline

rodar ao terminar para gerar summary:
python3 /home/daep/export_experiment_report.py --name baseline_seed_k

** Em snapshots e octomaps, ele pega o mais recente por timestamp (*.pkl, *.png, *.bt).