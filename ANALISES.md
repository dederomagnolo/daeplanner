# Analises dos Baselines de Deteccao de Arvores

## Objetivo

Avaliar se o planner encontra arvores de forma correta e eficiente, antes de comparar com a versao que guia a exploracao por arvores.

## Artefatos por run

Ao usar `./daep/experiment_run.sh init ...`, `./daep/run_tree_stack_realtime.sh ...` e depois `./daep/experiment_run.sh finalize`, a run deve conter:

- `data/tree_map_final.csv`: mapa final de arvores fundidas.
- `data/tree_map_history.csv`: historico temporal do mapa, append-only.
- `data/tree_detection_history.csv`: historico das deteccoes cruas do detector.
- `data/rrt_tree_log.csv`: nos da RRT gerada em cada chamada do planner.
- `data/rrt_goal_log.csv`: resumo do goal escolhido por chamada do planner.
- `snapshots/<tag>/`: checkpoints manuais ou automaticos da run.
- `result/summary.md`: resumo final derivado dos insumos.
- `result/metrics.json`: metricas estruturadas.
- `result/matching.csv`: pareamento 1:1 entre ground truth e mapa final.
- `result/tree_discovery_timeseries.csv`: TP/FN/FP, precision, recall e F1 por tempo.
- `result/tree_discovery_summary.csv`: tempos ate 25%, 50%, 80% e 100% de recall.
- `result/tree_discovery_curves.svg`: curva temporal de recall acumulado, precision e F1.
- `result/snapshot_discovery_summary.csv`: avaliacao dos snapshots salvos.
- `result/route_trees_ground_truth.svg`: rota de goals, arvores detectadas e ground truth.

Por padrao, `result/` nao duplica os CSVs de `data/`; o `manifest.json` aponta quais insumos foram usados.

## Metricas principais

- **Precision**: entre as arvores detectadas, quantas eram arvores reais.
- **Recall**: entre as arvores reais, quantas foram detectadas.
- **F1**: equilibrio entre precision e recall.
- **Erro de posicao**: distancia XY entre arvore real e deteccao pareada.
- **Erro de diametro**: diferenca entre diametro estimado e diametro de referencia.
- **Tempo ate recall 50/80/100%**: rapidez da descoberta.
- **AUC do recall acumulado**: qualidade temporal geral; maior significa descobrir cedo e manter bom recall.
- **FP/FN**: falsos positivos e arvores reais nao encontradas.
- **suspect_merge**: indica possivel fusao indevida de arvores diferentes.

## Como interpretar

Um baseline forte deve ter alto recall final, baixo FP, erro de posicao baixo e curvas de descoberta que sobem cedo. Se a versao guiada por arvores tiver recall final parecido, mas atinge 50% ou 80% mais cedo e com menor caminho voado, ela provavelmente e mais eficiente.

Se o recall sobe rapido mas precision cai, o planner esta encontrando regioes promissoras, mas o detector/fuser esta criando falsas arvores. Se precision e alta mas recall baixo, a exploracao esta conservadora ou nao esta visitando regioes suficientes.

## Comparacao recomendada

Use seeds pareadas entre baseline e tree-guided:

```bash
./experiment_run.sh init --name baseline --seed 1
./simulation.sh world_jean
./exploration.sh world_jean_exploration.yaml 1 2000
./run_tree_stack_realtime.sh /camera/depth/points world
./experiment_run.sh finalize
```

No exemplo acima, `2000` e o limite passado ao script `exploration.sh` para a execucao da exploracao. Mantenha o mesmo valor entre baseline e tree-guided para a comparacao ser justa.

Repita com as mesmas seeds para a versao guiada por arvores. Compare principalmente:

- `final_recall`, `final_precision`, `final_f1`
- `time_to_50pct_min`, `time_to_80pct_min`
- `recall_auc_normalized`
- `Path length` e `Planning` do `logfile.csv`
- FP/FN e lista de GT sem match
- `route_trees_ground_truth.svg` para ver a rota de goals junto com arvores detectadas e GT
- `snapshot_discovery_summary.csv` para conferir os checkpoints salvos

Para regenerar o relatorio de uma run ja existente, use o `run_id`:

```bash
./export_experiment_report.py --name new-baseline_seed1_20260427_224815 --overwrite
```

Quando o `run_id` existe em `experimentos/<run_id>/`, o script grava em `experimentos/<run_id>/result` e usa automaticamente `data/`, `tree_snapshots/`, `snapshots/` e `octomaps/` dessa run. Runs antigas em `experimentos/runs/<run_id>/` continuam sendo reconhecidas.

## Coletas adicionadas

O fuser agora salva `tree_map_history.csv` a cada export periodico. O detector salva `tree_detection_history.csv` a cada nuvem processada. Esses dois arquivos permitem separar problemas de exploracao, detector e fusao do mapa.

## Sobre salvar a RRT

O experimento agora salva a RRT gerada em cada chamada do planner. O arquivo completo fica em `data/rrt_tree_log.csv`, com uma linha por no da arvore, e o resumo da decisao fica em `data/rrt_goal_log.csv`, com uma linha por chamada do planner.

- `planning_iteration`
- `selected_goal_source`
- `best_node_id`
- `node_id`, `parent_id`, `x`, `y`, `z`, `yaw`
- `gain`, `dynamic_gain`, `dfm_score`
- `dynamic_score`
- flag se o ponto entrou no ramo escolhido

Com `max_sampled_nodes` em dezenas/centenas, o CSV tende a ser leve. Se aumentar para milhares de nos ou muitas runs, ajuste `RRT_TREE_LOG_EVERY_N` para salvar a arvore completa a cada N iteracoes.
