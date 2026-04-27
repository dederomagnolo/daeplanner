# Analises dos Baselines de Deteccao de Arvores

## Objetivo

Avaliar se o planner encontra arvores de forma correta e eficiente, antes de comparar com a versao que guia a exploracao por arvores.

## Artefatos por run

Ao usar `./daep/experiment_run.sh init ...`, `./daep/run_tree_stack_realtime.sh ...` e depois `./daep/experiment_run.sh finalize`, a run deve conter:

- `data/tree_map_final.csv`: mapa final de arvores fundidas.
- `data/tree_map_history.csv`: historico temporal do mapa, append-only.
- `data/tree_detection_history.csv`: historico das deteccoes cruas do detector.
- `result/summary.md`: resumo final da run.
- `result/metrics.json`: metricas estruturadas.
- `result/matching.csv`: pareamento 1:1 entre ground truth e mapa final.
- `result/tree_discovery_timeseries.csv`: TP/FN/FP, precision, recall e F1 por tempo.
- `result/tree_discovery_summary.csv`: tempos ate 25%, 50%, 80% e 100% de recall.
- `result/tree_discovery_curves.svg`: curva temporal de recall acumulado, precision e F1.

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
./daep/experiment_run.sh init --name baseline --seed 1
./daep/simulation.sh world_jean
./daep/exploration.sh world_jean_exploration.yaml 1 2000
./daep/run_tree_stack_realtime.sh /camera/depth/points world
./daep/experiment_run.sh finalize
```

Repita com as mesmas seeds para a versao guiada por arvores. Compare principalmente:

- `final_recall`, `final_precision`, `final_f1`
- `time_to_50pct_min`, `time_to_80pct_min`
- `recall_auc_normalized`
- `Path length` e `Planning` do `logfile.csv`
- FP/FN e lista de GT sem match

## Coletas adicionadas

O fuser agora salva `tree_map_history.csv` a cada export periodico. O detector salva `tree_detection_history.csv` a cada nuvem processada. Esses dois arquivos permitem separar problemas de exploracao, detector e fusao do mapa.
