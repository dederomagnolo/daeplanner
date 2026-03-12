hits: quantas atualizações essa árvore recebeu no mapa (número de observações fundidas).
std_xy: dispersão espacial da árvore no plano XY (incerteza de posição).
std_diameter: dispersão do diâmetro estimado ao longo das observações (incerteza do diâmetro).
confidence: média da confiança das detecções que alimentaram essa árvore (conf_sum / hits).
confirmed: 1/0 se passou no critério de confirmação (mínimo de hits + limites de incerteza + confiança mínima).
suspect_merge: 1/0 se a entrada parece mistura de árvores (muitos source_ids + alta dispersão).
age_sec: tempo desde a última observação (agora - last_seen).
last_seen_sec: timestamp ROS da última vez que foi vista.
source_ids: IDs das detecções/rastros que foram fundidos nessa árvore.

confirmed_only: usa árvores com confirmed == 1 no tree_map_final.csv (critério interno do seu fuser).
matched_tp_only: usa só árvores do mapa que entraram como TP no matching 1:1 contra GT no limiar escolhido (critério externo de acerto vs ground truth).
all_map: usa todas as árvores do mapa, sem filtro.



confirmed = passa em todos os critérios:
hits >= min_confirmations (default 4)
std_xy <= max_std_xy (default 0.70)
std_diameter <= max_std_diameter (default 0.28)