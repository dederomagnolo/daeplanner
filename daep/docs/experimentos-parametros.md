ctrl shift V

# Experimentos e Parametros

Este arquivo registra os experimentos e os parametros relevantes para comportamento de exploracao.

## Parametros foco (world_jean_exploration.yaml)

- `aep/gain/zero`
- `aep/gain/lambda`
- `aep/tree/max_sampling_radius`
- `daep/gain/cache_node_threshold`
- `daep/gain/node_gain_threshold`
- `daep/dfm/zeta`
- `daep/rrt/max_sampled_nodes`
- `daep/fixed_z_from_start`

## Tabela de runs

| run_id | zero | lambda | max_sampling_radius | cache_node_threshold | node_gain_threshold | zeta | rrt_max_sampled_nodes | fixed_z | observacoes |
|---|---:|---:|---:|---:|---:|---:|---:|---|---|
| fixedZ_seed1_20260505_222011 | 110 | 0.75 | 10 | 12 | 12 | 0.5 | 90 | true | voltou bastante para areas conhecidas |
| fixedZ_seed1_20260506_001207 | 85 | 0.60 | 12 | 8 | 18 | 0.8 | 120 | true | menos trajeto total (bugou faltando 9 min), ainda com bastante cached_branch |
| fixedZ_seed1_20260506_004133 | 110 | 0.60 | 14 | 5 | 22 | 0.8 | 160 | true | ajuste agressivo; validar se reduz vai-e-volta |
| fixedZ_seed1_20260506_014903 | 15 | 0.75 | 10 | 15 | 15 | 0.5 | 70 | true | alinhado aos YAMLs dos experimentos base |
| run5_pending | 85 | 0.60 | 12 | 8 | 18 | 0.8 | 120 | true | reduzido global_planner_counter=12 e look_ahead_horizon=7 para tentar diminuir vai-e-volta |

## Tabela de shape (atual vs proximo)

| profile | zero | lambda | max_sampling_radius | cache_node_threshold | node_gain_threshold | zeta | rrt_max_sampled_nodes | fixed_z | global_planner_counter | look_ahead_horizon | detector_ransac_min_inlier_ratio | detector_max_diameter |
|---|---:|---:|---:|---:|---:|---:|---:|---|---:|---:|---:|---:|
| shape_atual_with_walls | 85 | 0.60 | 12 | 8 | 18 | 0.8 | 120 | true | 12 | 7 | 0.45 | 0.50 |
| proximo_padrao_daep_outros_envs | 15 | 0.75 | 10 | 15 | 15 | 0.5 | 70 | true | 25 | 5 | 0.45 | 0.50 |

## Forest 1 (run atual vs run2 proposta)

| run | world | zero | lambda | max_sampling_radius | cache_node_threshold | node_gain_threshold | zeta | rrt_max_sampled_nodes | fixed_z | global_planner_counter | look_ahead_horizon | observacao |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|---:|---:|---|
| run1_atual (forest-trees-only_seed1_20260508_210037) | forest_1 | 110 | 0.75 | 10 | 12 | 12 | 0.5 | 90 | true | 25 | 5 | predominio de `cached_branch` (142/200), orbitagem local |
| run2_proposta (ajustes aplicados no yaml) | forest_1 | 110 | 0.75 | 14 | 18 | 15 | 0.5 | 130 | true | 10 | 5 | estimular abertura de fronteira/global e reduzir loop local |

## Notas

- O arquivo ativo modificado foi: `daep/catkin_ws/src/aeplanner/rpl_exploration/config/world_jean_exploration.yaml`
- Para comparar runs, usar principalmente:
  - `result/summary.md`
  - `result/metrics.json`
  - `data/path.csv`
  - `data/rrt_goal_log.csv`


Idle (parado tentando global e sem progresso)
A tese relata isso no AEP: dificuldade em alcançar cached nodes distantes, ficando sem progresso até bater tempo máximo.
Trecho: “...planner getting stuck... attempting to construct a path... no progress... remains idle until the maximum time limit is exceeded.” (daep-thesis, seção Stop Criterion, linhas ~3725–3733 extraídas)

Loop local/cached (não está parado, mas repete região)
A lógica do DAEP mantém branch anterior quando a melhoria é pequena e só aciona global sob condição de score/threshold.
Trechos:

“If the difference... is less than 10%, retain the old best node...” (linhas ~3195–3199)
“If the current best node has a dynamic score lower than gzero, the global planner is activated...” (linhas ~3200–3202)

Raios bloqueados por obstáculos reduzem ganho
Na tese (cap. método), eles descrevem explicitamente que os raios de ray-casting podem ser bloqueados por obstáculos estáticos, reduzindo ganho observável (trechos em torno de Fig. 5.6 / linhas ~3527–3529 no texto extraído).


No planner, o score do nó depende de mais coisas além de ser desconhecido:

Visibilidade efetiva
Se na borda o ponto “vê” pouco (oclusão por árvores/copa/estrutura), o ganho cai.

Custo/distância
O score penaliza caminho longo/ruim até o nó. Mesmo com desconhecido, pode perder para um nó mais perto no miolo.

Acúmulo ao longo do branch
Se já existe branch local bom (cached) com score alto, ele continua sendo favorecido.

Thresholds e regra de troca local/global
Enquanto o melhor nó local passa do limiar (gzero/zero_gain), ele não força migração global.
