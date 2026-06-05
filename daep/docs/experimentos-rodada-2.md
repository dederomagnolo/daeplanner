# Experimentos - Rodada 2 (world_jean)

## Tabela de Runs
| run_id | aep_gain_zero | aep_lambda | aep_r_max | max_sampled_nodes | cache_node_threshold | node_gain_threshold | boost_magnitude | global_planner_counter | coverage_pct | descoberta_tp |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| rodada2-wjean_seed1_20260601_234157 | 60 | 0.3 | 6 | 70 | 50 | 50 | 6 | 25 | 78.75 | 17 |
| rodada2-wjean_seed1_20260602_014606 | 60 | 0.2 | 6 | 70 | 50 | 50 | 6 | 25 | 78.21 | 16 |
| rodada2-wjean_seed1_20260602_130032 | 60 | 0.4 | 6 | 70 | 60 | 60 | 6 | 25 | 77.03 | 15 |
| rodada2-wjean_seed1_20260602_142209 | 60 | 0.3 | 6 | 70 | 50 | 50 | 6 | 25 | 79.73 | 18 |

## Referencia Rapida (config atual)
- `aep/gain/zero: 60`
- `aep/gain/lambda: 0.3`
- `aep/gain/r_max: 6`
- `daep/rrt/max_sampled_nodes: 70`
- `daep/gain/cache_node_threshold: 50`
- `daep/gain/node_gain_threshold: 50`
- `boost_magnitude: 6`
- `daep/fixed_z_from_start: false`
- `global_planner_counter: 25`
