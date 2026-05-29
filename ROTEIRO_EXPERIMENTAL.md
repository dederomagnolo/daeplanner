## Roteiro Experimental (organizado por `run_id`)

### 1) Iniciar uma run (gera ID + pastas)
```bash
./experiment_run.sh init --name baseline --seed k
```

Isso cria:
`/home/daep/experimentos/<run_id>/`

Subpastas principais:
- `data/` (tree_map, historicos, RRT e CSVs do planner)
- `tree_snapshots/` (PKL/PNG do plotter de cluster)
- `snapshots/` (snapshots manuais por tag/timestamp)
- `octomaps/`
- `result/`
- `logs/`

O contexto ativo da run fica em:
`/tmp/daeplanner_current_run.env`

### 2) Rodar exploração + tree stack (em terminais separados)
```bash
./simulation.sh world_jean
./exploration.sh world_jean_exploration.yaml k 2000
./run_tree_stack_realtime.sh /camera/depth/points world
```

Com o contexto ativo, os outputs vao automaticamente para a pasta da run, incluindo `data/rrt_tree_log.csv` e `data/rrt_goal_log.csv`.

### 3) Snapshot manual a qualquer momento
```bash
./experiment_run.sh snapshot
```

Ou com tag custom:
```bash
./experiment_run.sh snapshot --tag snapshot_t1200
```

Os snapshots manuais são salvos em:
`<run_dir>/snapshots/<tag>`

### 3.1) Snapshot periódico (ex.: a cada 10 minutos)
Iniciar em background:
```bash
./experiment_run.sh autosnapshot start --interval-sec 600
```

Ver status:
```bash
./experiment_run.sh autosnapshot status
```

Parar:
```bash
./experiment_run.sh autosnapshot stop
```

### 4) Fechar experimento (salva octomap primeiro + gera relatório final)
```bash
./experiment_run.sh finalize
```

Fluxo do `finalize`:
1. sincroniza CSV/JSON para `<run_dir>/data`
2. salva octomap final em `<run_dir>/octomaps`
3. roda `export_experiment_report.py` e grava em `<run_dir>/result`
4. valida e informa o caminho do resumo em Markdown: `<run_dir>/result/summary.md`

### 5) Verificar run ativa
```bash
./experiment_run.sh status
```

### 6) Reativar uma run antiga
```bash
./experiment_run.sh use --id <run_id>
```
