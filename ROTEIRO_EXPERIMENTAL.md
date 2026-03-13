## Roteiro Experimental (organizado por `run_id`)

### 1) Iniciar uma run (gera ID + pastas)
```bash
./daep/experiment_run.sh init --name baseline --seed k
```

Isso cria:
`/home/daep/experimentos/runs/<run_id>/`

Subpastas principais:
- `data/` (tree_map e CSVs do planner)
- `tree_snapshots/` (PKL/PNG do plotter de cluster)
- `snapshots/` (snapshots manuais por tag/timestamp)
- `octomaps/`
- `result/`
- `logs/`

O contexto ativo da run fica em:
`/tmp/daeplanner_current_run.env`

### 2) Rodar exploração + tree stack (em terminais separados)
```bash
./daep/simulation.sh world_jean
./daep/exploration.sh world_jean_exploration.yaml k 2000
./daep/run_tree_stack_realtime.sh /camera/depth/points world
```

Com o contexto ativo, os outputs vão automaticamente para a pasta da run.

### 3) Snapshot manual a qualquer momento
```bash
./daep/experiment_run.sh snapshot
```

Ou com tag custom:
```bash
./daep/experiment_run.sh snapshot --tag snapshot_t1200
```

Os snapshots manuais são salvos em:
`<run_dir>/snapshots/<tag>`

### 4) Fechar experimento (salva octomap primeiro + gera relatório final)
```bash
./daep/experiment_run.sh finalize
```

Fluxo do `finalize`:
1. sincroniza CSV/JSON para `<run_dir>/data`
2. salva octomap final em `<run_dir>/octomaps`
3. roda `export_experiment_report.py` e grava em `<run_dir>/result`

### 5) Verificar run ativa
```bash
./daep/experiment_run.sh status
```

### 6) Reativar uma run antiga
```bash
./daep/experiment_run.sh use --id <run_id>
```
