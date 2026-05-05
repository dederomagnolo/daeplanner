# Offline point-cloud exploration simulator

This folder contains a first offline simulator for quick DAEP/AEP-style
experiments without ROS/Gazebo.

The important modeling choice is:

- `ground_truth`: the full point cloud, hidden from the planner.
- `observed_map`: a voxel map that starts unknown and is revealed by a
  simulated camera FoV.

The planner only sees `observed_map`. Unknown voxels still contribute to the
information gain, so the run remains an exploration experiment rather than a
path through a known map.

## Quick smoke test

```bash
python3 daep/offline_sim/offline_explorer.py \
  --demo \
  --output /tmp/daep_offline_demo \
  --max-iterations 5 \
  --target-coverage-pct 5
```

## Run with a point cloud

For the default 10x10 scene, the simulator assumes:

- cloud: `daep/offline_sim/3d_scene_prepared_10x10.pcd`
- config: `daep/offline_sim/world_jean_offline_config.yaml`, aligned with `world_jean_exploration.yaml`
- start: `0,0,1.5,0`
- max iterations: `91` (same planner-decision budget observed in baseline-run-B seed 1)
- stop condition: max iterations only

So the usual run can be started with only the seed:

```bash
python3 daep/offline_sim/offline_explorer.py --seed 1
```

By default this writes to `daep/offline_sim/runs/offline-seed1`. To name a
baseline folder automatically:

```bash
python3 daep/offline_sim/offline_explorer.py --run-name baseline-1-offline --seed 1
```

This writes to `daep/offline_sim/runs/baseline-1-offline-seed1`. Use
`--output` only when you need a fully custom path.

```bash
python3 daep/offline_sim/offline_explorer.py \
  --cloud /path/to/map.xyz \
  --config daep/catkin_ws/src/aeplanner/rpl_exploration/config/granso_exploration.yaml \
  --output daep/offline_sim/runs/granso_seed1 \
  --start -4,-19,2.5,0 \
  --seed 1 \
  --max-iterations 200
```

The report is generated automatically when the simulation finishes:

```bash
python3 daep/offline_sim/offline_explorer.py --run-name baseline-1-offline --seed 1
```

Use `--no-report` only when you want to skip report generation.

The report reads tree ground-truth positions from the shared canonical CSV:
`daep/ground_truth/world_tree_ground_truth.csv`.

The simulator also uses that CSV for offline tree-observation metrics when it
is available. It runs an offline port of the tree detector over a synthetic
camera cloud, writes the detector candidates, and then matches those candidates
against the ground truth for evaluation.

Add `--report-copy-inputs` only when you want a self-contained report folder
with duplicated CSV/JSON inputs under `report/data/`.

Supported input formats:

- `.xyz`, `.txt`, `.csv`: first three numeric columns are interpreted as `x y z`.
- `.ply`: ASCII PLY with `x`, `y`, `z` vertex properties.
- `.pcd`: ASCII or binary PCD with `x`, `y`, `z` fields.
- `.obj`: vertex lines (`v x y z`).

## Outputs

The simulator writes:

- `path.csv`
- `logfile.csv`
- `coverage.csv`
- `tree_coverage.csv`
- `tree_detections.csv`
- `tree_observations.csv`
- `rrt_tree_log.csv`
- `rrt_goal_log.csv`
- `summary.json`

The CSV names and headers are close to the ROS experiment outputs, so the
existing report scripts can consume most of the data directly.

## Generate an offline report

The full ROS report script expects tree-detection artifacts that this offline
simulator does not generate. Use the offline report exporter for these runs:

```bash
python3 daep/offline_sim/export_offline_report.py \
  --run-dir daep/offline_sim/runs/granso_seed1 \
  --cloud daep/offline_sim/3d_scene_prepared_10x10.pcd \
  --config daep/catkin_ws/src/aeplanner/rpl_exploration/config/world_jean_exploration.yaml
```

It writes `report.md`, `route_trees_ground_truth.svg` (hidden point-cloud
projection + route + tree positions from the shared ground-truth CSV),
`world_tree_ground_truth.csv`, `coverage.svg`, `tree_coverage.svg`, `rrt_goal_score.svg`,
`rrt_tree_samples.svg` (RRT search-tree snapshots), `manifest.json`, and copied
CSV/JSON inputs under
`<run-dir>/report/` if `--copy-inputs` is used.

## Current scope

Implemented now:

- hidden point-cloud ground truth;
- observed voxel map;
- FoV/range raycasting;
- local RRT sampling;
- explicit gain calculation similar to `gainCubature`;
- simple frontier fallback when local gain is below the threshold;
- CSV logs for route, coverage, and RRT internals.

Not modeled yet:

- dynamic pedestrians;
- DFM updates;
- Kalman prediction and dynamic ray blocking;
- drone controller dynamics, latency, and Gazebo collisions.
