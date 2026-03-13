#!/bin/bash
set -euo pipefail

load_experiment_context() {
  local context_file="${EXPERIMENT_CONTEXT_FILE:-/tmp/daeplanner_current_run.env}"
  if [[ -f "${context_file}" ]]; then
    # shellcheck disable=SC1090
    set -a
    source "${context_file}"
    set +a
    echo "[tree_stack] loaded context: ${context_file}"
  fi
}

# Usage:
#   ./run_tree_stack_realtime.sh [input_cloud_topic] [target_frame] [csv_out] [json_out] [snapshot_dir] [experiment_seed]
#
# Defaults are tuned for the usual DAEPlanner setup and axis range used in your plots.

input_cloud_topic="${1:-/camera/depth/points}"
target_frame="${2:-world}"
run_plotter="${RUN_PLOTTER:-true}"
run_cluster_plotter="${RUN_CLUSTER_PLOTTER:-true}"
run_fuser="${RUN_FUSER:-true}"

cd
source .bashrc
load_experiment_context

default_data_dir="${EXPERIMENT_DATA_DIR:-/home/daep/data}"
default_snapshot_dir="${EXPERIMENT_SNAPSHOT_DIR:-/home/daep/tree_snapshots}"
csv_out="${3:-${TREE_MAP_CSV_OUT:-${default_data_dir}/tree_map_final.csv}}"
json_out="${4:-${TREE_MAP_JSON_OUT:-${default_data_dir}/tree_map_final.json}}"
snapshot_dir="${5:-${default_snapshot_dir}}"
experiment_seed="${6:-${EXPERIMENT_SEED:--1}}"
run_id="${EXPERIMENT_RUN_ID:-}"

mkdir -p "$(dirname "${csv_out}")"
mkdir -p "$(dirname "${json_out}")"
mkdir -p "${snapshot_dir}"

echo "[tree_stack] input_cloud_topic=${input_cloud_topic}"
echo "[tree_stack] target_frame=${target_frame}"
echo "[tree_stack] csv_out=${csv_out}"
echo "[tree_stack] json_out=${json_out}"
echo "[tree_stack] snapshot_dir=${snapshot_dir}"
echo "[tree_stack] experiment_seed=${experiment_seed}"
if [[ -n "${run_id}" ]]; then
  echo "[tree_stack] run_id=${run_id}"
fi

roslaunch tree_identifier tree_stack.launch \
  input_cloud_topic:="${input_cloud_topic}" \
  target_frame:="${target_frame}" \
  run_plotter:="${run_plotter}" \
  run_cluster_plotter:="${run_cluster_plotter}" \
  run_fuser:="${run_fuser}" \
  tracker_use_array_input:=true \
  fuser_use_array_input:=true \
  detector_clustering_mode:=dbscan_gmm \
  experiment_seed:="${experiment_seed}" \
  fuser_csv_output_path:="${csv_out}" \
  fuser_json_output_path:="${json_out}" \
  cluster_plotter_snapshot_dir:="${snapshot_dir}" \
  fixed_axes:=true \
  x_min:=-10 x_max:=10 \
  y_min:=-8 y_max:=7
