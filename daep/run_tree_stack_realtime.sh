#!/bin/bash
set -euo pipefail

# Usage:
#   ./run_tree_stack_realtime.sh [input_cloud_topic] [target_frame] [csv_out] [json_out] [snapshot_dir]
#
# Defaults are tuned for the usual DAEPlanner setup and axis range used in your plots.

input_cloud_topic="${1:-/camera/depth/points}"
target_frame="${2:-world}"
csv_out="${3:-/home/daep/data/tree_map_final.csv}"
json_out="${4:-/home/daep/data/tree_map_final.json}"
snapshot_dir="${5:-/home/daep/tree_snapshots}"

run_plotter="${RUN_PLOTTER:-true}"
run_cluster_plotter="${RUN_CLUSTER_PLOTTER:-true}"
run_fuser="${RUN_FUSER:-true}"

cd
source .bashrc

mkdir -p "$(dirname "${csv_out}")"
mkdir -p "$(dirname "${json_out}")"
mkdir -p "${snapshot_dir}"

echo "[tree_stack] input_cloud_topic=${input_cloud_topic}"
echo "[tree_stack] target_frame=${target_frame}"
echo "[tree_stack] csv_out=${csv_out}"
echo "[tree_stack] json_out=${json_out}"
echo "[tree_stack] snapshot_dir=${snapshot_dir}"

roslaunch tree_identifier tree_stack.launch \
  input_cloud_topic:="${input_cloud_topic}" \
  target_frame:="${target_frame}" \
  run_plotter:="${run_plotter}" \
  run_cluster_plotter:="${run_cluster_plotter}" \
  run_fuser:="${run_fuser}" \
  tracker_use_array_input:=true \
  fuser_use_array_input:=true \
  detector_clustering_mode:=dbscan_gmm \
  fuser_csv_output_path:="${csv_out}" \
  fuser_json_output_path:="${json_out}" \
  cluster_plotter_snapshot_dir:="${snapshot_dir}" \
  fixed_axes:=true \
  x_min:=-10 x_max:=10 \
  y_min:=-8 y_max:=7
