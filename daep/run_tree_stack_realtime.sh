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

extract_xy_bounds_from_planner_config() {
  local cfg="$1"
  if [[ -z "${cfg}" || ! -f "${cfg}" ]]; then
    return 1
  fi

  local min_line max_line
  min_line="$(grep -E '^[[:space:]]*boundary/min:' "${cfg}" | tail -n 1 || true)"
  max_line="$(grep -E '^[[:space:]]*boundary/max:' "${cfg}" | tail -n 1 || true)"
  if [[ -z "${min_line}" || -z "${max_line}" ]]; then
    return 1
  fi

  local min_vals max_vals
  min_vals="$(echo "${min_line}" | sed -E 's/.*\[(.*)\].*/\1/' | tr -d ' ')"
  max_vals="$(echo "${max_line}" | sed -E 's/.*\[(.*)\].*/\1/' | tr -d ' ')"

  local min_x min_y max_x max_y
  min_x="$(echo "${min_vals}" | cut -d',' -f1)"
  min_y="$(echo "${min_vals}" | cut -d',' -f2)"
  max_x="$(echo "${max_vals}" | cut -d',' -f1)"
  max_y="$(echo "${max_vals}" | cut -d',' -f2)"
  if [[ -z "${min_x}" || -z "${min_y}" || -z "${max_x}" || -z "${max_y}" ]]; then
    return 1
  fi

  x_min_plot="$(awk -v v="${min_x}" 'BEGIN{printf "%.6f", v-5.0}')"
  x_max_plot="$(awk -v v="${max_x}" 'BEGIN{printf "%.6f", v+5.0}')"
  y_min_plot="$(awk -v v="${min_y}" 'BEGIN{printf "%.6f", v-5.0}')"
  y_max_plot="$(awk -v v="${max_y}" 'BEGIN{printf "%.6f", v+5.0}')"
  return 0
}

# Usage:
#   ./run_tree_stack_realtime.sh [input_cloud_topic] [target_frame] [csv_out] [json_out] [snapshot_dir] [experiment_seed]
#
# Tree/map histories are taken from the active run context when available.
# snapshot_dir is kept only as the default/fallback location used by external snapshot capture helpers.
# Detector/tracker/fuser tuning comes from TREE_MAPPER_CONFIG_YAML.
# Defaults are tuned for the usual DAEPlanner setup and axis range used in your plots.

input_cloud_topic="${1:-/camera/depth/points}"
target_frame="${2:-world}"
run_plotter="${RUN_PLOTTER:-true}"
# Cluster plotter disabled by design in the default realtime stack flow.
# Keep this hardcoded to avoid accidental re-enable via old env vars.
run_cluster_plotter="false"
run_fuser="${RUN_FUSER:-true}"

cd
source .bashrc
load_experiment_context

default_data_dir="${EXPERIMENT_DATA_DIR:-/home/daep/experimentos/adhoc/data}"
default_snapshot_dir="${EXPERIMENT_SNAPSHOT_DIR:-/home/daep/snapshots}"
csv_out="${3:-${TREE_MAP_CSV_OUT:-${default_data_dir}/tree_map_final.csv}}"
json_out="${4:-${TREE_MAP_JSON_OUT:-${default_data_dir}/tree_map_final.json}}"
snapshot_dir="${5:-${default_snapshot_dir}}"
experiment_seed="${6:-${EXPERIMENT_SEED:--1}}"
tree_map_history_out="${TREE_MAP_HISTORY_CSV_OUT:-${default_data_dir}/tree_map_history.csv}"
tree_detection_history_out="${TREE_DETECTION_HISTORY_CSV_OUT:-${default_data_dir}/tree_detection_history.csv}"
run_id="${EXPERIMENT_RUN_ID:-}"
planner_config_path="${EXPERIMENT_PLANNER_CONFIG:-}"
tree_mapper_config_yaml="${TREE_MAPPER_CONFIG_YAML:-/home/daep/catkin_ws/src/tree_mapper/config/daeplanner.yaml}"
if [[ ! -f "${tree_mapper_config_yaml}" ]]; then
  echo "[tree_stack] error: TREE_MAPPER_CONFIG_YAML not found: ${tree_mapper_config_yaml}" >&2
  exit 1
fi
runtime_run_dir="${TREE_MAPPER_RUN_DIR:-${EXPERIMENT_RUN_DIR:-}}"
if [[ -z "${runtime_run_dir}" ]]; then
  csv_parent="$(dirname "${csv_out}")"
  if [[ "$(basename "${csv_parent}")" == "data" ]]; then
    runtime_run_dir="$(dirname "${csv_parent}")"
  fi
fi

x_min_plot=""
x_max_plot=""
y_min_plot=""
y_max_plot=""
if ! extract_xy_bounds_from_planner_config "${planner_config_path}"; then
  echo "[tree_stack] error: failed to read boundary/min,max from EXPERIMENT_PLANNER_CONFIG='${planner_config_path}'" >&2
  echo "[tree_stack] expected a valid YAML with boundary/min and boundary/max for fixed tree plot axes." >&2
  exit 1
fi

mkdir -p "$(dirname "${csv_out}")"
mkdir -p "$(dirname "${json_out}")"
mkdir -p "$(dirname "${tree_map_history_out}")"
mkdir -p "$(dirname "${tree_detection_history_out}")"
mkdir -p "${snapshot_dir}"
if [[ -n "${runtime_run_dir}" ]]; then
  mkdir -p "${runtime_run_dir}"
fi

echo "[tree_stack] input_cloud_topic=${input_cloud_topic}"
echo "[tree_stack] target_frame=${target_frame}"
echo "[tree_stack] csv_out=${csv_out}"
echo "[tree_stack] json_out=${json_out}"
echo "[tree_stack] tree_map_history_out=${tree_map_history_out}"
echo "[tree_stack] tree_detection_history_out=${tree_detection_history_out}"
echo "[tree_stack] snapshot_dir=${snapshot_dir}"
echo "[tree_stack] experiment_seed=${experiment_seed}"
echo "[tree_stack] config_yaml=${tree_mapper_config_yaml}"
echo "[tree_stack] runtime_run_dir=${runtime_run_dir:-"(auto)"}"
echo "[tree_stack] planner_config=${planner_config_path}"
echo "[tree_stack] plot_bounds_xy_plus5: x=[${x_min_plot}, ${x_max_plot}] y=[${y_min_plot}, ${y_max_plot}]"
if [[ -n "${run_id}" ]]; then
  echo "[tree_stack] run_id=${run_id}"
fi

roslaunch tree_mapper tree_stack_runtime.launch \
  config_yaml:="${tree_mapper_config_yaml}" \
  input_cloud_topic:="${input_cloud_topic}" \
  target_frame:="${target_frame}" \
  run_plotter:="${run_plotter}" \
  run_cluster_plotter:="${run_cluster_plotter}" \
  run_fuser:="${run_fuser}" \
  run_snapshot_exporter:=true \
  run_shrub_detector:=false \
  run_shrub_fuser:=false \
  experiment_seed:="${experiment_seed}" \
  run_output_dir:="${runtime_run_dir}" \
  run_id:="${run_id}" \
  fuser_csv_output_path:="${csv_out}" \
  fuser_json_output_path:="${json_out}" \
  fuser_history_csv_output_path:="${tree_map_history_out}" \
  detector_history_csv_output_path:="${tree_detection_history_out}" \
  fixed_axes:=true \
  x_min:="${x_min_plot}" x_max:="${x_max_plot}" \
  y_min:="${y_min_plot}" y_max:="${y_max_plot}"
