#!/bin/bash
set -euo pipefail

# Usage:
#   ./save_tree_runtime_state.sh [output_root] [tag]
#
# Examples:
#   ./save_tree_runtime_state.sh
#   ./save_tree_runtime_state.sh /home/daep/meus-resultados/world-jean snapshot_t120
#
# Context-aware behavior:
#   If /tmp/daeplanner_current_run.env exists, defaults are taken from it.
#   This makes snapshots deterministic per run_id (no global "latest experiment" confusion).
#
# Environment overrides:
#   EXPERIMENT_CONTEXT_FILE (default: /tmp/daeplanner_current_run.env)
#   SNAPSHOT_TOPIC          (default: /tree_cluster_xy_plotter/save_snapshot)
#   SNAPSHOT_DIR            (default: /home/daep/tree_snapshots)
#   CSV_SOURCE              (explicit tree_map_final.csv path)
#   JSON_SOURCE             (explicit tree_map_final.json path)
#   EXPERIMENT_DATA_DIR     (preferred data directory)
#   EXPERIMENT_SOURCE_DATA_DIR (fallback data directory, default: /home/daep/data)
#   OCTOMAP_TOPIC           (default: /aeplanner/octomap_full)
#   SAVE_OCTOMAP            (default: true)
#   EXTRA_DATA_FILES        (space-separated list, defaults include planner CSVs)

load_experiment_context() {
  local context_file="${EXPERIMENT_CONTEXT_FILE:-/tmp/daeplanner_current_run.env}"
  if [[ -f "${context_file}" ]]; then
    # shellcheck disable=SC1090
    set -a
    source "${context_file}"
    set +a
    echo "[save] loaded context: ${context_file}"
  fi
}

pick_data_file() {
  local file_name="$1"
  if [[ -n "${EXPERIMENT_DATA_DIR:-}" && -f "${EXPERIMENT_DATA_DIR}/${file_name}" ]]; then
    echo "${EXPERIMENT_DATA_DIR}/${file_name}"
    return 0
  fi
  if [[ -n "${EXPERIMENT_SOURCE_DATA_DIR:-}" && -f "${EXPERIMENT_SOURCE_DATA_DIR}/${file_name}" ]]; then
    echo "${EXPERIMENT_SOURCE_DATA_DIR}/${file_name}"
    return 0
  fi
  if [[ -f "/home/daep/data/${file_name}" ]]; then
    echo "/home/daep/data/${file_name}"
    return 0
  fi
  echo ""
}

copy_if_exists() {
  local src="$1"
  local dst_dir="$2"
  if [[ -n "${src}" && -f "${src}" ]]; then
    cp "${src}" "${dst_dir}/"
    echo "[save] copied $(basename "${src}")"
  fi
}

cd
source .bashrc
load_experiment_context

default_output_root="/home/daep/meus-resultados/runtime_saves"
if [[ -n "${EXPERIMENT_RUN_DIR:-}" ]]; then
  default_output_root="${EXPERIMENT_RUN_DIR}/snapshots"
fi

output_root="${1:-${default_output_root}}"
tag="${2:-snapshot_$(date +%Y%m%d_%H%M%S)}"

snapshot_topic="${SNAPSHOT_TOPIC:-/tree_cluster_xy_plotter/save_snapshot}"
snapshot_dir="${SNAPSHOT_DIR:-${EXPERIMENT_SNAPSHOT_DIR:-/home/daep/tree_snapshots}}"
octomap_topic="${OCTOMAP_TOPIC:-${EXPERIMENT_OCTOMAP_TOPIC:-/aeplanner/octomap_full}}"
save_octomap="${SAVE_OCTOMAP:-true}"

csv_source="${CSV_SOURCE:-}"
if [[ -z "${csv_source}" ]]; then
  csv_source="$(pick_data_file tree_map_final.csv)"
fi

json_source="${JSON_SOURCE:-}"
if [[ -z "${json_source}" ]]; then
  json_source="$(pick_data_file tree_map_final.json)"
fi

extra_data_files="${EXTRA_DATA_FILES:-coverage.csv path.csv logfile.csv intervals.csv collision.csv}"

dest="${output_root}/${tag}"
mkdir -p "${dest}"

topic_exists() {
  local topic_name="$1"
  rostopic list 2>/dev/null | grep -Fxq "${topic_name}"
}

echo "[save] destination=${dest}"

if topic_exists "${snapshot_topic}"; then
  rostopic pub -1 "${snapshot_topic}" std_msgs/Empty "{}" >/dev/null 2>&1 || true
  sleep 1
  echo "[save] snapshot requested on ${snapshot_topic}"
else
  echo "[warn] snapshot topic not found: ${snapshot_topic}"
fi

latest_pkl="$(ls -1t "${snapshot_dir}"/*.pkl 2>/dev/null | head -n1 || true)"
if [[ -n "${latest_pkl}" && -f "${latest_pkl}" ]]; then
  cp "${latest_pkl}" "${dest}/"
  echo "[save] copied $(basename "${latest_pkl}")"
else
  echo "[warn] no PKL found in ${snapshot_dir}"
fi

latest_png="$(ls -1t "${snapshot_dir}"/*.png 2>/dev/null | head -n1 || true)"
if [[ -n "${latest_png}" && -f "${latest_png}" ]]; then
  cp "${latest_png}" "${dest}/"
  echo "[save] copied $(basename "${latest_png}")"
else
  echo "[warn] no PNG found in ${snapshot_dir}"
fi

if [[ -n "${csv_source}" && -f "${csv_source}" ]]; then
  cp "${csv_source}" "${dest}/"
  echo "[save] copied $(basename "${csv_source}")"
else
  echo "[warn] CSV not found: ${csv_source:-tree_map_final.csv}"
fi

if [[ -n "${json_source}" && -f "${json_source}" ]]; then
  cp "${json_source}" "${dest}/"
  echo "[save] copied $(basename "${json_source}")"
else
  echo "[warn] JSON not found: ${json_source:-tree_map_final.json}"
fi

for file_name in ${extra_data_files}; do
  src="$(pick_data_file "${file_name}")"
  if [[ -n "${src}" ]]; then
    copy_if_exists "${src}" "${dest}"
  else
    echo "[warn] extra data file not found: ${file_name}"
  fi
done

if [[ "${save_octomap}" = "true" ]]; then
  if topic_exists "${octomap_topic}"; then
    rosrun octomap_server octomap_saver -f "${dest}/octomap_${tag}" "octomap_full:=${octomap_topic}" >/dev/null || true
    if [[ -s "${dest}/octomap_${tag}.bt" ]]; then
      echo "[save] octomap saved: ${dest}/octomap_${tag}.bt"
    else
      echo "[warn] octomap save requested but file not created"
    fi
  else
    echo "[warn] octomap topic not found: ${octomap_topic}"
  fi
fi

saved_csv="${dest}/$(basename "${csv_source:-tree_map_final.csv}")"
if [[ -f "${saved_csv}" ]]; then
  tree_count="$(awk 'NR > 1 && NF > 0 {c++} END {print c + 0}' "${saved_csv}")"
  echo "[save] csv_tree_count=${tree_count}"
fi

echo "[save] done"
ls -1 "${dest}"
