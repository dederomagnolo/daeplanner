#!/bin/bash
set -euo pipefail

# Usage:
#   ./save_tree_runtime_state.sh [output_root] [tag]
#
# Examples:
#   ./save_tree_runtime_state.sh
#   ./save_tree_runtime_state.sh /home/daep/meus-resultados/world-jean exp01_t120
#
# Environment overrides:
#   SNAPSHOT_TOPIC  (default: /tree_cluster_xy_plotter/save_snapshot)
#   SNAPSHOT_DIR    (default: /home/daep/tree_snapshots)
#   CSV_SOURCE      (default: /home/daep/data/tree_map_final.csv)
#   JSON_SOURCE     (default: /home/daep/data/tree_map_final.json)
#   OCTOMAP_TOPIC   (default: /aeplanner/octomap_full)
#   SAVE_OCTOMAP    (default: true)

output_root="${1:-/home/daep/meus-resultados/runtime_saves}"
tag="${2:-$(date +%Y%m%d_%H%M%S)}"

snapshot_topic="${SNAPSHOT_TOPIC:-/tree_cluster_xy_plotter/save_snapshot}"
snapshot_dir="${SNAPSHOT_DIR:-/home/daep/tree_snapshots}"
csv_source="${CSV_SOURCE:-/home/daep/data/tree_map_final.csv}"
json_source="${JSON_SOURCE:-/home/daep/data/tree_map_final.json}"
octomap_topic="${OCTOMAP_TOPIC:-/aeplanner/octomap_full}"
save_octomap="${SAVE_OCTOMAP:-true}"

cd
source .bashrc

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
if [ -n "${latest_pkl}" ] && [ -f "${latest_pkl}" ]; then
  cp "${latest_pkl}" "${dest}/"
  echo "[save] copied $(basename "${latest_pkl}")"
else
  echo "[warn] no PKL found in ${snapshot_dir}"
fi

latest_png="$(ls -1t "${snapshot_dir}"/*.png 2>/dev/null | head -n1 || true)"
if [ -n "${latest_png}" ] && [ -f "${latest_png}" ]; then
  cp "${latest_png}" "${dest}/"
  echo "[save] copied $(basename "${latest_png}")"
else
  echo "[warn] no PNG found in ${snapshot_dir}"
fi

if [ -f "${csv_source}" ]; then
  cp "${csv_source}" "${dest}/"
  echo "[save] copied $(basename "${csv_source}")"
else
  echo "[warn] CSV not found: ${csv_source}"
fi

if [ -f "${json_source}" ]; then
  cp "${json_source}" "${dest}/"
  echo "[save] copied $(basename "${json_source}")"
else
  echo "[warn] JSON not found: ${json_source}"
fi

if [ "${save_octomap}" = "true" ]; then
  if topic_exists "${octomap_topic}"; then
    rosrun octomap_server octomap_saver -f "${dest}/octomap_${tag}" octomap_full:="${octomap_topic}" >/dev/null
    echo "[save] octomap saved: ${dest}/octomap_${tag}.bt"
  else
    echo "[warn] octomap topic not found: ${octomap_topic}"
  fi
fi

saved_csv="${dest}/$(basename "${csv_source}")"
if [ -f "${saved_csv}" ]; then
  tree_count="$(awk 'NR > 1 && NF > 0 {c++} END {print c + 0}' "${saved_csv}")"
  echo "[save] csv_tree_count=${tree_count}"
fi

echo "[save] done"
ls -1 "${dest}"
