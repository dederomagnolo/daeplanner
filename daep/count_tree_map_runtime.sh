#!/bin/bash
set -euo pipefail

# Usage:
#   ./count_tree_map_runtime.sh [topic_base]
#
# Default topic_base:
#   /tree_map_ids

topic_base="${1:-/tree_map_ids}"
data_topic="${topic_base}/data"

cd
source .bashrc

if ! rostopic list 2>/dev/null | grep -Fxq "${topic_base}"; then
  echo "topic_not_found=${topic_base}"
  exit 1
fi

raw="$(rostopic echo -n1 "${data_topic}" 2>/dev/null || true)"

trimmed="$(echo "${raw}" | tr -d ' \r\n\t')"
if [ -z "${trimmed}" ] || [ "${trimmed}" = "[]" ]; then
  echo "count=0"
  echo "ids=[]"
  exit 0
fi

if echo "${raw}" | grep -q '^- '; then
  ids="$(echo "${raw}" | sed -n 's/^- //p' | paste -sd, -)"
  count="$(echo "${raw}" | sed -n 's/^- //p' | wc -l | tr -d ' ')"
else
  ids="$(echo "${raw}" | tr -d ' \r\n\t[]')"
  if [ -z "${ids}" ]; then
    count=0
  else
    count="$(echo "${ids}" | awk -F',' '{print NF}')"
  fi
fi

if [ -z "${ids}" ]; then
  echo "count=0"
  echo "ids=[]"
  exit 0
fi

echo "count=${count}"
echo "ids=[${ids}]"
