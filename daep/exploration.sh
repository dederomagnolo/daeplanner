#!/bin/bash
set -euo pipefail

cd
source .bashrc

if [[ $# -lt 1 ]]; then
  echo "Usage: exploration.sh <config_file> [seed] [max_time_sec]"
  echo "   or: exploration.sh <config_file> [seed] --max-time <seconds>"
  exit 1
fi

config="$1"
shift || true

seed=""
max_time_sec="${EXPLORATION_MAX_TIME:-}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --max-time|--max_time|--timeout)
      if [[ $# -lt 2 ]]; then
        echo "Error: missing value for $1"
        exit 1
      fi
      max_time_sec="$2"
      shift 2
      ;;
    *)
      # Backward compatible positional parsing:
      # exploration.sh <config> <seed> <max_time_sec>
      if [[ -z "$seed" ]]; then
        seed="$1"
      elif [[ -z "$max_time_sec" ]]; then
        max_time_sec="$1"
      else
        echo "Error: unexpected extra argument: $1"
        exit 1
      fi
      shift
      ;;
  esac
done

launch_cmd=(roslaunch rpl_exploration exploration.launch "config_file:=$config")
if [[ -n "$seed" && "$seed" =~ ^-?[0-9]+$ ]]; then
  launch_cmd+=("experiment_seed:=$seed")
fi

save_octomap_with_seed_and_timestamp() {
  local ts seed_tag config_tag octomap_name octomap_path
  ts="$(date +%Y%m%d_%H%M%S)"
  if [[ -n "$seed" && "$seed" =~ ^-?[0-9]+$ ]]; then
    seed_tag="seed${seed}"
  else
    seed_tag="seedNA"
  fi
  config_tag="$(basename "$config")"
  config_tag="${config_tag%.*}"
  config_tag="${config_tag//[^a-zA-Z0-9_-]/_}"

  octomap_name="octomap_${config_tag}_${seed_tag}_${ts}"
  octomap_path="/home/daep/octomaps/${octomap_name}"
  mkdir -p /home/daep/octomaps

  if rostopic list 2>/dev/null | grep -qx "/aeplanner/octomap_full"; then
    echo "Saving octomap: ${octomap_name}.bt"
    if rosrun octomap_server octomap_saver -f "$octomap_path" octomap_full:=/aeplanner/octomap_full >/dev/null; then
      echo "Octomap saved at: ${octomap_path}.bt"
    else
      echo "Warning: failed to save octomap from /aeplanner/octomap_full"
    fi
  else
    echo "Warning: topic /aeplanner/octomap_full not available; octomap not saved"
  fi
}

# Legacy behavior: no timeout => keep roslaunch in foreground.
if [[ -z "$max_time_sec" ]]; then
  "${launch_cmd[@]}"
  exit $?
fi

if ! [[ "$max_time_sec" =~ ^[0-9]+$ ]] || (( max_time_sec <= 0 )); then
  echo "Error: max_time_sec must be a positive integer. Got: $max_time_sec"
  exit 1
fi

echo "Running exploration with stop criterion:"
echo "- config: $config"
if [[ -n "$seed" ]]; then
  echo "- seed: $seed"
fi
echo "- max_time_sec: $max_time_sec"

"${launch_cmd[@]}" &
launch_pid=$!

completion_flag="$(mktemp)"
cleanup() {
  rm -f "$completion_flag"
}
trap cleanup EXIT

# Stream /exploration_completed and set a local flag when it reaches True.
(
  rostopic echo /exploration_completed 2>/dev/null | while IFS= read -r line; do
    if [[ "$line" == *"data: True"* ]]; then
      echo "true" > "$completion_flag"
      break
    fi
  done
) &
echo_pid=$!

stop_echo() {
  kill "$echo_pid" 2>/dev/null || true
  wait "$echo_pid" 2>/dev/null || true
}

deadline=$(( $(date +%s) + max_time_sec ))
last_minutes_remaining=""
reason=""

while kill -0 "$launch_pid" 2>/dev/null; do
  if [[ -s "$completion_flag" ]]; then
    reason="completed"
    break
  fi

  now=$(date +%s)
  if (( now >= deadline )); then
    reason="timeout"
    break
  fi

  minutes_remaining=$(( (deadline - now + 59) / 60 ))
  if [[ "$minutes_remaining" != "$last_minutes_remaining" ]]; then
    echo "Minutes remaining: $minutes_remaining"
    last_minutes_remaining="$minutes_remaining"
  fi

  sleep 1
done

stop_echo

if [[ "$reason" == "completed" ]]; then
  echo "Planner finished early (/exploration_completed=True)."
  save_octomap_with_seed_and_timestamp
  kill -INT "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
  exit 0
fi

if [[ "$reason" == "timeout" ]]; then
  echo "Time is up (${max_time_sec}s). Stopping exploration launch."
  save_octomap_with_seed_and_timestamp
  kill -INT "$launch_pid" 2>/dev/null || true
  sleep 2
  kill -TERM "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
  exit 0
fi

# roslaunch exited by itself
wait "$launch_pid"
