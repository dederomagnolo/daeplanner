#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_BASE_DIR="/home/daep/experimentos/runs"
DEFAULT_CONTEXT_FILE="/tmp/daeplanner_current_run.env"

print_usage() {
  cat <<'EOF'
Usage:
  ./experiment_run.sh init --name <experiment_name> [--seed <n>] [--id <run_id>] [--base-dir <dir>]
  ./experiment_run.sh use [--id <run_id> | --run-dir <dir>] [--base-dir <dir>]
  ./experiment_run.sh status
  ./experiment_run.sh snapshot [--tag <snapshot_tag>] [--save-octomap true|false]
  ./experiment_run.sh autosnapshot start [--interval-sec <n>] [--save-octomap true|false] [--prefix <tag_prefix>]
  ./experiment_run.sh autosnapshot stop
  ./experiment_run.sh autosnapshot status
  ./experiment_run.sh finalize [--report-name <name>] [--skip-octomap] [--skip-report]

Examples:
  ./experiment_run.sh init --name baseline --seed 7
  ./experiment_run.sh snapshot
  ./experiment_run.sh autosnapshot start --interval-sec 600
  ./experiment_run.sh finalize --report-name result
EOF
}

context_file() {
  echo "${EXPERIMENT_CONTEXT_FILE:-${DEFAULT_CONTEXT_FILE}}"
}

sanitize_token() {
  local value="$1"
  value="${value// /_}"
  value="${value//[^a-zA-Z0-9_.-]/_}"
  value="${value##[_-]}"
  value="${value%%[_-]}"
  if [[ -z "${value}" ]]; then
    value="exp"
  fi
  echo "${value}"
}

timestamp() {
  date +%Y%m%d_%H%M%S
}

topic_exists() {
  local topic_name="$1"
  rostopic list 2>/dev/null | grep -Fxq "${topic_name}"
}

load_context_or_fail() {
  local ctx
  ctx="$(context_file)"
  if [[ ! -f "${ctx}" ]]; then
    echo "No active experiment context found: ${ctx}" >&2
    echo "Run: ./experiment_run.sh init --name <name> [--seed <n>]" >&2
    exit 1
  fi
  # shellcheck disable=SC1090
  set -a
  source "${ctx}"
  set +a
}

write_context_file() {
  local ctx_path="$1"
  {
    printf 'EXPERIMENT_RUN_ID=%q\n' "${EXPERIMENT_RUN_ID}"
    printf 'EXPERIMENT_NAME=%q\n' "${EXPERIMENT_NAME}"
    printf 'EXPERIMENT_SEED=%q\n' "${EXPERIMENT_SEED}"
    printf 'EXPERIMENT_RUN_DIR=%q\n' "${EXPERIMENT_RUN_DIR}"
    printf 'EXPERIMENT_DATA_DIR=%q\n' "${EXPERIMENT_DATA_DIR}"
    printf 'EXPERIMENT_SOURCE_DATA_DIR=%q\n' "${EXPERIMENT_SOURCE_DATA_DIR}"
    printf 'EXPERIMENT_SNAPSHOT_DIR=%q\n' "${EXPERIMENT_SNAPSHOT_DIR}"
    printf 'EXPERIMENT_OCTOMAP_DIR=%q\n' "${EXPERIMENT_OCTOMAP_DIR}"
    printf 'EXPERIMENT_OCTOMAP_TOPIC=%q\n' "${EXPERIMENT_OCTOMAP_TOPIC}"
    printf 'EXPERIMENT_RESULT_DIR=%q\n' "${EXPERIMENT_RESULT_DIR}"
    printf 'EXPERIMENT_CONTEXT_FILE=%q\n' "${ctx_path}"
    printf 'TREE_MAP_CSV_OUT=%q\n' "${EXPERIMENT_DATA_DIR}/tree_map_final.csv"
    printf 'TREE_MAP_JSON_OUT=%q\n' "${EXPERIMENT_DATA_DIR}/tree_map_final.json"
  } > "${ctx_path}"
}

sync_data_into_run() {
  local files=(
    tree_map_final.csv
    tree_map_final.json
    coverage.csv
    path.csv
    logfile.csv
    intervals.csv
    collision.csv
  )
  mkdir -p "${EXPERIMENT_DATA_DIR}"
  local src_base="${EXPERIMENT_SOURCE_DATA_DIR:-/home/daep/data}"
  for file_name in "${files[@]}"; do
    local src="${src_base}/${file_name}"
    if [[ -f "${src}" ]]; then
      cp "${src}" "${EXPERIMENT_DATA_DIR}/${file_name}"
    fi
  done
}

save_octomap_now() {
  local suffix="$1"
  local octomap_topic="${EXPERIMENT_OCTOMAP_TOPIC:-/aeplanner/octomap_full}"
  local ts out_base out_file
  ts="$(timestamp)"
  out_base="${EXPERIMENT_OCTOMAP_DIR}/octomap_${EXPERIMENT_RUN_ID}_${suffix}_${ts}"
  out_file="${out_base}.bt"
  mkdir -p "${EXPERIMENT_OCTOMAP_DIR}"
  if ! topic_exists "${octomap_topic}"; then
    echo "[finalize] octomap topic not found: ${octomap_topic}"
    return 1
  fi
  if rosrun octomap_server octomap_saver -f "${out_file}" "octomap_full:=${octomap_topic}" >/dev/null \
     && [[ -s "${out_file}" ]]; then
    echo "[finalize] octomap saved: ${out_file}"
    return 0
  fi
  echo "[finalize] failed to save octomap: ${out_file}" >&2
  return 1
}

autosnapshot_pid_file() {
  echo "${EXPERIMENT_RUN_DIR}/logs/autosnapshot.pid"
}

autosnapshot_log_file() {
  echo "${EXPERIMENT_RUN_DIR}/logs/autosnapshot.log"
}

is_pid_running() {
  local pid="${1:-}"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null
}

run_snapshot_capture() {
  local tag="$1"
  local save_octomap="$2"
  sync_data_into_run
  SAVE_OCTOMAP="${save_octomap}" \
  SNAPSHOT_DIR="${EXPERIMENT_SNAPSHOT_DIR}" \
  CSV_SOURCE="${EXPERIMENT_DATA_DIR}/tree_map_final.csv" \
  JSON_SOURCE="${EXPERIMENT_DATA_DIR}/tree_map_final.json" \
  EXPERIMENT_SOURCE_DATA_DIR="${EXPERIMENT_SOURCE_DATA_DIR}" \
  "${SCRIPT_DIR}/save_tree_runtime_state.sh" "${EXPERIMENT_RUN_DIR}/snapshots" "${tag}"
}

cmd_init() {
  local name=""
  local seed="-1"
  local run_id=""
  local base_dir="${DEFAULT_BASE_DIR}"

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --name)
        name="${2:-}"
        shift 2
        ;;
      --seed)
        seed="${2:-}"
        shift 2
        ;;
      --id|--run-id)
        run_id="${2:-}"
        shift 2
        ;;
      --base-dir)
        base_dir="${2:-}"
        shift 2
        ;;
      *)
        echo "Unknown option for init: $1" >&2
        exit 1
        ;;
    esac
  done

  if [[ -z "${name}" ]]; then
    echo "init requires --name <experiment_name>" >&2
    exit 1
  fi
  if [[ -n "${seed}" && ! "${seed}" =~ ^-?[0-9]+$ ]]; then
    echo "seed must be integer, got: ${seed}" >&2
    exit 1
  fi

  local name_token
  name_token="$(sanitize_token "${name}")"
  if [[ -z "${run_id}" ]]; then
    if [[ "${seed}" =~ ^-?[0-9]+$ && "${seed}" != "-1" ]]; then
      run_id="${name_token}_seed${seed}_$(timestamp)"
    else
      run_id="${name_token}_$(timestamp)"
    fi
  fi
  run_id="$(sanitize_token "${run_id}")"

  EXPERIMENT_RUN_ID="${run_id}"
  EXPERIMENT_NAME="${name}"
  EXPERIMENT_SEED="${seed}"
  EXPERIMENT_RUN_DIR="${base_dir}/${run_id}"
  EXPERIMENT_DATA_DIR="${EXPERIMENT_RUN_DIR}/data"
  EXPERIMENT_SOURCE_DATA_DIR="/home/daep/data"
  EXPERIMENT_SNAPSHOT_DIR="${EXPERIMENT_RUN_DIR}/tree_snapshots"
  EXPERIMENT_OCTOMAP_DIR="${EXPERIMENT_RUN_DIR}/octomaps"
  EXPERIMENT_OCTOMAP_TOPIC="/aeplanner/octomap_full"
  EXPERIMENT_RESULT_DIR="${EXPERIMENT_RUN_DIR}/result"

  mkdir -p \
    "${EXPERIMENT_RUN_DIR}" \
    "${EXPERIMENT_DATA_DIR}" \
    "${EXPERIMENT_SNAPSHOT_DIR}" \
    "${EXPERIMENT_OCTOMAP_DIR}" \
    "${EXPERIMENT_RUN_DIR}/snapshots" \
    "${EXPERIMENT_RESULT_DIR}" \
    "${EXPERIMENT_RUN_DIR}/logs"

  cat > "${EXPERIMENT_RUN_DIR}/run_meta.json" <<EOF
{
  "run_id": "${EXPERIMENT_RUN_ID}",
  "experiment_name": "${EXPERIMENT_NAME}",
  "seed": ${EXPERIMENT_SEED},
  "created_at": "$(date --iso-8601=seconds)"
}
EOF

  local ctx
  ctx="$(context_file)"
  write_context_file "${ctx}"
  write_context_file "${EXPERIMENT_RUN_DIR}/run_context.env"

  echo "[init] run_id=${EXPERIMENT_RUN_ID}"
  echo "[init] run_dir=${EXPERIMENT_RUN_DIR}"
  echo "[init] context=${ctx}"
  echo "[init] snapshot_dir=${EXPERIMENT_SNAPSHOT_DIR}"
  echo "[init] data_dir=${EXPERIMENT_DATA_DIR}"
}

cmd_use() {
  local run_id=""
  local run_dir=""
  local base_dir="${DEFAULT_BASE_DIR}"

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --id|--run-id)
        run_id="${2:-}"
        shift 2
        ;;
      --run-dir)
        run_dir="${2:-}"
        shift 2
        ;;
      --base-dir)
        base_dir="${2:-}"
        shift 2
        ;;
      *)
        echo "Unknown option for use: $1" >&2
        exit 1
        ;;
    esac
  done

  if [[ -z "${run_dir}" ]]; then
    if [[ -z "${run_id}" ]]; then
      echo "use requires --id <run_id> or --run-dir <dir>" >&2
      exit 1
    fi
    run_dir="${base_dir}/${run_id}"
  fi

  if [[ ! -d "${run_dir}" ]]; then
    echo "run_dir not found: ${run_dir}" >&2
    exit 1
  fi

  local src_ctx="${run_dir}/run_context.env"
  local dst_ctx
  dst_ctx="$(context_file)"
  if [[ ! -f "${src_ctx}" ]]; then
    echo "run context missing in run_dir: ${src_ctx}" >&2
    exit 1
  fi
  cp "${src_ctx}" "${dst_ctx}"
  echo "[use] active context=${dst_ctx}"
  echo "[use] run_dir=${run_dir}"
}

cmd_status() {
  load_context_or_fail
  echo "run_id=${EXPERIMENT_RUN_ID}"
  echo "experiment_name=${EXPERIMENT_NAME}"
  echo "seed=${EXPERIMENT_SEED}"
  echo "run_dir=${EXPERIMENT_RUN_DIR}"
  echo "data_dir=${EXPERIMENT_DATA_DIR}"
  echo "source_data_dir=${EXPERIMENT_SOURCE_DATA_DIR}"
  echo "snapshot_dir=${EXPERIMENT_SNAPSHOT_DIR}"
  echo "octomap_dir=${EXPERIMENT_OCTOMAP_DIR}"
  echo "result_dir=${EXPERIMENT_RESULT_DIR}"
  if [[ -f "${EXPERIMENT_DATA_DIR}/tree_map_final.csv" ]]; then
    local n
    n="$(awk 'NR > 1 && NF > 0 {c++} END {print c + 0}' "${EXPERIMENT_DATA_DIR}/tree_map_final.csv")"
    echo "tree_map_count=${n}"
  fi
}

cmd_snapshot() {
  load_context_or_fail
  local tag="snapshot_$(timestamp)"
  local save_octomap="false"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --tag)
        tag="${2:-}"
        shift 2
        ;;
      --save-octomap)
        save_octomap="${2:-false}"
        shift 2
        ;;
      *)
        echo "Unknown option for snapshot: $1" >&2
        exit 1
        ;;
    esac
  done
  run_snapshot_capture "${tag}" "${save_octomap}"
}

cmd_autosnapshot_loop() {
  load_context_or_fail
  local interval_sec="600"
  local save_octomap="false"
  local prefix="snapshot_auto"

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --interval-sec)
        interval_sec="${2:-600}"
        shift 2
        ;;
      --save-octomap)
        save_octomap="${2:-false}"
        shift 2
        ;;
      --prefix)
        prefix="$(sanitize_token "${2:-snapshot_auto}")"
        shift 2
        ;;
      *)
        echo "Unknown option for _autosnapshot_loop: $1" >&2
        exit 1
        ;;
    esac
  done

  if [[ ! "${interval_sec}" =~ ^[0-9]+$ ]] || [[ "${interval_sec}" -le 0 ]]; then
    echo "autosnapshot interval_sec must be a positive integer, got: ${interval_sec}" >&2
    exit 1
  fi

  trap 'echo "[autosnapshot] stop signal received"; exit 0' SIGINT SIGTERM

  echo "[autosnapshot] loop started run_id=${EXPERIMENT_RUN_ID} interval_sec=${interval_sec} save_octomap=${save_octomap} prefix=${prefix}"
  while true; do
    local tag
    tag="${prefix}_$(timestamp)"
    echo "[autosnapshot] capturing tag=${tag}"
    run_snapshot_capture "${tag}" "${save_octomap}" || echo "[autosnapshot] warning: snapshot failed for tag=${tag}" >&2
    echo "[autosnapshot] sleeping ${interval_sec}s"
    sleep "${interval_sec}"
  done
}

cmd_autosnapshot() {
  load_context_or_fail

  local action="status"
  if [[ $# -gt 0 ]]; then
    action="$1"
    shift
  fi

  local pid_file log_file pid
  pid_file="$(autosnapshot_pid_file)"
  log_file="$(autosnapshot_log_file)"

  case "${action}" in
    start)
      local interval_sec="600"
      local save_octomap="false"
      local prefix="snapshot_auto"
      while [[ $# -gt 0 ]]; do
        case "$1" in
          --interval-sec)
            interval_sec="${2:-600}"
            shift 2
            ;;
          --save-octomap)
            save_octomap="${2:-false}"
            shift 2
            ;;
          --prefix)
            prefix="$(sanitize_token "${2:-snapshot_auto}")"
            shift 2
            ;;
          *)
            echo "Unknown option for autosnapshot start: $1" >&2
            exit 1
            ;;
        esac
      done
      if [[ ! "${interval_sec}" =~ ^[0-9]+$ ]] || [[ "${interval_sec}" -le 0 ]]; then
        echo "autosnapshot interval_sec must be a positive integer, got: ${interval_sec}" >&2
        exit 1
      fi

      if [[ -f "${pid_file}" ]]; then
        pid="$(cat "${pid_file}" 2>/dev/null || true)"
        if is_pid_running "${pid}"; then
          echo "[autosnapshot] already running (pid=${pid})"
          echo "[autosnapshot] log=${log_file}"
          return 0
        fi
        rm -f "${pid_file}"
      fi

      mkdir -p "${EXPERIMENT_RUN_DIR}/logs"
      local ctx="${EXPERIMENT_RUN_DIR}/run_context.env"
      nohup env EXPERIMENT_CONTEXT_FILE="${ctx}" \
        "${SCRIPT_DIR}/experiment_run.sh" _autosnapshot_loop \
        --interval-sec "${interval_sec}" \
        --save-octomap "${save_octomap}" \
        --prefix "${prefix}" >> "${log_file}" 2>&1 &
      pid=$!
      echo "${pid}" > "${pid_file}"
      echo "[autosnapshot] started pid=${pid}"
      echo "[autosnapshot] interval_sec=${interval_sec}"
      echo "[autosnapshot] save_octomap=${save_octomap}"
      echo "[autosnapshot] prefix=${prefix}"
      echo "[autosnapshot] log=${log_file}"
      ;;
    stop)
      if [[ ! -f "${pid_file}" ]]; then
        echo "[autosnapshot] not running"
        return 0
      fi
      pid="$(cat "${pid_file}" 2>/dev/null || true)"
      if is_pid_running "${pid}"; then
        kill "${pid}" 2>/dev/null || true
        sleep 1
        if is_pid_running "${pid}"; then
          kill -9 "${pid}" 2>/dev/null || true
        fi
        echo "[autosnapshot] stopped pid=${pid}"
      else
        echo "[autosnapshot] stale pid file removed"
      fi
      rm -f "${pid_file}"
      ;;
    status)
      if [[ -f "${pid_file}" ]]; then
        pid="$(cat "${pid_file}" 2>/dev/null || true)"
        if is_pid_running "${pid}"; then
          echo "[autosnapshot] running pid=${pid}"
        else
          echo "[autosnapshot] not running (stale pid file)"
        fi
      else
        echo "[autosnapshot] not running"
      fi
      echo "[autosnapshot] pid_file=${pid_file}"
      echo "[autosnapshot] log_file=${log_file}"
      if [[ -f "${log_file}" ]]; then
        echo "[autosnapshot] log tail:"
        tail -n 8 "${log_file}" || true
      fi
      ;;
    *)
      echo "Unknown autosnapshot action: ${action}" >&2
      echo "Use: autosnapshot start|stop|status" >&2
      exit 1
      ;;
  esac
}

cmd_finalize() {
  load_context_or_fail
  local report_name="result"
  local skip_octomap="false"
  local skip_report="false"
  local report_dir=""
  local summary_md=""
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --report-name)
        report_name="${2:-result}"
        shift 2
        ;;
      --skip-octomap)
        skip_octomap="true"
        shift
        ;;
      --skip-report)
        skip_report="true"
        shift
        ;;
      *)
        echo "Unknown option for finalize: $1" >&2
        exit 1
        ;;
    esac
  done

  sync_data_into_run

  if [[ "${skip_octomap}" != "true" ]]; then
    save_octomap_now "final" || true
  fi

  if [[ "${skip_report}" = "true" ]]; then
    echo "[finalize] skip-report=true, finishing without report generation"
    return 0
  fi

  report_dir="${EXPERIMENT_RUN_DIR}/${report_name}"
  summary_md="${report_dir}/summary.md"

  python3 "${SCRIPT_DIR}/export_experiment_report.py" \
    --name "${report_name}" \
    --base-dir "${EXPERIMENT_RUN_DIR}" \
    --data-dir "${EXPERIMENT_DATA_DIR}" \
    --snapshots-dir "${EXPERIMENT_SNAPSHOT_DIR}" \
    --octomaps-dir "${EXPERIMENT_OCTOMAP_DIR}" \
    --overwrite

  if [[ ! -f "${summary_md}" ]]; then
    echo "[finalize] error: summary.md was not generated: ${summary_md}" >&2
    return 1
  fi

  echo "[finalize] report_dir=${report_dir}"
  echo "[finalize] summary_md=${summary_md}"
}

main() {
  if [[ $# -lt 1 ]]; then
    print_usage
    exit 1
  fi
  local cmd="$1"
  shift
  case "${cmd}" in
    init) cmd_init "$@" ;;
    use) cmd_use "$@" ;;
    status) cmd_status "$@" ;;
    snapshot) cmd_snapshot "$@" ;;
    autosnapshot) cmd_autosnapshot "$@" ;;
    _autosnapshot_loop) cmd_autosnapshot_loop "$@" ;;
    finalize) cmd_finalize "$@" ;;
    help|-h|--help) print_usage ;;
    *)
      echo "Unknown command: ${cmd}" >&2
      print_usage
      exit 1
      ;;
  esac
}

main "$@"
