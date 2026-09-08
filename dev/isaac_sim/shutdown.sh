#!/usr/bin/env bash
# Gracefully stop only processes registered by this Isaac port's launchers.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
isaac_load_env

stop_sim=true
stop_stack=true
dry_run=false
shutdown_timeout="${ISAAC_SHUTDOWN_TIMEOUT:-20}"

usage() {
  cat <<'EOF'
Usage: ./shutdown.sh [options]

Options:
  --sim-only          Stop only the managed Isaac scene runner.
  --stack-only        Stop only the managed ROS stack.
  --timeout SECONDS   Wait this long after SIGTERM (default: 20).
  --dry-run           Validate and print targets without signaling them.
  -h, --help          Show this help.

This script never uses pkill. It trusts only PID records created by run_sim.sh
and run_stack.sh, verifies process start time and command marker against /proc,
then sends SIGTERM to that dedicated process group or exact PID.
EOF
}

while (($#)); do
  case "$1" in
    --sim-only)
      stop_sim=true
      stop_stack=false
      ;;
    --stack-only)
      stop_sim=false
      stop_stack=true
      ;;
    --timeout)
      (($# >= 2)) || isaac_die "--timeout requires an integer number of seconds"
      shutdown_timeout="$2"
      shift
      ;;
    --dry-run)
      dry_run=true
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    *)
      isaac_die "Unknown option: $1"
      ;;
  esac
  shift
done

[[ "${shutdown_timeout}" =~ ^[0-9]+$ ]] || isaac_die \
  "--timeout must be a non-negative integer; got ${shutdown_timeout}"

isaac_prepare_runtime_dir
shutdown_failures=0

stop_recorded_process() {
  local kind="${1:?process kind is required}"
  local record pid pgid actual_pgid signal_target target_description
  local deadline now record_status

  record="$(isaac_pid_file "${kind}")"
  if [[ ! -e "${record}" ]]; then
    isaac_info "No managed ${kind} PID record exists; nothing to stop."
    return 0
  fi

  record_status=0
  isaac_record_is_live "${record}" || record_status=$?
  if [[ "${record_status}" -ne 0 ]]; then
    if [[ "${record_status}" -eq 2 ]]; then
      isaac_fail "Refusing to signal an untrusted or PID-reused record: ${record}"
      ((shutdown_failures += 1))
      return 1
    fi
    isaac_warn "Removing stale ${kind} PID metadata: ${record}"
    rm -f -- "${record}"
    return 0
  fi

  pid="$(isaac_record_field "${record}" pid)"
  pgid="$(isaac_record_field "${record}" pgid || true)"
  actual_pgid="$(ps -o pgid= -p "${pid}" 2>/dev/null | tr -d '[:space:]')"

  signal_target="${pid}"
  target_description="PID ${pid}"
  if [[ "${pgid}" =~ ^[0-9]+$ && "${actual_pgid}" == "${pgid}" && "${pgid}" == "${pid}" ]]; then
    signal_target="-${pgid}"
    target_description="dedicated process group ${pgid}"
  fi

  if [[ "${dry_run}" == true ]]; then
    isaac_info "Dry run: would send SIGTERM to managed ${kind} ${target_description}."
    return 0
  fi

  isaac_info "Sending SIGTERM to managed ${kind} ${target_description}."
  if ! /bin/kill -TERM -- "${signal_target}" 2>/dev/null; then
    isaac_fail "SIGTERM failed for managed ${kind} ${target_description}."
    ((shutdown_failures += 1))
    return 1
  fi

  deadline=$((SECONDS + shutdown_timeout))
  while {
    if [[ "${signal_target}" == -* ]]; then
      /bin/kill -0 -- "${signal_target}" 2>/dev/null
    else
      kill -0 "${pid}" 2>/dev/null
    fi
  }; do
    now=${SECONDS}
    if (( now >= deadline )); then
      isaac_fail "Managed ${kind} PID ${pid} did not exit within ${shutdown_timeout}s; no SIGKILL was sent."
      ((shutdown_failures += 1))
      return 1
    fi
    sleep 0.25
  done

  rm -f -- "${record}"
  isaac_pass "Managed ${kind} stopped gracefully."
}

# Stop ROS consumers before their simulation-time source.
if [[ "${stop_stack}" == true ]]; then
  stop_recorded_process stack || true
fi
if [[ "${stop_sim}" == true ]]; then
  stop_recorded_process sim || true
fi

if (( shutdown_failures > 0 )); then
  exit 1
fi
