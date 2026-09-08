#!/usr/bin/env bash
# Launch exactly one managed Isaac Sim scene runner in the foreground.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
isaac_load_env

headless=false
isaac_is_true "${ISAAC_HEADLESS:-false}" && headless=true
scene_script="${ISAAC_SCENE_SCRIPT:-${ISAAC_DEFAULT_SCENE_SCRIPT}}"
stage_path="${ISAAC_STAGE_PATH:-}"
declare -a scene_args=()

usage() {
  cat <<EOF
Usage: ./run_sim.sh [options] [-- scene-arguments...]

Options:
  --headless           Run without the Isaac UI.
  --visible            Force a visible UI (the default).
  --scene PATH         Override ${ISAAC_DEFAULT_SCENE_SCRIPT}.
  --stage PATH         Pass an optional USD stage path to the scene runner.
  -h, --help           Show this help.

The script finds Isaac Sim through ISAAC_SIM_PATH, ISAAC_SIM_ROOT, or standard
native-install paths, then runs the standalone scene with Isaac's python.sh.
EOF
}

while (($#)); do
  case "$1" in
    --headless)
      headless=true
      ;;
    --visible)
      headless=false
      ;;
    --scene)
      (($# >= 2)) || isaac_die "--scene requires a path"
      scene_script="$2"
      shift
      ;;
    --stage)
      (($# >= 2)) || isaac_die "--stage requires a path"
      stage_path="$2"
      shift
      ;;
    --)
      shift
      scene_args+=("$@")
      break
      ;;
    -h | --help)
      usage
      exit 0
      ;;
    *)
      isaac_die "Unknown option: $1 (put scene-specific arguments after --)"
      ;;
  esac
  shift
done

[[ -r "${scene_script}" ]] || isaac_die "Scene entrypoint is missing: ${scene_script}"
scene_script="$(cd -- "$(dirname -- "${scene_script}")" && pwd -P)/$(basename -- "${scene_script}")"

isaac_root="$(isaac_resolve_root)" || isaac_die \
  "Isaac Sim ${ISAAC_EXPECTED_VERSION} was not found. Set ISAAC_SIM_ROOT or ISAAC_SIM_PATH."
isaac_python="$(isaac_resolve_python "${isaac_root}")" || isaac_die \
  "Isaac's executable python.sh was not found under ${isaac_root}."

detected_version="$(isaac_detect_version "${isaac_root}" "${isaac_python}" || true)"
if [[ -z "${detected_version}" ]]; then
  if ! isaac_is_true "${ISAAC_ALLOW_UNVERIFIED_VERSION:-false}"; then
    isaac_die "Could not verify Isaac Sim ${ISAAC_EXPECTED_VERSION}; set ISAAC_SIM_VERSION after checking the installation."
  fi
  isaac_warn "Isaac Sim version is unverified because ISAAC_ALLOW_UNVERIFIED_VERSION is enabled"
elif [[ "${detected_version}" != "${ISAAC_EXPECTED_VERSION}" ]]; then
  if ! isaac_is_true "${ISAAC_ALLOW_VERSION_MISMATCH:-false}"; then
    isaac_die "Isaac Sim ${detected_version} found; this port requires ${ISAAC_EXPECTED_VERSION}."
  fi
  isaac_warn "Running untested Isaac Sim ${detected_version}; expected ${ISAAC_EXPECTED_VERSION}"
else
  isaac_pass "Isaac Sim ${detected_version}: ${isaac_root}"
fi

isaac_source_ros true
ros2 pkg prefix rmw_fastrtps_cpp >/dev/null 2>&1 || isaac_die \
  "ROS 2 Fast DDS support is missing: package rmw_fastrtps_cpp"

if ! command -v nvidia-smi >/dev/null 2>&1 || ! nvidia-smi >/dev/null 2>&1; then
  isaac_die "The NVIDIA driver/GPU is unavailable; nvidia-smi failed."
fi

if [[ "${headless}" == false ]]; then
  [[ -n "${DISPLAY:-}" ]] || isaac_die \
    "DISPLAY is unset. Set DISPLAY=:0 for this host or use --headless."
  if command -v glxinfo >/dev/null 2>&1; then
    gl_report="$(glxinfo -B 2>/dev/null || true)"
    [[ "${gl_report}" == *"direct rendering: Yes"* ]] || isaac_die \
      "DISPLAY=${DISPLAY} does not provide direct rendering."
    [[ "${gl_report}" == *"OpenGL vendor string: NVIDIA Corporation"* ]] || isaac_die \
      "DISPLAY=${DISPLAY} is not using the NVIDIA OpenGL driver."
  else
    isaac_warn "glxinfo is unavailable; continuing without a visible-rendering probe"
  fi
fi

isaac_prepare_runtime_dir
if ! command -v flock >/dev/null 2>&1; then
  isaac_die "flock is required for single-instance launch protection."
fi
exec 9>"${ISAAC_RUNTIME_DIR}/sim.lock"
flock -n 9 || isaac_die "Another run_sim.sh invocation owns ${ISAAC_RUNTIME_DIR}/sim.lock"

isaac_assert_no_live_record sim
scene_count="$(isaac_count_scene_runners "${scene_script}")"
(( scene_count == 0 )) || isaac_die \
  "Refusing to start a duplicate scene: ${scene_count} runner(s) already match ${scene_script}"
mapfile -t conflicting_gazebo_pids < <(isaac_conflicting_gazebo_pids)
(( ${#conflicting_gazebo_pids[@]} == 0 )) || isaac_die \
  "Refusing to compete with Gazebo process PID(s): ${conflicting_gazebo_pids[*]}"
mapfile -t external_isaac_pids < <(isaac_external_isaac_pids "${scene_script}")
(( ${#external_isaac_pids[@]} == 0 )) || isaac_die \
  "Refusing to start beside unmanaged Isaac/Kit process PID(s): ${external_isaac_pids[*]}"

declare -a launch_command=("${isaac_python}" "${scene_script}")
if [[ "${headless}" == true ]]; then
  launch_command+=(--headless)
fi
if [[ -n "${stage_path}" ]]; then
  launch_command+=(--stage-path "${stage_path}")
fi
launch_command+=("${scene_args[@]}")

export ISAAC_SIM_ROOT="${isaac_root}"
export ISAAC_SIM_PATH="${isaac_root}"
export ISAAC_SIM_VERSION="${detected_version:-${ISAAC_EXPECTED_VERSION}}"
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

isaac_info "Starting the managed scene in $([[ "${headless}" == true ]] && printf headless || printf visible) mode"
isaac_info "Scene: ${scene_script}"
[[ -n "${stage_path}" ]] && isaac_info "Stage override: ${stage_path}"
isaac_info "ROS: Jazzy, ${RMW_IMPLEMENTATION}, domain ${ROS_DOMAIN_ID}"

if command -v setsid >/dev/null 2>&1; then
  setsid "${launch_command[@]}" &
else
  isaac_warn "setsid is unavailable; shutdown will signal only the recorded top-level PID"
  "${launch_command[@]}" &
fi
sim_pid=$!

if ! isaac_wait_for_marker "${sim_pid}" "${scene_script}"; then
  wait "${sim_pid}" 2>/dev/null || true
  isaac_die "Isaac scene runner exited before its PID could be registered."
fi
isaac_write_pid_record sim "${sim_pid}" "${scene_script}" || {
  isaac_term_owned_child "${sim_pid}"
  wait "${sim_pid}" 2>/dev/null || true
  isaac_die "Could not create scoped simulator PID metadata."
}
isaac_pass "Managed simulator started as PID ${sim_pid}"

forward_sim_signal() {
  isaac_term_owned_child "${sim_pid}"
}

cleanup_sim_record() {
  isaac_remove_own_record sim "${sim_pid}"
}

trap forward_sim_signal INT TERM HUP
trap cleanup_sim_record EXIT

set +e
wait "${sim_pid}"
sim_status=$?
set -e
exit "${sim_status}"
