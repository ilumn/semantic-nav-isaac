#!/usr/bin/env bash
# Launch the ROS 2 semantic-navigation stack against the managed Isaac scene.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
isaac_load_env

stack_package="${ISAAC_STACK_PACKAGE:-isaac_semantic_nav}"
stack_launch_file="${ISAAC_STACK_LAUNCH_FILE:-isaac_semantic_nav.launch.py}"
declare -a launch_args=()

usage() {
  cat <<EOF
Usage: ./run_stack.sh [ROS launch arguments...]

Sources ROS 2 Jazzy and the port workspace, forces Fast DDS, and runs:

  ros2 launch ${stack_package} ${stack_launch_file} use_sim_time:=true

Additional arguments such as detector_device:=cuda:0 are passed to ros2 launch.
Set ISAAC_STACK_PACKAGE or ISAAC_STACK_LAUNCH_FILE to override the defaults.
EOF
}

while (($#)); do
  case "$1" in
    -h | --help)
      usage
      exit 0
      ;;
    --)
      shift
      launch_args+=("$@")
      break
      ;;
    *)
      launch_args+=("$1")
      ;;
  esac
  shift
done

isaac_source_ros true
ros2 pkg prefix rmw_fastrtps_cpp >/dev/null 2>&1 || isaac_die \
  "ROS 2 Fast DDS support is missing: package rmw_fastrtps_cpp"
semantic_site_packages=""
if isaac_add_semantic_pythonpath; then
  semantic_site_packages="${ISAAC_SEMANTIC_SITE_PACKAGES}"
  isaac_pass "Semantic runtime environment: ${ISAAC_SEMANTIC_SITE_PACKAGES}"
fi
semantic_runtime_report="$(isaac_probe_semantic_runtime || true)"
[[ -n "${semantic_runtime_report}" && "${semantic_runtime_report}" == *"cuda=True"* && "${semantic_runtime_report}" == *"pins=True"* && "${semantic_runtime_report}" == *"ros_imports=True"* && "${semantic_runtime_report}" == *"worker_import=True"* ]] || isaac_die \
  "Pinned CUDA detector/refiner dependencies are unavailable. Run ./bootstrap_runtime.sh --check."
isaac_pass "GPU detector runtime: ${semantic_runtime_report}"
isaac_verify_port_assets || isaac_die \
  "A required model or vendored runtime wheel is missing or has the wrong SHA-256 digest. Run ./fetch_models.sh --check and ./preflight.sh for details."

package_prefix="$(ros2 pkg prefix "${stack_package}" 2>/dev/null || true)"
[[ -n "${package_prefix}" ]] || isaac_die \
  "ROS package ${stack_package} is not discoverable. Rebuild ${ISAAC_ROS_WS:-${ISAAC_DEFAULT_ROS_WS}}."
launch_path="${package_prefix}/share/${stack_package}/launch/${stack_launch_file}"
[[ -r "${launch_path}" ]] || isaac_die "Launch file is missing: ${launch_path}"

export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

isaac_prepare_runtime_dir
if ! command -v flock >/dev/null 2>&1; then
  isaac_die "flock is required for single-instance launch protection."
fi
exec 9>"${ISAAC_RUNTIME_DIR}/stack.lock"
flock -n 9 || isaac_die "Another run_stack.sh invocation owns ${ISAAC_RUNTIME_DIR}/stack.lock"
isaac_assert_no_live_record stack

scene_script="${ISAAC_SCENE_SCRIPT:-${ISAAC_DEFAULT_SCENE_SCRIPT}}"
scene_count="$(isaac_count_scene_runners "${scene_script}")"
if (( scene_count > 1 )); then
  isaac_die "Single-instance violation: ${scene_count} Isaac scene runners are active."
elif (( scene_count == 0 )); then
  isaac_warn "No managed Isaac scene runner was detected; the stack will wait for simulation topics."
fi

has_use_sim_time=false
for launch_arg in "${launch_args[@]}"; do
  [[ "${launch_arg}" == use_sim_time:=* ]] && has_use_sim_time=true
done
[[ "${has_use_sim_time}" == true ]] || launch_args=(use_sim_time:=true "${launch_args[@]}")

launch_command=(
  ros2 launch "${stack_package}" "${stack_launch_file}"
  "${launch_args[@]}"
)

isaac_info "Starting ROS stack: ${stack_package}/${stack_launch_file}"
isaac_info "ROS: Jazzy, ${RMW_IMPLEMENTATION}, domain ${ROS_DOMAIN_ID}"

if command -v setsid >/dev/null 2>&1; then
  setsid "${launch_command[@]}" &
else
  isaac_warn "setsid is unavailable; shutdown will signal only the recorded top-level PID"
  "${launch_command[@]}" &
fi
stack_pid=$!

if ! isaac_wait_for_marker "${stack_pid}" "${stack_package}"; then
  wait "${stack_pid}" 2>/dev/null || true
  isaac_die "ROS stack exited before its PID could be registered."
fi
isaac_write_pid_record stack "${stack_pid}" "${stack_package}" || {
  isaac_term_owned_child "${stack_pid}"
  wait "${stack_pid}" 2>/dev/null || true
  isaac_die "Could not create scoped stack PID metadata."
}
isaac_pass "Managed ROS stack started as PID ${stack_pid}"

forward_stack_signal() {
  isaac_term_owned_child "${stack_pid}"
}

cleanup_stack_record() {
  isaac_remove_own_record stack "${stack_pid}"
}

trap forward_stack_signal INT TERM HUP
trap cleanup_stack_record EXIT

set +e
wait "${stack_pid}"
stack_status=$?
set -e
exit "${stack_status}"
