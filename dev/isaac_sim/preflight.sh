#!/usr/bin/env bash
# Non-destructive host and workspace checks for Isaac Sim semantic navigation.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"
isaac_load_env

headless=false
while (($#)); do
  case "$1" in
    --headless) headless=true ;;
    -h | --help)
      cat <<'EOF'
Usage: ./preflight.sh [--headless]

Checks Isaac Sim 6.0.1, ROS 2 Jazzy/Fast DDS, the local ROS workspace,
NVIDIA GPU access, visible-display GL support, and single-instance state.
No simulator or GUI is launched.
EOF
      exit 0
      ;;
    *) isaac_die "Unknown argument: $1" ;;
  esac
  shift
done

failure_count=0
warning_count=0

check_pass() {
  isaac_pass "$*"
}

check_warn() {
  ((warning_count += 1))
  isaac_warn "$*"
}

check_fail() {
  ((failure_count += 1))
  isaac_fail "$*"
}

printf 'Isaac semantic-navigation preflight\n'
printf '  port root:       %s\n' "${ISAAC_PORT_ROOT}"
printf '  expected Isaac:  %s\n' "${ISAAC_EXPECTED_VERSION}"
printf '  render mode:     %s\n' "$([[ "${headless}" == true ]] && printf headless || printf visible)"
printf '  ROS domain:      %s\n' "${ROS_DOMAIN_ID:-0}"
printf '\n'

isaac_root=""
isaac_python=""
isaac_app=""
detected_version=""
if isaac_root="$(isaac_resolve_root)"; then
  check_pass "Isaac Sim root: ${isaac_root}"
  if isaac_python="$(isaac_resolve_python "${isaac_root}")"; then
    check_pass "Isaac Python launcher: ${isaac_python}"
  else
    check_fail "Isaac Python launcher python.sh was not found under ${isaac_root}"
  fi
  if isaac_app="$(isaac_resolve_app "${isaac_root}")"; then
    check_pass "Isaac application launcher: ${isaac_app}"
  else
    check_warn "isaac-sim.sh was not found under ${isaac_root}; standalone Python can still be used"
  fi
  if detected_version="$(isaac_detect_version "${isaac_root}" "${isaac_python}")"; then
    if [[ "${detected_version}" == "${ISAAC_EXPECTED_VERSION}" ]]; then
      check_pass "Isaac Sim version is exactly ${detected_version}"
    else
      check_fail "Isaac Sim ${detected_version} detected; this port targets ${ISAAC_EXPECTED_VERSION}"
    fi
  else
    check_fail "Could not determine the Isaac Sim version (set ISAAC_SIM_VERSION=${ISAAC_EXPECTED_VERSION} only after verifying it)"
  fi
else
  check_fail "Isaac Sim was not found; set ISAAC_SIM_ROOT or ISAAC_SIM_PATH to a ${ISAAC_EXPECTED_VERSION} installation"
fi

ros_setup="/opt/ros/jazzy/setup.bash"
ros_ready=false
if [[ -r "${ros_setup}" ]]; then
  nounset_was_enabled=0
  [[ $- == *u* ]] && nounset_was_enabled=1
  set +u
  # shellcheck disable=SC1091
  source "${ros_setup}"
  (( nounset_was_enabled )) && set -u
  if [[ "${ROS_DISTRO:-}" == "jazzy" ]]; then
    check_pass "ROS 2 Jazzy sourced from ${ros_setup}"
    ros_ready=true
  else
    check_fail "Expected ROS_DISTRO=jazzy after sourcing ${ros_setup}; got ${ROS_DISTRO:-unset}"
  fi
else
  check_fail "ROS 2 Jazzy setup is missing: ${ros_setup}"
fi

ros_workspace="${ISAAC_ROS_WS:-${ISAAC_DEFAULT_ROS_WS}}"
workspace_setup="${ros_workspace}/install/setup.bash"
if [[ -d "${ros_workspace}/src" ]]; then
  check_pass "ROS workspace source tree: ${ros_workspace}/src"
else
  check_fail "ROS workspace source tree is missing: ${ros_workspace}/src"
fi

if [[ -r "${workspace_setup}" ]]; then
  nounset_was_enabled=0
  [[ $- == *u* ]] && nounset_was_enabled=1
  set +u
  # shellcheck disable=SC1090
  source "${workspace_setup}"
  (( nounset_was_enabled )) && set -u
  check_pass "Workspace overlay built: ${workspace_setup}"
  newer_source="$(find "${ros_workspace}/src" -type f -newer "${workspace_setup}" \
    \( -name CMakeLists.txt -o -name package.xml -o -name setup.py -o -name setup.cfg \
       -o -name '*.cpp' -o -name '*.hpp' -o -name '*.py' -o -name '*.yaml' \
       -o -name '*.msg' -o -name '*.srv' -o -name '*.action' \) \
    -print -quit 2>/dev/null || true)"
  if [[ -n "${newer_source}" ]]; then
    check_warn "Workspace source is newer than install/setup.bash; rebuild before launch (${newer_source})"
  fi
else
  check_fail "Workspace overlay is not built: ${workspace_setup}"
fi

if [[ "${ros_ready}" == true ]]; then
  if ros2 pkg prefix rmw_fastrtps_cpp >/dev/null 2>&1; then
    check_pass "Fast DDS RMW package is installed: rmw_fastrtps_cpp"
  else
    check_fail "Fast DDS RMW package is missing: rmw_fastrtps_cpp"
  fi
  if [[ "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" == "rmw_fastrtps_cpp" ]]; then
    check_pass "RMW selection: rmw_fastrtps_cpp"
  else
    check_warn "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}; launchers will override it with rmw_fastrtps_cpp"
  fi
  if [[ -r "${workspace_setup}" ]] && ros2 pkg prefix isaac_semantic_nav >/dev/null 2>&1; then
    package_prefix="$(ros2 pkg prefix isaac_semantic_nav)"
    check_pass "isaac_semantic_nav is discoverable: ${package_prefix}"
  elif [[ -r "${workspace_setup}" ]]; then
    check_fail "Workspace is sourced, but package isaac_semantic_nav is not discoverable"
  fi
fi

semantic_site_packages=""
if isaac_add_semantic_pythonpath; then
  semantic_site_packages="${ISAAC_SEMANTIC_SITE_PACKAGES}"
  check_pass "Semantic runtime environment: ${ISAAC_SEMANTIC_SITE_PACKAGES}"
fi
semantic_runtime_report="$(isaac_probe_semantic_runtime || true)"
if [[ -n "${semantic_runtime_report}" && "${semantic_runtime_report}" == *"cuda=True"* && "${semantic_runtime_report}" == *"pins=True"* && "${semantic_runtime_report}" == *"ros_imports=True"* && "${semantic_runtime_report}" == *"worker_import=True"* ]]; then
  check_pass "GPU detector runtime: ${semantic_runtime_report}"
else
  check_fail "Pinned GPU detector/refiner dependencies are unavailable to /usr/bin/python3; run ./bootstrap_runtime.sh --check"
fi

for worker_tool in ffmpeg ffprobe; do
  if command -v "${worker_tool}" >/dev/null 2>&1; then
    check_pass "Semantic refiner tool: $(command -v "${worker_tool}")"
  else
    check_fail "Semantic refiner requires ${worker_tool}, but it is not installed"
  fi
done

if isaac_verify_port_assets; then
  check_pass "Pinned detector, refiner, CLIP model, and local CLIP wheel assets"
else
  check_fail "A required model or vendored runtime wheel is missing or has the wrong SHA-256 digest (run ./fetch_models.sh --check)"
fi

if [[ "${ROS_DOMAIN_ID:-0}" =~ ^[0-9]+$ ]] && (( ${ROS_DOMAIN_ID:-0} <= 232 )); then
  check_pass "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
else
  check_fail "ROS_DOMAIN_ID must be an integer from 0 through 232; got ${ROS_DOMAIN_ID:-unset}"
fi

if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  if [[ -r "${FASTRTPS_DEFAULT_PROFILES_FILE}" ]]; then
    check_pass "Fast DDS profiles: ${FASTRTPS_DEFAULT_PROFILES_FILE}"
  else
    check_fail "FASTRTPS_DEFAULT_PROFILES_FILE is unreadable: ${FASTRTPS_DEFAULT_PROFILES_FILE}"
  fi
fi
[[ -n "${CYCLONEDDS_URI:-}" ]] && check_warn "CYCLONEDDS_URI is set but ignored because this port uses Fast DDS"

if command -v nvidia-smi >/dev/null 2>&1; then
  gpu_report="$(nvidia-smi --query-gpu=index,name,driver_version,memory.total --format=csv,noheader 2>/dev/null || true)"
  if [[ -n "${gpu_report}" ]]; then
    while IFS= read -r gpu_line; do
      check_pass "NVIDIA GPU: ${gpu_line}"
      [[ "${gpu_line}" == *RTX* ]] || check_warn "GPU is not identified as an RTX model: ${gpu_line}"
    done <<< "${gpu_report}"
  else
    check_fail "nvidia-smi exists but cannot communicate with the NVIDIA driver"
  fi
else
  check_fail "nvidia-smi is not installed"
fi

if [[ "${headless}" == true ]]; then
  check_pass "Headless mode selected; DISPLAY/GLX checks skipped"
else
  if [[ -n "${DISPLAY:-}" ]]; then
    check_pass "DISPLAY=${DISPLAY}"
  else
    check_fail "DISPLAY is unset for a visible launch (this host normally renders on DISPLAY=:0)"
  fi

  if command -v glxinfo >/dev/null 2>&1 && [[ -n "${DISPLAY:-}" ]]; then
    gl_report="$(glxinfo -B 2>/dev/null || true)"
    if [[ "${gl_report}" == *"direct rendering: Yes"* && "${gl_report}" == *"OpenGL vendor string: NVIDIA Corporation"* ]]; then
      gl_renderer="$(sed -n 's/^OpenGL renderer string: /renderer=/p' <<< "${gl_report}" | head -n 1)"
      gl_version="$(sed -n 's/^OpenGL core profile version string: /GL=/p' <<< "${gl_report}" | head -n 1)"
      check_pass "Direct NVIDIA GLX works (${gl_renderer:-renderer=unknown}; ${gl_version:-GL=unknown})"
    else
      check_fail "DISPLAY=${DISPLAY} cannot create a direct NVIDIA GLX context"
    fi
  elif ! command -v glxinfo >/dev/null 2>&1; then
    check_warn "glxinfo is unavailable; visible rendering cannot be preflighted"
  fi
fi

if command -v vulkaninfo >/dev/null 2>&1; then
  if vulkaninfo --summary >/dev/null 2>&1; then
    check_pass "Vulkan loader/device summary succeeds"
  else
    check_fail "vulkaninfo --summary failed"
  fi
elif [[ -r /usr/share/vulkan/icd.d/nvidia_icd.json ]]; then
  check_warn "NVIDIA Vulkan ICD is present, but vulkaninfo is not installed"
else
  check_fail "NVIDIA Vulkan ICD is missing"
fi

scene_script="${ISAAC_SCENE_SCRIPT:-${ISAAC_DEFAULT_SCENE_SCRIPT}}"
if [[ -r "${scene_script}" ]]; then
  check_pass "Scene entrypoint: ${scene_script}"
else
  check_fail "Scene entrypoint is missing: ${scene_script}"
fi

for process_kind in sim stack; do
  process_record="$(isaac_pid_file "${process_kind}")"
  if [[ -e "${process_record}" ]]; then
    if isaac_record_is_live "${process_record}"; then
      check_pass "Managed ${process_kind} process is live (PID $(isaac_record_field "${process_record}" pid))"
    else
      record_status=$?
      if [[ "${record_status}" -eq 2 ]]; then
        check_fail "Untrusted or PID-reused metadata requires manual inspection: ${process_record}"
      else
        check_warn "Stale ${process_kind} metadata exists: ${process_record}"
      fi
    fi
  else
    check_pass "No managed ${process_kind} process is currently recorded"
  fi
done

scene_count="$(isaac_count_scene_runners "${scene_script}")"
if [[ "${scene_count}" -eq 0 ]]; then
  check_pass "Single-instance invariant: zero port scene runners before launch"
elif [[ "${scene_count}" -eq 1 ]]; then
  check_pass "Single-instance invariant: exactly one port scene runner"
else
  check_fail "Single-instance violation: ${scene_count} port scene runners are active"
fi

mapfile -t conflicting_sim_pids < <(isaac_conflicting_gazebo_pids)
if (( ${#conflicting_sim_pids[@]} == 0 )); then
  check_pass "No Gazebo process is competing for simulation time"
else
  check_fail "Found ${#conflicting_sim_pids[@]} conflicting Gazebo process(es), PID(s): ${conflicting_sim_pids[*]}"
fi

mapfile -t external_isaac_pids < <(isaac_external_isaac_pids "${scene_script}")
if (( ${#external_isaac_pids[@]} == 0 )); then
  check_pass "No unmanaged Isaac/Kit simulator process is active"
else
  check_fail "Found unmanaged Isaac/Kit process(es), PID(s): ${external_isaac_pids[*]}"
fi

printf '\nPreflight result: %d failure(s), %d warning(s).\n' "${failure_count}" "${warning_count}"
if (( failure_count > 0 )); then
  exit 1
fi
