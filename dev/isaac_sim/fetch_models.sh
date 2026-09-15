#!/usr/bin/env bash
# Fetch the exact model checkpoints validated by the Isaac Sim port.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=lib/common.sh
source "${SCRIPT_DIR}/lib/common.sh"

mode=check
force=false

usage() {
  cat <<'EOF'
Usage: ./fetch_models.sh [--check | --download] [--force]

  --check      Verify that all required checkpoints exist and match (default).
  --download   Download missing checkpoints from their pinned upstream URLs.
  --force      With --download, replace a checkpoint whose digest is incorrect.

Downloads are written atomically and accepted only when their SHA-256 digest
matches the version validated by this port.
EOF
}

while (($#)); do
  case "$1" in
    --check) mode=check ;;
    --download) mode=download ;;
    --force) force=true ;;
    -h | --help)
      usage
      exit 0
      ;;
    *) isaac_die "Unknown option: $1" ;;
  esac
  shift
done

if [[ "${force}" == true && "${mode}" != download ]]; then
  isaac_die "--force is valid only with --download"
fi

declare -a model_specs=(
  "YOLOv8s-World-v2|${ISAAC_REFINER_MODEL}|${ISAAC_REFINER_MODEL_SHA256}|https://github.com/ultralytics/assets/releases/download/v8.2.0/yolov8s-worldv2.pt"
  "CLIP ViT-B/32|${ISAAC_CLIP_MODEL}|${ISAAC_CLIP_MODEL_SHA256}|https://openaipublic.azureedge.net/clip/models/40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af/ViT-B-32.pt"
)

if [[ "${mode}" == download ]]; then
  command -v curl >/dev/null 2>&1 || isaac_die "curl is required to download checkpoints"
fi

status=0
for spec in "${model_specs[@]}"; do
  IFS='|' read -r label target expected url <<< "${spec}"
  if [[ -r "${target}" ]]; then
    actual="$(sha256sum -- "${target}" | awk '{print $1}')"
    if [[ "${actual}" == "${expected}" ]]; then
      isaac_pass "${label}: ${target}"
      continue
    fi
    if [[ "${mode}" != download || "${force}" != true ]]; then
      isaac_fail "${label}: checksum mismatch at ${target}"
      isaac_info "Use --download --force to replace the invalid file"
      status=1
      continue
    fi
  elif [[ "${mode}" != download ]]; then
    isaac_fail "${label}: missing ${target}"
    status=1
    continue
  fi

  mkdir -p -- "$(dirname -- "${target}")"
  temp_file="$(mktemp --tmpdir="$(dirname -- "${target}")" ".$(basename -- "${target}").part.XXXXXX")"
  isaac_info "Downloading ${label} from ${url}"
  if ! curl --fail --location --retry 3 --retry-all-errors --connect-timeout 30 \
    --output "${temp_file}" "${url}"; then
    rm -f -- "${temp_file}"
    isaac_fail "${label}: download failed"
    status=1
    continue
  fi
  actual="$(sha256sum -- "${temp_file}" | awk '{print $1}')"
  if [[ "${actual}" != "${expected}" ]]; then
    rm -f -- "${temp_file}"
    isaac_fail "${label}: downloaded checksum mismatch"
    status=1
    continue
  fi
  chmod 0644 -- "${temp_file}"
  mv -f -- "${temp_file}" "${target}"
  isaac_pass "${label}: installed ${target}"
done

if ! isaac_add_semantic_pythonpath; then
  isaac_fail "Runtime venv is unavailable; run ${SCRIPT_DIR}/bootstrap_runtime.sh first"
  status=1
elif [[ "${mode}" == download ]]; then
  isaac_info "Downloading LocateAnything-3B revision ${ISAAC_LIVE_MODEL_REVISION}"
  if ! /usr/bin/python3 - "${ISAAC_LIVE_MODEL_ID}" "${ISAAC_LIVE_MODEL_REVISION}" <<'PY'
import sys
from huggingface_hub import snapshot_download

path = snapshot_download(repo_id=sys.argv[1], revision=sys.argv[2])
print(path)
PY
  then
    isaac_fail "LocateAnything-3B download failed"
    status=1
  fi
elif ! /usr/bin/python3 - "${ISAAC_LIVE_MODEL_ID}" "${ISAAC_LIVE_MODEL_REVISION}" <<'PY' >/dev/null 2>&1
import sys
from huggingface_hub import snapshot_download

snapshot_download(repo_id=sys.argv[1], revision=sys.argv[2], local_files_only=True)
PY
then
  isaac_fail "LocateAnything-3B revision ${ISAAC_LIVE_MODEL_REVISION} is missing from the Hugging Face cache"
  status=1
else
  isaac_pass "LocateAnything-3B revision ${ISAAC_LIVE_MODEL_REVISION}"
fi

if ((status != 0)); then
  if [[ "${mode}" == check ]]; then
    isaac_info "Run ${SCRIPT_DIR}/fetch_models.sh --download to install missing checkpoints"
  fi
  exit "${status}"
fi

isaac_pass "All required model checkpoints are present and verified"
