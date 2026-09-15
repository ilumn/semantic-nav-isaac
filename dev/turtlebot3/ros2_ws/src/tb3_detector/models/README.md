# tb3_detector model cache

The primary detector uses `nvidia/LocateAnything-3B` from the standard Hugging
Face cache. Model weights are not stored in this directory or committed to Git.

From the repository root, install the pinned runtime and fetch the immutable
model revision:

```bash
dev/isaac_sim/bootstrap_runtime.sh --allow-download
dev/isaac_sim/fetch_models.sh --download
```

`fetch_models.sh --check` verifies that the full pinned snapshot can be resolved
without network access. The configured revision is:

```text
c32291ca5e996f5a7a485845b4f57a233936bba0
```

The model is distributed under the NVIDIA License for research and evaluation
use only. See `THIRD_PARTY_NOTICES.md`.
