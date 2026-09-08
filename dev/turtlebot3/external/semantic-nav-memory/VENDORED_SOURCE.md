# Vendored Source

This directory contains a runtime-focused tracked-source snapshot of:

```text
https://github.com/ilumn/semantic-nav-memory.git
commit c5ad423425a0ba2e8a2007d8368eaa4825949435
```

It is vendored here so a robot can clone the main semantic navigation repository
and still have the optional `tb3_semantic_refiner` worker source available.
Large demo media from the upstream repo is intentionally omitted because it is
not needed for robot operation.

Generated runtime artifacts are intentionally excluded:

- `.git/`
- `.venv/`
- `.venv-cpu/`
- `.local-bin/`
- `.pytest_cache/`
- `.ultralytics/`
- `semantic_nav_memory.egg-info/`
- Python `__pycache__/`
- demo/smoke media

The worker remains optional. Real robot bringup should start with
`refiner_worker_enabled:=false` and enable this dependency only after live
navigation and camera grounding are validated.
