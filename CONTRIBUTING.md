# Contributing

Thank you for improving the Isaac Sim semantic-navigation port. Keep changes
reproducible across workstations and preserve the ROS topic/TF ownership
contract documented in `docs/ISAAC_SIM_PORT.md`.

## Local setup

1. Install Isaac Sim 6.0.1 and ROS 2 Jazzy on Ubuntu 24.04.
2. Copy `dev/isaac_sim/.env.example` to `dev/isaac_sim/.env` and set the local
   Isaac installation and display values. Never commit `.env`.
3. Run `dev/isaac_sim/fetch_models.sh --download`.
4. Run `dev/isaac_sim/bootstrap_runtime.sh --allow-download`.
5. Generate semantic assets with
   `python3 dev/isaac_sim/scene/tools/convert_semantic_assets.py --execute`.
6. Build with `dev/isaac_sim/build_ros.sh --clean-cache`.

The Isaac installation, Python virtualenv, model weights, generated USD/assets,
ROS build/install/log trees, and runtime PID files are local state and must not
be committed.

## Before opening a pull request

Run the portable checks from the repository root:

```bash
dev/isaac_sim/check_source.sh
```

If the change affects the scene, ROS graph, navigation parameters, sensors,
frames, or perception path, also run the visible stack on an RTX workstation:

```bash
dev/isaac_sim/preflight.sh
dev/isaac_sim/run_sim.sh
# In a second terminal:
dev/isaac_sim/run_stack.sh
```

Confirm `/isaac_semantic_nav/contract_status` reports `ok: true` with no
violations, then shut down through `dev/isaac_sim/shutdown.sh`.

## Change discipline

- Create a focused branch and keep commits reviewable.
- Explain user-visible behavior, verification performed, and any known
  limitations in the pull request.
- Update `scene_manifest.json` and the documented contract together.
- Preserve one owner per TF edge and exactly one `/clock` publisher.
- Do not weaken contract thresholds solely to make a run pass.
- Do not add generated files, machine-specific paths, credentials, model
  checkpoints, or large recordings.
- Document the source, license, version, and digest of any new vendored asset.

CI validates portable Python, shell, JSON, and scene logic. It does not claim
that Isaac rendering, ROS middleware, or GPU behavior passed; those require the
live workflow above.
