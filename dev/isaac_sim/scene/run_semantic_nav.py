#!/usr/bin/env python3
"""Standalone Isaac Sim 6.0.1 launcher for the semantic-navigation scene."""

from __future__ import annotations

import sys
from pathlib import Path


SCENE_ROOT = Path(__file__).resolve().parent
if str(SCENE_ROOT) not in sys.path:
    sys.path.insert(0, str(SCENE_ROOT))


def _pure_preflight(argv: list[str] | None):
    # This package is deliberately stdlib-only; importing it is safe before
    # SimulationApp.  Every Isaac/Kit import stays below SimulationApp creation.
    from semantic_nav_isaac_scene.manifest import load_manifest
    from semantic_nav_isaac_scene.conversion import require_enabled_intermediates
    from semantic_nav_isaac_scene.runtime_args import (
        build_parser,
        resolve_urdf_path,
        stage_mode,
        validate_stage_path,
    )

    parser = build_parser()
    args = parser.parse_args(argv)
    if args.rebuild_stage and args.stage_path is None:
        parser.error("--rebuild-stage requires --stage-path")
    manifest_path = (args.manifest or SCENE_ROOT / "config" / "scene_manifest.json").resolve()
    manifest = load_manifest(manifest_path)
    try:
        if args.stage_path is not None:
            args.stage_path = validate_stage_path(args.stage_path)
        mode = stage_mode(args.stage_path, rebuild=args.rebuild_stage)
        require_enabled_intermediates(manifest)
        urdf_path = None if mode == "open" else resolve_urdf_path(manifest, args.urdf_path)
    except (FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))
    return args, manifest, mode, urdf_path


def simulation_app_launch_config(*, headless: bool) -> dict[str, object]:
    """Return the pure-data Kit launch contract, testable without Isaac Sim."""

    return {
        "headless": bool(headless),
        # RTX LiDAR is not safe with multi-GPU rendering on the target release;
        # force one GPU even on dual-GPU workstations to avoid CUDA error 700.
        "multi_gpu": False,
        "renderer": "RaytracedLighting",
        "width": 1280,
        "height": 720,
    }


def main(argv: list[str] | None = None) -> int:
    args, manifest, mode, urdf_path = _pure_preflight(argv)

    # NVIDIA requires SimulationApp to exist before importing Kit, USD, graph,
    # sensor, importer, or bridge modules.
    from isaacsim import SimulationApp

    launch_config = simulation_app_launch_config(headless=args.headless)
    simulation_app = SimulationApp(launch_config)
    timeline = None
    try:
        import omni.timeline

        from semantic_nav_isaac_scene.isaac_builder import (
            SceneBuildError,
            assert_supported_runtime,
            compose_scene,
            enable_required_extensions,
            open_managed_stage,
            save_stage,
        )

        enable_required_extensions(simulation_app)
        runtime_version = assert_supported_runtime(manifest)
        print(f"semantic-nav: Isaac Sim {runtime_version}, PhysX, ROS 2 Jazzy")

        if mode == "open":
            stage = open_managed_stage(simulation_app, args.stage_path, manifest)
            print(f"semantic-nav: opened managed stage {args.stage_path}")
        else:
            assert urdf_path is not None
            result = compose_scene(
                simulation_app,
                manifest,
                urdf_path=urdf_path,
                cache_dir=args.asset_cache,
                xacro_executable=args.xacro_executable,
            )
            stage = result.stage
            print(f"semantic-nav: composed stage from {urdf_path}")
            for target, asset_mode in sorted(result.semantic_asset_modes.items()):
                print(f"semantic-nav: semantic target {target}: {asset_mode}")
            for warning in result.warnings:
                print(f"semantic-nav: warning: {warning}", file=sys.stderr)
            if args.stage_path is not None:
                saved = save_stage(stage, args.stage_path)
                print(f"semantic-nav: saved managed stage {saved}")

        if args.compose_only:
            if args.stage_path is None:
                print("semantic-nav: compose-only stage was intentionally kept in memory")
            return 0

        topics = manifest["ros2"]["topics"]
        print("semantic-nav: required ROS interfaces: " + ", ".join(topics.values()))
        print("semantic-nav: TF owner: Isaac publishes odom -> base_footprint only")
        print("semantic-nav: starting timeline (Ctrl-C or close the window to stop)")
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        steps = 0
        while simulation_app.is_running():
            simulation_app.update()
            steps += 1
            if args.max_steps is not None and steps >= args.max_steps:
                break
        return 0
    except KeyboardInterrupt:
        print("\nsemantic-nav: interrupted")
        return 130
    except Exception as exc:
        # Keep the entry point useful even when an exception happens before the
        # SceneBuildError class is imported.
        print(f"semantic-nav: fatal: {exc}", file=sys.stderr)
        return 1
    finally:
        if timeline is not None:
            timeline.stop()
        simulation_app.close()


if __name__ == "__main__":
    raise SystemExit(main())
