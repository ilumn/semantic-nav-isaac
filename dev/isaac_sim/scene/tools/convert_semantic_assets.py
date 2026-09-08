#!/usr/bin/env python3
"""Create OBJ intermediates from the three source Collada semantic assets."""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


SCENE_ROOT = Path(__file__).resolve().parent.parent
if str(SCENE_ROOT) not in sys.path:
    sys.path.insert(0, str(SCENE_ROOT))

from semantic_nav_isaac_scene.conversion import (  # noqa: E402
    all_conversion_plans,
    bundled_converter_path,
    obj_material_files,
    prepare_obj_material_dependencies,
    unresolved_obj_material_dependencies,
)
from semantic_nav_isaac_scene.manifest import load_manifest  # noqa: E402


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Plan or run DAE -> OBJ conversion before Isaac's OBJ -> USD conversion.",
    )
    parser.add_argument("--manifest", type=Path, default=SCENE_ROOT / "config" / "scene_manifest.json")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--assimp", help="Use an Assimp CLI executable instead of the bundled helper.")
    parser.add_argument("--execute", action="store_true", help="Build the helper if needed and run conversions.")
    return parser


def _build_helper(manifest) -> Path:
    output = bundled_converter_path(manifest)
    build_script = SCENE_ROOT / "tools" / "build_dae_to_obj.sh"
    completed = subprocess.run(
        ["bash", str(build_script), str(output)],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if completed.returncode != 0:
        raise RuntimeError(
            f"bundled DAE converter build failed (exit {completed.returncode}): {completed.stderr.strip()}"
        )
    if not output.is_file():
        raise RuntimeError(f"converter build reported success but binary is missing: {output}")
    return output


def _run_atomic_obj_conversion(plan) -> None:
    """Rebuild an OBJ without requiring Assimp to overwrite an open path."""

    output = plan.intermediate
    if output.suffix.lower() != ".obj":
        raise RuntimeError(f"this DAE converter requires an OBJ output, got: {output}")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.stem}.rebuild-",
        suffix=".obj",
        dir=output.parent,
    )
    os.close(descriptor)
    temporary = Path(temporary_name)
    temporary.unlink()
    temporary_materials: list[Path] = []
    try:
        command = (*plan.command[:-1], str(temporary))
        completed = subprocess.run(command, text=True, capture_output=True, check=False)
        if completed.returncode != 0:
            raise RuntimeError(
                f"conversion failed (exit {completed.returncode}): {completed.stderr.strip()}"
            )
        if not temporary.is_file() or temporary.stat().st_size == 0:
            raise RuntimeError(f"converter wrote no usable output: {temporary}")

        temporary_materials = obj_material_files(temporary)
        if len(temporary_materials) != 1 or not temporary_materials[0].is_file():
            raise RuntimeError(
                f"Assimp OBJ rebuild requires one material library, got {temporary_materials}"
            )
        final_material = output.with_suffix(".mtl")
        lines = temporary.read_text(encoding="utf-8", errors="replace").splitlines()
        rewritten = [
            f"mtllib {final_material.name}" if line.lstrip().startswith("mtllib ") else line
            for line in lines
        ]
        temporary.write_text("\n".join(rewritten) + "\n", encoding="utf-8")

        os.replace(temporary_materials[0], final_material)
        temporary_materials.clear()
        os.replace(temporary, output)
    finally:
        temporary.unlink(missing_ok=True)
        for material in temporary_materials:
            material.unlink(missing_ok=True)


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    manifest = load_manifest(args.manifest)

    if args.execute and args.assimp is None and shutil.which("assimp") is None:
        helper = bundled_converter_path(manifest)
        if not helper.is_file():
            print(f"building bundled libassimp helper: {helper}")
            _build_helper(manifest)

    plans = all_conversion_plans(
        manifest,
        directory=args.output_dir,
        assimp_executable=args.assimp,
        # An execution is a reproducible rebuild from the declared DAE inputs,
        # not a timestamp/cache hit. Planning without --execute remains a
        # non-mutating readiness check.
        refresh_existing=args.execute,
    )
    targets = {str(item["id"]): item for item in manifest["semantic_targets"]}
    failed = False
    for plan in plans:
        print(f"{plan.target_id}: {plan.state}: {plan.explanation}")
        if plan.source_dae is not None:
            print(f"  source: {plan.source_dae}")
        print(f"  output: {plan.intermediate}")
        if not args.execute:
            continue
        target = targets[plan.target_id]
        if plan.state == "ready":
            if plan.intermediate.suffix.lower() == ".obj":
                problems = unresolved_obj_material_dependencies(plan.intermediate, target)
                if problems:
                    print("  material validation unexpectedly failed: " + "; ".join(problems), file=sys.stderr)
                    failed = True
                else:
                    print("  all MTL texture references resolve")
            continue
        if plan.state != "convertible" or plan.command is None:
            failed = True
            continue
        plan.intermediate.parent.mkdir(parents=True, exist_ok=True)
        try:
            _run_atomic_obj_conversion(plan)
        except RuntimeError as exc:
            print(f"  {exc}", file=sys.stderr)
            failed = True
            continue
        if not plan.intermediate.is_file() or plan.intermediate.stat().st_size == 0:
            print(f"  converter wrote no usable output: {plan.intermediate}", file=sys.stderr)
            failed = True
            continue
        if plan.intermediate.suffix.lower() == ".obj":
            if plan.source_dae is None:
                print("  cannot stage OBJ materials without its DAE source", file=sys.stderr)
                failed = True
                continue
            try:
                prepare_obj_material_dependencies(plan.source_dae, plan.intermediate, target)
            except RuntimeError as exc:
                print(f"  {exc}", file=sys.stderr)
                failed = True
                continue
            print("  copied and validated all MTL texture dependencies")
        print(f"  wrote {plan.intermediate.stat().st_size} bytes")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
