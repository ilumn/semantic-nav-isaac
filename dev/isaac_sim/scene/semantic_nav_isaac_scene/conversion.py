"""Planning helpers for the explicit DAE -> OBJ/glTF -> USD asset path."""

from __future__ import annotations

import hashlib
import shutil
import shlex
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

from .runtime_args import resolve_asset_candidate, scene_root_for_manifest


@dataclass(frozen=True)
class ConversionPlan:
    target_id: str
    source_dae: Path | None
    intermediate: Path
    command: tuple[str, ...] | None
    state: str
    explanation: str


def intermediate_directory(manifest: Mapping[str, Any], override: str | Path | None = None) -> Path:
    if override is not None:
        return Path(override).expanduser().resolve()
    return scene_root_for_manifest(manifest) / "generated" / "semantic_assets" / "intermediate"


def bundled_converter_path(manifest: Mapping[str, Any]) -> Path:
    """Path produced by ``tools/build_dae_to_obj.sh`` (and ignored by git)."""

    return scene_root_for_manifest(manifest) / "generated" / "tools" / "dae_to_obj"


def _directive_tokens(path: Path, names: set[str]) -> list[tuple[str, str]]:
    directives: list[tuple[str, str]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            tokens = shlex.split(line, comments=True, posix=True)
        except ValueError:
            continue
        if len(tokens) >= 2 and tokens[0] in names:
            # Assimp may emit map options before the filename; the filename is
            # the final token for the material files used by this project.
            directives.append((tokens[0], tokens[-1]))
    return directives


def obj_material_files(obj_path: str | Path) -> list[Path]:
    obj = Path(obj_path).resolve()
    return [(obj.parent / value).resolve() for _, value in _directive_tokens(obj, {"mtllib"})]


def unresolved_obj_material_dependencies(
    obj_path: str | Path,
    target: Mapping[str, Any] | None = None,
) -> list[str]:
    """List missing MTL/texture bindings; an empty list means OBJ is ready."""

    obj = Path(obj_path).resolve()
    problems: list[str] = []
    if not obj.is_file():
        return [f"OBJ does not exist: {obj}"]
    material_files = obj_material_files(obj)
    if not material_files:
        problems.append(f"OBJ has no mtllib directive: {obj}")
        return problems

    configured = dict((target or {}).get("material_overrides", {}))
    seen_bindings: set[tuple[str, str]] = set()
    texture_directives = {"map_Ka", "map_Kd", "map_Ks", "map_Ke", "map_Ns", "map_d", "bump", "map_bump", "norm"}
    for material in material_files:
        if not material.is_file():
            problems.append(f"missing material library: {material}")
            continue
        for directive, value in _directive_tokens(material, texture_directives):
            seen_bindings.add((directive, value))
            texture = Path(value)
            if not texture.is_absolute():
                texture = material.parent / texture
            if not texture.resolve().is_file():
                problems.append(f"unresolved {directive} texture in {material}: {value}")

    for directive, value in configured.items():
        if (str(directive), str(value)) not in seen_bindings:
            problems.append(f"missing required {directive} binding: {value}")
    return problems


def intermediate_dependency_files(
    intermediate_path: str | Path,
    target: Mapping[str, Any] | None = None,
) -> list[Path]:
    """Return every file whose bytes affect an intermediate asset.

    OBJ assets are not self-contained: both their material libraries and every
    texture referenced by those libraries affect the USD produced by
    Omniverse Asset Converter.  Keeping this dependency walk next to the MTL
    validator prevents the two definitions from drifting apart.
    """

    intermediate = Path(intermediate_path).resolve()
    if not intermediate.is_file():
        raise FileNotFoundError(f"intermediate asset does not exist: {intermediate}")
    if intermediate.suffix.lower() != ".obj":
        return [intermediate]

    problems = unresolved_obj_material_dependencies(intermediate, target)
    if problems:
        raise RuntimeError("OBJ material dependency validation failed:\n  - " + "\n  - ".join(problems))

    dependencies = [intermediate]
    texture_directives = {
        "map_Ka",
        "map_Kd",
        "map_Ks",
        "map_Ke",
        "map_Ns",
        "map_d",
        "bump",
        "map_bump",
        "norm",
    }
    for material in obj_material_files(intermediate):
        dependencies.append(material)
        for _, value in _directive_tokens(material, texture_directives):
            texture = Path(value)
            if not texture.is_absolute():
                texture = material.parent / texture
            dependencies.append(texture.resolve())

    # A texture can be referenced by multiple materials. Hash its bytes once,
    # while retaining a deterministic path order for reproducible cache keys.
    return sorted(set(dependencies), key=lambda path: str(path))


def intermediate_asset_fingerprint(
    intermediate_path: str | Path,
    target: Mapping[str, Any] | None = None,
) -> str:
    """SHA-256 cache key for an intermediate and all material dependencies."""

    intermediate = Path(intermediate_path).resolve()
    dependencies = intermediate_dependency_files(intermediate, target)
    digest = hashlib.sha256()
    for dependency in dependencies:
        try:
            logical_name = dependency.relative_to(intermediate.parent.parent)
        except ValueError:
            logical_name = Path(dependency.name)
        payload = dependency.read_bytes()
        digest.update(str(logical_name).encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(len(payload)).encode("ascii"))
        digest.update(b"\0")
        digest.update(payload)
        digest.update(b"\0")
    return digest.hexdigest()


def _copy_tree_without_collisions(source: Path, destination: Path) -> None:
    for source_file in sorted(path for path in source.rglob("*") if path.is_file()):
        relative = source_file.relative_to(source)
        destination_file = destination / relative
        if destination_file.is_file():
            if destination_file.read_bytes() != source_file.read_bytes():
                raise RuntimeError(
                    f"material dependency collision at {destination_file}; source was {source_file}"
                )
            continue
        destination_file.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source_file, destination_file)


def prepare_obj_material_dependencies(
    source_dae: str | Path,
    obj_path: str | Path,
    target: Mapping[str, Any],
) -> None:
    """Copy Gazebo model materials and apply any declarative MTL overrides."""

    source = Path(source_dae).resolve()
    obj = Path(obj_path).resolve()
    source_materials = source.parent.parent / "materials"
    if source_materials.is_dir():
        # Assimp preserves ../materials/... references from a DAE placed in its
        # model's meshes directory.  Mirror that layout beside intermediate/.
        _copy_tree_without_collisions(source_materials, obj.parent.parent / "materials")

    overrides = dict(target.get("material_overrides", {}))
    if overrides:
        material_files = obj_material_files(obj)
        if len(material_files) != 1 or not material_files[0].is_file():
            raise RuntimeError(
                f"material overrides for {target['id']} require exactly one existing MTL, got {material_files}"
            )
        material = material_files[0]
        lines = material.read_text(encoding="utf-8", errors="replace").splitlines()
        override_names = {str(name) for name in overrides}
        filtered: list[str] = []
        for line in lines:
            try:
                tokens = shlex.split(line, comments=True, posix=True)
            except ValueError:
                tokens = []
            if tokens and tokens[0] in override_names:
                continue
            filtered.append(line)
        filtered.extend(f"{name} {value}" for name, value in overrides.items())
        material.write_text("\n".join(filtered) + "\n", encoding="utf-8")

    problems = unresolved_obj_material_dependencies(obj, target)
    if problems:
        raise RuntimeError("OBJ material dependency validation failed:\n  - " + "\n  - ".join(problems))


def find_existing_intermediate(
    manifest: Mapping[str, Any],
    target: Mapping[str, Any],
    directory: str | Path | None = None,
) -> Path | None:
    base = intermediate_directory(manifest, directory)
    for name in target["intermediate_names"]:
        path = (base / str(name)).resolve()
        if path.is_file() and (
            path.suffix.lower() != ".obj" or not unresolved_obj_material_dependencies(path, target)
        ):
            return path
    return None


def require_enabled_intermediates(
    manifest: Mapping[str, Any],
    directory: str | Path | None = None,
) -> dict[str, Path]:
    """Return validated intermediates or fail before Isaac starts.

    Enabled semantic targets are acceptance-critical camera content.  They must
    never be replaced silently by primitive geometry just because a generated,
    gitignored cache is absent in a fresh checkout.
    """

    ready: dict[str, Path] = {}
    missing: list[str] = []
    for target in manifest["semantic_targets"]:
        if not bool(target["enabled"]):
            continue
        intermediate = find_existing_intermediate(manifest, target, directory)
        if intermediate is None:
            missing.append(str(target["id"]))
        else:
            ready[str(target["id"])] = intermediate
    if missing:
        scene_root = scene_root_for_manifest(manifest)
        generator = scene_root / "tools" / "convert_semantic_assets.py"
        raise FileNotFoundError(
            "validated OBJ/glTF intermediates are required for enabled semantic targets "
            f"{', '.join(missing)}; generate them before launch with:\n"
            f"  python3 {generator} --execute"
        )
    return ready


def plan_dae_conversion(
    manifest: Mapping[str, Any],
    target: Mapping[str, Any],
    *,
    directory: str | Path | None = None,
    assimp_executable: str | None = None,
    refresh_existing: bool = False,
) -> ConversionPlan:
    """Plan one semantic asset conversion without executing external programs."""

    target_id = str(target["id"])
    output_dir = intermediate_directory(manifest, directory)
    source = resolve_asset_candidate(manifest, target["dae_candidates"])
    existing = None if refresh_existing else find_existing_intermediate(manifest, target, output_dir)
    if existing is not None:
        return ConversionPlan(
            target_id,
            source,
            existing,
            None,
            "ready",
            "an OBJ/glTF intermediate already exists",
        )

    output = output_dir / str(target["intermediate_names"][0])
    if source is None:
        return ConversionPlan(
            target_id,
            None,
            output,
            None,
            "missing-source",
            "none of the declared Collada source paths exists",
        )

    assimp_cli = assimp_executable or shutil.which("assimp")
    helper = bundled_converter_path(manifest)
    if assimp_cli is None and not helper.is_file():
        return ConversionPlan(
            target_id,
            source,
            output,
            None,
            "missing-converter",
            "build the bundled libassimp helper with tools/build_dae_to_obj.sh",
        )

    if assimp_cli is not None:
        command = (str(Path(assimp_cli).resolve()), "export", str(source), str(output))
        explanation = "Assimp CLI can create the OBJ intermediate accepted by Omniverse Asset Converter"
    else:
        command = (str(helper.resolve()), str(source), str(output))
        explanation = (
            "the bundled libassimp helper can create the OBJ intermediate accepted by "
            "Omniverse Asset Converter"
        )

    return ConversionPlan(
        target_id,
        source,
        output,
        command,
        "convertible",
        explanation,
    )


def all_conversion_plans(
    manifest: Mapping[str, Any],
    *,
    directory: str | Path | None = None,
    assimp_executable: str | None = None,
    refresh_existing: bool = False,
) -> list[ConversionPlan]:
    return [
        plan_dae_conversion(
            manifest,
            target,
            directory=directory,
            assimp_executable=assimp_executable,
            refresh_existing=refresh_existing,
        )
        for target in manifest["semantic_targets"]
    ]
