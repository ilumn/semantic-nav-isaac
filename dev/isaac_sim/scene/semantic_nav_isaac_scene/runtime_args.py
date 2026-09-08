"""Pure-Python argument and path resolution helpers for the scene runner."""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence


_ENV_PATTERN = re.compile(r"\$\{([A-Za-z_][A-Za-z0-9_]*)\}|\$([A-Za-z_][A-Za-z0-9_]*)")
_XACRO_MARKER_PATTERN = re.compile(r"<\s*xacro:|\$\{[^}]+\}")
_XML_COMMENT_PATTERN = re.compile(r"<!--.*?-->", re.DOTALL)


@dataclass(frozen=True)
class XacroExpansionPlan:
    source: Path
    output: Path
    command: tuple[str, ...] | None
    requires_expansion: bool
    cached: bool


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Compose or open the TurtleBot3 semantic-navigation scene in Isaac Sim 6.0.1.",
    )
    parser.add_argument("--headless", action="store_true", help="Run without the Isaac Sim desktop UI.")
    parser.add_argument(
        "--stage-path",
        type=Path,
        help="Open this USD if it exists; otherwise compose and save a new USD here.",
    )
    parser.add_argument(
        "--rebuild-stage",
        action="store_true",
        help="Recompose and replace --stage-path instead of opening an existing managed stage.",
    )
    parser.add_argument("--manifest", type=Path, help="Override config/scene_manifest.json.")
    parser.add_argument("--urdf-path", type=Path, help="Override TurtleBot3 Waffle Pi URDF discovery.")
    parser.add_argument(
        "--xacro-executable",
        type=Path,
        help="Override xacro discovery when the .urdf contains xacro expressions.",
    )
    parser.add_argument(
        "--asset-cache",
        type=Path,
        help="Runtime-only cache for imported robot and converted semantic USD assets.",
    )
    parser.add_argument(
        "--max-steps",
        type=positive_int,
        help="Stop after this many app updates; otherwise run until the UI closes or SIGINT.",
    )
    parser.add_argument(
        "--compose-only",
        action="store_true",
        help="Compose/save the stage and exit without starting the simulation timeline.",
    )
    return parser


def positive_int(text: str) -> int:
    value = int(text)
    if value <= 0:
        raise argparse.ArgumentTypeError("value must be greater than zero")
    return value


def expand_template(value: str, environment: Mapping[str, str]) -> str | None:
    """Expand ``$NAME``/``${NAME}``, returning ``None`` for missing variables."""

    missing = False

    def replace(match: re.Match[str]) -> str:
        nonlocal missing
        name = match.group(1) or match.group(2)
        if name not in environment:
            missing = True
            return match.group(0)
        return environment[name]

    expanded = _ENV_PATTERN.sub(replace, value)
    return None if missing else os.path.expanduser(expanded)


def resolve_candidate(
    value: str | Path,
    *,
    base_dir: str | Path,
    environment: Mapping[str, str] | None = None,
) -> Path | None:
    """Resolve a possibly templated path and return it only when it exists."""

    env = os.environ if environment is None else environment
    expanded = expand_template(str(value), env)
    if expanded is None:
        return None
    candidate = Path(expanded)
    if not candidate.is_absolute():
        candidate = Path(base_dir) / candidate
    candidate = candidate.resolve()
    return candidate if candidate.exists() else None


def scene_root_for_manifest(manifest: Mapping[str, Any]) -> Path:
    manifest_path = Path(str(manifest["_manifest_path"])).resolve()
    if manifest_path.parent.name == "config":
        return manifest_path.parent.parent
    return manifest_path.parent


def resolve_urdf_path(
    manifest: Mapping[str, Any],
    explicit: str | Path | None = None,
    *,
    environment: Mapping[str, str] | None = None,
) -> Path:
    """Resolve the Waffle Pi URDF from CLI, env override, then manifest candidates."""

    env = dict(os.environ if environment is None else environment)
    env.setdefault("ROS_DISTRO", str(manifest["target"]["ros_distro"]))
    robot = manifest["robot"]
    attempts: list[str] = []

    if explicit is not None:
        path = Path(explicit).expanduser().resolve()
        attempts.append(str(path))
        if path.is_file() and path.suffix.lower() == ".urdf":
            return path
        raise FileNotFoundError(f"--urdf-path is not a readable .urdf file: {path}")

    override_name = str(robot["urdf_env"])
    override = env.get(override_name)
    if override:
        path = Path(override).expanduser().resolve()
        attempts.append(f"{override_name}={path}")
        if path.is_file() and path.suffix.lower() == ".urdf":
            return path
        raise FileNotFoundError(f"{override_name} does not name a readable .urdf file: {path}")

    base_dir = scene_root_for_manifest(manifest)
    for value in robot["urdf_candidates"]:
        expanded = expand_template(str(value), env)
        attempts.append(str(value) if expanded is None else expanded)
        path = resolve_candidate(value, base_dir=base_dir, environment=env)
        if path is not None and path.is_file() and path.suffix.lower() == ".urdf":
            return path
    formatted = "\n  - ".join(attempts)
    raise FileNotFoundError(
        f"TurtleBot3 Waffle Pi URDF was not found. Checked:\n  - {formatted}\n"
        f"Set {override_name} or pass --urdf-path."
    )


def infer_ros_package_mapping(urdf_path: str | Path, package_name: str) -> dict[str, str]:
    """Infer the ``name``/``path`` mapping required by URDFImporterConfig."""

    path = Path(urdf_path).resolve()
    candidates = [path.parent, *path.parents]
    for candidate in candidates:
        if candidate.name == package_name and (candidate / "meshes").is_dir():
            return {"name": package_name, "path": str(candidate)}
    raise FileNotFoundError(
        f"cannot infer ROS package {package_name!r} from URDF path {path}; "
        "the package root must contain a meshes directory"
    )


def urdf_requires_xacro(path: str | Path) -> bool:
    """Return whether a nominal ``.urdf`` still contains xacro syntax."""

    source = Path(path).expanduser().resolve()
    try:
        text = source.read_text(encoding="utf-8")
    except FileNotFoundError as exc:
        raise FileNotFoundError(f"URDF source does not exist: {source}") from exc
    # xacro deliberately preserves XML comments.  The installed Waffle Pi file
    # contains a commented xacro:include, so comments cannot be treated as live
    # syntax when validating the expanded output.
    uncommented = _XML_COMMENT_PATTERN.sub("", text)
    return _XACRO_MARKER_PATTERN.search(uncommented) is not None


def resolve_xacro_executable(
    ros_distro: str,
    *,
    explicit: str | Path | None = None,
    environment: Mapping[str, str] | None = None,
) -> Path | None:
    """Resolve xacro without requiring a sourced shell."""

    env = os.environ if environment is None else environment
    candidates: list[str | Path] = []
    if explicit is not None:
        candidates.append(explicit)
    elif env.get("XACRO_EXECUTABLE"):
        candidates.append(str(env["XACRO_EXECUTABLE"]))
    else:
        on_path = shutil.which("xacro", path=env.get("PATH"))
        if on_path:
            candidates.append(on_path)
        candidates.append(Path("/opt/ros") / ros_distro / "bin" / "xacro")

    for value in candidates:
        candidate = Path(value).expanduser().resolve()
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate
    return None


def plan_xacro_expansion(
    source: str | Path,
    cache_dir: str | Path,
    *,
    ros_distro: str,
    explicit_executable: str | Path | None = None,
    environment: Mapping[str, str] | None = None,
) -> XacroExpansionPlan:
    """Build the argv-only xacro plan used by the runtime importer."""

    source_path = Path(source).expanduser().resolve()
    output = Path(cache_dir).expanduser().resolve() / "robot" / f"{source_path.stem}.expanded.urdf"
    required = urdf_requires_xacro(source_path)
    if not required:
        return XacroExpansionPlan(source_path, source_path, None, False, True)
    executable = resolve_xacro_executable(
        ros_distro,
        explicit=explicit_executable,
        environment=environment,
    )
    if executable is None:
        expected = Path("/opt/ros") / ros_distro / "bin" / "xacro"
        raise FileNotFoundError(
            f"{source_path} contains xacro syntax but no executable was found. "
            f"Install/source ROS xacro, set XACRO_EXECUTABLE, pass --xacro-executable, "
            f"or provide {expected}."
        )
    cached = output.is_file() and output.stat().st_mtime_ns >= source_path.stat().st_mtime_ns
    return XacroExpansionPlan(
        source_path,
        output,
        (str(executable), str(source_path), "namespace:="),
        True,
        cached,
    )


def prepare_importable_urdf(
    source: str | Path,
    cache_dir: str | Path,
    *,
    ros_distro: str,
    explicit_executable: str | Path | None = None,
    environment: Mapping[str, str] | None = None,
) -> Path:
    """Expand xacro into the runtime cache and return importer-ready XML."""

    plan = plan_xacro_expansion(
        source,
        cache_dir,
        ros_distro=ros_distro,
        explicit_executable=explicit_executable,
        environment=environment,
    )
    if not plan.requires_expansion:
        return plan.output
    if plan.cached and plan.output.stat().st_size > 0 and not urdf_requires_xacro(plan.output):
        return plan.output
    assert plan.command is not None
    plan.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = plan.output.with_suffix(plan.output.suffix + f".tmp.{os.getpid()}")
    try:
        with temporary.open("wb") as stream:
            process_environment = dict(os.environ if environment is None else environment)
            prefix = Path("/opt/ros") / ros_distro
            python_sites = sorted((prefix / "lib").glob("python*/site-packages"))
            if python_sites:
                existing_pythonpath = process_environment.get("PYTHONPATH", "")
                additions = os.pathsep.join(str(path) for path in python_sites)
                process_environment["PYTHONPATH"] = (
                    additions + (os.pathsep + existing_pythonpath if existing_pythonpath else "")
                )
            process_environment.setdefault("AMENT_PREFIX_PATH", str(prefix))
            process_environment.setdefault("ROS_DISTRO", ros_distro)
            completed = subprocess.run(
                plan.command,
                stdout=stream,
                stderr=subprocess.PIPE,
                env=process_environment,
                check=False,
            )
        if completed.returncode != 0:
            stderr = completed.stderr.decode("utf-8", errors="replace").strip()
            raise RuntimeError(
                f"xacro failed while expanding {plan.source} (exit {completed.returncode}): {stderr}"
            )
        if not temporary.is_file() or temporary.stat().st_size == 0:
            raise RuntimeError(f"xacro produced an empty URDF for {plan.source}")
        if urdf_requires_xacro(temporary):
            raise RuntimeError(f"xacro output still contains unresolved expressions: {temporary}")
        os.replace(temporary, plan.output)
    finally:
        if temporary.exists():
            temporary.unlink()
    return plan.output


def resolve_asset_candidate(
    manifest: Mapping[str, Any],
    candidates: Sequence[str],
    *,
    environment: Mapping[str, str] | None = None,
) -> Path | None:
    base_dir = scene_root_for_manifest(manifest)
    for candidate in candidates:
        resolved = resolve_candidate(candidate, base_dir=base_dir, environment=environment)
        if resolved is not None and resolved.is_file():
            return resolved
    return None


def stage_mode(stage_path: str | Path | None, *, rebuild: bool = False) -> str:
    """Return ``memory``, ``open``, or ``compose`` for CLI stage semantics."""

    if stage_path is None:
        return "memory"
    path = Path(stage_path).expanduser().resolve()
    if path.exists() and not path.is_file():
        raise ValueError(f"stage path exists but is not a file: {path}")
    if path.exists() and not rebuild:
        return "open"
    return "compose"


def validate_stage_path(stage_path: str | Path) -> Path:
    path = Path(stage_path).expanduser().resolve()
    if path.suffix.lower() not in {".usd", ".usda", ".usdc"}:
        raise ValueError(f"stage path must end in .usd, .usda, or .usdc: {path}")
    return path
