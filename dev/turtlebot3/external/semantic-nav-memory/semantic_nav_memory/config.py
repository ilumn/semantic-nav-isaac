from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

import yaml


ROOT_DIR = Path(__file__).resolve().parent.parent
FRONTEND_DIR = ROOT_DIR / "frontend"
DATA_DIR = ROOT_DIR / "data"
JOBS_DIR = DATA_DIR / "jobs"
MODELS_DIR = ROOT_DIR / "assets" / "models"
YOLO_CONFIG_DIR = ROOT_DIR / ".ultralytics"
RUNTIME_HOME_DIR = ROOT_DIR / ".runtime_home"
CACHE_DIR = ROOT_DIR / ".cache"
PROMPTS_PATH = ROOT_DIR / "semantic_nav_memory" / "prompts.yaml"
YOLO_WORLD_MODEL_SPECS = {
    "small": {
        "label": "Small",
        "checkpoint": "yolov8s-worldv2.pt",
        "description": "Fastest general-purpose option.",
    },
    "medium": {
        "label": "Medium",
        "checkpoint": "yolov8m-worldv2.pt",
        "description": "More detail on mid-size objects.",
    },
    "large": {
        "label": "Large",
        "checkpoint": "yolov8l-worldv2.pt",
        "description": "Better recall for complex scenes.",
    },
    "xlarge": {
        "label": "XL",
        "checkpoint": "yolov8x-worldv2.pt",
        "description": "Highest detail, slowest runtime.",
    },
}

AERIAL_KEYWORDS = {
    "aerial",
    "drone",
    "roof",
    "neighborhood",
    "suburb",
    "suburban",
    "backyard",
    "pool",
    "driveway",
    "street",
    "birdseye",
    "bird's-eye",
}
OUTDOOR_KEYWORDS = {
    "car",
    "truck",
    "bicycle",
    "motorcycle",
    "building",
    "house",
    "roof",
    "tree",
    "bush",
    "road",
    "driveway",
    "sidewalk",
    "fence",
    "garage",
    "mailbox",
    "utility pole",
    "lawn",
    "pool",
    "street",
    "backyard",
}
INDOOR_KEYWORDS = {
    "chair",
    "desk",
    "table",
    "door",
    "monitor",
    "keyboard",
    "cup",
    "bottle",
    "backpack",
    "cabinet",
    "plant",
    "trash can",
    "sofa",
    "bed",
}


def _prompt_payload() -> dict:
    with PROMPTS_PATH.open("r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _dedupe(values: list[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for value in values:
        normalized = value.strip()
        if not normalized:
            continue
        lowered = normalized.lower()
        if lowered in seen:
            continue
        seen.add(lowered)
        result.append(normalized)
    return result


def default_prompt_preset() -> str:
    return str(_prompt_payload().get("default_preset", "general"))


def prompt_presets() -> dict[str, dict]:
    payload = _prompt_payload()
    presets = payload.get("presets", {})
    return presets if isinstance(presets, dict) else {}


def prompt_preset_details(name: str) -> dict:
    presets = prompt_presets()
    if name in presets:
        return presets[name]
    return presets.get(default_prompt_preset(), {})


def preset_prompts(name: str) -> list[str]:
    details = prompt_preset_details(name)
    prompts = details.get("prompts", [])
    return _dedupe(prompts if isinstance(prompts, list) else [])


def infer_scene_profile(video_name: str, prompt_preset: str, prompt_vocabulary: list[str]) -> str:
    preset_profile = str(prompt_preset_details(prompt_preset).get("scene_profile", "general"))
    if preset_profile in {"indoor", "outdoor", "aerial"}:
        return preset_profile

    haystack = " ".join([video_name, *prompt_vocabulary]).lower()
    if any(keyword in haystack for keyword in AERIAL_KEYWORDS):
        return "aerial"

    outdoor_hits = sum(keyword in haystack for keyword in OUTDOOR_KEYWORDS)
    indoor_hits = sum(keyword in haystack for keyword in INDOOR_KEYWORDS)
    if outdoor_hits >= max(2, indoor_hits + 1):
        return "outdoor"
    if indoor_hits >= max(2, outdoor_hits + 1):
        return "indoor"
    return "general"


def resolve_prompt_request(video_name: str, preset: str | None, extra_prompts: list[str]) -> tuple[str, str, list[str]]:
    selected_preset = preset if preset in prompt_presets() else default_prompt_preset()
    prompt_vocabulary = _dedupe([*preset_prompts(selected_preset), *extra_prompts])
    if not prompt_vocabulary:
        selected_preset = default_prompt_preset()
        prompt_vocabulary = preset_prompts(selected_preset)
    scene_profile = infer_scene_profile(video_name, selected_preset, prompt_vocabulary)
    if selected_preset == default_prompt_preset() and scene_profile in prompt_presets() and scene_profile != "general":
        selected_preset = scene_profile
        prompt_vocabulary = _dedupe([*preset_prompts(selected_preset), *extra_prompts])
    return selected_preset, scene_profile, prompt_vocabulary


DEFAULT_PROMPTS = preset_prompts(default_prompt_preset())


def default_yolo_model_size() -> str:
    return "small"


def _yolo_model_candidates(checkpoint: str) -> list[Path]:
    return [
        MODELS_DIR / checkpoint,
        YOLO_CONFIG_DIR / "weights" / checkpoint,
    ]


def resolve_yolo_model_selection(model_size: str | None) -> tuple[str, str]:
    selected_size = model_size if model_size in YOLO_WORLD_MODEL_SPECS else default_yolo_model_size()
    checkpoint = str(YOLO_WORLD_MODEL_SPECS[selected_size]["checkpoint"])
    for candidate in _yolo_model_candidates(checkpoint):
        if candidate.exists():
            return selected_size, str(candidate)
    return selected_size, checkpoint


def yolo_world_model_variants() -> dict[str, dict[str, str | bool]]:
    variants: dict[str, dict[str, str | bool]] = {}
    for size, spec in YOLO_WORLD_MODEL_SPECS.items():
        checkpoint = str(spec["checkpoint"])
        variants[size] = {
            "label": str(spec["label"]),
            "description": str(spec["description"]),
            "checkpoint": checkpoint,
            "cached": any(candidate.exists() for candidate in _yolo_model_candidates(checkpoint)),
        }
    return variants


@dataclass(slots=True)
class PipelineConfig:
    frame_sample_fps: float = 2.0
    max_keyframes: int = 128
    max_image_size: int = 1440
    yolo_model_size: str = field(default_factory=default_yolo_model_size)
    yolo_model_name: str = ""
    yolo_confidence: float = 0.18
    yolo_image_size: int = 960
    yolo_prompt_batch_size: int = 12
    yolo_tile_overlap_ratio: float = 0.16
    yolo_nms_iou: float = 0.45
    prompt_vocabulary: list[str] = field(default_factory=lambda: list(DEFAULT_PROMPTS))
    prompt_preset: str = field(default_factory=default_prompt_preset)
    scene_profile: str = "general"
    relation_near_ratio: float = 0.1
    place_cluster_ratio: float = 0.18
    entity_merge_ratio: float = 0.08
    reconstruction_random_seed: int = 7
    reconstruction_min_registered_keyframes: int = 8
    reconstruction_min_registered_ratio: float = 0.35
    reconstruction_max_serialized_points: int = 100000
    reconstruction_sift_max_num_features: int = 20000
    reconstruction_sift_peak_threshold: float = 0.0035
    reconstruction_use_domain_size_pooling: bool = True
    reconstruction_use_affine_shape: bool = False
    camera_fx: float | None = None
    camera_fy: float | None = None
    camera_cx: float | None = None
    camera_cy: float | None = None
    camera_width: int | None = None
    camera_height: int | None = None

    def __post_init__(self) -> None:
        resolved_size, resolved_name = resolve_yolo_model_selection(self.yolo_model_size)
        self.yolo_model_size = resolved_size
        if not self.yolo_model_name:
            self.yolo_model_name = resolved_name


def ensure_runtime_dirs() -> None:
    for path in (DATA_DIR, JOBS_DIR, MODELS_DIR, YOLO_CONFIG_DIR, RUNTIME_HOME_DIR, CACHE_DIR):
        path.mkdir(parents=True, exist_ok=True)


def configure_environment() -> None:
    ensure_runtime_dirs()
    os.environ.setdefault("YOLO_CONFIG_DIR", str(YOLO_CONFIG_DIR))
    os.environ.setdefault("XDG_CACHE_HOME", str(CACHE_DIR))
    home_cache = Path.home() / ".cache"
    if not home_cache.exists() or not os.access(home_cache, os.W_OK):
        os.environ["HOME"] = str(RUNTIME_HOME_DIR)
