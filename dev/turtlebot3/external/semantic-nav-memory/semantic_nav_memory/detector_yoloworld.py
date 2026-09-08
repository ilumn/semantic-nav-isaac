from __future__ import annotations

import os
from pathlib import Path

import cv2

from .config import MODELS_DIR, PipelineConfig, YOLO_CONFIG_DIR, configure_environment
from .detector import Detector
from .models import BBox, Detection, Keyframe


os.environ.setdefault("YOLO_CONFIG_DIR", str(YOLO_CONFIG_DIR))

# Ultralytics' YOLO-World text encoder calls clip.load() without a cache path,
# which otherwise reaches into ~/.cache and may trigger a hidden 338 MiB
# download. Keep the required ViT-B/32 checkpoint beside the YOLO-World model
# so this vendored worker remains self-contained and preflight-verifiable.
import clip


_clip_load = clip.load


def _load_project_clip(*args, **kwargs):
    kwargs.setdefault("download_root", str(MODELS_DIR))
    return _clip_load(*args, **kwargs)


clip.load = _load_project_clip

from ultralytics import YOLOWorld, settings


def _chunks(values: list[str], size: int) -> list[list[str]]:
    chunk_size = max(1, size)
    return [values[index : index + chunk_size] for index in range(0, len(values), chunk_size)]


def _iou(left: BBox, right: BBox) -> float:
    x1 = max(left.x1, right.x1)
    y1 = max(left.y1, right.y1)
    x2 = min(left.x2, right.x2)
    y2 = min(left.y2, right.y2)
    inter_w = max(0.0, x2 - x1)
    inter_h = max(0.0, y2 - y1)
    intersection = inter_w * inter_h
    union = left.width * left.height + right.width * right.height - intersection
    if union <= 0:
        return 0.0
    return intersection / union


def _nms(detections: list[Detection], threshold: float) -> list[Detection]:
    accepted: list[Detection] = []
    for candidate in sorted(detections, key=lambda item: item.confidence, reverse=True):
        suppress = False
        for kept in accepted:
            if candidate.detector_label != kept.detector_label:
                continue
            if _iou(candidate.bbox, kept.bbox) >= threshold:
                suppress = True
                break
        if not suppress:
            accepted.append(candidate)
    return accepted


def _tile_specs(width: int, height: int, scene_profile: str, overlap_ratio: float) -> list[tuple[str, int, int, int, int]]:
    tiles: list[tuple[str, int, int, int, int]] = [("full", 0, 0, width, height)]
    if scene_profile not in {"outdoor", "aerial"}:
        return tiles

    grid_size = 2
    overlap_x = int(width * overlap_ratio / grid_size)
    overlap_y = int(height * overlap_ratio / grid_size)
    tile_width = max(width // grid_size, 64)
    tile_height = max(height // grid_size, 64)

    for row in range(grid_size):
        for col in range(grid_size):
            x1 = max(0, col * tile_width - overlap_x)
            y1 = max(0, row * tile_height - overlap_y)
            x2 = min(width, (col + 1) * tile_width + overlap_x)
            y2 = min(height, (row + 1) * tile_height + overlap_y)
            tiles.append((f"tile_{row + 1}{col + 1}", x1, y1, x2, y2))
    return tiles


def _box_color(label: str) -> tuple[int, int, int]:
    base = abs(hash(label))
    blue = 90 + base % 120
    green = 120 + (base // 7) % 100
    red = 140 + (base // 17) % 90
    return (blue, green, red)


class YOLOWorldDetector(Detector):
    def __init__(self, config: PipelineConfig):
        configure_environment()
        os.environ.setdefault("YOLO_CONFIG_DIR", str(YOLO_CONFIG_DIR))
        settings.update({"weights_dir": str(YOLO_CONFIG_DIR / "weights"), "runs_dir": str(YOLO_CONFIG_DIR / "runs")})
        self.config = config
        self.model = YOLOWorld(config.yolo_model_name)

    def _effective_imgsz(self) -> int:
        if self.config.scene_profile == "aerial":
            return max(self.config.yolo_image_size, 1280)
        if self.config.scene_profile == "outdoor":
            return max(self.config.yolo_image_size, 1120)
        return self.config.yolo_image_size

    def _effective_confidence(self) -> float:
        if self.config.scene_profile == "aerial":
            return min(self.config.yolo_confidence, 0.12)
        if self.config.scene_profile == "outdoor":
            return min(self.config.yolo_confidence, 0.15)
        return self.config.yolo_confidence

    def _render_annotations(self, image, detections: list[Detection], output_path: Path) -> None:
        canvas = image.copy()
        for detection in detections:
            color = _box_color(detection.detector_label)
            x1, y1, x2, y2 = (
                int(round(detection.bbox.x1)),
                int(round(detection.bbox.y1)),
                int(round(detection.bbox.x2)),
                int(round(detection.bbox.y2)),
            )
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
            label = f"{detection.detector_label} {detection.confidence:.2f}"
            cv2.putText(
                canvas,
                label,
                (x1, max(18, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                color,
                1,
                cv2.LINE_AA,
            )
        cv2.imwrite(str(output_path), canvas)

    def detect(self, keyframes: list[Keyframe], output_dir: Path) -> list[Detection]:
        output_dir.mkdir(parents=True, exist_ok=True)
        detections: list[Detection] = []
        prompt_batches = _chunks(self.config.prompt_vocabulary, self.config.yolo_prompt_batch_size)
        imgsz = self._effective_imgsz()
        confidence = self._effective_confidence()

        for frame_index, keyframe in enumerate(keyframes):
            image = cv2.imread(keyframe.image_path)
            if image is None:
                continue

            frame_height, frame_width = image.shape[:2]
            tiles = _tile_specs(frame_width, frame_height, self.config.scene_profile, self.config.yolo_tile_overlap_ratio)
            frame_candidates: list[Detection] = []

            for batch_index, prompt_batch in enumerate(prompt_batches, start=1):
                self.model.set_classes(prompt_batch)
                for tile_id, x1, y1, x2, y2 in tiles:
                    crop = image[y1:y2, x1:x2]
                    if crop.size == 0:
                        continue
                    results = self.model.predict(
                        source=crop,
                        conf=confidence,
                        imgsz=imgsz,
                        verbose=False,
                        device="cpu",
                    )
                    if not results:
                        continue

                    result = results[0]
                    if result.boxes is None or result.boxes.cls is None:
                        continue

                    boxes = result.boxes.xyxy.cpu().tolist()
                    scores = result.boxes.conf.cpu().tolist()
                    class_ids = result.boxes.cls.cpu().tolist()
                    for local_index, (box, score, class_id) in enumerate(zip(boxes, scores, class_ids, strict=False), start=1):
                        global_box = BBox(
                            x1=float(box[0] + x1),
                            y1=float(box[1] + y1),
                            x2=float(box[2] + x1),
                            y2=float(box[3] + y1),
                        )
                        area_ratio = (global_box.width * global_box.height) / max(frame_width * frame_height, 1)
                        label = str(result.names[int(class_id)])
                        frame_candidates.append(
                            Detection(
                                detection_id=f"det_{frame_index + 1:03d}_{batch_index:02d}_{local_index:03d}_{tile_id}",
                                keyframe_id=keyframe.keyframe_id,
                                detector_label=label,
                                confidence=float(score),
                                bbox=global_box,
                                class_distribution={label: float(score)},
                                prompt_source=f"preset:{self.config.prompt_preset}",
                                prompt_batch=list(prompt_batch),
                                tile_id=tile_id,
                                tile_bounds=BBox(x1=float(x1), y1=float(y1), x2=float(x2), y2=float(y2)),
                                bbox_area_ratio=float(area_ratio),
                            )
                        )

            merged = _nms(frame_candidates, self.config.yolo_nms_iou)
            annotated_path = output_dir / f"{Path(keyframe.image_name).stem}_annotated.jpg"
            self._render_annotations(image, merged, annotated_path)
            for merged_index, detection in enumerate(merged, start=1):
                detection.detection_id = f"det_{frame_index + 1:03d}_{merged_index:03d}"
                detection.annotated_image_path = str(annotated_path)
                detections.append(detection)
        return detections
