from __future__ import annotations

from collections import defaultdict

from .models import Detection, Tracklet


def _iou(a, b) -> float:
    x1 = max(a.x1, b.x1)
    y1 = max(a.y1, b.y1)
    x2 = min(a.x2, b.x2)
    y2 = min(a.y2, b.y2)
    inter_w = max(0.0, x2 - x1)
    inter_h = max(0.0, y2 - y1)
    intersection = inter_w * inter_h
    union = a.width * a.height + b.width * b.height - intersection
    if union <= 0:
        return 0.0
    return intersection / union


def build_tracklets(detections: list[Detection]) -> list[Tracklet]:
    detections_by_label: dict[str, list[Detection]] = defaultdict(list)
    for detection in detections:
        detections_by_label[detection.detector_label].append(detection)

    tracklets: list[Tracklet] = []
    for label, items in detections_by_label.items():
        items = sorted(items, key=lambda item: (item.keyframe_id, item.detection_id))
        active_tracks: list[dict] = []
        for detection in items:
            best_track = None
            best_iou = 0.0
            for track in active_tracks:
                score = _iou(track["last_bbox"], detection.bbox)
                if score > best_iou:
                    best_iou = score
                    best_track = track
            if best_track and best_iou >= 0.2:
                best_track["detections"].append(detection)
                best_track["last_bbox"] = detection.bbox
            else:
                active_tracks.append({"detections": [detection], "last_bbox": detection.bbox})

        for index, track in enumerate(active_tracks, start=1):
            confidence = sum(item.confidence for item in track["detections"]) / len(track["detections"])
            tracklets.append(
                Tracklet(
                    tracklet_id=f"trk_{label.replace(' ', '_')}_{index:03d}",
                    label=label,
                    detection_ids=[item.detection_id for item in track["detections"]],
                    keyframe_ids=[item.keyframe_id for item in track["detections"]],
                    confidence=confidence,
                )
            )
    return tracklets
