from __future__ import annotations

import json
from collections import deque
from pathlib import Path

import cv2


class BundleWriter:
    """Persist sampled refinement frames and metadata into a rolling bundle."""

    def __init__(self, bundle_root: str, max_frames: int) -> None:
        self._root = Path(bundle_root)
        self._frames_dir = self._root / "latest" / "frames"
        self._manifest_path = self._root / "latest" / "manifest.json"
        self._frames_dir.mkdir(parents=True, exist_ok=True)
        self._written_paths: deque[Path] = deque(maxlen=max(1, int(max_frames)))
        self._manifest: dict = {"frames": []}
        self._save_manifest()

    def write_frame(self, frame_index: int, bgr_image, metadata: dict) -> str:
        filename = f"frame_{frame_index:06d}.jpg"
        path = self._frames_dir / filename
        cv2.imwrite(str(path), bgr_image)

        if len(self._written_paths) == self._written_paths.maxlen:
            stale = self._written_paths[0]
            stale.unlink(missing_ok=True)
            self._manifest["frames"] = [
                item for item in self._manifest["frames"] if item.get("image_path") != str(stale)
            ]

        self._written_paths.append(path)
        self._manifest["frames"].append(
            {
                **metadata,
                "image_path": str(path),
            }
        )
        self._save_manifest()
        return str(path)

    def _save_manifest(self) -> None:
        self._manifest_path.parent.mkdir(parents=True, exist_ok=True)
        self._manifest_path.write_text(json.dumps(self._manifest, indent=2), encoding="utf-8")
