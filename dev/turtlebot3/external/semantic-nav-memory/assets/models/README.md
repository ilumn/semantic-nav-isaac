The semantic refiner requires these checkpoints, downloaded explicitly from the
repository root with `dev/isaac_sim/fetch_models.sh --download`:

- `yolov8s-worldv2.pt`
- SHA-256: `9b2c17ab6124a913e9b3a5c170617920d91b0f01111a8479da69f00e2cf27792`
- `ViT-B-32.pt` (CLIP text encoder required by YOLO-World)
- SHA-256: `40d365715913c9da98579312b702a82c18be219cc2a73407c4526f58eba950af`

Place additional local detector checkpoints here when you do not want
Ultralytics to download them on first run.

Typical filenames:

- `yolov8s-worldv2.pt`
- `yolov8m-worldv2.pt`
- `yolov8l-worldv2.pt`
- `yolov8x-worldv2.pt`

All checkpoint binaries are ignored by Git. Their expected SHA-256 digests are
listed above and enforced by the downloader and Isaac preflight.
