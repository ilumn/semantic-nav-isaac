# tb3_detector/models

Place YOLOv8 weight files here.

## Required for the Isaac runtime

| Filename       | Download source |
|----------------|-----------------|
| `yolov8n.pt`   | Ultralytics v8.2.0 release asset |

From the repository root, download and verify the validated weight with:

```bash
dev/isaac_sim/fetch_models.sh --download
```

The expected digest is:

```text
SHA-256  f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36
```

## Naming convention

- `yolov8n.pt`           — official nano weights (COCO-80)
- `yolov8s.pt`           — official small weights (COCO-80)
- `yolov8n_tb3_lab.pt`   — custom fine-tuned on your lab scene (future)

## .gitignore

All weight files are excluded from version control because they are large
binaries. Use the repository downloader, Git LFS, or a model registry for
additional shared weights.
