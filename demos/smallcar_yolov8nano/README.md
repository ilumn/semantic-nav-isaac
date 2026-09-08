# YOLOv8nano Demo (Raspberry Pi 4)

YOLOv8n inference with Picamera2; annotated frames are served over MJPEG HTTP (port **8080**).

**Paths:** virtualenv `demos/venv` · code `demos/smallcar_yolov8nano` · target **aarch64** (Pi 4).

---

## Quick Start

From the **repository root**:

```bash
source demos/venv/bin/activate
pip install ultralytics numpy==1.26.4 opencv-python==4.8.1.78
cd demos/smallcar_yolov8nano
python3 yolov8_mjpeg_server.py
```

---

## Environment Setup (one-time)

Create the venv once under `demos/` (if it does not exist):

```bash
cd demos
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
pip install ultralytics numpy==1.26.4 opencv-python==4.8.1.78
```

`ultralytics` may be installed at latest; pin `numpy` and `opencv-python` as above for reproducibility on Raspberry Pi OS (aarch64).

---

## Smoke Test

```bash
source demos/venv/bin/activate
cd demos/smallcar_yolov8nano
python3 yolov8_smoketest_no_gui.py
```

**Expected:** lines like `frame 0: detections=N`, `frame 1: detections=…` (N ≥ 0).

---

## Run MJPEG Server

```bash
source demos/venv/bin/activate
cd demos/smallcar_yolov8nano
python3 yolov8_mjpeg_server.py
```

---

## Access Stream

On the Pi:

```bash
hostname -I
```

On another machine on the **same LAN**, open in a browser:

```text
http://<pi-ip>:8080/
```

---

## Notes

- Pi and viewer must be on the **same network**.
- **Camera** must be connected and enabled (Picamera2 / libcamera).

---

## Success Criteria

- Smoke test prints `frame X: detections=…` without errors.
- Browser shows the live stream with bounding boxes and labels.

---

*Extended setup and pitfalls: [setup.md](./setup.md)*
