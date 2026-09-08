# Picamera2 YOLOv5 TFLite Patch Documentation

## Demo Video

📹 [Watch Demo Video](./video/2026.3.2-YOLOv5%20TFLite%20in%20small%20car-demo.mp4)

## Section 1 — Overview

This patch enables successful execution of YOLOv5 TensorFlow Lite object detection with label overlays on Raspberry Pi 4 Model B (4GB) running Raspberry Pi OS Bullseye using Picamera2. The upstream example (`picamera2/examples/tensorflow/yolo_v5_real_time_with_labels.py`) contains several compatibility issues when running on Bullseye with VNC and specific hardware configurations.

**What this patch enables:**
- Real-time YOLOv5 object detection using TensorFlow Lite models
- Label overlays on camera feed without Qt dependencies
- Stable operation under VNC remote desktop sessions
- Compatibility with Raspberry Pi OS Bullseye and Picamera2 0.3.12-2

**Why this patch exists:**
The upstream example assumes newer Picamera2 API features (e.g., `Platform` class) and Qt-based preview systems that are incompatible with Bullseye and VNC environments. This patch provides a stable, reproducible solution for resource-constrained Raspberry Pi 4 setups.

## Section 2 — Supported Environment Matrix

| Component | Version/Model | Notes |
|-----------|---------------|-------|
| Hardware | Raspberry Pi 4 Model B (4GB) | 4GB RAM minimum |
| OS | Raspberry Pi OS Bullseye | 32-bit or 64-bit |
| Picamera2 | 0.3.12-2 | Debian package: `python3-picamera2` |
| Python | 3.9+ | Default Bullseye Python 3 |
| Camera Module | OV5647 / IMX219 | Via libcamera |
| TensorFlow Lite | 2.13+ | `tensorflow-lite-runtime` |
| OpenCV | 4.5+ | Debian package: `python3-opencv` |
| Display | VNC (optional) | TightVNC or RealVNC |
| Model | yolov5s-fp16.tflite | Quantized FP16 model |

**Tested Configuration:**
- Raspberry Pi 4 Model B (4GB)
- Raspberry Pi OS Bullseye (32-bit)
- python3-picamera2 0.3.12-2
- python3-opencv 4.5.1
- tensorflow-lite-runtime 2.13.0
- TightVNC Server

## Section 3 — Full Reproduction Pipeline

### Step 1: Install Dependencies

```bash
# Update package list
sudo apt update

# Install system dependencies
sudo apt install -y python3-picamera2 python3-opencv python3-numpy python3-pil

# Install TensorFlow Lite runtime
pip3 install tensorflow-lite-runtime

# Verify installations
python3 -c "import picamera2; import cv2; import numpy; print('Dependencies OK')"
```

**Important:** Do not install `opencv-python` via pip. Use only the Debian package `python3-opencv` to avoid Qt backend conflicts.

### Step 2: Clone Picamera2 Repository

```bash
# Clone upstream repository
git clone https://github.com/raspberrypi/picamera2.git
cd picamera2

# Checkout tested version (adjust commit hash as needed)
git checkout <commit-hash-from-upstream-picamera2.version>
```

**Note:** The exact commit hash should match the version specified in `upstream/picamera2.version`.

### Step 3: Download Model and Labels

```bash
# Create model directory
mkdir -p models

# Download YOLOv5 TFLite model (example URL - adjust as needed)
cd models
wget https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s-fp16.tflite
# Or use your own yolov5s-fp16.tflite model

# Download COCO class labels (80 classes)
cat > coco_labels.txt << 'EOF'
person
bicycle
car
motorcycle
airplane
bus
train
truck
boat
traffic light
fire hydrant
stop sign
parking meter
bench
bird
cat
dog
horse
sheep
cow
elephant
bear
zebra
giraffe
backpack
umbrella
handbag
tie
suitcase
frisbee
skis
snowboard
sports ball
kite
baseball bat
baseball glove
skateboard
surfboard
tennis racket
bottle
wine glass
cup
fork
knife
spoon
bowl
banana
apple
sandwich
orange
broccoli
carrot
hot dog
pizza
donut
cake
chair
couch
potted plant
bed
dining table
toilet
tv
laptop
mouse
remote
keyboard
cell phone
microwave
oven
toaster
sink
refrigerator
book
clock
vase
scissors
teddy bear
hair drier
toothbrush
EOF
cd ..
```

### Step 4: Apply Patch

**Option A: Apply Git Patch**

```bash
# Apply patch file
git apply /path/to/quadruped-semantic-navigation/patches/picamera2/0001-yolo-v5-real-time-with-labels.patch

# Verify patch applied
git status
```

**Option B: Manual Copy**

```bash
# Copy patched script
cp /path/to/quadruped-semantic-navigation/patches/picamera2/yolo_v5_real_time_with_labels.py \
   examples/tensorflow/yolo_v5_real_time_with_labels.py
```

### Step 5: Run Demo

```bash
# Navigate to examples directory
cd examples/tensorflow

# Run with default settings
python3 yolo_v5_real_time_with_labels.py

# Or specify model path
python3 yolo_v5_real_time_with_labels.py --model ../models/yolov5s-fp16.tflite \
                                         --labels ../models/coco_labels.txt \
                                         --width 640 --height 640
```

**Expected Output:**
- Camera preview window with bounding boxes and labels
- FPS counter in terminal
- Press 'q' to quit

## Section 4 — Patch Details

### Patch A: Picamera2 API Mismatch — Platform Import

**Symptom:**
```
ImportError: cannot import name 'Platform' from 'picamera2'
```

**Root Cause:**
The upstream script imports `Platform` from `picamera2`, which is available in newer Picamera2 versions (Bookworm+) but not in Bullseye's `python3-picamera2 0.3.12-2`. The script also contains Pi5-specific logic that is unnecessary for Pi4.

**Fix:**
```python
# REMOVED:
from picamera2 import Picamera2, Platform

# REPLACED WITH:
from picamera2 import Picamera2

# REMOVED Pi5 detection logic:
# if Platform.is_pi5():
#     ...
```

**Impact:** Enables script to run on Bullseye without Platform API dependency.

---

### Patch B: Qt Preview and Post-Callback Overlay Removal

**Symptom:**
```
qt.qpa.plugin: Could not load the Qt platform plugin "xcb"
Aborted (core dumped)
```

**Root Cause:**
VNC sessions do not provide a proper X11 display environment for Qt applications. The upstream script uses `start_preview()` which requires Qt backend, and `post_callback` overlay pipeline that conflicts with VNC's X11 forwarding.

**Fix:**
```python
# REMOVED:
picam2.start_preview()
picam2.post_callback = draw_detections

# REPLACED WITH:
# OpenCV-based frame capture and display loop
while True:
    frame = picam2.capture_array()
    frame_with_detections = draw_detections_opencv(frame, detections)
    cv2.imshow('YOLOv5 Detection', frame_with_detections)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
```

**Impact:** Eliminates Qt dependency, enables stable VNC operation, reduces memory footprint.

---

### Patch C: OpenCV Stability — Debian Package Enforcement

**Symptom:**
```
ImportError: libQt5Core.so.5: cannot open shared object file
# Or
cv2.imshow() crashes or shows blank window
```

**Root Cause:**
pip-installed `opencv-python` includes Qt backend dependencies that conflict with VNC's X11 environment. The pip package also has different library paths than system-installed OpenCV.

**Fix:**
```bash
# Uninstall pip OpenCV
pip3 uninstall opencv-python opencv-contrib-python

# Install Debian package
sudo apt install python3-opencv

# Verify cv2 path
python3 -c "import cv2; print(cv2.__file__)"
# Should show: /usr/lib/python3/dist-packages/cv2/...
```

**Code Verification:**
```python
# Add at script start:
import cv2
assert '/usr/lib' in cv2.__file__ or '/usr/local/lib' in cv2.__file__, \
    "Use system OpenCV, not pip package"
```

**Impact:** Ensures OpenCV uses system libraries compatible with VNC, prevents Qt backend conflicts.

---

### Patch D: Stream Dimension Constraint

**Symptom:**
```
RuntimeError: lores stream dimensions may not exceed main stream dimensions
```

**Root Cause:**
Picamera2 requires that the low-resolution (lores) stream dimensions must be less than or equal to the main stream dimensions. The upstream script may configure lores stream larger than main stream.

**Fix:**
```python
# Ensure main >= lores
main_size = (640, 640)
lores_size = (640, 640)  # Must be <= main_size

config = picam2.create_preview_configuration(
    main={"size": main_size},
    lores={"size": lores_size}
)
picam2.configure(config)
```

**Impact:** Prevents stream dimension errors, ensures proper camera configuration.

---

### Patch E: YUV Conversion Compatibility

**Symptom:**
```
ValueError: Invalid YUV format
# Or incorrect color rendering in converted frames
```

**Root Cause:**
Different YUV420 formats (I420 vs NV12) have different memory layouts. The upstream conversion may assume a specific format that doesn't match the camera output on certain configurations.

**Fix:**
```python
# Implement robust YUV to RGB conversion with fallback
def yuv_to_rgb_robust(yuv_frame, width, height):
    try:
        # Try NV12 format first (common on Pi)
        rgb = cv2.cvtColor(yuv_frame, cv2.COLOR_YUV2RGB_NV12)
    except:
        try:
            # Fallback to I420
            rgb = cv2.cvtColor(yuv_frame, cv2.COLOR_YUV2RGB_I420)
        except:
            # Final fallback: manual conversion
            y = yuv_frame[:height, :]
            u = yuv_frame[height:height+height//2, :width//2]
            v = yuv_frame[height+height//2:, :width//2]
            # ... manual YUV to RGB conversion
            rgb = manual_yuv_to_rgb(y, u, v, width, height)
    return rgb
```

**Impact:** Handles different YUV formats gracefully, improves compatibility across camera modules.

## Section 5 — Troubleshooting

### Issue: `cannot import Platform`

**Solution:**
```bash
# Verify Picamera2 version
python3 -c "import picamera2; print(picamera2.__version__)"

# If version < 0.3.13, Platform is not available
# Apply Patch A to remove Platform dependency
```

---

### Issue: Qt xcb Plugin Crash

**Symptoms:**
- `qt.qpa.plugin: Could not load the Qt platform plugin "xcb"`
- Script crashes immediately on `start_preview()`

**Solution:**
1. Verify Qt is not being used:
   ```bash
   pip3 list | grep -i qt
   # Should show no Qt packages
   ```

2. Apply Patch B to remove Qt dependencies

3. If using VNC, ensure DISPLAY is set:
   ```bash
   echo $DISPLAY
   # Should show :0 or :1
   export DISPLAY=:0
   ```

---

### Issue: Lores Stream Dimensions Exceed Main Stream

**Error Message:**
```
RuntimeError: lores stream dimensions may not exceed main stream dimensions
```

**Solution:**
```python
# Check current configuration
print(f"Main: {main_size}, Lores: {lores_size}")

# Ensure lores <= main
if lores_size[0] > main_size[0] or lores_size[1] > main_size[1]:
    lores_size = main_size  # Set lores equal to main
```

Apply Patch D to enforce dimension constraints.

---

### Issue: cv2 Path Incorrect

**Symptoms:**
- OpenCV imports but `cv2.imshow()` doesn't work
- Wrong OpenCV version detected

**Solution:**
```bash
# Check which OpenCV is imported
python3 -c "import cv2; print(cv2.__file__)"

# If path contains 'site-packages', uninstall pip version
pip3 uninstall opencv-python opencv-contrib-python

# Install system package
sudo apt install python3-opencv

# Verify
python3 -c "import cv2; print(cv2.__file__); print(cv2.__version__)"
# Should show /usr/lib/... and version 4.5+
```

---

### Issue: VNC Latency

**Note:**
VNC introduces network latency. For best performance:
- Use wired Ethernet connection
- Reduce VNC color depth: `vncserver -depth 16`
- Consider using SSH X11 forwarding instead: `ssh -X pi@raspberrypi`
- Or run directly on Pi with HDMI display

---

### Issue: Camera Not Detected

**Solution:**
```bash
# List available cameras
libcamera-hello --list-cameras

# Test camera access
libcamera-hello -t 0

# Enable camera interface if needed
sudo raspi-config
# Interface Options > Camera > Enable
```

---

### Issue: Low FPS Performance

**Optimization Tips:**
1. Use quantized models (FP16 or INT8)
2. Reduce input resolution (e.g., 416x416 instead of 640x640)
3. Close unnecessary background processes
4. Ensure adequate power supply (official Pi PSU recommended)
5. Check CPU temperature: `vcgencmd measure_temp`

## Section 6 — Design Rationale

### Why We Do Not Fork Picamera2 Repository

**Reason 1: Upstream Maintenance**
Forking creates a maintenance burden. When upstream Picamera2 releases updates (bug fixes, performance improvements, new features), a fork requires manual merging and conflict resolution. Patches allow us to track upstream changes and reapply our modifications cleanly.

**Reason 2: License and Attribution**
Picamera2 is maintained by Raspberry Pi Foundation. Keeping patches separate maintains clear attribution and respects upstream development. Our patches are clearly marked as modifications, not replacements.

**Reason 3: Repository Size**
Including full Picamera2 source code would bloat this repository unnecessarily. Patches are small, version-controlled, and can be applied to any compatible upstream version.

### Why We Keep Patches Separate

**Reason 1: Reproducibility**
Patches are version-controlled and can be applied deterministically. Anyone can:
1. Clone upstream at a specific commit
2. Apply our patches
3. Reproduce the exact environment

This is superior to maintaining modified source files that may drift from upstream.

**Reason 2: Transparency**
Separate patches make it clear what we changed and why. Each patch can be reviewed independently, and the diff shows exactly what modifications were made.

**Reason 3: Flexibility**
If upstream fixes our issues in future versions, we can simply remove patches. If we need to update to a new upstream version, we test patches against it and update only what's necessary.

### How This Improves Reproducibility

**Version Tracking:**
- `upstream/picamera2.version` records the exact upstream commit tested
- Patches are designed to apply cleanly to that version
- Future versions can be tested by updating the version file

**Automated Application:**
- Scripts (`apply_patches_picamera2.sh`) automate patch application
- Reduces human error in manual copying
- Ensures consistent application across environments

**Documentation:**
- This document explains every patch in detail
- Troubleshooting section addresses common issues
- Others can reproduce without debugging from scratch

**Testing:**
- Patches are tested against specific hardware/OS combinations
- Environment matrix documents what works
- Reduces "works on my machine" issues

---

## Summary

This patch set transforms the upstream Picamera2 YOLOv5 example into a stable, VNC-compatible solution for Raspberry Pi 4 Model B running Bullseye. By addressing API mismatches, Qt dependencies, OpenCV conflicts, and stream configuration issues, we enable reproducible object detection workflows without maintaining a fork of the upstream repository.

**Key Achievements:**
- ✅ Bullseye compatibility without Platform API
- ✅ VNC-friendly operation without Qt
- ✅ Stable OpenCV integration via system packages
- ✅ Proper stream dimension handling
- ✅ Robust YUV format conversion

**Maintenance:**
When upstream Picamera2 updates, test patches against new version and update `upstream/picamera2.version` accordingly. If upstream incorporates our fixes, remove corresponding patches.
