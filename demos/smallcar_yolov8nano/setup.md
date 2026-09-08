# YOLOv8nano Setup Guide

This guide provides step-by-step instructions to set up YOLOv8nano object detection on Raspberry Pi 4 Model B for the small car platform.

## Tested Configuration

The following configuration has been tested and verified:

```bash
# Hardware
$ cat /proc/device-tree/model
Raspberry Pi 4 Model B Rev 1.1

# OS
$ cat /etc/os-release
PRETTY_NAME="Debian GNU/Linux 11 (bullseye)"
VERSION_ID="11"

# Architecture
$ uname -m
aarch64

# Picamera2
$ dpkg -l | grep picamera2
ii  python3-picamera2  0.3.12-2  arm64  libcamera-based Python 3 bindings

# OpenCV
$ python3 -c "import cv2; print(cv2.__version__)"
4.5.1

# TensorFlow Lite Runtime
$ pip3 show ai-edge-litert
Name: ai-edge-litert
Version: 2.1.0
```

**Key Components:**
- Hardware: Raspberry Pi 4 Model B (2GB)
- OS: Raspberry Pi OS Bullseye (64-bit, aarch64)
- Picamera2: 0.3.12-2 (APT package)
- OpenCV: 4.5.1 (APT package, NOT pip)
- TensorFlow Lite Runtime: 2.1.0 (pip)
- Display: TightVNC (MJPEG streaming preferred)

## Installation Steps

### Step 1: Install Required APT Packages

```bash
# Update package list
sudo apt update

# Install system dependencies
sudo apt install -y python3-picamera2 python3-opencv python3-numpy python3-pil

# Verify installations
python3 -c "import picamera2; import cv2; import numpy; print('System packages OK')"
```

**Critical:** Do not install `opencv-python` via pip. Use only the Debian package `python3-opencv` to avoid Qt backend conflicts.

### Step 2: Create Virtual Environment

```bash
# From repo root, navigate to demo directory
cd demos/smallcar_yolov8nano

# Create venv with system site packages (so picamera2 is accessible)
python3 -m venv venv --system-site-packages

# Activate venv
source venv/bin/activate
```

**Why `--system-site-packages`?**
Picamera2 is installed system-wide via APT. Using `--system-site-packages` allows the venv to import picamera2 while still isolating ultralytics dependencies.

### Step 3: Install Ultralytics with Version Pins

```bash
# Ensure pip is up to date
pip install --upgrade pip

# Install ultralytics with NumPy and SciPy version constraints
pip install ultralytics "numpy<2" "scipy<1.12"
```

**Why pin NumPy < 2?**
Ultralytics pip install can upgrade NumPy to 2.x, which breaks Bullseye-compiled modules (e.g., simplejpeg, picamera2) due to ABI incompatibility. Pinning NumPy < 2 prevents this issue.

### Step 4: Verify Imports

```bash
# Test critical imports
python3 -c "from picamera2 import Picamera2; print('Picamera2 OK')"
python3 -c "from ultralytics import YOLO; print('Ultralytics OK')"
python3 -c "import cv2; assert '/usr/lib' in cv2.__file__ or '/usr/local/lib' in cv2.__file__; print('OpenCV OK (system package)')"
```

If all commands succeed, the environment is ready.

## Troubleshooting

### Issue 1: NumPy ABI Mismatch

**Symptoms:**
```
ValueError: numpy.dtype size changed, may indicate binary incompatibility
RuntimeError: module compiled against NumPy 1.x cannot run in NumPy 2.x
```

**Root Cause:**
Ultralytics pip install upgraded NumPy to 2.x, breaking compiled extensions.

**Fix:**
```bash
# From repo root
cd demos/smallcar_yolov8nano
source venv/bin/activate

# Uninstall problematic packages
pip uninstall -y numpy scipy ultralytics

# Reinstall with constraints
pip install ultralytics "numpy<2" "scipy<1.12"

# Verify NumPy version
python3 -c "import numpy; print(numpy.__version__)"
# Should show 1.x, not 2.x
```

### Issue 2: Qt xcb Plugin Crash

**Symptoms:**
```
qt.qpa.plugin: Could not load the Qt platform plugin "xcb"
Aborted (core dumped)
```

**Root Cause:**
pip-installed OpenCV pulled in Qt backend dependencies that conflict with VNC.

**Fix:**
```bash
# Check which OpenCV is imported
python3 -c "import cv2; print(cv2.__file__)"

# If path contains 'site-packages', uninstall pip version
pip uninstall -y opencv-python opencv-contrib-python

# Ensure system package is installed
sudo apt install python3-opencv

# Verify system OpenCV
python3 -c "import cv2; print(cv2.__file__); print(cv2.__version__)"
# Should show /usr/lib/... and version 4.5+
```

### Issue 3: Picamera2 4-Channel Frame Issue

**Symptoms:**
```
ValueError: Input image must be 3-channel BGR
# Or incorrect color rendering
```

**Root Cause:**
Picamera2 may return 4-channel BGRA frames, but YOLO expects 3-channel BGR.

**Fix:**
The scripts already handle this conversion:
```python
# In both scripts, frames are converted:
if frame.shape[2] == 4:
    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
```

If you see this error, ensure you're using the provided scripts (not modified versions).

### Issue 4: VNC Display Latency

**Symptoms:**
- Seconds-level delay when displaying GUI windows
- High CPU usage when using `cv2.imshow()`

**Root Cause:**
VNC adds network latency for X11 forwarding.

**Solution:**
Use MJPEG streaming (`yolov8_mjpeg_server.py`) instead of GUI windows. The MJPEG server provides:
- Lower latency (direct HTTP streaming)
- Better performance (no X11 overhead)
- Remote access from any device with a browser

### Issue 5: Camera Not Detected

**Symptoms:**
```
RuntimeError: Failed to create camera component
```

**Fix:**
```bash
# List available cameras
libcamera-hello --list-cameras

# Test camera access
libcamera-hello -t 0

# Enable camera interface if needed
sudo raspi-config
# Navigate to: Interface Options > Camera > Enable
# Reboot after enabling
```

### Issue 6: Import Errors After Venv Creation

**Symptoms:**
```
ModuleNotFoundError: No module named 'picamera2'
```

**Fix:**
```bash
# From repo root
cd demos/smallcar_yolov8nano

# Ensure venv was created with --system-site-packages
deactivate  # if venv is active
rm -rf venv
python3 -m venv venv --system-site-packages
source venv/bin/activate

# Verify picamera2 is accessible
python3 -c "from picamera2 import Picamera2; print('OK')"
```

## Verification

After completing setup, run the smoke test:

```bash
# From repo root
cd demos/smallcar_yolov8nano
source venv/bin/activate

# Run smoke test (no GUI, prints detection counts)
python3 yolov8_smoketest_no_gui.py
```

Expected output:
```
frame 0: detections=2
frame 1: detections=1
...
```

If the smoke test runs without errors, setup is complete. Proceed to `README.md` for usage instructions.
