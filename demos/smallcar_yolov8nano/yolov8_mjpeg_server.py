#!/usr/bin/env python3
import io
import time
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2
import numpy as np
from picamera2 import Picamera2
from ultralytics import YOLO


# -----------------------------
# Config (start with 640x480)
# -----------------------------
HOST = "0.0.0.0"
PORT = 8080

FRAME_W, FRAME_H = 640, 480          # change to (320, 240) if needed
MODEL_NAME = "yolov8n.pt"
IMG_SIZE = 320                       # model inference size (try 416 if too slow) 640
CONF_THRES = 0.4

JPEG_QUALITY = 80                    # lower = smaller/faster
TARGET_FPS = 10                      # soft cap, set 0 for unlimited


# Shared frame buffer (MJPEG)
_latest_jpeg = None
_lock = threading.Lock()


def bgr3(frame: np.ndarray) -> np.ndarray:
    """Ensure frame is 3-channel BGR."""
    if frame is None:
        return frame
    if len(frame.shape) == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    if frame.shape[2] == 4:
        # Picamera2 often returns BGRA
        return cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    return frame


def producer_loop():
    global _latest_jpeg

    # Camera
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"size": (FRAME_W, FRAME_H)})
    picam2.configure(config)
    picam2.start()
    time.sleep(0.5)

    # Model
    model = YOLO(MODEL_NAME)  # auto-download if missing

    last_t = time.time()

    try:
        while True:
            # FPS cap (optional)
            if TARGET_FPS > 0:
                now = time.time()
                dt = now - last_t
                min_dt = 1.0 / TARGET_FPS
                if dt < min_dt:
                    time.sleep(min_dt - dt)
                last_t = time.time()

            frame = picam2.capture_array("main")
            frame = bgr3(frame)

            # YOLOv8 inference
            results = model.predict(
                source=frame,
                imgsz=IMG_SIZE,
                conf=CONF_THRES,
                verbose=False
            )

            annotated = results[0].plot()  # draws boxes+labels (BGR)

            # Encode to JPEG
            ok, jpg = cv2.imencode(
                ".jpg",
                annotated,
                [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
            )
            if not ok:
                continue

            with _lock:
                _latest_jpeg = jpg.tobytes()

    finally:
        picam2.stop()


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            html = f"""
            <html>
              <head><title>YOLOv8 MJPEG Preview</title></head>
              <body style="background:#111;color:#eee;font-family:Arial;">
                <h2>YOLOv8n MJPEG Preview</h2>
                <p>Resolution: {FRAME_W}x{FRAME_H} | imgsz={IMG_SIZE} | conf={CONF_THRES}</p>
                <img src="/stream.mjpg" style="max-width:100%;border:1px solid #444;" />
              </body>
            </html>
            """
            data = html.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
            return

        if self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            while True:
                with _lock:
                    frame = _latest_jpeg

                if frame is None:
                    time.sleep(0.05)
                    continue

                try:
                    self.wfile.write(b"--frame\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    time.sleep(0.001)
                except BrokenPipeError:
                    break
                except ConnectionResetError:
                    break
            return

        self.send_response(404)
        self.end_headers()

    def log_message(self, format, *args):
        # silence default logging
        return


def main():
    t = threading.Thread(target=producer_loop, daemon=True)
    t.start()

    server = HTTPServer((HOST, PORT), Handler)
    print(f"[INFO] Server running: http://{HOST}:{PORT}/")
    print("[INFO] Open from your laptop: http://<pi-ip>:8080/")
    print("[INFO] Ctrl+C to stop.")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
