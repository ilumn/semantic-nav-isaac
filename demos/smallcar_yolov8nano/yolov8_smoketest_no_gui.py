from ultralytics import YOLO
from picamera2 import Picamera2
import time
import cv2

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()
time.sleep(0.5)

model = YOLO("yolov8n.pt")  # auto-download

try:
    for i in range(20):
        frame = picam2.capture_array("main")

        # Fix 4-channel frames (RGBA/BGRA) -> 3-channel BGR
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

        results = model.predict(source=frame, imgsz=640, conf=0.4, verbose=False)
        r0 = results[0]
        print(f"frame {i}: detections={len(r0.boxes)}")
finally:
    picam2.stop()
