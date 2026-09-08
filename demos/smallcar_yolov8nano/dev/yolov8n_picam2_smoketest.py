from ultralytics import YOLO
from picamera2 import Picamera2
import cv2, time

MODEL = "yolov8n.pt"  # auto-downloads on first run

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()
time.sleep(0.5)

model = YOLO(MODEL)

try:
    while True:
        frame = picam2.capture_array("main")  # BGR-ish frame
        results = model.predict(source=frame, imgsz=640, conf=0.4, verbose=False)
        annotated = results[0].plot()  # draws boxes + labels
        cv2.imshow("YOLOv8n Smoke Test (press q)", annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    picam2.stop()
    cv2.destroyAllWindows()
