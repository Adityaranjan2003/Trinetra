import os
import cv2
import numpy as np
import time
from picamera2 import Picamera2

# Absolute paths to YOLO files
CFG_PATH = "/home/a4/Desktop/drone/yolov4-tiny.cfg"
WEIGHTS_PATH = "/home/a4/Desktop/drone/yolov4-tiny.weights"
NAMES_PATH = "/home/a4/Desktop/drone/coco.names"

# Check if YOLO files exist
if not all(os.path.exists(f) for f in [CFG_PATH, WEIGHTS_PATH, NAMES_PATH]):
    print("ERROR: Missing YOLO files! Please check the paths.")
    exit(1)

# Load class names
with open(NAMES_PATH, "r") as f:
    class_names = f.read().strip().split("\n")

# Load YOLO Model
net = cv2.dnn.readNet(WEIGHTS_PATH, CFG_PATH)

# Set up the Raspberry Pi Camera
camera = Picamera2()
camera_config = camera.create_preview_configuration(main={"size": (640, 480)})
camera.configure(camera_config)
camera.start()
time.sleep(2)  # Allow the camera to initialize

def detect_objects(frame):
    height, width = frame.shape[:2]
    blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)
    
    layer_names = net.getUnconnectedOutLayersNames()
    outputs = net.forward(layer_names)
    
    boxes, confidences, class_ids = [], [], []

    for output in outputs:
        for detection in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.5:  # Detection confidence threshold
                center_x, center_y, w, h = (detection[:4] * np.array([width, height, width, height])).astype("int")

                x = int(center_x - (w / 2))
                y = int(center_y - (h / 2))

                boxes.append([x, y, int(w), int(h)])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    return boxes, confidences, class_ids

while True:
    frame = camera.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    boxes, confidences, class_ids = detect_objects(frame)

    for i in range(len(boxes)):
        x, y, w, h = boxes[i]
        label = f"{class_names[class_ids[i]]}: {confidences[i]:.2f}"
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("YOLO Object Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

camera.stop()
cv2.destroyAllWindows()
