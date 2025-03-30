import cv2
import numpy as np

# Load YOLO model
yolo_net = cv2.dnn.readNet("yolov3-tiny.weights", "yolov3-tiny.cfg")
layer_names = yolo_net.getLayerNames()
output_layers = [layer_names[i - 1] for i in yolo_net.getUnconnectedOutLayers()]

# Load class labels (COCO dataset)
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Open webcam (change index if needed)
cap = cv2.VideoCapture(0)  # 0 = default webcam

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    height, width, channels = frame.shape
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
    yolo_net.setInput(blob)
    detections = yolo_net.forward(output_layers)

    for detection in detections:
        for obj in detection:
            scores = obj[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]

            if confidence > 0.5:
                obj_x, obj_y, obj_w, obj_h = (obj[0] * width, obj[1] * height, obj[2] * width, obj[3] * height)
                x_center, y_center = int(obj_x), int(obj_y)

                # Draw bounding box
                x, y, w, h = int(obj_x - obj_w / 2), int(obj_y - obj_h / 2), int(obj_w), int(obj_h)
                color = (0, 255, 0)  # Green box for detected objects
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                cv2.putText(frame, f"{classes[class_id]} ({confidence:.2f})", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # Check if the object is in the center (danger zone)
                if width // 3 < x_center < 2 * width // 3:
                    print("Obstacle detected in front! (Stop)")
                elif x_center < width // 3:
                    print("Obstacle on left! (Move right)")
                elif x_center > 2 * width // 3:
                    print("Obstacle on right! (Move left)")

    # Display the output
    cv2.imshow("Obstacle Detection", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
