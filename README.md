# Trinetra

A comprehensive integration of computer vision and flight controller telemetry for drone applications. This system combines real-time object detection with ArduPilot data for autonomous surveillance, monitoring, and data collection.

## Overview

This project provides an integrated solution for:
- Establishing connection with ArduPilot flight controllers
- Processing live camera feed for object detection using YOLOv4
- Overlaying flight telemetry on video feed
- Automatically logging detected objects with corresponding flight data
- Saving images when objects are detected

## Hardware Requirements

- Raspberry Pi 3B (64-bit desktop environment)
- PiCamera module (AWM 20624 80C 60V VW-1)
- ArduPilot compatible flight controller
- USB-A to micro USB cable (for flight controller connection)
- 16GB+ SD card

## Software Requirements

- Python 3.6+
- OpenCV
- NumPy
- PyMAVLink
- Picamera2
- YOLOv4-tiny model files

## Installation

1. Clone this repository:
```
git clone https://github.com/yourusername/drone-vision-telemetry.git
cd drone-vision-telemetry
```

2. Install required Python packages:
```
pip install opencv-python numpy pymavlink picamera2
```

3. Download YOLOv4-tiny model files and place them in the correct location:
```
mkdir -p /home/a4/Desktop/drone/
# Download the following files and place them in the directory above:
# - yolov4-tiny.cfg
# - yolov4-tiny.weights
# - coco.names
```

## Usage

### Basic Usage

Run the integrated system:
```
python final/drone_integrated.py
```

This will:
1. Connect to ArduPilot via USB
2. Initialize the camera
3. Start object detection
4. Display live video with telemetry overlay
5. Save detected objects and images

Press 'q' to quit the application or use Ctrl+C in the terminal.

### Individual Components

The system consists of three main components that can be run separately:

1. **Camera Only**: Simple image capture
```
python final/drone_cam.py
```

2. **Object Detection**: Camera feed with YOLO object detection
```
python final/obj-detect.py
```

3. **ArduPilot Connection**: Just connect and monitor the drone's telemetry
```
python final/ardupilot_connect.py
```
or
```
python final/ardupilot_connect_improved.py
```

## File Structure

- `final/drone_integrated.py` - Main integrated system
- `final/drone_cam.py` - Basic camera capture script
- `final/obj-detect.py` - Object detection with YOLO
- `final/ardupilot_connect.py` - Basic ArduPilot connection
- `final/ardupilot_connect_improved.py` - Improved ArduPilot monitoring
- `drone_data/` - Directory where captured data is stored
  - `images/` - Saved images from detections
  - `logs/` - JSON logs of detection data with telemetry

## Features

- **Multithreaded Design**: Separate threads for video processing and telemetry
- **Real-time Object Detection**: Using YOLOv4-tiny model
- **Flight Data Overlay**: Shows mode, armed status, altitude, heading, and attitude
- **Automatic Data Logging**: JSON formatted logs with timestamp and position data
- **Detection Image Capture**: Automatically saves images when objects are detected
- **Clean Exit Handling**: Proper resource cleanup on exit

## Data Output

All detection data is saved in JSON format with the following structure:
```json
{
  "class": "person",
  "confidence": 0.92,
  "box": [100, 150, 50, 120],
  "center_x": 125,
  "center_y": 210,
  "timestamp": "20230401-152233",
  "vehicle_state": {
    "mode": "STABILIZE",
    "armed": true,
    "latitude": 47.623,
    "longitude": -122.332,
    "altitude": 10.5,
    ...
  }
}
```

## Troubleshooting

- **Connection Issues**: Ensure the correct USB port is specified in the code (`/dev/ttyACM0` by default)
- **Camera Not Working**: Check if camera module is enabled in Raspberry Pi configuration
- **YOLO Model Missing**: Verify the paths to YOLO files in the code match your system
