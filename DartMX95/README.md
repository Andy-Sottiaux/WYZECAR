# WYZECAR - Vision-Based Human Following System

**DART-MX95 + ESP32 + ROS2 Humble + YOLOv8**

A robot car that follows humans using computer vision with smooth, professional motion control.

## Quick Start

### 1. Flash ESP32 Firmware (from development machine)
```bash
cd WYZECAR/DartMX95
pio run -t upload
pio device monitor -b 115200
```

### 2. On DART-MX95: Start the System
```bash
cd /root/WYZECAR
./DartMX95/scripts/start_wyzecar_host.sh
```

### 3. View in Browser
Open `http://<DART-IP>:8080` to see the camera feed with YOLO detection overlay.

---

## Hardware Configuration

### I2C Wiring (DART-MX95 J6 Header → ESP32)

| DART J6 Pin | Signal    | ESP32 Pin | Function    |
|-------------|-----------|-----------|-------------|
| Pin 18      | I2C3_SCL  | GPIO22    | I2C Clock   |
| Pin 20      | I2C3_SDA  | GPIO21    | I2C Data    |
| Pin 12      | GND       | GND       | Ground      |

- **I2C Bus**: `/dev/i2c-3`
- **ESP32 Address**: `0x42`

### Motor Wiring (ESP32 → L298N)

| ESP32 Pin | L298N Pin | Function         |
|-----------|-----------|------------------|
| GPIO25    | ENA       | Rear Motor PWM   |
| GPIO26    | IN1       | Rear Motor Dir A |
| GPIO27    | IN2       | Rear Motor Dir B |
| GPIO12    | IN3       | Front Motor Dir A|
| GPIO13    | IN4       | Front Motor Dir B|
| GPIO14    | ENB       | Front Motor PWM  |
| GPIO33    | -         | Steering Servo   |

### Camera
- **Model**: See3CAM_CU30 (or compatible USB camera)
- **Connection**: USB-C to DART-MX95 J33
- **Device**: `/dev/video13`

---

## System Architecture

```
Camera → DART-MX95 (ROS2) → ESP32 → Motors
         │
         ├─ Camera Node (v4l2_camera)
         │     ↓ /image_raw
         ├─ Human Detector (YOLOv8)
         │     ↓ /target_person
         ├─ Follower (PID control)
         │     ↓ /cmd_vel
         └─ Motor Controller (I2C)
               ↓ I2C to ESP32
```

## ROS2 Nodes

| Node | Package | Function |
|------|---------|----------|
| `v4l2_camera_node` | v4l2_camera | USB camera driver |
| `human_detector` | wyzecar_vision | YOLOv8 person detection |
| `follower` | wyzecar_vision | PID following controller |
| `motor_controller` | wyzecar_control | Smooth I2C motor control |
| `web_viewer.py` | - | MJPEG web server on port 8080 |

## Key Features

### Smooth Motor Control
- Acceleration ramping (25%/s)
- Deceleration ramping (35%/s)
- Servo slew rate limiting (60°/s)
- 25 Hz control loop

### YOLO Detection
- YOLOv8-nano model
- 416px input resolution
- ~5 FPS on ARM CPU
- Automatic model download

### Safety Systems
- Command timeout (2 seconds)
- Emergency stop command
- Watchdog timers on ESP32

---

## Docker Container

### Build (one-time)
```bash
cd /root/WYZECAR
docker build -t wyzecar:humble -f DartMX95/docker/Dockerfile DartMX95/docker
```

### Run Manually
```bash
docker run --rm -it --name wyzecar \
  --network=host \
  --device=/dev/i2c-3 \
  --device=/dev/video13 \
  --privileged \
  -v $PWD:/workspace \
  -w /workspace \
  wyzecar:humble bash
```

### Inside Container
```bash
source /opt/ros/humble/setup.bash
source /workspace/wyzecar_ws/install/setup.bash
bash /workspace/DartMX95/scripts/start_wyzecar.sh all
```

---

## I2C Commands (ESP32)

| Command | Bytes | Description |
|---------|-------|-------------|
| Set Motors | `0x01 rear front servo` | Set speeds (-100 to 100) and servo (0-180) |
| E-Stop | `0x02` | Emergency stop |
| Status | `0x03` | Request status |

### Test from DART-MX95
```bash
# Detect ESP32
i2cdetect -y 3

# Drive forward 50%, servo center
i2cset -y 3 0x42 0x01 50 50 90 i

# Emergency stop
i2cset -y 3 0x42 0x02
```

---

## Troubleshooting

### ESP32 not detected
- Check wiring: SDA→Pin20, SCL→Pin18, GND→Pin12
- Verify ESP32 is powered and running
- Monitor ESP32 serial at 115200 baud

### Camera not detected
- Check USB-C connection to J33
- Run `ls /dev/video*` to find device
- Try `v4l2-ctl --list-devices`

### YOLO model download fails
- Check network connectivity in container
- Pre-download: `python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"`

### Motors don't respond
- Test with direct i2cset commands
- Check ESP32 serial monitor for received commands
- Verify L298N wiring and power

---

## File Structure

```
DartMX95/
├── docker/
│   ├── Dockerfile          # ROS2 Humble + YOLO container
│   └── ros_entrypoint.sh   # Container entry point
├── docs/
│   ├── architecture.md     # System design
│   ├── i2c_protocol.md     # I2C command reference
│   ├── pinout.md           # DART-MX95 pinout
│   └── wiring.md           # Complete wiring guide
├── ros2/
│   ├── human_detector.py   # YOLOv8 detection node
│   ├── follower.py         # PID following controller
│   ├── motor_controller.py # Smooth motor control
│   └── web_viewer.py       # MJPEG web server
├── scripts/
│   ├── start_wyzecar.sh       # In-container startup
│   ├── start_wyzecar_host.sh  # Host launcher
│   └── wyzecar.service        # Systemd service
├── src/
│   └── main.cpp            # ESP32 motor firmware
└── platformio.ini          # ESP32 build config
```

---

## Development

### Rebuild ROS2 Workspace
```bash
cd /workspace/wyzecar_ws
colcon build
source install/setup.bash
```

### Update Python Files
```bash
cp /workspace/DartMX95/ros2/*.py /workspace/wyzecar_ws/src/wyzecar_vision/wyzecar_vision/
cp /workspace/DartMX95/ros2/motor_controller.py /workspace/wyzecar_ws/src/wyzecar_control/wyzecar_control/
colcon build
```

### Monitor Topics
```bash
ros2 topic list
ros2 topic hz /image_raw
ros2 topic echo /target_person
ros2 topic echo /cmd_vel
```
