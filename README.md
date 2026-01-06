# WYZECAR - Vision-Based Autonomous RC Car

An autonomous RC car platform that uses computer vision for human detection and following, built with ROS2, ESP32, and YOLOv8.

## Features

- **Autonomous Human Following**: Uses YOLOv8 for real-time person detection and PID control for smooth following
- **Remote Control Mode**: Web-based interface with WASD keyboard control
- **Real-time Video Streaming**: 15 FPS camera feed with detection overlays
- **Smooth Motion Control**: Advanced motor control with acceleration ramping and startup kick
- **Cross-Browser Support**: Works in Chrome, Safari, and Firefox
- **Robust Architecture**: Built on ROS2 Humble with Docker containerization

## Quick Start

1. **SSH into your DART-MX95**:
   ```bash
   ssh root@<dart-ip>
   sudo -i
   ```

2. **Clone the repository** (if not already done):
   ```bash
   git clone https://github.com/Andy-Sottiaux/WYZECAR.git /root/WYZECAR
   ```

3. **Start WYZECAR**:
   ```bash
   cd /root/WYZECAR
   ./wyzecar.sh
   ```

4. **Open the web interface**:
   - Navigate to `http://<dart-ip>:8080` in your browser
   - Use WASD keys to drive in remote control mode

## Usage

The main control script is `wyzecar.sh` which provides all functionality:

```bash
# Start in remote control mode (default)
./wyzecar.sh

# Start in autonomous follow mode
./wyzecar.sh follow

# Other commands
./wyzecar.sh stop      # Stop WYZECAR
./wyzecar.sh status    # Show system status
./wyzecar.sh logs      # View live logs
./wyzecar.sh update    # Pull latest from GitHub
```

## Hardware Setup

### Components
- **Platform**: Variscite DART-MX95 SoM (NXP i.MX 95)
  - 6x Cortex-A55 @ 2.0GHz
  - NPU for AI/ML acceleration
- **Camera**: USB camera (V4L2 compatible)
- **Motor Controller**: ESP32 with L298N driver
- **Motors**: 2x DC motors (rear drive + front steering)
- **Servo**: Standard RC servo for steering
- **Communication**: I2C bus 3, address 0x42

### Wiring

#### I2C Connection (J6 Header on DART-MX95)
| DART Pin | ESP32 Pin | Function |
|----------|-----------|----------|
| Pin 18 | GPIO22 | I2C3_SCL |
| Pin 20 | GPIO21 | I2C3_SDA |
| Pin 12 | GND | Ground |

#### Motor Driver (L298N)
| ESP32 Pin | L298N Pin | Function |
|-----------|-----------|----------|
| GPIO25 | ENA | Rear motor PWM |
| GPIO26 | IN1 | Rear motor Dir A |
| GPIO27 | IN2 | Rear motor Dir B |
| GPIO14 | ENB | Front motor PWM |
| GPIO12 | IN3 | Front motor Dir A |
| GPIO13 | IN4 | Front motor Dir B |
| GPIO33 | - | Servo PWM signal |

## System Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│   Web Browser   │────▶│  Web Viewer Node │◀────│  Camera Node    │
│   (WASD Keys)   │     │   (Port 8080)    │     │  (V4L2 @ 15fps) │
└─────────────────┘     └──────────────────┘     └─────────────────┘
                               │                           │
                               ▼                           ▼
                        ┌──────────────────┐     ┌─────────────────┐
                        │   Cmd_Vel Topic  │     │  Human Detector │
                        │   (/cmd_vel)     │     │   (YOLOv8)      │
                        └──────────────────┘     └─────────────────┘
                               │                           │
                               ▼                           ▼
                        ┌──────────────────┐     ┌─────────────────┐
                        │ Motor Controller │◀────│ Follower Node   │
                        │  (I2C to ESP32)  │     │  (PID Control)  │
                        └──────────────────┘     └─────────────────┘
                               │
                               ▼
                        ┌──────────────────┐
                        │      ESP32       │
                        │  Motor Control   │
                        └──────────────────┘
```

## Configuration

Key parameters can be adjusted in the ROS2 launch:

- **Camera**: 320x240 @ 15 FPS (optimized for DART-MX95)
- **Detection**: YOLOv8n @ 416px input, processing every 2 frames
- **Following**: Target distance 1.0m, max speed 0.5 m/s
- **Steering**: Full deflection at ±1.0 rad/s angular velocity

## Project Structure

```
WYZECAR/
├── wyzecar.sh          # Main control script
├── ros2/               # ROS2 nodes (Python)
│   ├── human_detector.py    # YOLOv8 person detection
│   ├── follower.py          # PID following controller
│   ├── motor_controller.py  # I2C motor commands
│   └── web_viewer.py        # Web dashboard
├── firmware/           # ESP32 firmware
│   ├── platformio.ini  # Build configuration
│   └── src/
│       └── main.cpp    # Motor control firmware
├── docker/             # Container configuration
│   ├── Dockerfile      # ROS2 Humble container
│   └── ros_entrypoint.sh
├── systemd/            # System service
│   └── wyzecar.service
├── docs/               # Documentation
│   ├── architecture.md
│   ├── i2c_protocol.md
│   ├── pinout.md
│   └── wiring.md
└── CubeOrangePlus/     # Alternative ArduRover config
```

## Operating Modes

### Remote Control Mode (Default)
- Control via web browser using WASD keys
- Real-time camera feed with human detection overlay
- Motor telemetry display
- No automatic following

### Autonomous Follow Mode
- Car automatically follows detected humans
- PID control maintains safe following distance (0.8-1.2m)
- Smooth acceleration and deceleration
- Falls back to remote control if no human detected

## I2C Protocol

Commands sent from DART to ESP32:

| Command | Byte 0 | Byte 1 | Byte 2 | Byte 3 |
|---------|--------|--------|--------|--------|
| Set Motors | 0x01 | rear_speed | front_speed | servo_angle |
| Emergency Stop | 0x02 | - | - | - |
| Request Status | 0x03 | - | - | - |

- Speeds: -100 to +100 (negative = reverse)
- Servo angle: 0-180 degrees (90 = center)

## Development

### Building ESP32 Firmware
```bash
cd firmware
pio run -t upload
```

### Building Docker Image
```bash
cd docker
docker build -t wyzecar:humble .
```

### Modifying ROS2 Nodes
The ROS2 nodes are in Python and don't require compilation. Simply edit the files in `ros2/` and restart the system.

### Installing Systemd Service
To start WYZECAR automatically on boot:
```bash
sudo cp systemd/wyzecar.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable wyzecar.service
sudo systemctl start wyzecar.service
```

## Troubleshooting

### Camera Not Found
```bash
ls -la /dev/video*
v4l2-ctl --list-devices
```

### I2C Communication Issues
```bash
i2cdetect -y 3  # Should show device at 0x42
i2cget -y 3 0x42 0x03  # Request status
```

### Container Not Starting
```bash
docker logs wyzecar
docker ps -a
```

### Web Interface Issues
- Chrome: Uses frame polling for compatibility
- Safari: Uses native MJPEG streaming
- Check browser console for errors

## Performance Tuning

- Reduce camera resolution for higher FPS
- Adjust YOLO processing frequency in `human_detector.py`
- Tune PID parameters in `follower.py`
- Modify acceleration rates in `motor_controller.py`

## License

MIT License - see LICENSE file for details

## Acknowledgments

- Built on ROS2 Humble and Ubuntu 22.04
- YOLOv8 by Ultralytics for human detection
- OpenCV for image processing
- Variscite DART-MX95 platform