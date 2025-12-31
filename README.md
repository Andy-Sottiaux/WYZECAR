# WyzeCar

Autonomous RC car platform with two hardware configurations:

1. **CubeOrangePlus** - ArduRover-based autopilot with ESP32 motor control
2. **DartMX95** - ROS2 vision-based human tracking platform (in development)

## CubeOrangePlus Configuration

RC car controlled by Cube Orange Plus running ArduRover firmware, with an ESP32 handling motor control.

### Hardware
- **Autopilot**: Cube Orange Plus (ArduRover)
- **Motor Controller**: ESP32 WROOM
- **Motor Driver**: L298N dual H-bridge
- **Motors**: 2x DC motors (differential drive)
- **Steering**: Servo (direct PWM from Cube)

### Signal Flow
```
Cube --[Throttle PWM]--> ESP32 --[Direction + PWM]--> L298N --> Motors
Cube --[Steering PWM]--> Servo
```

### ESP32 Firmware Features
- Reads throttle PWM from Cube Orange Plus (1000-2000us)
- Controls L298N motor driver via GPIO/PWM
- Two modes: AUTO (Cube control) and MANUAL (serial commands)
- Safety timeout stops motors if PWM signal lost

### Serial Commands
```
M1:<speed>   - Set Motor 1 speed (-255 to 255)
M2:<speed>   - Set Motor 2 speed (-255 to 255)
STOP         - Stop all motors
STATUS       - Show current status
MODE:AUTO    - Cube PWM control
MODE:MANUAL  - Serial control
```

### Building (PlatformIO)
```bash
cd CubeOrangePlus
pio run
pio run -t upload
```

### GUI Motor Test
```bash
cd CubeOrangePlus/gui
pip install -r requirements.txt
python motor_control.py
```

## DartMX95 Configuration (Planned)

Vision-based human tracking and following using ROS2.

### Hardware
- **SoM**: Variscite DART-MX95 (NXP i.MX 95)
  - 6x Cortex-A55 @ 2.0GHz
  - NPU for AI/ML acceleration
- **Carrier**: Sonata board
- **Camera**: USB camera (AR0330 sensor)
- **Motors**: L298N + DC motors (same as CubeOrangePlus)

### Software Stack
- Host OS: Debian
- Container: Docker with Ubuntu 22.04
- Middleware: ROS2 Humble
- Vision: YOLOv8-nano for human detection

### ROS2 Nodes (Planned)
- `camera_node` - Publishes camera images
- `human_detector_node` - Detects humans, publishes target position
- `motor_controller_node` - Controls L298N via GPIO/PWM
- `follower_node` - PID controller to follow detected human

## Project Structure
```
WyzeCar/
├── CubeOrangePlus/
│   ├── src/main.cpp         # ESP32 firmware
│   ├── platformio.ini       # PlatformIO config
│   ├── gui/                  # Python motor test GUI
│   ├── WIRING.txt           # Wiring diagrams
│   └── ARDUROVER_SETUP.txt  # ArduRover configuration
│
└── DartMX95/
    ├── ARCHITECTURE.txt     # System architecture
    ├── SETUP_GUIDE.txt      # Setup instructions
    └── WIRING.txt           # Wiring diagrams
```

## Wiring Overview

### Power
- Main battery (7-12V) powers L298N
- L298N 5V regulator powers ESP32 and servo
- Separate 5V BEC powers Cube Orange Plus

### Connections
| ESP32 Pin | L298N Pin | Function |
|-----------|-----------|----------|
| GPIO25 | ENA | Motor 1 PWM |
| GPIO26 | IN1 | Motor 1 Dir A |
| GPIO27 | IN2 | Motor 1 Dir B |
| GPIO12 | IN3 | Motor 2 Dir A |
| GPIO13 | IN4 | Motor 2 Dir B |
| GPIO14 | ENB | Motor 2 PWM |
| GPIO34 | - | Cube throttle input |

See `CubeOrangePlus/WIRING.txt` for full diagrams.

## License

MIT
