# WYZECAR DartMX95 - Quick Reference

This is the main implementation of WYZECAR using the DART-MX95 platform.

## Quick Start

```bash
# SSH into DART-MX95
ssh root@<dart-ip>
sudo -i

# Start WYZECAR
cd /root/WYZECAR/DartMX95
./wyzecar.sh
```

Open `http://<dart-ip>:8080` in your browser and use WASD keys to drive!

## Main Control Script

The `wyzecar.sh` script handles everything:

```bash
./wyzecar.sh              # Start in remote control mode (default)
./wyzecar.sh follow       # Start in autonomous follow mode
./wyzecar.sh stop         # Stop WYZECAR
./wyzecar.sh status       # Show system status
./wyzecar.sh logs         # View live logs
./wyzecar.sh update       # Pull latest from GitHub
./wyzecar.sh help         # Show all options
```

## Hardware Setup

### Required Components
- DART-MX95 SoM + carrier board
- ESP32 WROOM development board
- L298N motor driver
- 2x DC motors (rear + front)
- 1x Servo (steering)
- USB camera
- 7-12V battery

### Wiring

#### I2C Connection (J6 on DART)
- Pin 18 (SCL) → ESP32 GPIO22
- Pin 20 (SDA) → ESP32 GPIO21  
- Pin 12 (GND) → ESP32 GND

#### Motor Connections
See main README for detailed motor wiring diagram.

## Software Architecture

### Docker Container
- Base: ROS2 Humble on Ubuntu 22.04
- Nodes run inside container
- Automatic startup via systemd (optional)

### ROS2 Nodes
1. **Camera** - V4L2 capture at 320x240 @ 15fps
2. **Human Detector** - YOLOv8n person detection
3. **Follower** - PID controller (follow mode only)
4. **Motor Controller** - I2C commands to ESP32
5. **Web Viewer** - Browser interface on port 8080

### Communication Flow
```
Camera → Detector → Follower → Motor Controller → ESP32 → Motors
                         ↓
Web Browser ← Web Viewer
```

## Configuration

Key parameters (in scripts/start_wyzecar.sh):
- Camera resolution: 320x240
- Camera FPS: 15
- YOLO model: yolov8n
- Detection rate: Every 2 frames
- Web port: 8080

## Development

### Modify ROS2 Nodes
Edit Python files in `ros2/` directory, then restart:
```bash
./wyzecar.sh restart
```

### Update ESP32 Firmware
```bash
pio run -t upload
pio device monitor  # View debug output
```

### View Container Logs
```bash
./wyzecar.sh logs
# Or specific node:
docker exec wyzecar tail -f /workspace/logs/wyzecar_*/motor.log
```

## Troubleshooting

### No Camera Image
1. Check USB: `ls -la /dev/video*`
2. Test camera: `v4l2-ctl --device=/dev/video13 --all`
3. Check container: `docker logs wyzecar`

### Motors Not Moving
1. Check I2C: `i2cdetect -y 3` (should show 42)
2. Test communication: `i2cset -y 3 0x42 0x03`
3. Check ESP32 serial output
4. Verify motor power supply

### Web Interface Issues
- Chrome: Automatically uses compatibility mode
- Safari: Native MJPEG streaming
- Check firewall: Port 8080 must be open

## Files Overview

- `wyzecar.sh` - Main control script
- `ros2/` - ROS2 Python nodes
- `src/main.cpp` - ESP32 firmware
- `scripts/` - Helper scripts
- `docker/` - Container config
- `docs/` - Technical documentation

For more details, see the main repository README.