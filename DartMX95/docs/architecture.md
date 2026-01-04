# WYZECAR System Architecture

## Overview

A vision-based human following robot using:
- **DART-MX95**: Main computer (ROS2, vision, AI)
- **ESP32**: Real-time motor controller (I2C slave)
- **L298N**: Motor driver
- **Camera**: USB camera for human detection

## Design Philosophy

- **Separation of Concerns**: Vision/AI on DART-MX95, real-time control on ESP32
- **Smooth Motion**: Acceleration ramping, velocity smoothing
- **Safety First**: Multiple watchdog layers

---

## System Diagram

```
┌─────────────────────────────────────────────────────────┐
│                     DART-MX95                           │
│                                                         │
│  ┌─────────────┐    ┌──────────────────┐               │
│  │ USB Camera  │───▶│  Camera Node     │               │
│  │ See3CAM     │    │  (v4l2_camera)   │               │
│  └─────────────┘    └────────┬─────────┘               │
│                              │ /image_raw              │
│                              ▼                         │
│                    ┌──────────────────┐                │
│                    │  Human Detector  │                │
│                    │  (YOLOv8-nano)   │                │
│                    └────────┬─────────┘                │
│                              │ /target_person          │
│                              ▼                         │
│                    ┌──────────────────┐                │
│                    │    Follower      │                │
│                    │ (PID Controller) │                │
│                    └────────┬─────────┘                │
│                              │ /cmd_vel                │
│                              ▼                         │
│                    ┌──────────────────┐                │
│                    │ Motor Controller │                │
│                    │ (Smooth Ramping) │                │
│                    └────────┬─────────┘                │
│                              │ I2C (/dev/i2c-3)        │
└──────────────────────────────┼──────────────────────────┘
                               │
                               ▼
┌──────────────────────────────────────────────────────────┐
│                      ESP32 WROOM                         │
│                                                          │
│  • I2C Slave (0x42)                                      │
│  • Hardware PWM                                          │
│  • Servo control                                         │
│  • Watchdog timer                                        │
└──────────────────┬───────────────────┬───────────────────┘
                   │                   │
                   ▼                   ▼
            ┌───────────┐       ┌───────────┐
            │   L298N   │       │  Steering │
            │  H-Bridge │       │   Servo   │
            └─────┬─────┘       └───────────┘
                  │
         ┌───────┴───────┐
         ▼               ▼
    ┌─────────┐     ┌─────────┐
    │  Motor  │     │  Motor  │
    │  (Rear) │     │ (Front) │
    └─────────┘     └─────────┘
```

---

## ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/image_raw` | sensor_msgs/Image | Camera frames (320x240) |
| `/target_person` | geometry_msgs/PointStamped | Target position (normalized -1 to 1) |
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/debug_image` | sensor_msgs/Image | Annotated image with detections |
| `/motor_debug` | std_msgs/String | Motor controller status |

---

## Node Details

### Human Detector (human_detector.py)
- **Model**: YOLOv8-nano
- **Input**: 416x416 pixels
- **Output**: Normalized position (-1 to 1) and distance
- **Frame skip**: Every 2 frames for performance

### Follower (follower.py)
- **Control**: PID for linear and angular velocity
- **Parameters**:
  - `max_linear_speed`: 0.45 m/s
  - `max_angular_speed`: 0.8 rad/s
  - `target_distance`: 0.4 (normalized)
  - `dead_zone`: 0.1

### Motor Controller (motor_controller.py)
- **Acceleration**: 25%/s
- **Deceleration**: 35%/s
- **Servo slew rate**: 60°/s
- **Control rate**: 25 Hz
- **Max speed**: 60%

---

## I2C Protocol

| Command | Format | Description |
|---------|--------|-------------|
| 0x01 | `[0x01, rear, front, servo]` | Set motor speeds |
| 0x02 | `[0x02]` | Emergency stop |
| 0x03 | `[0x03]` | Request status |

- **Address**: 0x42
- **Bus**: I2C-3 (/dev/i2c-3)
- **Speed**: 100 kHz

---

## Safety Systems

1. **ESP32 Watchdog**: 2 second timeout
2. **Command Timeout**: Stops if no commands for 2 seconds
3. **Acceleration Limiting**: Prevents sudden speed changes
4. **Emergency Stop**: I2C command 0x02

---

## Performance

| Metric | Value |
|--------|-------|
| Camera FPS | 30 |
| Detection FPS | ~5 |
| Control Rate | 25 Hz |
| Latency (image to motor) | ~100ms |
| Max Speed | 60% (~0.5 m/s) |

---

## File Structure

```
ros2/
├── human_detector.py   # YOLOv8 person detection
├── follower.py         # PID following logic
├── motor_controller.py # Smooth motor control
└── web_viewer.py       # Browser MJPEG stream

src/
└── main.cpp            # ESP32 firmware
```
