# DART-MX95 + ESP32 Setup Guide

This guide walks through setting up the DART-MX95 (Debian host) with the ESP32 motor controller using I2C communication, running Ubuntu 22.04 + ROS 2 Humble inside a Docker container.

## Hardware Configuration (Verified Working ✓)

### I2C Wiring (J6 Header)

| DART-MX95 J6 Pin | Signal   | ESP32 Pin | Function      |
| ---------------- | -------- | --------- | ------------- |
| Pin 18           | I2C3_SCL | GPIO22    | I2C Clock     |
| Pin 20           | I2C3_SDA | GPIO21    | I2C Data      |
| Pin 12           | GND      | GND       | Common Ground |

- **I2C Bus**: `/dev/i2c-3`
- **ESP32 Address**: `0x42`
- **Pull-ups**: 10kΩ on SOM (no external needed)

### Motor/Servo Wiring (ESP32 to L298N)

| ESP32 Pin | L298N Pin | Function          |
| --------- | --------- | ----------------- |
| GPIO25    | ENA       | Left Motor PWM    |
| GPIO26    | IN1       | Left Motor Dir A  |
| GPIO27    | IN2       | Left Motor Dir B  |
| GPIO12    | IN3       | Right Motor Dir A |
| GPIO13    | IN4       | Right Motor Dir B |
| GPIO14    | ENB       | Right Motor PWM   |
| GPIO33    | -         | Servo Signal      |

---

## 1. Prepare the Debian Host

1. Update the base system and install required tools:

   ```bash
   sudo apt update
   sudo apt install -y git curl wget build-essential i2c-tools
   ```

2. Verify I2C bus is available:

   ```bash
   ls /dev/i2c-*
   # Should show /dev/i2c-3
   ```

3. Test ESP32 connection (after flashing firmware):

   ```bash
   i2cdetect -y 3
   # Should show 42 at address 0x42
   ```

4. Install Docker Engine:

   ```bash
   sudo install -m 0755 -d /etc/apt/keyrings
   curl -fsSL https://download.docker.com/linux/debian/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
   echo \
     "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian \
     $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list
   sudo apt update
   sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
   ```

5. Grant your user access to Docker and I2C:

   ```bash
   sudo usermod -aG docker,i2c $USER
   newgrp docker
   ```

6. Verify Docker is working:
   ```bash
   docker run --rm hello-world
   ```

---

## 2. Flash ESP32 Firmware (from development machine)

On your development machine (Mac/PC with PlatformIO):

```bash
cd WYZECAR/DartMX95
pio run -t upload
pio device monitor -b 115200
```

You should see:

```
╔════════════════════════════════════════════╗
║  ESP32 I2C Motor Controller v3.0           ║
║  Using ESP-IDF I2C Slave API               ║
╚════════════════════════════════════════════╝
[5s] I2C 0x42 | L=0 R=0 S=90 | Last: 0ms ago
```

---

## 3. Test I2C Communication

On the DART-MX95:

```bash
# Detect ESP32
i2cdetect -y 3
# Should show 42

# Test motor command (50% forward, servo center)
i2cset -y 3 0x42 0x01 50 50 90 i

# Emergency stop
i2cset -y 3 0x42 0x02
```

---

## 4. Build the Development Container

1. From the repo root, build the Ubuntu 22.04 + ROS 2 Humble image:

   ```bash
   cd WYZECAR
   docker build -t wyzecar:humble -f DartMX95/docker/Dockerfile DartMX95/docker
   ```

2. Confirm image exists:
   ```bash
   docker images wyzecar:humble
   ```

---

## 5. Launch the Container with I2C Access

Run the container with I2C device access:

```bash
cd WYZECAR
docker run --rm -it \
  --device=/dev/i2c-3 \
  --privileged \
  -v $PWD:/workspace \
  -w /workspace \
  wyzecar:humble \
  bash
```

- `--device=/dev/i2c-3` exposes the I2C bus to the container
- `--privileged` allows I2C access (or use proper group permissions)
- `/workspace` inside the container corresponds to the repo root

---

## 6. Create the ROS 2 Workspace

Inside the container shell:

```bash
cd /workspace
mkdir -p wyzecar_ws/src
cd wyzecar_ws/src

# Create the wyzecar_control package
ros2 pkg create --build-type ament_python wyzecar_control
```

Copy the I2C motor controller node:

```bash
cp /workspace/DartMX95/ros2_motor_controller_i2c.py wyzecar_control/wyzecar_control/motor_controller.py
```

Update `wyzecar_control/setup.py` entry points:

```python
entry_points={
    'console_scripts': [
        'motor_controller = wyzecar_control.motor_controller:main',
    ],
},
```

Install dependencies and build:

```bash
# Install smbus2 for I2C communication
pip3 install smbus2

cd /workspace/wyzecar_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

---

## 7. Run and Test the Motor Controller Node

1. Start the node inside the container:

   ```bash
   ros2 run wyzecar_control motor_controller --ros-args -p i2c_bus:=3
   ```

   You should see:

   ```
   [INFO] Motor Controller I2C Node started on bus 3, address 0x42
   [INFO] ESP32 I2C connection established
   ```

2. In another terminal, publish velocity commands:

   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 1
   ```

3. Monitor motor status:

   ```bash
   ros2 topic echo /motor_status
   ```

4. To stop, Ctrl+C the node. It sends emergency stop during shutdown.

---

## 8. I2C Command Reference

| Command    | Bytes        | Description                | Example                            |
| ---------- | ------------ | -------------------------- | ---------------------------------- |
| Set Motors | `0x01 L R S` | L/R: -100 to 100, S: 0-180 | `i2cset -y 3 0x42 0x01 50 50 90 i` |
| E-Stop     | `0x02`       | Emergency stop             | `i2cset -y 3 0x42 0x02`            |
| Status     | `0x03`       | Request status             | `i2cset -y 3 0x42 0x03`            |

---

## 9. Common Troubleshooting

### ESP32 not detected (no 42 in i2cdetect)

- Check wiring: SDA=GPIO21→Pin20, SCL=GPIO22→Pin18
- Verify common ground connection
- Check ESP32 power (LED blinking?)
- Monitor ESP32 serial at 115200 baud

### I2C permission denied in container

- Use `--privileged` flag
- Or add user to i2c group: `sudo usermod -aG i2c $USER`

### Motors don't respond

- Test with direct i2cset command first
- Check ESP32 serial monitor for received commands
- Verify L298N wiring and power

### ROS2 node can't find smbus2

- Install inside container: `pip3 install smbus2`

---

## Quick Test Commands

```bash
# From DART-MX95 shell (not Docker):

# Check ESP32 is connected
i2cdetect -y 3

# Drive forward 50%
i2cset -y 3 0x42 0x01 50 50 90 i

# Turn left (left motor slower)
i2cset -y 3 0x42 0x01 30 50 90 i

# Steer servo left
i2cset -y 3 0x42 0x01 50 50 45 i

# Stop
i2cset -y 3 0x42 0x02
```

---

## 10. Connect the Camera (CAM2C CUM10330_MOD)

1. Connect the CAM2C CUM10330_MOD camera via USB-C to the DART-MX95 **J33** port.

2. Verify the camera is detected:

   ```bash
   # On DART-MX95 host (not in Docker):
   ls /dev/video*
   # Should show /dev/video0 (or similar)

   v4l2-ctl --list-devices
   # Should list the CUM10330_MOD camera
   ```

3. Test camera capture:

   ```bash
   # Capture a test frame (requires v4l-utils)
   v4l2-ctl --device=/dev/video0 --stream-mmap --stream-count=1 --stream-to=test.raw
   ```

---

## 11. Launch Container with Camera + I2C Access

Run the container with both camera and I2C access:

```bash
cd WYZECAR
docker run --rm -it \
  --device=/dev/i2c-3 \
  --device=/dev/video0 \
  --privileged \
  -v $PWD:/workspace \
  -w /workspace \
  wyzecar:humble \
  bash
```

Inside the container, verify devices:

```bash
ls /dev/video0   # Camera
ls /dev/i2c-3    # I2C bus
```

---

## 12. Set Up Vision Nodes

Inside the container, copy the vision nodes:

```bash
cd /workspace/wyzecar_ws/src

# Create vision package
ros2 pkg create --build-type ament_python wyzecar_vision

# Copy the detector and follower nodes
cp /workspace/DartMX95/human_detector.py wyzecar_vision/wyzecar_vision/human_detector.py
cp /workspace/DartMX95/follower.py wyzecar_vision/wyzecar_vision/follower.py
```

Update `wyzecar_vision/setup.py` entry points:

```python
entry_points={
    'console_scripts': [
        'human_detector = wyzecar_vision.human_detector:main',
        'follower = wyzecar_vision.follower:main',
    ],
},
```

Rebuild:

```bash
cd /workspace/wyzecar_ws
colcon build
source install/setup.bash
```

---

## 13. Run the Human Following System

### Terminal 1: Camera Node

```bash
source /workspace/wyzecar_ws/install/setup.bash
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/video0 \
  -p image_size:=[640,480] \
  -p pixel_format:=YUYV
```

### Terminal 2: Human Detector (YOLO)

```bash
source /workspace/wyzecar_ws/install/setup.bash
ros2 run wyzecar_vision human_detector --ros-args \
  -p model:=yolov8n.pt \
  -p confidence_threshold:=0.5
```

On first run, YOLOv8-nano model (~6MB) will be downloaded automatically.

### Terminal 3: Follower Controller

```bash
source /workspace/wyzecar_ws/install/setup.bash
ros2 run wyzecar_vision follower --ros-args \
  -p target_distance:=0.4 \
  -p max_linear_speed:=0.5 \
  -p max_angular_speed:=1.0
```

### Terminal 4: Motor Controller

```bash
source /workspace/wyzecar_ws/install/setup.bash
ros2 run wyzecar_control motor_controller --ros-args -p i2c_bus:=3
```

---

## 14. Verify the Vision Pipeline

Monitor topics to verify everything is connected:

```bash
# List all active topics
ros2 topic list

# Check if camera is publishing
ros2 topic hz /image_raw

# Check detected target position
ros2 topic echo /target_person

# Check velocity commands being sent
ros2 topic echo /cmd_vel
```

Expected data flow:

```
/image_raw → Human Detector → /target_person → Follower → /cmd_vel → Motor Controller → ESP32
```

---

## 15. Tune Following Parameters

The follower node has several tunable parameters:

| Parameter           | Default       | Description                                      |
| ------------------- | ------------- | ------------------------------------------------ |
| `target_distance`   | 0.4           | Desired distance to person (0=very close, 1=far) |
| `max_linear_speed`  | 0.5           | Maximum forward/reverse speed                    |
| `max_angular_speed` | 1.0           | Maximum turning speed                            |
| `lost_timeout`      | 2.0           | Seconds before stopping if target lost           |
| `dead_zone`         | 0.1           | Error threshold before movement                  |
| `linear_kp/ki/kd`   | 0.8/0.05/0.1  | PID gains for distance control                   |
| `angular_kp/ki/kd`  | 1.2/0.02/0.15 | PID gains for steering control                   |

Adjust at runtime:

```bash
ros2 param set /follower target_distance 0.3
ros2 param set /follower max_linear_speed 0.3
```

---

## 16. Create a Launch File (Optional)

Create `wyzecar_bringup/launch/follow.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            parameters=[{
                'video_device': '/dev/video0',
                'image_size': [640, 480],
                'pixel_format': 'YUYV'
            }]
        ),
        Node(
            package='wyzecar_vision',
            executable='human_detector',
            parameters=[{
                'model': 'yolov8n.pt',
                'confidence_threshold': 0.5
            }]
        ),
        Node(
            package='wyzecar_vision',
            executable='follower',
            parameters=[{
                'target_distance': 0.4,
                'max_linear_speed': 0.5
            }]
        ),
        Node(
            package='wyzecar_control',
            executable='motor_controller',
            parameters=[{'i2c_bus': 3}]
        ),
    ])
```

Then launch all nodes with:

```bash
ros2 launch wyzecar_bringup follow.launch.py
```

---

## Camera Troubleshooting

### Camera not detected (/dev/video0 missing)

- Check USB-C cable connection to J33
- Try a different USB-C cable (data-capable)
- Verify camera power LED
- Check `dmesg | tail -30` for USB errors

### Low FPS or lag

- Reduce resolution: `image_size:=[320,240]`
- Check CPU usage: `htop`
- Ensure adequate cooling for DART-MX95

### YOLO model download fails

- Check network connectivity in container
- Pre-download: `python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"`

---

Following these steps sets up a reproducible containerized ROS 2 environment on the DART-MX95 with reliable I2C communication to the ESP32 motor controller, and a complete vision-based human following system using YOLO.
