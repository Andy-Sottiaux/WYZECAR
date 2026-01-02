# DART-MX95 Bring-Up Guide

This guide walks through bringing a DART-MX95 board (Debian host) online with Docker, running Ubuntu 22.04 + ROS 2 Humble inside a container, building the motor controller workspace, and driving the ESP32 motor controller.

## 1. Prepare the Debian Host

1. Update the base system and install required tools:
   ```bash
   sudo apt update
   sudo apt install -y git curl wget build-essential lsb-release ca-certificates gnupg software-properties-common minicom
   ```
2. Install Docker Engine (official repository):
   ```bash
   sudo install -m 0755 -d /etc/apt/keyrings
   curl -fsSL https://download.docker.com/linux/debian/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
   echo \
     "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/debian \
     $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list
   sudo apt update
   sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
   ```
3. Grant your user access to Docker and serial devices, then re-login:
   ```bash
   sudo usermod -aG docker,dialout $USER
   newgrp docker
   ```
4. Verify Docker is working: `docker run --rm hello-world`.
5. Confirm the ESP32 enumerates when connected via USB: `ls /dev/ttyUSB*` (or `/dev/ttyACM*`). Optionally check with minicom `sudo minicom -D /dev/ttyUSB0 -b 115200`.

## 2. Build the Development Container

1. From the repo root (`WYZECAR/`), build the Ubuntu 22.04 + ROS 2 Humble image defined in `DartMX95/docker/Dockerfile`:
   ```bash
   cd WYZECAR
   docker build -t wyzecar:humble -f DartMX95/docker/Dockerfile DartMX95/docker
   ```
2. (Optional) confirm image exists: `docker images wyzecar:humble`.

## 3. Launch the Container with Serial Access

Run the container with the workspace mounted and the ESP32 serial port passed through (swap the device path if needed):
```bash
cd WYZECAR
docker run --rm -it \
  --device=/dev/ttyUSB0 \
  --group-add $(getent group dialout | cut -d: -f3) \
  -v $PWD:/workspace \
  -w /workspace \
  wyzecar:humble \
  bash
```
- `--device=/dev/ttyUSB0` exposes the ESP32 to the container.
- `--group-add` allows the container user to open the serial port without elevating permissions.
- `/workspace` inside the container corresponds to the repo root.
- The image entrypoint automatically sources `/opt/ros/humble` and `/workspace/wyzecar_ws/install` if present.

## 4. Create the ROS 2 Workspace

Inside the container shell:
```bash
cd /workspace
mkdir -p wyzecar_ws/src
cd wyzecar_ws/src
# Create the wyzecar_control package skeleton
ros2 pkg create --build-type ament_python wyzecar_control
```
Copy the provided motor controller node into the package (overwrite the generated placeholder):
```bash
cp /workspace/DartMX95/ros2_motor_controller.py wyzecar_control/wyzecar_control/motor_controller.py
```
Update `wyzecar_control/setup.py` to install the node entry point:
```python
entry_points={
    'console_scripts': [
        'motor_controller = wyzecar_control.motor_controller:main',
    ],
},
```
Install package dependencies and build:
```bash
cd /workspace/wyzecar_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```
(Every new shell needs `source /workspace/wyzecar_ws/install/setup.bash`, but the container entrypoint handles this automatically if the path exists.)

## 5. Run and Test the Motor Controller Node

1. Ensure the ESP32 is connected and visible (`ls /dev/ttyUSB0`).
2. Start the node inside the container:
   ```bash
   ros2 run wyzecar_control motor_controller --ros-args -p serial_port:=/dev/ttyUSB0
   ```
   You should see log output indicating whether `STATUS` responses are received.
3. In another container shell (or another terminal attached to the same container), publish velocity commands:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" -r 1
   ```
   Adjust `linear.x` and `angular.z` to exercise different directions. Negative values drive backwards.
4. Monitor ESP32 feedback:
   ```bash
   ros2 topic echo /motor_status
   ```
5. To stop everything, Ctrl+C the `ros2 run` process. The node sends `STOP` during shutdown.

## 6. Common Troubleshooting

- **Serial permissions**: If the node cannot open `/dev/ttyUSB0`, ensure the device was passed with `--device` and that `--group-add` uses the host `dialout` GID (run `getent group dialout`).
- **rosdep failures**: Retry `rosdep update` from inside the container. Network access must be available (ask for approval if restricted).
- **ESP32 not responding**: Use `minicom` on the host to verify the firmware is running. Check cabling per `WIRING.txt`.
- **Multiple USB devices**: Use `dmesg | tail` after plugging in the ESP32 to confirm the correct `/dev/tty*` name, then pass that to the container and node parameter.

Following these steps sets up a reproducible containerized ROS 2 environment on the DART-MX95, enabling motor command validation against the ESP32 hardware without contaminating the Debian host system.
