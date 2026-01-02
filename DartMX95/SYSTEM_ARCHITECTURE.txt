================================================================================
                    WYZECAR - VISION-BASED HUMAN FOLLOWING SYSTEM
                         DART-MX95 + ESP32 + ROS2 ARCHITECTURE
================================================================================

PROJECT OVERVIEW
-----------------
Goal: Autonomous robot car that follows humans using computer vision
Hardware: DART-MX95 (brain) + ESP32 (motor controller) + L298N + Camera
Software: ROS2 Humble + YOLO detection + Real-time motor control

DESIGN PHILOSOPHY
-----------------
- Separation of Concerns: Vision/AI on DART-MX95, Real-time control on ESP32
- Professional Architecture: Industry-standard distributed control system
- Safety First: Multiple layers of safety systems and watchdogs
- Modularity: Each component can be developed and tested independently

================================================================================
                              SYSTEM ARCHITECTURE
================================================================================

                          ┌─────────────────────────────┐
                          │         HUMAN TARGET        │
                          │      (Being Followed)       │
                          └─────────────────────────────┘
                                        ▲
                                        │ Vision
                                        │
           ┌────────────────────────────▼────────────────────────────┐
           │                  CAM2C CUM10330_MOD                    │
           │              (USB-C Camera Module)                     │
           │         • 3.4MP Resolution (2304x1536)                │
           │         • 60fps @ 1080p                               │
           │         • Onboard ISP                                 │
           │         • Auto exposure/white balance                 │
           └────────────────────────────┬────────────────────────────┘
                                        │ USB-C
                                        │ Video Stream
                                        ▼
           ┌─────────────────────────────────────────────────────────┐
           │              DART-MX95 (MAIN COMPUTER)                  │
           │                                                         │
           │  ┌─────────────────────────────────────────────────┐    │
           │  │              ROS2 HUMBLE                        │    │
           │  │                                                 │    │
           │  │  ┌──────────────┐  ┌──────────────────────┐     │    │
           │  │  │ Camera Node  │  │ Human Detector Node  │     │    │
           │  │  │              │  │                      │     │    │
           │  │  │ • v4l2_camera│  │ • YOLOv8-nano        │     │    │
           │  │  │ • USB Video  │  │ • Person detection   │     │    │
           │  │  │ • 30fps      │  │ • Bounding boxes     │     │    │
           │  │  └──────┬───────┘  │ • Target selection   │     │    │
           │  │         │          └──────┬───────────────┘     │    │
           │  │         │                 │                     │    │
           │  │         ▼                 ▼                     │    │
           │  │    /image_raw       /target_person              │    │
           │  │                           │                     │    │
           │  │                           ▼                     │    │
           │  │  ┌──────────────────────────────────────────┐   │    │
           │  │  │          Follower Node                   │   │    │
           │  │  │                                          │   │    │
           │  │  │ • PID Controller                         │   │    │
           │  │  │ • Distance calculation                   │   │    │
           │  │  │ • Path planning                          │   │    │
           │  │  │ • Safety systems                         │   │    │
           │  │  └──────────────┬───────────────────────────┘   │    │
           │  │                 │                               │    │
           │  │                 ▼                               │    │
           │  │            /cmd_vel                             │    │
           │  │                 │                               │    │
           │  │                 ▼                               │    │
           │  │  ┌──────────────────────────────────────────┐   │    │
           │  │  │        Motor Controller Node            │   │    │
           │  │  │                                          │   │    │
           │  │  │ • UART communication                     │   │    │
           │  │  │ • Command translation                    │   │    │
           │  │  │ • Safety monitoring                      │   │    │
           │  │  │ • Emergency stop                         │   │    │
           │  │  └──────────────┬───────────────────────────┘   │    │
           │  └─────────────────│───────────────────────────────┘    │
           │                    │ UART Commands                      │
           │                    │ (115200 baud)                      │
           └────────────────────┼──────────────────────────────────────┘
                                │
                                ▼
           ┌─────────────────────────────────────────────────────────┐
           │                   ESP32 WROOM                           │
           │              (Real-Time Controller)                     │
           │                                                         │
           │  ┌─────────────────────────────────────────────────┐    │
           │  │            Motor Control Firmware              │    │
           │  │                                                 │    │
           │  │ • Hardware PWM generation                       │    │
           │  │ • Real-time motor control                       │    │
           │  │ • Servo positioning                             │    │
           │  │ • Safety watchdog                               │    │
           │  │ • UART command processing                       │    │
           │  │ • Emergency stop handling                       │    │
           │  └─────────────┬───────────────┬───────────────────┘    │
           └────────────────┼───────────────┼──────────────────────────┘
                            │               │
                            ▼               ▼
         ┌─────────────────────────┐   ┌─────────────────┐
         │        L298N             │   │  Steering Servo │
         │   Motor Driver           │   │                 │
         │                         │   │ • SG90 or similar│
         │ • Dual H-Bridge         │   │ • 5V operation   │
         │ • PWM speed control     │   │ • 0-180° range   │
         │ • Direction control     │   │ • 50Hz PWM       │
         └─────┬───────────┬───────┘   └─────────────────┘
               │           │
               ▼           ▼
         ┌──────────┐ ┌──────────┐
         │ Motor A  │ │ Motor B  │
         │ (Left)   │ │ (Right)  │
         │          │ │          │
         │ DC Gear  │ │ DC Gear  │
         │ Motor    │ │ Motor    │
         └──────────┘ └──────────┘

================================================================================
                              ROS2 NODE DETAILS
================================================================================

CAMERA NODE
-----------
Package: ros2_v4l2_camera
Type: Standard ROS2 camera driver
Function: Capture video stream from USB camera

Topics Published:
- /image_raw (sensor_msgs/Image): Raw camera frames at 30fps
- /camera_info (sensor_msgs/CameraInfo): Camera calibration data

Parameters:
- video_device: /dev/video0
- image_size: [640, 480] (for performance) or [1920, 1080] (for accuracy)
- framerate: 30.0
- pixel_format: YUYV

HUMAN DETECTOR NODE
-------------------
Package: wyzecar_vision
Type: Custom Python node
Function: Detect and track humans using YOLO

Topics Subscribed:
- /image_raw (sensor_msgs/Image): Camera frames

Topics Published:
- /detections (vision_msgs/Detection2DArray): All detected objects
- /target_person (geometry_msgs/PointStamped): Primary person to follow
- /debug_image (sensor_msgs/Image): Annotated debug image

Algorithm:
1. Receive camera frame
2. Resize for YOLO processing (640x640)
3. Run YOLOv8-nano inference
4. Filter for person class (confidence > 0.5)
5. Select largest person as target
6. Calculate normalized position (-1 to 1)
7. Publish target coordinates

FOLLOWER NODE
-------------
Package: wyzecar_control
Type: Custom Python node
Function: Generate movement commands to follow target

Topics Subscribed:
- /target_person (geometry_msgs/PointStamped): Target position

Topics Published:
- /cmd_vel (geometry_msgs/Twist): Movement commands

Control Algorithm:
1. Calculate distance to target (using bounding box size)
2. Calculate steering angle (based on X position)
3. PID controller for smooth following
4. Safety checks (lost target, obstacles, limits)
5. Generate linear and angular velocities

Parameters:
- target_distance: 2.0 meters (desired following distance)
- max_linear_speed: 0.5 m/s
- max_angular_speed: 1.0 rad/s
- lost_timeout: 3.0 seconds
- pid_gains: {kp: 1.0, ki: 0.1, kd: 0.05}

MOTOR CONTROLLER NODE
---------------------
Package: wyzecar_control
Type: Custom Python node
Function: Interface between ROS2 and ESP32

Topics Subscribed:
- /cmd_vel (geometry_msgs/Twist): Movement commands

Topics Published:
- /motor_status (std_msgs/String): ESP32 status feedback

Communication:
- Serial port: /dev/ttyUSB0 (or similar)
- Baud rate: 115200
- Protocol: Text-based commands
- Watchdog: 1 second timeout

Functions:
1. Convert Twist messages to motor commands
2. Send UART commands to ESP32
3. Monitor ESP32 status responses
4. Handle emergency stops
5. Implement safety timeouts

================================================================================
                            CONTROL FLOW DIAGRAM
================================================================================

1. VISION PROCESSING:
   Camera → Image → YOLO → Person Detection → Target Selection

2. DECISION MAKING:
   Target Position → Distance Calculation → PID Controller → Movement Commands

3. MOTOR EXECUTION:
   ROS2 cmd_vel → UART Commands → ESP32 → Hardware PWM → Motors

4. SAFETY MONITORING:
   Watchdog Timers → Emergency Stops → Target Lost Handling

Time Flow (typical 30Hz operation):
┌─ 33ms ──┐
│ Camera  │ → Image (33ms)
└─────────┘
            ┌─ 50ms ──┐
            │ YOLO    │ → Detection (50ms)
            └─────────┘
                      ┌─ 5ms ───┐
                      │ Control │ → Commands (5ms)
                      └─────────┘
                                ┌─ 1ms ───┐
                                │ ESP32   │ → Motors (1ms)
                                └─────────┘

Total latency: ~90ms from image to motor response

================================================================================
                              SAFETY SYSTEMS
================================================================================

MULTIPLE LAYERS OF SAFETY:

1. SOFTWARE WATCHDOGS:
   - ROS2 node monitoring (each node checks others)
   - ESP32 watchdog timer (1 second timeout)
   - Target lost timeout (3 seconds → stop)
   - Communication timeout (1 second → emergency stop)

2. HARDWARE SAFETY:
   - Emergency stop button (hard-wired to ESP32)
   - Battery voltage monitoring
   - Motor current monitoring (if available)
   - Physical bumper switches (optional)

3. ALGORITHMIC SAFETY:
   - Maximum speed limits
   - Acceleration limits
   - Turn rate limits
   - Cliff detection (via distance sensors, optional)

4. TARGET VALIDATION:
   - Confidence threshold for person detection
   - Minimum target size (avoid false positives)
   - Maximum target distance (prevent chasing distant people)
   - Target stability (avoid jumping between targets)

EMERGENCY STOP SEQUENCE:
1. Any safety system triggers emergency stop
2. ROS2 publishes zero velocity
3. ESP32 immediately stops all motors
4. System enters safe mode
5. Manual reset required to resume

================================================================================
                              PERFORMANCE SPECS
================================================================================

PROCESSING PERFORMANCE:
- Camera: 30 FPS at 640x480 (or 1920x1080)
- YOLO Detection: ~20 FPS on DART-MX95 ARM cores
- Control Loop: 30 Hz update rate
- Motor Commands: 100 Hz (ESP32 hardware PWM)

FOLLOWING PERFORMANCE:
- Detection Range: 0.5 - 10 meters
- Following Distance: 1-3 meters (configurable)
- Maximum Speed: 0.5 m/s (safety limited)
- Turn Radius: <1 meter
- Response Time: <100ms (image to motor)

ACCURACY:
- Person Detection: >95% (YOLO confidence > 0.5)
- Position Accuracy: ±5cm at 2m distance
- Speed Accuracy: ±2% (hardware PWM)
- Steering Accuracy: ±1° (servo positioning)

POWER CONSUMPTION:
- DART-MX95: ~3W (vision processing)
- ESP32: ~200mW (motor control)
- Motors: 1-5W each (depending on load)
- Total System: <15W typical

================================================================================
                            DEVELOPMENT PHASES
================================================================================

PHASE 1: BASIC HARDWARE SETUP
[ ] Wire ESP32 to L298N motors
[ ] Connect DART-MX95 to ESP32 via UART
[ ] Install camera and test USB connection
[ ] Power system integration and testing

PHASE 2: MOTOR CONTROL FOUNDATION
[ ] Flash ESP32 with motor control firmware
[ ] Implement UART communication protocol
[ ] Create ROS2 motor controller node
[ ] Test manual motor control via ROS2

PHASE 3: VISION SYSTEM
[ ] Set up camera node in ROS2
[ ] Implement YOLO detection node
[ ] Test person detection and tracking
[ ] Optimize detection performance

PHASE 4: FOLLOWING ALGORITHM
[ ] Implement follower control node
[ ] Develop PID controller for smooth following
[ ] Add safety systems and emergency stops
[ ] Tune control parameters

PHASE 5: INTEGRATION & TESTING
[ ] Full system integration testing
[ ] Performance optimization
[ ] Safety system validation
[ ] Real-world testing scenarios

PHASE 6: ADVANCED FEATURES
[ ] Multiple person tracking
[ ] Obstacle avoidance
[ ] Voice commands
[ ] Remote monitoring dashboard

================================================================================
                              FILE STRUCTURE
================================================================================

DART-MX95 Software:
├── wyzecar_ws/
│   ├── src/
│   │   ├── wyzecar_vision/
│   │   │   ├── nodes/
│   │   │   │   ├── camera_node.py
│   │   │   │   ├── human_detector.py
│   │   │   │   └── debug_viewer.py
│   │   │   └── launch/
│   │   │       └── vision.launch.py
│   │   ├── wyzecar_control/
│   │   │   ├── nodes/
│   │   │   │   ├── follower.py
│   │   │   │   └── motor_controller.py
│   │   │   └── launch/
│   │   │       └── control.launch.py
│   │   └── wyzecar_bringup/
│   │       └── launch/
│   │           └── wyzecar.launch.py
│   └── config/
│       ├── camera_params.yaml
│       ├── detection_params.yaml
│       └── control_params.yaml

ESP32 Firmware:
├── esp32_motor_control/
│   ├── src/
│   │   ├── main.cpp
│   │   ├── motor_control.cpp
│   │   ├── servo_control.cpp
│   │   ├── uart_comm.cpp
│   │   └── safety_systems.cpp
│   ├── include/
│   │   └── config.h
│   └── platformio.ini

Documentation:
├── docs/
│   ├── wiring_guide.md
│   ├── software_setup.md
│   ├── calibration_guide.md
│   └── troubleshooting.md

================================================================================
                              NEXT STEPS
================================================================================

1. Complete hardware wiring per ESP32_MOTOR_WIRING.txt
2. Flash ESP32 with motor control firmware
3. Set up ROS2 development environment on DART-MX95
4. Implement and test individual ROS2 nodes
5. Integrate complete system and begin testing

This architecture provides a professional, scalable foundation for a
vision-based human following robot with safety and performance as priorities.

================================================================================