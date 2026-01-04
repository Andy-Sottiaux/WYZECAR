#!/bin/bash
# WYZECAR Startup Script
# Run this inside the Docker container to start all nodes

set -e

LOGDIR="/workspace/logs"
mkdir -p $LOGDIR
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RUNLOGDIR="$LOGDIR/wyzecar_$TIMESTAMP"
mkdir -p "$RUNLOGDIR"
LOGFILE="$RUNLOGDIR/run.log"
export ROS_LOG_DIR="$RUNLOGDIR/ros2"

# Make Ultralytics cache writable inside container (prevents warnings + partial writes)
export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-/tmp/Ultralytics}"
mkdir -p "$YOLO_CONFIG_DIR" 2>/dev/null || true

# Ensure Python flushes logs promptly when redirected
export PYTHONUNBUFFERED=1

# Line-buffered output so logs flush immediately
export STDBUF_CMD="stdbuf -oL -eL"

# Source ROS2
source /opt/ros/humble/setup.bash
source /workspace/wyzecar_ws/install/setup.bash 2>/dev/null || true

print_banner() {
    echo ""
    echo "╔════════════════════════════════════════════╗"
    echo "║     WYZECAR Human Following System         ║"
    echo "╚════════════════════════════════════════════╝"
    echo ""
}

# Check what to do
case "${1:-all}" in
  build)
    echo "Building ROS2 workspace..."
    cd /workspace/wyzecar_ws
    colcon build
    source install/setup.bash
    echo "Build complete!"
    ;;
    
  all)
    print_banner
    echo "  Camera:  /dev/video13"
    echo "  I2C:     /dev/i2c-3 @ 0x42"
    echo "  Web:     http://0.0.0.0:8080"
    echo "  Logs:    $RUNLOGDIR"
    echo ""
    echo "Starting nodes..."
    
    # Start nodes - separate log files per process (much easier to debug crashes)
    CAMLOG="$RUNLOGDIR/camera.log"
    DETLOG="$RUNLOGDIR/detector.log"
    FOLLOG="$RUNLOGDIR/follower.log"
    MOTLOG="$RUNLOGDIR/motor.log"
    WEBLOG="$RUNLOGDIR/web.log"

    echo "=== WYZECAR run $TIMESTAMP ===" > "$LOGFILE"
    echo "Logs:" >> "$LOGFILE"
    echo "  camera:   $CAMLOG" >> "$LOGFILE"
    echo "  detector: $DETLOG" >> "$LOGFILE"
    echo "  follower: $FOLLOG" >> "$LOGFILE"
    echo "  motor:    $MOTLOG" >> "$LOGFILE"
    echo "  web:      $WEBLOG" >> "$LOGFILE"
    echo "" >> "$LOGFILE"

    # Camera at 320x240 UYVY (MJPG not supported by v4l2_camera)
    $STDBUF_CMD ros2 run v4l2_camera v4l2_camera_node --ros-args \
      -p video_device:=/dev/video13 \
      -p image_size:=[320,240] \
      -p pixel_format:=UYVY \
      >> "$CAMLOG" 2>&1 &
    CAM_PID=$!
    echo "  ✓ Camera node (320x240)"
    sleep 2
    
    # Human detector with aggressive frame skipping for ARM CPU
    $STDBUF_CMD ros2 run wyzecar_vision human_detector --ros-args \
      -p process_every_n_frames:=5 \
      -p input_size:=256 \
      -p confidence_threshold:=0.4 \
      >> "$DETLOG" 2>&1 &
    DET_PID=$!
    echo "  ✓ Human detector (YOLO @ 256px, skip 5)"
    sleep 5
    
    $STDBUF_CMD ros2 run wyzecar_vision follower \
      >> "$FOLLOG" 2>&1 &
    FOL_PID=$!
    echo "  ✓ Follower controller"
    sleep 1
    
    $STDBUF_CMD ros2 run wyzecar_control motor_controller --ros-args \
      -p i2c_bus:=3 \
      -p command_timeout:=9999.0 \
      >> "$MOTLOG" 2>&1 &
    MOT_PID=$!
    echo "  ✓ Motor controller"
    sleep 1
    
    echo ""
    echo "════════════════════════════════════════════"
    echo "  All nodes running!"
    echo "  Open: http://192.168.5.183:8080"
    echo "  Logs: tail -f $RUNLOGDIR/*.log"
    echo "  Press Ctrl+C to stop"
    echo "════════════════════════════════════════════"
    echo ""
    
    # Web viewer in foreground (if it exits, we want the reason in WEBLOG)
    $STDBUF_CMD python3 /workspace/DartMX95/ros2/web_viewer.py >> "$WEBLOG" 2>&1
    
    # Cleanup on exit
    echo ""
    echo "Stopping all nodes..."
    kill "$CAM_PID" "$DET_PID" "$FOL_PID" "$MOT_PID" 2>/dev/null || true
    pkill -f "ros2 run" 2>/dev/null || true
    echo "Done. Full logs saved to: $RUNLOGDIR"
    ;;
    
  camera)
    ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video13 -p image_size:=[640,480]
    ;;
    
  detector)
    ros2 run wyzecar_vision human_detector
    ;;
    
  follower)
    ros2 run wyzecar_vision follower
    ;;
    
  motor)
    ros2 run wyzecar_control motor_controller --ros-args -p i2c_bus:=3 -p command_timeout:=9999.0
    ;;
    
  web)
    python3 /workspace/DartMX95/ros2/web_viewer.py
    ;;
    
  status)
    echo ""
    echo "Active ROS2 nodes:"
    ros2 node list 2>/dev/null || echo "  (none or ROS2 not running)"
    echo ""
    echo "Topic rates (5 sec sample):"
    timeout 5 ros2 topic hz /image_raw --window 5 2>/dev/null || echo "  /image_raw: not publishing"
    timeout 5 ros2 topic hz /target_person --window 5 2>/dev/null || echo "  /target_person: not publishing"
    ;;
    
  logs)
    if [ -f "$LOGDIR/wyzecar_*.log" ]; then
      tail -f "$LOGDIR"/wyzecar_*.log | head -1
    else
      echo "No logs found. Start with: $0 all"
    fi
    ;;
    
  *)
    print_banner
    echo "Usage: $0 {all|build|status|logs|camera|detector|follower|motor|web}"
    echo ""
    echo "  all      - Start everything (recommended)"
    echo "  build    - Build the ROS2 workspace"
    echo "  status   - Show running nodes and topic rates"
    echo "  logs     - Tail the log file"
    echo ""
    echo "Individual nodes (for debugging):"
    echo "  camera   - Camera only (verbose)"
    echo "  detector - Human detector only (verbose)"
    echo "  follower - Follower only (verbose)"
    echo "  motor    - Motor controller only (verbose)"
    echo "  web      - Web viewer only"
    ;;
esac

