#!/usr/bin/env bash
#
# WYZECAR - Unified Control Script
# 
# This is the main entry point for WYZECAR operations.
# Run this script on the DART-MX95 host to control the system.
#
# Usage:
#   ./wyzecar.sh [command] [options]
#
# Commands:
#   start [mode]    Start WYZECAR (modes: remote, follow, all, nodetect)
#   stop            Stop WYZECAR
#   restart [mode]  Restart WYZECAR
#   status          Show system status
#   logs            View live logs
#   update          Pull latest code from GitHub
#   help            Show this help
#
# Quick start:
#   ./wyzecar.sh              # Start in remote control mode
#   ./wyzecar.sh follow       # Start in autonomous follow mode
#

set -euo pipefail

# Get script location
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$SCRIPT_DIR"

# Configuration
DOCKER_IMAGE="${IMAGE:-wyzecar:humble}"
CONTAINER_NAME="wyzecar"
DEFAULT_MODE="remote"
I2C_DEVICE="${I2C_DEVICE:-/dev/i2c-3}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video13}"

# Check if running inside container
IN_CONTAINER=false
if [ -f /.dockerenv ]; then
    IN_CONTAINER=true
fi

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper functions
print_banner() {
    echo -e "${BLUE}"
    echo "╔════════════════════════════════════════════╗"
    echo "║            WYZECAR Control System          ║"
    echo "╚════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${YELLOW}→ $1${NC}"
}

# Check if running as root
check_root() {
    if [ "$EUID" -ne 0 ] && [ "$IN_CONTAINER" = "false" ]; then
        print_error "Please run as root (use: sudo -i)"
        exit 1
    fi
}

# Include the container startup logic
source_ros() {
    source /opt/ros/humble/setup.bash
    source /workspace/wyzecar_ws/install/setup.bash 2>/dev/null || true
}

# Container mode: start ROS2 nodes
container_start() {
    local MODE="${1:-all}"
    
    # Setup environment
    export LOGDIR="/workspace/logs"
    mkdir -p "$LOGDIR"
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    RUNLOGDIR="$LOGDIR/wyzecar_$TIMESTAMP"
    mkdir -p "$RUNLOGDIR"
    export ROS_LOG_DIR="$RUNLOGDIR/ros2"
    
    # Configure environment
    export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-/tmp/Ultralytics}"
    mkdir -p "$YOLO_CONFIG_DIR" 2>/dev/null || true
    export PYTHONUNBUFFERED=1
    export RCUTILS_LOGGING_USE_STDOUT=1
    export RCUTILS_COLORIZED_OUTPUT=0
    export RCUTILS_LOGGING_BUFFERED_STREAM=0
    
    # Line buffering
    STDBUF_CMD="stdbuf -oL -eL"
    if ! command -v stdbuf >/dev/null 2>&1; then
        STDBUF_CMD=""
    fi
    
    # Source ROS2
    source_ros
    
    print_banner
    
    case "$MODE" in
        all|remote|nodetect)
            # Mode flags
            REMOTE_MODE=0
            DISABLE_DETECTION=0
            
            case "$MODE" in
                remote)
                    REMOTE_MODE=1
                    echo "  🎮 REMOTE CONTROL MODE"
                    ;;
                nodetect)
                    DISABLE_DETECTION=1
                    echo "  ⚠ NO DETECTION MODE"
                    ;;
                all)
                    echo "  🤖 AUTONOMOUS FOLLOW MODE"
                    ;;
            esac
            
            echo ""
            echo "  Camera:  /dev/video13"
            echo "  I2C:     /dev/i2c-3 @ 0x42"
            echo "  Web:     http://0.0.0.0:8080"
            echo "  Logs:    $RUNLOGDIR"
            echo ""
            echo "Starting nodes..."
            
            # Log files
            CAMLOG="$RUNLOGDIR/camera.log"
            DETLOG="$RUNLOGDIR/detector.log"
            FOLLOG="$RUNLOGDIR/follower.log"
            MOTLOG="$RUNLOGDIR/motor.log"
            WEBLOG="$RUNLOGDIR/web.log"
            
            # Start camera node
            $STDBUF_CMD ros2 run v4l2_camera v4l2_camera_node --ros-args \
              -p video_device:=/dev/video13 \
              -p image_size:=[320,240] \
              -p pixel_format:=UYVY \
              -p framerate:=15.0 \
              >> "$CAMLOG" 2>&1 &
            CAM_PID=$!
            print_success "Camera node (320x240 @ 15fps)"
            sleep 2
            
            # Start detector
            DETECT_FLAGS=""
            if [ "$DISABLE_DETECTION" = "1" ]; then
                DETECT_FLAGS="-p disable_detection:=true"
            fi
            $STDBUF_CMD python3 /workspace/ros2/human_detector.py --ros-args \
              -p process_every_n_frames:=2 \
              -p input_size:=416 \
              -p confidence_threshold:=0.45 \
              $DETECT_FLAGS \
              >> "$DETLOG" 2>&1 &
            DET_PID=$!
            print_success "Human detector"
            sleep 3
            
            # Start follower (only in auto mode)
            FOL_PID=""
            if [ "$REMOTE_MODE" = "0" ]; then
                $STDBUF_CMD python3 /workspace/ros2/follower.py \
                  >> "$FOLLOG" 2>&1 &
                FOL_PID=$!
                print_success "Follower controller"
            fi
            
            # Start motor controller
            $STDBUF_CMD python3 /workspace/ros2/motor_controller.py --ros-args \
              -p i2c_bus:=3 \
              -p command_timeout:=9999.0 \
              >> "$MOTLOG" 2>&1 &
            MOT_PID=$!
            print_success "Motor controller"
            
            echo ""
            echo "════════════════════════════════════════════════"
            if [ "$REMOTE_MODE" = "1" ]; then
                echo "  Open: http://<dart-ip>:8080"
                echo "  Use WASD keys to drive!"
            else
                echo "  Open: http://<dart-ip>:8080"
                echo "  Car will follow detected humans"
            fi
            echo ""
            echo "  Logs: tail -f $RUNLOGDIR/*.log"
            echo "  Press Ctrl+C to stop"
            echo "════════════════════════════════════════════════"
            echo ""
            
            # Start web viewer (foreground)
            $STDBUF_CMD python3 /workspace/ros2/web_viewer.py >> "$WEBLOG" 2>&1
            
            # Cleanup
            echo ""
            echo "Stopping nodes..."
            kill "$CAM_PID" "$DET_PID" "$MOT_PID" 2>/dev/null || true
            [ -n "$FOL_PID" ] && kill "$FOL_PID" 2>/dev/null || true
            ;;
            
        build)
            echo "Building ROS2 workspace..."
            cd /workspace/wyzecar_ws
            colcon build
            source install/setup.bash
            print_success "Build complete"
            ;;
            
        *)
            print_error "Unknown mode: $MODE"
            exit 1
            ;;
    esac
}

# Host mode: Docker operations
host_update() {
    print_info "Updating WYZECAR from GitHub..."
    cd "$REPO_ROOT"
    
    if ! git diff-index --quiet HEAD --; then
        print_error "Uncommitted changes detected. Please commit or stash first."
        exit 1
    fi
    
    git pull origin main
    print_success "Updated successfully"
    
    # Make scripts executable
    chmod +x "$REPO_ROOT/wyzecar.sh"
}

host_start() {
    local MODE="${1:-$DEFAULT_MODE}"
    
    print_banner
    
    # Stop any running instance
    docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
    
    # Auto-detect video device if needed
    if [[ ! -e "$VIDEO_DEVICE" ]]; then
        print_info "Camera not at $VIDEO_DEVICE, searching..."
        for d in /dev/video*; do
            if [[ -e "$d" ]]; then
                VIDEO_DEVICE="$d"
                print_success "Found camera at $VIDEO_DEVICE"
                break
            fi
        done
    fi
    
    # Verify devices
    if [[ ! -e "$I2C_DEVICE" ]]; then
        print_error "I2C device not found: $I2C_DEVICE"
        exit 1
    fi
    
    if [[ ! -e "$VIDEO_DEVICE" ]]; then
        print_error "No camera device found"
        exit 1
    fi
    
    print_info "Starting WYZECAR..."
    print_info "Camera: $VIDEO_DEVICE"
    print_info "I2C: $I2C_DEVICE"
    
    # Create logs directory
    mkdir -p "$REPO_ROOT/logs"
    
    # Map mode names for compatibility
    case "$MODE" in
        follow|auto)
            CONTAINER_MODE="all"
            ;;
        remote|manual)
            CONTAINER_MODE="remote"
            ;;
        *)
            CONTAINER_MODE="$MODE"
            ;;
    esac
    
    # Start container with this script
    docker run -d --name "$CONTAINER_NAME" \
        --network=host \
        --device="$I2C_DEVICE" \
        --device="$VIDEO_DEVICE" \
        --privileged \
        -v "$REPO_ROOT:/workspace" \
        -w /workspace \
        "$DOCKER_IMAGE" \
        bash -c "/workspace/wyzecar.sh container-start $CONTAINER_MODE"
    
    # Wait and check
    print_info "Waiting for system to start..."
    sleep 5
    
    if docker ps | grep -q "$CONTAINER_NAME"; then
        print_success "WYZECAR started successfully!"
        echo ""
        echo "  View at: http://$(hostname -I | awk '{print $1}'):8080"
        echo "  Logs: ./wyzecar.sh logs"
        echo "  Stop: ./wyzecar.sh stop"
    else
        print_error "Failed to start WYZECAR"
        docker logs "$CONTAINER_NAME" 2>&1 | tail -20
        exit 1
    fi
}

host_stop() {
    print_info "Stopping WYZECAR..."
    
    if docker ps | grep -q "$CONTAINER_NAME"; then
        docker stop "$CONTAINER_NAME" >/dev/null 2>&1
        docker rm "$CONTAINER_NAME" >/dev/null 2>&1
        print_success "WYZECAR stopped"
    else
        print_info "WYZECAR is not running"
    fi
}

host_status() {
    print_banner
    
    echo "Container Status:"
    if docker ps | grep -q "$CONTAINER_NAME"; then
        print_success "WYZECAR is running"
        echo ""
        docker ps --filter "name=$CONTAINER_NAME" --format "table {{.Status}}\t{{.Ports}}"
        
        echo ""
        echo "ROS2 Topics:"
        docker exec "$CONTAINER_NAME" bash -c "source /opt/ros/humble/setup.bash && ros2 topic list" 2>/dev/null || echo "  Unable to list topics"
        
        echo ""
        echo "Latest logs:"
        docker logs "$CONTAINER_NAME" 2>&1 | tail -5
    else
        print_error "WYZECAR is not running"
    fi
    
    echo ""
    echo "System Info:"
    echo "  Dart IP: $(hostname -I | awk '{print $1}')"
    echo "  Web UI: http://$(hostname -I | awk '{print $1}'):8080"
}

host_logs() {
    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        print_error "WYZECAR is not running"
        exit 1
    fi
    
    print_info "Showing live logs (Ctrl+C to exit)..."
    docker logs -f "$CONTAINER_NAME"
}

show_help() {
    print_banner
    echo "Usage: $0 [command] [options]"
    echo ""
    echo "Commands:"
    echo "  start [mode]    Start WYZECAR (default: remote)"
    echo "                  Modes: remote, follow, all, nodetect"
    echo "  stop            Stop WYZECAR"
    echo "  restart [mode]  Restart WYZECAR"
    echo "  status          Show system status"
    echo "  logs            View live logs"
    echo "  update          Pull latest code from GitHub"
    echo "  help            Show this help message"
    echo ""
    echo "Quick Start:"
    echo "  $0              Start in remote control mode"
    echo "  $0 follow       Start in autonomous follow mode"
    echo ""
    echo "Examples:"
    echo "  $0 start remote      # WASD keyboard control"
    echo "  $0 start follow      # Autonomous human following"
    echo "  $0 logs              # View system logs"
    echo "  $0 update && $0      # Update and restart"
}

# Main logic
if [ "$IN_CONTAINER" = "true" ]; then
    # Running inside container
    CMD="${1:-all}"
    shift || true
    
    if [ "$CMD" = "container-start" ]; then
        # Special internal command for container startup
        MODE="${1:-all}"
        container_start "$MODE"
    else
        # Legacy compatibility
        container_start "$CMD"
    fi
else
    # Running on host
    check_root
    
    # Parse command
    COMMAND="${1:-start}"
    shift || true
    
    case "$COMMAND" in
        start)
            host_start "$@"
            ;;
        stop)
            host_stop
            ;;
        restart)
            host_stop
            sleep 2
            host_start "$@"
            ;;
        status)
            host_status
            ;;
        logs)
            host_logs
            ;;
        update)
            host_update
            ;;
        help|-h|--help)
            show_help
            ;;
        # Shortcuts
        remote|follow|all|nodetect)
            host_start "$COMMAND"
            ;;
        *)
            print_error "Unknown command: $COMMAND"
            echo "Use '$0 help' for usage information"
            exit 1
            ;;
    esac
fi