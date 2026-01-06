#!/usr/bin/env bash
#
# WYZECAR - Unified Startup Script
# 
# This is the main entry point for running WYZECAR on the DART-MX95.
# Simply SSH into your Dart and run: ./wyzecar.sh
#
# Usage:
#   ./wyzecar.sh              # Start in remote control mode (default)
#   ./wyzecar.sh follow       # Start in autonomous follow mode  
#   ./wyzecar.sh stop         # Stop WYZECAR
#   ./wyzecar.sh status       # Show system status
#   ./wyzecar.sh logs         # View live logs
#   ./wyzecar.sh update       # Pull latest code from GitHub
#

set -euo pipefail

# Configuration
REPO_DIR="/root/WYZECAR"
DOCKER_IMAGE="wyzecar:humble"
CONTAINER_NAME="wyzecar"
DEFAULT_MODE="remote"

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
    if [ "$EUID" -ne 0 ]; then
        print_error "Please run as root (use: sudo -i)"
        exit 1
    fi
}

# Update from GitHub
do_update() {
    print_info "Updating WYZECAR from GitHub..."
    cd "$REPO_DIR"
    
    # Check for uncommitted changes
    if ! git diff-index --quiet HEAD --; then
        print_error "Uncommitted changes detected. Please commit or stash first."
        exit 1
    fi
    
    git pull origin main
    print_success "Updated successfully"
    
    # Make scripts executable
    chmod +x "$REPO_DIR/DartMX95/wyzecar.sh"
    chmod +x "$REPO_DIR/DartMX95/scripts/start_wyzecar.sh"
    chmod +x "$REPO_DIR/DartMX95/scripts/start_wyzecar_host.sh"
}

# Start WYZECAR
do_start() {
    local MODE="${1:-$DEFAULT_MODE}"
    
    print_banner
    
    # Stop any running instance
    docker rm -f "$CONTAINER_NAME" >/dev/null 2>&1 || true
    
    # Detect devices
    local I2C_DEVICE="${I2C_DEVICE:-/dev/i2c-3}"
    local VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video13}"
    
    # Auto-detect video device if not found
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
    
    print_info "Starting WYZECAR in ${MODE} mode..."
    print_info "Camera: $VIDEO_DEVICE"
    print_info "I2C: $I2C_DEVICE"
    
    # Create logs directory
    mkdir -p "$REPO_DIR/logs"
    
    # Map mode names
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
    
    # Start container
    docker run -d --name "$CONTAINER_NAME" \
        --network=host \
        --device="$I2C_DEVICE" \
        --device="$VIDEO_DEVICE" \
        --privileged \
        -v "$REPO_DIR:/workspace" \
        -w /workspace \
        "$DOCKER_IMAGE" \
        bash -c "source /opt/ros/humble/setup.bash && bash /workspace/DartMX95/scripts/start_wyzecar.sh $CONTAINER_MODE"
    
    # Wait for startup
    print_info "Waiting for system to start..."
    sleep 5
    
    # Check if running
    if docker ps | grep -q "$CONTAINER_NAME"; then
        print_success "WYZECAR started successfully!"
        echo ""
        echo "═══════════════════════════════════════════════"
        if [[ "$MODE" == "remote" || "$MODE" == "manual" ]]; then
            echo "  🎮 REMOTE CONTROL MODE"
            echo ""
            echo "  Open in browser: http://$(hostname -I | awk '{print $1}'):8080"
            echo "  Use WASD keys to drive!"
        else
            echo "  🤖 AUTONOMOUS FOLLOW MODE"
            echo ""
            echo "  Open in browser: http://$(hostname -I | awk '{print $1}'):8080"
            echo "  Car will follow detected humans"
        fi
        echo ""
        echo "  View logs: ./wyzecar.sh logs"
        echo "  Stop: ./wyzecar.sh stop"
        echo "═══════════════════════════════════════════════"
    else
        print_error "Failed to start WYZECAR"
        echo "Container logs:"
        docker logs "$CONTAINER_NAME" 2>&1 | tail -20
        exit 1
    fi
}

# Stop WYZECAR
do_stop() {
    print_info "Stopping WYZECAR..."
    
    if docker ps | grep -q "$CONTAINER_NAME"; then
        docker stop "$CONTAINER_NAME" >/dev/null 2>&1
        docker rm "$CONTAINER_NAME" >/dev/null 2>&1
        print_success "WYZECAR stopped"
    else
        print_info "WYZECAR is not running"
    fi
}

# Show status
do_status() {
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

# View logs
do_logs() {
    if ! docker ps | grep -q "$CONTAINER_NAME"; then
        print_error "WYZECAR is not running"
        exit 1
    fi
    
    print_info "Showing live logs (Ctrl+C to exit)..."
    docker logs -f "$CONTAINER_NAME"
}

# Show help
do_help() {
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

# Main script logic
check_root

# Change to repo directory
cd "$REPO_DIR/DartMX95" 2>/dev/null || {
    print_error "WYZECAR repository not found at $REPO_DIR"
    print_info "Please clone: git clone https://github.com/Andy-Sottiaux/WYZECAR.git $REPO_DIR"
    exit 1
}

# Parse command
COMMAND="${1:-start}"
shift || true

case "$COMMAND" in
    start)
        do_start "$@"
        ;;
    stop)
        do_stop
        ;;
    restart)
        do_stop
        sleep 2
        do_start "$@"
        ;;
    status)
        do_status
        ;;
    logs)
        do_logs
        ;;
    update)
        do_update
        ;;
    help|-h|--help)
        do_help
        ;;
    # Default: treat as mode for start command
    remote|follow|all|nodetect)
        do_start "$COMMAND"
        ;;
    *)
        print_error "Unknown command: $COMMAND"
        echo "Use '$0 help' for usage information"
        exit 1
        ;;
esac