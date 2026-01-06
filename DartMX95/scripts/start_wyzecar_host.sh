#!/usr/bin/env bash
# Host-side launcher for WYZECAR (run on DART-MX95 outside Docker)
#
# Usage (on DART host):
#   sudo -i
#   /root/WYZECAR/DartMX95/scripts/start_wyzecar_host.sh [remote|all|nodetect|...]
#
# Optional env overrides:
#   VIDEO_DEVICE=/dev/video13
#   I2C_DEVICE=/dev/i2c-3
#   IMAGE=wyzecar:humble
#
# This script:
# - Finds the repo root reliably (works from any CWD)
# - Verifies required devices exist
# - Starts the Docker container with the correct mounts/devices
# - Runs the in-container startup script: /workspace/DartMX95/scripts/start_wyzecar.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

IMAGE="${IMAGE:-wyzecar:humble}"
I2C_DEVICE="${I2C_DEVICE:-/dev/i2c-3}"
VIDEO_DEVICE="${VIDEO_DEVICE:-/dev/video13}"

if [[ ! -e "$VIDEO_DEVICE" ]]; then
  # Best-effort fallback: pick the first available /dev/video*
  for d in /dev/video*; do
    if [[ -e "$d" ]]; then
      VIDEO_DEVICE="$d"
      break
    fi
  done
fi

echo ""
echo "╔════════════════════════════════════════════╗"
echo "║     WYZECAR Host Launcher (DART-MX95)      ║"
echo "╚════════════════════════════════════════════╝"
echo ""
echo "Repo:   $REPO_ROOT"
echo "Image:  $IMAGE"
echo "I2C:    $I2C_DEVICE"
echo "Video:  $VIDEO_DEVICE"
echo ""

if [[ ! -d "$REPO_ROOT/DartMX95" ]]; then
  echo "ERROR: Repo root does not look like WYZECAR (missing DartMX95/)."
  exit 1
fi

if [[ ! -e "$I2C_DEVICE" ]]; then
  echo "ERROR: I2C device not found: $I2C_DEVICE"
  echo "Hint: try: ls -l /dev/i2c-*"
  exit 1
fi

if [[ ! -e "$VIDEO_DEVICE" ]]; then
  echo "ERROR: Video device not found: $VIDEO_DEVICE"
  echo "Hint: try: ls -l /dev/video*"
  exit 1
fi

mkdir -p "$REPO_ROOT/logs"

echo "Starting container..."
echo "Press Ctrl+C to stop."
echo ""

# Forward any args to the in-container script (defaults to "all" if none).
MODE_ARGS=("$@")
if [[ ${#MODE_ARGS[@]} -eq 0 ]]; then
  MODE_ARGS=("all")
fi

# Build a safely-quoted argument string for bash -lc.
MODE_ARGS_Q=""
for a in "${MODE_ARGS[@]}"; do
  MODE_ARGS_Q+=" $(printf '%q' "$a")"
done

# If you get 'name already in use', remove the old container.
docker rm -f wyzecar >/dev/null 2>&1 || true

exec docker run --rm -it --name wyzecar \
  --network=host \
  --device="$I2C_DEVICE" \
  --device="$VIDEO_DEVICE" \
  --privileged \
  -v "$REPO_ROOT:/workspace" \
  -w /workspace \
  "$IMAGE" \
  bash -lc "bash /workspace/DartMX95/scripts/start_wyzecar.sh${MODE_ARGS_Q}"


