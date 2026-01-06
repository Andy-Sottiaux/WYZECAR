#!/bin/bash
set -e

# Set default ROS_DISTRO if not set
: ${ROS_DISTRO:=humble}

# Source ROS 2 setup with unbound variable protection
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  set +u  # Temporarily disable unbound variable checking
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u  # Re-enable unbound variable checking
fi

# Source workspace overlay if it exists
if [ -n "${WORKSPACE_DIR}" ] && [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
fi

exec "$@"
