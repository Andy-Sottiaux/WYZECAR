#!/bin/bash
set -e

# Source ROS 2 setup
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

# Source workspace overlay if it exists
if [ -n "${WORKSPACE_DIR}" ] && [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
  source "${WORKSPACE_DIR}/install/setup.bash"
fi

exec "$@"
