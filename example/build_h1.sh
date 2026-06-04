#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

PACKAGE_NAME="topstar_ros2_example"
ROS_SETUP="/opt/ros/humble/setup.bash"
CYCLONEDDS_SETUP="$HOME/topstar_ros2/cyclonedds_ws/install/setup.bash"

if ! command -v colcon >/dev/null 2>&1; then
  echo "Error: colcon is not installed or not in PATH." >&2
  exit 1
fi

if [ ! -f "$ROS_SETUP" ]; then
  echo "Error: ROS 2 Humble setup not found at $ROS_SETUP." >&2
  exit 1
fi

if [ ! -f "$CYCLONEDDS_SETUP" ]; then
  echo "Error: CycloneDDS workspace setup not found at $CYCLONEDDS_SETUP." >&2
  exit 1
fi

set +u
source "$ROS_SETUP"
source "$CYCLONEDDS_SETUP"
set -u

echo "Building H1-only package: ${PACKAGE_NAME}"

# Default to symlink install for faster iteration; pass additional flags as args.
colcon build --packages-select "$PACKAGE_NAME" --symlink-install "$@"
