#!/usr/bin/env bash
set -euo pipefail
PROJECT="/home/sed/Desktop/concept-vla"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# If the terminal was not ROS-sourced, try the common installed distros.
if ! command -v ros2 >/dev/null 2>&1; then
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
  elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
  fi
fi

if [[ -f "${PROJECT}/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "${PROJECT}/install/setup.bash"
fi

exec python3 "${SCRIPT_DIR}/flight_batch_ui.py" "$@"
