#!/usr/bin/env bash
# Source ROS + the workspace, start a virtual display when there is none
# (Gazebo Classic cameras need an X server even without the GUI).
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f /ws/install/setup.bash ]; then source /ws/install/setup.bash; fi
if [ -f /ws/devel/setup.bash ]; then source /ws/devel/setup.bash; fi
if [ -z "${DISPLAY:-}" ] && command -v Xvfb >/dev/null 2>&1; then
  Xvfb :99 -screen 0 1280x1024x24 -nolisten tcp >/dev/null 2>&1 &
  export DISPLAY=:99
fi
exec "$@"
