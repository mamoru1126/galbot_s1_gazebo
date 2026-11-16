#!/usr/bin/env bash
# Build the simulation image.
#   ./scripts/docker_build.sh jazzy     # ROS 2 Jazzy + Gazebo Harmonic (default)
#   ./scripts/docker_build.sh noetic    # ROS 1 Noetic + Gazebo Classic 11
set -euo pipefail
DISTRO="${1:-jazzy}"
case "$DISTRO" in
  jazzy|noetic) ;;
  *) echo "usage: $0 [jazzy|noetic]" >&2; exit 1 ;;
esac
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
docker build -f "$ROOT/docker/$DISTRO/Dockerfile" -t "galbot_s1_gazebo:$DISTRO" "$ROOT"
echo "built galbot_s1_gazebo:$DISTRO"
