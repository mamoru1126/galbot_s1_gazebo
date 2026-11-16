#!/usr/bin/env bash
# Run the Galbot S1 simulation in Docker.
#
#   ./scripts/docker_run.sh jazzy                 # Gazebo GUI + web UI (http://localhost:8000)
#   ./scripts/docker_run.sh noetic
#   ./scripts/docker_run.sh jazzy --headless      # no Gazebo window, web UI only
#   ./scripts/docker_run.sh jazzy --no-gpu        # do not pass the NVIDIA GPU
#   ./scripts/docker_run.sh jazzy -- cameras:=false   # extra launch arguments after --
#   ./scripts/docker_run.sh jazzy --shell         # bash inside the container
set -euo pipefail
DISTRO="${1:-jazzy}"
shift || true
HEADLESS=false
GPU=auto
SHELL_MODE=false
LAUNCH_ARGS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --headless) HEADLESS=true ;;
    --no-gpu) GPU=no ;;
    --gpu) GPU=yes ;;
    --shell) SHELL_MODE=true ;;
    --) shift; LAUNCH_ARGS+=("$@"); break ;;
    *) LAUNCH_ARGS+=("$1") ;;
  esac
  shift
done

IMAGE="galbot_s1_gazebo:$DISTRO"
ARGS=(--rm -it --net=host --ipc=host --name "galbot_s1_$DISTRO" -e QT_X11_NO_MITSHM=1)

if [ "$HEADLESS" = false ]; then
  if [ -z "${DISPLAY:-}" ]; then
    echo "DISPLAY is not set - starting headless (use a desktop session for the Gazebo GUI)"
    HEADLESS=true
  else
    xhost +local:docker >/dev/null 2>&1 || true
    ARGS+=(-e "DISPLAY=$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
  fi
fi

if [ "$GPU" = yes ] || { [ "$GPU" = auto ] && docker info 2>/dev/null | grep -qi nvidia; }; then
  ARGS+=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all)
fi

GUI_ARG="gui:=true"
if [ "$HEADLESS" = true ]; then GUI_ARG="gui:=false"; fi

if [ "$SHELL_MODE" = true ]; then
  CMD=(bash)
elif [ "$DISTRO" = noetic ]; then
  CMD=(roslaunch galbot_s1_gazebo sim.launch "$GUI_ARG" "${LAUNCH_ARGS[@]}")
else
  CMD=(ros2 launch galbot_s1_gazebo sim.launch.py "$GUI_ARG" "${LAUNCH_ARGS[@]}")
fi

echo "web UI: http://localhost:8000  (from another device: http://$(hostname -I 2>/dev/null | awk '{print $1}'):8000)"
exec docker run "${ARGS[@]}" "$IMAGE" "${CMD[@]}"
