#!/usr/bin/env bash
# Runs inside the container: xacro -> URDF for both simulators, then pytest.
set -eu
SRC=/ws/src/galbot_s1_gazebo
OUT=/tmp/galbot_urdf
mkdir -p "$OUT"
if [ "${ROS_VERSION}" = "1" ]; then
  GZ_PKG="$(rospack find galbot_s1_gazebo)"
  DESC_PKG="$(rospack find galbot_s1_description)"
else
  GZ_PKG="$(ros2 pkg prefix galbot_s1_gazebo)/share/galbot_s1_gazebo"
  DESC_PKG="$(ros2 pkg prefix galbot_s1_description)/share/galbot_s1_description"
fi
xacro "$GZ_PKG/urdf/galbot_s1_sim.urdf.xacro" sim:=gz > "$OUT/gz.urdf"
xacro "$GZ_PKG/urdf/galbot_s1_sim.urdf.xacro" sim:=classic > "$OUT/classic.urdf"
xacro "$DESC_PKG/urdf/galbot_s1.urdf.xacro" > "$OUT/description.urdf"
if command -v check_urdf >/dev/null 2>&1; then
  check_urdf "$OUT/description.urdf" | head -n 5
fi
export GALBOT_URDF_DIR="$OUT"
export GALBOT_SRC="$SRC"
cd "$SRC/galbot_s1_control"
python3 -m pytest -q -p no:cacheprovider test
