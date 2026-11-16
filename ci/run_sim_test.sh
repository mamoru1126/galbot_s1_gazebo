#!/usr/bin/env bash
# Runs inside the simulation container: start the headless simulation, run the
# end-to-end client, collect diagnostics into $OUT.
set -u
OUT="${1:-/ci_out}"
mkdir -p "$OUT"
SRC=/ws/src/galbot_s1_gazebo

if [ "${ROS_VERSION}" = "1" ]; then
  roslaunch galbot_s1_gazebo sim.launch gui:=false > "$OUT/launch.log" 2>&1 &
else
  ros2 launch galbot_s1_gazebo sim.launch.py gui:=false > "$OUT/launch.log" 2>&1 &
fi
LAUNCH_PID=$!

timeout 2400 python3 "$SRC/ci/sim_test_client.py" "$OUT" 2>&1 | tee "$OUT/client.log"
RC=${PIPESTATUS[0]}

{
  echo "==== diagnostics ===="
  if [ "${ROS_VERSION}" = "1" ]; then
    timeout 20 rosnode list
    timeout 20 rostopic list
    timeout 20 rosservice call /controller_manager/list_controllers
  else
    timeout 20 ros2 node list
    timeout 20 ros2 topic list
    timeout 30 ros2 control list_controllers
  fi
} > "$OUT/diagnostics.log" 2>&1

kill -INT "$LAUNCH_PID" 2>/dev/null
sleep 8
pkill -9 -f gzserver 2>/dev/null
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f ruby 2>/dev/null
echo "exit code $RC"
exit "$RC"
