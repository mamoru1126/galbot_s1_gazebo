# ROS 2 Jazzy + Gazebo Harmonic
#   docker build -f docker/jazzy/Dockerfile -t galbot_s1_gazebo:jazzy .
FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz \
      ros-jazzy-gz-ros2-control \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-rosbridge-server \
      ros-jazzy-xacro \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher-gui \
      python3-numpy \
      python3-yaml \
      python3-opencv \
      python3-pytest \
      xvfb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . /ws/src/galbot_s1_gazebo
RUN rm -rf /ws/src/galbot_s1_gazebo/docker /ws/src/galbot_s1_gazebo/.github \
 && . /opt/ros/jazzy/setup.sh \
 && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "galbot_s1_gazebo", "sim.launch.py"]
