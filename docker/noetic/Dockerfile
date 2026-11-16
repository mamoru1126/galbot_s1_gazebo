# ROS 1 Noetic + Gazebo Classic 11
#   docker build -f docker/noetic/Dockerfile -t galbot_s1_gazebo:noetic .
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-gazebo-ros-pkgs \
      ros-noetic-gazebo-ros-control \
      ros-noetic-ros-control \
      ros-noetic-ros-controllers \
      ros-noetic-rosbridge-server \
      ros-noetic-xacro \
      ros-noetic-robot-state-publisher \
      ros-noetic-joint-state-publisher-gui \
      python3-numpy \
      python3-yaml \
      python3-opencv \
      python3-pytest \
      xvfb \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . /ws/src/galbot_s1_gazebo
RUN rm -rf /ws/src/galbot_s1_gazebo/docker /ws/src/galbot_s1_gazebo/.github \
 && . /opt/ros/noetic/setup.sh \
 && catkin_make -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "galbot_s1_gazebo", "sim.launch"]
