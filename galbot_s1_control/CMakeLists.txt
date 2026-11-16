cmake_minimum_required(VERSION 3.5)
project(galbot_s1_control)

# Same Python nodes for ROS 1 (catkin) and ROS 2 (ament_cmake), selected by $ROS_VERSION.
set(NODES scripts/commander.py scripts/base_controller.py scripts/camera_relay.py)

if("$ENV{ROS_VERSION}" STREQUAL "2")
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  ament_python_install_package(${PROJECT_NAME})
  install(PROGRAMS ${NODES} DESTINATION lib/${PROJECT_NAME})
  ament_package()
else()
  find_package(catkin REQUIRED)
  catkin_python_setup()
  catkin_package()
  catkin_install_python(PROGRAMS ${NODES} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
endif()
