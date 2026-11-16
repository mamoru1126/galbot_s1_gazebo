cmake_minimum_required(VERSION 3.5)
project(galbot_s1_description)

# One package for ROS 1 (catkin) and ROS 2 (ament_cmake), selected by $ROS_VERSION.
if("$ENV{ROS_VERSION}" STREQUAL "2")
  find_package(ament_cmake REQUIRED)
  install(DIRECTORY meshes urdf launch rviz DESTINATION share/${PROJECT_NAME})
  ament_package()
else()
  find_package(catkin REQUIRED)
  catkin_package()
  install(DIRECTORY meshes urdf launch rviz
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endif()
