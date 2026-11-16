cmake_minimum_required(VERSION 3.10)
project(galbot_s1_gazebo)

# One package for ROS 1 (catkin + Gazebo Classic 11) and ROS 2 (ament + Gazebo Harmonic),
# selected by $ROS_VERSION.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(SHARE_DIRS config launch urdf worlds scripts)

if("$ENV{ROS_VERSION}" STREQUAL "2")
  find_package(ament_cmake REQUIRED)

  # Gazebo Harmonic libraries come from the ROS 2 vendor packages (Jazzy and newer)
  find_package(gz_sim_vendor REQUIRED)
  find_package(gz-sim QUIET)
  if(NOT gz-sim_FOUND)
    find_package(gz-sim8 REQUIRED)
  endif()
  find_package(gz_plugin_vendor REQUIRED)
  find_package(gz-plugin QUIET COMPONENTS register)
  if(NOT gz-plugin_FOUND)
    find_package(gz-plugin2 REQUIRED COMPONENTS register)
  endif()

  if(TARGET gz-sim::gz-sim)
    set(GZ_SIM_TARGET gz-sim::gz-sim)
  else()
    set(GZ_SIM_TARGET gz-sim8::gz-sim8)
  endif()
  if(TARGET gz-plugin::register)
    set(GZ_PLUGIN_TARGET gz-plugin::register)
  else()
    set(GZ_PLUGIN_TARGET gz-plugin2::register)
  endif()

  add_library(galbot_sim_helper_gz SHARED src/gz/sim_helper_system.cpp)
  target_link_libraries(galbot_sim_helper_gz ${GZ_SIM_TARGET} ${GZ_PLUGIN_TARGET})
  install(TARGETS galbot_sim_helper_gz LIBRARY DESTINATION lib)

  install(DIRECTORY ${SHARE_DIRS} DESTINATION share/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)
  ament_package()
else()
  find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs gazebo_ros)
  find_package(gazebo REQUIRED)
  catkin_package()

  include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
  link_directories(${GAZEBO_LIBRARY_DIRS})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

  add_library(galbot_planar_base_classic SHARED src/classic/planar_base_plugin.cpp)
  target_link_libraries(galbot_planar_base_classic ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
  add_library(galbot_sim_helper_classic SHARED src/classic/sim_helper_plugin.cpp)
  target_link_libraries(galbot_sim_helper_classic ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

  install(TARGETS galbot_planar_base_classic galbot_sim_helper_classic
    LIBRARY DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION})
  install(DIRECTORY ${SHARE_DIRS} DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
endif()
