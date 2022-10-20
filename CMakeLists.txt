# toplevel CMakeLists.txt for a catkin workspace
# catkin/cmake/toplevel.cmake

cmake_minimum_required(VERSION 2.8.3)
project(segway)

# Set true in build-host.sh
option(GAZEBO "Enable Gazebo simulation" FALSE)

if (GAZEBO)
  add_definitions(-DGAZEBO)
  add_subdirectory(segway_control)
else(GAZEBO)
  # WARNING NOT WORKING - Must be set in buil-targes.sh
  #add_subdirectory( ./../../library/quimesis_can_utils)
  #add_subdirectory( ./../../quimesis_quimmotor)
  #include_directories( ./../../library/quimesis_can_utils)
  #include_directories( ./../../quimesis_quimmotor)
  add_subdirectory( rtimulib_ros)
  add_subdirectory( filters)
endif(GAZEBO)

add_subdirectory(quimesis_segway_joystick)
add_subdirectory(quimesis_segway_description)
add_subdirectory(quimesis_segway_feedback_controller)
add_subdirectory(quimesis_segway_pipe_pid)

add_subdirectory(print_topics)





## Mark executables and/or libraries for installation
 install(TARGETS
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
