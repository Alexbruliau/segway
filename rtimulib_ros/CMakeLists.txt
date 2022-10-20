cmake_minimum_required(VERSION 2.8.3)
project(rtimulib_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
    LIBRARIES rtimulib_ros
    CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rtimulib_ros src/rtimulib_ros.cpp)

target_link_libraries(rtimulib_ros
  ${catkin_LIBRARIES}
  RTIMULib
)

## Mark executables and/or libraries for installation
 install(TARGETS rtimulib_ros
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

 ## Mark other files for installation (e.g. launch and bag files, etc.)
 install(FILES
   launch/rtimulib_ros.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
 )

 install(FILES
   config/RTIMULib.ini
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
 )
