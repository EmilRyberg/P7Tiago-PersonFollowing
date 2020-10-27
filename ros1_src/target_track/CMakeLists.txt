cmake_minimum_required(VERSION 3.0.2)
project(target_track)

set(OpenCV_DIR /usr/share/OpenCV)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  geometry_msgs
  OpenCV
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES target_track
#  CATKIN_DEPENDS rospy
#  DEPENDS system_lib
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/KCF_tracker.py
  src/RQ_tracker.py
  src/temp2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)