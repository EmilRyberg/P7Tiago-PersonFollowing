cmake_minimum_required(VERSION 3.0.2)
project(nav_node)

find_package(catkin REQUIRED COMPONENTS
  tf2_ros
  geometry_msgs
  rospy
  move_base
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
  src/nav.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)