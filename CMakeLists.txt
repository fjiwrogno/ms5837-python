cmake_minimum_required(VERSION 3.0.2)
project(ms5837_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/ms5837_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
