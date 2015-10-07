cmake_minimum_required(VERSION 2.8.3)
project(robbie_utils)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  diagnostic_msgs
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs
)

catkin_python_setup()

#   std_msgs

catkin_package(
#  INCLUDE_DIRS include
 #  LIBRARIES robbie_utils
 #  CATKIN_DEPENDS roscpp rospy std_msgs
 #  DEPENDS system_lib
 CATKIN_DEPENDS actionlib control_msgs diagnostic_msgs geometry_msgs rospy sensor_msgs std_msgs trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

