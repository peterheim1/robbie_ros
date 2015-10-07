cmake_minimum_required(VERSION 2.8.3)
project(robbie_bringup)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  geometry_msgs
  nav_msgs
  robbie_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_python_setup()

#   std_msgs

catkin_package(
#  INCLUDE_DIRS include
 #  LIBRARIES robbie_bringup
 #  CATKIN_DEPENDS roscpp rospy std_msgs
 #  DEPENDS system_lib
 CATKIN_DEPENDS control_msgs geometry_msgs nav_msgs robbie_msgs rospy sensor_msgs std_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
