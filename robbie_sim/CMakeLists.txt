cmake_minimum_required(VERSION 2.8.3)
project(robbie_sim)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  geometry_msgs
  moveit_commander
  moveit_msgs
  roscpp
  rospy
  std_msgs
  tf
  trajectory_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
 #  LIBRARIES robbie_sim
 #  CATKIN_DEPENDS roscpp rospy std_msgs
 #  DEPENDS system_lib
 CATKIN_DEPENDS actionlib actionlib_msgs control_msgs geometry_msgs moveit_commander moveit_msgs rospy std_msgs tf trajectory_msgs
)

 
