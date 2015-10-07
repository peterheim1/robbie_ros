cmake_minimum_required(VERSION 2.8.3)
project(robbie_test)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  geometry_msgs
  moveit_commander
  nav_msgs
  rbx2_msgs
  rbx2_tasks
  roscpp
  rospy
  sensor_msgs
  smach
  smach_ros
  std_msgs
  tf
  trajectory_msgs
  visualization_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
 #  LIBRARIES robbie_test
 #  CATKIN_DEPENDS roscpp rospy std_msgs
 #  DEPENDS system_lib
 CATKIN_DEPENDS actionlib actionlib_msgs control_msgs geometry_msgs moveit_commander nav_msgs rbx2_msgs rbx2_tasks rospy sensor_msgs smach smach_ros std_msgs tf trajectory_msgs visualization_msgs
)

