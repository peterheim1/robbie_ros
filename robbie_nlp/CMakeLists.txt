cmake_minimum_required(VERSION 2.8.3)
project(robbie_nlp)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)


catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robbie_bringup
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

