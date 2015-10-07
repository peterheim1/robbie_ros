cmake_minimum_required(VERSION 2.8.3)
project(robbie_moveit)

find_package(catkin REQUIRED COMPONENTS actionlib geometry_msgs moveit_commander moveit_msgs moveit_python rbx2_dynamixels rospy tf trajectory_msgs)
catkin_python_setup()

catkin_package( CATKIN_DEPENDS actionlib geometry_msgs moveit_commander moveit_msgs moveit_python rbx2_dynamixels rospy tf trajectory_msgs)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})