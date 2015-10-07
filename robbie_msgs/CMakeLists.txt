cmake_minimum_required(VERSION 2.8.3)
project(robbie_msgs)
find_package(catkin REQUIRED COMPONENTS actionlib actionlib_msgs message_generation roscpp rospy)

catkin_python_setup()

## Generate services in the 'srv' folder
add_service_files(FILES
   Parse.srv
   AutoDock.srv
 )

add_message_files(FILES
                  RobbieCmd.msg
                  
                 )

generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs message_runtime)
