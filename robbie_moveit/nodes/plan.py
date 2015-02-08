##!/usr/bin/env python


import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    group = MoveGroupCommander("right_arm")
  
    # move to a random target
    group.set_random_target()
    group.go()


    rospy.spin()
    roscpp_shutdown()

