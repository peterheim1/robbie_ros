#!/usr/bin/env python


import sys
import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint


if __name__=='__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    right_arm = MoveGroupCommander("right_arm")
    rospy.sleep(1)

    # clean the scene
    scene.remove_world_object("table")
    scene.remove_world_object("part")
    rospy.logwarn("cleaning world")
    #right_arm.set_named_target("r_start")
    #right_arm.go()
   
    #right_gripper.set_named_target("open")
    #right_gripper.go()
   
    rospy.sleep(3)

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()

    # add a table
    p.pose.position.x = 1.0
    p.pose.position.y = 0.2
    p.pose.position.z = 0.3
    scene.add_box("table", p, (0.7, 1, 0.7))

    # add an object to be grasped
    p.pose.position.x = 0.7
    p.pose.position.y = -0.2
    p.pose.position.z = 0.85
    scene.add_box("part", p, (0.07, 0.01, 0.2))

  
    # move to a random target
    #right_arm.set_named_target("r_start")
    #right_arm.go()
    #rospy.sleep(1)

    #right_arm.set_random_target()
    #right_arm.go()
    #rospy.sleep(1)

   
    #right_arm.set_position_target([.75,-0.3, 1])
    #right_arm.go()
    #rospy.sleep(1)

    grasps = []
   
    g = Grasp()
    g.id = "test"
    grasp_pose = PoseStamped()
    grasp_pose.header.frame_id = "base_link"
    grasp_pose.pose.position.x = 0.47636
    grasp_pose.pose.position.y = -0.21886
    grasp_pose.pose.position.z = 0.7164
    grasp_pose.pose.orientation.x = 0.00080331
    grasp_pose.pose.orientation.y = 0.001589
    grasp_pose.pose.orientation.z = -2.4165e-06
    grasp_pose.pose.orientation.w = 1

    rospy.logwarn("moving to arm")
    right_arm.set_pose_target(grasp_pose)
    right_arm.go()
    rospy.sleep(3)
   
    

    rospy.sleep(2)
    rospy.logwarn("pick part")
    # pick the object
    robot.right_arm.pick("part")
   


    

    rospy.spin()
    roscpp_shutdown()
