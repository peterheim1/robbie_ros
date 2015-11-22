#!/usr/bin/env python

"""
    get_beer.py - Version 0.1 2015-03-11
    
    pick up the beer can and deliver it another 
"""

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler
from copy import deepcopy

GROUP_NAME_ARM = 'right_arm'
GROUP_NAME_GRIPPER = 'right_gripper'

GRIPPER_FRAME = 'right_gripper_link'

GRIPPER_OPEN = [0.1]
GRIPPER_CLOSED = [0.5]
GRIPPER_NEUTRAL = [0.0]

GRIPPER_JOINT_NAMES = ['right_arm_gripper_joint']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'odom'

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('start_pos')
        
        # Use the planning scene object to add or remove objects
        scene = PlanningSceneInterface()
        
        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size = 5)
        
        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size = 5)
        
        # Create a dictionary to hold object colors
        self.colors = dict()
                        
        # Initialize the move group for the right arm
        right_arm = MoveGroupCommander(GROUP_NAME_ARM)
        
        # Initialize the move group for the right gripper
        right_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
 
        # Allow some leeway in position (meters) and orientation (radians)
        right_arm.set_goal_position_tolerance(0.05)
        right_arm.set_goal_orientation_tolerance(0.1)

        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(True)
        
        # Set the right arm reference frame
        right_arm.set_pose_reference_frame(REFERENCE_FRAME)
        
        # Allow 10 seconds per planning attempt
        right_arm.set_planning_time(10)
        
       
       
        # Start the arm in the "resting" pose stored in the SRDF file
        right_arm.set_named_target('right_start')
        right_arm.go()
        
        # Open the gripper to the neutral position
        right_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        right_gripper.go()
       
        

        rospy.sleep(1)

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)
        
    

if __name__ == "__main__":
    MoveItDemo()

    
