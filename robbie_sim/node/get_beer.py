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
from actionlib import SimpleActionClient
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *

GROUP_NAME_ARM = 'right_arm'
GROUP_NAME_GRIPPER = 'right_gripper'

GRIPPER_FRAME = 'right_gripper_link'

GRIPPER_OPEN = [0.1]
GRIPPER_CLOSED = [0.5]
GRIPPER_NEUTRAL = [0.0]

GRIPPER_JOINT_NAMES = ['right_arm_gripper_joint']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'map'

#Map location of targets
X_FRIDGE = 5.027
Y_FRIDGE = -5829
X_PERSON = 1.9
Y_PERSON = 1.8

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_demo')
        
        # Use the planning scene object to add or remove objects
        scene = PlanningSceneInterface()
        
        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene)
        
        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped)
        
        # Create a dictionary to hold object colors
        self.colors = dict()
        
        #move_base action
        self.fridge = (Pose(Point(X_FRIDGE, Y_FRIDGE, 0.0), Quaternion(0.0, 0.0,  0, 1))) #location of the beer
        self.person = (Pose(Point(X_PERSON, Y_PERSON, 0.0), Quaternion(0.0, 0.0,  0, 1))) #person requesting the beer
        self.station = (Pose(Point(0.5, 0.0, 0.0), Quaternion(0.0, 0.0,  0, 1))) #person requesting the beer
        self.client = SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
                        
        # Initialize the move group for the right arm
        right_arm = MoveGroupCommander(GROUP_NAME_ARM)
        left_arm = MoveGroupCommander('left_arm')
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
        
        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 10
        
        # Set a limit on the number of place attempts
        max_place_attempts = 5
                
        # Give the scene a chance to catch up
        rospy.sleep(2)

        # Give each of the scene objects a unique name        
        table_id = 'table'
        box1_id = 'box1'
        box2_id = 'box2'
        target_id = 'target'
        tool_id = 'tool'
        person1_id = 'person1'
                
        # Remove leftover objects from a previous run
        scene.remove_world_object(table_id)
        scene.remove_world_object(box1_id)
        scene.remove_world_object(box2_id)
        scene.remove_world_object(target_id)
        scene.remove_world_object(tool_id)
        scene.remove_world_object(person1_id)
        
        # Remove any attached objects from a previous session
        scene.remove_attached_object(GRIPPER_FRAME, target_id)
        
        # Give the scene a chance to catch up    
        rospy.sleep(1)
        
        # Start the arm in the "resting" pose stored in the SRDF file
        right_arm.set_named_target('right_start')
        right_arm.go()

        left_arm.set_named_target('left_start')
        left_arm.go()
        
        # Open the gripper to the neutral position
        right_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        right_gripper.go()
       
        rospy.sleep(1)

        # Set the height of the table off the ground
        table_ground = 0.65
        
        # Set the dimensions of the scene objects [l, w, h]
        table_size = [0.2, 0.7, 0.01]
        box1_size = [0.1, 0.05, 0.05]
        box2_size = [0.05, 0.05, 0.15]
        person1_size = [0.3, 0.7, 0.01]
        
        # Set the target size [l, w, h]
        target_size = [0.02, 0.01, 0.12]
        
        # Add a table top and two boxes to the scene
        table_pose = PoseStamped()
        table_pose.header.frame_id = REFERENCE_FRAME
        table_pose.pose.position.x = X_FRIDGE + 0.55
        table_pose.pose.position.y = Y_FRIDGE + 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)
        
        box1_pose = PoseStamped()
        box1_pose.header.frame_id = REFERENCE_FRAME
        box1_pose.pose.position.x = X_FRIDGE + 0.55
        box1_pose.pose.position.y = Y_FRIDGE + -0.1
        box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
        box1_pose.pose.orientation.w = 1.0   
        scene.add_box(box1_id, box1_pose, box1_size)
        
        box2_pose = PoseStamped()
        box2_pose.header.frame_id = REFERENCE_FRAME
        box2_pose.pose.position.x = X_FRIDGE + 0.54
        box2_pose.pose.position.y = Y_FRIDGE + 0.13
        box2_pose.pose.position.z = table_ground + table_size[2] + box2_size[2] / 2.0
        box2_pose.pose.orientation.w = 1.0   
        scene.add_box(box2_id, box2_pose, box2_size) 

        #add the person to the scene
        person1_pose = PoseStamped()
        person1_pose.header.frame_id = REFERENCE_FRAME
        person1_pose.pose.position.x = X_PERSON + 0.54
        person1_pose.pose.position.y = Y_PERSON + 0.13
        person1_pose.pose.position.z = table_ground + table_size[2] + person1_size[2] / 2.0
        person1_pose.pose.orientation.w = 1.0   
        scene.add_box(person1_id, person1_pose, person1_size)       
        
        # Set the target pose in between the boxes and on the table
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        target_pose.pose.position.x = X_FRIDGE + 0.50
        target_pose.pose.position.y = Y_FRIDGE + 0.0
        target_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
        target_pose.pose.orientation.w = 1.0
        
        # Add the target object to the scene
        scene.add_box(target_id, target_pose, target_size)
        
        # Make the table red and the boxes orange
        self.setColor(table_id, 0.8, 0, 0, 1.0)
        self.setColor(box1_id, 0.8, 0.4, 0, 1.0)
        self.setColor(box2_id, 0.8, 0.4, 0, 1.0)
        self.setColor(person1_id, 0.8, 0, 0, 1.0)
        
        # Make the target yellow
        self.setColor(target_id, 0.9, 0.9, 0, 1.0)
        
        # Send the colors to the planning scene
        self.sendColors()
        
        # Set the support surface name to the table object
        right_arm.set_support_surface_name(table_id)
        
        # Specify a pose to place the target after being picked up
        place_pose = PoseStamped()
        place_pose.header.frame_id = REFERENCE_FRAME
        place_pose.pose.position.x = X_PERSON + 0.50
        place_pose.pose.position.y = Y_PERSON + -0.25
        place_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
        place_pose.pose.orientation.w = 1.0

        #move to target#########################################################################################move to target
        self.move_to(self.fridge)

        # Initialize the grasp pose to the target pose
        grasp_pose = target_pose
        
        # Shift the grasp pose by half the width of the target to center it
        grasp_pose.pose.position.y -= target_size[1] / 2.0
                
        # Generate a list of grasps
        grasps = self.make_grasps(grasp_pose, [target_id])

        # Publish the grasp poses so they can be viewed in RViz
        for grasp in grasps:
            self.gripper_pose_pub.publish(grasp.grasp_pose)
            rospy.sleep(0.2)
    
        # Track success/failure and number of attempts for pick operation
        result = None
        n_attempts = 0
        
        # Repeat until we succeed or run out of attempts
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
            n_attempts += 1
            rospy.loginfo("Pick attempt: " +  str(n_attempts))
            result = right_arm.pick(target_id, grasps)
            rospy.sleep(0.2)
        
        # If the pick was successful, attempt the place operation   
        if result == MoveItErrorCodes.SUCCESS:
            result = None
            n_attempts = 0
            #_------------------------now we move to the other table__________-------------------------------------------
            right_arm.set_named_target('r_travel')
            right_arm.go()
            self.move_to(self.person)
            
            #_------------------------now we move to the other table__________-------------------------------------------
            # Generate valid place poses
            places = self.make_places(place_pose)
            
            # Repeat until we succeed or run out of attempts
            while result != MoveItErrorCodes.SUCCESS and n_attempts < max_place_attempts:
                n_attempts += 1
                rospy.loginfo("Place attempt: " +  str(n_attempts))
                for place in places:
                    result = right_arm.place(target_id, place)
                    if result == MoveItErrorCodes.SUCCESS:
                        break
                rospy.sleep(0.2)
                
            if result != MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Place operation failed after " + str(n_attempts) + " attempts.")
        else:
            rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")
                
        # Return the arm to the "resting" pose stored in the SRDF file
        right_arm.set_named_target('right_start')
        right_arm.go()
        
        # Open the gripper to the neutral position
        right_gripper.set_joint_value_target(GRIPPER_NEUTRAL)
        right_gripper.go()
       
        rospy.sleep(1)
        #move to station
        self.move_to(self.station)

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)

    # move to location
    def move_to(self, location):
        goal = MoveBaseGoal()
        goal.target_pose.pose = location
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
  
        self.client.send_goal(goal)
        #self.client.wait_for_result()
        self.client.wait_for_result(rospy.Duration.from_sec(40.0))
        

        if self.client.get_state() == GoalStatus.SUCCEEDED:
            result = self.client.get_result()
            print "Result: SUCCEEDED " 
        elif self.client.get_state() == GoalStatus.PREEMPTED:
            print "Action pre-empted"
        else:
            print "Action failed"
        
    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, joint_positions):
        # Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()
        
        # Set the joint names to the gripper joint names
        t.joint_names = GRIPPER_JOINT_NAMES
        
        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()
        
        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions
        
        # Set the gripper effort
        tp.effort = GRIPPER_EFFORT
        
        tp.time_from_start = rospy.Duration(1.0)
        
        # Append the goal point to the trajectory points
        t.points.append(tp)
        
        # Return the joint trajectory
        return t
    
    # Generate a gripper translation in the direction given by vector
    def make_gripper_translation(self, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()
        
        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]
        
        # The vector is relative to the gripper frame
        g.direction.header.frame_id = GRIPPER_FRAME
        
        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired
        
        return g

    # Generate a list of possible grasps
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
        # Initialize the grasp object
        g = Grasp()
        
        # Set the pre-grasp and grasp postures appropriately
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_CLOSED)
                
        # Set the approach and retreat parameters as desired
        g.pre_grasp_approach = self.make_gripper_translation(0.01, 0.1, [1.0, 0.0, 0.0])
        g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, [0.0, -1.0, 1.0])
        
        # Set the first grasp pose to the input pose
        g.grasp_pose = initial_pose_stamped
    
        # Pitch angles to try
        pitch_vals = [0, 0.1, -0.1, 0.2, -0.2, 0.3, -0.3]
        
        # Yaw angles to try
        yaw_vals = [0]

        # A list to hold the grasps
        grasps = []

        # Generate a grasp for each pitch and yaw angle
        for y in yaw_vals:
            for p in pitch_vals:
                # Create a quaternion from the Euler angles
                q = quaternion_from_euler(0, p, y)
                
                # Set the grasp pose orientation accordingly
                g.grasp_pose.pose.orientation.x = q[0]
                g.grasp_pose.pose.orientation.y = q[1]
                g.grasp_pose.pose.orientation.z = q[2]
                g.grasp_pose.pose.orientation.w = q[3]
                
                # Set and id for this grasp (simply needs to be unique)
                g.id = str(len(grasps))
                
                # Set the allowed touch objects to the input list
                g.allowed_touch_objects = allowed_touch_objects
                
                # Don't restrict contact force
                g.max_contact_force = 0
                
                # Degrade grasp quality for increasing pitch angles
                g.grasp_quality = 1.0 - abs(p)
                
                # Append the grasp to the list
                grasps.append(deepcopy(g))
                
        # Return the list
        return grasps
    
    # Generate a list of possible place poses
    def make_places(self, init_pose):
        # Initialize the place location as a PoseStamped message
        place = PoseStamped()
        
        # Start with the input place pose
        place = init_pose
        
        # A list of x shifts (meters) to try
        x_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
        
        # A list of y shifts (meters) to try
        y_vals = [0, 0.005, 0.01, 0.015, -0.005, -0.01, -0.015]
        
        pitch_vals = [0]
        
        # A list of yaw angles to try
        yaw_vals = [0]

        # A list to hold the places
        places = []
        
        # Generate a place pose for each angle and translation
        for y in yaw_vals:
            for p in pitch_vals:
                for y in y_vals:
                    for x in x_vals:
                        place.pose.position.x = init_pose.pose.position.x + x
                        place.pose.position.y = init_pose.pose.position.y + y
                        
                        # Create a quaternion from the Euler angles
                        q = quaternion_from_euler(0, p, y)
                        
                        # Set the place pose orientation accordingly
                        place.pose.orientation.x = q[0]
                        place.pose.orientation.y = q[1]
                        place.pose.orientation.z = q[2]
                        place.pose.orientation.w = q[3]
                        
                        # Append this place pose to the list
                        places.append(deepcopy(place))
        
        # Return the list
        return places
    
    # Set the color of an object
    def setColor(self, name, r, g, b, a = 0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()
        
        # Set the id to the name given as an argument
        color.id = name
        
        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        
        # Update the global color dictionary
        self.colors[name] = color

    # Actually send the colors to MoveIt!
    def sendColors(self):
        # Initialize a planning scene object
        p = PlanningScene()

        # Need to publish a planning scene diff        
        p.is_diff = True
        
        # Append the colors from the global color dictionary 
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # Publish the scene diff
        self.scene_pub.publish(p)

if __name__ == "__main__":
    MoveItDemo()

    
