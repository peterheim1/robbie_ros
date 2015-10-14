#!/usr/bin/env python

"""
  interaction.py: 
  helper function for AI interface
  Copyright (c) Escaliente Robotics.  All right reserved.

  todo
  needs a global blackboard

"""

import rospy
import re
import time
import pywapi
import sys
import time
import tf
import math
from datetime import datetime, timedelta
from time import localtime, strftime
from std_msgs.msg import String
from actionlib import SimpleActionClient
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from sound_play.libsoundplay import SoundClient
from datetime import datetime, timedelta
from pi_trees_ros.pi_trees_ros import *
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


from tf import transformations

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class AutoDock(Task):
    def __init__(self, timer=2, *args):
        name = "AUTODOCK_"
        super(AutoDock, self).__init__(name)   
        self.finished = False
        self.counter = timer
        self.name = name
        self._Auto_dock_Publisher = rospy.Publisher('auto_dock', String, queue_size=1)     
    def run(self):
        if self.finished:
            #rospy.loginfo("Docking Success")
            return TaskStatus.SUCCESS
        else:
            while self.counter > 0:
                rospy.loginfo('auto dock start.')
                self._Auto_dock_Publisher.publish("dock")   
                self.counter -= 1
                rospy.sleep(30)
                return TaskStatus.RUNNING    
            message = "Docking is complete"
            rospy.loginfo(message)
            self.finished = True

class NowTime:
    '''
    '''
    def __init__(self):
        
        #define afternoon and morning
        self.noon = strftime("%p:", localtime())
        if self.noon == "AM:":
            self.noon1 = "Goood Morning  "
        else:
            self.noon1 ="Good After Noon   "
        #self.local = strftime("%H:%M:", localtime())
        #local time
        self.local = strftime("%H:%M", localtime())
        self.Now ="The time is   " + str(self.local)
    def __str__(self):
        return str(self.Now)



class Weather:
    """ 
    a class to return the weather from cairns
    """
    def __init__(self):
        
        self.weather_com_result = pywapi.get_weather_from_weather_com('ASXX0020')
        #rospy.sleep(2)
        #return the processed value
        self.value = "It is " + str(self.weather_com_result['current_conditions']['text']) + " and " + self.weather_com_result['current_conditions']['temperature'] + "C now in Cairns.\n\n"
    def __str__(self):
        return str(self.value)

class Move_arm:
    """ 
    a class to move the arms to a pre defined position
    """
    def __init__(self, task):
        #self.task = text#whole message from greeting
        self.right_arm_client = SimpleActionClient('right_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)       
        self.right_arm_client.wait_for_server()
        self.left_arm_client = SimpleActionClient('left_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)       
        self.left_arm_client.wait_for_server()
        right_start  = [0.0, 0, 1.5, 0, 0, -0, -0]
        left_start  = [0.0, 0, 1.5, 0, 0, -0, -0]
        right_bel = [0.0, -0.965024, 1.3841119999999998, -1.52308, -1.31542, 0.0, -0.78029]

        if task =='right_start':
            self.move_right_arm(right_start)
        if task =='left_start':
            self.move_left_arm(left_start)
        
        

    def move_right_arm(self,pose):
        # Which joints define the arm?
        arm_joints = ['pan_joint',
                      'right_arm_tilt_joint',
                      'right_arm_lift_joint',
                      'right_arm_rotate_joint', 
                      'right_arm_elbow_joint',
                      'right_arm_wrist_yaw_joint',
                      'right_arm_wrist_tilt_joint']
        
        
        #if reset:
            # Set the arm back to the resting position
        arm_goal  = pose
      
        # Create a single-point arm trajectory with the arm_goal as the end-point
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = arm_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = arm_goal
        arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
        # Send the trajectory to the arm action server
        rospy.loginfo('Moving the arm to goal position...')
        
        # Create an empty trajectory goal
        arm_goal = FollowJointTrajectoryGoal()
        
        # Set the trajectory component to the goal trajectory created above
        arm_goal.trajectory = arm_trajectory
        
        # Specify zero tolerance for the execution time
        arm_goal.goal_time_tolerance = rospy.Duration(0.0)
    
        # Send the goal to the action server
        self.right_arm_client.send_goal(arm_goal)

    def move_left_arm(self, pose):
        # Which joints define the arm?
        arm_joints = ['pan_joint',
                      'left_arm_tilt_joint',
                      'left_arm_lift_joint',
                      'left_arm_rotate_joint', 
                      'left_arm_elbow_joint',
                      'left_arm_wrist_yaw_joint',
                      'left_arm_wrist_tilt_joint']
        
        
        #if reset:
            # Set the arm back to the resting position
        arm_goal = pose
      
        # Create a single-point arm trajectory with the arm_goal as the end-point
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = arm_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = arm_goal
        arm_trajectory.points[0].velocities = [0.0 for i in arm_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in arm_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
        # Send the trajectory to the arm action server
        rospy.loginfo('Moving the arm to goal position...')
        
        # Create an empty trajectory goal
        arm_goal = FollowJointTrajectoryGoal()
        
        # Set the trajectory component to the goal trajectory created above
        arm_goal.trajectory = arm_trajectory
        
        # Specify zero tolerance for the execution time
        arm_goal.goal_time_tolerance = rospy.Duration(0.0)
    
        # Send the goal to the action server
        self.left_arm_client.send_goal(arm_goal)


class Move_head:
    """ 
    a class to move the head pan and tilt servos to a position in radians 
    """
    def __init__(self):
        #self.task = text#whole message from greeting
        self.head_move_client = SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.head_move_client.wait_for_server()

    def head_move(self, pan , tilt):   
        # Which joints define the head?
        head_joints = ['head_pan_joint', 'head_tilt_mod_joint']
        # Create a single-point head trajectory with the head_goal as the end-point
        head_trajectory = JointTrajectory()
        head_trajectory.joint_names = head_joints
        head_trajectory.points.append(JointTrajectoryPoint())
        head_trajectory.points[0].positions = pan , tilt
        head_trajectory.points[0].velocities = [0.0 for i in head_joints]
        head_trajectory.points[0].accelerations = [0.0 for i in head_joints]
        head_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
        # Send the trajectory to the head action server
        rospy.loginfo('Moving the head to goal position...')
        
        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory = head_trajectory
        head_goal.goal_time_tolerance = rospy.Duration(0.0)
    
        # Send the goal
        self.head_move_client.send_goal(head_goal)
        
        # Wait for up to 5 seconds for the motion to complete 
        self.head_move_client.wait_for_result(rospy.Duration(2.0))

  
        
