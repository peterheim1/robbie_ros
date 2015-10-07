#!/usr/bin/env python

"""
    trajectory_demo.py - Version 0.1 2014-01-14
    
    Send a trajectory to the FollowJointTrajectoryAction server
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryDemo():
    def __init__(self):
        rospy.init_node('trajectory_demo')
        
        # Set to True to move back to the starting configurations
        reset = rospy.get_param('~reset', False)
        
        # Set to False to wait for arm to finish before moving head
        sync = rospy.get_param('~sync', True)
        
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
        arm_goal  = [0.0, 0, 1.5, 0, -0.1, -0, -0]
            
        #else:
            # Set a goal configuration for the arm
        #arm_goal  = [0.0, -1.1701, 1.5347, 1.1978, -1.218, -0.1466, -0.6527]
            
    
        # Connect to the right arm trajectory action server
        rospy.loginfo('Waiting for right arm trajectory controller...')
        
        arm_client = actionlib.SimpleActionClient('right_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        arm_client.wait_for_server()
        
        rospy.loginfo('...connected.')
        
        
    
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
        arm_client.send_goal(arm_goal)
        
        
        
        rospy.loginfo('...done')
        
if __name__ == '__main__':
    try:
        TrajectoryDemo()
    except rospy.ROSInterruptException:
        pass
    
