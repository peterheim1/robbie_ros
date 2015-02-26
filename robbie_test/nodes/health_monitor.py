#!/usr/bin/env python

"""
    patrol_tree.py - Version 1.0 2013-03-18
    
    Navigate a series of waypoints while monitoring battery levels.
    Uses the pi_trees package to implement a behavior tree task manager.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""
import rospy
import time
from std_msgs.msg import Float32, String, Int16
from geometry_msgs.msg import Twist
from pi_trees_ros.pi_trees_ros import *
from rbx2_tasks.task_setup import *#use standered setup for testing
from rbx2_msgs.srv import *
from robbie.srv import *
from phoenix_robot.clean_house_tasks_tree import *
from collections import OrderedDict
from math import pi, sqrt

from datetime import datetime, timedelta
from time import localtime, strftime

class Patrol():
    def __init__(self):
        rospy.init_node("task_action_node")

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        self.target = MoveBaseGoal()
        
        # Initialize a number of parameters and variables
        setup_task_environment(self)

        
        
        # Set the docking station pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.docking_station_pose
        
        # Assign the docking station pose to a move_base action task
        NAV_DOCK_TASK = SimpleActionTask("NAV_DOC_TASK", "move_base", MoveBaseAction, goal, reset_after=True)
        #NAV_TASK = SimpleActionTask("NAV_TASK", "move_base", MoveBaseAction, self.target, reset_after=False)
        #MOVE_BASE_TASK = SimpleActionTask("MOVE_BASE_TASK", "move_base", MoveBaseAction, goal, reset_after=False)
        
        # Create the root node
        BEHAVE = Sequence("BEHAVE")
        
        # Create the "stay healthy" selector
        STAY_HEALTHY = Selector("STAY_HEALTHY")

        # Create the "stay navigator" selector
        NAVIGATOR = Selector("NAVIGATOR")
        
        # Create the patrol loop decorator
        #LOOP_PATROL = Loop("LOOP_PATROL", iterations=self.n_patrols)
        
        # Add the two subtrees to the root node in order of priority
        BEHAVE.add_child(STAY_HEALTHY)
        BEHAVE.add_child(NAVIGATOR)
        
        # Add the TASK check  "NAVIGATOR" task
        with NAVIGATOR:
            # check for navigation target (uses MonitorTask)
            #change to a service task
            GOTO_LOCATION = MonitorTask("GOTO_LOCATION", "goto", String, self.goto_location)

            NAVIGATOR.add_child(GOTO_LOCATION)
            #GOTO_LOCATION.add_child(MOVE_BASE_TASK)
        
        # Add the battery check and recharge tasks to the "stay healthy" task
        with STAY_HEALTHY:
            # The check battery condition (uses MonitorTask)
            CHECK_BATTERY = MonitorTask("CHECK_BATTERY", "battery_level", Float32, self.check_battery)
            #AUTODOCK = AutoDock()
            
            AUTODOCK = ServiceTask("AUTODOCK", "auto_dock", Docker, 100, result_cb=self.recharge_cb)
            CHARGING = MonitorTask("CHARGING", "battery_status", Float32, self.battery_state)
            
            # Build the recharge sequence using inline construction
            #RECHARGE = Sequence("RECHARGE", [NAV_DOCK_TASK, AUTODOCK, CHARGE_ROBOT])
            RECHARGE = Sequence("RECHARGE")
            RECHARGE.add_child(NAV_DOCK_TASK)#navigate to the autodock prestage area
            RECHARGE.add_child(AUTODOCK)#start auto dock sequence finish when docked
            RECHARGE.add_child(CHARGING)#signal when chargeing is complete

                
            # Add the check battery and recharge tasks to the stay healthy selector
            STAY_HEALTHY.add_child(CHECK_BATTERY)
            STAY_HEALTHY.add_child(RECHARGE)
                
        # Display the tree before beginning execution
        print "action task Behavior Tree"
        print_tree(BEHAVE)
        
        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.run()
            rospy.sleep(0.1)
            
    def check_battery(self, msg):
        if msg.data is None:
            return TaskStatus.RUNNING
        else:
            if msg.data < 11.0:
                #rospy.loginfo("Battery low - level: " + str(int(msg.data)))
                return TaskStatus.FAILURE
            else:
                #rospy.loginfo("Battery charged - level: " + str(int(msg.data)))
                return TaskStatus.SUCCESS

    def battery_state(self, msg):
        #if msg.data is None:
            #return TaskStatus.RUNNING
        #else:
            if msg.data < 13:
                rospy.loginfo("BATTERY charging - level: " + str(int(msg.data)))
                return TaskStatus.RUNNING
            else:
                rospy.loginfo("Battery charged - level: " + str(int(msg.data)))
                return TaskStatus.FAILURE
    
    def recharge_cb(self, result):
        rospy.loginfo("Auto dock completed")
        #return TaskStatus.SUCCESS

    def goto_location(self, msg):
        if msg.data is None:
            return TaskStats.RUNNING
        else:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = (Pose(Point(1.5, 1.5, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))) #self.room_locations[msg.data]
            ##self.target = goal
            #print goal
            #print str(self.room_locations[msg.data])
            #MOVE_BASE_TASK = SimpleActionTask("MOVE_BASE_TASK", "move_base", MoveBaseAction, goal, reset_after=False)
            return TaskStatus.SUCCESS
            
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_all_goals()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    tree = Patrol()

