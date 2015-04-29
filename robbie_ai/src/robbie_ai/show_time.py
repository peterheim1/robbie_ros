#!/usr/bin/env python

"""
    
"""

import rospy
import re
from actionlib import SimpleActionClient
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
import sys
from phoenix_robot.interaction import *
from phoenix_robot.task_setup import *
import time
from datetime import datetime, timedelta
from time import localtime, strftime

class ShowTime:
    def __init__(self):
        rospy.init_node('show_time')
        rospy.on_shutdown(self.cleanup)
        # Initialize a number of parameters and variables for nav locations
        
        

        # Set the default TTS voice to use
        self.voice = rospy.get_param("~voice", "voice_en1_mbrola")
        self.robot = rospy.get_param("~robot", "robbie")
        self.start =(Pose(Point(1.7, 0, 0.0), Quaternion(0.0, 0.0, 1.000, 0.018)))
        self.left =(Pose(Point(1.960, -1.854, 0.0), Quaternion(0.0, 0.0, 0.916, 0.401)))
        self.right =(Pose(Point(2.123, 1.592, 0.0), Quaternion(0.0, 0.0, 0.877, -0.480)))
        self.auto_dock =(Pose(Point(0.5, 0, 0.0), Quaternion(0.0, 0.0, 0.002, 1.000)))
       

        

        self.client = SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        # Create the sound client object
        self.soundhandle = SoundClient()
        
        # Wait a moment to let the client connect to the
        # sound_play server
        rospy.sleep(2)
        
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()
        
        
        self.noon = strftime("%p:", localtime())
        if self.noon == "AM:":
            self.noon1 = "Goood Morning  "
        else:
            self.noon1 ="Good After Noon   "
        self.local = strftime("%H:%M:", localtime())
        #local time
        self.local = strftime("%H:%M:", localtime())

        #self.soundhandle.say(self.noon1 + self.robot +"   is on line" + " the time is   " + self.local, self.voice)

        rospy.sleep(6)
        self.move_to(self.start)
        self.soundhandle.say("Welcome to SHOW time.     This is where I get to demonstrate my capabilities" , self.voice)
        rospy.sleep(5)

        self.soundhandle.say("I can move to my left", self.voice)
        rospy.sleep(2)
        self.move_to(self.left)
        
        self.soundhandle.say("now back to the start position", self.voice)
        rospy.sleep(2)
        self.move_to(self.start)
        
        self.soundhandle.say("I can move to my right", self.voice)
        rospy.sleep(2)
        self.move_to(self.right)
        
        self.soundhandle.say("And again back to the start position", self.voice)
        rospy.sleep(2)
        self.move_to(self.start)

        rospy.sleep(2)
        self.soundhandle.say("Thank you for your time ", self.voice)
        
        

        rospy.sleep(2)

        #rospy.sleep(2)
        self.move_to(self.auto_dock)
    

    def move_to(self, location):
        goal = MoveBaseGoal()
        goal.target_pose.pose = location
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
  
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(50.0))

        if self.client.get_state() == GoalStatus.SUCCEEDED:
            result = self.client.get_result()
            print "Result: SUCCEEDED " 
        elif self.client.get_state() == GoalStatus.PREEMPTED:
            print "Action pre-empted"
        else:
            print "Action failed"


    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down show time node...")


