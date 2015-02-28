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
    def __init__(self, script_path):
        rospy.init_node('show time')
        rospy.on_shutdown(self.cleanup)
        # Initialize a number of parameters and variables for nav locations
        
        

        # Set the default TTS voice to use
        self.voice = rospy.get_param("~voice", "voice_don_diphone")
        self.robot = rospy.get_param("~robot", "robbie")
        self.room_locations = (('start', (Pose(Point(4.245, -1.588, 0.0), Quaternion(0.0, 0.0, -0.411, 0.911)))),
                      ('left', (Pose(Point(4.245, -1.588, 0.0), Quaternion(0.0, 0.0, -0.411, 0.911)))),
                      ('right', (Pose(Point(4.245, -1.588, 0.0), Quaternion(0.0, 0.0, -0.411, 0.911)))),
                      ('auto_dock', (Pose(Point(4.245, -1.588, 0.0), Quaternion(0.0, 0.0, -0.411, 0.911)))))
        

        

        self.client = SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        # Create the sound client object
        self.soundhandle = SoundClient()
        
        # Wait a moment to let the client connect to the
        # sound_play server
        rospy.sleep(2)
        
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()
        # Set the wave file path if used
        self.wavepath = rospy.get_param("~wavepath", script_path + "/../sounds")
         #define afternoon and morning
        self.noon = strftime("%p:", localtime())
        if self.noon == "AM:":
            self.noon1 = "Goood Morning  "
        else:
            self.noon1 ="Good After Noon   "
        self.local = strftime("%H:%M:", localtime())
        #local time
        self.local = strftime("%H:%M:", localtime())

        self.soundhandle.say(self.noon1 + self.robot +"   is on line" + " the time is   " + self.local, self.voice)
        rospy.sleep(2)
        self.move_to("start")
        #self.soundhandle.say("" + self.local, self.voice)
        self.soundhandle.say("Welcome to SHOW time.     This is where I get to demonstrate my capabilities" + self.local, self.voice)


    

    def move_to(self, location):
        goal = MoveBaseGoal()
        goal.target_pose.pose = self.room_locations[location]
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
  
        self.client.send_goal(goal)
        self.client.wait_for_result()

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

if __name__=="__main__":
    try:
        ShowTime(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("show time node terminated.")
