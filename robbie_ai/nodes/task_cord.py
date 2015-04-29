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
from robbie_ai.interaction import *
from robbie_ai.task_setup import *
import time
from datetime import datetime, timedelta
from time import localtime, strftime

class ActionTasks:
    def __init__(self, script_path):
        rospy.init_node('task_cordinator')
        rospy.on_shutdown(self.cleanup)
        # Initialize a number of parameters and variables for nav locations
        setup_task_environment(self)
        
        

        # Set the default TTS voice to use
        self.voice = rospy.get_param("~voice", "voice_en1_mbrola")
        self.robot = rospy.get_param("~robot", "robbie")
        #self.NavPublisher = rospy.Publisher("goto", String)

        # Publisher to manually control the robot (e.g. to stop it)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)

        self.client = SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        # Create the sound client object
        self.soundhandle = SoundClient()

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
        
        # Wait a moment to let the client connect to the
        # sound_play server
        rospy.sleep(1)
        
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()

        self.soundhandle.say(self.noon1 + self.robot +"   is on line" + " the time is   " + self.local, self.voice)
        rospy.sleep(2)

        # Subscribe to the recognizer output and set the callback function
        rospy.Subscriber('/nltk_interpret', String, self.commands)
        self.pub = rospy.Publisher('auto_dock', String, queue_size=1)

# here we do our main text prosessing    
    def commands(self,text):
        task = text.data
        ar = task.split(':')
        rospy.logwarn(ar)
        driver = Driver()
        
        if ar[0] == "stop":
            self.soundhandle.say("stopping", self.voice)
            self.move_base.cancel_all_goals()
            self.cmd_vel_pub.publish(Twist())
        if ar[0] == "go":
            loc = ar[6]+ar[9]
            self.soundhandle.say(loc, self.voice) 
            self.move_to(loc)
        if ar[0] == "pick":  
            self.soundhandle.say(str(PickUp(task)), self.voice)
        if ar[0] == "run"and ar[5] == "demo": 
            self.soundhandle.say("ok running    show time", self.voice)
            ShowTime()
        if ar[0] == "get" and ar[9] == "beer":  
            self.soundhandle.say("ok get a beer for you", self.voice)
        if ar[0] == "turn" and ar[5] == "left":
            self.soundhandle.say("ok turning left", self.voice) 
            driver.turn(angle = -0.5 * math.pi, angularSpeed = 0.5);#90 degrees left turn   
        if ar[0] == "turn" and ar[5] == "right":
            driver.turn(angle = 0.5 * math.pi, angularSpeed = 0.5);#90 degrees right turn  
            self.soundhandle.say("ok turning right", self.voice)
        if ar[0] == "move" and ar[5] == "forward": 
            driver.driveX(distance = 1.0, speed = 0.15) 
            self.soundhandle.say("ok, moving forward.", self.voice)
        if ar[0] == "look":  
            self.soundhandle.say("ok, looking     "+ str(ar[5]), self.voice)
            Move_Head(task)
        if ar[1] == "what"and ar[5] == "this":  
            self.soundhandle.say(" I have no idea. My vision system is not on line.", self.voice)
        if ar[1] == "what"and ar[5] == "time": 
            local = strftime("%H:%M:", localtime()) 
            self.soundhandle.say(" the time is   " + local , self.voice)
        if ar[1] == "what"and ar[5] == "weather": 
            self.soundhandle.say(str(Weather()) , self.voice)
        if ar[5] == "charge":
            self.soundhandle.say("Starting auto dock sequence", self.voice)
            self.move_base.cancel_all_goals()
            self.cmd_vel_pub.publish(Twist())
            self.pub.publish("dock")
        if ar[11] != "":
            #self.soundhandle.say(str(Hello()), self.voice)
            self.soundhandle.say("hello", self.voice)
    def move_to(self, location):
        goal = MoveBaseGoal()
        goal.target_pose.pose = self.room_locations[location]
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
  
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(40.0))

        if self.client.get_state() == GoalStatus.SUCCEEDED:
            result = self.client.get_result()
            print "Result: SUCCEEDED " 
        elif self.client.get_state() == GoalStatus.PREEMPTED:
            print "Action pre-empted"
        else:
            print "Action failed"


    def cleanup(self):
        self.soundhandle.stopAll()
        self.cmd_vel_pub.publish(Twist())
        rospy.loginfo("Shutting down talk node...")

if __name__=="__main__":
    try:
        ActionTasks(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Talk node terminated.")
