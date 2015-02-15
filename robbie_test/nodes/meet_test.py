#!/usr/bin/env python
"""
    the aim of this node is to start looking for face 
    if a know face is found for the first time that day then say hello
    else just node head

    status/todo
    change face tracker to publish face found then pause itself
    recognise face or add face 
then restart
"""

import rospy
import time
import re
import sys
import rospy
import actionlib
from datetime import datetime, timedelta
from time import localtime, strftime
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
from face_recognition.msg import *

#from phoenix_robot.interact import *

class Meet_N_Greet:
    def __init__(self, script_path):
        rospy.init_node('meet_n_greet')

        rospy.on_shutdown(self.cleanup)
        
        # Set the default TTS voice to use
        self.voice = rospy.get_param("~voice", "voice_don_diphone")
        self.robot = rospy.get_param("~robot", "robbie")
        self.greeted =[]

        #set action server
        self.client = actionlib.SimpleActionClient('face_recognition', face_recognition.msg.FaceRecognitionAction)
        # listening for goals.
        self.client.wait_for_server()

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
        
        # Create the sound client object
        self.soundhandle = SoundClient()
        
        # Wait a moment to let the client connect to the
        # sound_play server
        rospy.sleep(1)
        
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()
        
        # Announce that we are ready for input
        rospy.sleep(1)
        self.soundhandle.say(self.noon1 + "meet and greet   is on line" + " the time is   " + self.local, self.voice)

        #start looking for a face with face recognition action server

        #wait for result 
        rospy.Subscriber("/face_recognition/result", FaceRecognitionActionFeedback, self.face_found)

        self.look_for_face()#put this in a while loop 

    def look_for_face(self):
        '''
        send command look for face once
        '''
        goal = face_recognition.msg.FaceRecognitionGoal(order_id=0, order_argument="none")
        self.client.send_goal(goal)
        #rospy.logwarn("message sent")

    def face_found(self,msg):
        '''
        clean up feedback to extract the name
        check if greeted before answer accordingly
        '''

        person = str(msg.result.names[0])
        rospy.logwarn(person)
        
        greet = person in self.greeted
        if greet == 0:#or !== "true"(not true)
            self.soundhandle.say("hello   "+ person +"    "+ self.noon1 +"          ",self.voice)
            rospy.sleep(1)
        else:
            self.soundhandle.say("hi     ",self.voice)
        #add name to greeted list
        self.greeted.append(person)
        print self.greeted
        #start looking again
        rospy.sleep(30)
        self.look_for_face()
        
        
        
        
        

    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down meetNgreet node...")

if __name__=="__main__":
    try:
        Meet_N_Greet(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("meetNgreet node terminated.")
