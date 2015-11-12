#!/usr/bin/env python
"""
    the aim of this node is to start looking for face 
    
"""

import rospy
import time
import re
import sys
import rospy
#import actionlib
from datetime import datetime, timedelta
from time import localtime, strftime
from std_msgs.msg import String
from cob_perception_msgs.msg import DetectionArray, Detection
from sound_play.libsoundplay import SoundClient
#from face_recognition.msg import *

#from phoenix_robot.interact import *

class Meet_N_Greet:
    def __init__(self, script_path):
        rospy.init_node('meet_n_greet')

        rospy.on_shutdown(self.cleanup)
        
        # Set the default TTS voice to use
        self.voice = rospy.get_param("~voice", "voice_en1_mbrola")
        self.robot = rospy.get_param("~robot", "robbie")
        self.greeted =[]
        #self.person =''
        

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
        #self.soundhandle.say(self.noon1 + "meet and greet   is on line" + " the time is   " + self.local, self.voice)

        #start looking for a face with face recognition action server  (cob_people_detection_msgs/DetectionArray)

        #result will be published on this topic
        rospy.Subscriber("/cob_people_detection/face_recognizer/face_recognitions", DetectionArray, self.face_found)
        
        

    def face_found(self,msg):
        '''
        clean up feedback to extract the name
        check if greeted before answer accordingly
        '''
        person = 'UnknownHead'
        for detection in msg.detections:
            person = detection.label
            #print person
        if person != 'UnknownHead':
            print person
            rospy.sleep(1)
            greet = self.greeted.count(person)
            print greet
            if greet < 1:
                self.soundhandle.say("hello   "+ person +"    "+ self.noon1 +"          ",self.voice)
                #add name to greeted list
                self.greeted.append(person)
                #rospy.sleep(10)
            #else:
                #self.soundhandle.say("hi     ",self.voice)
        #start looking again
        #rospy.sleep(10)
        


        

    def Unknown(self,msg):
        self.soundhandle.say("hello     what is your name   " ,self.voice)
        rospy.sleep(3)
        
        
        
        

    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down meetNgreet node...")

if __name__=="__main__":
    try:
        Meet_N_Greet(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("meetNgreet node terminated.")
