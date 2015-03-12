#!/usr/bin/env python

"""
    
"""

import rospy
import time
import re
from datetime import datetime, timedelta
from time import localtime, strftime
from std_msgs.msg import String
from sound_play.libsoundplay import SoundClient
import sys
from robbie_ai.interact import *

class TalkBack:
    def __init__(self, script_path):
        rospy.init_node('speech_cordinator')

        rospy.on_shutdown(self.cleanup)
        
        # Set the default TTS voice to use
        self.voice = rospy.get_param("~voice", "voice_don_diphone")
        self.robot = rospy.get_param("~robot", "robbie")
        
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
        self.soundhandle.playWave(self.wavepath + "/R2D2a.wav")
        rospy.sleep(1)
        self.soundhandle.say(self.noon1 + self.robot +"   is on line" + " the time is   " + self.local, self.voice)
        
        #rospy.loginfo("Say one of the navigation commands...")

        # Subscribe to the recognizer output and set the callback function
        rospy.Subscriber('/speech_text', String, self.talkback)
        self.pub = rospy.Publisher('/speech_parse', String, queue_size=5)
        
    def talkback(self, msg):
        # republis input to task coord or chat engine
        rospy.loginfo(msg.data)
        self.hold = msg.data
        matchObj = re.match( r'^robbie', self.hold, re.M|re.I)
        if matchObj:
            self.task1 = re.sub('robbie ','',self.hold,1)
            self.hold1 = Dictum(self.task1)
            self.pub.publish(self.task1)
            self.soundhandle.say(self.task1, self.voice)
            # send sentence to NLTK /nltk_parse
        else:
            self.soundhandle.say("alt input",self.voice)
            #send to chat minion chat
        
        

    def cleanup(self):
        self.soundhandle.stopAll()
        rospy.loginfo("Shutting down talk node...")

if __name__=="__main__":
    try:
        TalkBack(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Talk node terminated.")
