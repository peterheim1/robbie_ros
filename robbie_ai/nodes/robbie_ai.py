#!/usr/bin/env python
'''
works 
listens on speech_text topic
responds on speak_text service
todo
add person details
add more topics

'''

import rospy
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
import aiml
import sys
import os
import time
from datetime import datetime, timedelta
from time import localtime, strftime
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from jsk_gui_msgs.msg import VoiceMessage

dir = os.path.dirname(os.path.abspath(__file__))
class Robbie_Chat():

    def __init__(self):
        rospy.init_node('robbie_chat_node', anonymous=True)
        self.robot ="Robbie"
        self.kern = aiml.Kernel()
        self.kern.setBotPredicate("name", self.robot)
        self.kern.setBotPredicate("location","Australia")
        self.kern.setBotPredicate("botmaster","Petrus")
        self.kern.setBotPredicate("master","Petrus")
        self.kern.setBotPredicate("birthday","1st of August 2011")
        self.kern.setBotPredicate("gender","male")
        self.kern.setBotPredicate("favoriteactor","walle")
        self.kern.setBotPredicate("favoriteactress","eve")
        self.kern.setBotPredicate("favoriteband","ELO")
        self.kern.setBotPredicate("favoritebook","The Iliad")
        self.kern.setBotPredicate("favoritecar","Mercedes")
        self.kern.setBotPredicate("favoritecolor","sliver")
        self.kern.setBotPredicate("favoritedrink","oil")
        self.kern.setBotPredicate("favoritefood","power")
        self.kern.setBotPredicate("favoriteicecream","power")
        self.kern.setBotPredicate("favoritemovie","star wars")
        self.kern.setBotPredicate("favoritesong","mister roboto")
        self.kern.setBotPredicate("favoritesport","darpa cup")
        self.kern.setBotPredicate("favoritetvshow","Futurama")
        #self.brainLoaded = False
        #self.forceReload = False
        # Set the default TTS voice to use
        self.voice = rospy.get_param("~voice", "voice_en1_mbrola")
        self.robot = rospy.get_param("~robot", "robbie")
        # Create the sound client object
        self.soundhandle = SoundClient()
        
        # Wait a moment to let the client connect to the
        # sound_play server
        rospy.sleep(2)
        
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()

        # Set the wave file path if used
        #self.wavepath = rospy.get_param("~wavepath", script_path + "/../sounds")

        #define afternoon and morning
        self.noon = strftime("%p:", localtime())
        if self.noon == "AM:":
            self.noon1 = "Goood Morning  "
        else:
            self.noon1 ="Good After Noon   "
        self.local = strftime("%H:%M:", localtime())
        #local time
        self.local = strftime("%H:%M:", localtime())

        #self.head_move.head_move(0.0, 0.0)
        self.soundhandle.say(self.noon1 + self.robot +"   is on line" + " the time is   " + self.local, self.voice)



        self.kern.learn(dir + "/../standard/std-*.aiml")
        self.kern.learn(dir+"/firsttry.aiml")
        self.kern.learn(dir + "/commands.aiml")


        
        rospy.Subscriber('/speech_text', String, self.speak_text)
        rospy.Subscriber('/voice', SpeechRecognitionCandidates, self.talkback)
        rospy.Subscriber('/Tablet/voice', VoiceMessage, self.talkback2)



    def speak_text(self, text):
        print text
        self.soundhandle.say(self.kern.respond(text), self.voice)

    def talkback2(self, msg):
        # Print the recognized words on the screen
        rospy.logwarn(msg.texts[0])
        hold = msg.texts[0]
        self.speak_text(hold)
        
        
    def talkback(self, msg):
        # Print the recognized words on the screen
        rospy.loginfo(msg.transcript[0])
        hold = msg.transcript[0]
        self.speak_text(hold)
        



if __name__ == '__main__':
    try:
        Robbie_Chat()
        rospy.spin()
    except rospy.ROSInterruptException: pass

