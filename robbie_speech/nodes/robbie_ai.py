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
from speech_recognition_msgs.msg import SpeechRecognitionCandidates
from jsk_gui_msgs.msg import VoiceMessage


class Robbie_Chat():

    def __init__(self):
        rospy.init_node('robbie_chat_node', anonymous=True)
        self.kern = aiml.Kernel()
        self.kern.setBotPredicate("name", "Robbie")
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
        rospy.sleep(1)
        
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()

        self.kern.learn("standard/std-*.aiml")
        self.kern.learn("firsttry.aiml")
        self.kern.learn("commands.aiml")


        
        rospy.Subscriber('/speech_text', String, self.speak_text)
        rospy.Subscriber('/voice', SpeechRecognitionCandidates, self.talkback)
        rospy.Subscriber('/Tablet/voice', VoiceMessage, self.talkback2)



    def speak_text(self, text):
        print text.data
        self.soundhandle.say(self.kern.respond(text.data), self.voice)

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

