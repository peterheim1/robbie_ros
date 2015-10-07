#!/usr/bin/env python


import rospy
from festival.srv import *
from std_msgs.msg import String
import aiml
import sys

class Robbie_Chat():

    

    def __init__(self):
        rospy.init_node('robbie_chat_node', anonymous=True)
        rospy.on_shutdown(self.cleanup)
        kern = aiml.Kernel()
        brainLoaded = False
        forceReload = False
        
        rospy.wait_for_service('speak_text')
        try:
            self.speak_text_service = rospy.ServiceProxy('speak_text', FestivalSpeech)
        except rospy.ServiceException, e:
            print "Failed to acquire Festival SpeakText service: %s"%e

        self.speak_text_service("Hello my name is Robbie the robot.  How are you feeling today?   ")
        rospy.Subscriber('/speech_text', String, self.speak_text)

        while not brainLoaded:
	    if forceReload or (len(sys.argv) >= 2 and sys.argv[1] == "reload"):
		# Use the Kernel's bootstrap() method to initialize the Kernel. The
		# optional learnFiles argument is a file (or list of files) to load.
		# The optional commands argument is a command (or list of commands)
		# to run after the files are loaded.
		kern.bootstrap(learnFiles="std-startup.xml", commands="load aiml b")
		brainLoaded = True
		# Now that we've loaded the brain, save it to speed things up for
		# next time.
		kern.saveBrain("standard.brn")
	else:
		# Attempt to load the brain file.  If it fails, fall back on the Reload
		# method.
		try:
			# The optional branFile argument specifies a brain file to load.
			kern.bootstrap(brainFile = "standard.brn")
			brainLoaded = True
		except:
			forceReload = True



    def speak_text(self, text):
        therapist = aiml.Kernel()
        #print "I AM SAYING : " + text
        rospy.loginfo(text.data)
        #print ">>>> CALLING FESTIVAL >>>>>"
        #self.speak_text_service(text.data)
        self.speak_text_service(therapist.respond(text.data))
        #print "<<<< FESTIVAL RETURNED <<<<<<"

    def cleanup(self):
        self.speak_text_service("shutting down Robbie node")
        rospy.loginfo("Shutting down robbie node...")
    
    def startup(self):
        self.speak_text_service("Robbie is starting")

if __name__ == '__main__':
    try:
        Robbie_Chat()
        rospy.spin()
    except rospy.ROSInterruptException: pass
