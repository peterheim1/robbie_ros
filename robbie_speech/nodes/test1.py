#!/usr/bin/env python


import roslib; roslib.load_manifest('howie_ros')
import rospy
from festival.srv import *
from std_msgs.msg import String
import aiml
import aiml
import sys

# Create a Kernel object.
kern = aiml.Kernel()
#
#rospy.wait_for_service('speak_text')
speak_text_service = rospy.ServiceProxy('speak_text', FestivalSpeech)
speak_text_service("Hello my name is say teen the robot. ")
# When loading an AIML set, you have two options: load the original
# AIML files, or load a precompiled "brain" that was created from a
# previous run. If no brain file is available, we force a reload of
# the AIML files.
brainLoaded = False
forceReload = False
name = "Robbie"
def speak_text(text):
    #therapist = aiml.Kernel()
    #print "I AM SAYING : " + text
    rospy.loginfo(text.data)
    #print ">>>> CALLING FESTIVAL >>>>>"
    speak_text_service(text.data)
    #self.speak_text_service(therapist.respond(text.data))
    #print "<<<< FESTIVAL RETURNED <<<<<<"

rospy.Subscriber('/speech_text', String, speak_text)
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



# Enter the main input/output loop.
print "\nINTERACTIVE MODE (ctrl-c to exit)"
while(True):
	resp = kern.respond(raw_input("> "))
        speak_text_service(resp)
        print resp
