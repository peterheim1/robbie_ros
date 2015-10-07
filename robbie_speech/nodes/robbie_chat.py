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


class Robbie_Chat():

    def __init__(self):
        rospy.init_node('robbie_chat_node', anonymous=True)
        self.kern = aiml.Kernel()
        self.kern.setBotPredicate("name","robbie")
        self.kern.setBotPredicate("location","Australia")
        self.kern.setBotPredicate("botmaster","Petrus")
        self.kern.setBotPredicate("gender","male")
        self.brainLoaded = False
        self.forceReload = False
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

        while not self.brainLoaded:
	    if self.forceReload or (len(sys.argv) >= 2 and sys.argv[1] == "reload"):
		    # Use the Kernel's bootstrap() method to initialize the Kernel. The
		    # optional learnFiles argument is a file (or list of files) to load.
		    # The optional commands argument is a command (or list of commands)
		    # to run after the files are loaded.
                    #self.kern.setBotPredicate(name. robbie)
		    self.kern.bootstrap(learnFiles="std-startup.xml", commands="load aiml b")
		    self.brainLoaded = True
		    # Now that we've loaded the brain, save it to speed things up for
		    # next time.
		    self.kern.saveBrain("standard.brn")
	    else:
		    # Attempt to load the brain file.  If it fails, fall back on the Reload
		    # method.
		    try:
			    # The optional branFile argument specifies a brain file to load.
			    self.kern.bootstrap(brainFile = "standard.brn")
			    self.brainLoaded = True
		    except:
			    self.forceReload = True


        
        rospy.Subscriber('/speech_text', String, self.speak_text)



    def speak_text(self, text):
        print text.data
        self.soundhandle.say(self.kern.respond(text.data), self.voice)





if __name__ == '__main__':
    try:
        Robbie_Chat()
        rospy.spin()
    except rospy.ROSInterruptException: pass

