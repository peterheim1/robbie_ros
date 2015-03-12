#!/usr/bin/env python

"""
  interact.py: classes AI interface
  Copyright (c) Escaliente Robotics.  All right reserved.

  todo
  add NLTK filtering

"""

import rospy
import re

class Dictum:
    """ 
    

    """

    def __init__(self, text):
        self.task = text#whole message from greeting
        matchObj = re.match( r'^where', self.task, re.M|re.I)
        matchget = re.match( r'^get', self.task, re.M|re.I)
        matchgo = re.match( r'^go', self.task, re.M|re.I)
        if matchObj:
            self.task2 = re.sub('where ','',self.task,1)
            self.task1 = str(self.task2) + " " + " on my right"
        elif matchget:
            self.task2 = re.sub('me ','you ',self.task,1)
            self.task1 = "OK " + str(self.task2) + "  " + " on my way"
            
        elif matchgo:
            self.task2 = re.sub('go ','going ',self.task,1)
            self.task1 = "OK " + str(self.task2) + "  " + " on my way"
    
        else:
            self.task1 ="i dont under stand the question"

        #return the processed value
        self.value = str(self.task1)
    def __str__(self):
        return str(self.value)
