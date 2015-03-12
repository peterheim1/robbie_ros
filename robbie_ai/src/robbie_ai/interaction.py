#!/usr/bin/env python

"""
  interaction.py: 
  helper function for AI interface
  Copyright (c) Escaliente Robotics.  All right reserved.

  todo
  needs a global blackboard

"""

import rospy
import re
import time
import pywapi
from datetime import datetime, timedelta
from time import localtime, strftime
from std_msgs.msg import String

class PickUp:
    """ 
    a class to fins and pick up a object 
    """
    def __init__(self, text):
        self.task = text#whole message from greeting
        ar = text.split(':')
        self.size =ar[2]
        self.color =ar[3]
        self.size =ar[4]
        self.shape =ar[5]
        self.object =ar[2]+ " "+ ar[3] + " " + ar[4] +" " + ar[5] 
        rospy.sleep(2)
        self.task1 ="i need a arm to pick up the " + self.object
        print self.object
        #return the processed value
        self.value = str(self.task1)
    def __str__(self):
        return str(self.value)

class Hello:
    """ 
    a class to greet people need to recognise people 
    """
    def __init__(self):
        
        self.task1 ="hello peter"
        rospy.sleep(2)
        #return the processed value
        self.value = str(self.task1)
    def __str__(self):
        return str(self.value)

class HelloUnknown:
    """ 
    a class to recognise a new person 
    """
    def __init__(self):
        
        self.task1 ="hello what is your name"
        rospy.sleep(2)
        #return the processed value
        self.value = str(self.task1)
    def __str__(self):
        return str(self.value)

class Weather:
    """ 
    a class to return the weather from cairns
    """
    def __init__(self):
        
        self.weather_com_result = pywapi.get_weather_from_weather_com('ASXX0020')
        rospy.sleep(2)
        #return the processed value
        self.value = "It is " + str(self.weather_com_result['current_conditions']['text']) + " and " + self.weather_com_result['current_conditions']['temperature'] + "C now in Cairns.\n\n"
    def __str__(self):
        return str(self.value)
