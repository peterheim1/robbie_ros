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
import sys
import time
import tf
import math
from datetime import datetime, timedelta
from time import localtime, strftime
from std_msgs.msg import String
from actionlib import SimpleActionClient
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *
from sound_play.libsoundplay import SoundClient
from datetime import datetime, timedelta
#from face_recognition.msg import *
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


from tf import transformations

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class NowTime:
    '''
    '''
    def __init__(self):
        
        #define afternoon and morning
        self.noon = strftime("%p:", localtime())
        if self.noon == "AM:":
            self.noon1 = "Goood Morning  "
        else:
            self.noon1 ="Good After Noon   "
        #self.local = strftime("%H:%M:", localtime())
        #local time
        self.local = strftime("%H:%M", localtime())
        self.Now ="The time is   " + str(self.local)
    def __str__(self):
        return str(self.Now)



class Weather:
    """ 
    a class to return the weather from cairns
    """
    def __init__(self):
        
        self.weather_com_result = pywapi.get_weather_from_weather_com('ASXX0020')
        #rospy.sleep(2)
        #return the processed value
        self.value = "It is " + str(self.weather_com_result['current_conditions']['text']) + " and " + self.weather_com_result['current_conditions']['temperature'] + "C now in Cairns.\n\n"
    def __str__(self):
        return str(self.value)
  
        
