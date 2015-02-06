#!/usr/bin/env python
'''
Created March, 2012

@author: Peter Heim

  Neck_Tilt.py - gateway to Arduino based arm controller
  Copyright (c) 2011 Peter Heim.  All right reserved.
  Borrowed heavily from Mike Feguson's ArbotiX base_controller.py code.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the Vanadium Labs LLC nor the names of its
        contributors may be used to endorse or promote products derived
        from this software without specific prior written permission.
 
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import rospy
import tf
import math
from math import sin, cos, pi
import sys
import time
from std_msgs.msg import String
from std_msgs.msg import Float64
from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import JointState

class Neck_Tilt(object):
        '''
        Helper class for communicating with an Neck_Tilt servo

        '''
    


        def __init__(self,):
            rospy.init_node('neck_tilt')
            rospy.Subscriber('/head_tilt_joint/state',JointState, self._HandleJoint_1_Command)
            rospy.Subscriber('/head_tilt_mod_joint/command',Float64, self._nect_tilt_Command)
            self._neckPublisher = rospy.Publisher("head_tilt_mod_joint/state", JointState)
            self._neck_Tilt_Publisher = rospy.Publisher("head_tilt_joint/command", Float64)
            self.gear_ratio = 0.3

        def _HandleJoint_1_Command(self, data):
                """ republish position. """
                neck =(data)
                self.gear_ratio = 0.3
                new_pos =float (data.current_pos) * self.gear_ratio
                new_goal=float (data.goal_pos) * self.gear_ratio
                new_error = new_goal - new_pos
                jointstate = JointState()
                jointstate.name =("head_tilt_mod_joint")
                jointstate.motor_ids =(neck.motor_ids)
                jointstate.motor_temps =(neck.motor_temps)
                jointstate.goal_pos =(new_goal)
                jointstate.current_pos =(new_pos)
                jointstate.velocity =(neck.velocity)
                jointstate.load =(neck.load)
                jointstate.is_moving =(neck.is_moving)
                jointstate.error =(new_error)
                #rospy.loginfo(jointstate)
                self._neckPublisher.publish(jointstate)

        def _nect_tilt_Command(self, data):
                """ republish position. """
                gear_ratio = self.gear_ratio *10
                position = data.data * gear_ratio
                rospy.loginfo(position)
                self._neck_Tilt_Publisher.publish(position)
            

if __name__ == '__main__':
        Neck_Tilt = Neck_Tilt()
        try:
                
                rospy.spin()

        except rospy.ROSInterruptException:
                pass



               
