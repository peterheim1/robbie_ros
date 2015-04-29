#!/usr/bin/env python
'''
Created March, 2012

@author: Peter Heim

  r_shoulder.py - gateway to Arduino based arm controller
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
from math import sin, cos, pi, radians, degrees
import sys
import time
from std_msgs.msg import String
from std_msgs.msg import Float64, Float32
from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import JointState
#from sensor_msgs.msg import JointState
from SerialDataGateway import SerialDataGateway

class R_shoulder(object):
        '''
        Helper class for communicating with an R_shoulder board over serial port
        '''
        
       

        def _HandleReceivedLine(self,  line):
                self._Counter = self._Counter + 1
                #rospy.logwarn(str(self._Counter) + " " + line)
                #if (self._Counter % 50 == 0):
                self._SerialPublisher.publish(String(str(self._Counter) + ", in:  " + line))

                if (len(line) > 0):
                        lineParts = line.split('\t')
                       
                        if (lineParts[0] == 'p1'):
                                self._BroadcastJointStateinfo_P1(lineParts)
                                return
                        if (lineParts[0] == 'p2'):
                                self._BroadcastJointStateinfo_P2(lineParts)
                                return
                        if (lineParts[0] == 'p3'):
                                self._BroadcastJointStateinfo_P3(lineParts)
                                return
                        if (lineParts[0] == 'p4'):
                                self._BroadcastJointStateinfo_P4(lineParts)
                                return
                        if (lineParts[0] == 'p5'):
                                self._BroadcastJointStateinfo_P5(lineParts)
                                return
                        
        
        def _BroadcastJointStateinfo_P1(self, lineParts):
                partsCount = len(lineParts)
                #rospy.logwarn(partsCount)
                if (partsCount  < 7):
                        pass
                try:
                        P1 = ((321-float(lineParts[1]))*0.008)-0.15
                        P2 = self.left_tilt
                        P3 = float(lineParts[3])# current
                        P4 = 0#float(lineParts[4])# speed
                        val = [P1, P2, P3, P4]
                        Motor_State = MotorState()
                        Motor_State.id = 11
                        Motor_State.goal = P2
                        Motor_State.position = P1
                        Motor_State.speed = P4
                        Motor_State.load = P3
                        Motor_State.moving = 0
                        Motor_State.timestamp = time.time()
                        self.P1_MotorPublisher.publish(Motor_State)
                        self._left_tilt_Publisher.publish(P1)
                        Joint_State = JointState()
                        Joint_State.name = "left_arm_tilt_joint"
                        Joint_State.goal_pos = P2
                        Joint_State.current_pos = P1
                        Joint_State.velocity = P4
                        Joint_State.load = P3
                        Joint_State.error = P1 - P2
                        Joint_State.is_moving = 0
                        Joint_State.header.stamp = rospy.Time.now()#.stamprospy.Time.from_sec(state.timestamp)
                        self._P1_JointPublisher.publish(Joint_State)
                        #rospy.logwarn(Joint_State)

                except:
                        rospy.logwarn("Unexpected error:left_arm_tilt_joint" + str(sys.exc_info()[0]))

        def _BroadcastJointStateinfo_P2(self, lineParts):
                partsCount = len(lineParts)
                #rospy.logwarn(partsCount)
                if (partsCount  < 4):
                        pass
                try:
                        
                        P1 = ((321-float(lineParts[1]))*0.008)-0.07
                        P2 = self.right_tilt
                        P3 = 0#float(lineParts[3])# current
                        P4 = 0#float(lineParts[4])# speed
                        val = [P1, P2, P3, P4]
                        Motor_State = MotorState()
                        Motor_State.id = 11
                        Motor_State.goal = P2
                        Motor_State.position = P1
                        Motor_State.speed = P4
                        Motor_State.load = P3
                        Motor_State.moving = 0
                        Motor_State.timestamp = time.time()
                        self.P2_MotorPublisher.publish(Motor_State)
                        self._right_tilt_Publisher.publish(P1)
                        Joint_State = JointState()
                        Joint_State.name = "right_arm_tilt_joint"
                        Joint_State.goal_pos = P2
                        Joint_State.current_pos = P1
                        Joint_State.velocity = P4
                        Joint_State.load = P3
                        Joint_State.error = P1 - P2
                        Joint_State.is_moving = 0
                        Joint_State.header.stamp = rospy.Time.now()#.stamprospy.Time.from_sec(state.timestamp)
                        self._P2_JointPublisher.publish(Joint_State)
                        #rospy.logwarn(Joint_State)

                except:
                        rospy.logwarn("Unexpected error:right_arm_tilt_joint" + str(sys.exc_info()[0]))

        def _BroadcastJointStateinfo_P3(self, lineParts):
                partsCount = len(lineParts)
                #rospy.logwarn(partsCount)
                if (partsCount  < 7):
                        pass
                try:
                        #offset = Float(-1.57)
                        P1 = (float(lineParts[1])/1000)-0.6
                        P2 = self.right_lift #0-((float(lineParts[2])* 0.00174532925)-1.57)
                        P3 = 0#float(lineParts[3])
                        P4 = 0
                        val = [P1, P2, P3, P4]
                        Motor_State = MotorState()
                        Motor_State.id = 11
                        Motor_State.goal = P2
                        Motor_State.position = P1
                        Motor_State.speed = P4
                        Motor_State.load = P3
                        Motor_State.moving = 0
                        Motor_State.timestamp = time.time()
                        self.P3_MotorPublisher.publish(Motor_State)
                        #rospy.logwarn(Motor_State)
                        self._left_lift_Publisher.publish(P1)

                        Joint_State = JointState()
                        Joint_State.name = "left_arm_lift_joint"
                        Joint_State.goal_pos = P2
                        Joint_State.current_pos = P1
                        Joint_State.velocity = P4
                        Joint_State.load = P3
                        Joint_State.error = P1 - P2
                        Joint_State.is_moving = 0
                        Joint_State.header.stamp = rospy.Time.now()
                        self._P3_JointPublisher.publish(Joint_State)

                except:
                        rospy.logwarn("Unexpected error:left_arm_lift_joint" + str(sys.exc_info()[0]))
                       

        def _BroadcastJointStateinfo_P4(self, lineParts):
                partsCount = len(lineParts)
                #rospy.logwarn(partsCount)
                if (partsCount  < 7):
                        pass
                try: 
                        #off = 1570
                        P1 = float(lineParts[1])/1000
                        P2 = self.right_lift #0-((float(lineParts[2])* 0.00174532925)-1.57)
                        P3 = 0 #float(lineParts[3])
                        P4 = 0
                        val = [P1, P2, P3, P4]
                        Motor_State = MotorState()
                        Motor_State.id = 11
                        Motor_State.goal = P2
                        Motor_State.position = P1
                        Motor_State.speed = P4
                        Motor_State.load = P3
                        Motor_State.moving = 0
                        Motor_State.timestamp = time.time()
                        self.P4_MotorPublisher.publish(Motor_State)
                        #rospy.logwarn(Motor_State)
                        self._right_lift_Publisher.publish(P1)

                        Joint_State = JointState()
                        Joint_State.name = "right_arm_lift_joint"
                        Joint_State.goal_pos = P2
                        Joint_State.current_pos = P1
                        Joint_State.velocity = P4
                        Joint_State.load = P3
                        Joint_State.error = P1 - P2
                        Joint_State.is_moving = 0
                        Joint_State.header.stamp = rospy.Time.now()
                        self._P4_JointPublisher.publish(Joint_State)
                except:
                        rospy.logwarn("Unexpected error:right_arm_lift_joint" + str(sys.exc_info()[0]))

        def _BroadcastJointStateinfo_P5(self, lineParts):
                partsCount = len(lineParts)
                #rospy.logwarn(partsCount)
                if (partsCount  < 7):
                        pass
                try:
                        
                        
                        P1 = 0 - radians(float(lineParts[1])/10)
                        P2 = self.left_elbow
                        P3 = 0#float(lineParts[3])# current
                        P4 = 0#float(lineParts[4])# speed
                        val = [P1, P2, P3, P4]
                        Motor_State = MotorState()
                        Motor_State.id = 11
                        Motor_State.goal = P2
                        Motor_State.position = P1
                        Motor_State.speed = P4
                        Motor_State.load = P3
                        Motor_State.moving = 0
                        Motor_State.timestamp = time.time()
                        self.P5_MotorPublisher.publish(Motor_State)
                        self._left_elbow_Publisher.publish(P1)
                        Joint_State = JointState()
                        Joint_State.name = "left_arm_elbow_joint"
                        Joint_State.goal_pos = P2
                        Joint_State.current_pos = P1
                        Joint_State.velocity = P4
                        Joint_State.load = P3
                        Joint_State.error = P1 - P2
                        Joint_State.is_moving = 0
                        Joint_State.header.stamp = rospy.Time.now()#.stamprospy.Time.from_sec(state.timestamp)
                        self._P5_JointPublisher.publish(Joint_State)
                        #rospy.logwarn(Joint_State)

                except:
                        rospy.logwarn("Unexpected error:left_arm_elbow_joint" + str(sys.exc_info()[0]))


            
               
               
               

       

       
        def _WriteSerial(self, message):
                self._SerialPublisher.publish(String(str(self._Counter) + ", out: " + message))
                self._SerialDataGateway.Write(message)

        def __init__(self,):
                '''
                Initializes the receiver class.
                port: The serial port to listen to.
                baudrate: Baud rate for the serial communication
                '''
               
                #port = rospy.get_param("~port", "/dev/ttyACM0")
                #baud = int(rospy.get_param("~baud", "115200"))
                #self.name = name
                self.rate = rospy.get_param("~rate", 100.0)
                self.fake = rospy.get_param("~sim", False)
                self.cal_pan = rospy.get_param("~cal_pan", 0)
                self.cal_tilt = rospy.get_param("~cal_tilt", 0)
                self.cal_lift = rospy.get_param("~cal_lift", 0)
                self.cal_rotate = rospy.get_param("~cal_rotate", 0)
                self.cal_elbow = rospy.get_param("~cal_elbow", 0)
                self.right_tilt = 0
                self.left_tilt = 0
                self.right_lift = 1.57
                self.left_lift = 1.57
                self.left_elbow = 0
               
                #name = rospy.get_param("~name")
                self._Counter = 0

                rospy.init_node('r_shoulder')

                port = rospy.get_param("~port", "/dev/ttyACM0")
                baudRate = int(rospy.get_param("~baudRate", 115200))

                rospy.logwarn("Starting Right shoulder with serial port: " + port + ", baud rate: " + str(baudRate))

                # subscriptions
                
                
                
                rospy.Subscriber('left_arm_tilt_joint/command',Float64, self._HandleJoint_1_Command)
                rospy.Subscriber('right_arm_tilt_joint/command',Float64, self._HandleJoint_2_Command)
                rospy.Subscriber('left_arm_lift_joint/command',Float64, self._HandleJoint_3_Command)
                rospy.Subscriber('right_arm_lift_joint/command',Float64, self._HandleJoint_4_Command)
                rospy.Subscriber('left_arm_elbow_joint/command',Float64, self._HandleJoint_8_Command)
                
                
                self._SerialPublisher = rospy.Publisher('arm_upper', String, queue_size=5)

               
                
                self.P1_MotorPublisher = rospy.Publisher("/left_arm_tilt/motor_state", MotorState, queue_size=5)
                self.P2_MotorPublisher = rospy.Publisher("/right_arm_tilt/motor_state", MotorState, queue_size=5)
                self.P3_MotorPublisher = rospy.Publisher("/left_arm_lift/motor_state", MotorState, queue_size=5)
                self.P4_MotorPublisher = rospy.Publisher("/right_arm_lift/motor_state", MotorState, queue_size=5)
                self.P5_MotorPublisher = rospy.Publisher("/left_arm_elbow/motor_state", MotorState, queue_size=5)

                self._P1_JointPublisher = rospy.Publisher("/left_arm_tilt_joint/state", JointState, queue_size=5)
                self._P2_JointPublisher = rospy.Publisher("/right_arm_tilt_joint/state", JointState, queue_size=5)
                self._P3_JointPublisher = rospy.Publisher("/left_arm_lift_joint/state", JointState, queue_size=5)
                self._P4_JointPublisher = rospy.Publisher("/right_arm_lift_joint/state", JointState, queue_size=5)
                self._P5_JointPublisher = rospy.Publisher("/left_arm_elbow_joint/state", JointState, queue_size=5)

                self._right_lift_Publisher = rospy.Publisher("right_lift", Float32, queue_size=5)
                self._right_tilt_Publisher = rospy.Publisher("right_tilt", Float32, queue_size=5)
                self._left_lift_Publisher = rospy.Publisher("left_lift", Float32, queue_size=5)
                self._left_tilt_Publisher = rospy.Publisher("left_tilt", Float32, queue_size=5)
                self._left_elbow_Publisher = rospy.Publisher("left_elbow", Float32, queue_size=5)
               

               

                self._SerialDataGateway = SerialDataGateway(port, baudRate,  self._HandleReceivedLine)

        def Start(self):
                rospy.loginfo("Starting start function")
                self._SerialDataGateway.Start()
                message = 'r \r'
                self._WriteSerial(message)

        def Stop(self):
                rospy.loginfo("Stopping")
                message = 'r \r'
                self._WriteSerial(message)
                sleep(5)
                self._SerialDataGateway.Stop()
               
        def _HandleJoint_1_Command(self, Command):
                """ Handle movement requests. 
                       left_arm_tilt_joint
                send message in degrees * 10

                """
                v = Command.data      # angel request in radians
                self.left_tilt = v
                v1 =int((90-degrees(v))/1.6)+27
                if v1 < 83: v1 = 83 #ticks
                if v1 > 160: v1 = 160 #ticks 
                rospy.logwarn("Handling left tilt command: " + str(v))
                message = 'j6 %d \r' % (v1)#% self._GetBaseAndExponents((v1)
                rospy.logwarn("Sending left_arm_tilt_joint command: " + (message))
                self._WriteSerial(message)

        def _HandleJoint_2_Command(self, Command):#tilt
                """ Handle movement requests. 
                       right_arm_tilt_joint
                send message in degrees * 10

                """
                v = Command.data      # angel request in radians
                self.right_tilt = v
                v1 =int((90-degrees(v))/1.6)+27
                if v1 < 83: v1 = 83 #ticks
                if v1 > 117: v1 = 117 #ticks      
                #rospy.logwarn(v1)
                message = 'j2 %d \r' % (v1)#% self._GetBaseAndExponents((v1)
                rospy.logwarn("Sending right_arm_tilt_joint command: " + (message))
                self._WriteSerial(message)
               
        def _HandleJoint_3_Command(self, Command):
                """ Handle movement requests. 
                             left_arm_lift_joint
                send message in degrees * 10

                """
                v = Command.data      # angel request in radians
                self.left_lift = v
                v1 =int(1023 -((v + 1.82) * 195.3786081396))#convert encoder value
                if v1 < 100: v1 = 100 #degrees * 10
                if v1 > 1000: v1 = 1000 #degrees * 10
                message = 'j5 %d \r' % (v1)#% self._GetBaseAndExponents((v1)
                rospy.logwarn("Sending left_arm_lift_joint command: " + (message))
                self._WriteSerial(message)
                #self._WriteSerial(message)
               
        def _HandleJoint_4_Command(self, Command):
                """ Handle movement requests. 
                           right_arm_lift_joint
                send message in degrees * 10

                """
                v = Command.data      # angel request in radians
                self.right_lift = v
                #v1 =int(1023 -((v + 1.05) * 195.3786081396))#convert encoder value
                v1 = int((abs(v-1.57)/0.0042913)+511)
                if v1 < 100: v1 = 100 #degrees * 10
                if v1 > 1000: v1 = 1000 #degrees * 10
                rospy.logwarn("Handling lift command: " + str(v1) )

                #message = 's %.2f %.2f %.2f\r' % self._GetBaseAndExponents((v1))
                message = 'j1 %d \r' % (v1)#% self._GetBaseAndExponents((v1)
                rospy.logwarn("Sending new right_arm_lift_joint message: " + (message))
                self._WriteSerial(message)

        def _HandleJoint_8_Command(self, Command):
                """ Handle movement requests. 
                             left_arm_elbow_joint
                send message in degrees * 10

                """
                v = Command.data      # 
                self.left_elbow = v
                v1 =int((90-degrees(v))/0.98)-30
                if v1 < 61: v1 = 61 #ticks
                if v1 > 165: v1 = 167 #ticks
                message = 'j8 %d \r' % (v1)#% self._GetBaseAndExponents((v1)
                rospy.logwarn("Sending left_arm_elbow_joint command: " + (message))
                self._WriteSerial(message) 
               

      
                                            
 

        def _GetBaseAndExponent(self, floatValue, resolution=4):
                '''
                Converts a float into a tuple holding two integers:
                The base, an integer with the number of digits equaling resolution.
                The exponent indicating what the base needs to multiplied with to get
                back the original float value with the specified resolution.
                '''

                if (floatValue == 0.0):
                        return (0, 0)
                else:
                        exponent = int(1.0 + math.log10(abs(floatValue)))
                        multiplier = math.pow(10, resolution - exponent)
                        base = int(floatValue * multiplier)

                        return(base, exponent - resolution)

        def _GetBaseAndExponents(self, floatValues, resolution=4):
                '''
                Converts a list or tuple of floats into a tuple holding two integers for each float:
                The base, an integer with the number of digits equaling resolution.
                The exponent indicating what the base needs to multiplied with to get
                back the original float value with the specified resolution.
                '''

                baseAndExponents = []
                for floatValue in floatValues:
                        baseAndExponent = self._GetBaseAndExponent(floatValue)
                        baseAndExponents.append(baseAndExponent[0])
                        baseAndExponents.append(baseAndExponent[1])

                return tuple(baseAndExponents)


if __name__ == '__main__':
        r_shoulder = R_shoulder()
        try:
                r_shoulder.Start()
                rospy.spin()

        except rospy.ROSInterruptException:
                r_shoulder.Stop()

