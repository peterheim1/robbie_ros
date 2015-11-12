#!/usr/bin/env python

import rospy
import tf
#import math
from math import sin, cos, pi, radians, degrees
import sys
import time
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_msgs.msg import Float32, Int16, Float64
from dynamixel_msgs.msg import MotorState
from dynamixel_msgs.msg import JointState
from SerialDataGateway import SerialDataGateway

class Arduino(object):
	'''
	Helper class for communicating with an Arduino board over serial port
	'''

	
	def _HandleReceivedLine(self,  line):
                self._Counter = self._Counter + 1
                #rospy.logwarn(str(self._Counter) + " " + line)
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
                        P1 = radians(float(lineParts[1])*0.2)
                        P2 = self.left_tilt
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
                        
                        P1 = radians(float(lineParts[1])*0.2)
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
                        P1 = radians(float(lineParts[1])*0.1)
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
                        P1 = radians(float(lineParts[1])*0.1)
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
                        
                        
                        P1 = 0 - (radians(float(lineParts[1])*0.1))
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

                

	def __init__(self, port="/dev/ttyACM0", baudrate=115200):
		'''
		Initializes the receiver class. 
		port: The serial port to listen to.
		baudrate: Baud rate for the serial communication
		'''

		self._Counter = 0
                self.right_tilt = 0
                self.left_tilt = 0
                self.right_lift = 1.57
                self.left_lift = 1.57
                self.left_elbow = 0

		rospy.init_node('arduino')

		port = rospy.get_param("~port", "/dev/ttyACM0")
		baudRate = int(rospy.get_param("~baudRate", 115200))

		rospy.loginfo("Starting with serial port: " + port + ", baud rate: " + str(baudRate))
                # subscriptions
                
                rospy.Subscriber('right_arm_tilt_joint/command',Float64, self.right_tilt_Command)
                rospy.Subscriber('left_arm_tilt_joint/command',Float64, self.left_tilt_Command)
                rospy.Subscriber('left_arm_lift_joint/command',Float64, self.left_lift_Command)
                rospy.Subscriber('right_arm_lift_joint/command',Float64, self.right_lift_Command)
                rospy.Subscriber('left_arm_elbow_joint/command',Float64, self.left_elbow_Command)
                self._SerialPublisher = rospy.Publisher('arm_upper', String)
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

        def right_tilt_Command(self, Command):
                """ Handle movement requests. 
                       right_arm_tilt_joint
                send message in degrees 

                """
                v = Command.data      # angel request in radians
                self.right_tilt = v
                v1 =abs(int(degrees(v)))
                rospy.logwarn("Handling right tilt command: " + str(v1))
                x = 'a %d \r' % (v1)
                self._WriteSerial(x)

        def left_tilt_Command(self, Command):
                """ Handle movement requests. 
                       left_arm_tilt_joint
                send message in degrees 

                """
                v = Command.data      # angel request in radians
                self.left_tilt = v
                v1 =abs(int(degrees(v)))
                rospy.logwarn("Handling left tilt command: " + str(v1))
                x = 'b %d \r' % (v1)
                self._WriteSerial(x)

        def right_lift_Command(self, Command):
                """ Handle movement requests. 
                           right_arm_lift_joint
                send message in degrees * 10

                """
                v = Command.data      # angel request in radians
                self.right_lift = v
                #v1 =int(1023 -((v + 1.05) * 195.3786081396))#convert encoder value
                v1 =abs(int(degrees(v))-90)
                rospy.logwarn("Handling right lift command: " + str(v1))
                x = 'c %d \r' % (v1)
                self._WriteSerial(x)

        def left_lift_Command(self, Command):
                """ Handle movement requests. 
                             left_arm_lift_joint
                send message in degrees * 10

                """
                v = Command.data      # angel request in radians
                self.left_lift = v
                v1 =abs(int(degrees(v))-90)
                rospy.logwarn("Handling left lift command: " + str(v1))
                x = 'd %d \r' % (v1)
                self._WriteSerial(x)

        def left_elbow_Command(self, Command):
                """ Handle movement requests. 
                       left_arm_elbow_joint
                send message in degrees

                """
                v = Command.data      # angel request in radians
                self.left_elbow = v
                v1 =abs(int(degrees(v)))
                rospy.logwarn("Handling left elbow command: " + str(v1))
                x = 'e %d \r' % (v1)
                self._WriteSerial(x)

	def Start(self):
		rospy.logdebug("Starting")
		self._SerialDataGateway.Start()

	def Stop(self):
		rospy.logdebug("Stopping")
		self._SerialDataGateway.Stop()


if __name__ == '__main__':
	arduino = Arduino()
	try:
		arduino.Start()
		rospy.spin()

	except rospy.ROSInterruptException:
		arduino.Stop()


