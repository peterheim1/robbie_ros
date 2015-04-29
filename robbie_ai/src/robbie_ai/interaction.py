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
from face_recognition.msg import *
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


from tf import transformations

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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

class Commands:
    """ 
    a class to run commands 
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

class Questions:
    """ 
    a class to answer questions 
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
        self.voice = rospy.get_param("~voice", "voice_en1_mbrola")
        # Create the sound client object
        self.soundhandle = SoundClient()
        
        # Wait a moment to let the client connect to the
        # sound_play server
        rospy.sleep(1)
        
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()

        #set action server
        self.client = SimpleActionClient('face_recognition', face_recognition.msg.FaceRecognitionAction)

        # listening for goals.
        self.client.wait_for_server()
        rospy.sleep(2)

        #result will be published on this topic
        rospy.Subscriber("/face_recognition/result", FaceRecognitionActionFeedback, self.face_found)
        rospy.Subscriber("/face_recognition/feedback", FaceRecognitionActionFeedback, self.Unknown)
        self.look_for_face()
    def look_for_face(self):
        '''
        send command look for face once
        '''
        goal = face_recognition.msg.FaceRecognitionGoal(order_id=0, order_argument="none")
        self.client.send_goal(goal)
        #self.client.wait_for_result(rospy.Duration.from_sec(6.0))
        #if self.client.get_state() == GoalStatus.FAILED:
            #self.unknown()

    def face_found(self,msg):
        '''
        clean up feedback to extract the name
        check if greeted before answer accordingly
        '''
        

        person = str(msg.result.names[0])
        rospy.logwarn(person)
        
        self.soundhandle.say("hello   "+ person ,self.voice)
        #self.task1 = "hello" + person
        #rospy.sleep(2)
        #return the processed value
        self.value = str(self.task1)

    def Unknown(self,msg):
        self.soundhandle.say("hello     what is your name   " ,self.voice)

    def __str__(self):
        return 

class HelloUnknown:
    """ 
    a class to recognise a new person 
    """
    def __init__(self):
        #set action server
        
        self.task1 ="hello what is your name"
        rospy.sleep(2)
        #return the processed value
        self.value = str(self.task1)
    def __str__(self):
        return str(self.value)


class Move_Head:
    """ 
    a class to fins and pick up a object 
    """
    def __init__(self, text):
        #self.task = text#whole message from greeting
        self.head_client = SimpleActionClient('head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.head_client.wait_for_server()

        ar = text.split(':')
        cmd = ar[0]
        direction = ar[5]
        if direction =='left':
            self.move_head(1.1, 0.0)
        if direction =='right':
            self.move_head(-1.1, 0.0)
        if direction =='front':
            self.move_head(0.0, 0.0)

    def move_head(self, pan , tilt):   
        # Which joints define the head?
        head_joints = ['head_pan_joint', 'head_tilt_mod_joint']
        # Create a single-point head trajectory with the head_goal as the end-point
        head_trajectory = JointTrajectory()
        head_trajectory.joint_names = head_joints
        head_trajectory.points.append(JointTrajectoryPoint())
        head_trajectory.points[0].positions = pan , tilt
        head_trajectory.points[0].velocities = [0.0 for i in head_joints]
        head_trajectory.points[0].accelerations = [0.0 for i in head_joints]
        head_trajectory.points[0].time_from_start = rospy.Duration(3.0)
    
        # Send the trajectory to the head action server
        rospy.loginfo('Moving the head to goal position...')
        
        head_goal = FollowJointTrajectoryGoal()
        head_goal.trajectory = head_trajectory
        head_goal.goal_time_tolerance = rospy.Duration(0.0)
    
        # Send the goal
        self.head_client.send_goal(head_goal)
        
        # Wait for up to 5 seconds for the motion to complete 
        self.head_client.wait_for_result(rospy.Duration(2.0))
        



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


class ShowTime:
    def __init__(self):
        #rospy.init_node('show_time')
        

        # Set the default TTS voice to use
        self.voice = rospy.get_param("~voice", "voice_en1_mbrola")
        self.robot = rospy.get_param("~robot", "robbie")
        self.start =(Pose(Point(1.7, 0, 0.0), Quaternion(0.0, 0.0, 1.000, 0.018)))
        self.left =(Pose(Point(1.960, -1.854, 0.0), Quaternion(0.0, 0.0, 0.916, 0.401)))
        self.right =(Pose(Point(2.123, 1.592, 0.0), Quaternion(0.0, 0.0, 0.877, -0.480)))
        self.auto_dock =(Pose(Point(0.5, -0.01, 0.0), Quaternion(0.0, 0.0, 0.002, 1.000)))
        self.pub = rospy.Publisher('auto_dock', String, queue_size=1)

        

        self.client = SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        # Create the sound client object
        self.soundhandle = SoundClient()
        
        # Wait a moment to let the client connect to the
        # sound_play server
        rospy.sleep(2)
        
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()
        
        self.noon = strftime("%p:", localtime())
        if self.noon == "AM:":
            self.noon1 = "Goood Morning  "
        else:
            self.noon1 ="Good After Noon   "
        self.local = strftime("%H:%M:", localtime())
        #local time
        self.local = strftime("%H:%M:", localtime())

        self.soundhandle.say(self.noon1 + self.robot +"   is on line" + " the time is   " + self.local, self.voice)

        rospy.sleep(6)
        self.move_to(self.start)
        self.soundhandle.say("Welcome to SHOW time.     This is where I get to demonstrate my capabilities" , self.voice)
        rospy.sleep(5)

        self.soundhandle.say("I can move to my left", self.voice)
        rospy.sleep(2)
        self.move_to(self.left)
        
        self.soundhandle.say("now back to the start position", self.voice)
        rospy.sleep(2)
        self.move_to(self.start)
        
        self.soundhandle.say("I can move to my right", self.voice)
        rospy.sleep(2)
        self.move_to(self.right)
        
        self.soundhandle.say("And again back to the start position", self.voice)
        rospy.sleep(2)
        self.move_to(self.start)

        rospy.sleep(2)
        self.soundhandle.say("Thank you for your time ", self.voice)
        
        

        rospy.sleep(2)

        #rospy.sleep(2)
        self.move_to(self.auto_dock)
        #autodock
        rospy.sleep(2)
        self.pub.publish("dock")

    def move_to(self, location):
        goal = MoveBaseGoal()
        goal.target_pose.pose = location
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
  
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(50.0))

        if self.client.get_state() == GoalStatus.SUCCEEDED:
            result = self.client.get_result()
            print "Result: SUCCEEDED " 
        elif self.client.get_state() == GoalStatus.PREEMPTED:
            print "Action pre-empted"
        else:
            print "Action failed"


class Driver(object):
	'''
	Implements the logic for driving a given distance or turning for a given amount
	by monitoring the transform messages that contain the odometry based pose.
	'''

	def __init__(self):
		
		
		self._VelocityCommandPublisher = rospy.Publisher("cmd_vel", Twist)
		self._TransformListener = tf.TransformListener()

		# wait for the listener to get the first transform message
		self._TransformListener.waitForTransform("/odom", "/base_footprint", rospy.Time(), rospy.Duration(4.0))

	def driveX(self, distance, speed):
		'''
		Drive in x direction a specified distance based on odometry information
		distance [m]: the distance to travel in the x direction (>0: forward, <0: backwards)
		speed [m/s]: the speed with which to travel; must be positive
		'''

		forward = (distance >= 0)
		
		# record the starting transform from the odom to the base_footprint frame
		# Note that here the 'from' frame precedes 'to' frame which is opposite to how they are
		# ordered in tf.TransformBroadcaster's sendTransform function.
		# startTranslation is a tuple holding the x,y,z components of the translation vector
		# startRotation is a tuple holding the four components of the quaternion
		(startTranslation, startRotation) = self._TransformListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
		
		done = False

		velocityCommand = Twist()
		if forward:
			velocityCommand.linear.x = speed # going forward m/s
		else:
			velocityCommand.linear.x = -speed # going forward m/s
			
		velocityCommand.angular.z = 0.0 # no angular velocity

		while not rospy.is_shutdown():
			try:
				(currentTranslation, currentRotation) = self._TransformListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
				
				dx = currentTranslation[0] - startTranslation[0]
				dy = currentTranslation[1] - startTranslation[1]
				
				distanceMoved = math.sqrt(dx * dx + dy * dy)
				#print distanceMoved
				if (forward):
					arrived = distanceMoved >= distance
				else:
					arrived = distanceMoved >= -distance
				
				if (arrived):
					break
				else:
					# send the drive command
					#print("sending vel command" + str(velocityCommand))
					self._VelocityCommandPublisher.publish(velocityCommand)
				
			except (tf.LookupException, tf.ConnectivityException):
				continue

			rospy.sleep(0.1)

		#stop
		velocityCommand.linear.x = 0.0
		velocityCommand.angular.z = 0.0
		self._VelocityCommandPublisher.publish(velocityCommand)
		
		return done


	def turn(self, angle, angularSpeed):
		'''
		Turn the robot based on odometry information
		angle [rad]: the angle to turn (positive angles mean clockwise rotation)
		angularSpeed [rad/s]: the speed with which to turn; must be positive
		'''

		ccw = (angle >= 0) # counter clockwise rotation
		
		# record the starting transform from the odom to the base frame
		# Note that here the 'from' frame precedes 'to' frame which is opposite to how they are
		# ordered in tf.TransformBroadcaster's sendTransform function.
		(startTranslation, startRotation) = self._TransformListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
		startAngle = 2 * math.atan2(startRotation[2], startRotation[3])

		#print "start angle: " + str(startAngle)
		previousAngle = startAngle
		angleOffset = 0.0
		
		done = False

		velocityCommand = Twist()
		velocityCommand.linear.x = 0.0 # going forward m/s
		if ccw:
			velocityCommand.angular.z = angularSpeed
		else:
			velocityCommand.angular.z = -angularSpeed

		while not rospy.is_shutdown():
			try:
				(currentTranslation, currentRotation) = self._TransformListener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
				currentAngle = 2 * math.atan2(currentRotation[2], currentRotation[3])
				#print "currentAngle: " + str(currentAngle)
				
				# we need to handle roll over of the angle
				if (currentAngle * previousAngle < 0 and math.fabs(currentAngle - previousAngle) > math.pi / 2):
					if (currentAngle > previousAngle):
						#print "subtracting"
						angleOffset = angleOffset - 2 * math.pi
					else:
						#print "adding"
						angleOffset = angleOffset + 2 * math.pi
				
				angleTurned = currentAngle + angleOffset - startAngle
				previousAngle = currentAngle
				
				#print "angleTurned: " + str(angleTurned)
				if (ccw):
					arrived = (angleTurned >= angle)
				else:
					arrived = (angleTurned <= angle)
				
				#print arrived

				if (arrived):
					break
				else:
					# send the drive command
					#print("sending vel command" + str(velocityCommand))
					self._VelocityCommandPublisher.publish(velocityCommand)
				
			except (tf.LookupException, tf.ConnectivityException):
				continue

			time.sleep(0.1)

		#stop
		velocityCommand.linear.x = 0.0
		velocityCommand.angular.z = 0.0
		self._VelocityCommandPublisher.publish(velocityCommand)
		
		return done

    

