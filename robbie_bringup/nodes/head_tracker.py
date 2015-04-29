#! /usr/bin/env python

# Copyright (c) 2010, Arizona Robotics Research Group, University of Arizona
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Anh Tran

# Based on the wubble_head_action.py script by Anh Tran. Modifications made by Patrick Goebel
# for the Pi Robot Project.

import rospy
import tf
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
from dynamixel_controllers.srv import *

import math

class PointHead():

    def __init__(self):
        # Initialize new node
        rospy.init_node('point_head_node', anonymous=True)
        
        rospy.on_shutdown(self.shutdown)
        
        rate = rospy.get_param('~rate', 20)
        self.default_joint_speed = rospy.get_param('~default_joint_speed', 0.6)
        r = rospy.Rate(rate)
        
        dynamixels = rospy.get_param('dynamixels', '')
                
        # Remap these in the launch file or command line if necessary
        self.camera_link = 'head_cam_link'
        self.head_pan_joint = 'head_pan_joint'
        self.head_tilt_joint = 'head_tilt_joint'
        self.head_pan_link = 'head_pan_link'
        self.head_tilt_link = 'head_tilt_link'
        
        self.servo_speed = dict()
        self.servo_position = dict()
        self.torque_enable = dict()

        """ Connect to the set_speed and torque_enable services for each servo.
            Also define a position publisher for each servo. """
        for name in sorted(dynamixels):
            try:
                controller = name
                #rospy.loginfo(controller)
                # The set_speed services
                set_speed_service = '/' + controller + '/set_speed'
                rospy.wait_for_service(set_speed_service)  
                self.servo_speed[name] = rospy.ServiceProxy(set_speed_service, SetSpeed, persistent=True)
                
                # Initialize the servo speed to the default_joint_speed
                self.servo_speed[name](self.default_joint_speed)
                
                # Torque enable/disable control for each servo
                torque_service = '/' + controller + '/torque_enable'
                rospy.wait_for_service(torque_service) 
                self.torque_enable[name] = rospy.ServiceProxy(torque_service, TorqueEnable)
                
                # Start each servo in the disabled state so we can move them by hand
                self.torque_enable[name](False)
    
                # The position controllers
                self.servo_position[name] = rospy.Publisher('/' + controller + '/command', Float64)
            except:
                rospy.loginfo("Can't contact servo services!")
        
        # Initialize the target point
        self.target_point = PointStamped()
        self.last_target_point = PointStamped()
        
        # Subscribe to the target_point topic
        rospy.Subscriber('/target_point', PointStamped, self.update_target_point)

        # Initialize tf listener
        self.tf = tf.TransformListener()
        
        # Give the tf buffer a chance to fill up
        rospy.sleep(2)
        
        self.tf.waitForTransform(self.camera_link, self.head_pan_link, rospy.Time(), rospy.Duration(1.0))
            
        # Reset the head position to neutral
        rospy.sleep(1)
        self.center_head()
        
        rospy.loginfo("Ready to track target point")
        
        while not rospy.is_shutdown():
            rospy.wait_for_message('/target_point', PointStamped)
            if self.target_point == self.last_target_point:
                continue
            try:
                target_angles = self.transform_target_point(self.target_point)
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.logerr("tf Failure")
                continue
                
            self.servo_position[self.head_pan_joint].publish(target_angles[0])
            self.servo_position[self.head_tilt_joint].publish(target_angles[1])
            
            self.last_target_point = self.target_point
            
            r.sleep()
        
    def update_target_point(self, msg):
        self.target_point = msg

    def center_head(self):
        self.servo_position[self.head_pan_joint].publish(0.0)
        self.servo_position[self.head_tilt_joint].publish(0.0)
        rospy.sleep(3)

    def transform_target_point(self, target):   
        # Wait for tf info (time-out in 5 seconds)
        self.tf.waitForTransform(self.head_pan_link, target.header.frame_id, rospy.Time.now(), rospy.Duration(5.0))
        self.tf.waitForTransform(self.head_tilt_link, target.header.frame_id, rospy.Time.now(), rospy.Duration(5.0))

        # Transform target point to pan reference frame & retrieve the pan angle
        pan_target = self.tf.transformPoint(self.head_pan_link, target)
        pan_angle = math.atan2(pan_target.point.y, pan_target.point.x)

        # Transform target point to tilt reference frame & retrieve the tilt angle
        tilt_target = self.tf.transformPoint(self.head_tilt_link, target)
        tilt_angle = math.atan2(tilt_target.point.z,
                math.sqrt(math.pow(tilt_target.point.x, 2) + math.pow(tilt_target.point.y, 2)))

        return [pan_angle, tilt_angle]
    
    def shutdown(self):
        rospy.loginfo("Shutting down point head node...")
        self.center_head()

if __name__ == '__main__':
    try:
        point_head = PointHead()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


