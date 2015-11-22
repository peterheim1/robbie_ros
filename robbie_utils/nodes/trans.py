#!/usr/bin/env python

"""

"""

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped, Point, Pose, PoseStamped
from math import radians
import tf

class HeadTrans():
    def __init__(self):
        # Initialize the node
        rospy.init_node("head_tf")
         

        # Subscribe the the 'joint_states' topic so we can know how the joints are positioned
        rospy.loginfo("Subscribing to joint_states...")
        
        self.joint_state = JointState()
        self.target = PointStamped()
        rospy.Subscriber('joint_states', JointState, self.update_joint_state)
        # Wait until we actually have joint state values
        while self.joint_state == JointState():
            rospy.sleep(1)
        current_tilt = self.joint_state.position[self.joint_state.name.index('head_tilt_mod_joint')]
        #current_pan = self.joint_state.position[self.joint_state.name.index('head_pan_joint')]  
        #print   current_tilt + '      ' + current_pan
        #rospy.loginfo(current_tilt)
        
        # Wait for the target topic to become alive
        rospy.loginfo("Waiting for target topic...")
        
        rospy.wait_for_message('target_pose', PoseStamped)

        rospy.Subscriber('target_pose', PoseStamped, self.update_joint_positions)
        rospy.loginfo(self.target.point)
 
    def update_joint_positions(self, msg):
        # Acquire the lock
        #self.lock.acquire()
        
        self.target.header.frame_id = msg.header.frame_id
        self.target.point = msg.pose.position
        rospy.loginfo(self.target.point)
        

        

    def update_joint_state(self, msg):
        #test = msg.name.index('head_pan_joint')
        #rospy.loginfo(msg)
        self.joint_state = msg
        
        


if __name__ == '__main__':
    try:
        HeadTrans()
    except rospy.ROSInterruptException:
        rospy.loginfo("Head tracking node terminated.")
