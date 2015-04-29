#!/usr/bin/env python

"""
    
"""

import rospy
from geometry_msgs.msg import PointStamped
import tf

class ActionTasks:
    def __init__(self):
        rospy.init_node('head_test')
        rospy.on_shutdown(self.cleanup)
        # Define the target as a PointStamped message

        # Initialize tf listener       
        tf_listener = tf.TransformListener()
        
        
        # Define the target publisher
        self.target_pub = rospy.Publisher('/target_point', PointStamped, queue_size=1)

        self.move_head()

    def move_head(self):
        target = PointStamped()
        target.header.frame_id = 'base_footprint'
        target.point.x = 1
        target.point.y = 0.3
        target.point.z = 1.0   
        target.header.stamp = rospy.Time.now()

        self.target_pub.publish(target)
        rospy.loginfo(target)

    def cleanup(self):
        
        rospy.loginfo("Shutting head_test node...")

if __name__=="__main__":
    try:
        ActionTasks()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("head_test node terminated.")
