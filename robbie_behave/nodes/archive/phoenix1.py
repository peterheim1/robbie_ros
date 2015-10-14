#!/usr/bin/env python
"""
the start of Robbies nervious system
primitives include 
		stay healthy
		emotions
		memory
stay healthy works but only a place holder here


"""

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from pi_trees_ros.pi_trees_ros import *


class Phoenix():
    def __init__(self):
        rospy.init_node("phoenix_tree")

        # Create the root node
        BEHAVE = Sequence("BEHAVE")

        # Create the "stay healthy" selector
        STAY_HEALTHY = Selector("STAY_HEALTHY")
        NAVIGATION = Selector("NAVIGATION")
        GESTURE = Selector("GESTURE")
        EMOTION = Selector("EMOTION")

        # Add the subtrees to the root node in order of priority
        BEHAVE.add_child(STAY_HEALTHY)
        BEHAVE.add_child(NAVIGATION)
        BEHAVE.add_child(GESTURE)
        BEHAVE.add_child(EMOTION)

        # Display the tree before beginning execution
        print "Patrol Behavior Tree"
        print_tree(BEHAVE)

        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.run()
            rospy.sleep(0.1)



    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_all_goals()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    tree = Phoenix()


