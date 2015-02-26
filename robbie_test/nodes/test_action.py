#!/usr/bin/env python

"""
    
"""

import rospy
from actionlib import SimpleActionClient
from move_base_msgs.msg import *
from actionlib_msgs.msg import *
from geometry_msgs.msg import *

class ActionTasks:
    def __init__(self, script_path):
        rospy.init_node('action_testr')
        rospy.on_shutdown(self.cleanup)
        self.fridge = (Pose(Point(0.295, -2.304, 0.0), Quaternion(0.0, 0.0,  -0.715, 0.699))) 

        self.client = SimpleActionClient("move_base", MoveBaseAction)
        self.client.wait_for_server()
        self.move_to(self.fridge)


    def move_to(self, location):
        goal = MoveBaseGoal()
        goal.target_pose.pose = location
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
  
        self.client.send_goal(goal)
        #self.client.wait_for_result()
        self.client.wait_for_result(rospy.Duration.from_sec(40.0))
        

        if self.client.get_state() == GoalStatus.SUCCEEDED:
            result = self.client.get_result()
            print "Result: SUCCEEDED " 
        elif self.client.get_state() == GoalStatus.PREEMPTED:
            print "Action pre-empted"
        else:
            print "Action failed"


    def cleanup(self):
        
        rospy.loginfo("Shutting down talk node...")

if __name__=="__main__":
    try:
        ActionTasks(sys.path[0])
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Talk node terminated.")
