#!/usr/bin/env python
'''

'''
import rospy

from pi_trees_ros.pi_trees_ros import *
from task import *
#from utilities import *
from std_msgs.msg import Float32, String, Int16,Float64
from rbx2_msgs.srv import *
from rbx2_tasks.task_setup import *
from robbie_msgs.msg import RobbieCmd
import time
import math
from tf.transformations import quaternion_from_euler
from actionlib import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist

        

# Initialize the black board
black_board = BlackBoard()



class PhoenixControl():
    def __init__(self):
        rospy.init_node("Phoenix_Control")
        rm= Robbie_memory()#from utilities
        rm.MeEmotion_write(0,0)#PUT EMOTION STATE TO 0
        
        #rm= Robbie_memory()#from utilities
        # Initialize a number of parameters and variables
        setup_task_environment(self)
        
       
        # The root node when all top levels tasks have SUCCESS: program exits
        BEHAVE = Sequence("behave")

        # Create the top level tasks in order.  Sequence executes each of its child behaviors until one of them fails
        # Create the "stay healthy" selector
        STAY_HEALTHY = Selector("STAY_HEALTHY")
        GOTO = Selector("GOTO")
        

        # Add the subtrees to the root node in order of priority
        BEHAVE.add_child(STAY_HEALTHY)
        BEHAVE.add_child(GOTO)
        

        # Set the docking station pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.docking_station_pose

        NAV_DOCK_TASK = SimpleActionTask("NAV_DOC_TASK", "move_base", MoveBaseAction, goal, result_timeout=50, reset_after=True)

        # Add the battery check and recharge tasks to the "stay healthy" task
        with STAY_HEALTHY:
            # The check battery condition (uses MonitorTask)
            CHECK_BATTERY = MonitorTask("CHECK_BATTERY", "battery_level", Float32, self.check_battery)
            
            # The charge robot task (uses ServiceTask)
            CHARGE_ROBOT = ServiceTask("CHARGE_ROBOT", "battery_simulator/set_battery_level", SetBatteryLevel, 100, result_cb=self.recharge_cb)
      
            # Build the recharge sequence using inline construction
            RECHARGE = Sequence("RECHARGE", [NAV_DOCK_TASK, CHARGE_ROBOT])
                
            # Add the check battery and recharge tasks to the stay healthy selector
            STAY_HEALTHY.add_child(CHECK_BATTERY)
            STAY_HEALTHY.add_child(RECHARGE)
            

        with GOTO:#rename to commands
            #GOTO_LOCATION = MonitorTask("GOTO_LOCATION", "nav_to", String, self.goto_location)
            MOVE_TOO = MonitorTask("MOVE_TOO", "move_to", RobbieCmd, self.Move_To)
            #TIMER1 = Timer1()
            #TIMER5 = Timer5()
       
            #GOTO.add_child(GOTO_LOCATION)
            GOTO.add_child(MOVE_TOO)

      
        print "Behavior Tree Structure"
        print_tree(BEHAVE)

        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.run()
            rospy.sleep(0.1)
            
       
            
    def check_battery(self, msg):
        if msg.data is None:
            return TaskStatus.RUNNING
        else:
            if msg.data < self.low_battery_threshold:
                #rospy.loginfo("LOW BATTERY - level: " + str(int(msg.data)))
                rm.E_Update(-0.1,0)               
                return TaskStatus.FAILURE
            else:
                return TaskStatus.SUCCESS
    
    def recharge_cb(self, result):
        rm.E_Update(0.6,0.5)
        rospy.loginfo("BATTERY CHARGED!")
    #all navigation and movement done through here
    def Move_To(self, msg):
        if msg is None:
            return TaskStats.RUNNING
        else:
            q_angle = quaternion_from_euler(0, 0, math.radians(msg.z), axes='sxyz')#convert degrees to querterion
            q = Quaternion(*q_angle)
            x = msg.x
            y = msg.y
            
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = msg.command
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = (Pose(Point(x, y, 0.0), Quaternion(*q_angle))) #self.room_locations[msg.data]
            print 'GOING TO   '+str(goal)
            rm.E_Update(0.5,0.5)
            self.move_base.send_goal(goal)
            self.move_base.wait_for_result(rospy.Duration.from_sec(40.0))
            if self.move_base.get_state() == GoalStatus.SUCCEEDED:
                result = self.move_base.get_result()
                print "Result: SUCCEEDED " 
            elif self.move_base.get_state() == GoalStatus.PREEMPTED:
                print "Action pre-empted"
            else:
                print "Action failed"
                self.shutdown()
            return TaskStatus.FAILURE
            
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_all_goals()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)



if __name__ == '__main__':
    tree = PhoenixControl()
