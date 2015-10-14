#!/usr/bin/env python
'''

'''
import rospy

from pi_trees_ros.pi_trees_ros import *
#from utilities import *
from std_msgs.msg import Float32, String, Int16,Float64
from rbx2_msgs.srv import *
from rbx2_tasks.task_setup import *
from robbie_msgs.msg import RobbieCmd
from robbie_behave.task import *
from robbie_behave.interaction import *
import time
import os
import math
from tf.transformations import quaternion_from_euler
from actionlib import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from sound_play.libsoundplay import SoundClient
        

# Initialize the black board
black_board = BlackBoard()



class PhoenixControl():
    def __init__(self):
        rospy.init_node("Phoenix_Control")
        rm= Robbie_memory()#from utilities
        rm.MeEmotion_write(0,0)#PUT EMOTION STATE TO 0
        self.head_move = Move_head()
        #Move_arm('right_start')
        #Move_arm('left_start')
        
        #rm= Robbie_memory()#from utilities
        # Initialize a number of parameters and variables
        setup_task_environment(self)
        # Set the default TTS voice to use
        self.voice = rospy.get_param("~voice", "voice_en1_mbrola")
        self.robot = rospy.get_param("~robot", "robbie")
        # Create the sound client object
        self.soundhandle = SoundClient()
        
        # Wait a moment to let the client connect to the
        # sound_play server
        rospy.sleep(1)
        
        # Make sure any lingering sound_play processes are stopped.
        self.soundhandle.stopAll()
        self.head_move.head_move(0.0,0.0)
        
       
        # The root node when all top levels tasks have SUCCESS: program exits
        BEHAVE = Sequence("behave")

        # Create the top level tasks in order.  Sequence executes each of its child behaviors until one of them fails
        # Create the "stay healthy" selector
        STAY_HEALTHY = Selector("STAY_HEALTHY")
        GOTO = Selector("GOTO")
        TIMER = Selector("TIMER")
        HEAD = Selector("HEAD")

        # Add the subtrees to the root node in order of priority
        BEHAVE.add_child(STAY_HEALTHY)
        BEHAVE.add_child(TIMER)
        BEHAVE.add_child(GOTO)
        BEHAVE.add_child(HEAD)
        

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
            AUTODOCK = AutoDock()
            CHARGED = MonitorTask("CHARGED", "charge_state", Float32, self.battery_state)
            
            # Build the recharge sequence using inline construction
            RECHARGE = Sequence("RECHARGE", [NAV_DOCK_TASK, AUTODOCK])
                
            # Add the check battery and recharge tasks to the stay healthy selector
            STAY_HEALTHY.add_child(CHECK_BATTERY)
            STAY_HEALTHY.add_child(RECHARGE)
            

        with GOTO:#rename to commands
            MOVE_TOO = MonitorTask("MOVE_TOO", "move_to", RobbieCmd, self.Move_To)
            
            GOTO.add_child(MOVE_TOO)

        with TIMER:#runs each task until one fails
            CLOCK = Clock1(60)#will check if greater than 60 secsince last activity
            EMOTION = Emotion1()#will check emotions after clock finishes

            TIMER.add_child(CLOCK)
            TIMER.add_child(EMOTION)

        with HEAD:# MOVE THE HEAD
            LOOK_AT = MonitorTask("LOOK_AT", "look_at", RobbieCmd, self.Look_At)

            HEAD.add_child(LOOK_AT)
      
        print "Behavior Tree Structure"
        print_tree(BEHAVE)

        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.run()
            rospy.sleep(0.1)

    def Look_At(self, msg):
        if msg is None:
            return TaskStats.RUNNING
        else:
            print msg.x
            print msg.y
            self.head_move.head_move(msg.x,msg.y)
            
       
            
    def check_battery(self, msg):
        if msg.data is None:
            return TaskStatus.RUNNING
        else:
            if msg.data < 11.0:
                #rospy.loginfo("LOW BATTERY - level: " + str(int(msg.data)))
                rm.E_Update(-0.1,0)               
                return TaskStatus.FAILURE
            else:
                return TaskStatus.SUCCESS

    def battery_state(self, msg):
        #if msg.data is None:
            #return TaskStatus.RUNNING
        #else:
            if msg.data < 13.5:
                #rospy.loginfo("BATTERY charging - level: " + str(int(msg.data)))
                return TaskStatus.RUNNING
            else:
                rospy.loginfo("Battery charged - level: " + str(int(msg.data)))
                rm.E_Update(0.5,0.5) 
                return TaskStatus.SUCCESS
    
    def recharge_cb(self, result):
        rm.E_Update(0.6,0.5)
        rospy.loginfo("BATTERY CHARGED!")
        self.soundhandle.say("I'm full now    ", self.voice)

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
            #print 'GOING TO   '+str(goal)
            rm.E_Update(0.5,0.5)
            self.move_base.send_goal(goal)
            self.move_base.wait_for_result(rospy.Duration.from_sec(60.0))
            if self.move_base.get_state() == GoalStatus.SUCCEEDED:
                result = self.move_base.get_result()
                print "Result: SUCCEEDED " 
                self.soundhandle.say("Success   ", self.voice)
            elif self.move_base.get_state() == GoalStatus.PREEMPTED:
                print "Action pre-empted"
                self.soundhandle.say("I was foiled again  ", self.voice)
            else:
                print "Action failed"
                self.soundhandle.say("Huston we have a problem        ", self.voice)
                self.shutdown()
            return TaskStatus.FAILURE
            
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_all_goals()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)



if __name__ == '__main__':
    tree = PhoenixControl()
