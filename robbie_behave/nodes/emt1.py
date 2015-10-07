#!/usr/bin/env python
'''
remove stay healthy and goto location for test

'''
import rospy
#from pi_trees_lib.pi_trees_lib import *
from pi_trees_ros.pi_trees_ros import *
from task import *
#from utilities import *
from std_msgs.msg import Float32, String, Int16
from rbx2_msgs.srv import *
from rbx2_tasks.task_setup import *
import time
import markovian


        

# Initialize the black board
black_board = BlackBoard()



class PhoenixControl():
    def __init__(self):
        rospy.init_node("Phoenix_Control")
        rm= Robbie_memory()#from utilities
        rm.MeEmotion_write(0,0)#PUT EMOTION STATE TO 0
        file_ = open('suntzu.txt')
        markov = markovian.Markov(file_)
        markov.generate_markov_text()
        
        
        #rm= Robbie_memory()#from utilities
        # Initialize a number of parameters and variables
        setup_task_environment(self)
        
       
        # The root node when all top levels tasks have SUCCESS: program exits
        BEHAVE = Sequence("behave")

        # Create the top level tasks in order.  Sequence executes each of its child behaviors until one of them fails
        # Create the "stay healthy" selector
        #STAY_HEALTHY = Selector("STAY_HEALTHY")
        TIMER = Selector("TIMER")
        POSE = Sequence("POSE")
        GOTO = Selector("GOTO")
        

        # Add the subtrees to the root node in order of priority
        #BEHAVE.add_child(STAY_HEALTHY)
        BEHAVE.add_child(TIMER)
        BEHAVE.add_child(POSE)
        BEHAVE.add_child(GOTO)
        
        '''
        # Set the docking station pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.docking_station_pose

        NAV_DOCK_TASK = SimpleActionTask("NAV_DOC_TASK", "move_base", MoveBaseAction, goal, result_timeout=40, reset_after=True)
        
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
        '''
        # Add the Timed event check  to the "TIMER" task
        with TIMER:
            CLOCK5 = Clock1(60)
            EMOTION = Emotion1()
            TIMER.add_child(CLOCK5)
            TIMER.add_child(EMOTION)
            

        with GOTO:
            GOTO_LOCATION = Timer1()#= MonitorTask("GOTO_LOCATION", "goto", String, self.goto_location)
            #TIMER1 = Timer1()
            #TIMER5 = Timer5()
       
            GOTO.add_child(GOTO_LOCATION)
            #GOTO.add_child(TIMER5)

        with POSE:
            CLOCK1 = Rand_Result()
            #EMOTION = Emotion1()
     
            POSE.add_child(CLOCK1)
            #POSE.add_child(EMOTION)
            

      
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

    def goto_location(self, msg):
        if msg.data is None:
            return TaskStats.RUNNING
        else:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = (Pose(Point(1.5, 1.5, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))) #self.room_locations[msg.data]
            ##self.target = goal
            print 'GOING TO   '+str(msg.data)
            rm.E_Update(0.5,0.5)
            
            #print str(self.room_locations[msg.data])
            #MOVE_BASE_TASK = SimpleActionTask("MOVE_BASE_TASK", "move_base", MoveBaseAction, goal, reset_after=False)
            return TaskStatus.FAILURE
            
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_all_goals()
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)






         
            





if __name__ == '__main__':
    tree = PhoenixControl()
