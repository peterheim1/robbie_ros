#!/usr/bin/env python

'''
'''
import rospy
from pi_trees_lib.pi_trees_lib import *
from task import *
from utilities import *
import time


        

# Initialize the black board
black_board = BlackBoard()



class TimingExample():
    def __init__(self):
        rospy.init_node("tree_timings")
        rm= Robbie_memory()#from utilities
        #Clock1 = Clock1()
       
        # The root node when all top levels tasks have SUCCESS: program exits
        BEHAVE = Sequence("behave")

        # Create the top level tasks in order.  Sequence executes each of its child behaviors until one of them fails
        TIMER = Sequence("TIMER")
        CLOCK = Sequence("CLOCK")

        # Add the subtrees to the root node in order of priority
        BEHAVE.add_child(TIMER)
        BEHAVE.add_child(CLOCK)
        # Add the Timed event check  to the "TIMER" task
        with TIMER:
            TIMER1 = Timer1()
            TIMER5 = Timer5()
       
            TIMER.add_child(TIMER1)
            TIMER.add_child(TIMER5)

        with CLOCK:
            CLOCK5 = Clock1()
            CLOCK1 = Rand_Result()
            
     
            CLOCK.add_child(CLOCK1)
            CLOCK.add_child(CLOCK5)
            

      
        print "Behavior Tree Structure"
        print_tree(BEHAVE)
            
        # Run the tree this runs the sequence if the last is SUCCESS: then the node exits
        while True:
            status = BEHAVE.run()
            if status == TaskStatus.SUCCESS:
                print "I feel.   " + str(rm.Emotion_State(black_board.emotion))
                break






         
            





if __name__ == '__main__':
    tree = TimingExample()
