#!/usr/bin/env python

'''
'''

from pi_trees_lib.pi_trees_lib import *
#from task import *
from utilities import *
import time

# A class to track global variables
class BlackBoard():
    def __init__(self):
        
        # A list to store rooms and tasks
        self.task_list = list()
        self.emotion = 0.1,0.5
        
        

# Initialize the black board
black_board = BlackBoard()



class TimingExample():
    def __init__(self):
        rm= Robbie_memory()#from utilities
       
        # The root node
        BEHAVE = Sequence("behave")
        # Create the "timer" selector
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
            CLOCK1 = CLOCK1()
            CLOCK5 = CLOCK5()
            
     
            CLOCK.add_child(CLOCK1)
            CLOCK.add_child(CLOCK5)
            

      
        print "Behavior Tree Structure"
        print_tree(BEHAVE)
            
        # Run the tree
        while True:
            status = BEHAVE.run()
            if status == TaskStatus.SUCCESS:
                print "I feel.   " + str(rm.Emotion_State(black_board.emotion))
                break






class Timer1(Task):
    def __init__(self, timer=5, *args):
        name = "Timer 1"
        super(Timer1, self).__init__(name)
        self.name = name
        self.counter = timer
        self.finished = False

    def run(self):
        if self.finished:
            #this loop is always run
            return TaskStatus.SUCCESS
        else:
            #this loop is running until complete
            
            while self.counter > 0:
                self.counter -= 1
                #print "timer 1   "+str(self.counter)
                time.sleep( 1)#1 second sleep
                return TaskStatus.RUNNING
            
            self.finished = True
            #this part is only run once
            emot = 1,1.2
            emotion = black_board.emotion
            emotion = map(sum,zip(emotion,emot))
            black_board.emotion = emotion
            print 'timer 1 after finish  '+str(black_board.emotion)
            
            
            

class Timer5(Task):
    def __init__(self, timer=5, *args):
        name = "Timer 5"
        super(Timer5, self).__init__(name)
        self.name = name
        self.counter = timer
        self.finished = False
       
    def run(self):
        if self.finished:
            #this loop is always run
            return TaskStatus.SUCCESS
        else:
            
            while self.counter > 0:
                self.counter -= 1
                
                #rospy.sleep(1)
                time.sleep( 1)
                
                return TaskStatus.RUNNING
            
            self.finished = True
            emot1 = 1,1.2
            emotion1 = black_board.emotion
            emotion1 = map(sum,zip(emotion1,emot1))
            black_board.emotion = emotion1
            print 'timer 5  after finish    '+str(black_board.emotion)
            
            
            





if __name__ == '__main__':
    tree = TimingExample()
