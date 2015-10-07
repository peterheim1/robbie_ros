#!/usr/bin/env python

'''
'''

from pi_trees_lib.pi_trees_lib import *
from task import *
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
        rm= Robbie_memory()
       
        # The root node
        BEHAVE = Sequence("behave")
        # Create the "timer" selector
        TIMER = Sequence("TIMER")
        # Add the subtrees to the root node in order of priority
        BEHAVE.add_child(TIMER)
        
        # Add the Timed event check  to the "TIMER" task
        with TIMER:
            TIMER1 = Timer1()
            TIMER5 = Timer5()
            
     
            TIMER.add_child(TIMER1)
            TIMER.add_child(TIMER5)
            

      
        print "Behavior Tree Structure"
        print_tree(BEHAVE)
            
        # Run the tree
        while True:
            status = BEHAVE.run()
            if status == TaskStatus.SUCCESS:
                print "I feel.   " + str(rm.Emotion_State(black_board.emotion))
                break






class Timer1(Task):
    def __init__(self, timer=10, *args):
        name = "Timer 1"
        super(Timer1, self).__init__(name)
        self.name = name
        self.counter = timer
        self.finished = False

    def run(self):
        if self.finished:
            
            emot = 1,1.2
            emotion = black_board.emotion
            emotion = map(sum,zip(emotion,emot))
            print 'timer 1  '+str(black_board.emotion)
            black_board.emotion = emotion
            return TaskStatus.SUCCESS
        else:
            print 'timer 1 running'
            
            while self.counter > 0:
                self.counter -= 1
                print "timer 1   "+self.counter
                time.sleep( 1)
                return TaskStatus.RUNNING
            
            self.finished = True
            print "finished timer 1"
            

class Timer5(Task):
    def __init__(self, timer=10, *args):
        name = "Timer 5"
        super(Timer5, self).__init__(name)
        self.name = name
        self.counter = timer
        self.finished = False
       
    def run(self):
        if self.finished:
            #print 'add emotion '
            emot1 = 1,1.2
            emotion1 = black_board.emotion
            emotion1 = map(sum,zip(emotion1,emot1))
            print 'timer 5  '+str(black_board.emotion)
            black_board.emotion = emotion1
            return TaskStatus.SUCCESS
        else:
            print 'timer 5 running'
            
            while self.counter > 0:
                self.counter -= 1
                print "timer 5   "+ self.counter
                #rospy.sleep(1)
                time.sleep( 1)
                #add emotion
                return TaskStatus.RUNNING
            
            self.finished = True
            print "finished timer 5"
            





if __name__ == '__main__':
    tree = TimingExample()
