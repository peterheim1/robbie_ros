#!/usr/bin/env python
'''


'''
import rospy
import time
import random
from pi_trees_lib.pi_trees_lib import *
from utilities import *
import markovian

# A class to track global variables
class BlackBoard():
    def __init__(self):
        
        # A list to store rooms and tasks
        self.task_list = list()
        self.emotion = 0.1,0.5
        self.Last_Event = time.time()

# Initialize the black board
black_board = BlackBoard()
rm= Robbie_memory()#from utilities
file_ = open('/home/tim/catkin_ws/src/robbie_ros/robbie_behave/nodes/suntzu.txt')
markov = markovian.Markov(file_)
#markov.generate_markov_text()

class Emotion1(Task):
    def __init__(self, timer=1, *args):
        name = "Emotion1"
        super(Emotion1, self).__init__(name)
        self.name = name
         

    def run(self):
        self.emotion_state = rm.Emotion_State()
        if self.emotion_state == 'bored':
            print "I'm feeling  bored "
            print 'i have to do somthing '
            rm.E_Update(0.5,0.5)#us random selected task
        if self.emotion_state == 'neutral':
            print "I'm feeling netural   "
            
        if self.emotion_state == 'vigilant':
            print "I'm vigilant"
        if self.emotion_state == 'happy':
            print "I'm happy"
            print markov.generate_markov_text()
            rm.E_Update(0.1,0.1)
        if self.emotion_state == 'sad':
            print "I'm very sad"
            print "i want to smash something"
            rm.E_Update(0.7,0.6)
        if self.emotion_state == 'excited':
            print "I'm excited"
        if self.emotion_state == 'vigilant':
            print "I'm vigilant"
        if self.emotion_state == 'angry':
            print "I'm angry"   
        if self.emotion_state == 'relaxed':
            print "I'm relaxed"
        if self.emotion_state == 'depressed':
            print "I'm depressed"
            print rm.MeEmotion_read()
            rm.E_Update(0.9,0.85)#us random selected task
            
            return TaskStatus.FAILURE
        else:
            return TaskStatus.SUCCESS
        
#this will block until complete
class Clock1(Task):
    def __init__(self, timer=60, *args):
        name = "Clock 1"
        super(Clock1, self).__init__(name)
        self.name = name
        self.counter = timer
        self.finished = False

    def run(self):
        if black_board.Last_Event + self.counter < time.time():
            print "clock 1 finished"+ str(rm.MeEmotion_read())
            rm.E_Update(-0.5,-0.5)
            black_board.Last_Event = time.time()
            return TaskStatus.FAILURE
        else:
            return TaskStatus.SUCCESS


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
            rm.E_Update(1,1.2)
            print 'timer 1 finished  '
            
            
            

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
            rm.E_Update( 1,1.2)
            
            print 'timer 5  fimished   '

class Rand_Result(Task):
    def __init__(self, timer=5, *args):
        name = "Rand_Result"
        super(Rand_Result, self).__init__(name)
        self.name = name
        self.counter = timer
        self.finished = False
       
    def run(self):
        if self.finished:
            #this loop is always run
            return TaskStatus.SUCCESS
        else:
            
            if random.random() < 0.5:
                print "task failed"
                rm.E_Update(-0.5,-0.5)
                
                print 'Rand_result FAILURE   '              
                #rospy.sleep(1)
                time.sleep( 1)
                return TaskStatus.FAILURE
            else:  
                rm.E_Update(1,1.2)
                print "task PASSED"
            self.finished = True
            rm.E_Update(0.8,1)
            
            print 'Rand_result finished    '
            
   
            
            
            


