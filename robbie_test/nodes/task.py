#!/usr/bin/env python
'''
menory system for robbie 
we use a pickel file to store memories as a dictionary
to share with other files
to use 

from task import *
rm= Robbie_memory()
rm.MeMemory_read()

rm.Emotion_State(2,5)




'''

import pickle
from scipy.spatial import distance


class Robbie_memory():
    def __init__(self):
        #rospy.init_node("robbie_memory")
        self.MeMemory = {}
        #self.MeEmoution ={}


    def MeMemory_read(self):
        '''
         open file and read data as dictionary

        '''
        pkl_file = open('/home/peter/catkin_hydro/src/robbie_ros/robbie_behave/nodes/robbie.pkl', 'rb') #change location
        self.MeMemory = pickle.load(pkl_file)
        pkl_file.close()
        return self.MeMemory

    def MeMemory_write(self, memory):
        output = open('/home/peter/catkin_hydro/src/robbie_ros/robbie_behave/nodes/robbie.pkl', 'wb')
        pickle.dump(memory, output)
        output.close()

    def MeEmotion_read(self):
        '''
         open file and read data as dictionary

        '''
        pkl_file = open('/home/peter/catkin_hydro/src/robbie_ros/robbie_behave/nodes/emotion.pkl', 'rb') #change location
        self.MeEmotion = pickle.load(pkl_file)
        pkl_file.close()
        return self.MeEmotion

    def MeEmotion_write(self, Emotion):
        output = open('/home/peter/catkin_hydro/src/robbie_ros/robbie_behave/nodes/emotion.pkl', 'wb')
        pickle.dump(Emotion, output)
        output.close()

    def Emotion_State(self, x):
        '''
         open file and read data as dictionary

        '''
        s = x[0],x[1]
        r = 5 
        neutral =0,0
        vigilant =0,r
        happy	=r,0
        sad	=-r,0
        bored	=0,-r
        exited = r*0.7071,r*0.7071
        angry = -r*0.7071,r*0.7071
        relaxed = r*0.7071,-r*0.7071
        depressed = -r*0.7071,-r*0.7071
        b= ["neutral", "vigilant", "happy", "sad", "bored", "exited", "angry", "relaxed", "depressed"]
        a = [distance.euclidean(s,neutral),distance.euclidean(s,vigilant),distance.euclidean(s,happy),distance.euclidean(s,sad),distance.euclidean(s,bored),distance.euclidean(s,exited),distance.euclidean(s,angry),distance.euclidean(s,relaxed),distance.euclidean(s,depressed)]

        return b[a.index(min(a))]

       


