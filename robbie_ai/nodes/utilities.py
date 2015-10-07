#!/usr/bin/env python
'''
menory system for robbie 
we use a pickel file to store memories as a dictionary
to share with other files
to use 

from utilities import *
rm= Robbie_memory()
rm.E_Update('1,1')
rm.MeMemory_read()

rm.Emotion_State(2,5)

personal information robbie will ask and store should we use a datadase?
{'id':0,'name':0,'info':{{'id':0,'name':0,'sex':0,'birthday':0,'mother':0,'father':0,'child1':0,'child2':0,'child3':0}}

{'id':0,'name':0,'sex':0,'birthday':0,'mother':0,'father':0,'child1':0,'child2':0,'child3':0}




'''

import pickle
import os
from scipy.spatial import distance
dir = os.path.dirname(os.path.abspath(__file__))



class Robbie_memory():
    def __init__(self):
        #rospy.init_node("robbie_memory")
        self.MeMemory = {}
        #self.MeEmoution ={}


    def MeMemory_read(self):
        '''
         open file and read data as dictionary

        '''
        pkl_file = open(dir + '/../data/robbie.pkl', 'rb') #change location
        self.MeMemory = pickle.load(pkl_file)
        pkl_file.close()
        return self.MeMemory

    def MeMemory_write(self, memory):
        output = open(dir + '/../data/robbie.pkl', 'wb')
        pickle.dump(memory, output)
        output.close()

    def MeEmotion_read(self):
        '''
         open file and read data as dictionary

        '''
        pkl_file = open(dir + '/../data/emotion.pkl', 'rb') #change location
        self.MeEmotion = pickle.load(pkl_file)
        pkl_file.close()
        return self.MeEmotion

    def MeEmotion_write(self, x,y):
        w = x,y
        output = open(dir + '/../data/emotion.pkl', 'wb')
        pickle.dump(w, output)
        output.close()

    def Emotion_State(self):
        '''
         open file and read data as dictionary

        '''
        w = self.MeEmotion_read()
        s = w[0],w[1]
        #print s
        r = 5 
        neutral =0,0
        vigilant =0,r
        happy	=r,0
        sad	=-r,0
        bored	=0,-r
        excited = r*0.7071,r*0.7071
        angry = -r*0.7071,r*0.7071
        relaxed = r*0.7071,-r*0.7071
        depressed = -r*0.7071,-r*0.7071
        b= ["neutral", "vigilant", "happy", "sad", "bored", "excited", "angry", "relaxed", "depressed"]
        a = [distance.euclidean(s,neutral),distance.euclidean(s,vigilant),distance.euclidean(s,happy),distance.euclidean(s,sad),distance.euclidean(s,bored),distance.euclidean(s,excited),distance.euclidean(s,angry),distance.euclidean(s,relaxed),distance.euclidean(s,depressed)]

        return b[a.index(min(a))]

    def E_Update(self, x,y):
        '''
         update robbies current emotional state

        '''       
        c = self.MeEmotion_read()#read the current emotion value           
        XE = x + c[0]
        YE = x + c[1]         
        self.MeEmotion_write(XE,YE)#write to disk

       


