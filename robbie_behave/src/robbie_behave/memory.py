#!/usr/bin/env python
'''
menory system for robbie 
we use a pickel file to store memories as a dictionary
to share with other files


'''

import pickle


class Robbie_memory():
    def __init__(self):
        #rospy.init_node("robbie_memory")
        self.MeMemory = {}


    def Memory_read(self):
        '''
         open file and read data as dictionary

        '''
        pkl_file = open('/home/peter/catkin_hydro/src/robbie_ros/robbie_behave/nodes/robbie.pkl', 'rb') #change location
        self.MeMemory = pickle.load(pkl_file)
        pkl_file.close()

    def Memory_write(self):
        output = open('/home/peter/catkin_hydro/src/robbie_ros/robbie_behave/nodes/robbie.pkl', 'wb')
        pickle.dump(self.MeMemory, output)
        output.close()
       


