#! /usr/bin/env python
'''
<star/> is not working here
commands need the short and the long name
sort and grade process commands for other functions(adds self.

'''

import rospy
from robbie_behave.interaction import *
from std_msgs.msg import String, Float64
from robbie_msgs.msg import RobbieCmd
import time
#import json_prolog


class Knowledge:
    """ 
    a class to python to return knowledge
    """
    def __init__(self):

        #command mapping
        self.commands ={'test':'self.Test','euro':'self.Euro','move':'self.Move','look':'self.Look','goto':'self.Goto','turn':'self.Turn','weather':'self.Weather'}

        #command interpertation
        self.room ={'bathroom': [1.5,1.5,90], 'kitchen': [5.027,-5.892,0],'auto_dock':[0,0,0], 'mats_room':[0.543,-4.742,0], 'tims_room':[2.159,-12.441,-90], 'entry':[3.622,-1.923,90], 'dining_room':[2.975,-7.595,37]}
        self.turn = {'left':[0,0,90],'right':[0,0,-90]}
        self.move = {'right':[0.5,-0.5,90],'left':[0.5,0.5,-90],'forward':[1,0,0],'backward':[-1,0,0]}
        self.look = {'right':[-1.1,0.0,0],'left':[1.1,0.0,0],'front':[0,0,0],'up':[0,-0.8,0],'down':[0,0.8,0]}

        #publishers
        self.mov_pub = rospy.Publisher('move_to', RobbieCmd, queue_size=5)
        self.look_at_pub = rospy.Publisher('look_at', RobbieCmd, queue_size=10)
        self.nav_pub = rospy.Publisher('nav_to', String, queue_size=10)


    def Sort(self, x):
        y = x.split(",")
        # make this a loop
        a1= y[0].strip(' ')# strip white spaces from name
        a2 = y[1].lstrip(' ')#strip white spaces from left side ofname
        t = self.commands[a1]
        #print eval(t)(a2)
        
        return eval(t)(a2)#eval will return the function name of a string

    def Grade(self, x, a):
        a3 = a.strip('{ }')
        y = x.split(",")
        # make this a loop
        a1= y[0].strip(' ')# strip white spaces from name
        a2 = y[1].lstrip(' ')#strip white spaces from left side ofname
        t = self.commands[a1]
        #print eval(t)(a2)
        
        return eval(t)(a2, a3)#eval will return the function name of a string and go to that function



    def Test(self,x,y):
        #a = x
        b =  "hi from test  " +str(x)+ ' extra  '+str(y)
        #print b
        return b


    def Euro(self,x,y):
        #a = x
        b =  "hi from euro  " +str(x)+ ' extra  '+str(y)
        #print y
        return y

    # interface to knowrob query to be tested
    def Know(self, x):
        y = x.split(",")
        # make this a loop
        a1= y[0].strip(' ')# strip white spaces from name
        a2 = y[1].lstrip(' ')#strip white spaces from left side ofname
        t = self.commands[a1]
        #prolog = json_prolog.Prolog()
        #query = prolog.query("member(A, [1, 2, 3, 4]), B = ['x', A]")
        #for solution in query.solutions():
            #print 'Found solution. A = %s, B = %s' % (solution['A'], solution['B'])
        #query.finish()

    def Move(self,x,y):
        c = str(x).rstrip(' ')
        t = self.move[str(c)]
        b = "I am moving "+ str(x)
        a = RobbieCmd()
        a.command = "base_footprint"
        a.x = t[0]
        a.y = t[1]
        a.z = t[2]
        self.mov_pub.publish(a)               
        return b

    def Turn(self,x,y):
        c = str(x).rstrip(' ')
        t = self.turn[str(c)]
        b = "I am Turning "+ str(x)
        a = RobbieCmd()
        a.command = "base_footprint"
        a.x = t[0]
        a.y = t[1]
        a.z = t[2]
        self.mov_pub.publish(a)
        return b

    def Look(self,x,y):
        c = str(x).rstrip(' ')
        t = self.look[c]
        b = "I am Looking "+ str(x)
        a = RobbieCmd()
        a.command = "base_footprint"
        a.x = t[0]
        a.y = t[1]
        a.z = t[2]
        self.look_at_pub.publish(a)   
        return b

    def Goto(self,x,y):
        c = str(x).rstrip(' ')
        t = self.goto[c]
        b = "I am going to the "+ str(x)
        a = RobbieCmd()
        a.command = "base_footprint"
        a.x = t[0]
        a.y = t[1]
        a.z = t[2]
        self.nav_pub.publish(a)
        return b

    def Weather(self,x,y):
        b = Weather()
        return b

    def Status(self,x,y):
        
        return y

    def Play(self,x,y):
        
        return y

    def Sing(self,x,y):
        
        return y

