#!/usr/bin/env python
'''
use as a move turn 
./test.py Joe 25
./move right 90
./move forward 1


'''
import sys
import rospy
from robbie_behave.interaction import *
from std_msgs.msg import String, Float64
from robbie_msgs.msg import RobbieCmd
import time

def talker():
    val1 = sys.argv[1]
    val2 = sys.argv[2]
    #room location
    room ={'bathroom': [1.5,1.5,90], 'kitchen': [5.027,-5.892,0],'auto_dock':[0,0,0], 'mats_room':[0.543,-4.742,0], 'tims_room':[2.159,-12.441,-90], 'entry':[3.622,-1.923,90], 'dining_room':[2.975,-7.595,37]}
    turn = {'left':[0,0,90],'right':[0,0,-90]}
    move = {'right':[0.5,-0.5,90],'left':[0.5,0.5,-90],'forward':[1,0,0],'backward':[-1,0,0]}
    look = {'right':[-1.1,0.0,0],'left':[1.1,0.0,0],'front':[0,0,0],'up':[0,-0.8,0],'down':[0,0.8,0]}
    
    
    mov_pub = rospy.Publisher('move_to', RobbieCmd, queue_size=5)
    look_at_pub = rospy.Publisher('look_at', RobbieCmd, queue_size=10)
    nav_pub = rospy.Publisher('nav_to', String, queue_size=10)

    rospy.init_node('director', anonymous=True)
    #turn and or move a direction from move dict
    if val1 == 'move':
        t = move[val2]
        print "I am "+str(val1)+"ing"+ str(val2)
        a = RobbieCmd()
        a.command = "base_footprint"
        a.x = t[0]
        a.y = t[1]
        a.z = t[2]
        mov_pub.publish(a)        
    #turn to dircectin from turn dict   
    if sys.argv[1] == 'turn':
        t = turn[val2]
        print "I am "+str(val1)+"ing"+ str(val2)
        a = RobbieCmd()
        a.command = "base_footprint"
        a.x = t[0]
        a.y = t[1]
        a.z = t[2]
        mov_pub.publish(a)
    # Navigate to a location from room dict
    if sys.argv[1] == 'goto':
        t = room[val2]
        print " I am going to the    " + str(val2)
        #print t
        a = RobbieCmd()
        a.command = "map"
        a.x = t[0]
        a.y = t[1]
        a.z = t[2]
        mov_pub.publish(a)

    if sys.argv[1] == 'look':
        #print "looking  " + str(sys.argv[2])
        t = look[val2]
        print " I am looking to the    " + str(val2)
        #print t
        a = RobbieCmd()
        a.command = "map"
        a.x = t[0]
        a.y = t[1]
        a.z = t[2]
        look_at_pub.publish(a)



       

    if sys.argv[1] == 'time':
        print NowTime()
    if sys.argv[1] == 'weather':
        print Weather()
    
    time.sleep(1)

if __name__ == '__main__':
    try:
        talker()
        #val1 = sys.argv[1]
        #val2 = sys.argv[2]
    except rospy.ROSInterruptException:
        pass

