#!/usr/bin/env python
'''
use as a move turn 
./test.py Joe 25
./move right 90
./move forward 1


'''
import sys
import rospy
from robbie_speech.interaction import *
from std_msgs.msg import String, Float64
from robbie_msgs.msg import RobbieCmd
import time

def talker():
    val1 = sys.argv[1]
    val2 = sys.argv[2]
    #room location
    room ={'bath': [1.5,1.5,90], 'kitchen': [1.5,-1.5,-90],'auto_dock':[0,0,0]}
    turn = {'left':[0,0,-90],'right':[0,0,90]}
    move = {'right':[0.5,0.5,90],'left':[0.5,-0.5,-90],'forward':[1,0,0],'backward':[-1,0,0]}
    
    
    mov_pub = rospy.Publisher('move_to', RobbieCmd, queue_size=5)
    look_at_pub = rospy.Publisher('look_at', String, queue_size=10)
    nav_pub = rospy.Publisher('nav_to', String, queue_size=10)

    rospy.init_node('director', anonymous=True)
    #turn and or move a direction from move dict
    if val1 == 'move':
        t = move[val2]
        print t
        a = RobbieCmd()
        a.command = "base_footprint"
        a.x = t[0]
        a.y = t[1]
        a.z = t[2]
        mov_pub.publish(a)        
    #turn to dircectin from turn dict   
    if sys.argv[1] == 'turn':
        t = turn[val2]
        print t
        a = RobbieCmd()
        a.command = "base_footprint"
        a.x = t[0]
        a.y = t[1]
        a.z = t[2]
        mov_pub.publish(a)
    # Navigate to a location from room dict
    if sys.argv[1] == 'goto':
        t = room[val2]
        #print t
        a = RobbieCmd()
        a.command = "map"
        a.x = t[0]
        a.y = t[1]
        a.z = t[2]
        mov_pub.publish(a)

    if sys.argv[1] == 'look':
        print "looking  " + str(sys.argv[2])
        mov_pub.publish(val1 + ":" + val2 )


       

    if sys.argv[1] == 'time':
        print NowTime()
    if sys.argv[1] == 'weather':
        print Weather()
    
    time.sleep(0.5)

if __name__ == '__main__':
    try:
        talker()
        #val1 = sys.argv[1]
        #val2 = sys.argv[2]
    except rospy.ROSInterruptException:
        pass

