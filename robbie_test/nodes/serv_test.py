#!/usr/bin/env python

from rbx2_msgs.srv import *
import rospy
import time

def handle_add_two_ints(req):
    #aa =str(req)
    #ar = aa.split()
    print req.value#ar[1]
    time.sleep(10)
    w = 0
    return DockerResponse()

def add_two_ints_server():
    rospy.init_node('nav_server')
    s = rospy.Service('auto_dock', Docker, handle_add_two_ints)
    print "Ready to recieve request."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
