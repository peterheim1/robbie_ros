#!/usr/bin/env python
from Tkinter import *
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker

def show_values():
    x = float(w1.get()*0.01)
    y = float(w2.get()*0.01)
    z = float(w3.get()*0.01)

    target_pub = rospy.Publisher('/target_point', PointStamped, queue_size=1)
    # Define a marker publisher
    marker_pub = rospy.Publisher('target_marker', Marker)
    target = PointStamped()
    target.header.frame_id = 'base_footprint'
    target.point.x = x
    target.point.y = y
    target.point.z = z  
    target.header.stamp = rospy.Time.now()

    target_pub.publish(target)
    print (x, y, z)

   
    # Initialize the marker
    marker = Marker()
    marker.ns = 'target_point'
    marker.header.frame_id ='/base_footprint'
    marker.id = 0
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration(0)
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.r = 0.0#marker_color[0.0]
    marker.color.g = 1.1#marker_color[1.0]
    marker.color.b = 0.0#marker_color[0.0]
    marker.color.a = 1.0#marker_color[1.0]
    marker.header.stamp = target.header.stamp
    #marker.header.frame_id = target.header.frame_id
    marker.pose.position = target.point
    marker_pub.publish(marker)
    rospy.logwarn(target.point)

rospy.init_node("slider_test")
master = Tk()
w1 = Scale(master, from_=0, to=150)
w1.set(0)
w1.pack()
print w1.get()
w2 = Scale(master, from_=-150, to=150, orient=HORIZONTAL)
w2.set(0)
w2.pack()

w3 = Scale(master, from_=0, to=150, orient=HORIZONTAL)
w3.set(0)
w3.pack()
Button(master, text='Show', command=show_values).pack()

mainloop()
