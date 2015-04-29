#!/usr/bin/env python
from Tkinter import *
import rospy
from std_msgs.msg import Float64

def show_values():

    pan = rospy.Publisher('/pan', Float64, queue_size=1)
    tilt = rospy.Publisher('/tilt', Float64, queue_size=1)
    x = float(w1.get()*0.01)
    y= float(w2.get()*0.01)
    pan.publish(x)
    tilt.publish(y)
    print (x, y)

rospy.init_node("slider_test")
master = Tk()
w1 = Scale(master, from_=-150, to=150)
w1.set(0)
w1.pack()
print w1.get()
w2 = Scale(master, from_=-150, to=150, orient=HORIZONTAL)
w2.set(0)
w2.pack()

w3 = Scale(master, from_=-150, to=150, orient=HORIZONTAL)
w3.set(0)
w3.pack()
Button(master, text='Show', command=show_values).pack()

mainloop()
