#!/usr/bin/env python

"""
    
"""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float32
import Tkinter as tk

class Demo1:
    def __init__(self, master):
        self.master = master
        # Subscribe to the recognizer output and set the callback function
        self.map = 0
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.talkback)
        rospy.Subscriber('battery_level', Float32, self.check_battery)
        self.w = tk.Label(master, text=self.map)
        self.frame = tk.Frame(self.master)
        self.button1 = tk.Button(self.frame, text = 'Quit', width = 25, command = self.close_windows)
        self.button1.pack()
        self.w.pack()
        self.frame.pack()
    def close_windows(self):
        self.master.destroy()


    def check_battery(self, msg):
        rospy.logwarn("Battery low - level: " + str(int(msg.data)))
        self.map = str(int(msg.data))
    def talkback(self, msg):
        # republis input to task coord or chat engine
        #self.map= msg.pose.pose.position.x
        rospy.logwarn(msg.data)


def main(): 
    root = tk.Tk()
    app = Demo1(root)
    root.mainloop()

if __name__ == '__main__':
    main()

