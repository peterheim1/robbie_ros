#!/usr/bin/env python
'''
test out nested branches

'''
from pi_trees_lib.pi_trees_lib import *

class Branch_test():
    def __init__(self):

        # The root node when all top levels tasks have SUCCESS: program exits
        BEHAVE = Sequence("behave")

        # Create the top level tasks in order.  Sequence executes each of its child behaviors until one of them fails
        # Create the "stay healthy" selector
        #STAY_HEALTHY = Selector("STAY_HEALTHY")
        INQUIRY = Sequence("INQUIRY")
        TIMER = Selector("TIMER")
        POSE = Sequence("POSE")
        GOTO = Selector("GOTO")
        

        # Add the subtrees to the root node in order of priority
        #BEHAVE.add_child(STAY_HEALTHY)
        BEHAVE.add_child(INQUIRY)
        BEHAVE.add_child(TIMER)
        BEHAVE.add_child(POSE)
        BEHAVE.add_child(GOTO)
        with TIMER:
            CLOCK5 = self.Clock1()#will check if greater than 60 secsince last activity
            EMOT = self.Emotion1()
            EMOTION = Sequence ("EMOTION",[EMOT])

            TIMER.add_child(CLOCK5)
            TIMER.add_child(EMOTION)
            


        print "Behavior Tree Structure"
        print_tree(BEHAVE)

        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.run()
            rospy.sleep(0.1)

    def Clock1(self):
        print "clock1"

    def Emotion1(self):
        print "Emotion11"


if __name__ == '__main__':
    tree = Branch_test()
