#!/usr/bin/env python
# Author: Jonathan Schultz


import rospy
import nltk
import os
import re

from std_msgs.msg import String

class SimpleChartParser():
    def __init__(self):
        self.pub = rospy.Publisher('nltk_parse', String, queue_size=5)
        #rospy.init_node('nltk_parse_node', anonymous=True)
        #path = rospy.get_param("/speech_text/lm_path")

        rospy.Subscriber("/speech_parse", String, self.callback)
        rospy.init_node('simple_chart_parser_node', anonymous=True)
        self.path = os.path.join(rospy.get_param("/simple_chart_parser/grammar_path"), "grammars", rospy.get_param("/simple_chart_parser/grammar_name"))
        self.grammar = nltk.data.load('file:%s'%self.path)
        self.parser = nltk.ChartParser(self.grammar)
        rospy.spin()

    def callback(self,data):
        try:
            print "SimpleChartParser received: " + data.data
            
            clean = data.data.replace(".", "")
            
            tokens = nltk.word_tokenize(clean)
            trees = self.parser.parse(tokens)
            temp = ''
            print trees
            for tree in trees:
                print tree
                temp += tree.pprint(70,0,'','()', False)
            print "*****************************************"
            data.data = temp
            self.pub.publish(data.data)
        except rospy.ServiceException, e:
            print "Reply failed: %s"%e


if __name__ == '__main__':
    try:
        r = SimpleChartParser()
    except rospy.ROSInterruptException: pass
