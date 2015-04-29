#!/usr/bin/env python
# Author: Jonathan Schultz



import tokenize

import rospy
import re
from robbie_msgs.srv import Parse
from std_msgs.msg import String

# interprets an Nltk parse tree for MeetGreet commands
class NltkInterpreter():

    def __init__(self):
        rospy.init_node('ntlk_interpreter_node', anonymous=True)

        # NON FUNCTIONAL CODE FOR STUDYING
        #self.service = rospy.Service('nltk_interpret', WWParseInterpretation, self.handle_interpret_request)

        # USE THIS EVENTUALLY INSTEAD OF rospy.Publisher()
        #self.service = rospy.Service('nltk_interpret', MeetGreetParseInterpretation, self.handle_interpret_request)   
        rospy.Subscriber("/nltk_parse", String, self.handle_interpret_request)

        # ONLY FOR TESTING
        #self.pub = rospy.Publisher('nltk_interpret', MeetGreetParseInterpretation)
        self.pub = rospy.Publisher('nltk_interpret', String, queue_size=5)       

        rospy.wait_for_service('nltk_parse')
        try:
            self.parse_nltk = rospy.ServiceProxy('nltk_parse', Parse) # formerly Parse
        except rospy.ServiceException, e:
            print "ntlk_parse Failed: %s"%e
        rospy.spin()

    def handle_interpret_request(self, request):
        command = ""
	question=""
	response=""
        size = ""
        color = ""
        shape = ""
	topic=""
	person=""
	major=""
	food=""
        place=""
        pro=""
        uh=""

        try:
            print "NLTK Interpreter String: " + request.data
            #parse_tree = self.parse_nltk(request.data).parse_tree.replace("\n","")
            parse_tree = request.data.replace("\n","")
            print "parse_tree_interpert = ", parse_tree

            if re.search("COMMAND", parse_tree):
                command = re.search("\(V (\w+)\)", parse_tree)
                command = "" if command == None else command.group(1)
                topic = re.search("\(TOPIC (\w+)\)", parse_tree)
                topic = "" if topic == None else topic.group(1)
                size = re.search("\(SIZE (\w+)\)", parse_tree)
                size = "" if size == None else size.group(1)
                color = re.search("\(COLOR (\w+)\)", parse_tree)
                color = "" if color == None else color.group(1)
                shape = re.search("\(SHAPE (\w+)\)", parse_tree)
                shape = "" if shape == None else shape.group(1)
                place = re.search("\(PLACE (\w+)\)", parse_tree)
                place = "" if place == None else place.group(1)
                person = re.search("\(PERSON (\w+)\)", parse_tree)
                person = "" if person == None else person.group(1)
                uh = re.search("\(UH (\w+)\)", parse_tree)
                uh = "" if uh == None else uh.group(1)
                food = re.search("\(FOOD (\w+)\)", parse_tree)
                food = "" if food == None else food.group(1)  

	    elif re.search("QUESTION", parse_tree):
		question = re.search("\(WH (\w+)\)", parse_tree)
		question = "" if question == None else question.group(1)
                topic = re.search("\(TOPIC (\w+)\)", parse_tree)
                topic = "" if topic == None else topic.group(1)
                person = re.search("\(PERSON (\w+)\)", parse_tree)
                person = "" if person == None else person.group(1)
                major = re.search("\(MAJOR (\w+)\){1}", parse_tree)
                major = "" if major == None else major.group(1)
                temp = re.search("\(MAJOR (\w+)\){2}", parse_tree)
                if (temp != None and temp.group(1) != major):
                    major += " " + temp.group(1)
                food = re.search("\(FOOD (\w+)\)", parse_tree)
                food = "" if food == None else food.group(1)  
                size = re.search("\(SIZE (\w+)\)", parse_tree)
                size = "" if size == None else size.group(1)
                color = re.search("\(COLOR (\w+)\)", parse_tree)
                color = "" if color == None else color.group(1)
                shape = re.search("\(SHAPE (\w+)\)", parse_tree)
                shape = "" if shape == None else shape.group(1)  
                pro = re.search("\(PRO (\w+)\)", parse_tree)
                pro = "" if pro == None else pro.group(1)
    
	    elif re.search("RESPONSE", parse_tree):
                topic = re.search("\(TOPIC (\w+)\)", parse_tree)
                topic = "" if topic == None else topic.group(1)
                person = re.search("\(PERSON (\w+)\)", parse_tree)
                person = "" if person == None else person.group(1)
                major = re.search("\(MAJOR (\w+)\){1}", parse_tree)
                major = "" if major == None else major.group(1)
                temp = re.search("\(MAJOR (\w+)\){2}", parse_tree)
		if (temp != None and temp.group(1) != major):
                    major += " " + temp.group(1)
                food = re.search("\(FOOD (\w+)\){1}", parse_tree)
                food = "" if food == None else food.group(1)    
                temp = re.search("\(FOOD (\w+)\){2}", parse_tree)
		if (temp != None and temp.group(1) != food):
                    food += " " + temp.group(1)
    
            elif re.search("YESNO", parse_tree):
                command = re.search("\(YESNO (\w+)\)", parse_tree)
                command = "" if command == None else command.group(1)

            


            else:
                rospy.logdebug("nltk_interpret.py has issues.")

        except rospy.ServiceException, e:
            print "NLTK Interpreter failed: %s"%e

        text = command + ":" + question + ":" + size + ":" + color + ":" + shape + ":" + topic + ":" + person + ":" + major + ":" + food + ":" + place +":" + pro +":" + uh

        ar = text.split(':')

        class data:
            command = ""
	    question=""
            size = ""
            color = ""
            shape = ""
	    topic=""
	    person=""
	    major=""
	    food=""
            place=""
            pro=""
            uh=""

        data.command = ar[0]
        data.question = ar[1]
        data.size = ar[2]
        data.color = ar[3]
        data.shape = ar[4]
        data.topic = ar[5]
        data.person = ar[6]
        data.major = ar[7]
        data.food = ar[8]
        data.place = ar[9]
        data.pro = ar[10]
        data.uh = ar[11]

        print "\nCOMMAND   : " + data.command
        print "QUESTION  : " + data.question
        print "SIZE      : " + data.size
        print "COLOR     : " + data.color
        print "SHAPE     : " + data.shape
        print "TOPIC     : " + data.topic
        print "PERSON    : " + data.person
        print "MAJOR     : " + data.major
        print "FOOD      : " + data.food
        print "place     : " + data.place
        print "pro       : " + data.pro
        print "uh        : " + data.uh

	print text
        self.pub.publish(text) # THIS IS KLUDGY (would be better being passed as a message in a service)
#        return [command, question, response, size, color, shape, topic, person, major, food, age]

if __name__ == '__main__':
    try:
        n = NltkInterpreter()
        rospy.spin()
    except rospy.ROSInterruptException: pass

