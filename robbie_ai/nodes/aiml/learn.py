#! /usr/bin/env python

from string import join, upper
import sys
import os
dir = os.path.dirname(os.path.abspath(__file__))


class Learning:
    """ 
    a class to fins and pick up a object 
    """
    def __init__(self, text):
        #print text

        xml = ['\n']
	#<category>
        #<pattern>TURN TO THE LEFT</pattern>
	#xml.append('<topic name=" %s">' % data[0].upper())

	xml.append('<category>')
	xml.append('<pattern>%s</pattern>' % text.upper())
	xml.append('<template></template>')
        xml.append('<template><srai></srai></template>')
	#xml.append(data[0])
	#xml.append('</template>')
	xml.append('</category>')
	
	
	
	xml.append('\n')
	
	str = join(xml,'\n')
	print str
	f = open(dir + '/../data/learnt.aiml', "a")
	f.write(str+ "\n")
	f.close()

if __name__ == "__main__":
    main()
