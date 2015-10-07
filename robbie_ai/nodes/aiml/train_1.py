#! /usr/bin/env python

from string import join, upper

import sys

def main():
	"""
	Really simple/hacky way of generating an aiml file
	that can be learnt and used to train the bot.
	"""
	data = sys.argv[1:]
	#meaning = data[1].upper()
	
	xml = ['\n']
	#<category>
        #<pattern>TURN TO THE LEFT</pattern>
	#xml.append('<topic name=" %s">' % data[0].upper())

	xml.append('<category>')
	xml.append('<pattern>%s</pattern>' % data[0].upper())
	xml.append('<template>%s</template>'% data[1])
	#xml.append(data[0])
	#xml.append('</template>')
	xml.append('</category>')
	
	
	
	xml.append('\n')
	
	str = join(xml,'\n')
	
	f = open("test.aiml", "a")
	f.write(str+ "\n")
	f.close()

if __name__ == "__main__":
    main()
