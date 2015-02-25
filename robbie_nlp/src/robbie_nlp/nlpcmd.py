#!/usr/bin/python


# Debug script for nlpx: we read lines from stdin and pass them to nlpx
# This script is not used by CCSR; instead nlpx will receive text from
# voice recognition engine (google)

import sys
import getopt
import re
from pattern.en import parse
from pattern.en import pprint
from pattern.en import parsetree
from pattern.en import wordnet
from pattern.en import pluralize, singularize
from pattern.en import conjugate, lemma, lexeme

from nlpx import ccsrNlpClass

loop = True         # If true, we continuously read and parse
brain = 'nlpxCCSR'  # By default, use nlpxCCSR python module as NLP brain. We can
                    # set this to 'ANNA' to use the remote brain API at
                    # http://droids.homeip.net/RoboticsWeb/
debug = True

try:
   opts, args = getopt.getopt(sys.argv[1:],"hnad",["help","noloop", "anna", "debug"])
except getopt.GetoptError:
   print 'nlp.py -h -l -a'
   sys.exit(2)
for opt, arg in opts:
   if opt == '-h':
      print 'nlp.py'
      sys.exit()
   elif opt in ("-n"):
      loop = False
   elif opt in ("-a"):
      brain = 'ANNA'
   elif opt in ("-d"):
      debug = True

appID = 'T3H9JX-RQQ2273TJ9'        # Fill in your own Wolfram AppID here
useFifos = False     # Only set True if integrated with CCSR robot platform
s = ccsrNlpClass(useFifos, appID)

print 'nplxCCSR v0.1: type a question...'
while (1):
   line = sys.stdin.readline()
   print brain
   if brain == 'nlpxCCSR':
      s.nlpParse(line, debug)
   elif brain == 'ANNA':
      s.annaApi(line)
   if not loop:
      break
