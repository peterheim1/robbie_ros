#!/usr/bin/python
'''
dump from csv not working
cant add name
sentance parsing need work wont parse name


'''


import sys
import getopt
import re
import subprocess
import xml.etree.ElementTree as ET
import csv
import random
import os
#import requests

from pattern.en import parse
from pattern.en import pprint
from pattern.en import parsetree
from pattern.en import wordnet
from pattern.en import pluralize, singularize
from pattern.en import conjugate, lemma, lexeme

from robbie_nlp.nlp_sa  import sentenceAnalysisClass
from robbie_nlp.nlp_cap import capabilitiesClass
from robbie_nlp.nlp_mem import memoryClass


class NlpClass:

    def __init__(self, useFifos, appID):
      #self.cap       = capabilitiesClass()   # capabilities
      self.my_memory  = memoryClass()         # memory of concepts
      # Add a concept 'I', defining CCSR identity
      self.my_memory.add('I')
      self.my_memory.concepts['I'].state = 'great'      # dynamically reflect mood by telemetry
      self.my_memory.concepts['I'].person = '1sg'       # 2st person singular
      self.my_memory.concepts['I'].isProperNoun = True 
      self.my_memory.concepts['I'].properties = {"name": "robbie"}


       # Synonymes for certain response forms, to give some natural response
      # variations
      self.responseVariations =  {"yes": ("Yes.",
                                          "Affirmative.",
                                          "Definitely.",
                                          "Sure.",
                                          "Absolutely."),
                                  "acknowledge": ("I see.",
                                                  "OK.",
                                                  "Acknowledged.",
                                                  "If you say so.",
                                                  "Copy that.",
                                                  "I'll remember that."),
                                  "gratitude":   ("Thanks!",
                                                  "I appreciate that.",
                                                  "You are too kind.",
                                                  "Oh stop it."),
                                  "insulted":    ("I'm sorry you feel that way.",
                                                  "Well, you're not too hot either.",
                                                  "Look who's talking.",
                                                  "Can't we just be nice."),
                                  "gratitudeReply": ("You're very welcome!",
                                                     "Sure thing!",
                                                     "No worries!",
                                                     "Don't mention it!"),
                                  "bye": ("See you later.",
                                          "It was a pleasure.",
                                          "Bye bye."),
                                  "hi": ("Hi, how are you.",
                                          "Hey there.",
                                          "What's going on!"),
                                  "no": ("No.",
                                         "Negative.",
                                         "I don't think so.",
                                         "Definitely not.",
                                         "No way.",
                                         "Not really.")
                                  }

    def getPersonalProperty(self, sa):
        
      # Question refers back to ccsr: how is 'your' X 
      print sa 
      if sa.getSentenceRole(sa.concept) in self.my_memory.concepts['I'].properties:
         #fails because requested field is set to none 
         self.response("say my " + self.my_memory.concepts['I'].properties[sa.getSentenceRole(sa.concept)][0] + " is " + self.my_memory.concepts['I'].properties[sa.getSentenceRole(sa.concept)][1])
      else:
         self.response("say I don't know what my " + sa.getSentenceRole(sa.concept) + " is ")

    def response(self, s):
        print'my responce is     :' + s

    def randomizedResponseVariation(self, response):
       idx = random.randint(0, len(self.responseVariations[response])-1)
       return self.responseVariations[response][idx]

    def nlpParse(self, line, debug=0):
        text = parsetree(line, relations=True, lemmata=True)
        for sentence in text:
           sa = sentenceAnalysisClass(sentence, debug)
           st = sa.sentenceType()
        if sa.debug:
            print st
            print 'concept is: ' + sa.concept
        if st == 'questionState':
            if sa.is2ndPersonalPronounPosessive('OBJ'):
               # Question refers back to ccsr: how is 'your' X. Look up robbie's personal property
               self.getPersonalProperty(sa)
        elif st == 'confirmState':
            print 'comfirm state'
        elif st == 'questionDefinition':
            if sa.is2ndPersonalPronounPosessive('OBJ'): 
               # Question refers back to ccsr: what is 'your' X. Look up robbie's personal property
               self.getPersonalProperty(sa)
        #statement will not remember name 
        elif st == 'statement':
            if sa.is2ndPersonalPronounPosessive('SBJ'): 
               # Refers back to ccsr: 'your' X is Y 
               if sa.getSentenceRole(sa.concept) not in self.my_memory.concepts['I'].properties:
                  self.ccsrmem.concepts['I'].properties[sa.getSentenceRole(sa.concept)] = [sa.getSentenceRole(sa.concept), sa.getSentencePhrase('OBJ')]
                  print sa.getSentenceRole(sa.concept), sa.getSentencePhrase('OBJ')
               self.response("say name " + self.randomizedResponseVariation('acknowledge')) 
            else:
               if sa.getSentenceRole(sa.concept) == 'I':
                  # Statement about CCSR, do not memorize this (CCSR maintains its own state based on CCSR telemetry
                  # but instead react to statement
                  print 'ww ' + sa.getSentenceRole('ADJP')
                  if sa.getSentenceRole('ADJP') in self.positivePhrases:
                     # Saying something nice will maximize happiness and arousal
                     self.response("set mood 500 500 ") 
                     self.response("say " + self.randomizedResponseVariation('gratitude')) 
                  else:
                     # Saying something insulting will minimize happiness and increase arousal
                     self.response("set mood -300 50 ") 
                     self.response("say " + self.randomizedResponseVariation('insulted')) 
               else:
                  if not self.my_memory.known(sa.getSentenceRole(sa.concept)):
                     self.my_memory.add(sa.getSentenceRole(sa.concept))  
                  self.my_memory.concepts[sa.getSentenceRole(sa.concept)].state = sa.getSentencePhrase('ADJP')
                  self.response("say write name " + self.randomizedResponseVariation('acknowledge')) 
         # State locality: 'X is in Y'
        elif st == 'stateLocality':
            print 'state location'
        elif st == 'command':
            print 'command'
        elif st == 'greeting':
            self.response("say " + self.randomizedResponseVariation('hi'))
        elif st == 'bye':
            print 'bye'
        elif st == 'adverbPhrase':
            print 'adverb phrase'
        else:
            print"say sorry, I don't understand"
       


