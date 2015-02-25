#!/usr/bin/python


# NLPX provides the ccsrNlpClass, the Natural Language Processing engine for CCSR robot
# The main interface is ccsrNlpClass.nlpParse('sentence'), which will interpret the sentence
# and reply by giving a command to the CCSR process through the nlp telemetry fifo interface.
# If the reply is pure verbal answer to voice input, the CCSR command 'say' will be used
# IF the result of the voice input is a CCSR action, the appropriate cmd will be synthesized
# e.g. 'turnto <angle>' or 'set pantilt <X> <Y>'

# nlpx will try to answer queries based on its own knowledge (memory class), but if unable,
# it will pass the full query to WolframAlpha API (cloud service), and pass the most appropriate
# 'pod' (wolfram answer), back to CCSR



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


# main CCSR NLP Class
class ccsrNlpClass:

   def __init__(self, useFifos, appID):
      self.cap       = capabilitiesClass()   # CCSR capabilities
      self.ccsrmem   = memoryClass()         # memory of concepts

      # Add a concept 'I', defining CCSR identity
      self.ccsrmem.add('I')
      self.ccsrmem.concepts['I'].state = 'great'      # dynamically reflect CCSR mood by telemetry
      self.ccsrmem.concepts['I'].person = '1sg'       # 2st person singular
      self.ccsrmem.concepts['I'].isProperNoun = True 

      # This is a list of useful 'pod names' in an XML file returned by Wolfram Alpha API
      # as a result of a query.
      self.wolframAlphaPodsUsed = ('Notable facts', 'Result', 'Definition')

      # translate CCSR status dump items to concepts for ccsrmem
      self.translateStatus =  {"compass": "compass heading",
                               "temperature": "temperature",
                               "happiness": "happiness",
                               "arousal": "arousal",
                               "battery": "battery level",
                               "power": "power usage",
                               "light": "ambient light level"}

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

      self.positivePhrases = ['smart',
                              'impressive',
                              'cool']
      self.negativePhrases = ['stupid',
                              'annoying',
                              'boring']


      self.emotionMap = [['angry.',
                          'aggrevated',
                          'very excited',
                          'very happy'],
                         ['frustrated',
                          'stressed',
                          'excited',
                          'happy.'],
                         ['sad',
                          'doing OK',
                          'doing well',
                          'doing very well.'],
                         ['depressed',
                          'sleepy',
                          'bored',
                          'relaxed']]
                         


     
   def randomizedResponseVariation(self, response):
       idx = random.randint(0, len(self.responseVariations[response])-1)
       return self.responseVariations[response][idx]

   
   
   # Respone to voice input back to CCSR process as telemetry through nlp fifo
   def response(self, s):
      print'my responce is     :' + s


#      if int(self.ccsrmem.concepts['I'].properties['happiness'][1]) > 0:
#         self.ccsrmem.concepts['I'].state = 'not feeling so great'      
#      else:
#         self.ccsrmem.concepts['I'].state = 'great'      

   def getPersonalProperty(self, sa):
      # Question refers back to ccsr: how is 'your' X  
      if sa.getSentenceRole(sa.concept) in self.ccsrmem.concepts['I'].properties:
         self.response("say my " + self.ccsrmem.concepts['I'].properties[sa.getSentenceRole(sa.concept)][0] + " is " + self.ccsrmem.concepts['I'].properties[sa.getSentenceRole(sa.concept)][1])
      else:
         self.response("say I don't know what my " + sa.getSentenceRole(sa.concept) + " is ")
      
   # Main function: generate a CCSR command as response to input text.
   # Text will be from google speech2text service.
   # 'how are you' => 'say I am great'
   # 'can you look left => 'say sure', 'set pantilt 180 0 20'
   def nlpParse(self, line, debug=0):
      text = parsetree(line, relations=True, lemmata=True)
      for sentence in text:
         sa = sentenceAnalysisClass(sentence, debug)
         st = sa.sentenceType()
         if sa.debug:
            print st
            print 'concept: ' + sa.concept
         # Question state: 'how is X'
         if st == 'questionState':
           
            if sa.is2ndPersonalPronounPosessive('OBJ'):
               # Question refers back to ccsr: how is 'your' X. Look up CCSR's personal property
               self.getPersonalProperty(sa)
            elif self.ccsrmem.known(sa.getSentenceRole(sa.concept)):
               # if we know anything about the concept, we rely on CCSR memory
               if self.ccsrmem.concepts[sa.getSentenceRole(sa.concept)].state == 'none':
                  self.response("say Sorry, I don't know how " + sa.getSentencePhrase(sa.concept) + ' ' + conjugate('be', self.ccsrmem.concepts[sa.getSentenceRole(sa.concept)].person))
               else:   
                  self.response("say " + sa.getSentencePhrase(sa.concept) + " " + conjugate('be', self.ccsrmem.concepts[sa.getSentenceRole(sa.concept)].person) + " " + self.ccsrmem.concepts[sa.getSentenceRole(sa.concept)].state)
            else:
               if sa.complexQuery():
                  # Nothing is knows about the concept, and the query is 'complex', let's ask the cloud
                  self.response("say let me look that up for you")
                  # Looking up stuff makes CCSR happy and excited 
                  self.response("mood 50 50")
                  for result in self.wolframAlphaAPI(sa):
                     self.response("say " + result)              
               else:
                  self.response("say Sorry, I don't know " + sa.getSentencePhrase(sa.concept))
         # Confirm state: 'is X Y'
         elif st == 'confirmState':
            if self.ccsrmem.known(sa.getSentenceRole(sa.concept)):
               if(sa.getSentenceRole(sa.concept) == 'I'):
                  print 'somthing should go here'
               if sa.getSentencePhrase('ADJP') == self.ccsrmem.concepts[sa.getSentenceRole(sa.concept)].state:
                  self.response("facial " + str(EXPR_NODYES)) # Nod Yes 
                  self.response("say " + self.randomizedResponseVariation('yes'))
               else:
                  self.response("facial " + str(EXPR_SHAKENO)) # Shake no 
                  self.response("say " + self.randomizedResponseVariation('no'))
                  self.response("say " + sa.getSentencePhrase(sa.concept) + " " + conjugate('be', self.ccsrmem.concepts[sa.getSentenceRole(sa.concept)].person) + " " + self.ccsrmem.concepts[sa.getSentenceRole(sa.concept)].state)
                  print self.ccsrmem.concepts['I'].state
            else:
               self.response("say Sorry, I don't know " + sa.getSentencePhrase(sa.concept))
         # Question definition: 'what/who is X'
         elif st == 'questionDefinition':
            if sa.is2ndPersonalPronounPosessive('OBJ'): 
               # Question refers back to ccsr: what is 'your' X. Look up CCSR's personal property
               
               self.getPersonalProperty(sa)
                     
         # State: 'X is Y'
         elif st == 'statement':
            if sa.is2ndPersonalPronounPosessive('SBJ'): 
               # Refers back to ccsr: 'your' X is Y 
               if sa.getSentenceRole(sa.concept) not in self.ccsrmem.concepts['I'].properties:
                  self.ccsrmem.concepts['I'].properties[sa.getSentenceRole(sa.concept)] = [sa.getSentenceRole(sa.concept), sa.getSentencePhrase('OBJ')]
               self.response("say " + self.randomizedResponseVariation('acknowledge')) 
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
                  if not self.ccsrmem.known(sa.getSentenceRole(sa.concept)):
                     self.ccsrmem.add(sa.getSentenceRole(sa.concept))  
                  self.ccsrmem.concepts[sa.getSentenceRole(sa.concept)].state = sa.getSentencePhrase('ADJP')
                  self.response("say " + self.randomizedResponseVariation('acknowledge')) 
         # State locality: 'X is in Y'
         elif st == 'stateLocality':
            if not self.ccsrmem.known(sa.getSentenceRole(sa.concept)):
               self.ccsrmem.add(sa.getSentenceRole(sa.concept))  
            self.ccsrmem.concepts[sa.getSentenceRole(sa.concept)].locality = sa.getSentencePhrase('PNP')
            self.response("say " + self.randomizedResponseVariation('acknowledge')) 
         # Question locality: 'Where is X'
         elif st == 'questionLocality':
            if self.ccsrmem.known(sa.getSentenceRole(sa.concept)):
               if self.ccsrmem.concepts[sa.getSentenceRole(sa.concept)].locality == 'none':
                  # Not knowing stuff makes CCSR sad and a little aroused 
                  self.response("mood -50 20")
                  self.response("say Sorry, I don't know where " + sa.getSentencePhrase(sa.concept) + ' ' + conjugate('be', self.ccsrmem.concepts[sa.getSentenceRole(sa.concept)].person))
               else:   
                  # Knowing stuff makes CCSR happy and a little aroused 
                  self.response("mood 50 20")
                  self.response("say " + sa.getSentencePhrase(sa.concept) + " " + conjugate('be', self.ccsrmem.concepts[sa.getSentenceRole(sa.concept)].person) + " " + self.ccsrmem.concepts[sa.getSentenceRole(sa.concept)].locality)
            else:
               # Not knowing stuff makes CCSR sad and a little aroused 
               self.response("mood -50 20")
               self.response("say Sorry, I don't know " + sa.getSentencePhrase(sa.concept))
         # Command
         elif st == 'command':
            if self.cap.capable(sa.getSentenceHead('VP')):
               # Command is a prefixed CCSR command to be given through telemetry
               self.response("facial " + str(EXPR_NODYES)) # Nod Yes 
               self.response("say " + self.randomizedResponseVariation('yes') + " I can") 
               for cmd in self.cap.constructCmd(sa):
                  self.response(cmd)
            elif sa.getSentenceHead('VP') == 'tell':
               # This is a request to tell something about a topic
               if len(sa.s.pnp) > 0:
                  # We have a prepositional phrase: 'tell me about X'
                  concept = sa.reflectObject(sa.s.pnp[0].head.string)
                  if self.ccsrmem.known(concept):
                     if  len(self.ccsrmem.concepts[concept].properties) > 0:
                        for p in self.ccsrmem.concepts[concept].properties:
                           self.response("say " + self.ccsrmem.posessivePronouns[self.ccsrmem.concepts[concept].person] + " " + self.ccsrmem.concepts[concept].properties[p][0] + " is " + self.ccsrmem.concepts[concept].properties[p][1])            
                     else:
                        self.response("say sorry, I can't tell you much about " + sa.reflectObject(sa.s.pnp[0].head.string))
                  else:
                     self.response("say let me look that up for you")
                     self.response("say " + self.wolframAlphaAPI(sa))              
            else:
               # Not knowing stuff makes CCSR sad and a little aroused 
               self.response("mood -50 20")
               self.response("facial " + str(EXPR_SHAKENO)) 
               self.response("say " + self.randomizedResponseVariation('no')) 
               self.response("say I'm afraid I can't do that. I don't know how to " + sa.getSentenceHead('VP'))
         # State locality: 'X is in Y'
         elif st == 'greeting':
            self.response("say " + self.randomizedResponseVariation('hi')) 
         elif st == 'bye':
            self.response("say " + self.randomizedResponseVariation('bye'))
            # Turn away and start autonomously exploring
            self.response("turn 1 100000")
            self.response("set state 7")
         elif st == 'gratitude':
            self.response("say " + self.randomizedResponseVariation('gratitudeReply')) 
         elif st == 'adverbPhrase':
            if sa.getSentenceHead('ADJP') == 'further':
               for cmd in self.cap.lastCmd:
                  self.response(cmd)
         else:
            self.response("say sorry, I don't understand")
         self.cap.lastCmd = self.cap.constructCmd(sa)
      self.response("listen")
