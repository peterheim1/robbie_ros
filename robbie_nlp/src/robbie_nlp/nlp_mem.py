#!/usr/bin/python

# nlpx memory class. This class provides the CCSR NLP module with a concept-based memory. A concept
# is extracted from an input sentence: e.g. 'the cat is very yellow' -> concept='cat', and
# the memory stores information about this concept, e.g. state='very yellow'
# Currently CCSR memory stores state, locality and properties.

# CCSR has a concept 'I', which contains information about itself. In the future, CCSR state such
# as battery level, location etc will be dumped from the CCSR process (by telemetry), and voice input can
# query these properties: e.g. 'what is your battery level'

import sys
import re

from pattern.en import parse
from pattern.en import pprint
from pattern.en import parsetree
from pattern.en import wordnet
from pattern.en import pluralize, singularize
from pattern.en import conjugate, lemma, lexeme

# Information about a single concept
class conceptClass:
   def __init__(self, state='none', locality='none'):
      self.state = state          # what/how is 'concept'
      self.reference = 'none'     # unused
      self.locality = locality    # where is 'concept'
      self.person = '3sg'         # e.g. a thing is 3rd-person, singular
      self.isProperNoun = False   # True if proper noun: e.g. Robert
      self.properties = {}        # Dict of custom properties, e.g. 'age' = 39, 'color' = 'blue'

# CCSR memory class. Collection of concepts      
class memoryClass():

   def __init__(self):
      self.concepts = {}
      self.person = {'I': '1sg',
                     'you': '2sg'
                     }
      self.posessivePronouns = {'1sg': 'my',
                                '2sg': 'your',
                                '3sg': 'its'
                     }

   # Add a concept to memory
   def add(self, c):
      self.concepts[c] = conceptClass()
      if c in self.person:
         self.concepts[c].person = self.person[c]
      else:
         self.concepts[c].person = '3sg'

   # Return True if concept 'c' (string) is in memory
   def known(self, c):
      return (c in self.concepts)






