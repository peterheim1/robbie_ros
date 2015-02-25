# Natural Language Processing Sentence Analysis class.
# This class contains tools for interpreting a text sentence (converted from audio by Google
# speech to text service).


import sys
import re

from pattern.en import parse
from pattern.en import pprint
from pattern.en import parsetree
from pattern.en import wordnet
from pattern.en import pluralize, singularize
from pattern.en import conjugate, lemma, lexeme


# Sentence Salysis Class. This is instantiated with a pattern.en sentence class
class sentenceAnalysisClass:
   def __init__(self, s, debug=0):
      self.s = s    # pattern.en sentence class
      self.concept    = ''
      self.property   = ''
      self.debug   = False
      
      # Dictionary of chunk sequence reqexps related to a specific sentence type
      self.chunkSequences = [[re.compile(':VP(:ADVP|:PP)'), 'command'],     # e.g. turn 180 degrees
                             [re.compile(':NP:VP:(ADJP|NP)'), 'statement'], # X is Y
                             [re.compile(':NP:VP:PP:NP'), 'stateLocality'],  # X is in Y
                             [re.compile('(:NP)*:ADJP'), 'adverbPhrase']     # a little further, a lot less   
                            ]

      # Dictionary containing groups of words (nouns, verbs, etc) with which CCSR can associate an idea or action  
      self.wordRef = {
                      'stateVerbs': ('be'),                                                         # verbs that make a statement
                      'commandVerbs':('turn', 'go', 'look', 'track', 'find', 'can','get', 'pick', 'put'), # verbs that CCSR can interpret as command
                      'greetings':('hello', 'hi', 'greetings', 'hey'),                                      # words that constitute a greeting
                      'bye':('goodbye', 'adieu')                                      # words that constitute a greeting
                      }
      self.reflex = {'I':'you', 'you':'I', 'yourself':'I'}
      if debug: 
         print self.s
         self.debug = True
         for chunk in self.s.chunks:
            print chunk.type, chunk.role, chunk.head, [(w.string, w.type) for w in chunk.words]

   # Create a ':' separated string from the sequential listo of chunks in the analysed sentence.
   # This string is used for regular expression matching later
   # e.g.: a pattern tree containing <NP><VP><NP> yields "NP:VP:NP"
   def chunkToString(self):
      a = ''
      for chunk in self.s.chunks:
         a = a + ':' + chunk.type
      return a

   # Return first chunk in sentence of type t
   def getFirstChunk(self, t):
      for chunk in self.s.chunks:
         if chunk.type == t:
            return chunk
      return None

   def getNthChunk(self, t, n):
      x = 0
      for chunk in self.s.chunks:
         if chunk.type == t:
            if x == n:
               return chunk
            x = x + 1 
      return None
   
   # Return the chunk sequence type of the analysed sentence
   # e.g.: <NP><VP><ADJP> would match a 'statement', such as 'the dog is brown'
   # This function is ony called as a sub-function by self.sentenceType
   def matchChunk(self):
      m = self.chunkToString()
      for seq in self.chunkSequences:
         p = seq[0]
         if p.match(m):
            return seq[1]
                    
   # Return type of sentence. Types are encoded es strings: e.g. 'statement', 'command', 'questionLocality', etc   
   # e.g. 'the cat is in the garden' = <NP><VP><PP><NP> => "stateLocality"
   def sentenceType (self):
      self.concept  = 'OBJ'
      self.property = 'SBJ'
      if (len(self.s.chunks) > 0):
          # We have chunks
          if (self.s.chunks[0].start == 0):
             # Sentence starts with a chunk
             if (self.s.chunks[0].type == 'VP'):
                # First chunk is verb-phrase
                if (self.s.chunks[0].head.lemma in self.wordRef['stateVerbs']):
                   return 'confirmState'              # is X Y?
                if (self.s.chunks[0].head.lemma in self.wordRef['commandVerbs']):
                   return 'command'                   # can you X Y
                if (self.s.chunks[0].head.lemma == 'do'):
                   if self.getSentenceRole('OBJ') == 'I':
                      if self.getNthChunk('VP',1).head.lemma == 'know':
                         self.concept = 'SBJ'
                         return self.sentenceType_WH()
             else:
                # use regexp-based chunk matching to find sentence type
                self.concept  = 'SBJ'
                if self.s.chunks[0].head.lemma == 'thank':
                   return 'gratitude'
                if self.s.chunks[0].head.string in self.wordRef['bye']:
                   return 'bye'
                return self.matchChunk()
          else:
             if (self.s.words[0].type == 'DT'):
                # Sentence starts with determiner: 'a little more, the big dog'
                return self.matchChunk()
             elif (self.s.words[0].type == 'UH'):
                # Sentence starts with determiner: 'a little more, the big dog'
                if self.s.words[0].string in self.wordRef['greetings']:
                   return 'greeting'
             else:
                # Sentence starts with an un-chunked word: 'what, where, how, etc'
                return self.sentenceType_WH()
      else:
          # no chunks: e.g. interjections: hello, wow, etc
          w = self.getFirstWord('UH')
          if (w != None):
              if w.string in self.wordRef['greetings']:
                  return 'greeting'

   # Return sentence type for WH-words: 'what, where, how, etc'
   def sentenceType_WH(self):
       w = self.getFirstWord('WRB')
       if (w != None):
          if (w.string == 'where'):
             return 'questionLocality'          # where is X
          elif (w.string == 'how'):
             return 'questionState'             # how is X
       w = self.getFirstWord('WP')
       if (w != None):
          if (w.string == 'what' or w.string == 'who'):              # what/who is X
             return 'questionDefinition'
          if (w.string == 'who'):               # who is X
             return 'questionProperNoun'
       return None
                
   # Return first word in the sentence of type 'type', return None if non-existent
   def getFirstWord(self, type):
      for w in self.s.words:
         if w.type == type:
            return w
      return None

   # Return string representing the primary word in the chunk with a specific role 'role'
   # e.g. role=OBJ for 'the yellow cat is in the garden' will return 'cat', being the primary object.
   # Strings will be reflected if applicable: you<->I, I<->you etc. This way, if CCSR is addressed with
   # 'how are you', the OBJ of the sentency is 'I', which corresponds with CCSR's identity
   # All Personal proper nouns are concatenated, assuming they form one name: 'who is Michael Jackson' => 'Michael Jackson' 
   def getSentenceRole (self, role):
      p = ''
      for chunk in self.s.chunks:
         if (chunk.role == role) or (chunk.type == role):
            if chunk.head.type == 'NNP-PERS':
                # Main word is personal name, so collect all other NNP-PERS in the phrase assuming they are part of the name
                for w in chunk.words:
                    if w.type == 'NNP-PERS':
                        p = p + w.string
                        if chunk.words.index(w) != len(chunk.words) - 1:
                           p = p + ' '
            else:
                # Sentence role is a thing, not a person
                p = chunk.head.string
            return self.reflectObject(p)
      return 'none'

   # Return True if the sentence 'role' is a 2nd-person posessive pronoun: e.g. 'your'
   # e.g. role=OBJ of 'what is your age' => True
   def is2ndPersonalPronounPosessive (self, role):
      for chunk in self.s.chunks:
         if chunk.role == role:
            for word in chunk.words:
                if word.type == 'PRP$':
                   p = re.compile("you.*")
                   return p.match(word.string)
      return False

   # Return string containg full phrase of a type of role. SO tag can be OBJ, SBJ, etc (role) or NP, VP, etc (type)
   # e.g. role=SBJ 'the yellow cat eats the brown bird' => 'the brown bird'
   def getSentencePhrase (self, tag):
      if tag == 'PNP':
         # FOr now, only support one prepositional noun phrase, e.g  'in the garden'
         return self.s.pnp[0].string 
      else:
         for chunk in self.s.chunks:
            if chunk.role == tag:
               if chunk.nearest('VP').head.type != 'MD':
                  return self.reflectObject(chunk.string)
            elif chunk.type == tag:
               return self.reflectObject(chunk.string)

   # Return first chunk with specified tag or role
   def getSentenceChunk (self, tag):
      for chunk in self.s.chunks:
         if chunk.role == tag:
            return chunk
         elif chunk.type == tag:
            return chunk



   # Get head (most imporant word) of phrase with a specific role or type. This is used to filter out the key
   # 'concept' that CCSR uses to access its memory
   # If a verb, get lemma (root of verb)
   # e.g. role=SBJ 'the yellow cat eats the brown bird' => 'bird' 
   def getSentenceHead (self, tag):
      for chunk in self.s.chunks:
         if chunk.role == tag:
            return self.reflectObject(chunk.head.lemma)
         elif chunk.type == tag:
            if chunk.head.type != 'MD':      # Filter out modal Auxilaries (can/could/etc)
               return self.reflectObject(chunk.head.lemma)

   # Reflect words if applicable: you<->I, my<->your, etc
   def reflectObject(self, obj):
      if obj in self.reflex:
         return self.reflex[obj]
      else:
         return obj  

   # called when sentence is a query, returns True if query is too complex to CCSR to resolve wih tits own concept memory,
   # and nlp should pass on query to cloud
   def complexQuery(self):
       # Currently definition of a 'complex' query is
       #   - concept phrase has more than 2 words (excluding a determiner)
       # e.g. 'where is the cat' => simple
       # e.g. 'where is the largest cat in the world' => complex
       print self.concept
       return (len(self.getSentenceChunk(self.concept).words) > 2) or (self.getSentenceChunk(self.concept).words[0].type != 'DT')
