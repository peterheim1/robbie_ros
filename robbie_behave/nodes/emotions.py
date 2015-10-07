#!/usr/bin/env python
'''

neutral", "vigilant", "happy", "sad", "bored", "exited", "angry", "relaxed", "depressed

'happy'; r0, 'excited': r0, 'bored': r0,

r = 5 
neutral =0,0
vigilant =0,r
happy	=r,0
sad	=-r,0
bored	=0,-r
exited = r*0.7071,r*0.7071
angry = -r*0.7071,r*0.7071
relaxed = r*0.7071,-r*0.7071
depressed = -r*0.7071,-r*0.7071


e = [distance.euclidean(s,neutral),distance.euclidean(s,vigilant),distance.euclidean(s,happy),distance.euclidean(s,sad),distance.euclidean(s,bored),distance.euclidean(s,exited),distance.euclidean(s,angry),distance.euclidean(s,relaxed),distance.euclidean(s,depressed)]

a.index(min(a)) #return the min value

b= ["neutral", "vigilant", "happy", "sad", "bored", "exited", "angry", "relaxed", "depressed"]

print b[a.index(min(a))]

for item in b:
    distance.euclidean(s,item)
    print item.index
   

'''
   def Emotion_State(self, s):
        '''
         open file and read data as dictionary

        '''
        #s = s
        r = 5 
        neutral = 0,0
        vigilant =0,r
        happy	=r,0
        sad	=-r,0
        bored	=0,-r
        exited = r*0.7071,r*0.7071
        angry = -r*0.7071,r*0.7071
        relaxed = r*0.7071,-r*0.7071
        depressed = -r*0.7071,-r*0.7071
        b= ["neutral", "vigilant", "happy", "sad", "bored", "exited", "angry", "relaxed", "depressed"]
        a = [distance.euclidean(s,exited),distance.euclidean(s,angry),distance.euclidean(s,relaxed),distance.euclidean(s,depressed),distance.euclidean(s,neutral),distance.euclidean(s,vigilant),distance.euclidean(s,happy),distance.euclidean(s,sad),distance.euclidean(s,bored)]

        return b[a.index(min(a))]

