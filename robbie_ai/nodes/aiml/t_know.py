#! /usr/bin/env python
'''
<star/> is not working here
commands need the short and the long name
'''

#import json_prolog


class Knowledge:
    """ 
    a class to python to return knowledge
    """
    def __init__(self):

        
        self.commands ={'test':'self.Test','euro':'self.Euro'}


    def Sort(self, x):
        y = x.split(",")
        # make this a loop
        a1= y[0].strip(' ')# strip white spaces from name
        a2 = y[1].lstrip(' ')#strip white spaces from left side ofname
        t = self.commands[a1]
        #print eval(t)(a2)
        
        return eval(t)(a2)#eval will return the function name of a string

    def Grade(self, x, a):
        a3 = a.strip('{ }')
        y = x.split(",")
        # make this a loop
        a1= y[0].strip(' ')# strip white spaces from name
        a2 = y[1].lstrip(' ')#strip white spaces from left side ofname
        t = self.commands[a1]
        #print eval(t)(a2)
        
        return eval(t)(a2, a3)#eval will return the function name of a string



    def Test(self,x,y):
        #a = x
        b =  "hi from test  " +str(x)+ ' extra  '+str(y)
        #print b
        return b


    def Euro(self,x,y):
        #a = x
        b =  "hi from euro  " +str(x)+ ' extra  '+str(y)
        #print y
        return y
    # interface to knowrob query to be tested
    def Know(self, x):
        y = x.split(",")
        # make this a loop
        a1= y[0].strip(' ')# strip white spaces from name
        a2 = y[1].lstrip(' ')#strip white spaces from left side ofname
        t = self.commands[a1]
        #prolog = json_prolog.Prolog()
        #query = prolog.query("member(A, [1, 2, 3, 4]), B = ['x', A]")
        #for solution in query.solutions():
            #print 'Found solution. A = %s, B = %s' % (solution['A'], solution['B'])
        #query.finish()

