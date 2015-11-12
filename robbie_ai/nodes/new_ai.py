#!/usr/bin/python
 
import aiml

 
k = aiml.Kernel()
 
# load the aiml file
#k.learn("standard/std-*.aiml")
k.learn("firsttry.aiml")
#k.learn("commands.aiml")
#k.learn("standard/std-*.aiml")
 
# set a constant
k.setBotPredicate("name", "Minion")
k.setBotPredicate("location","Australia")
k.setBotPredicate("botmaster","Petrus")
k.setBotPredicate("master","Petrus")
k.setBotPredicate("birthday","1st of August 2011")
k.setBotPredicate("gender","male")
k.setBotPredicate("favoriteactor","walle")
k.setBotPredicate("favoriteactress","eve")
k.setBotPredicate("favoriteband","ELO")
k.setBotPredicate("favoritebook","The Iliad")
k.setBotPredicate("favoritecar","Mercedes")
k.setBotPredicate("favoritecolor","sliver")
k.setBotPredicate("favoritedrink","oil")
k.setBotPredicate("favoritefood","power")
k.setBotPredicate("favoriteicecream","power")
k.setBotPredicate("favoritemovie","star wars")
k.setBotPredicate("favoritesong","mister roboto")
k.setBotPredicate("favoritesport","darpa cup")
k.setBotPredicate("favoritetvshow","Futurama")


 
while True:
    input = raw_input("> ")
    response = k.respond(input)
    # print out on the shell
    print response
    
