'''
default variables

bathroom
auto_dock
mats_room
tims_room
entry
dining_room
kitchen
>>> for key, value in room.iteritems():
...     print value
... 
[1.5, 1.5, 90]
[0, 0, 0]
[0.543, -4.742, 0]
[2.159, -12.441, -90]
[3.622, -1.923, 90]
[2.975, -7.595, 37]
[5.027, -5.892, 0]




'''

room ={
    'bathroom': [1.5,1.5,90], 
    'kitchen': [5.027,-5.892,0],
    'auto_dock':[0,0,0], 
    'mats_room':[0.543,-4.742,0], 
    'tims_room':[2.159,-12.441,-90], 
    'entry':[3.622,-1.923,90], 
    'dining_room':[2.975,-7.595,37]
}

turn = {
    'left':[0,0,90],
    'right':[0,0,-90]
}
    
move = {
    'right':[0.5,-0.5,90],
    'left':[0.5,0.5,-90],
    'forward':[1,0,0],
    'backward':[-1,0,0]
}


