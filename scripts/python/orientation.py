# -*- coding: iso-8859-15
import math
import sys

if len(sys.argv) >= 2:

    pitch = float( sys.argv[1] )
    roll  = float( sys.argv[2] )

    inc = math.sqrt(math.pow(pitch, 2) + math.pow(roll, 2))
    ori = (360 - (math.atan2(roll, -pitch)) * 180/math.pi) % 360
    
    newPitch = -inc * math.cos(ori * math.pi/180)
    newRoll = -inc * math.sin(ori * math.pi/180)
    
    
    print("pitch> ", pitch)
    print("roll> ", roll)
    print("inclination> ", inc)
    print("orientation> ", ori)
    print("new pitch> ", newPitch)
    print("new Roll> ", newRoll)

else:
	print "Este programa necesita un parámetro";
