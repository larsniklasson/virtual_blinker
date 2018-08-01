#!/usr/bin/env python

import sys
sys.path.append("..")
from config import *
import time
import random
import numpy as np
import virtual_blinker.msg as cm 
import rospy
from RE2 import *
from config import *
from utils.Intersection import *



#--------NOTE: THIS FILE NOT UPDATED--------------
random.seed(1)
np.random.seed(1)

intersection = Intersection()

rospy.init_node('debugger')

try:
    vis = rospy.get_param('vis')
except:
    vis = 0

if vis: rospy.sleep(5)

c = 0
with open("debug.txt") as f:
    for line in f:
        c += 1
        if rospy.is_shutdown():
            break

        t, ms, ds = eval(line[:-1])

        if c == 1:
            nr_cars = len(ms)
            travelling_directions = [intersection.getTravellingDirection(x, y, theta) for (x, y, theta, _) in ms]
            re = RiskEstimator(travelling_directions)
            publishers = [rospy.Publisher("true_car_state" + str(i), cm.CarStateTrue, queue_size=50) for i in range(nr_cars)]


        
        re.update_state(t, ms, ds)


        for id,(x,y,theta,s) in zip(range(len(ms)),ms):
            m = cm.CarState(x,y,theta,s,id,0)
            publishers[id].publish(m)
        
        """
        print
        print "intentions 3:"
        print re.intentionDensities[3]
        print "risk:"
        for i in range(nr_cars):
            print i, ":", re.getRisk(i)
        
        
        
        if c % 5 == 0 and t > 2:
            raw_input()
        """
        
