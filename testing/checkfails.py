import os
import math

import sys
sys.path.append("..")

from utils.course import *
import config


d = './tests/'
folders = [os.path.join(d, o) for o in os.listdir(d)
                    if os.path.isdir(os.path.join(d,o))]



#print folders2
for f in folders:
    #print f
    
    try:
        with open(os.path.join(f, "0.txt"), "r") as f0, open(os.path.join(f, "1.txt"), "r") as f1:
            t0 = f0.readlines()
            t0 = [x.strip() for x in t0] 
            t1 = f1.readlines()
            t1 = [x.strip() for x in t1] 


        eb0,hs0 = eval(t0[-1])
        eb1,hs1 = eval(t1[-1])

    except:
        print f


    
    