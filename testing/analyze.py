import os
import math


import pickle
import sys
sys.path.append("..")

from utils.course import *
import config

from shapely.geometry import Polygon

d = './tests/'
folders = [os.path.join(d, o) for o in os.listdir(d)
                    if os.path.isdir(os.path.join(d,o))]

times = {}


D= {}
for test_var in range(4):
    for var in range(3):
        for deviation in range(9):
            if test_var == 0:
                D[test_var, var, deviation] = {"ttc": [],"nr_coll":0, "missed":0, "false_alarms_semi":0, "false_alarms_non":0}
            else:
                D[test_var, var, deviation] = {"nr_collisions":0, "nr_collisions_fa":0, "nr_prio_viol":0, "prio_viol_time":0, "travel_time":0, "nr_eb":0}




folders2 = []

for f in folders:

    x = f[8:].split("_")

    #print x
    test_var = int(x[0])
    variation = int(x[1])
    starting_dist = int(x[2])
    deviation = int(x[3])
    random_seed = int(x[4])

    folders2.append((f, test_var))

folders2 = sorted(folders2, key=lambda (x,y): y)


count = 0

for f,_ in folders2:
    
    count += 1

    
    collisionTime = -1

    x = f[8:].split("_")


    test_var = int(x[0])
    variation = int(x[1])
    starting_dist = int(x[2])
    deviation = int(x[3])
    random_seed = int(x[4])
    danger = int(x[5])

    with open(os.path.join(f, "0.txt"), "r") as f0, open(os.path.join(f, "1.txt"), "r") as f1:
        t0 = f0.readlines()
        t0 = [x.strip() for x in t0] 
        t1 = f1.readlines()
        t1 = [x.strip() for x in t1] 


    eb0,hs0 = eval(t0[-1])
    eb1,hs1 = eval(t1[-1])

    del t0[-1]
    del t1[-1]

    eb = False

    if not (eb0 == -1 and eb1 == -1):
        
        eb = True
        if eb0 == -1:
            eb_min = eb1
        elif eb1 == -1:
            eb_min = eb0
        else:
            eb_min = min(eb0, eb1)
        
        
    if hs0 == -1 and hs1 == -1:
        hs = 99999999999999    
    else:
        
        hs = max(hs0, hs1)


    collision = False

    for a,b in zip(t0,t1):
        x0,y0,theta0,t00 = eval(a)
        x1,y1,theta1,t11 = eval(b)

        addLength = 1.0
        yo = 1 + addLength/2

        length = 4.7 + addLength
        width = 2.0 + addLength/2.0
        
        cx0 = x0 + yo * math.cos(theta0) - (length/2) * math.cos(theta0)
        cy0 = y0 + yo * math.sin(theta0) - (length/2) * math.sin(theta0)


        toprightx0 = cx0 + length/2 * math.cos(theta0) + width/2 * math.sin(theta0)
        toprighty0 = cy0 + length/2 * math.sin(theta0) - width/2 * math.cos(theta0)

        topleftx0 = cx0 + length/2 * math.cos(theta0) - width/2 * math.sin(theta0)
        toplefty0 = cy0 + length/2 * math.sin(theta0) + width/2 * math.cos(theta0)

        bottomrightx0 = cx0 - length/2 * math.cos(theta0) + width/2 * math.sin(theta0)
        bottomrighty0 = cy0 - length/2 * math.sin(theta0) - width/2 * math.cos(theta0)

        bottomleftx0 = cx0 - length/2 * math.cos(theta0) - width/2 * math.sin(theta0)
        bottomlefty0 = cy0 - length/2 * math.sin(theta0) + width/2 * math.cos(theta0)



        cx1 = x1 + yo * math.cos(theta1) - (length/2) * math.cos(theta1)
        cy1 = y1 + yo * math.sin(theta1) - (length/2) * math.sin(theta1)


        toprightx1 = cx1 + length/2 * math.cos(theta1) + width/2 * math.sin(theta1)
        toprighty1 = cy1 + length/2 * math.sin(theta1) - width/2 * math.cos(theta1)

        topleftx1 = cx1 + length/2 * math.cos(theta1) - width/2 * math.sin(theta1)
        toplefty1 = cy1 + length/2 * math.sin(theta1) + width/2 * math.cos(theta1)

        bottomrightx1 = cx1 - length/2 * math.cos(theta1) + width/2 * math.sin(theta1)
        bottomrighty1 = cy1 - length/2 * math.sin(theta1) - width/2 * math.cos(theta1)

        bottomleftx1 = cx1 - length/2 * math.cos(theta1) - width/2 * math.sin(theta1)
        bottomlefty1 = cy1 - length/2 * math.sin(theta1) + width/2 * math.cos(theta1)
        


        p1 = Polygon([(toprightx0,toprighty0), (topleftx0,toplefty0), (bottomrightx0,bottomrighty0), (bottomleftx0,bottomlefty0)])
        p2 = Polygon([(toprightx1,toprighty1), (topleftx1,toplefty1), (bottomrightx1,bottomrighty1), (bottomleftx1,bottomlefty1)])
        

        if p1.intersects(p2) and not (eb and t00 > hs):
            collision = True
            collisionTime = t00
            break


    r = config.rate / config.speedup

    time0 = int(eval(t0[-1])[-1]) / r
    time1 = int(eval(t1[-1])[-1]) / r

    
    #D[variation][deviation]["finish_times"][0][starting_dist] = time0
    
    #D[variation][deviation]["finish_times"][1][starting_dist] = time1

    if test_var == 0:
        
        if danger == 2 and starting_dist == 0:
            if deviation in [0,2,3,4]:
                times[variation, deviation] = time0

        if danger == 0:
            if collisionTime == -1:
                print "no collision. test =", f
                
            else:
                D[test_var, variation, deviation]["nr_coll"] += 1
                if eb:
                    ttc = (collisionTime - eb_min) / r
                    D[test_var, variation, deviation]["ttc"].append(ttc)
                else:
                    print "missed. test=", f
                    D[test_var, variation, deviation]["missed"] += 1
        
        if danger == 1:
            if collision:
                print "collision on danger 1. test=", f
            if eb:
                D[test_var, variation, deviation]["false_alarms_semi"] += 1
        if danger == 2:
            if collision:

                print "collision on danger 2. test=", f
            if eb:
                D[test_var, variation, deviation]["false_alarms_non"] += 1

    else:
        if danger == 0:
            if collision:
                print "collision. test =", f
                D[test_var, variation, deviation]["nr_collisions"] += 1
        else:
            if collision:
                print "collision. test =", f
                D[test_var, variation, deviation]["nr_collisions_fa"] += 1

        
        opttime = 1
        
        if deviation in [0,2,3,4]:
            opttime = times[var, deviation]
        else:
            opttime = times[var,0]
        

        
        prio_lost = time0 - opttime

        if prio_lost > 0.5:
            D[test_var, variation, deviation]["nr_prio_viol"] += 1
            D[test_var, variation, deviation]["prio_viol_time"] += prio_lost

        if eb:
            if test_var == 3 and deviation != 1:
                print "triggered eb in testvar3. Test=", f
            D[test_var, variation, deviation]["nr_eb"] += 1

        D[test_var, variation, deviation]["travel_time"] += time0 + time1            


def save_obj(obj, name ):
    with open(name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def load_obj(name ):
    with open(name + '.pkl', 'rb') as f:
        return pickle.load(f)    


save_obj(D, "D")



