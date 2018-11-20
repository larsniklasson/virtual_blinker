import os
import math


import sys
sys.path.append("..")

from utils.course import *
import config

from shapely.geometry import Polygon

d = '.'
folders = [os.path.join(d, o) for o in os.listdir(d) 
                    if os.path.isdir(os.path.join(d,o))]



D = {}

for i in range(3):
    D[i] = {}

    for j in range(7):
        D[i][j] = {"collisions": [], "nr_eb" : 0}


for f in folders:


    x = f[2:].split("_")

    #print x
    variation = int(x[1])
    starting_dist = float(x[2])
    deviation = int(x[3])

    if int(variation) == 0:
        ex = config.intersection.getExtras("north", "straight", "south", "left")
        d0 = config.intersection.extra_distances[ex[0]]
        d1 = config.intersection.extra_distances[ex[1]]
        c0 = config.intersection.courses["north", "straight"]
        c1 = config.intersection.courses["south", "left"]
        
    
    if int(variation) == 1:
        ex = config.intersection.getExtras("north", "straight", "west", "straight")
        d0 = config.intersection.extra_distances[ex[0]]
        d1 = config.intersection.extra_distances[ex[1]]
        c0 = config.intersection.courses["north", "straight"]
        c1 = config.intersection.courses["west", "straight"]

    if int(variation) == 2:
        ex = config.intersection.getExtras("north", "straight", "west", "right")
        d0 = config.intersection.extra_distances[ex[0]]
        d1 = config.intersection.extra_distances[ex[1]]
        c0 = config.intersection.courses["north", "straight"]
        c1 = config.intersection.courses["west", "right"]

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

    for a1 in t0:

        x0,y0,theta0,t00 = eval(a1)

        if c0.getDistance(x0,y0) > c0.distance_at_crossing + d0:
            gap_0 = t00
            break

        
    for b1 in t1:
        x1,y1,theta1,t11 = eval(b1)

        if c1.getDistance(x1,y1) > c1.distance_at_crossing + d1:
            gap_1 = t11
            break


    for a,b in zip(t0,t1):
        x0,y0,theta0,t00 = eval(a)
        x1,y1,theta1,t11 = eval(b)

        length = 4.7+0.2
        width = 2.0+0.1
        
        cx0 = x0 + 1 * math.cos(theta0) - (length/2) * math.cos(theta0)
        cy0 = y0 + 1 * math.sin(theta0) - (length/2) * math.sin(theta0)


        toprightx0 = cx0 + length/2 * math.cos(theta0) + width/2 * math.sin(theta0)
        toprighty0 = cy0 + length/2 * math.sin(theta0) - width/2 * math.cos(theta0)

        topleftx0 = cx0 + length/2 * math.cos(theta0) - width/2 * math.sin(theta0)
        toplefty0 = cy0 + length/2 * math.sin(theta0) + width/2 * math.cos(theta0)

        bottomrightx0 = cx0 - length/2 * math.cos(theta0) + width/2 * math.sin(theta0)
        bottomrighty0 = cy0 - length/2 * math.sin(theta0) - width/2 * math.cos(theta0)

        bottomleftx0 = cx0 - length/2 * math.cos(theta0) - width/2 * math.sin(theta0)
        bottomlefty0 = cy0 - length/2 * math.sin(theta0) + width/2 * math.cos(theta0)



        cx1 = x1 + 1 * math.cos(theta1) - (length/2) * math.cos(theta1)
        cy1 = y1 + 1 * math.sin(theta1) - (length/2) * math.sin(theta1)


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


    #print "eb", eb
    #print "time: ", int(eval(t0[-1])[-1]), int(eval(t1[-1])[-1])

    r = 60.0

    time0 = int(eval(t0[-1])[-1]) / r
    time1 = int(eval(t1[-1])[-1]) / r

    
    #D[variation][deviation]["finish_times"][0][starting_dist] = time0
    
    #D[variation][deviation]["finish_times"][1][starting_dist] = time1

    #print "collision: ", collision
    

    
    if collision:
        D[variation][deviation]["collisions"].append(starting_dist)
    
    if eb:
        D[variation][deviation]["nr_eb"] += 1


            
    
    
    


print D[0]

print D[0][0]["collisions"]
print D[0][0]["nr_eb"]