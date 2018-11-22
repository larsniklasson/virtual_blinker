import os
import math


import pickle
import sys
sys.path.append("..")

from utils.course import *
import config

from shapely.geometry import Polygon

d = './a0'
folders = [os.path.join(d, o) for o in os.listdir(d) 
                    if os.path.isdir(os.path.join(d,o))]
d = './a1'
folders += [os.path.join(d, o) for o in os.listdir(d) 
                    if os.path.isdir(os.path.join(d,o))]
d = './a2'
folders += [os.path.join(d, o) for o in os.listdir(d) 
                    if os.path.isdir(os.path.join(d,o))]
d = './a3'
folders += [os.path.join(d, o) for o in os.listdir(d) 
                    if os.path.isdir(os.path.join(d,o))]


D = {}

#test_var
for k in range(4):
    D[k] = {}

    #variation
    for i in range(3):
        D[k][i] = {}

        #deviation
        for j in range(7):
            if k == 0:
                D[k][i][j] = {"c": {"count":0, "missed": [], "ttc": []}, "s" : {"count": 0, "eb": 0}, "n": {"count":0, "eb": 0}}
            else:
                D[k][i][j] = {"nr_collisions": 0, "nr_prio_viol": 0, "prio_viol_time": 0, "throughput": 0}


time0_list = {}
for i in range(3):
    for j in range(3):
        time0_list[i,j] = 999999


for f in folders:
    x = f[5:].split("_")
    test_var = int(x[0])
    variation = int(x[1])
    starting_dist = int(x[2])
    deviation = int(x[3])
    random_seed = int(x[4])


    if test_var == 2 and starting_dist == 0:
         with open(os.path.join(f, "0.txt"), "r") as f0, open(os.path.join(f, "1.txt"), "r") as f1:
            t0 = f0.readlines()
            t0 = [x.strip() for x in t0] 

            del t0[-1]
            time0 = int(eval(t0[-1])[-1]) / r

            time0_list[variation, deviation] = time0
            




print time0_list
sys.exit(0)



for f in folders:

    try:
        x = f[5:].split("_")
    
    
        test_var = int(x[0])
        variation = int(x[1])
        starting_dist = int(x[2])
        deviation = int(x[3])
        random_seed = int(x[4])

        c0 = config.intersection.courses["north", "straight"]
        
        if variation == 0:
            c1 = config.intersection.courses["south", "left"]
            ex = config.intersection.getExtras("north", "straight", "south", "left")
            d0 = config.intersection.extra_distances[ex[0]]
            d1 = config.intersection.extra_distances[ex[1]]
            
        
        if variation == 1:
            c1 = config.intersection.courses["west", "straight"]
            ex = config.intersection.getExtras("north", "straight", "west", "straight")
            d0 = config.intersection.extra_distances[ex[0]]
            d1 = config.intersection.extra_distances[ex[1]]

        if variation == 2:
            c1 = config.intersection.courses["west", "right"]
            ex = config.intersection.getExtras("north", "straight", "west", "right")
            d0 = config.intersection.extra_distances[ex[0]]
            d1 = config.intersection.extra_distances[ex[1]]

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




        if test_var == 0:
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

        collision = False

        for a,b in zip(t0,t1):
            x0,y0,theta0,t00 = eval(a)
            x1,y1,theta1,t11 = eval(b)

            length = 4.7+0.1
            width = 2.0+0.05
            
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

        r = config.rate #60.0

        time0 = int(eval(t0[-1])[-1]) / r
        time1 = int(eval(t1[-1])[-1]) / r

        
        #D[variation][deviation]["finish_times"][0][starting_dist] = time0
        
        #D[variation][deviation]["finish_times"][1][starting_dist] = time1

        
        if collision:
            D[variation][deviation]["c"]["count"] += 1
            if not eb:
                D[variation][deviation]["c"]["missed"].append(starting_dist)
            else:
                D[variation][deviation]["c"]["ttc"].append((starting_dist, (collisionTime - eb_min) / r))
            
        else:
            gap = (gap_0 - gap_1) / r
            if gap > -1.5 and gap < 2.0:
                D[variation][deviation]["s"]["count"] += 1
                if eb:
                    D[variation][deviation]["s"]["eb"] += 1
                    
            else:
                D[variation][deviation]["n"]["count"] += 1
                if eb:
                    D[variation][deviation]["n"]["eb"] += 1
            
    except:
        print "error on ", f
    


def save_obj(obj, name ):
    with open('obj/'+ name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def load_obj(name ):
    with open('obj/' + name + '.pkl', 'rb') as f:
        return pickle.load(f)    


save_obj(D, "test_var0_dict")



