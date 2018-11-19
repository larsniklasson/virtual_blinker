import os
import math

from shapely.geometry import Polygon

d = '.'
folders = [os.path.join(d, o) for o in os.listdir(d) 
                    if os.path.isdir(os.path.join(d,o))]


for f in folders:
    print "f ", f
    
    x = f[2:].split("_")
    
    test_var = x[0]
    variation = x[1]
    starting_dist = x[2]
    deviation = x[3]
    random_seed = x[4]

    with open(os.path.join(f, "0.txt"), "r") as f0, open(os.path.join(f, "1.txt"), "r") as f1:
        t0 = f0.readlines()
        t0 = [x.strip() for x in t0] 
        t1 = f1.readlines()
        t1 = [x.strip() for x in t1] 

    
    eb = False
    eb_time = 99999999999999

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


    




    has_reached_p_0 = False
    has_reached_p_1 = False

    
    p0 = (3.25, -2.374208829065749)
    p1 = (3.25, 3.25)
    p2 = (3.25, 7.5)

    p_variations = [p0, p1, p2]
    
    p = p_variations[int(variation)]


    collision = False

    for a1 in t0:

        x0,y0,theta0,t00 = eval(a1)

        if not has_reached_p_0:
            d = math.sqrt( (x0 - p[0])**2 + (y0-p[1])**2)
            
            if  d < 0.5:
                has_reached_p_0 = True
                gap_0 = t00

    for b1 in t1:
        x1,y1,theta1,t11 = eval(b1)


        
        if not has_reached_p_1:
            d = math.sqrt((x1-p[0])**2+(y1-p[1])**2)
            #print d
            if (d < 1.5 and int(variation) == 2) or d < 0.75:
                has_reached_p_1 = True
                gap_1 = t11


    
    for a,b in zip(t0,t1):
        x0,y0,theta0,t00 = eval(a)
        x1,y1,theta1,t11 = eval(b)


        #0 edges

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
        

        
        if p1.intersects(p2) and not collision:

            print eb, t00, hs
            if (eb and t00 > hs):
                pass
            else:
                collision = True
                collisionTime = t00


    print "eb", eb
    print "time: ", int(eval(t0[-1])[-1]), int(eval(t1[-1])[-1])

    print "collision: ", collision
    if collision:
        if int(test_var) == 0:
            print "ttc: ", (collisionTime - eb_min) / 30.0
    print "gap", (gap_0 - gap_1) / 30.0
