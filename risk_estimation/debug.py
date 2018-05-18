from driver import *
from Intersection import *
from threading import Thread, Lock

plot = True

#for the use of a debugger 
with open("debug.txt") as f:
    first = True
    for line in f:
        x = eval(line[:-1]) # (timestamp, measurement_vector)
        if first:
            re = RiskEstimator(200, Intersection(), x[1], np.eye(3)*0.05, 0.05, x[0], Lock(), plot, True)
            id, Ic, Is = x[2]
            re.setKnownIc(id, Ic)
            #re.setKnownIs(id, Is)
            first = False

        else:
            print x[0] # timestamp
            

            id, Ic, Is = x[2]
            #re.setKnownIc(id, Ic)
            #re.setKnownIs(id, Is)
            re.update_state(x[0], x[1])
        