from driver import *
from Intersection import *
from threading import Thread, Lock
import time

start_time = time.time()
print start_time

plot = True

xy_var = 0.15 *10
theta_var = 0.05*10
speed_var = 0.10*10

cov = np.array([[xy_var, 0, 0], [0, xy_var, 0], [0, 0, theta_var]])

#for the use of a debugger 
with open("debug.txt") as f:
    first = True
    for line in f:
        x = eval(line[:-1]) # (timestamp, measurement_vector)
        if first:
            re = RiskEstimator(1000, Intersection(), x[1], cov, speed_var, x[0], Lock(), plot, plot)
            id, Ic, Is = x[2]
            re.setKnownIc(id, Ic)
            re.setKnownIs(id, Is)
            first = False

        else:
            print x[0] # timestamp
            

            id, Ic, Is = x[2]
            re.setKnownIc(id, Ic)
            re.setKnownIs(id, Is)
            re.update_state(x[0], x[1])
        

endtime = time.time()

print "runtime: ", (endtime - start_time)