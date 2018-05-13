from driver import *
from Intersection import *

plot = True

#for the use of a debugger 
with open("debug.txt") as f:
    first = True
    for line in f:
        x = eval(line[:-1]) # (timestamp, measurement_vector)
        if first:
            re = RiskEstimator(400, Intersection(), x[1], np.eye(3)*0.05, 0.05, x[0], plot)
            first = False

        else:
            print x[0] # timestamp
            re.update_state(x[0], x[1])
        