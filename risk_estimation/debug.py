
import sys
sys.path.append("..")
from config import *
from driver import *
import time
import random
import numpy as np
plot = True

xy_deviation = SIM_CONFIG["xy_deviation"]
theta_deviation = SIM_CONFIG["theta_deviation"]
speed_deviation = SIM_CONFIG["speed_deviation"]

deviations = xy_deviation, theta_deviation, speed_deviation

total_nr_particles = SIM_CONFIG["total_nr_particles"]

random.seed()
np.random.seed()
plotafter = 0

with open("debug.txt") as f:
    first = True
    for line in f:
        x = eval(line[:-1]) # (timestamp, measurement_vector)
        if first:
            pppf = total_nr_particles / (len(x[1])**2)
            re = RiskEstimator(pppf, x[1], x[0], deviations, plot, plotafter)
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
