from driver import *
from Intersection import *

plot = True

with open("debug.txt") as f:
    first = True
    for line in f:
        x = eval(line[:-1])
        if first:
            re = RiskEstimator(400, Intersection(), x[1], np.eye(3)*0.05, 0.05, x[0], plot)
            first = False

        else:
            print x[0]
            re.update_state(x[0], x[1])
        