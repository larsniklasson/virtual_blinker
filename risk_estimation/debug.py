from driver import *
from Intersection import *

plot = True

with open("debug.txt") as f:
    first = True
    l = 0
    for line in f:
        l += 1
        print l
        if l == 9:
            pass

        x = eval(line[:-1])
        if first:
            re = RiskEstimator(400, Intersection(), x[1], np.eye(3)*0.05, 0.05, x[0], plot, "/home/lars/catkin_ws/src/virtual_blinker/risk_estimation/plotfolder")
            #re.setKnownIc(1, "straight")
            #re.setKnownIs(1, "go")
            first = False

        else:
            print x[0]
            re.update_state(x[0], x[1])
        