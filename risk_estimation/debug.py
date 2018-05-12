from driver import *
from Intersection import *
with open("debug.txt") as f:
    first = True
    for line in f:
        x = eval(line[:-1])
        if first:
            re = RiskEstimator(400, Intersection(), x[1], np.eye(3)*0.1, 1.5, x[0])
            first = False

        re.update_state(x[0], x[1])
        