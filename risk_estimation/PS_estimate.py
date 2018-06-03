import math
import utils
import numpy as np
import sys
sys.path.append("..")
from config import *

pdc = RISK_CONFIG["prediction_dev_coeff"]

def PS_estimate((x,y,theta,speed), Is, Ic, travelling_direction, intersection, interval, deviations, no_sample = False):
    c = intersection.courses[travelling_direction, Ic]
    new_PS = c.predictNextState(x, y, theta, speed, interval, Is)

    if no_sample:
        return new_PS
    
    
    return sample(new_PS, deviations)


def sample((xnew, ynew, thetanew, newspeed), deviations):
    xy_deviation, theta_deviation, speed_deviation = deviations

    x_estimate = np.random.normal(xnew, xy_deviation / pdc)
    y_estimate = np.random.normal(ynew, xy_deviation / pdc)
    theta_estimate = np.random.normal(thetanew, theta_deviation / pdc)
    s_estimate = np.random.normal(newspeed, speed_deviation / pdc)
    ps_estimate = np.array((x_estimate, y_estimate, theta_estimate, s_estimate))


    return ps_estimate