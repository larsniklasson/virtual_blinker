import math
import utils
import numpy as np
import sys
sys.path.append("..")
from config import *

pdc = RISK_CONFIG["prediction_dev_coeff"]

def PS_estimate((x,y,theta,speed), Is, Ic, travelling_direction, intersection, interval, deviations, no_sample = False):
    c = intersection.courses[travelling_direction, Ic]
    new_PS, dev = c.predictNextState(x, y, theta, speed, interval, Is, deviations, pdc)

    if no_sample:
        return new_PS, dev
    
    
    return sample(new_PS, dev)


def sample((xnew, ynew, thetanew, newspeed), deviations):
    x_deviation, y_deviation, theta_deviation, speed_deviation = deviations

    x_estimate = np.random.normal(xnew, x_deviation)
    y_estimate = np.random.normal(ynew, y_deviation)
    theta_estimate = np.random.normal(thetanew, theta_deviation)
    s_estimate = np.random.normal(newspeed, speed_deviation)
    ps_estimate = np.array((x_estimate, y_estimate, theta_estimate, s_estimate))


    return ps_estimate