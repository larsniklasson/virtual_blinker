import math
import utils
import Intersection
import numpy as np

def PS_estimate(x,y,theta,speed, Is, Ic, travelling_direction, intersection, interval, pose_covariance, speed_deviation):
    c = intersection.courses[travelling_direction, Ic]
    xnew, ynew, thetanew, newspeed = c.predictNextState(x, y, theta,speed, interval, Is)

    #TODO add half half with forward projection ?

    p_estimate = np.random.multivariate_normal((xnew, ynew, thetanew), pose_covariance)
    s_estimate = np.random.normal(newspeed, speed_deviation)

    return p_estimate, s_estimate