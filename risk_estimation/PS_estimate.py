import math
import utils
import Intersection
import numpy as np
import config


def PS_estimate(p, travelling_direction, intersection, interval, pose_covariance, speed_deviation):
    c = intersection.courses[travelling_direction, p.Ic]
    xnew, ynew, thetanew, newspeed = c.predictNextState(p.P[0], p.P[1], p.P[2], p.S, interval, p.Is)

    #TODO add half half with forward projection ?

    p_estimate = np.random.multivariate_normal((xnew, ynew, thetanew), pose_covariance)
    s_estimate = np.random.normal(newspeed, speed_deviation)

    return p_estimate, s_estimate