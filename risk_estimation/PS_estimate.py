import math
import utils
import Intersection
import numpy as np

def PS_estimate(x,y,theta,speed, Is, Ic, travelling_direction, intersection, interval, pose_covariance, speed_deviation, flag = False):
    c = intersection.courses[travelling_direction, Ic]
    xnew, ynew, thetanew, newspeed = c.predictNextState(x, y, theta,speed, interval, Is)

    if flag:
        return xnew, ynew, thetanew, newspeed
    
    
    return sample(xnew, ynew, thetanew, newspeed, pose_covariance, speed_deviation)


def sample(xnew, ynew, thetanew, newspeed, pose_covariance, speed_deviation):
    xy_cov = 0.47/5#pose_covariance[0][0]
    theta_cov = 0.20/5#pose_covariance[2][2]

    x_estimate = np.random.normal(xnew, xy_cov)
    y_estimate = np.random.normal(ynew, xy_cov)
    theta_estimate = np.random.normal(thetanew, theta_cov)
    p_estimate = x_estimate, y_estimate, theta_estimate

    #p_estimate = np.random.multivariate_normal((xnew, ynew, thetanew), pose_covariance)
    s_estimate = np.random.normal(newspeed, 0.4/5)#speed_deviation)

    return p_estimate, s_estimate