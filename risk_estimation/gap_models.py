import sys
sys.path.append("..")
from config import *

import math
import scipy
gm = RISK_CONFIG["gap_model"]
L = gm["L"]
x0 = gm["x0"]
k = gm["k"]
x0_2 = gm["x0_2"]
k2 = gm["k2"]

#Placeholder general gap model 
def gap_model(gap):
    #https://en.wikipedia.org/wiki/Logistic_function

    if gap >= 0:
        return L / (1 + math.exp(-k * (gap - x0)))
    else:
        return L / (1 + math.exp(-k2 * (-gap - x0_2)))



def accept_left_turn_across_path(g_min):
    #left turn across path cases adapted from: https://cloudfront.escholarship.org/dist/prd/content/qt8455h5gq/qt8455h5gq.pdf

    #as per the source, if the g_min is <= 3 seconds, this is not sufficient at all. and g_min >= 12 seconds is accept at all times
    if (g_min <= 3): #gap is not enough at all, 
        return (1. ,0.)
    elif (g_min >= 12): #time gap is enough, so you should go
        return (0., 1.)
    else:
        #i have used the logistic funciton mentioned in the paper as a probablity function. 
        accept50 = 6.1 # figure from the source, Table 3.
        slope = 0.34   # also from source, Table 3
        #equation below does not confirm to the graph stated in cited paper, although it is taken
        #from the paper. i adjusted it a bit by adding a constant so
        #y = 100 / (1 + 10**((math.log10(accept50)-g_min))* slope)

        #y = 100 / (1 + 10**((math.log10(accept50)-g_min+5.315))* slope)
        #remove the 100/ :

        y = 1/(1 + 10**(((math.log10(accept50)-g_min+5.315))* slope))

        #y is probablity it should accept (go)
        #todo need to check this if stop = 0 and stop = 1
        return(1-y,y)

def merge_case_gap_acceptance(T_gap,V):
    #this was used in french paper.
    #model used: https://ac.els-cdn.com/S1369847805000859/1-s2.0-S1369847805000859-main.pdf?_tid=6eb52fc0-5fd3-47d2-9351-9e527e235418&acdnat=1524493356_5d40d9769f70853ac3950f238bff705b
    #used the equation 4 in model (page 5)
    
    # the logistic equation derived from the hypotheses about perception of time and speed
    # is equation 4. Take a look at Beta * Ln (T_cr) that was there.
    # during simulation and experimented data, T_cr was not obtained.
    # under section 3 , third paragraph explains that.
    # the model that fitted the experimental data does not include the said term
    # in its equation. 
    # i am also using the model used to fit the experimental data and not the 
    # model that was derived using hypothesis. This is because the parameter values that
    # were estimated did not take into account of the term. 
    # constants used in formula:

    # numerical stabilization:
    # https://lingpipe-blog.com/2012/02/16/howprevent-overflow-underflow-logistic-regression/
    # https://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.special.expit.html

    
    
    # simulator values , Appendix B.1, All traffic observations
    #lambda_ = 3.611
    #alpha = 0.602
    #gamma = 19.347

    # obtained values were from Appendix B.2. Traffic observations, All
    lambda_ = 4.915
    alpha = 0.867
    gamma = 0.985

    #V cannot be negative. but partice filter can generate negative values if 
    # mean is centered around zero. 

    coeff = lambda_ * (math.log(T_gap) + (1-alpha)*math.log(V) - math.log(gamma))
    accept =  scipy.special.expit(coeff)

    return (1-accept,accept)
