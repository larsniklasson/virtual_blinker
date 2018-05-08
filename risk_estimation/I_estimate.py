
#Es_estimate should be a sample from
#probablity we get in Es_estimate.py
#in that function, it gives a probablity that
#Expectation to stop or go at the intersection
#in here, we require a sample from the probablity
#ie either stop(0) or go(1) in Es_estimate
#
#Is_tminus1 is the previous value sampled from
#the previous call to this function at time t-1
#this should also be either stop(0) or go(1)
#s is whether we are estimating if Is is stop (0)
#or go(1)
# returns a discrete probality denstiy for (s = 0, s=1)
def Is_estimate(Is_tminus1, Es_estimate):
    #todo : assert that the parameters are either
    # {0 , 1}
    P_comply = 0.9
    estimate = 0
    
    if (Is_tminus1 == 1 and Es_estimate == 1):
        estimate = P_comply
    elif (Is_tminus1 == 1 and Es_estimate == 0):
        estimate = 0.5
    elif (Is_tminus1 == 0 and Es_estimate == 1):
        estimate = 0.5
    elif (Is_tminus1 == 0 and Es_estimate == 0):
        estimate = 1.0 - P_comply
    
    return (1-estimate, estimate)

    #modfied Is according to:
    #http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.593.6997&rep=rep1&type=pdf
    #this is another source 
    #if (Is_tminus1 == 0):
    #    return (P_comply, 1-P_comply)
    #else:
    #    return (1-P_comply, P_comply)




#estimate intended course for t given
#intended course for t-1 and number of courses in 
#the intersection
def Ic_estimate(Ic_tminus1,N_c):
    # N_c will mostly be 4 in our cases, becasue we have only four courses
    #outputs discrete probablity density of courses.  {0 , 1 , 2 , 3}
    P_same = 0.9

    #in here, we make a list with N_c elements each having value of 1-Psame/nc-1
    N_c = 3
    density = [ (1 - P_same) / (N_c - 1) ] * 4

    #uturn forbidden for now
    density[0] = 0.

    density[Ic_tminus1] = P_same

    return density

    #if (c == Ic_tminus1):
    #    return P_same
    #else:
    #    return ((1 - P_same) /( N_c - 1))
    #pass