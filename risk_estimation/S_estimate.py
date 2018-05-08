import numpy as np
import math
import utils


def S_estimate(intersection_info,S_tminus1, P_tminus1, Ic_at_t, Is_at_t, interval,speed_deviation):
    #speed estimation needs the interval, ie the duration between t-1 and t.
    #this is because we need to get an estimate of speed after one interval later.
    #that depends on current speed an if the intention is to stop, one interval later,
    #speed will probably be lower. but if the duration of interval is larger, then
    #speed will be even lower.  so we need exact interval duration

    #explain in half time report the simple model we have used here.
    #speed profiles are used only when vehicle intends to stop at intersection
    #and that vehicle is going towards intersection


    #the speed prediction after one interval presented in the paper does not explain very much detail.
    #the paper points to a literature on typical speeds at people approach stop intersections.
    #I found another paper [1] by the same author that describes this more clearly.
    #but I am not sure if that is what the original paper implements

    #[1] - Stephanie Lefevre, Christian Laugier, Javier Ibanez-Guzman. Risk Assessment at Road Intersections:
    #      Comparing Intention and Expectation. IEEE Intelligent Vehicles Symposium, Jun 2012, Alcala de
    #      Henares, Spain. pp.165-171, 2012. <hal-00743219>

    """
    In the paper above, they have two speed profiles. average speed at which vehicle approaches
    and maximum speed at which a vehicle is safely able to navigate across the intersection.
    the speed at the next instant after one interval is given by a formula that takes 
    distance from intersection right now and use that to predict speed in the next instant.

    I am implementing a simplified version of what is implemented in the paper
    In the paper, there are two speed profiles. one when the intention is to stop at the intersection,
    and the other is when the intention is to go. 
    In fig. 4 of the paper, these two profiles are depicted in a diagram. 
    x axis is distance to the intersection boundary. 
    y axis is the speed.

    I used a simplified version of the calculation where:
    When the intention is to stop:
        - Use a constant deceleration model to predict speed after one time interval.
        - This deceleration is set up such a way that speed will be zero at the intersection boundary.
        - (because intention is to stop there.)
        - if the distance to intersection is beyond zero (ie. the vehicle has entered the intersection or
          vehicle has left the intersection after navigating), speed predicted will be zero
          because:
            - when it has left the intersection, the concept of 'intention to stop at the intersection'
              has no meaning.
            - we like to think that our expectation to 'stop at intersection after it has left the intersection'
              (which is a bit absurd) would be to 'go' because we expect vehicles to go foward continuing thier
              way after intersection has been navigated.
            - when the above bullet point is true, we would like to match our intention to the above 
              (intention will be go as well). so we shouldnt favour particles that have intention to stop.
            - one way to make that is to have predicted speed a normal distribution centered on zero
              when this happens, particles with intention to stop will not be favoured because
              likelihood will be so low.
            - this way, risk can be zero for a vehicle (wrt to that intersection) after it has
              already been navigated.(expectation and intention now matches) and this is what we want.
    When the intention is to go:
        - use a constant speed model to predict speed after one interval.
        - make a normal distribution centered on the current speed and sample from it.
        - This corresponds to Fig4 second diagram being a flat line
    This simplified approach may have the following side effect:
        - Risk estimator reports that there is considerable risk (~ 0.25 out of 1)
          at a certain distance from the intersection eventhough we are sure that there is no risk
          for that vehicle because there is no other vehicle to intervene or that vehicle is in priority
        - This happens because at that distance, the particle filter thinks that the vehicle's intention
          is to stop (because the approach speed does match the constant deceleration profile more than
          constant speed profile) and those particles will have higher likelihood, thus more weight.
        - This effect is not long lasting as markov chain converge, the constant speed again takes over.
    We can counter this effect by considering a threshold for risk higher than what we observe

    """
    return mu_s(intersection_info, S_tminus1,P_tminus1,Ic_at_t,Is_at_t,interval,speed_deviation)

def speed_predict(distance, speed,interval):
    #use distance and speed to match the deceleration curve or
    #the profile used to stop the vehicle
    #after matching, predict the speed one interval later
    profile = get_speedprofile(speed,distance)
    #this might be bad as it uses a constant speed model, but in actual reality,
    #speed slows down
    #Q.5 = normal driver,
    #Q.97 = mid extreme driver
    #Q.1 = the very extreme driver
    #if intention is to go:
    dist_travelled = speed*interval
    if (profile == 1):
        return profile1_lookup(distance-dist_travelled) #distance from intersection decreases 

def get_speedprofile(speed,distance):
    return 1 #only one profile for now

def profile1_lookup(distance):
    #mock result
    #using Q.5 documented below,
    if (distance <= 0):
        return 0
    y = math.sqrt((-1* distance)/(-5/12.))
    return y



def mu_s(intersection_info, S_tminus1,P_tminus1,Ic_at_t,Is_at_t,interval,speed_deviation):
    #typical speed profiles:
    

    if (Is_at_t == 0): #if intention is to stop:
        # this was a bit difficult for me to write the correct code.
        # when intention at the intersection is to stop, 
        # driver would slow down the vehicle as it is approaching. 
        # this was modelled by a constant deceleration profile.
        # but what happens to a vehicle after it enters the intersection?
        # and the intention is to stop?
        # It took quite a lot of time to figure out what should happen. I first started with
        # the assumption that eventhough intention at the intersection is to stop, and if it is 
        # already inside the intersection, we return a sample from a normal distribtion with mean of previous speed.
        # while this motivates the idea that the speed wont change once the person is navigating ,
        # this would favour particles with intention to stop as well as intention to go.
        # and this would be bad because if the expectation is to go, particles with
        # intentions to go and stop will equally be favoured. this has the side effect that 
        # risk estimation reports that there are risks even after vehicle has navigated the intersection
        if(intersection_info.vehicle_lies_outside_intersection_square(P_tminus1) and intersection_info.is_vehicle_facing_towards_intersection(P_tminus1)):
            dist_from_intersection = distance_stop_position(P_tminus1,intersection_info)
            speed_estimate = speed_predict(dist_from_intersection,S_tminus1,interval)
            return np.random.normal(speed_estimate,speed_deviation)
        else:
            return np.random.normal(0,speed_deviation)


        #if the vehicle is approaching towards intersection:
        #if intention is to stop:
        #then I have created three profiles based on fig5 in 
        #http://ieeexplore.ieee.org/document/4290122/citations?part=1
        #these profiles are decided depending on velocity and how far is the vehicle
        #from the intersection.

        #only use the speed profile if vehicle is going towards intersection
        #print("speedprofile")

    else:
        #if intention is to go, I assume that the speed will not change.
        # todo: verify this assumption that the speed wont change.S_tminus1
        #return a sample fron a normal distribution with mean S_tminus1, and fixed variance:
        return np.random.normal(S_tminus1,speed_deviation)

def distance_stop_position(pose,intersection_info):
    #todo fix this by correctly getting distance.
    return utils.pyth_distance(pose,intersection_info.center) - intersection_info.lane_width
    
"""
I am trying to implement 
'
Driver Braking Behavior during Intersection Approaches and
Implications for Warning Strategies for Driver Assistant Systems
'

available at
http://ieeexplore.ieee.org/document/4290122/

In the document, it describes velocity of cars when approaching
a stop intersection / red light as a function of distance from 
the intersection. Data was obtained by surveying using a laser
scanner.

The raw data are not provided in the paper.

The french paper cites this literature in finding speed profiles.
I will be using Fig5 to implement three speed profiles:

1) normal drivers (indicated by Q.5 graph)
2) sporty drivers (indicated by Q.97 graph)
3) The very extreme sporty drivers. (indicated by Q.1 graph)

Since the data is directly not available, 
I will approximate the three graphs by a constant deceleration
equation.
Thus, in terms of velocity, it will be an inverse quadratic equation.

I will use the derivative of the inverse quadratic equation to 
estimate the speed after one interval has been past.

for vehicles that are further than 60 meters, I would use a constant
speed model as the intersection is too far away to influence the
decision to try to stop or reduce speed, by the driver. But I do
acknowledge this is an assumption.

I will use the paper to find approximate data points along the curve.
and fit to a constant deceleration model

for Q.5,
    I would model the curev as inverse quadratic.
    first, we gee the equation for the curve in the quadratic form:
    the curve has the following points:
    (0,60)
    (12,0)
    (-12,0)
    solving the equation leads :
    Ax^2 + Bx + C = 0
    where:
    A = -5/12
    B = 0
    C = 60

    We swap the variables x, y to get the inverse quadratic form:
    y = +- sqrt((x - C )/A), if we set C = 0,, we would get a similar curve
    to the Q.5:

for Q.97,
    same as above, but following points:
    (0,60)
    (16,0)
    (-16,0)
    then:
    A = -15/64
    B = 0
    C = 60



"""
