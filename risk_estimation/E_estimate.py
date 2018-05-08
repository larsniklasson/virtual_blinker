import gap_models
#from Intersection import IntersectionCourses
# -*- coding: utf-8 -*-
""" Module to estimate Random variable expect maneuver (stop or go ) in an intersection

.. module:: Risk_estimation
    :synopsis: Contains procedures that estimate variables specified in French paper.
.. moduleauthor:: Ibrahim Fayaz <phayax@gmail.com>

The expected longitudinal motion of a vehicle is derived from previous intended course, pose and speed
of all the vehicles in the scene.
In general terms , this estimation requires these things:
 - vehicles (list)
 - type of intersection:
    whether it is a
    - Give way intersection
    - Stop intersection
 - courses in the intersection:
   these lateral courses are with respect to a common numbering system. and is relative (for now it is relative, but this may change)
   to the vehicle.
 - intersection junction point as xy cordinate
 - conjunction of courses for the N vehicles in the scene, at t-1

 This will be a numpy array with dimensions stated as follows:
 axis 0 = course taken {0, 1, 2, etc} , this axis length is 1 as it contains one number
 for now.
     
 Course numbers mean:
           0. U turn
           1. turning left  
           2. going straight ahead (this is a merge situation with the traffic from the left side and right side)
           3. going right side (this also can be considered a merge situation with the traffic from right and across)

 axis 1 = vehicle number {0,... N -1 }, this axis length is 1 as it contains one number


 All information about intersection (junction point, intersection type etc.) are 
 encapsulated in module Intersection.py

- conjunction of positions of the N vehicles in the scene, at t-1 

 this will be a numpy array with dimensions stated as follows:

 - axis 0 = pose()  these are true pose of the vehicles (phi) and not the measurement Z, length is three, as it contains X,Y cord, and theta 
 - axis 1 = vehicle number {0, ... N-1}, length is 1 as it contains one number

 conjunction of speed of the N vehicles in the scene, at t-1 
 this will be a numpy array with dimensions stated as follows:
 axis 0 = speed()  these are true speed of the vehicles (phi) and not the measurement Z, length is one as it contains one number
 axis 1 = vehicle number {0, ... N-1}, length is 1 as it contains one number

 t is the current timestep from the global clock
 interval is the step 

 in the case where the vehicle slows down whe approaching intersection, it will not 
 stop right at the border. it will stop few feet away from border. time to reach intersection
 should be time to reach this border. 
 
 if vehicle has been slowed down enough, before it reaches this border, it will take a long time to
 reach the border. Instead, we could reason that it has arrived at the intersection and is waiting
 for a time gap so that it can navigate the intersection.


 """
import math
import numpy as np
#from Intersection import IntersectionType, IntersectionCourses, Intersection
import Intersection 
#import Intersection.Intersection
import utils

def Es_estimate ( P_tminus1,S_tminus1,Ic_tminus1 ,myved, intersection_info, courses, poses,speeds ,t,interval,vehicle_entering_directions):
    """ Estimates Expected longitudinal motion (Es) at time t , stated in section 4.3.1 of the paper. Output is a discrete probablity density

    Args:
        s (int): Expected longitudinal motion is stop or go( 1 meaning go, 0 meaning stop)
                 ^ above parameter has been omitted and now returns the density. (s = 0, s = 1) as tuples
        n (int): vehicle in question as index (we are calculating expectation to stop (or go) for vechile n at t)
        intersection_info (object): intersection_info of Intersection type (more details below)
        vehicle_entering_directions (list of string) : when a vehicle is inside an intersection it has to give where it came from. this is that 
                                                       parameter

    Returns:
        float tuple. discrete Probablity distribution as follows:
        (a,b) => a = probablity that the espectation to stop  (ie, s = 0),
                 b = probablity that the espectation to go    (ie, s = 1)
                 a+b should be 1


    """

    #vehicles_going_towards_intersection = filter_poses(poses, n)

            

    
    # first calculate when s = stop (ie = 0)
    #Es_estimate = 0 # return value for the function

    # algorithm part i
    #project forward (or backward) the positon of vehicle n until t_n when it reaches the intersection,
    #using vehicles previous state and constant speed model
    # project_vehicle will be an offset from current time, so we add it to get global time when it will be at that position
    #todo: fix when project_vehicle is false // ans: always it returns a number, so this is fixed
    ######t_n = t + project_vehicle(poses[n],speeds[n], intersection_info,interval,courses[n],vehicle_entering_directions)
    #t_n = t + project_vehicle(P_tminus1,S_tminus1, intersection_info,interval,Ic_tminus1,myved)


    #if the vehicle has already finished navigating the intersection, Expectation is to go,
    #this is not mentioned in the paper. 
    loc = intersection_info.find_vehicle_pos(P_tminus1)
    if(isinstance(loc ,tuple)):
        #print(loc[0])
        if loc[0] != myved: # the vehicle is outside intersection, but not where it came from,
        #    print("finished navigating")
            return (0.,1.)  # this implies that it has finished navigating the intersectin

    #if vehicle is very near to the intersection border ( almost arrived)
    #it does not make much sense to predict when it will arrive as in the case of stopping,
    #the speed will be so low and time to arrive will be quite higher. and in the case of going
    #it will not make much difference as time of arrivaal at the border or just one meter away
    #from border would almost be the same.
    #it would make sense to say that vehicle has arrived at intersection and wait for a time gap
    #if it is very near to the intersection than predicting when it will arrive as it is so
    #near.

    if intersection_info.vehicle_arrived(P_tminus1,myved):
        #since we considered it as arrived, current time will be the time of arrival.
        t_n = t
    else:
        pv =  project_vehicle(P_tminus1,S_tminus1, intersection_info,interval,Ic_tminus1,myved)
        if (pv is None):
            print("this is messed ")
        t_n = t+ pv
    #print("tn = " + str(t_n))
    
    # algorithm part ii
    #find set of vehicles that have the right of way wrt. the maneuver of vehicle n
    #this will be a set or a list of indices:
    #v_row = intersection_info.right_of_way(poses, n,vehicle_entering_directions)
    v_row = intersection_info.right_of_way_by_pose(P_tminus1,myved,poses,vehicle_entering_directions)

    #print("v_row is "+ str(v_row))


    # this happens when the vehicle has priority over all other vehicles (say you are on the priority road in a giveway/stop intersection)
    # in this case, the cars expectation to stop is 0 and expectation to go is 1.0
    # this situation is not stated in the paper, but can be deduced from what is stated in the paper
    # ^ above is wrong.
    # whe you have the right of way, we also need to check for the incoming
    # cars coming from the other direction who also have the right of way.
    # v_row == [] only if there are no other cars that are travelling towards intersection

    if v_row == [] : 
        #Es_estimate = (0.,1.)
        #print("returning")
        return (0.,1.)

    #this is a bit complicated. for stop intersection
    if (intersection_info.type == Intersection.IntersectionType.STOP_4 or intersection_info.type == Intersection.IntersectionType.STOP_3):
        #for a stop intersection, and you do not have the right of way:
        #     probablity that it should stop is 1.0 until it reaches the intersection.
        #     once it reaches, the vehicle continues to wait for a good time gap as per steps 3 and 4.
        #     caveat: vehicle_arrived uses a different condition to check if it has arrived.
        #     project vehicle uses time at which the vehicle crosses the four corners in either direction.
        #     but vehicle arrived uses the buffer field in intersection_info
        #     may need to fix this

        #v_row = intersection_info.right_of_way(poses, n)
        ##if you have the right of way, 
        #if v_row == [] : 
        #    # do not stop, just go
        #    Es_estimate = (0.,1.)
        #    return Es_estimate
        
        #if others have right of way and you are then supposed to stop at the intersection

        if (intersection_info.vehicle_arrived(P_tminus1)):
            #if it is already at the intersection time to reach would be t , is this reasonable?
            t_n = t
            pass
            #continue finding for a time gap
            #project people who have the right of way to determine when they arrive at

        else:
            #expectation to stop is 1, go is 0
            Es_estimate = (1., 0.)
            return Es_estimate

    #store it in an np array so that we can use it later
    #np_vrow = np.array(v_row)
    #np_vrow_t = np.expand_dims(np_vrow,axis = 0)

    #for each vehicle in v_row, project foward get time to reach the intersection
    v_row_time_to_reach = []
    for vehicle_index in v_row:
        t_m = t + project_vehicle(poses[vehicle_index],speeds[vehicle_index], intersection_info,interval,courses[vehicle_index],vehicle_entering_directions[vehicle_index])
        v_row_time_to_reach.append(t_m)
    

    #duplicate line from below:
    #tm_minus_tn = list(map(lambda x: x - t_n, v_row_time_to_reach))
    #print("v_row-timetoreach " + str(v_row_time_to_reach))
    

    
    # algorithm part iii
    # t_n = time for vehicle in question to reach intersection
    # if in any of the vechiles t_m is greater than t_n, it means vechicle in question will reach the intersection
    # sooner than the vehicle in vrow, but vehicles in vrow have the right of way, thus the vehicle has to stop at the
    # intersection. # find the index such that t_m - t_n is minimum
    #todo: filter values for t_m - t_n >= 0
    tm_minus_tn = list(map(lambda x: x - t_n, v_row_time_to_reach))
    #print("tm-tn" + str(tm_minus_tn))


    #find only positive ones:
    blob = zip(v_row,tm_minus_tn)
    positive_indexes = list(filter( lambda x: x[1] >= 0, blob))
    #print("positive indexes" + str(positive_indexes))
    if (positive_indexes == []): #case when those who have right of way arrives earlier than vehicle in question
        """
        Section 4.3.1 part iii considers vehicles that have a positive timegap. 
        negative time gap implies that priority vehicle got earlier than vehicle in question.

        It is reasonable to consider negative timegap cases as well, upto a certain point.
        our expectation for the vehicle in question cannot be just 'go' because
        priority vehicle was there first. the vehicle in question must stop until the priority
        vehicle finished its maneuver because it arrived early. 

        How much time shall we give to priority vehicle to finish its maneuver before 
        saying that vehicle in question can go? Well, this is also a time gap model in the negative
        aspect

        I don't know why exactly the paper does not consider this. But if we don't consider it,
        the risk calculation may say expectation is to 'go' eventhough priority vehicle has not
        finished navigating the intersection. This would result in a false-negative risk
        (risk calculation says it is not risky, but in real world it is.)

        an example simulation that demonstrates this is 'scenario2' folder data

        where both vehicles arrive at intersection at same time. when this happens,
        most likely state for priority vehicle may arrive just a bit earlier than
        vehicle in question (eg 0.3 seconds). this would mark as negative time gap
        and vehicle in question expectation decides it can still go.
        if we were to plot risk vs time graph on this vehicle, it would jump around
        high and low unstably.

        If i were to modify the algorithm to account for this, i would use 
        time gap as square of the difference in vehicle arriving time and use that 
        on the gap acceptance.

        This would mean that when the timegap is -2 second (priority vehicle arrives 2 seconds
        earlier than vehicle in question), that gap is still not enough to decide non priority
        vehicle to go. Thus it would subtly solve the problem
        
        This modification would entail taking minimum as closer to zero than absolute minimum
        in time gap

        """
        """
        update: 17 apr 2018
        I talked to Lars about this. His idea was this:
        generally people expect cars to drive through. since this is the case, we can see
        if vehicle in question gets projected where the priority vehicle will be if it were
        to go straight through. if it was outside intersection, then we say it would be safe.
        This also make sense.

        In the general case with many vehicles, some will have negative time gap and others will
        have positive timegap. We take the following approach in selecting the timegap to decide
        if vehicle expectation is to stop or go:
            1) if there are both positive and negative time gaps, we select the smallest 
            positive time gap. This is make sense as irrespective of what negative timegaps we have
            the vehicle will have to stop at the smallest positive time gap because otherwise
            it would be a trafic violation as he is supposed to stop for vehicles that are approaching
            from priority lane. This is also what the paper does.

            2) if there are negative time gaps only, we could select the highest negative time gap.
            We do not select lowest negative timegap. 
            This is because if it happens to be that lowest negatie time gap was -20 seconds, 
            and we select that, the priority vehicle reaches 20 seconds earlier than vehicle in question
            and the priority vehicle have enough time to do the maneuver. but there can be a vehicle
            with say -3 seconds in the timegap list, which we do not select for consideration. 
            -3 can easily collide with our vehicle if it does not stop. Thus we identified an unsafe
            situation as safe here. 

            I am aware that selecting the highest negative timegap can also lead to bad things.
            in a list of time gaps, the correct dangerous vehicle may be in the middle. But this can
            add complexity to the particle filter.

        at this point, blob contains all negative timegap rows
        we take the vrow car that arrives at the intersection earliest of all negative timegap.
        that is max of existing timegap.

        pv contains duration of time to arrive at intersection for vehicle in question.

        
        """
        """
        update: 18 apr 2018
        Elad suggested another method to handle negative timegap:
        for the priority vehicle, if the intention is to stop, we assume that it will
        stop at the intersection border, speed up and navigate the intersection and eventually
        leave the intersection.. throught this time, expectation is to stop.
        if our vehicle is predicted to arrive during this time, it is to stop
        we have to compare time of arrival of our vehicle agaist finishing time of execution
        of this maneuver by the priority vehicle.

        if the intention is to go, we could model like lars.
        If we do this, we are relying on more variables than specified in paper.
        The paper does not use Is_tminus1 of all other vehicles to compute Es.

        """
        #select the highest negative timegap:

        #negative_indexes = list(filter( lambda x: x[1] < 0, blob))
        k = np.argmax(np.array(list(map(lambda x: x[1],blob))))

        # pv contains duration of time to arrive at intersection for vehicle in question.
        #project vehicle k for pv duration towards intersection
        #see if vehicle k is inside intersection by then.
        #if it is, mark as dangerous.
        #if it is not, mark as not dangerous
        
        #return (0,1)            #there is no vehicle that would prevent you from navigating , so go?
                                # ask elad about this
    else:
        k = np.argmin(np.array(list(map(lambda x: x[1],positive_indexes))))
    #print("k = " + str(k))
    #k = positive_indexes[index][0]
    #print("k = " + str(k))
    #g_min = v_row_time_to_reach[k] - t_n
    oncoming_vehicle_index = blob[k][0]
    g_min = abs(tm_minus_tn[k])

    #print("g_min is " + str(g_min))
    #calculate probablity g_min is not sufficient depending on the kind of intersection:


    #merging cases,  adapted from : https://doi.org/10.1016/j.trf.2005.10.001
    #this paper is a bit complicated and i think i need some more reading and understanding 
    #straight ahead and right turn

    #determine if merging case:
    #merging cases happens when im not on priority, but turning right.



    #determine if left turn across path:
    #left turn across path happens when the minimum positive timegap we had was
    #with an opposite road and that my turn is a left turn.

    # needs  more literature reading
    if (vehicle_entering_directions[oncoming_vehicle_index] == utils.get_opposite_direction(myved)):
        if (Ic_tminus1 == Intersection.IntersectionCourses.LEFT_TURN.value ):
            return gap_models.accept_left_turn_across_path(g_min)
        else:
            #as long as it is coming from oposite direction and not a left turn, is expectation to go?
            #i think so, other maneuvers remaining are straight and right turn, in these two cases,
            #opposing vehicle has to wait, not this vehicle, this vehicle has to go.
            return (0,1)
    else:
        return gap_models.merge_case_gap_acceptance(g_min,speeds[oncoming_vehicle_index])
    

def project_vehicle(pose, speed, intersection_info,interval,course, vehicle_entering_direction): 
    """ Projects the vehicle (with given pose and speed) towards the intersection and gives time of arrival

    Args:
        pose (tuple of x,y,theta):  Pose of vehicle
        speed (int/float): Speed of vehicle
        intersection_info (object): intersection_info of Intersection type
        Interval (float): not used 

    Returns:
        float. Time of arrival of the vehicle to the intersection using a constant speed model
    """

    """
    More notes:
    projection will be backward if the vehicle is inside the intersection. we are estimating time of arrival.
    in this case, this time will be negative as it has already arrived the intersection before this method has been called

    For this to happen, we have to know where the vehicle is coming from, when they entered the intersection
    this is given by vehicle_entering_direction parameter

    """

    #private method definitions follow
    def __estimate_time_u_turn(pose,intersection_info,vehicle_entering_direction):
        # for this, base point is center x,bottom_right_y, if coming from south
        if vehicle_entering_direction == "south":
            base_corner = (intersection_info.center[0],intersection_info.corner_bottom_right[1])
            angle_subtract =  0
        if vehicle_entering_direction == "east":
            base_corner = (intersection_info.corner_bottom_right[0],intersection_info.center[1])
            angle_subtract = math.pi/2
        if vehicle_entering_direction == "north":
            base_corner = (intersection_info.center[0],intersection_info.corner_top_right[1])
            angle_subtract = math.pi
        if vehicle_entering_direction == "west": #todo fix this, when angle crosses x axis
            base_corner = (intersection_info.corner_bottom_left[0],intersection_info.center[1])
            angle_subtract = 0

        r = utils.pyth_distance(base_corner, pose)
        #vehicle coordinate wrt bottom left corner
        angle = utils.angle(pose,base_corner)
        angle = angle - angle_subtract

        #special case for uturns coming from west as the angle formed can cross the x axis,
        if (vehicle_entering_direction == "west"):
            if (angle <= math.pi/2):
                angle = angle + math.pi/2
            else:
                angle = angle - 3*math.pi/2
        #angle = intersection_info.angle_estimate(pose,IntersectionCourses.U_TURN,vehicle_entering_direction)

        dist = angle * r
        #estimate time 
        time = - dist / speed
        return time


    def __estimate_time_right_turn(pose,intersection_info,vehicle_entering_direction):
        if vehicle_entering_direction == "south":
            base_corner = intersection_info.corner_bottom_right
            angle_subtract = math.pi
        if vehicle_entering_direction == "east":
            base_corner = intersection_info.corner_top_right
            angle_subtract = math.pi/2
        if vehicle_entering_direction == "north":
            base_corner = intersection_info.corner_top_left
            angle_subtract = 0
        if vehicle_entering_direction == "west":
            base_corner = intersection_info.corner_bottom_left
            angle_subtract = 3*math.pi/2
        #on a right turn, coming from south, we estimate the distance travelled within
        # the intersection using angle it took and distance from the bottom right corner
        r = utils.pyth_distance(base_corner, pose)
        #vehicle coordinate wrt bottom left corner
        angle = utils.angle(pose,base_corner)
        #print("angle is " + str(math.degrees(angle)))
        angle = 2*math.pi - angle # inversion of direction
        angle = angle - angle_subtract
        dist = angle * r
        #estimate time 
        time = - dist / speed
        return time

    def __estimate_time_straight_ahead(pose,intersection_info,vehicle_entering_direction):
        if vehicle_entering_direction == "south":
            base_corner = (intersection_info.corner_bottom_right[0]-(intersection_info.lane_width/2),intersection_info.corner_bottom_right[1])
        if vehicle_entering_direction == "east":
            base_corner = (intersection_info.corner_top_right[0],intersection_info.corner_top_right[1]-(intersection_info.lane_width/2))
        if vehicle_entering_direction == "north":
            base_corner = (intersection_info.corner_top_left[0]+(intersection_info.lane_width/2),intersection_info.corner_top_left[1])
        if vehicle_entering_direction == "west":
            base_corner = (intersection_info.corner_bottom_left[0],intersection_info.corner_bottom_left[1]+(intersection_info.lane_width/2))
        #vehicle coordinate wrt bottom left corner
        dist = utils.pyth_distance(base_corner,pose)
        #estimate time 
        time = - dist / speed
        return time

    
    def __estimate_time_left_turn(pose,intersection_info,vehicle_entering_direction):

        if vehicle_entering_direction == "south":
            base_corner = intersection_info.corner_bottom_left
            angle_subtract = 0
        elif vehicle_entering_direction == "east":
            base_corner = intersection_info.corner_bottom_right
            angle_subtract = math.pi/2
        elif vehicle_entering_direction == "north":
            base_corner = intersection_info.corner_top_right
            angle_subtract = math.pi
        else: # vehicle_entering_direction == "west":
            base_corner = intersection_info.corner_top_left
            angle_subtract = 3*math.pi/2

        #on a left turn, coming from south, we estimate the distance travelled within
        # the intersection using angle it took and distance from the bottom left corner
        r = utils.pyth_distance(base_corner, pose)
        #vehicle coordinate wrt bottom left corner
        angle = utils.angle(pose,base_corner)
        angle = angle - angle_subtract
        dist = angle * r
        #estimate time 
        time = dist / speed
        return -1*time




    # todo: need to vectorize this (give multiple poses,speed and returns time in np array)
    #the parameter interval is not used for now
    #computes time it takes for vehicle to reach the intersection
    #pose is a tuple (x, y, theta)
    #little caveat : it does foward projection only and assumes vehicle is driving towards the intersection
    # have to fix this later
    #if the vehicles are outside the intersection boundaries:
    if intersection_info.vehicle_lies_outside_intersection_square(pose):
        #Earlier calculation was done a bit wrong, now i am fixing it.
        arrival_time = False
        if (vehicle_entering_direction == 'north'):
            arrival_time  = vehicle_2p_intersect(pose,speed,intersection_info.corner_top_left,intersection_info.corner_top_right)
        elif (vehicle_entering_direction == 'east'):
            arrival_time = vehicle_2p_intersect(pose,speed,intersection_info.corner_top_right,intersection_info.corner_bottom_right)
        elif (vehicle_entering_direction == 'west'):
            arrival_time = vehicle_2p_intersect(pose,speed,intersection_info.corner_top_left,intersection_info.corner_bottom_left)
        elif (vehicle_entering_direction == 'south'):
            arrival_time = vehicle_2p_intersect(pose,speed,intersection_info.corner_bottom_left,intersection_info.corner_bottom_right)

        #open question: could there be a case were one of the generated particles, out of the randomneess
        #lies tangent to where it is heading. eg: a vehicle is coming to the intersection from north, but
        #the theta of the vehicle is either 0 or 180 degrees. (lies parallel to where it is heading)
        #when this happens, arrival_time will be False. I am reasoning that it will take an infinite amount of time
        if arrival_time is False:
            arrival_time = 1e17
        
        return arrival_time
    else:
        #if it is inside, the vehicle would have 
        # arrived a bit before, ie. time will be negative. 
        # we estimate the time of arrival at the intersection by 
        # backprojecting the vehicle along the distance of arc of the course 
        # using constant speed model.
        # so, for that:
        # 1 - find the distace it takes to form the arc given its position.
        # 2 - using the speed given, calculate time it will take to arrive at the intersection
        # sometimes it may not be an arc as course can be straight ahead

        # part 1
        #find the correct arc in order to make distance calculation
        if course == Intersection.IntersectionCourses.U_TURN.value:
            t = __estimate_time_u_turn(pose,intersection_info,vehicle_entering_direction)
            return t
        if course == Intersection.IntersectionCourses.LEFT_TURN.value:
            t = __estimate_time_left_turn(pose,intersection_info,vehicle_entering_direction)
            return t

        if course == Intersection.IntersectionCourses.STRAIGHT_AHEAD.value:
            #for straight ahead, distance travelled is estimated by distance from 
            # y coordinate of the vehicle from the y cordinate of bottom right or left corner of intersection
            # if it is coming from south
            t = __estimate_time_straight_ahead(pose,intersection_info,vehicle_entering_direction)
            return t
        if course == Intersection.IntersectionCourses.RIGHT_TURN.value:
            #on right turns, angle is made from bottom right corner if coming from south
            t = __estimate_time_right_turn(pose,intersection_info,vehicle_entering_direction)
            return t
        
        print("course is " + str(course))

        #do backprojection towards at the start
    


def vehicle_2p_intersect(pose, speed, p1, p2):
    """ Given points p1 and p2, this function will return time it takes for vehicle to cross the line formed from 
        p1 and p2 using constant speed model

    Args:
        pose (tuple of x,y,theta):  Pose of vehicle
        speed (int/float): Speed of vehicle
        p1 (tuple of x,y): First point
        p2 (tuple of x,y): Second point

    Returns:
        float. Time of arrival of the vehicle to the intersection using a constant speed model
    """
    # give vehicle pose, speed, and points  p1 and p2 and it will return (p1 and p2 are tuples of (x,y))
    # intersection point where vehicle crosses the line p1 p2

    
    vel_x = speed * math.cos(pose[2])
    vel_y = speed * math.sin(pose[2])
    newpoint_x = pose[0] + vel_x 
    newpoint_y = pose[1] + vel_y 

    #returns tuple (x,y) for intersecting, False otherwise
    p = line_intersection((pose,(newpoint_x,newpoint_y)), (p1,p2)) 
    if (p == False):
        return False

    #now calculate time to reach there:
    time_to_reach = time_toreach(pose,speed,p)
    return time_to_reach

# gives the time it will take to reach p0. assumes vehicle is travelling towards p0
def time_toreach(pose,speed,p0):
    """ Returns time for vehicle to reach given specified point (assumes it is travelling towards the point)

    Args:
        pose (tuple of x,y,theta):  Pose of vehicle
        speed (int/float): Speed of vehicle
        p0 (tuple of x,y): Point in question

    Returns:
        float. Time of arrival of the vehicle to the intersection using a constant speed model
    """
    distance = math.sqrt( (p0[0] - pose[0])**2 + (p0[1] - pose[1])**2 )
    if speed == 0:
        return False
    time = distance / speed
    return time
    
def line_eqn(p1,p2):
    """ gives a tuple of (m,b), a line equation formed by p1 and p2 (y = mx+b). Internal method

    Args:
        p1 (tuple of x,y): Point
        p2 (tuple of x,y): Point

    Returns:
        (tuple of m,b). Equation parameters slope and intercept
    """
    gradient = (p1[1]-p2[1])/(p1[0]-p2[0])
    intercept = p1[1] - (gradient*p1[0])
    return gradient,intercept

def pose_speed_eqn(pose, speed):
    #gives gradient and intercept for given pose, speed:
    newpoint_x = pose[0] + (speed * math.cos(pose[2]))
    newpoint_y = pose[1] + (speed * math.sin(pose[2]))
    
    m,b = line_eqn((pose[0], pose[1]), (newpoint_x, newpoint_y))


#adapted from : https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines-in-python
def line_intersection(line1, line2):
    """ returns point of intersection given two lines. If they are parallel, False is returned

    Args:
        line1 (tuple of (x,y), (x2,y2): Two points that make the line
        line2 (tuple of (x,y), (x2,y2): Two points that make the line

    Returns:
        (tuple of x,y). Intersection point, or False if they are parallel
    """
    """
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1]) #Typo was here

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]

    div = det(xdiff, ydiff)
    if div == 0:
       return False

    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return x, y
    """
    def line(p1, p2):
        A = (p1[1] - p2[1])
        B = (p2[0] - p1[0])
        C = (p1[0]*p2[1] - p2[0]*p1[1])
        return A, B, -C
    
    def intersection(L1, L2):
        D  = L1[0] * L2[1] - L1[1] * L2[0]
        Dx = L1[2] * L2[1] - L1[1] * L2[2]
        Dy = L1[0] * L2[2] - L1[2] * L2[0]
        if D != 0:
            x = Dx / D
            y = Dy / D
            return x,y
        else:
            return False

    L1 = line(line1[0],line1[1])
    L2 = line(line2[0],line2[1])
    R = intersection(L1, L2)
    if R:
        return (R)
    else:
        return False