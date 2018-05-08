import math
import utils
import Intersection
import numpy as np
import config
#import scipy

#import pose_avg

def vehicle_project_foward(P_tminus1,S_tminus1, interval):
    #projects vehicle assuming constant speed model, 
    # for interval later
    (x,y,theta) = P_tminus1
    vel_x = S_tminus1 * math.cos(theta)
    vel_y = S_tminus1 * math.sin(theta)

    #calculate distance travelled during interval:
    dist_x = vel_x * interval
    dist_y = vel_y * interval
    return (x+dist_x, y+dist_y, theta)

def vehicle_project_foward2(P_tminus1,S_tminus1,Ic_tminus1, inverval,ved):

    pass
#estimate pose while performing maneuver.
#if the vehicle has not yet reached the intersection
# intended pose is a straight line through the 
#right lane where it is coming from 
def P_estimate(P_tminus1, S_tminus1,Ic_tminus1, intersection_info, interval,vehicle_entering_direction,pose_covariance):

    #estimate = 0
    #first:
    #find the pose for the vehicle at t (one interval later) by using constant speed model
    p_at_t = vehicle_project_foward(P_tminus1,S_tminus1,interval)

    #check if vehicle is inside the intersection
    #or if it is outside (ie, in the lane.)


    #gives a tuple, if outside: (xy of the middle of the right lane) and which side
    # (north, east, west south of the intersection)
    # this find_vehicle_pos assumes that the vehicle is travelling towards the intersection
    # or at least is inside the intersection (but not already navigated the intersection)
    # if vehicle is inside intersection, it returns True
    location = intersection_info.find_vehicle_pos(P_tminus1)
    mean_pose_at_t = 0
    if location == True: # that is, if it is inside intersection junction
        #project P_at_t into the exemplar path
        #this will be a bit difficult as i think we have to figure out a way from which direction
        #did the car enter the intersection.

        proj_at_t = project_on_exemplarpath(P_tminus1,S_tminus1,Ic_tminus1,intersection_info,interval,vehicle_entering_direction)

        #get the mean of projection and P_at_t

        #compute deviation
    
    else:
        #if vehicle lies outside intersection, we have to foward project
        #along the exemplar path. exemplar path in this case would be along the midpoint received.
        #the theta will be changed depending on side 
        side, midpoint = location #unpack
    
        proj_at_t= ()
    
        #now project P_at_t to the exemplar path in the lane
        #for vehicles coming from east and west,  the projection would have y cordinate of the mid point.abs

        if (config.PF_OPTIONS['P_model']==1):
            if (side == 'west'):
                proj_at_t = (P_tminus1[0], midpoint[1], midpoint[2]) #theta is zero as it is coming fro west and going to east
            elif (side == 'east'):
                proj_at_t = (P_tminus1[0], midpoint[1], midpoint[2]) # x, y, theta = from east towards west
            elif (side == 'north'): #for vehicles coming from north and south, x will be midpoint's x
                proj_at_t = (midpoint[0],P_tminus1[1],midpoint[2])
            else:
                proj_at_t = (midpoint[0],P_tminus1[1],midpoint[2])
            #project on exemplar path
            proj_at_t = vehicle_project_foward(proj_at_t,S_tminus1,interval)
        else:
            proj_at_t = vehicle_project_foward2(P_tminus1,S_tminus1,Ic_tminus1, interval, vehicle_entering_direction)



    
    #compute average of poses of p_at_t and porj_at_t
    avgpose = utils.pose_avg(p_at_t,proj_at_t)
    
    #sample a pose from a normal distribution with mean pose and deviation:

    sample = np.random.multivariate_normal(avgpose,pose_covariance)
    sample[2] = sample[2] % (2*math.pi) #confine generated angle to be within [0 2pi]
    return sample
    #return scipy.stats.multivariate_normal(cov=pose_covariance).rvs()

def project_on_exemplarpath(P_tminus1,S_tminus1,Ic_tminus1,intersection_info,interval,vehicle_entering_direction):
    #exemplar path needs modification for vehicles near end of its turn
    #distance taken during interval:

    if (Ic_tminus1 == Intersection.IntersectionCourses.STRAIGHT_AHEAD.value):
        #proj_at_t = vehicle_project_foward(P_tminus1,S_tminus1,interval)
        if (vehicle_entering_direction == 'south'):
            ideal_x = intersection_info.center[0] + intersection_info.lane_width/2
            ideal_y = P_tminus1[1]
            theta = math.pi/2
        if (vehicle_entering_direction == 'east'):
            ideal_y = intersection_info.center[1] + intersection_info.lane_width/2
            ideal_x = P_tminus1[0]
            theta = math.pi
        if (vehicle_entering_direction == 'north'):
            ideal_x = intersection_info.center[0] - intersection_info.lane_width/2
            ideal_y = P_tminus1[1]
            theta = 3*math.pi/2
        if (vehicle_entering_direction == 'west'):
            ideal_y = intersection_info.center[1] - intersection_info.lane_width/2
            ideal_x = P_tminus1[0]
            theta = 0
        
        return vehicle_project_foward((ideal_x,ideal_y,theta),S_tminus1,interval)
        
    else:
        #for curved paths:
        # we project the pose into the curved path by getting the 
        # angle it makes with intersections bottom left corner (for turn left)
        # or angle it makes with bottom right corner (for right turn)
        base_angle = angle_estimate(intersection_info,P_tminus1,Ic_tminus1,vehicle_entering_direction)
        #print("base angle = " + str(math.degrees(base_angle)))
        #print("base angle is " + str(base_angle))

        # we then estimate distance it would have travelled in the
        # curved path in the interval assuming the speed stays constant
        # from the distance we get the angle formed from the distance
        dist = S_tminus1 * interval

        if (vehicle_entering_direction == 'south'):
            start_point = ((intersection_info.center[0] + intersection_info.lane_width/2), intersection_info.corner_bottom_right[1])
            left_turn_base_corner = intersection_info.corner_bottom_left
            left_turn_angle_correction = math.pi/2

            right_turn_base_corner = intersection_info.corner_bottom_right
            right_turn_angle_correction = 3*math.pi/2

            u_turn_base = (intersection_info.center[0], intersection_info.corner_bottom_left[1])
            u_turn_angle_correction = math.pi/2

        elif(vehicle_entering_direction == 'west'):
            start_point = (intersection_info.corner_top_left[0],(intersection_info.corner_bottom_left[1] + intersection_info.lane_width/2))
            left_turn_base_corner = intersection_info.corner_top_left
            left_turn_angle_correction = 0

            right_turn_base_corner = intersection_info.corner_bottom_left
            right_turn_angle_correction = 0

            u_turn_base = (intersection_info.corner_bottom_left[0], intersection_info.center[1])
            u_turn_angle_correction = 0
            

        elif(vehicle_entering_direction == 'north'):
            start_point = ((intersection_info.corner_bottom_left[0] + intersection_info.lane_width/2), intersection_info.corner_top_left[1])
            left_turn_base_corner = intersection_info.corner_top_right
            left_turn_angle_correction = 3*math.pi/2

            right_turn_base_corner = intersection_info.corner_top_left
            right_turn_angle_correction = math.pi/2

            u_turn_base = (intersection_info.center[0], intersection_info.corner_top_left[1])
            u_turn_angle_correction = 3*math.pi/2

        else:  #east case
            start_point = (intersection_info.corner_top_right[0], (intersection_info.center[1] + intersection_info.lane_width/2))
            left_turn_base_corner = intersection_info.corner_bottom_right
            left_turn_angle_correction = math.pi

            right_turn_base_corner = intersection_info.corner_top_right
            right_turn_angle_correction = math.pi

            u_turn_base = (intersection_info.corner_bottom_right[0], intersection_info.center[1])
            u_turn_angle_correction = math.pi



        if (Ic_tminus1 == Intersection.IntersectionCourses.LEFT_TURN.value):
            radius = intersection_info.lane_width + intersection_info.lane_width/2
            delta_angle = dist / radius
            final_angle = min(base_angle + delta_angle, math.pi/2)
            x , y = utils.rotate_point(left_turn_base_corner,start_point,final_angle)
            return (x,y, final_angle + left_turn_angle_correction)

        elif (Ic_tminus1 == Intersection.IntersectionCourses.RIGHT_TURN.value):
            radius = intersection_info.lane_width/2
            delta_angle = dist / radius
            final_angle = min(base_angle + delta_angle, math.pi/2)
            #inversion of direction applied here
            x, y = utils.rotate_point(right_turn_base_corner, start_point, 2*math.pi - final_angle)
            return (x,y, math.pi*2 - (final_angle + right_turn_angle_correction))

            #tangent = - math.pi/2
            #radius = intersection_info.lane_width / -2 #-2 used to make radius negative which will
            #adjust_x ,adjust_y = intersection_info.corner_bottom_right
        elif (Ic_tminus1 == Intersection.IntersectionCourses.U_TURN.value):
            radius = intersection_info.lane_width /2
            delta_angle = dist / radius
            final_angle = min(base_angle + delta_angle, math.pi) #at max, it can be at most 180deg
#            print("final angle = " + str(math.degrees(final_angle)))
            x,y = utils.rotate_point(u_turn_base,start_point,final_angle)
            return (x,y, final_angle + u_turn_angle_correction)

            #tangent = math.pi/2
            #adjust_x ,adjust_y = intersection_info.center[0], intersection_info.corner_bottom_left[1]
        else:
            radius = 1
            print("uknown course " + str(Ic_tminus1))
        # we add the previous angle to this angle to get the
        # final angle after the interval has passed.

        delta_angle =  dist / radius
        final_angle = base_angle + delta_angle
        #print("final angle is " + str(final_angle))
        x,y = utils.pol2rect(radius, final_angle)
        x = x + adjust_x
        y = y + adjust_y
        return (x,y,final_angle+tangent)
        







def angle_estimate(intersection_info, pose,course,vehicle_entering_direction):
    if (course == Intersection.IntersectionCourses.LEFT_TURN.value):
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
            #print("west case")
            base_corner = intersection_info.corner_top_left
            angle_subtract = 3*math.pi/2

        #on a left turn, coming from south, we estimate the distance travelled within
        # the intersection using angle it took and distance from the bottom left corner
        #vehicle coordinate wrt bottom left corner
        angle = utils.angle(pose,base_corner)
        angle = angle - angle_subtract
        return angle 

    if (course == Intersection.IntersectionCourses.RIGHT_TURN.value):
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
        #r = utils.pyth_distance(base_corner, pose)
        #vehicle coordinate wrt bottom left corner
        angle = utils.angle(pose,base_corner)
        #print("angle is " + str(math.degrees(angle)))
        angle = 2*math.pi - angle # inversion of direction
        angle = angle - angle_subtract
        return angle 
        #dist = angle * r
        ##estimate time 
        #time = - dist / speed
        #return time
    if (course == Intersection.IntersectionCourses.U_TURN.value):
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

        #vehicle coordinate wrt bottom left corner
        angle = utils.angle(pose,base_corner)
        #print("pose = " + str(pose))
        #print("base corner = " + str(base_corner))
        #print("util angle = " + str(math.degrees(angle)))
        angle = angle - angle_subtract

        #special case for uturns coming from west as the angle formed can cross the x axis,
        if (vehicle_entering_direction == "west"):
            if (angle <= math.pi/2):
                angle = angle + math.pi/2
            else:
                angle = angle - 3*math.pi/2
        
        return angle 


