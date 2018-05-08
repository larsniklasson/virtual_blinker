

# a python class to contain intersection information
from enum import Enum
import math
import utils

class IntersectionType(Enum):
    #last number indicates how many roads roads are there, 4 - four road, 3 - T junction
    GIVE_WAY_4 = 0
    STOP_4 = 1
    GIVE_WAY_3 = 2
    STOP_3 = 3

class IntersectionCourses(Enum):
    U_TURN = 0
    LEFT_TURN = 1
    STRAIGHT_AHEAD = 2
    RIGHT_TURN = 3


class Intersection:
    #for now it is assumed that these
    #intersections are aligned to x and y axis
    #center_point as tuples
    #lane width in meters
    #intersection type from enum above
    #marker_alignment = one of 'north-south' or 'east-west'

    def __init__(self,center_point,lane_width,intersection_type,marker_alignment):
        #center points
        self.center = center_point
        #@self.center_y = center_point[1]
        #xy tuples
        self.corner_top_right = ((center_point[0] + lane_width), center_point[1] + lane_width)
        self.corner_top_left =  ((center_point[0] - lane_width), center_point[1] + lane_width)
        self.corner_bottom_right = ( (center_point[0] + lane_width) , (center_point[1] - lane_width))
        self.corner_bottom_left =  ( (center_point[0] - lane_width) , (center_point[1] - lane_width))
        self.lane_width = lane_width

        self.type = intersection_type
        self.marker = marker_alignment #or east-west

        # this buffer is a leeway given to vehicles that arrive on intersection
        # to detect if the vehicle's position has crossed the buffer distance,
        # this is used to detect if a vehicle has arrived at the intersection or not
        self.buffer = 2

    def vehicle_arrived(self, pose,ved):
        #a boolean expression to see if the vehicle has arrived at the intersection.
        #one way to do is to use the self.buffer and compute pyth distance from 
        #intersection center against vehicle position and see if it is lower.
        #if we do that way, vehicles that are inside intersection will also be marked
        #as arrived. This would prevent backward projection of time of arrival. 
        
        #the inital version of this function used that. but this would be a mistake
        #as I have pointed out earlier. 

        #the other way to see vehicle arrived is to use vehicle's ved and buffer
        #to see if the vehicles position is within the intersection border and
        #and a border of 'buffer length' offsetted towards outside from intersection border 
        #if vehicle lies in this narrow range, it will be considered as arrived 
        #at intersection
        #print("ved = " + ved)
        if (ved == 'north'):
            if (pose[1] <= self.corner_top_right[1] + self.buffer) and (pose[1] >= self.corner_top_right[1]):
                return True
        elif (ved == 'east'):
            # print("pose [0] = " + str(pose[0]))
            # print("upper = " + str(self.corner_top_right[0] + self.buffer))
            # print("lower = " + str(self.corner_top_right[0] ))
            if (pose[0] <= self.corner_top_right[0] + self.buffer) and (pose[0] >= self.corner_top_right[0]):
                return True
        elif (ved == 'south'):
            if (pose[1] <= self.corner_bottom_right[1]) and (pose[1] >= self.corner_bottom_right[1] - self.buffer):
                return True
        else: #west case
            if (pose[0] <= self.corner_top_left[0] ) and (pose[0] >= self.corner_top_left[0]-self.buffer):
                return True
        
        return False

        #if (utils.pyth_distance((pose[0],pose[1]), self.center) <= self.buffer):
        #    return True
        #else:
        #    return False

    def is_vehicle_facing_towards_intersection(self,pose):
        angle_threshold = math.radians(60)
        #angle of intersection with respect to the pose
        intersection_angle  = utils.angle(self.center,pose)

        upperbound = pose[2] + angle_threshold
        lowerbound = pose[2] - angle_threshold
        #check if the zero angle is between upper and lower bounds of angle
        if ((upperbound > math.radians(360))):
            upperbound = upperbound - (2*math.pi)
            return not (intersection_angle < lowerbound and intersection_angle > upperbound)
        elif (lowerbound < 0):
            lowerbound = lowerbound + (2*math.pi)
            return not (intersection_angle > upperbound and intersection_angle < lowerbound)
            #above if condition means that the zero angle is between the upper and lower.
            # if thats the case, facing away means actually within the bounds because
            # we inverse the direction which we compare
            #return (intersection_angle < upperbound and intersection_angle > lowerbound)
            #return (intersection_angle > lowerbound and intersection_angle <upperbound)
        else:
            return (intersection_angle < upperbound and intersection_angle > lowerbound)



    def vehicle_lies_outside_intersection_square(self,vehicle):
        v_x,v_y = vehicle[0],vehicle[1]
        if (v_x <= self.corner_top_right[0] and v_x >= self.corner_top_left[0]): #check x
            if (v_y >= self.corner_bottom_left[1] and v_y <= self.corner_top_left[1]): #check y
                return  False
        return True

    def vehicle_lies_eastwest(self,vehicle): #checks if the vehicle given lies in the horizontal road
        v_x,v_y,v_theta = vehicle
        if (v_x < self.corner_top_left[0] or v_x > self.corner_top_right[0]): 
            if (v_y >= self.corner_bottom_right[1] and v_y <= self.corner_top_right[1]): #limit vehicle to the y axis
                return True
        
        return False
    
    def vehicle_lies_northsouth(self,vehicle):
        v_x,v_y,v_theta = vehicle
        if (v_y < self.corner_bottom_left[1] or v_y > self.corner_top_left[1]): 
            if (v_x >= self.corner_bottom_left[0] and v_x <= self.corner_bottom_right[0]): #limit vehicle to the y axis
                return True
        
        return False

    

    def filter_poses(self, poses, n):
        """ Returns a list of indexes that are facing towards the intersection


        Args:
            poses (tuple of x,y,theta):  Poses of vehicle
            n (int of index): index of vehicle in question used in calculation for Es_.

        Returns:
            list. list of poses which the vehicles that are facing away are removed
        """

        # get which side of intersection they are        
        sides = list(map(self.find_vehicle_pos,poses))
        sides = map(lambda x: x[2] if isinstance(x,tuple) else False, sides)

        indexes = range(len(poses))
        glob = zip(indexes, sides)

        # if they are inside intersection, they are neglected as still they can be considered going towards or facing
        # the intersection

        north_indexes = list(filter(lambda x: x[2] == 'north',glob))
        south_indexes = list(filter(lambda x: x[2] == 'south',glob))
        east_indexes =  list(filter(lambda x: x[2] == 'east',glob))
        west_indexes =  list(filter(lambda x: x[2] == 'west',glob))

        #for the vehicles coming from north, theta should be between pi and 0 if it were facing away
        #retain indexes if it is between pi and 2*pi
        south_indexes = list(filter(lambda x: x[1][2]% 2*math.pi >= math.pi and x[1][2]%2*math.pi <= 2*math.pi),south_indexes)

        #retain indexes if it is between pi and 0
        north_indexes = list(filter(lambda x: x[1][2]% 2*math.pi >= 0 and x[1][2]%2*math.pi <= math.pi),north_indexes)

        #retain indexes if it is between pi/2 and 3pi/2
        east_indexes = list(filter(lambda x: x[1][2]% 2*math.pi >= math.pi/2 and x[1][2]%2*math.pi <= 3*math.pi/2),east_indexes)

        #retain indexes if it is between either 0 and pi/2      or     between 3pi/2 and 2pi
        west_indexes = list(filter(lambda x: (x[1][2]% 2*math.pi >= 0 and x[1][2]%2*math.pi <= math.pi/2 ) or \
                                             (x[1][2]% 2*math.pi >= 3*math.pi/2 and x[1][2]%2*math.pi <= 2*math.pi )\
                                    ),west_indexes)

        #now combine everything
        
    def right_of_way_by_pose(self, mypose,myved, vehicle_set,ved):
        #assumes vehicle does not lie in the intersection square(have to fix this later)
        #^ I am fixing the above issue by looking up the ved and deciding who has the right
        #of way, and this seems easier

        v_row = []

        # for people who are on the piority road, right of way vehicles can be 
        # vehicles coming from the opposite direction towards intersection
        if self.marker == 'north-south':
            if (myved == 'east'):
                #wrt_vehicle has right of way, then vrow will be [] 
                #above is wrong, i am fixing github issue #13
                ns_indexes = range(len(ved))
                blob  = zip(ns_indexes,ved)
                #filter vehicles that are coming from opposite of my own direction
                ns_indexes = filter( lambda x : x[1] == 'west',blob)
            elif(myved == 'west'):
                ns_indexes = range(len(ved))
                blob  = zip(ns_indexes,ved)
                #filter vehicles that are coming from opposite of my own direction
                ns_indexes = filter( lambda x : x[1] == 'east',blob)
            else: # for non priority roads, you have to consider vehicles coming from priority road(east and west)
                ns_indexes = range(len(ved))
                blob  = zip(ns_indexes,ved)
                ns_indexes = filter( lambda x : x[1] == 'east' or x[1] == 'west',blob)

            #in here, vehicles that have finished navigating intersection needs
            #to be removed from this list because they are no longer a threat.
            #otherwise, vehicle can continue calculating time gap for a vehicle that
            #has already finished navigating intersection
            #this filter function removes vehicles that are outside intersection AND
            #its vehicle pos not equal to ved (position where it is at does not match
            # where it is entered from. ie. has finished navigating the intersection)
            # 
            ns_indexes = filter( \
                    lambda x: not ( self.vehicle_lies_outside_intersection_square(vehicle_set[x[0]]) and \
                                    self.find_vehicle_pos(vehicle_set[x[0]])[0] != x[1])\
                , ns_indexes)
            return list(map(lambda x: x[0],ns_indexes))
        else: #for when the marker is east-west
            if (myved == 'north'):
                ns_indexes = range(len(ved))
                blob  = zip(ns_indexes,ved)
                #filter vehicles that are coming from opposite of my own direction
                ns_indexes = filter( lambda x : x[1] == 'south',blob)
            elif (myved == 'south'):
                #wrt_vehicle has right of way, then vrow will be [] 
                #above is wrong, fixing issue #13
                ns_indexes = range(len(ved))
                blob  = zip(ns_indexes,ved)
                #filter vehicles that are coming from opposite of my own direction
                ns_indexes = filter( lambda x : x[1] == 'north',blob)
            else: #for non priority road, consider vehicles coming from priority road (ie north and south)
                ns_indexes = range(len(ved))
                blob  = zip(ns_indexes,ved)
                ns_indexes = filter( lambda x : x[1] == 'south' or x[1] == 'north',blob)
            #in here, vehicles that have finished navigating intersection needs
            #to be removed from this list because they are no longer a threat.
            #otherwise, vehicle can continue calculating time gap for a vehicle that
            #has already finished navigating intersection
            #this filter function removes vehicles that are outside intersection AND
            #its vehicle pos not equal to ved (position where it is at does not match
            # where it is entered from. ie. has finished navigating the intersection)
            # 
            ns_indexes = filter( \
                    lambda x: not ( self.vehicle_lies_outside_intersection_square(vehicle_set[x[0]]) and \
                                    self.find_vehicle_pos(vehicle_set[x[0]])[0] != x[1])\
                , ns_indexes)
            return list(map(lambda x: x[0],ns_indexes))

    def right_of_way(self, vehicle_set, wrt_vehicle,ved):

        #assumes vehicle does not lie in the intersection square(have to fix this later)
        #^ I am fixing the above issue by looking up the ved and deciding who has the right
        #of way, and this seems easier
        v_row = []

        if self.marker == 'north-south':
            if (ved[wrt_vehicle] == 'east' or ved[wrt_vehicle] == 'west'):
                #wrt_vehicle has right of way, then vrow will be [] 
                return  []
            else:
                ns_indexes = range(len(ved))
                blob  = zip(ns_indexes,ved)
                ns_indexes = filter( lambda x : x[1] == 'east' or x[1] == 'west',blob)
                return list(map(lambda x: x[0],ns_indexes))
        else:
            if (ved[wrt_vehicle] == 'north' or ved[wrt_vehicle] == 'south'):
                #wrt_vehicle has right of way, then vrow will be [] 
                return  []
            else:
                ns_indexes = range(len(ved))
                blob  = zip(ns_indexes,ved)
                ns_indexes = filter( lambda x : x[1] == 'south' or x[1] == 'north',blob)
                return list(map(lambda x: x[0],ns_indexes))
        
        #this fix will make the following code obsolete
        #depending on the intersection type, returns a set of vehicles which have the right of way
        #with respect to wrt_vehicle
        
        #vehicles coming from east-west have right of way when markers are placed in north and south part of intersection
        v_row = [] #these are list of indexes

        if self.marker == 'north-south':
            if self.vehicle_lies_eastwest(vehicle_set[wrt_vehicle]): #if wrt_vehicle already lies on the priority road, there is no one that has higher priority
                return []

            #check if wrt_vehicle lies in north/south roads
            #if so, vehicles on east-west will have right of way wrt to the wrt_vehicle.
            #find vehicles that are outside intersection square that is in the east-west direction and
            # on its right lane (check for right side lane not implemented yet)
            #v_row = list(filter(lambda x: vehicle_lies_east_west(x),vehicle_set))
            for i,v in enumerate(vehicle_set):
                if self.vehicle_lies_eastwest(v):
                    v_row.append(i)
            
            return v_row


        if self.marker == 'east-west':
            if self.vehicle_lies_northsouth(vehicle_set[wrt_vehicle]): #if wrt_vehicle already lies on the priority road, there is no one that has higher priority
                return []
            #if wrt_vehicle does not lie on north/south roads, no vehicles have the right of way because 
            #wrt_vehicle lies on the priority lane
            for i,v in enumerate(vehicle_set):
                if self.vehicle_lies_northsouth(v):
                    v_row.append(i)
        
            return v_row

        

    #this method finds where the vehicle is with respect to the intersection
    #returns which side and the midpoint of the right lane from which it travells
    #this function is not used anymore
    def find_vehicle_pos(self, pos):
        #this method assumes intersection is aligned to global x y axis
        x,y,theta = pos

        #check if pos is inside the junction:
        if (x < self.corner_top_right[0] and x > self.corner_top_left[0] and \
            y < self.corner_top_right[1] and y > self.corner_bottom_right[1]):
            #inside the junction
            return True
        else:
            #find which side of intersection
            side = 'none'
            if (y >= self.corner_top_right[1]):
                side = 'north'
            elif (x >= self.corner_top_right[0]):
                side = 'east'
            elif (x <= self.corner_top_left[0]):
                side = 'west'
            #if (y <= self.corner_bottom_right[1]):
            else:
                side = 'south'
            
            #depending on side, find the mid point of the right lane:
            midpoint = (0,0)
            if (side == 'north'):
                #find which side of the road (right lane going towards the intersection or the left lane, 
                # going away from the intersection)
                if (x < self.center[0]): 
                    midpoint = ((self.center[0] + self.corner_top_left[0])/2, (self.corner_top_right[1]),3*math.pi/2)
                else:
                    midpoint = ((self.center[0] + self.corner_top_right[0])/2, (self.corner_top_right[1]),math.pi/2)
            elif (side == 'west'):
                if (y < self.center[1]):
                    midpoint = (self.corner_bottom_left[0], (self.corner_bottom_left[1] + self.center[1])/2,0)
                else:
                    midpoint = (self.corner_bottom_left[0], (self.corner_top_left[1] + self.center[1])/2,math.pi)
            elif (side == 'east'):
                if (y > self.center[1]):
                    midpoint = (self.corner_bottom_right[0], (self.center[1] + self.corner_top_right[1])  / 2,math.pi)
                else:
                    midpoint = (self.corner_bottom_right[0], (self.center[1] + self.corner_bottom_right[1])  / 2,0)
            elif (side == 'south'):
                if (x > self.center[0]):
                    midpoint =( ((self.center[0] + self.corner_bottom_right[0]) /2) , self.corner_bottom_right[1],math.pi/2)
                else:
                    midpoint =( ((self.center[0] + self.corner_bottom_left[0]) /2) , self.corner_bottom_right[1],3*math.pi/2)
            
            return (side, midpoint)

    def __str__(self):
        
        return "[intersection: " + self.marker + " ,]" + \
        "top-right : " +    str(self.corner_top_right   ) + \
        "top-left : " +     str(self.corner_top_left    ) + \
        "bottom-left : " +  str(self.corner_bottom_left ) + \
        "bottom-right : " + str(self.corner_bottom_right)


    def angle_estimate(self,pose,course,vehicle_entering_direction):
        if (course == IntersectionCourses.LEFT_TURN):
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
            #vehicle coordinate wrt bottom left corner
            angle = utils.angle(pose,base_corner)
            angle = angle - angle_subtract
            return angle
            #dist = angle * r
            ##estimate time 
            #time = dist / speed
            #return -1*tim
        if (course == IntersectionCourses.RIGHT_TURN):
            if vehicle_entering_direction == "south":
                base_corner = self.corner_bottom_right
                angle_subtract = math.pi
            if vehicle_entering_direction == "east":
                base_corner = self.corner_top_right
                angle_subtract = math.pi/2
            if vehicle_entering_direction == "north":
                base_corner = self.corner_top_left
                angle_subtract = 0
            if vehicle_entering_direction == "west":
                base_corner = self.corner_bottom_left
                angle_subtract = 3*math.pi/2
            #on a right turn, coming from south, we estimate the distance travelled within
            # the intersection using angle it took and distance from the bottom right corner
            r = utils.pyth_distance(base_corner, pose)
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
        if (course == IntersectionCourses.U_TURN):
            if vehicle_entering_direction == "south":
                base_corner = (self.center[0],self.corner_bottom_right[1])
                angle_subtract =  0
            if vehicle_entering_direction == "east":
                base_corner = (self.corner_bottom_right[0],self.center[1])
                angle_subtract = math.pi/2
            if vehicle_entering_direction == "north":
                base_corner = (self.center[0],self.corner_top_right[1])
                angle_subtract = math.pi
            if vehicle_entering_direction == "west": #todo fix this, when angle crosses x axis
                base_corner = (self.corner_bottom_left[0],self.center[1])
                angle_subtract = 0

            #vehicle coordinate wrt bottom left corner
            angle = utils.angle(pose,base_corner)
            angle = angle - angle_subtract

            #special case for uturns coming from west as the angle formed can cross the x axis,
            if (vehicle_entering_direction == "west"):
                if (angle <= math.pi/2):
                    angle = angle + math.pi/2
                else:
                    angle = angle - 3*math.pi/2
            
            return angle
    

