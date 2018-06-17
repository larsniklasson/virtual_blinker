from math import *

# A speed-distance graph made up of multiple functions, either rootfunctions (constant acceleration, see below) 
# or horizontal lines (acceleration = 0)
# To get the time from this you integrate 1/f
class SpeedProfile:
    def __init__(self, functionList, distance_at_crossing, catchup_acc, catchup_deacc, distance_at_end_of_crossing):
        self.functionList = functionList #list where each entry is a tuple: (d, f) where d is the point where another function takes over from f, i.e. f:s endpoint
        self.distance_at_crossing = distance_at_crossing
        self.catchup_acc = catchup_acc
        self.catchup_deacc = catchup_deacc
        self.distance_at_end_of_crossing = distance_at_end_of_crossing

    #if already passed some function, don't include
    def getFilteredFunctionList(self, distance):
        for i, (f_limit, _) in enumerate(self.functionList):
            if distance < f_limit:
                return self.functionList[i:]

    def getSpeed(self, distance):
        for f_limit, f in self.functionList:
            if distance < f_limit:
                return f.getValue(distance)
        

    #predict next distance and speed
    def predict_old(self, distance, speed, t):

        function_list = self.getFilteredFunctionList(distance)

        ideal_speed = function_list[0][1].getValue(distance)
        if abs(speed - ideal_speed) < 0.5:
            #just follow speed profile
            new_distance = self.getDistanceFollowProfile(distance, t, ffl=function_list)
            return new_distance, self.getSpeed(new_distance)

        elif speed > ideal_speed:
            #need to slow down
            acc = self.catchup_deacc
        else:
            #need to speed up
            acc = self.catchup_acc

        
        r = getRootFunction(distance,speed,acc)

        #the current speed does match the profile, we need to catch up (or down) to it.
        for f_limit, f in function_list:
            x = f.solveRoot(r) #intersection point  NOTE: Not talking about the 4-way intersection here
            if x < f_limit:
                #intersection point happened to be in the same function segment
                time_to_intersection_point = r.solveInverseIntegral(distance, x)
                if time_to_intersection_point < t:
                    #we caught up to profile and still has some time left
                    new_distance =  self.getDistanceFollowProfile(x, t - time_to_intersection_point)
                    newspeed = self.getSpeed(new_distance)
                    break
                else:
                    #t too small to catch up. This means we only follow r. 
                    new_distance =  r.getUpperLimit(t, distance)
                    newspeed = speed + t * acc
                    break
        
        return new_distance, newspeed

    #predict next distance and speed
    def predict(self, distance, speed, t):

        function_list = self.getFilteredFunctionList(distance)

        ideal_speed = function_list[0][1].getValue(distance)
        if abs(speed - ideal_speed) < 0.5:
            #just follow speed profile
            new_distance = self.getDistanceFollowProfile(distance, t, ffl=function_list)
            return new_distance, self.getSpeed(new_distance)

        elif speed > ideal_speed:
            #need to slow down
            acc = self.catchup_deacc
        else:
            #need to speed up
            acc = self.catchup_acc

        
        r = getRootFunction(distance,speed,acc)

        #the current speed does match the profile, we need to catch up (or down) to it.
        for f_limit, f in function_list:
            x = f.solveRoot(r) #intersection point  NOTE: Not talking about the 4-way intersection here
            if x < f_limit:
                #intersection point happened to be in the same function segment
                time_to_intersection_point = r.solveInverseIntegral(distance, x)
                if time_to_intersection_point < t:
                    #we caught up to profile and still has some time left
                    new_distance =  self.getDistanceFollowProfile(x, t - time_to_intersection_point)
                    newspeed = self.getSpeed(new_distance)
                    break
                else:
                    #t too small to catch up. This means we only follow r. 
                    new_distance =  r.getUpperLimit(t, distance)
                    newspeed = self.getSpeed(new_distance) #newspeed = speed + t * acc
                    break
        
        return new_distance, newspeed


    def getDistanceFollowProfile(self, distance, t, ffl = None):
        #current function and it's limit
        if not ffl:
            ffl = self.getFilteredFunctionList(distance)

        f_limit, f = ffl[0]

        #how long does it take to get to the current function segment's limit?
        time_to_limit = f.solveInverseIntegral(distance, f_limit)
        if time_to_limit > t:
            #we won't reach the limit so follow current segment for t sec and see what distance we get
            return f.getUpperLimit(t, distance)
        else:
            #we still have time left. Call the function recursively
            return self.getDistanceFollowProfile(f_limit, t-time_to_limit)                
    
 
    #like getFilteredFunctionList but only return segments before or during the crossing starts
    def getFsCrossing(self, distance):
        function_list = self.getFilteredFunctionList(distance)
        new_function_list = []
        for f_limit, f in function_list:
            new_function_list.append((f_limit, f))
            if f_limit >= self.distance_at_crossing:
                return new_function_list


    #get predicted time to reach crossing
    def getTimeToCrossing(self, distance, speed):
        function_list = self.getFsCrossing(distance)
        ideal_speed = function_list[0][1].getValue(distance)

        if abs(speed - ideal_speed) < 0.5:
            #alread matching profile
            return self.getTimeToCrossingFollowProfile(distance, function_list)

        elif speed > ideal_speed:
            #slow down 
            acc = self.catchup_deacc
        else:
            #speed up
            acc = self.catchup_acc

        r = getRootFunction(distance,speed,acc)

        for f_limit, f in function_list:
            x = f.solveRoot(r) #intersection_point
            if x > self.distance_at_crossing:
                #intersection point happened after crossing. means we just follow r until the crossing and get time until it reaches
                return r.solveInverseIntegral(distance,self.distance_at_crossing)
            if x < f_limit:
                #intersection happened before function limit, get the time it took and then follow the profile
                return r.solveInverseIntegral(distance, x) + self.getTimeToCrossingFollowProfile(x)
            

    


    def getTimeToCrossingFollowProfile(self, distance, function_list=None):
        if not function_list: function_list = self.getFsCrossing(distance)
        t_sum = 0
        for f_limit, f in function_list:
            if f_limit < self.distance_at_crossing:
                #follow this segment until it's limit
                t_sum += f.solveInverseIntegral(distance, f_limit)
                #update current distance
                distance = f_limit
            else:
                #follow this segment until crossing
                t_sum += f.solveInverseIntegral(distance, self.distance_at_crossing)
        return t_sum




    #like getFilteredFunctionList but only return segments before or during the crossing starts
    def getFsEndOfCrossing(self, distance):
        function_list = self.getFilteredFunctionList(distance)
        new_function_list = []
        for f_limit, f in function_list:
            new_function_list.append((f_limit, f))
            if f_limit >= self.distance_at_end_of_crossing:
                return new_function_list


    #get predicted time to reach crossing
    def getTimeToEndOfCrossing(self, distance, speed):
        function_list = self.getFsEndOfCrossing(distance)
        ideal_speed = function_list[0][1].getValue(distance)

        if abs(speed - ideal_speed) < 0.5:
            #alread matching profile
            return self.getTimeToEndOfCrossingFollowProfile(distance, function_list)

        elif speed > ideal_speed:
            #slow down 
            acc = self.catchup_deacc
        else:
            #speed up
            acc = self.catchup_acc

        r = getRootFunction(distance,speed,acc)

        for f_limit, f in function_list:
            x = f.solveRoot(r) #intersection_point
            if x > self.distance_at_end_of_crossing:
                #intersection point happened after crossing. means we just follow r until the crossing and get time until it reaches
                return r.solveInverseIntegral(distance,self.distance_at_end_of_crossing)
            if x < f_limit:
                #intersection happened before function limit, get the time it took and then follow the profile
                return r.solveInverseIntegral(distance, x) + self.getTimeToEndOfCrossingFollowProfile(x)
            

    


    def getTimeToEndOfCrossingFollowProfile(self, distance, function_list=None):
        if not function_list: function_list = self.getFsEndOfCrossing(distance)
        t_sum = 0
        for f_limit, f in function_list:
            if f_limit < self.distance_at_end_of_crossing:
                #follow this segment until it's limit
                t_sum += f.solveInverseIntegral(distance, f_limit)
                #update current distance
                distance = f_limit
            else:
                #follow this segment until crossing
                t_sum += f.solveInverseIntegral(distance, self.distance_at_end_of_crossing)
        return t_sum


#f(x) = k
class HorizontalLineFunction:
    def __init__(self, k):
        self.k = float(k)
    
    def getValue(self,x):
        return self.k

    #integrate 1/f
    def solveInverseIntegral(self, a, b):
        if self.k == 0:
            return float("inf")
        return (b - a) / self.k

    # given the result (t) of the (inverse) integral and the lower limit, calculate the upper limit
    def getUpperLimit(self, t, a):
        return self.k*t + a

    #get intersection point (x value) of the point where f intersects sqrt(a*x+b)
    def solveRoot(self, f):
        a = f.a
        b = f.b
        return (self.k**2-b)/a

#Constant acceleration in a distance-speed diagram => f(x) = sqrt(a*x + b)
class RootFunction:
    def __init__(self, a, b):
        self.a = float(a)
        self.b = float(b)
    
    def getValue(self, x):
        return sqrt(self.a*x + self.b)
    
    #integrate 1/x
    def solveInverseIntegral(self, a, b):
        #f = lambda x: 2*sqrt(max(self.a*x + self.b, 0))/self.a
        #return f(b) - f(a)
        return 2*sqrt(max(self.a*b + self.b, 0))/self.a - 2*sqrt(max(self.a*a + self.b, 0))/self.a

    def getUpperLimit(self, t, a):
        return (1.0/4) * (4*t*sqrt(self.a*a + self.b) + self.a * t**2 + 4*a)

    def solveRoot(self, f):
        return (f.b - self.b) / (self.a - f.a)
        


#find the corresponding function sqrt(ax + b) that passes through the point (distance, speed) and which has acceleration (dv/dt) acc
def getRootFunction(distance, speed, acc):
    a = 2*acc
    b = speed**2 - 2*acc*distance
    return RootFunction(a,b)

    
#create two profiles for the V-profile, one for intention=stop and one for intention=go
def createVProfiles(fastspeed, slowspeed, crossing_distance, slowdown_acc, speedup_acc, catchup_acc, catchup_deacc, crossing_end_distance):
    #math stuff to work out the parameters of the functions and their limits
    s1 = fastspeed
    s2 = slowspeed

    cd = crossing_distance

    l0 = cd - (-s1**2)/(2.0*slowdown_acc)

    #slowdown root function
    a1 = 2.0*slowdown_acc
    b1 = s1**2.0 - 2*slowdown_acc*l0

    l1 = (s2**2.0-b1)/(a1)

    l2 = cd
    l3 = cd + (s1**2.0-s2**2)/(2.0*speedup_acc)

    #speedup root function
    a2 = 2.0*speedup_acc
    b2 = s2**2.0-2*speedup_acc*cd

    #for intention=go
    flist_go = [(l0 , HorizontalLineFunction(s1)), 
                    (l1 , RootFunction(a1, b1)),
                    (l2 , HorizontalLineFunction(s2)),
                    (l3 , RootFunction(a2, b2)),
                    (float("inf"),  HorizontalLineFunction(s1))]
    
    #for intention=stop
    flist_stop = [(l0, HorizontalLineFunction(s1)),
                  (cd, RootFunction(a1,b1)),
                  (float("inf"), HorizontalLineFunction(0))]
    
    p1 = SpeedProfile(flist_go, cd, catchup_acc, catchup_deacc, crossing_end_distance)
    p2 = SpeedProfile(flist_stop, cd, catchup_acc, catchup_deacc, crossing_end_distance)
    return p1,p2



#create two profiles for the Flat-profile, one for intention=stop and one for intention=go
def createFlatProfiles(fastspeed, crossing_distance, slowdown_acc,catchup_acc, catchup_deacc, crossing_end_distance):
    s1 = fastspeed
    d = crossing_distance
    
    d0 = d - (-s1**2)/(2.0*slowdown_acc)

    a1 = 2.0*slowdown_acc
    b1 = s1**2.0 - 2*slowdown_acc*d0

    flist_go = [(float("inf"), HorizontalLineFunction(s1))]
    flist_stop = [(d0, HorizontalLineFunction(s1)),
                  (d, RootFunction(a1,b1)),
                  (float("inf"), HorizontalLineFunction(0)) ]

    p1 = SpeedProfile(flist_go, d, catchup_acc, catchup_deacc, crossing_end_distance)

    p2 = SpeedProfile(flist_stop, d, catchup_acc, catchup_deacc, crossing_end_distance)
    return p1,p2
    


