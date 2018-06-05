from math import *
from spline import *
from speed_profile import *

#Course has a path and a speedmodel, and can use those to get time to crossing and to predict the next state after time t
#3 different speed models/paths are made (start from north) and then rotated to get 3*4=12 in total

#V-profile:
# s1 -------            --------------
#           \         /
#         a  \       / k
#             \     / 
#              \   /
# s2            ---
#                \
# 0 --------------|---
#                  d


#Flat-profile:
# s1 ----------------------------------
#           \
#         b  \
#             \
#              \
# 0  ------------------------------------

class Course:

    #travelling_direction is "south", "north", "east" or "west"
    #turn is "left", "straight" or "right"

    def __init__(self, travelling_direction, turn):
        self.travelling_direction = travelling_direction
        self.turn = turn

        self.request_line = -20 #y
        self.starting_point = (3.25, -125)

        #s1 in diagram
        self.fastspeed = 50/3.6 #50 km/h

        self.distance_to_crossing = 125-7.5

        if self.turn == "left":
            
            #curve is quarter circle
            self.curve_start = (3.25, -7.5)
            self.curve_end = (-7.5, 3.25)
            self.end_point = (-125, 3.25)
            self.radius = abs(self.curve_start[0] - self.curve_end[0])
            self.circle_mid = self.curve_start[0]-self.radius, self.curve_end[1]-self.radius
            
            #s2 in diagram
            self.slowspeed = 20/3.6

            # a in diagram
            self.slowdown_acc = -5
            # k in diagram
            self.speedup_acc = 5

            #acceleration used when given speed doesn't match that profile - then we assume the vehicle will conform to the model
            # It will speed up/slow down until it matches profile again. These accelerations are used to calculate this part
            self.catchup_acc = 6#6
            self.catchup_deacc = -6#-6

            #get 2 speed profiles, one for intention=go and one for intention=stop
            self.sp_go, self.sp_stop = createVProfiles(self.fastspeed, self.slowspeed, self.distance_to_crossing, self.slowdown_acc, self.speedup_acc, self.catchup_acc, self.catchup_deacc)

            #spline these points to get smooth 90 degree curve
            curve = [(3.25, -7.6),(3.25, -7.55),(3.25, -7.5),(-7.5, 3.25),(-7.55, 3.25),(-7.6, 3.25)]
            splinecurve = spline(curve)

            self.path = [self.starting_point] + splinecurve + [self.end_point]

        elif self.turn == "right":
            
            self.curve_start = (3.25, -7.5)
            self.curve_end = (7.5, -3.25)
            self.end_point = (125, -3.25)
            self.radius = abs(self.curve_start[0] - self.curve_end[0])
            self.circle_mid = self.curve_start[0]+self.radius, self.curve_end[1]-self.radius

            self.slowspeed = 20/3.6
            self.slowdown_acc = -6
            self.speedup_acc = 4
            self.catchup_acc = 5
            self.catchup_deacc = -7

            self.sp_go, self.sp_stop = createVProfiles(self.fastspeed, self.slowspeed, self.distance_to_crossing, self.slowdown_acc, self.speedup_acc, self.catchup_acc, self.catchup_deacc)


            curve = [(3.25, -7.6),(3.25, -7.55),(3.25, -7.5),(7.5, -3.25),(7.55, -3.25),(7.6, -3.25)]
            splinecurve = spline(curve)

            self.path = [self.starting_point] + splinecurve + [self.end_point]

        elif self.turn == "straight":
            self.end_point = (3.25, 125)

            #same as b in diagram
            self.slowdown_acc = -6
            self.catchup_acc = 8
            self.catchup_deacc = -9

            self.sp_go, self.sp_stop = createFlatProfiles(self.fastspeed, self.distance_to_crossing, self.slowdown_acc, self.catchup_acc, self.catchup_deacc)

            self.path = [self.starting_point, self.end_point]

        self.path = map(lambda (x,y) : self.rotate(x,y,0,dir=-1)[0:2], self.path)


    def getPath(self):
        return self.path

    #when travelling_direction is not north, we want to rotate x,y around origin, so we can reuse the "north" model
    #we also rotate back afterwards. dir=1 is rotate to north mode. dir=-1 rotate back from north mode
    def rotate(self, x, y, theta, dir=1):
        td = self.travelling_direction
        if td == "west":
            x0 = dir * y
            y0 = dir * -x
            theta0 = theta - dir * pi/2
        elif td == "north":
            x0 = x
            y0 = y
            theta0 = theta
        elif td == "south":
            x0 = -x
            y0 = -y
            theta0 = theta - dir * pi
        elif td == "east":
            x0 = -y * dir
            y0 = x * dir
            theta0 = theta + dir * pi/2
        
        return x0, y0, theta0
        
    
    def getSpeed(self, x, y, theta, Is):
        
        x,y,theta = self.rotate(x,y, theta)

        #get distance travelled and lookup speed
        d = self.getDistance(x,y, theta)
        if Is == "stop":
            return self.sp_stop.getSpeed(d)
        else:
            return self.sp_go.getSpeed(d)

    def getDistance(self, x, y, theta):

        if self.turn == "straight" or y < self.curve_start[1]:
            return y - self.starting_point[1]


        if (self.turn == "left" and x > self.curve_end[0]) or (self.turn =="right" and x < self.curve_end[0]):
            
            cx, cy = self.circle_mid

            vx = x - cx
            vy = y - cy
            magv = sqrt(vx**2 + vy**2)
            ax = cx + vx/magv * self.radius
            ay = cy + vy/magv * self.radius

            t = asin((ay - cy)/self.radius)

            if self.turn == "left":
                theta = t + pi/2
            elif self.turn == "right":
                theta = pi/2 - t
            
            return  self.curve_start[1] - self.starting_point[1] + self.radius*t
        else:
            #vehicle has exited the curve

            return  self.curve_start[1] - self.starting_point[1] + pi*self.radius/2 + abs(x - self.curve_end[0])

    def hasReachedPointOfNoReturn(self, x, y, theta):
        x, y, theta = self.rotate(x, y, theta)
        d = self.getDistance(x,y,theta)
        return d > self.distance_to_crossing+1

    def hasPassedRequestLine(self, x, y):
        _,y,_ = self.rotate(x,y,0)
        return y > self.request_line

    def hasLeftIntersection(self, x, y, theta):
        x,y,theta = self.rotate(x,y,theta)
        d = self.getDistance(x, y, theta)
        if self.turn == "straight":
            return d >= self.distance_to_crossing + 7.5*2-2.5
        else:
            return d >= self.distance_to_crossing + self.radius*pi/2-2.5
    
    def getTimeToCrossing(self, x, y, theta, speed, Is):
        x,y,theta = self.rotate(x,y,theta)

        #get distance and lookup time to crossing
        d = self.getDistance(x,y,theta)
        if d >= self.distance_to_crossing:
            return 0
        else:
            if Is == "stop":
                return self.sp_stop.getTimeToCrossing(d, speed)
            else:
                return self.sp_go.getTimeToCrossing(d, speed)

    
    def predictNextState(self, x, y, theta, speed, t, Is):

        x,y,theta = self.rotate(x,y,theta)

        #lookup current distance and then predict the distance (and speed) after t sec
        d = self.getDistance(x,y,theta)
        if Is == "stop":
            newd, newspeed = self.sp_stop.predict(d, speed, t)
        else:
            newd, newspeed = self.sp_go.predict(d, speed, t)

        #get new position from distance. Also send old distance and x to account 
        # for the fact that vehicle might be a little off the path center
        xnew, ynew,thetanew = self.getPose(newd)

        xnew, ynew, thetanew = self.rotate(xnew, ynew, thetanew, dir = -1)
        return (xnew, ynew, thetanew, newspeed)
    
    def getStartingPose(self, d):
        return self.rotate(*self.getPose(d), dir=-1)


    #get ideal pose after a certain distance
    def getPose(self, d):

        if self.turn == "straight":
            return self.starting_point[0], self.starting_point[1] + d, pi/2

        #length of section 1,2. i.e. that part before the curve and the curve itself.
        section1 = self.curve_start[1] - self.starting_point[1]
        section2 = self.radius*2*pi/4
        if d <= section1:
            return (self.starting_point[0], self.starting_point[1] + d, pi/2)
        elif d < section1+section2:
            #in the curve
            #trigonometry
            d2 = d - section1
            v = d2/self.radius
            a,b = self.circle_mid

            if self.turn == "left":
                theta = pi/2+v
                ang = v
            elif self.turn == "right":
                theta = pi/2-v
                ang = pi-v
            
            x = a + self.radius*cos(ang)
            y = b + self.radius*sin(ang)

            return (x,y,theta)
        else:
            d2 = d - section1 - section2
            if self.turn == "left":
                return self.curve_end[0] - d2, self.curve_end[1], pi
            else:
                return self.curve_end[0] + d2, self.curve_end[1], 0
