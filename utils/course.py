from math import *
from geometry import *
from spline import *

class Course:
    def __init__(self, travelling_direction, turn):
        self.travelling_direction = travelling_direction
        self.turn = turn

        self.requestLine = -70 #y
        self.startPoint = (3.25, -125)

        self.fastspeed = 50/3.6

        self.distance_to_crossing = 125-7.5

        if self.turn == "left":
            
            self.curveStart = (3.25, -7.5)
            self.curveEnd = (-7.5, 3.25)
            self.endPoint = (-125, 3.25)
            self.radius = abs(self.curveStart[0] - self.curveEnd[0])
            self.circle_mid = self.curveStart[0]-self.radius, self.curveEnd[1]-self.radius

            
            self.slowspeed = 20/3.6
            self.slowdown_acc = -5
            self.speedup_acc = 5
            self.catchup_acc = 6
            self.catchup_deacc = -6

            self.sp_go, self.sp_stop = createVProfiles(self.fastspeed, self.slowspeed, self.distance_to_crossing, self.slowdown_acc, self.speedup_acc, self.catchup_acc, self.catchup_deacc)

            curve = [(3.25, -7.6),(3.25, -7.55),(3.25, -7.5),(-7.5, 3.25),(-7.55, 3.25),(-7.6, 3.25)]
            splinecurve = spline(curve)

            self.path = [self.startPoint] + splinecurve + [self.endPoint]
        elif self.turn == "right":


            
            self.curveStart = (3.25, -7.5)
            self.curveEnd = (7.5, -3.25)
            self.endPoint = (125, -3.25)
            self.radius = abs(self.curveStart[0] - self.curveEnd[0])
            self.circle_mid = self.curveStart[0]+self.radius, self.curveEnd[1]-self.radius

            self.slowspeed = 25/3.6
            self.slowdown_acc = -6
            self.speedup_acc = 4
            self.catchup_acc = 5
            self.catchup_deacc = -7

            self.sp_go, self.sp_stop = createVProfiles(self.fastspeed, self.slowspeed, self.distance_to_crossing, self.slowdown_acc, self.speedup_acc, self.catchup_acc, self.catchup_deacc)


            curve = [(3.25, -7.6),(3.25, -7.55),(3.25, -7.5),(7.5, -3.25),(7.55, -3.25),(7.6, -3.25)]
            splinecurve = spline(curve)

            self.path = [self.startPoint] + splinecurve + [self.endPoint]

        elif self.turn == "straight":
            self.endPoint = (3.25, 125)

            self.slowdown_acc = -6
            self.catchup_acc = 8
            self.catchup_deacc = -9

            self.sp_go, self.sp_stop = createFlatProfiles(self.fastspeed, self.distance_to_crossing, self.slowdown_acc, self.catchup_acc, self.catchup_deacc)

            self.path = [self.startPoint, self.endPoint]

        self.path = map(lambda (x,y) : self.rotate(x,y,0,dir=-1)[0:2], self.path)
            


    def getPath(self):
        return self.path

    def rotate(self, x, y, theta, dir=1):
        td = self.travelling_direction
        if td == "west":
            x0 = dir * y
            y0 = dir * x0
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
        
    
    def getSpeed(self, x, y, theta, intention_stop):
        
        x,y,theta = self.rotate(x,y, theta)

        d = self.getDistance(x,y, theta)

        if intention_stop:
            return self.sp_stop.getSpeed(d)
        else:
            return self.sp_go.getSpeed(d)

    def getDistance(self, x, y, theta):

        if self.turn == "straight":
            return abs(self.startPoint[1] - y)
        
        if y < self.curveStart[1]:
            return abs(self.startPoint[1] - y)


        elif (self.turn == "left" and x > self.curveEnd[0]) or (self.turn =="right" and x < self.curveEnd[0]):
            t = theta-pi/2
            if self.turn == "right":
                t = -t
            return abs(self.startPoint[1] - self.curveStart[1]) + self.radius*t
        else:
            return abs(self.startPoint[1] - self.curveStart[1]) + pi*self.radius/2 + abs(x - self.curveEnd[0])

    def hasPassedRequestLine(self, x, y):

        _,y,_ = self.rotate(x,y,0)
        return y > self.requestLine
    
    def getTimeToCrossing(self, x, y, theta, speed, intention_stop):

        x,y,theta = self.rotate(x,y,theta)
        d = self.getDistance(x,y,theta)
        if d >= self.distance_to_crossing:
            return 0
        else:
            if intention_stop:
                return self.sp_stop.getTimeToCrossing(d, speed)
            else:
                return self.sp_go.getTimeToCrossing(d, speed)

    def getStart(self, distance):
        return self.rotate(self.startPoint[0], self.startPoint[1] + distance, pi/2, dir=-1)

    def getNewState(self, x, y, theta, speed, t, intention_stop):

        x,y,theta = self.rotate(x,y,theta)
        d = self.getDistance(x,y,theta)
        if intention_stop:
            newd, newspeed = self.sp_stop.predict(d, speed, t)
        else:
            newd, newspeed = self.sp_go.predict(d, speed, t)



        xnew, ynew,thetanew = self.getNewPos(d, x, newd)

        xnew, ynew, thetanew = self.rotate(xnew, ynew, thetanew, dir = -1)
        return (xnew, ynew, thetanew, newspeed)
    
    
    def getPose(self, d):

        if self.turn == "straight":
            return self.startPoint[0], self.startPoint[1] + d, pi/2

        s1 = abs(self.startPoint[1]-self.curveStart[1])
        s2 = self.radius*2*pi/4
        if d < s1:
            return (self.startPoint[0], self.startPoint[1] + d, pi/2)
        elif d < s1+s2:
            d2 = d - s1
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
            d2 = d - s1 - s2
            if self.turn == "left":
                return self.curveEnd[0] - d2, self.curveEnd[1], pi
            else:
                return self.curveEnd[0] + d2, self.curveEnd[1], 0


    def getNewPos(self, old_d, x, newd):
        p_old = self.getPose(old_d)
        p_new = self.getPose(newd)
        if self.turn == "straight" or p_old[0] > self.curveEnd[0]:
            return (p_new[0] - (p_old[0] - x), p_new[1], p_new[2])
        else:
            return p_new
            



class Flat:
    def __init__(self, k):
        self.k = float(k)
    def f(self,x):
        return self.k

    def solveIntegral(self, d0, d1):
        if self.k == 0:
            return float("inf")
        return (d1-d0)/self.k

    def getIntegralLimit(self, t, d0):
        return self.k*t+d0

    def solveRoot(self, a, b):
        return (self.k**2-b)/a

class Root:
    def __init__(self, a, b):
        self.a = float(a)
        self.b = float(b)
    
    def f(self, x):
        return sqrt(self.a*x + self.b)
    
    def solveIntegral(self, d0, d1):
        f = lambda x: 2*sqrt(self.a*x+self.b)/self.a
        return f(d1)-f(d0)

    def getIntegralLimit(self, t, d0):
        return (1.0/4)*(4*t*sqrt(self.a*d0+self.b) + self.a*t**2 + 4*d0)

    def solveRoot(self, a2, b2):
        return (b2-self.b)/(self.a-a2)
        


class SpeedProfile:
    def __init__(self, flist, d, catchup_acc, catchup_deacc):
        self.flist = flist
        self.d = d
        self.catchup_acc = catchup_acc
        self.catchup_deacc = catchup_deacc

    def getFs(self,d):
        for i in range(len(self.flist)):
            d1, _ = self.flist[i]
            if d < d1:
                return self.flist[i:]

    def getSpeed(self, d):
        for d_, f in self.flist:
            if d < d_:
                return f.f(d)
        

    def predict(self, d, speed, t):

        fs = self.getFs(d)

        if speed > fs[0][1].f(d):
            acc = self.catchup_deacc
        elif speed == fs[0][1].f(d):
            newd = self.getDistanceApply(d,t)
            return newd, self.getSpeed(newd)
        else:
            acc = self.catchup_acc

        
        a,b = self.getAB(d,speed,acc)
        r = Root(a,b)


        for d_,f in fs:
            x = f.solveRoot(a,b)
            if x < d_:
                t0 = r.solveIntegral(d, x)
                if t0 < t:
                    newd =  self.getDistanceApply(x, t - t0)
                    newspeed = self.getSpeed(newd)
                    
                    break
                else:
                    newd =  r.getIntegralLimit(t, d)
                    newspeed = speed * t * acc
                    break
        
        #newspeed = self.getSpeed(newd)
        return newd, newspeed


    def getDistanceApply(self, d, t):
        d_, f = self.getFs(d)[0]
        t0 = f.solveIntegral(d, d_)
        if t0 > t:
            return f.getIntegralLimit(t,d)
        else:
            return self.getDistanceApply(d_, t-t0)                
    
                
        
    def getFsCrossing(self, d):
        fs = self.getFs(d)
        newfs = []
        for d_, f in fs:
            newfs.append((d_, f))
            if d_ >= self.d:
                return newfs



    def getTimeToCrossing(self, d, speed):
        fs = self.getFsCrossing(d)
        if speed > fs[0][1].f(d):
            acc = self.catchup_deacc
        else:
            acc = self.catchup_acc
        a,b = self.getAB(d,speed,acc)

        r = Root(a,b)

        for d_, f in fs:
            x = f.solveRoot(a,b)
            if x > self.d:
                return r.solveIntegral(d,self.d)
            if x < d_:
                return r.solveIntegral(d, x) + self.solveIntegralTilCrossingApply(x)
            


    def solveIntegralTilCrossingApply(self, x):
        fs = self.getFsCrossing(x)
        r = 0
        for d_, f in fs:
            if d_ < self.d:
                r += f.solveIntegral(x, d_)
                x = d_
            else:
                r += f.solveIntegral(x, self.d)
        return r

    def getAB(self, distance, speed, acc):
        a = 2*acc
        b = speed**2-2*acc*distance
        return a,b

    

def createVProfiles(fastspeed, slowspeed, crossing_distance, slowdown_acc, speedup_acc, catchup_acc, catchup_deacc):
    s1 = fastspeed
    s2 = slowspeed

    d = crossing_distance

    d0 = d - (-s1**2)/(2.0*slowdown_acc)

    a1 = 2.0*slowdown_acc
    b1 = s1**2.0 - 2*slowdown_acc*d0

    d1 = (s2**2.0-b1)/(a1)

    d2 = d
    d3 = d + (s1**2.0-s2**2)/(2.0*speedup_acc)

    a2 = 2.0*speedup_acc
    b2 = s2**2.0-2*speedup_acc*d



    flist_go = [(d0 , Flat(s1)), 
                    (d1 , Root(a1, b1)),
                    (d2 , Flat(s2)),
                    (d3 , Root(a2, b2)),
                    (float("inf"),  Flat(s1))]
    
    flist_stop = [(d0, Flat(s1)),
                  (d, Root(a1,b1)),
                  (float("inf"), Flat(0))]
    
    p1 = SpeedProfile(flist_go, d, catchup_acc, catchup_deacc)
    p2 = SpeedProfile(flist_stop, d, catchup_acc, catchup_deacc)
    return p1,p2



def createFlatProfiles(fastspeed, crossing_distance, slowdown_acc,catchup_acc, catchup_deacc):
    s1 = fastspeed
    d = crossing_distance
    
    d0 = d - (-s1**2)/(2.0*slowdown_acc)

    a1 = 2.0*slowdown_acc
    b1 = s1**2.0 - 2*slowdown_acc*d0

    flist_go = [(float("inf"), Flat(s1))]
    flist_stop = [(d0, Flat(s1)),
                  (d, Root(a1,b1)),
                  (float("inf"), Flat(0)) ]

    p1 = SpeedProfile(flist_go, d, catchup_acc, catchup_deacc)

    p2 = SpeedProfile(flist_stop, d, catchup_acc, catchup_deacc)
    return p1,p2
    



