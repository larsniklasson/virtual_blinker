#!/usr/bin/env python
from math import *
import rospy
import virtual_blinker.msg as cm
from spline import *
from error_calc import *
from pid import *
import random
import sys
from geometry import *

sys.path.append("..")

from utils.course import *
from risk_estimation.driver import *

from collections import OrderedDict


SLOWDOWN = 1


KP = 0.4
KI = 0.00
KD = 0.05

wg = 100

lookahead = 5

# The simulator
class Car:

    def __init__(self):
        
        self.state_dict = {}
        
        rospy.init_node(' ', anonymous=False)


        if rospy.get_name() == '/car1':
            self.id = 1
            
            self.course = Course("north", "left")

            self.x, self.y, self.theta = self.course.getStart(40)
            
            rospy.Subscriber('car_state2', cm.CarState, self.stateCallback, queue_size=1)
            self.state_pub = rospy.Publisher('car_state', cm.CarState, queue_size=10)
            self.path_pub = rospy.Publisher('car_path', cm.Path, queue_size=10)

            
            
        else:
            
            self.id = 2
            
            
            self.course = Course("south", "straight")

            self.x, self.y, self.theta = self.course.getStart(20)
            
            
            rospy.Subscriber('car_state', cm.CarState, self.stateCallback, queue_size=1)
            self.state_pub = rospy.Publisher('car_state2', cm.CarState, queue_size=10)
            self.path_pub = rospy.Publisher('car_path2', cm.Path, queue_size=10)
            

        

        path = self.course.getPath()

        self.intention_stop = False

        self.speed = self.course.getSpeed(self.x, self.y, self.theta, self.intention_stop)
        
        self.rate = rospy.Rate(50)

        self.carlength = 4
        
        self.error_calc = ErrorCalc(path)


        self.pid = PID(KP, KI, KD, wg)
        self.t = 0
        self.risk = 0

        
        rospy.sleep(3)
        self.path_pub.publish(cm.Path([cm.Position(x,y) for x,y in path]))
        

    def stateCallback(self, msg):
        
        while msg.t not in self.state_dict:
            rospy.sleep(0.001)
        
        (x, y, theta, speed) = self.state_dict[msg.t]


        measurements = OrderedDict()

        if self.id == 1:
            measurements[0] = (x,y,theta,speed)
            measurements[1] = (msg.x, msg.y, msg.theta, msg.speed)
        else:
            measurements[0] = (msg.x, msg.y, msg.theta, msg.speed)
            measurements[1] = (x,y,theta,speed)

        #self.risk_estimator.update_state(msg.t/(50.0*SLOWDOWN) , measurements)
        

    def update(self):
        
        
        now = rospy.get_time()
        dt = now - self.last_time
        self.last_time = now
        
        self.t += 1
        

        p = getLookAheadPoint((self.x, self.y), self.theta, lookahead*(self.speed / (50/3.6))) #PID "tuned" for 50 km/h
        error, d = self.error_calc.calculateError(p)
        if d == 0 :return
        
       
        angle = self.pid.update(error)
        
        angle = min(angle, radians(25))
        angle = max(angle, radians(-25))
        
        
        v = dt * self.speed/SLOWDOWN
        self.x += v * cos(self.theta)
        self.y += v * sin(self.theta)
        
        self.theta += v * tan(angle) / self.carlength
        
        
        targetspeed = self.course.getSpeed(self.x, self.y, self.theta, self.intention_stop)
        if targetspeed < self.speed:
            targetacc = self.course.catchup_deacc
            self.speed += dt*targetacc/SLOWDOWN
            self.speed = max(self.speed, targetspeed)
        else:
            targetacc = self.course.catchup_acc
            self.speed += dt*targetacc/SLOWDOWN
            self.speed = min(self.speed, targetspeed)
        
        
        self.state_dict[self.t] = (self.x, self.y, self.speed, self.theta)
        
        

    def spin(self):
        #-----sync-----
        now = rospy.get_rostime()
        s, ns = now.secs, now.nsecs
        ns = ns * 10**(-9)
        target = s + 2   #TODO slight chance that target will be different for the processes (haven't happened yet though)
        
        diff = target - (s + ns)
        print "diff", diff
        rospy.sleep(diff)
        self.pid.clear()
        self.last_time = rospy.get_time()
    
        while not rospy.is_shutdown():
            self.rate.sleep()
            self.update()
            ts = cm.CarState(self.x, self.y, self.theta, self.speed, self.id, self.t)
            self.state_pub.publish(ts)


if __name__ == '__main__':
    s = Car()
    s.spin()
