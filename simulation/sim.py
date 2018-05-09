#!/usr/bin/env python

import sys
sys.path.append("..")
from math import *
import rospy
import virtual_blinker.msg as cm
from utils.spline import *
from error_calc import *
from pid import *
import random
from geometry import *
from utils.course import *
from risk_estimation.driver import *

from collections import OrderedDict


SLOWDOWN = 1


KP = 0.4
KI = 0.00
KD = 0.05

wg = 100

carlength = 4
lookahead = 5

# The simulator
class Car:

    def __init__(self):
        
        
        
        rospy.init_node('car', anonymous=False)

        name = rospy.get_name()
        self.id = int(name[-1])
        
        nr_cars = rospy.get_param('nr_cars')
        x = rospy.get_param(rospy.get_name())
        td,turn,startdist =  x.split(" ")

        self.state_dicts = [{} for _ in range(nr_cars)]

        
        self.course = Course(td, turn)
        self.x, self.y, self.theta = self.course.getStart(int(startdist))

        subs = ["car_state" + str(i) for i in range(1,nr_cars+1) if i != self.id]

        for s in subs:
            rospy.Subscriber(s, cm.CarState, self.stateCallback, queue_size=1)

        self.state_pub = rospy.Publisher('car_state' + str(self.id), cm.CarState, queue_size=10)
        self.path_pub = rospy.Publisher('car_path' + str(self.id), cm.Path, queue_size=10)

        path = self.course.getPath()
        self.intention_stop = False
        self.speed = self.course.getSpeed(self.x, self.y, self.theta, self.intention_stop)
        self.error_calc = ErrorCalc(path)


        self.pid = PID(KP, KI, KD, wg)
        self.t = 0
        
        rospy.sleep(3)
        self.path_pub.publish(cm.Path([cm.Position(x,y) for x,y in path], self.id))
    

        

    def stateCallback(self, msg):

        self.state_dicts[msg.id-1][msg.t] = (msg.x, msg.y, msg.theta, msg.speed)
        
        
        while not all([(msg.t in d) for d in self.state_dicts]):
            rospy.sleep(0.001)

        ms = [d[msg.t] for d in self.state_dicts]

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
        
        self.theta += v * tan(angle) / carlength
        
        
        targetspeed = self.course.getSpeed(self.x, self.y, self.theta, self.intention_stop)
        if targetspeed < self.speed:
            targetacc = self.course.catchup_deacc
            self.speed += dt*targetacc/SLOWDOWN
            self.speed = max(self.speed, targetspeed)
        else:
            targetacc = self.course.catchup_acc
            self.speed += dt*targetacc/SLOWDOWN
            self.speed = min(self.speed, targetspeed)
        
        
        d = self.state_dicts[self.id-1]

        d[self.t] = (self.x, self.y, self.speed, self.theta)
        
        if self.t - 50 in d: del d[self.t - 50]
        
        

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

        rate = rospy.Rate(50)
    
        while not rospy.is_shutdown():
            rate.sleep()
            self.update()
            ts = cm.CarState(self.x, self.y, self.theta, self.speed, self.id, self.t)
            self.state_pub.publish(ts)


if __name__ == '__main__':
    s = Car()
    s.spin()
