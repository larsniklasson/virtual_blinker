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
from risk_estimation.Intersection import *

SLOWDOWN = 1

KP = 0.4
KI = 0.00
KD = 0.05

RATE = 50.0/SLOWDOWN

windup_guard = 100

carlength = 4
lookahead = 5

# The simulator
class Car:

    def __init__(self):
        
        rospy.init_node('car', anonymous=False)

        #get params set in launch file
        name = rospy.get_name()
        self.id = int(name[-1])
        
        nr_cars = rospy.get_param('nr_cars')
        p = rospy.get_param(rospy.get_name())
        travelling_direction, turn, startdist =  p.split(" ")

        #save all measurements from all cars. time as key. older entries are removed to avoid too large dicts
        self.state_dicts = [{} for _ in range(nr_cars)]

        self.course = Course(travelling_direction, turn)
        self.x, self.y, self.theta = self.course.getStartingPose(int(startdist))

        #the other vehicle's state topics
        state_sub_topics = ["car_state" + str(i) for i in range(nr_cars) if i != self.id]

        for s in state_sub_topics:
            rospy.Subscriber(s, cm.CarState, self.stateCallback, queue_size=10)

        self.state_pub = rospy.Publisher('car_state' + str(self.id), cm.CarState, queue_size=10)
        self.path_pub = rospy.Publisher('car_path' + str(self.id), cm.Path, queue_size=10)


        self.intention_stop = False
        self.speed = self.course.getSpeed(self.x, self.y, self.theta, self.intention_stop)


        path = self.course.getPath()
        self.error_calc = ErrorCalc(path) #calculates how far away we are from ideal path
        self.pid = PID(KP, KI, KD, windup_guard)

        self.t = 0
        
        #sleep to let rviz start up
        rospy.sleep(1.5)
        self.path_pub.publish(cm.Path([cm.Position(x,y) for x,y in path], self.id))

        self.fm = True
    

        

    def stateCallback(self, msg):

        if self.t - msg.t > 5: # old af message, flush queue. 
            return 

        #save measurement
        self.state_dicts[msg.id][msg.t] = (msg.x, msg.y, msg.theta, msg.speed)

        #if we have all measurements for a certain time-stamp perform risk estimation
        if all([(msg.t in d) for d in self.state_dicts]):
            ms = [d[msg.t] for d in self.state_dicts]
            if self.id == 1: #only send one for now
                
                real_time = msg.t/(RATE*SLOWDOWN)
                with open('debug.txt', 'a') as f:
                    f.write(str((real_time, ms)) + "\n")

                rospy.sleep(0.2)


                #if self.fm:
                #    self.risk_estimator = RiskEstimator(400, Intersection(), ms, np.eye(3)*0.1, 1.5, real_time)
                #    self.fm = False
                #else:
                #    self.risk_estimator.update_state(real_time, ms)
                
        

    def update(self):
        
        now = rospy.get_time()
        dt = now - self.last_time
        self.last_time = now
        
        self.t += 1
        
        # scale lookahead w.r.t. speed. slower speed => smaller lookahead and vice versa
        p = getLookaheadPoint((self.x, self.y), self.theta, lookahead*(self.speed / (50/3.6))) #PID "tuned" for 50 km/h
        error, d = self.error_calc.calculateError(p)
        if d == 0 :return #ran out of path

        steering_angle = self.pid.update(error)
        steering_angle = min(steering_angle, radians(25))
        steering_angle = max(steering_angle, radians(-25))
        
        v = dt * self.speed/SLOWDOWN
        self.x += v * cos(self.theta)
        self.y += v * sin(self.theta)
        self.theta += v * tan(steering_angle) / carlength
        
        # follow speed profile
        targetspeed = self.course.getSpeed(self.x, self.y, self.theta, self.intention_stop)
        if targetspeed < self.speed:
            targetacc = self.course.catchup_deacc
            self.speed += dt*targetacc/SLOWDOWN
            self.speed = max(self.speed, targetspeed)
        else:
            targetacc = self.course.catchup_acc
            self.speed += dt*targetacc/SLOWDOWN
            self.speed = min(self.speed, targetspeed)
        
        
        #save current state. Also delete old one
        d = self.state_dicts[self.id]
        d[self.t] = (self.x, self.y, self.theta, self.speed)
        if self.t - 50 in d: del d[self.t - 50]
        
        

    def spin(self):
        #-----sync-----
        #---sleep until next even second, slight chance that this won't sync
        now = rospy.get_rostime()
        s, ns = now.secs, now.nsecs

        if s % 2 == 0:
            target = s + 2
        else:
            target = s + 1

        ns = ns * 10**(-9)
        diff = target - (s + ns)
        print "diff", diff
        rospy.sleep(diff) # 
        self.pid.clear()
        self.last_time = rospy.get_time()

        rate = rospy.Rate(RATE)
    
        while not rospy.is_shutdown():
            rate.sleep()
            self.update()
            ts = cm.CarState(self.x, self.y, self.theta, self.speed, self.id, self.t)
            self.state_pub.publish(ts)


if __name__ == '__main__':
    s = Car()
    s.spin()
