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
#from maneuver_negotiation.maneuver_negotiator import *
#rom maneuver_negotiation.cloud import *
from threading import Thread, Lock

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

        
        self.Is = "stop"
        if self.id == 0: self.Is = "go"
        self.speed = self.course.getSpeed(self.x, self.y, self.theta, self.Is)


        path = self.course.getPath()
        self.error_calc = ErrorCalc(path) #calculates how far away we are from ideal path
        self.pid = PID(KP, KI, KD, windup_guard)

        self.t = 0
        
        #sleep to let rviz start up
        rospy.sleep(1.5)
        self.path_pub.publish(cm.Path([cm.Position(x,y) for x,y in path], self.id))

        self.fm = True
        self.last_es = [-1, -1, -1]

        self.debug = True
    
        # to avoid modifying particle and weights while another thread such as maneuver negotiator's no_conflict is
        # reading
        self.risk_estimator_mutex = Lock() 
        self.man_init = False
        

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
                
                plot = False and self.id == 1
                closed_loop = True

                save = True
            
                if save and not self.fm:
                    with open('../risk_estimation/debug.txt', 'a') as f:
                        f.write(str((real_time, ms, (self.id, self.course.turn, self.Is))) + "\n")
                
                
                if self.fm:
                    
                    with open('../risk_estimation/debug.txt', 'w') as f:
                        f.write(str((real_time, ms, (self.id, self.course.turn, self.Is))) + "\n")

                    self.intersection = Intersection()
                    #run risk estimator
                    self.risk_estimator = RiskEstimator(400, self.intersection, ms, np.eye(3)*0.15, 0.15, real_time,self.risk_estimator_mutex, plot, wipe_dir=True)
                    self.fm = False
                    self.risk_estimator.setKnownIc(self.id, self.course.turn)
                    self.risk_estimator.setKnownIs(self.id, self.Is)


                    #run maneuver negotiator
                    #self.maneuver_negotiator = ManeuverNegotiator(self.id,self.intersection,0,self.risk_estimator)
                    #self.maneuver_negotiator.initialize()
                
                else:
                        
                    self.risk_estimator.update_state(real_time, ms)
                    
                    if closed_loop: 
                        es_go = self.risk_estimator.getExpectation(self.id)  
                        print "Expectation to go: ", es_go
                        #print self.risk_estimator.isManeuverOk(0, "left")
                        old_is = self.Is

                        #maintain 3 last es_go.
                        # if all 3 are greater than 0.5 then we go
                        self.last_es.pop(0)
                        self.last_es.append(es_go)
                        if self.course.hasReachedPointOfNoReturn(self.x, self.y, self.theta) or all([e > 0.5 for e in self.last_es]) or self.last_es[-1] > 0.99:
                            self.Is = "go"
                        elif all([e <= 0.5 and e >= 0 for e in self.last_es]) or self.last_es[-1] < 0.01:
                            self.Is = "stop"
                        
                        if old_is != self.Is:
                            self.risk_estimator.setKnownIs(self.id, self.Is)

                
        

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
        targetspeed = self.course.getSpeed(self.x, self.y, self.theta, self.Is)
        if targetspeed < self.speed:
            targetacc = self.course.catchup_deacc
            self.speed += dt*targetacc/SLOWDOWN
            self.speed = max(self.speed, targetspeed)
        else:
            targetacc = self.course.catchup_acc
            self.speed += dt*targetacc/SLOWDOWN
            self.speed = min(self.speed, targetspeed)
        
        """
        #start trymaneuever
        if not self.man_init and self.course.hasPassedRequestLine(self.x, self.y):
            print("initiating trymaneuver")
            self.man_init = True
            thread1 = Thread(target=self.maneuver_negotiator.tryManeuver,args=())
            thread1.start()
        """    

        

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
    #random.seed(10003)
    s = Car()
    s.spin()
