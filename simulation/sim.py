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

from config import *

from collections import OrderedDict
from utils.Intersection import *
#from maneuver_negotiation.maneuver_negotiator import *
#rom maneuver_negotiation.cloud import *
from threading import Thread, Lock

SLOWDOWN = SIM_CONFIG["slowdown"]
RATE = SIM_CONFIG["rate"]
carlength = SIM_CONFIG["carlength"]
lookahead = SIM_CONFIG["lookahead"]
xy_deviation = SIM_CONFIG["xy_deviation"]
theta_deviation = SIM_CONFIG["theta_deviation"]
speed_deviation = SIM_CONFIG["speed_deviation"]

deviations = xy_deviation, theta_deviation, speed_deviation

total_nr_particles = SIM_CONFIG["total_nr_particles"]

discard_measurement_time = SIM_CONFIG["discard_measurement_time"]
Es_threshold = SIM_CONFIG["Es_threshold"]
risk_threshold = SIM_CONFIG["risk_threshold"]
save_id = SIM_CONFIG["save_id"]

# The simulator
class Car:

    def __init__(self):
        
        rospy.init_node('car', anonymous=False)

        #get params set in launch file
        name = rospy.get_name()
        self.id = int(name[-1])
        
        nr_cars = rospy.get_param('nr_cars')
        self.save = rospy.get_param('save') and save_id == self.id
        self.plot = rospy.get_param('plot')

        if self.save:
            open('../risk_estimation/debug.txt', 'w').close()

        cd = CARS[self.id]
        travelling_direction = cd["travelling_direction"]
        turn = cd["turn"]
        startdist = cd["starting_distance"]
        use_riskestimation = cd["use_riskestimation"]

        #save all measurements from all cars. time as key. older entries are removed to avoid too large dicts
        self.state_dicts = [{} for _ in range(nr_cars)]

        self.course = Course(travelling_direction, turn)
        self.x, self.y, self.theta = self.course.getStartingPose(int(startdist))

        #the other vehicle's state topics
        if use_riskestimation:
            state_sub_topics = ["car_state" + str(i) for i in range(nr_cars) if i != self.id]

            for s in state_sub_topics:
                rospy.Subscriber(s, cm.CarState, self.stateCallback, queue_size=10)

        #for noisy measurements
        self.state_pub = rospy.Publisher('car_state' + str(self.id), cm.CarState, queue_size=10)
        
        #for rviz
        self.true_state_pub = rospy.Publisher('true_car_state' + str(self.id), cm.CarState, queue_size=10)
        self.path_pub = rospy.Publisher('car_path' + str(self.id), cm.Path, queue_size=10)

        
        self.Is = "stop"
        if not use_riskestimation: self.Is = "go"

        self.speed = self.course.getSpeed(self.x, self.y, self.theta, self.Is)


        path = self.course.getPath()

        self.error_calc = ErrorCalc(path) #calculates how far away we are from ideal path
        self.pid = PID(*SIM_CONFIG["pid"])

        self.t = 0
        
        #sleep to let rviz start up
        rospy.sleep(1.5)
        self.path_pub.publish(cm.Path([cm.Position(x,y) for x,y in path], self.id))

        self.fm = True

        self.last_es = [-1, -1, -1] #TODO maybe not useful anymore since much better particle filter

        #has maneuver negotation been initiated or not
        self.man_init = False



        self.nr_particles_per_particle_filter = total_nr_particles / (nr_cars ** 2)

        
        

    def stateCallback(self, msg):

        if self.t - msg.t > 1/RATE * discard_measurement_time: # old af message, flush queue. 
            return 

        #save measurement
        self.state_dicts[msg.id][msg.t] = (msg.x, msg.y, msg.theta, msg.speed)

        #if we have all measurements for a certain time-stamp perform risk estimation
        if all([(msg.t in d) for d in self.state_dicts]):
            ms = [d[msg.t] for d in self.state_dicts]
            
            actual_time = float(msg.t)/(RATE*SLOWDOWN)
        
            if self.save:
                with open('../risk_estimation/debug.txt', 'a') as f:
                    f.write(str((actual_time, ms, (self.id, self.course.turn, self.Is))) + "\n")
            
            
            if self.fm:

                self.intersection = Intersection()
                #run risk estimator
                self.risk_estimator = RiskEstimator(self.nr_particles_per_particle_filter,
                     ms, actual_time, deviations, self.plot)
                self.fm = False

                self.risk_estimator.setKnownIc(self.id, self.course.turn)
                self.risk_estimator.setKnownIs(self.id, self.Is)


                #run maneuver negotiator
                #self.maneuver_negotiator = ManeuverNegotiator(self.id,self.intersection,0,self.risk_estimator)
                #self.maneuver_negotiator.initialize()
            
            else:
                    
                self.risk_estimator.update_state(actual_time, ms)
            
                es_go = self.risk_estimator.getExpectation(self.id)  
                print "Expectation to go: ", es_go, "id = ", self.id
                #print self.risk_estimator.isManeuverOk(0, "left")
                old_is = self.Is

                #maintain 3 last es_go.
                # if all 3 are greater than threshold then we go
                self.last_es.pop(0)
                self.last_es.append(es_go)
                if self.course.hasReachedPointOfNoReturn(self.x, self.y, self.theta) or all([e > Es_threshold for e in self.last_es]) or self.last_es[-1] > 0.99:
                    self.Is = "go"
                elif all([e <= Es_threshold and e >= 0 for e in self.last_es]) or self.last_es[-1] < 0.01:
                    self.Is = "stop"
                
                """risk = max(self.risk_estimator.get_risk())
                if risk > risk_threshold:
                    self.Is = "stop"
                """


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
        steering_angle = min(steering_angle, radians(40))#25
        steering_angle = max(steering_angle, radians(-40))#25
        
        v = dt * self.speed/SLOWDOWN
        self.x += v * cos(self.theta)
        self.y += v * sin(self.theta)
        self.theta += v * tan(steering_angle) / carlength
        
        # follow speed profile
        targetspeed = self.course.getSpeed(self.x, self.y, self.theta, self.Is)
        if self.id == 0:
            targetspeed += 5
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

        
        #add noise
        xs = np.random.normal(self.x, xy_deviation)
        ys = np.random.normal(self.y, xy_deviation)
        ts = np.random.normal(self.theta, theta_deviation)
        ss = np.random.normal(self.speed, speed_deviation)

        #save current state. Also delete old one
        d = self.state_dicts[self.id]
        d[self.t] = xs, ys, ts, ss
        if self.t - 50 in d: del d[self.t - 50]

        #publish noisy and true state
        self.state_pub.publish(cm.CarState(xs, ys, ts, ss, self.id, self.t))
        self.true_state_pub.publish(cm.CarState(self.x, self.y, self.theta, self.speed, self.id, self.t))
        
        

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
        rospy.sleep(diff)
        self.pid.clear()
        self.last_time = rospy.get_time()

        rate = rospy.Rate(RATE)
    
        while not rospy.is_shutdown():
            rate.sleep()
            self.update()


if __name__ == '__main__':
    random.seed(1)
    np.random.seed(1)
    s = Car()
    s.spin()
