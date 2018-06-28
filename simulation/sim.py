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
from risk_estimation.RE2 import *

from config import *
from utils.Intersection import *
#from maneuver_negotiation.maneuver_negotiator import *
#rom maneuver_negotiation.cloud import *

SLOWDOWN = SIM_CONFIG["slowdown"]
RATE = SIM_CONFIG["rate"]
carlength = SIM_CONFIG["carlength"]
lookahead = SIM_CONFIG["lookahead"]
xy_deviation = SIM_CONFIG["xy_deviation"]
theta_deviation = SIM_CONFIG["theta_deviation"]
speed_deviation = SIM_CONFIG["speed_deviation"]

deviations = xy_deviation, theta_deviation, speed_deviation

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

        if self.id == 0:
            rospy.set_param("sync_time", rospy.get_rostime().secs + 4)
        
        while 1:
            if rospy.has_param("sync_time"):
                self.sync_time = rospy.get_param("sync_time")
                break
            else:
                rospy.sleep(0.01)

        
        self.nr_cars = rospy.get_param('nr_cars')
        self.save = rospy.get_param('save') and save_id == self.id

        if self.save:
            open('../risk_estimation/debug.txt', 'w').close()

        cd = CARS[self.id]
        travelling_direction = cd["travelling_direction"]
        turn = cd["turn"]
        startdist = cd["starting_distance"]
        self.use_riskestimation = cd["use_riskestimation"]

        #save all measurements from all cars. time as key. older entries are removed to avoid too large dicts
        self.state_dicts = [{} for _ in range(self.nr_cars)]

        self.course = Course(travelling_direction, turn)
        self.x, self.y, self.theta = self.course.getStartingPose(int(startdist))

        #the other vehicle's state topics
        state_sub_topics = ["car_state" + str(i) for i in range(self.nr_cars) if i != self.id]

        for s in state_sub_topics:
            rospy.Subscriber(s, cm.CarState, self.stateCallback, queue_size=10)

        #for noisy measurements
        self.state_pub = rospy.Publisher('car_state' + str(self.id), cm.CarState, queue_size=10)
        
        #for rviz
        self.true_state_pub = rospy.Publisher('true_car_state' + str(self.id), cm.CarState, queue_size=10)
        self.path_pub = rospy.Publisher('car_path' + str(self.id), cm.Path, queue_size=10)

        
        self.Is = "go"
        if not self.use_riskestimation: self.Is = "go"

        self.speed = self.course.getSpeed(self.x, self.y, self.theta, self.Is)


        path = self.course.getPath()

        self.error_calc = ErrorCalc(path) #calculates how far away we are from ideal path
        self.pid = PID(*SIM_CONFIG["pid"])

        self.t = 0

        #let publishers register
        rospy.sleep(1)
        
        self.path_pub.publish(cm.Path([cm.Position(x,y) for x,y in path], self.id))

        self.fm = True

        #has maneuver negotation been initiated or not
        self.man_init = False
        

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
                    f.write(str((actual_time, ms)) + "\n")
            
            
            if self.fm:

                self.intersection = Intersection()
                #run risk estimator
                td = [self.intersection.getTravellingDirection(x, y, theta) for (x, y, theta, _) in ms]
                self.risk_estimator = RE2(td)
                self.fm = False

                #run maneuver negotiator
                #self.maneuver_negotiator = ManeuverNegotiator(self.id,self.intersection,0,self.risk_estimator)
                #self.maneuver_negotiator.initialize()
            
            else:    

                dev = [0.2, 0.2, 0.04, 0.1]

                ms = [(x+np.random.normal(0,dev[0]/5), \
                y+np.random.normal(0,dev[1]/5), \
                theta+np.random.normal(0,dev[2]/5), \
                speed+np.random.normal(0,dev[3]/5)) for x,y,theta,speed in ms]
                
                self.risk_estimator.update_state(actual_time, ms, [dev]*len(ms))
                

                es_go = self.risk_estimator.expectationDensities[self.id, self.course.turn]
                #print es_go, self.id

                if self.use_riskestimation:
                    if es_go > Es_threshold or self.course.hasReachedPointOfNoReturn(self.x, self.y, self.theta):
                        self.Is = "go"
                    else:
                        self.Is = "stop"
                    
                    risk = max([self.risk_estimator.getRisk2(self.id, i) for i in range(self.nr_cars)])
                    if risk > risk_threshold:
                        self.Is = "stop"
                    
                


    def update(self):
        
        now = rospy.get_time()
        dt = now - self.last_time
        self.last_time = now
        
        self.t += 1
        
        # scale lookahead w.r.t. speed. slower speed => smaller lookahead and vice versa
        p = getLookaheadPoint((self.x, self.y), self.theta, lookahead*(self.speed / (50/3.6))) #PID "tuned" for 50 km/h
        error, d = self.error_calc.calculateError(p)
        #if d == 0 :return #ran out of path

        steering_angle = self.pid.update(error)
        steering_angle = min(steering_angle, radians(40))#25
        steering_angle = max(steering_angle, radians(-40))#25
        
        v = dt * self.speed/SLOWDOWN
        self.x += v * cos(self.theta)
        self.y += v * sin(self.theta)
        self.theta += v * tan(steering_angle) / carlength
        
        # follow speed profile
        targetspeed = self.course.getSpeed(self.x, self.y, self.theta, self.Is)

        try:
            fs = self.risk_estimator.checkVehicleInFront(self.id)
            
            if fs != -1:
                targetspeed = min(targetspeed, fs)
            
        except:
            pass
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
        xs = self.x#np.random.normal(self.x, xy_deviation)
        ys = self.y#np.random.normal(self.y, xy_deviation)
        ts = self.theta#np.random.normal(self.theta, theta_deviation)
        ss = self.speed#np.random.normal(self.speed, speed_deviation)

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

        target = self.sync_time
        
        #if s % 2 == 0:
        #    target = s + 2
        #else:
        #    target = s + 1

        ns = ns * 10**(-9)
        diff = target - (s + ns)
        print "diff", diff
        rospy.sleep(diff)
        self.pid.clear()
        self.last_time = rospy.get_time()

        rate = rospy.Rate(RATE)
        first = True
        while not rospy.is_shutdown():
            rate.sleep()
            self.update()
            if self.id == 0 and first:
                rospy.delete_param("sync_time")
                first = False
        


if __name__ == '__main__':
    random.seed(1)
    np.random.seed(1)
    s = Car()
    s.spin()
