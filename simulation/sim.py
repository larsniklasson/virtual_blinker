#!/usr/bin/env python

import sys
sys.path.append("..")
from math import *
import rospy
import virtual_blinker.msg as msg
from utils.spline import *
from error_calc import *
from pid import *
import random
from geometry import *
from utils.course import *
from risk_estimation.risk_estimator import *
from std_msgs.msg import String
from maneuver_negotiation.maneuver_negotiator import *
from threading import Thread

import datetime

import os


import config
#from maneuver_negotiation.maneuver_negotiator import *
#rom maneuver_negotiation.cloud import *

# The simulator
class CarSim:

    def __init__(self):
        
        rospy.init_node('car', anonymous=False)
        name = rospy.get_name()
        self.id = int(name[-1])

        #sync and wipe map stuff
        if self.id == 0:
            #car with id=0 sets the sync time and also wipes the map
            rospy.set_param("sync_time", time.time() + 0.07)
            self.wipe_publisher = rospy.Publisher("wipe_map", String, queue_size=10)
        
        c = rospy.get_time()
        while 1:
            now = rospy.get_time()
            if now - c > 10.0:
                print "whhhhhat"
                self.end = True
                break
            if rospy.has_param("sync_time"):
                self.sync_time = rospy.get_param("sync_time")
                break
            else:
                rospy.sleep(0.01)

        
        #get ros-params
        self.nr_cars = 2
        variation = rospy.get_param('variation')
        starting_dist = rospy.get_param('starting_dist')
        danger = rospy.get_param('danger')
        deviation = rospy.get_param('deviation')
        self.test_var = rospy.get_param('test_var')
        
        random_seed = rospy.get_param('seed')

        now = datetime.datetime.now()
        
        self.testdir = "../testing/tests/" + str(self.test_var) + "_" + str(variation) + "_" + str(starting_dist)\
         + "_" + str(deviation) + "_" + str(random_seed) + "_" + str(danger)

        if self.id == 0:
            if not os.path.exists(self.testdir): 
                os.mkdir(self.testdir)



        np.random.seed(random_seed)
        random.seed(random_seed)

        #save all measurements from all cars. time as key. 
        # older entries are removed to avoid too large dicts
        self.car_state_dictionaries = [{} for _ in range(self.nr_cars)]
        
        CAR_DICT = config.getCarDict(self.test_var, variation, starting_dist, deviation, danger)

        initial_tds = []
        initial_poses = []
        for i in range(self.nr_cars):
            c = CAR_DICT[i]
            td = c.travelling_direction
            st_d = c.starting_distance
            course = config.intersection.courses[td, "straight"]
            x,y,theta = course.getPose(st_d)
            speed = course.getSpeed(x, y, "stop")
            initial_poses.append((x,y,theta,speed))
            initial_tds.append(td)
        


        this_car = CAR_DICT[self.id]
        travelling_direction = this_car.travelling_direction
        turn = this_car.turn
        starting_distance = this_car.starting_distance

        self.use_eb = this_car.use_eb
        self.use_mn = this_car.use_mn
        self.lose_com = this_car.lose_com
        self.speed_dev = this_car.speed_dev
        self.noisy = this_car.noisy

        self.course = Course(travelling_direction, turn)
        self.x, self.y, self.theta = self.course.getPose(starting_distance)

        self.risk_estimator = RiskEstimator(self.id, self.course.turn, initial_poses, initial_tds)
        
        if self.use_mn: self.maneuver_negotiator = ManueverNegotiator(self.risk_estimator, self.id, self.course.turn, self.nr_cars, self)

        #the other vehicle's state topics to subscribe to
        state_sub_topics = ["car_state" + str(i) for i in range(self.nr_cars)]

        for s in state_sub_topics:
            rospy.Subscriber(s, msg.CarState, self.stateCallback, queue_size=10)


        if self.use_mn: rospy.Subscriber("man_neg" + str(self.id), msg.ManNeg, self.maneuver_negotiator.messageCallback, queue_size=10)
        

        
        self.state_publisher = rospy.Publisher('car_state' + str(self.id), msg.CarState, queue_size=10)
        
        #for rviz
        self.true_pose_publisher = rospy.Publisher('true_pose' + str(self.id), msg.TruePose, queue_size=10)
        self.path_publisher = rospy.Publisher('car_path' + str(self.id), msg.Path, queue_size=10)

        
        self.Is = "stop" if self.use_mn else "go"
        self.emergency_break = False
        self.speed = self.course.getSpeed(self.x, self.y, self.Is)


        path = self.course.getPath()

        #for following the path
        self.error_calc = ErrorCalc(path) #calculates how far away we are from ideal path
        self.pid = PID(*config.pid)

        self.t = 0

        rospy.sleep(0.01) #let publishers register
        if self.id == 0: self.wipe_publisher.publish(String("")) # for cleaning up paths/cars on the map
        #rospy.sleep(0.02) #make sure map is wiped before publishing new paths
        self.path_publisher.publish(msg.Path([msg.Position(x,y) for x,y in path], self.id))

        #has vehicle entered try maneuever state or not
        self.has_initiated_trymaneuever = False

        self.has_triggered_eb = False
        
        self.has_stopped = False

        self.end = False

        self.eb_time = -1

        if self.id == 1:
            rospy.sleep(0.005)
        self.f = open(self.testdir + "/" + str(self.id) + ".txt", "w")

        self.stop_time = -1

        self.onehaspassed = False
        self.zerohaspassed = False
        self.onehaspassed2 = False

        self.course1 = config.intersection.courses[CAR_DICT[1].travelling_direction, CAR_DICT[1].turn]




    def stateCallback(self, msg):

        #print msg.t

        if self.lose_com:

            if self.lose_com == 1:
                d = -10
            
            if self.lose_com == 2:
                d = 15

            if self.lose_com == 3:
                d = 35

            if msg.id == 1 and self.course1.hasPassedRequestLine(msg.x, msg.y, d):
                self.onehaspassed = True

            if msg.id == 1 and self.onehaspassed and self.course1.getDistance(msg.x, msg.y) > self.course1.distance_at_crossing and sqrt(msg.x**2+msg.y**2) > 30:
                self.onehaspassed2 = True

            if msg.id == 0 and msg.y > 30:
                self.zerohaspassed = True

        
        listenToThese = [0,1]

        if self.onehaspassed and not (self.zerohaspassed or self.onehaspassed2):
            if msg.id != self.id:
                return
            else:
                listenToThese = [self.id]




        d = self.car_state_dictionaries[msg.id]
        d[msg.t] = msg

        if msg.t - 50 in d: del d[msg.t - 50]

        have_all = True
        for c in listenToThese:
            if not msg.t in self.car_state_dictionaries[c]:
                have_all = False
                break

        
        #if we have measurements from all cars for a certain time-stamp then perform risk estimation
        if have_all:
            poses = {}
            deviations = {}

            for c in listenToThese:
                entry = self.car_state_dictionaries[c][msg.t]

                p = (entry.x, entry.y, entry.theta, entry.speed)
                dev = (entry.x_deviation, entry.y_deviation,
                              entry.theta_deviation, entry.speed_deviation)
                
                poses[c] = p
                deviations[c] = dev

            actual_time = float(msg.t)/(config.rate * config.slowdown)
        
            self.risk_estimator.update_state(actual_time, poses, deviations)
            
            
            risk = max(self.risk_estimator.getRisk(0,1), self.risk_estimator.getRisk(1,0))
            
            
            if risk > config.risk_threshold and not self.has_triggered_eb:


                if self.use_eb:
                    self.has_triggered_eb = True
                    self.eb_time = msg.t
                    print "Emergency break activated on car ", self.id
                else:
                    if self.test_var == 0:
                        if self.eb_time == -1:
                            self.eb_time = msg.t
                            print "shouldve triggered eb here", msg.t, self.id

            

    def update(self):

        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        self.t += 1
        
        # scale lookahead w.r.t. speed. slower speed => smaller lookahead and vice versa
        lookahead_distance = config.lookahead * min(1, self.speed / (50/3.6)) #PID "tuned" for 50 km/h
        p = getLookaheadPoint((self.x, self.y), self.theta, lookahead_distance) 
        error, _ = self.error_calc.calculateError(p)
        
        steering_angle = self.pid.update(error)
        steering_angle = min(steering_angle, radians(40))
        steering_angle = max(steering_angle, radians(-40))
        
        v = dt * self.speed / config.slowdown
        self.x += v * cos(self.theta)
        self.y += v * sin(self.theta)
        self.theta += v * tan(steering_angle) / config.carlength

        if not self.use_mn:
            self.Is = "go"
        else:
        
            if not self.has_initiated_trymaneuever:
                
                self.Is = "stop"

            if self.maneuver_negotiator.granted:
                
                self.Is = "go"

            if self.risk_estimator.grant_list != []:
                
                self.Is = "stop"

        # follow speed profile
        targetspeed = self.course.getSpeed(self.x, self.y, self.Is)
        #if self.Is == "go":
        
        if self.speed_dev == 1:
            if self.course.getDistance(self.x, self.y) > self.course.distance_at_crossing - 30.0:
                if self.Is == "go":
                    targetspeed += 15/3.6

        if self.speed_dev == 2:
            if self.course.getDistance(self.x, self.y) > self.course.distance_at_crossing - 30.0:
                if self.Is == "go":
                    targetspeed -= 10/3.6
                    targetspeed = max(12/3.6, targetspeed)


        #adapt speed to vehicle in front
        recommended_speed = self.risk_estimator.recommendSpeedIfVehicleInFront(self.id)
        # -1 = no recommendation
        if recommended_speed != -1: 
            targetspeed = min(targetspeed, recommended_speed)
            
        if self.has_triggered_eb:
            if self.speed < 0.05:
                self.has_stopped = True
                self.stop_time = self.t
            
            if not self.has_stopped:
                targetspeed = 0
                

        
        if targetspeed < self.speed:
            targetacceleration = self.course.match_profile_deacceleration
            if self.emergency_break:
                targetacceleration = -15
            self.speed += dt * targetacceleration / config.slowdown
            self.speed = max(self.speed, targetspeed)
        else:
            targetacceleration = self.course.match_profile_acceleration
            self.speed += dt*targetacceleration / config.slowdown
            self.speed = min(self.speed, targetspeed)
        
        #start trymaneuever
        if self.use_mn:
            if not self.has_initiated_trymaneuever and self.course.hasPassedRequestLine(self.x, self.y):
                print "initiating trymaneuver ", self.id
                self.has_initiated_trymaneuever = True
                self.maneuver_negotiator.tryManeuver()
                #thread1 = Thread(target=self.maneuver_negotiator.tryManeuver,args=())
                #thread1.start()



        #----Fake filtering----
        #noisy measurement


        if self.noisy:
            dev = config.deviations_high
        else:
            dev = config.deviations_low

        x_noise = np.random.normal(0, dev[0])
        y_noise = np.random.normal(0, dev[1])
        t_noise = np.random.normal(0, dev[2])
        s_noise = np.random.normal(0, dev[3])

        # "filtering"
        x_filtered = self.x + (x_noise) / 3.0
        y_filtered = self.y + (y_noise) / 3.0
        t_filtered = self.theta + (t_noise) / 3.0
        s_filtered = self.speed + (s_noise) / 3.0

        x_dev_filtered = abs(x_noise/2.0)
        y_dev_filtered = abs(y_noise/2.0)
        t_dev_filtered = abs(t_noise/2.0)
        s_dev_filtered = abs(s_noise/2.0)


        carstate_msg = msg.CarState(
            x_filtered, y_filtered, t_filtered, s_filtered, 
            x_dev_filtered, y_dev_filtered, t_dev_filtered, s_dev_filtered, 
            self.id, self.t)
        
        
        #publish noisy and true state
        self.state_publisher.publish(carstate_msg)
        self.true_pose_publisher.publish(msg.TruePose(self.x, self.y, self.theta, self.speed, self.id))

        self.f.write(str((self.x, self.y, self.theta, self.t)) + "\n")

        

        if self.t > 25*config.rate:
            self.f.write("fail")
            self.end = True
            self.f.close()

        if self.course.hasPassedRequestLine(self.x, self.y) and sqrt(self.x**2 + self.y**2) > 40:
            self.end = True

            self.f.write(str((self.eb_time, self.stop_time)))

            self.f.close()


        
        

    def spin(self):
        #-----sync-----
        now = time.time()
        target = self.sync_time
        diff = target - now
        print "diff", diff
        #sleep until sync time
        rospy.sleep(diff)
        self.pid.clear()
        self.last_time = time.time()
        self.latest_msg = [time.time()]*self.nr_cars

        rate = rospy.Rate(config.rate)
        first = True
        while not (rospy.is_shutdown() or self.end):
            rate.sleep()
            self.update()
            if self.id == 0 and first:
                rospy.delete_param("sync_time")
                first = False
        


if __name__ == '__main__':
    s = CarSim()
    s.spin()
