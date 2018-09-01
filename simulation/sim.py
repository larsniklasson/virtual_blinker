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
            rospy.set_param("sync_time", int(time.time()) + 4)
            self.wipe_publisher = rospy.Publisher("wipe_map", String, queue_size=10)
        
        while 1:
            if rospy.has_param("sync_time"):
                self.sync_time = rospy.get_param("sync_time")
                break
            else:
                rospy.sleep(0.01)

        
        #get ros-params
        self.nr_cars = rospy.get_param('nr_cars')
        self.save = rospy.get_param('save') and config.save_id == self.id
        random_seed = rospy.get_param('random') #if this is 0 the non-random car dict will be used

        if self.save:
            open('../risk_estimation/debug.txt', 'w').close()
        
        np.random.seed(random_seed)
        random.seed(random_seed)

        #save all measurements from all cars. time as key. 
        # older entries are removed to avoid too large dicts
        self.car_state_dictionaries = [{} for _ in range(self.nr_cars)]
        
        CAR_DICT = config.getCarDict(random_seed)

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
        self.is_good_behaving = this_car.is_good_behaving
        self.speed_deviation = this_car.speed_deviation

        self.course = Course(travelling_direction, turn)
        self.x, self.y, self.theta = self.course.getPose(starting_distance)

        self.risk_estimator = RiskEstimator(self.id, self.course.turn, initial_poses, initial_tds)
        self.maneuver_negotiator = ManueverNegotiator(self.risk_estimator, self.id, self.course.turn, self.nr_cars)

        #the other vehicle's state topics to subscribe to
        state_sub_topics = ["car_state" + str(i) for i in range(self.nr_cars)]

        for s in state_sub_topics:
            rospy.Subscriber(s, msg.CarState, self.stateCallback, queue_size=10)


        rospy.Subscriber("man_neg" + str(self.id), msg.ManNeg, self.maneuver_negotiator.messageCallback, queue_size=10)
        

        
        self.state_publisher = rospy.Publisher('car_state' + str(self.id), msg.CarState, queue_size=10)
        
        #for rviz
        self.true_pose_publisher = rospy.Publisher('true_pose' + str(self.id), msg.TruePose, queue_size=10)
        self.path_publisher = rospy.Publisher('car_path' + str(self.id), msg.Path, queue_size=10)

        
        self.Is = "stop" if self.is_good_behaving else "go"
        self.emergency_break = False
        self.speed = self.course.getSpeed(self.x, self.y, self.Is)


        path = self.course.getPath()

        #for following the path
        self.error_calc = ErrorCalc(path) #calculates how far away we are from ideal path
        self.pid = PID(*config.pid)

        self.t = 0

        rospy.sleep(0.1) #let publishers register
        if self.id == 0: self.wipe_publisher.publish(String("")) # for cleaning up paths/cars on the map
        rospy.sleep(1) #make sure map is wiped before publishing new paths
        self.path_publisher.publish(msg.Path([msg.Position(x,y) for x,y in path], self.id))

        #has vehicle entered try maneuever state or not
        self.has_initiated_trymaneuever = False
        self.has_sent_release = False
        self.laser_raptors = False

    def stateCallback(self, msg):

        #after t > 10 car nr 0 gets no msgs other than its own
        #if self.t > 10 and self.id == 0 and msg.id != 0:
        #    return

        #drop 50% of msgs (except for own msg)
        #if msg.id != self.id and random.random() > 0.5:
        #    return

        if self.t - msg.t > 1/config.rate * config.discard_measurement_time: # old af message, flush queue. 
            return 

        self.latest_msg[msg.id] = time.time()
        d = self.car_state_dictionaries[msg.id]
        d[msg.t] = msg

        if msg.t - 50 in d: del d[msg.t - 50]

        listenToThese = []
        for i in range(self.nr_cars):
            if time.time() - self.latest_msg[i] <= 0.1:
                listenToThese.append(i)
        
        have_all = True
        for c in listenToThese:
            if not msg.t in self.car_state_dictionaries[c]:
                have_all = False
                break

        
        #if we have measurements from all cars for a certain time-stamp then perform risk estimation
        if have_all:
            poses = {}
            deviations = {}
            blinkers = {}
            emergency_breaks = {}

            for c in listenToThese:
                entry = self.car_state_dictionaries[c][msg.t]

                p = (entry.x, entry.y, entry.theta, entry.speed)
                dev = (entry.x_deviation, entry.y_deviation,
                              entry.theta_deviation, entry.speed_deviation)
                
                poses[c] = p
                deviations[c] = dev
                blinkers[c] = entry.blinker
                emergency_breaks[c] = entry.emergency_break

            actual_time = float(msg.t)/(config.rate * config.slowdown)
        
            self.risk_estimator.update_state(actual_time, poses, deviations, blinkers, emergency_breaks)
            

            if self.is_good_behaving:
                
                """
                Es_go = self.risk_estimator.getExpectation(self.id, self.course.turn)
                
                
                if Es_go > config.Es_threshold or self.course.hasReachedPointOfNoReturn(self.x, self.y, self.theta):
                    self.Is = "go"
                else:
                    self.Is = "stop"
                """
                
                #maximum risk out of all the other cars
                rl = [self.risk_estimator.getRisk(self.id, i) for i in range(self.nr_cars) if i != self.id]
                risk = max(rl)
                
                if risk > config.risk_threshold:
                    self.emergency_break = True
                    print "Emergency break activated on car ", self.id
                else:
                    self.emergency_break = False


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
        
        if not self.has_initiated_trymaneuever:
            self.Is = "stop"

        if self.maneuver_negotiator.granted:
            self.Is = "go"

        if self.risk_estimator.grant_list != []:
            self.Is = "stop"

        # follow speed profile
        targetspeed = self.course.getSpeed(self.x, self.y, self.Is)
        if self.Is == "go":
            targetspeed += self.speed_deviation

        #if self.id == 0 and self.course.getDistance(self.x, self.y) > self.course.distance_at_crossing+2 and self.t < 200:
        #    targetspeed = 0

        #adapt speed to vehicle in front
        recommended_speed = self.risk_estimator.recommendSpeedIfVehicleInFront(self.id)
        # -1 = no recommendation
        if recommended_speed != -1: 
            targetspeed = min(targetspeed, recommended_speed)
            
        if self.emergency_break:
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
        
        if self.speed < 0.1:
            self.laser_raptors = True
        
        #start trymaneuever
        if not self.has_initiated_trymaneuever and self.course.hasPassedRequestLine(self.x, self.y):
            print "initiating trymaneuver ", self.id
            self.has_initiated_trymaneuever = True
            self.maneuver_negotiator.tryManeuver()
            #thread1 = Thread(target=self.maneuver_negotiator.tryManeuver,args=())
            #thread1.start()



        #----Fake filtering----
        #noisy measurement
        x_noise = np.random.normal(self.x, config.x_deviation)
        y_noise = np.random.normal(self.y, config.y_deviation)
        t_noise = np.random.normal(self.theta, config.theta_deviation)
        s_noise = np.random.normal(self.speed, config.speed_deviation)

        # "filtering"
        x_filtered = self.x + (x_noise - self.x) / 5.0
        y_filtered = self.y + (y_noise - self.y) / 5.0
        t_filtered = self.theta + (t_noise - self.theta) / 5.0
        s_filtered = self.speed + (s_noise - self.speed) / 5.0

        x_dev_filtered = abs(x_noise - self.x)
        y_dev_filtered = abs(y_noise - self.y)
        t_dev_filtered = abs(t_noise - self.theta)
        s_dev_filtered = abs(s_noise - self.speed)


        blinker = ""#self.course.turn if self.is_good_behaving else ""

        carstate_msg = msg.CarState(
            x_filtered, y_filtered, t_filtered, s_filtered, 
            x_dev_filtered, y_dev_filtered, t_dev_filtered, s_dev_filtered, 
            blinker, self.emergency_break, self.id, self.t)
        
        
        #publish noisy and true state
        self.state_publisher.publish(carstate_msg)
        self.true_pose_publisher.publish(msg.TruePose(self.x, self.y, self.theta, self.speed, self.id))
        

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
        while not rospy.is_shutdown():
            rate.sleep()
            self.update()
            if self.id == 0 and first:
                rospy.delete_param("sync_time")
                rospy.delete_param("random")
                first = False
        


if __name__ == '__main__':
    s = CarSim()
    s.spin()
