import sys
sys.path.append("..")
from math import *
from threading import Lock
import config
import numpy as np

#cdf function of normal distribution
#much faster than scipy version
def normal_cdf(x, mu, sigma):
    q = erf((x - mu) / (sigma * sqrt(2.0)))
    return (1.0 + q) / 2.0

class RiskEstimator:
    def __init__(self, initial_poses):
        
        self.lock = Lock()
        self.nr_cars = len(initial_poses)
        self.intersection = config.intersection
        self.travelling_directions = [self.intersection.getTravellingDirection(x,y,theta) 
                                      for x,y,theta,_ in initial_poses]
        
        self.grant_list = [0,4]
        self.queue = []

        self.emergency_breaks = [False] * self.nr_cars
        self.intention_densities = {}  
        self.expectation_densities = {}
        self.turns = self.intersection.turns

        default_I_dens = 1.0/(len(self.turns)*2)
        default_E_dens = 0.5


        for id in range(self.nr_cars):
            II = {}
            for Ic in self.turns:
                for Is in ["go", "stop"]:
                    II[Ic,Is] = default_I_dens
                self.expectation_densities[id, Ic] = default_E_dens
                
            self.intention_densities[id] = II

        self.latest_poses = initial_poses

    def addVehicleToGrantList(self, id):
        with self.lock:
            self.grant_list.append(id)
    def removeVehicleFromGrantList(self, id):
        with self.lock:
            self.grant_list.remove(id)
    
    def update_state(self, t, poses, deviations, blinkers, emergency_breaks):
        with self.lock:
            self.emergency_breaks = emergency_breaks
            self.latest_poses = poses

            
            self.queue = []
            for td in self.intersection.travelling_directions:
                c = self.intersection.courses[td, "straight"]
                inds = [i for i in range(len(poses)) if self.travelling_directions[i] == td]
                ds = []
                for i in inds:
                    x,y,_,_ = poses[i]
                    d = c.getDistance(x,y)

                    if d < c.distance_at_crossing+1:
                        ds.append((i,d))

                ds.sort(key=lambda x: x[1])
                for i,_ in ds[:-1]:
                    self.queue.append(i)

            

            #------intention----------------
            for car in range(self.nr_cars):
                D = {}
                for Ic in self.turns:
                    for Is in ["go", "stop"]:
                        LPose = 1.0/self.expected_error(poses[car], deviations[car], self.travelling_directions[car], Ic, Is)
                        
                        e = self.expectation_densities[car,Ic]
                        ee = e if Is == "go" else 1-e
                        LE = 1 + 3 * ee

                        LB = 5 if blinkers[car] == Ic else 1

                        L = LPose * LE * LB

                        D[Ic,Is] = L
                
                sigma = sum(D.values())
                for key,value in D.iteritems():
                    D[key] = value/sigma
            
                self.intention_densities[car] = D
            

            #------------project forward------------------
            self.forward_projection_dict = {}

            for car in range(self.nr_cars):
                td = self.travelling_directions[car]
                x, y, theta, speed = poses[car]
                x_dev, y_dev, _, s_dev = deviations[car]
                for Ic in self.turns:
                    c = self.intersection.courses[td,Ic]
                    
                    optimalTheta = c.getPose(c.getDistance(x,y))[2]

                    #position 0.8 std devitation forward (sort of) along path 
                    fwd_x, fwd_y = x + x_dev * 0.8 * cos(optimalTheta), y + y_dev * 0.8 * sin(optimalTheta)
                    
                    #position 0.8 std deviation backwatd along path
                    bwd_x, bwd_y = x - x_dev * 0.8 * cos(optimalTheta), y - y_dev * 0.8 * sin(optimalTheta)

                    for i in self.intersection.turn_extras[Ic]:

                        extra = self.intersection.extra_distances[i]       
                        
                        #fwd case: left side of normal dist. If fwd_x/fwd_y past intersection, we want to "pull"
                        # the left side of the normal distrubition towards 0 to get largest span. Thus increase speed
                        # other cases are similar.
                        # +- 0.5 logic: if deviation is 0, we still want some gap in projection
                        if c.getDistance(fwd_x,fwd_y) > c.distance_at_crossing + extra:
                            fwd_s = speed - s_dev*0.8 - 0.5
                        else:
                            fwd_s = speed + s_dev*0.8 + 0.5

                        if c.getDistance(bwd_x,bwd_y) > c.distance_at_crossing + extra:
                            bwd_s = speed + s_dev*0.8 + 0.5
                        else:
                            bwd_s = speed - s_dev*0.8 - 0.5

                        fwd_s = max(fwd_s, 0.01)
                        bwd_s = max(bwd_s, 0.01)

                        #sampling from x,y and speed with 0.8 * std deviation in "forward/backward" direction, 
                        # the resulting times to crossing is about 1 std deviation
                        negSigma_time = c.getTimeToCrossing(fwd_x, fwd_y, fwd_s, "go", extra)
                        posSigma_time = c.getTimeToCrossing(bwd_x, bwd_y, bwd_s, "go", extra)

                        mu = (negSigma_time + posSigma_time)/2

                        sigma = abs(mu-negSigma_time)

                        self.forward_projection_dict[car, Ic, i] = mu, sigma
            
            
            #this will become new expectation_densities
            E = {}

            self.gap_dict = {}

            for egocar in range(self.nr_cars):
                td_ego = self.travelling_directions[egocar]
                for turn_ego in self.turns:
                    x,y,theta,speed = poses[egocar]
                    c = self.intersection.courses[td_ego, turn_ego]
                    if c.hasLeftIntersection(x,y):
                        E[egocar, turn_ego] = 1.0
                        continue
                    
                    #1.0 => Es = go
                    min_es = 1.0
                    for othercar in range(self.nr_cars):
                        
                        if egocar == othercar:
                            continue

                        td_other = self.travelling_directions[othercar]

                        ego_granted = egocar in self.grant_list and egocar not in self.queue
                        ego_row = self.intersection.hasRightOfWay(td_ego, turn_ego, td_other)
                        other_granted = othercar in self.grant_list and othercar not in self.queue                        
                        if other_granted and ego_granted:
                            if ego_row:
                                continue
                        
                        if not other_granted:
                            if ego_row or ego_granted:
                                continue
                        
                        e_sum = 0
                        for turn_other in self.turns:
                            
                            if not self.intersection.doesCoursesIntersect(td_ego, turn_ego, td_other, turn_other):
                                #no intersection of courses => no gap => "gap" is enough
                                probability_gap_enough = 1
                            else:
                                ego_extradist_indice, other_extradist_indice = self.intersection.getExtras(td_ego, turn_ego, td_other, turn_other)

                                mean_ego, std_ego = self.forward_projection_dict[egocar, turn_ego, ego_extradist_indice]

                                mean_other, std_other = self.forward_projection_dict[othercar, turn_other, other_extradist_indice]
                                gap_mean, gap_std = mean_other - mean_ego, sqrt(std_ego**2 + std_other**2)
                                
                                self.gap_dict[egocar, turn_ego, othercar, turn_other] = gap_mean, gap_std
                                probability_gap_enough = 1 - (normal_cdf(config.gap_upper_limit, gap_mean, gap_std) - \
                                            normal_cdf(config.gap_lower_limit, gap_mean, gap_std))
                                
                            e_sum += probability_gap_enough * self.intentionCarTurn(othercar, turn_other)
                            
                        if e_sum < min_es:
                            min_es = e_sum
                    
                    E[egocar,turn_ego] = min_es
                    
            self.expectation_densities = E

    def expected_error(self, mu_arr, dev_arr, td, turn, i):
        
        x,y,_,_ = mu_arr

        c = self.intersection.courses[td, turn]
        opt = getOptimalPose(c, x, y, i)

        mu_arr = np.array(mu_arr)
        dev_arr = np.array(dev_arr)
        opt = np.array(opt)
        weights = np.array(config.error_weights)

        exp_err_array = expected_error_one_variable(mu_arr, dev_arr, opt, weights)
        return np.sum(exp_err_array)

    
    def getRisk(self, ego_car, risk_car):
        with self.lock:
            td_ego = self.travelling_directions[ego_car]
            td_risk = self.travelling_directions[risk_car]
            sum = 0
            for ego_turn in self.turns:
                for risk_turn in self.turns:
                    if self.intersection.doesCoursesIntersect(td_ego, ego_turn, td_risk, risk_turn):
                        
                        

                        if self.emergency_breaks[risk_car]:
                            #if risk car is emergency breaking and both vehicles are close to intersection
                            #point, then add some risk. This can, for example, happen if the risk car triggers the EB (for some
                            # other car) in the middle of the intersection 
                            risk_extradist_indice, ego_extradist_indice = \
                                self.intersection.getExtras(td_risk, risk_turn, td_ego, ego_turn)

                            tti_risk, _ = self.forward_projection_dict[risk_car, risk_turn, risk_extradist_indice]
                            tti_ego, _ = self.forward_projection_dict[ego_car, ego_turn, ego_extradist_indice]
                            
                            #TODO don't ignore deviations here. although this happens so rarely
                            if tti_risk > -1 and tti_risk < 1 and tti_ego < 2 and tti_ego > -1:
                                sum += self.intentionCarTurn(risk_car, risk_turn) * self.intentionCarTurn(ego_car, ego_turn)

                        egoInGL = ego_car in self.grant_list and ego_car not in self.queue
                        riskInGL = risk_car in self.grant_list and risk_car not in self.queue
                        riskROW = self.intersection.hasRightOfWay(td_risk, risk_turn, td_ego)
                        
                        flag = False
                        if riskInGL:
                            if egoInGL and not riskROW:
                                flag = True
                        else:
                            if egoInGL or not riskROW:
                                flag = True

                        #TODO simplify maybe

                        
                        if flag:
                            try:
                                #the gap size for triggering EB is a little smaller than for expectation. 
                                # (dont trigger when not absolutely needed). Maybe this is not so smart, 
                                # but traffic is faster
                                gap_mean, gap_std = self.gap_dict[risk_car, risk_turn, ego_car, ego_turn]
                                p_gap_not_enough = (normal_cdf(config.risk_gap_upper_limit, gap_mean, gap_std) - \
                                        normal_cdf(config.risk_gap_lower_limit, gap_mean, gap_std))
                                e = self.expectation_densities[risk_car, risk_turn]
                                sum += (1-e) * self.intention_densities[risk_car][risk_turn, "go"] * \
                                    self.intentionCarTurn(ego_car, ego_turn) * p_gap_not_enough

                            except:
                                #happens i believe for keyerror in gap_dict. Not sure when that
                                # happens though. I know it means no risk at least
                                pass

            return sum

    #some crazy stuff to get speed of vehicle in front 
    def recommendSpeedIfVehicleInFront(self, ego_car):
        with self.lock:
            d_min = 999999999
            ego_pose = self.latest_poses[ego_car]
            ego_td = self.travelling_directions[ego_car]
            for ego_turn in self.turns:
                ego_course = self.intersection.courses[ego_td, ego_turn]
                ego_P = self.intentionCarTurn(ego_car, ego_turn)
                if ego_P < 0.15:
                    continue

                for other_car in range(self.nr_cars):
                    if other_car == ego_car:
                        continue
                    other_pose = self.latest_poses[other_car]
                    other_td = self.travelling_directions[other_car]
                    for other_turn in self.turns:
                        other_P = self.intentionCarTurn(other_car, other_turn)
                        if other_P < 0.15:
                            continue
                        ego_hasLeft = ego_course.hasLeftIntersection(*ego_pose[:2])
                        if ego_td == other_td and (not ego_hasLeft) or ego_hasLeft and self.intersection.doesCoursesOverlap(ego_td, ego_turn, other_td, other_turn):
                            
                            d_ego = ego_course.getDistance(*ego_pose[:2])
                            d_other = ego_course.getDistance(*other_pose[:2])
                            d = d_other - d_ego
                            if d < d_min and d > 0:
                                d_min = d
                                speed = other_pose[-1]

            ego_speed = ego_pose[-1]
            if d_min < 5 + 2 * ego_speed:
                return max(0, speed - 1)
            else:
                return -1


    """
    def getRisk(self, car):
        with self.lock:
            sum = 0
            for turn in self.turns:
                e = self.expectation_densities[car, turn]
                #sum += e * self.intentionDensities[car][turn, "stop"]
                sum += (1-e) * self.intentionDensities[car][turn, "go"]

            return sum
    """

    def noConflict(self, car, turn):
        with self.lock:
            return self.expectation_densities[car, turn] > config.grant_threshold
        

    def getExpectation(self, id, turn):
        return self.expectation_densities[id, turn]
        
    def intentionCarTurn(self, car, turn):
        a = self.intention_densities[car]
        return a[turn, "go"] + a[turn, "stop"]

#including speed.
def getOptimalPose(c, x, y, Is):

    d = c.getDistance(x,y)
    if d > c.distance_at_crossing and Is=="stop":
        # put the optimal stop position to start of intersection
        # if vehicle has entered intersection
        opt_x, opt_y, opt_theta = c.getPose(c.distance_at_crossing)
    else:
        opt_x, opt_y, opt_theta = c.getPose(d)
    
    opt_s = c.sp_go.getSpeed(d) if Is=="go" else c.sp_stop.getSpeed(d)
    return (opt_x, opt_y, opt_theta, opt_s)

#expected error of some pose variable
def expected_error_one_variable(mu, sigma, optvalue, weight):
    return weight * ((optvalue - mu)**2 + sigma**2)
